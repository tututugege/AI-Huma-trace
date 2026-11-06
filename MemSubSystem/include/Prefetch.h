#pragma once
// L1D data prefetcher (EXP-009).
//
// Circuit-style rules (checked at run time, a violation aborts the simulation):
//   * every piece of state is a pf::Table (entries declared with PF_ENTRY: each field has a declared width, values are
//     truncated to it on write) or a pf::Fifo; the total declared size is printed and bounded by PF_STATE_BUDGET_BITS;
//   * a table has a fixed number of read, write and CAM-search ports per cycle; exceeding them aborts;
//   * the prefetcher is evaluated once per cycle: comb() reads only state (cur), seq() updates it.
// Inputs per cycle: up to LSU_LDU_COUNT load access events, fill / eviction feedback. Output: one candidate line from the
// head of the request FIFO (the L1D probes the tag array through an idle load port and allocates an MSHR on a miss).
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "config.h"

#ifndef CFG_PF_ALGO
#define CFG_PF_ALGO 0  // 0 off, 1 next-line, 2 IP-stride, 3 stream+stride ...
#endif
#ifndef CFG_PF_TRAIN_PORTS
#define CFG_PF_TRAIN_PORTS 2
#endif
#ifndef CFG_PF_QUEUE
#define CFG_PF_QUEUE 8
#endif
#ifndef CFG_PF_MSHR_RESERVE
#define CFG_PF_MSHR_RESERVE 2  // a prefetch takes an MSHR only if more than this many stay free
#endif
#ifndef CFG_PF_STATE_BUDGET_BITS
#define CFG_PF_STATE_BUDGET_BITS (4 * 1024 * 8)  // 4 KB of predictor state
#endif

constexpr int PF_TRAIN_PORTS = CFG_PF_TRAIN_PORTS;
constexpr int PF_QUEUE = CFG_PF_QUEUE;
constexpr int PF_MSHR_RESERVE = CFG_PF_MSHR_RESERVE;
constexpr int PF_LINE_BITS = 32 - 6;  // physical line number (64 B lines, 32-bit physical address)


#define PF_ALGO (pf::algo())
namespace pf {

inline void fail(const char *what, const char *name) {
  std::fprintf(stderr, "[PF] circuit rule violated: %s (%s)\n", what, name);
  std::fflush(stderr);
  std::abort();
}

inline int64_t sext(uint32_t v, int w) {
  const uint32_t m = 1u << (w - 1);
  return int64_t((v ^ m)) - int64_t(m);
}
inline uint32_t wmask(int w) { return w >= 32 ? 0xFFFFFFFFu : ((1u << w) - 1u); }

// runtime design parameters (degree, distance, thresholds): not state, read once from PF_<name>
inline int param(const char *name, int def) {
  std::string k = std::string("PF_") + name;
  const char *v = std::getenv(k.c_str());
  return v ? std::atoi(v) : def;
}

inline int algo() {  // 0 off, 1 next-line, 2 IP-stride, 3 best-offset, 4 best-offset + IP-stride (runtime: PF_ALGO)
  static const int v = param("ALGO", CFG_PF_ALGO);
  return v;
}

struct StateReport {
  struct Item { const char *name; long bits; int rd, wr, cam; long cam_bits; };
  std::vector<Item> items;
  long total() const { long t = 0; for (auto &i : items) t += i.bits; return t; }
};

}  // namespace pf

// Entry definition: PF_ENTRY(Name, X) with X(field, width) lines. Gives BITS and trunc().
#define PF_DECL_(n, w) uint32_t n = 0;
#define PF_SUM_(n, w) +(w)
#define PF_TRUNC_(n, w) n &= pf::wmask(w);
#define PF_ENTRY(NAME, FIELDS)                        \
  struct NAME {                                       \
    FIELDS(PF_DECL_)                                  \
    static constexpr int BITS = 0 FIELDS(PF_SUM_);    \
    void trunc() { FIELDS(PF_TRUNC_) }                \
  };

namespace pf {

class Port {
 public:
  void tick(const char *name) {
    if (rd_n > rd || wr_n > wr || cam_n > cam) fail("port count exceeded in one cycle", name);
    rd_n = wr_n = cam_n = 0;
  }
  int rd = 1, wr = 1, cam = 0;
  int rd_n = 0, wr_n = 0, cam_n = 0;
};

// Storage array: N entries, fixed ports. Direct-indexed read/write, plus an optional CAM search over all entries.
template <class E, int N>
class Table {
 public:
  Table(const char *nm, int rd_ports, int wr_ports, int cam_ports = 0) : name(nm) {
    p.rd = rd_ports; p.wr = wr_ports; p.cam = cam_ports;
  }
  const E &rd(int i) { p.rd_n++; return e[i % N]; }
  void wr(int i, E v) { p.wr_n++; v.trunc(); e[i % N] = v; }
  // CAM: returns index of the first entry for which pred(entry) holds, or -1. cam_bits = compared width per entry.
  template <class F>
  int cam(F pred, int cam_bits) {
    p.cam_n++; cam_w = cam_bits;
    for (int i = 0; i < N; i++) if (pred(e[i])) return i;
    return -1;
  }
  void tick() { p.tick(name); }
  void report(StateReport &r) const { r.items.push_back({name, long(E::BITS) * N, p.rd, p.wr, p.cam, long(cam_w) * N}); }
  E e[N];
  const char *name;
  Port p;
  int cam_w = 0;
};

// Circular FIFO, depth N: 1 enqueue port per cycle slot count enq_ports, 1 dequeue.
template <class E, int N>
class Fifo {
 public:
  Fifo(const char *nm, int enq_ports) : name(nm), enq_max(enq_ports) {}
  bool full() const { return cnt == N; }
  bool empty() const { return cnt == 0; }
  int count() const { return cnt; }
  const E &front() const { return e[head]; }
  bool push(E v) {
    if (++enq_n > enq_max) fail("FIFO enqueue ports exceeded", name);
    if (cnt == N) return false;
    v.trunc(); e[(head + cnt) % N] = v; cnt++; return true;
  }
  void pop() { if (++deq_n > 1) fail("FIFO dequeue ports exceeded", name); head = (head + 1) % N; cnt--; }
  template <class F>
  bool contains(F pred, int cam_bits) {  // CAM over the entries (counts as one search)
    if (++cam_n > 1) fail("FIFO CAM ports exceeded", name);
    cam_w = cam_bits;
    for (int i = 0; i < cnt; i++) if (pred(e[(head + i) % N])) return true;
    return false;
  }
  void tick() { enq_n = deq_n = cam_n = 0; }
  void report(StateReport &r) const { r.items.push_back({name, long(E::BITS) * N + 2 * 8, 1, enq_max, 1, long(cam_w) * N}); }
  E e[N];
  int head = 0, cnt = 0;
  const char *name;
  int enq_max, enq_n = 0, deq_n = 0, cam_n = 0, cam_w = 0;
};

}  // namespace pf

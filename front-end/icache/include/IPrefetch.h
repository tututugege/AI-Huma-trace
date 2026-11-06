#pragma once
// I-cache prefetch engine (EXP-009).
//
// The I-cache is blocking with one miss in flight. The instruction bytes come from functional memory, so the cache arrays only
// hold tag/valid; a prefetch therefore needs (1) an AXI read on its own transaction id, so the timing and the LLC / DRAM
// traffic are real, and (2) a tag/valid write into the I-cache arrays when the response is back.
//
// Two candidate generators (PF_IMODE): 0 = next lines of every demand miss (PF_IDEG lines); 1 = FDIP: look at the fetch
// addresses the branch predictor already queued (the fetch-address FIFO) one per cycle, translate them with a small µTLB that
// the demand translations fill, probe the tags and prefetch the lines that are missing.
// Up to PF_IIDS (<= 4) prefetches are in flight (AXI ids 12..15; the demand miss uses ids < 12) and the demand miss has
// priority on the AXI request channel. The install uses the array write port only in cycles without a demand refill write.
// State: slots 4 x (2 + 26 + 2) + µTLB N x 41 + scan/recent 26 x 4 + ~40 control bits.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace ipf {

constexpr uint8_t kIdBase = 12;  // ids kIdBase .. kIdBase+3 belong to the prefetch engine (the I-cache allocates ids < 12)
constexpr int kMaxSlots = 4;
constexpr int kMaxUtlb = 64;

struct Stats {
  uint64_t trig = 0, cand = 0, present = 0, issued = 0, filled = 0, dup = 0, notrans = 0, inflight = 0;
};
inline Stats g_stats;

inline int param(const char *name, int def) {
  const char *v = std::getenv((std::string("PF_") + name).c_str());
  return v ? std::atoi(v) : def;
}

struct In {
  bool trig = false;         // a demand miss request was issued (line base address in trig_addr)
  uint32_t trig_addr = 0;
  bool flush = false;        // fence.i / invalidate: drop everything
  bool demand_axi = false;   // the demand path drives the AXI request channel this cycle
  bool acc_pf = false;       // an AXI request of the engine was accepted (id acc_id)
  uint8_t acc_id = 0;
  bool resp_pf = false;      // an AXI response with an engine id is visible (id resp_id)
  uint8_t resp_id = 0;
  // FDIP: queued fetch addresses (virtual, head first), a translation learned from the demand path, redirect
  int fifo_n = 0;
  uint32_t fifo[16] = {0};
  bool utlb_fill = false;
  uint32_t fill_vpn = 0, fill_ppn = 0;
  bool refetch = false;
};
struct Drive {
  bool valid = false;
  uint32_t addr = 0;
  uint8_t id = 0;
  bool resp_ready = false;
};

}  // namespace ipf

// implemented in icache.cpp (owns the tag/valid arrays) and fetch_address_FIFO.cpp
bool icache_pf_present(uint32_t paddr, int *victim_way);
int fetch_addr_fifo_peek(uint32_t *out, int max);
struct IcachePfInst { bool valid = false; uint32_t index = 0, way = 0, tag = 0; };
extern IcachePfInst g_icache_pf_inst;
extern bool g_icache_pf_inst_done;

namespace ipf {

class Engine {
 public:
  Engine() {
    deg = param("IDEG", 0);
    mode = param("IMODE", 0);
    if (mode == 1 && deg == 0) deg = 1;
    nslots = param("IIDS", 1);
    if (nslots < 1) nslots = 1;
    if (nslots > kMaxSlots) nslots = kMaxSlots;
    utlb_n = param("IUTLB", 8);
    if (utlb_n > kMaxUtlb) utlb_n = kMaxUtlb;
    scan_max = param("ISCAN", 15);
    for (int i = 0; i < kMaxSlots; i++) slot[i].id = uint8_t(kIdBase + i);
    if (std::getenv("PF_STATS") && deg > 0)
      std::fprintf(stderr, "[IPF] mode=%d degree=%d slots=%d utlb=%d state~%d bits\n", mode, deg, nslots, utlb_n,
                   nslots * 30 + (mode == 1 ? utlb_n * 41 + 26 * 4 : 0) + 40);
  }
  ~Engine() {
    if (std::getenv("PF_STATS") && deg > 0)
      std::fprintf(stderr,
                   "[IPF-STATS] trig=%llu cand=%llu present=%llu issued=%llu filled=%llu dup=%llu notrans=%llu inflight=%llu\n",
                   (unsigned long long)g_stats.trig, (unsigned long long)g_stats.cand, (unsigned long long)g_stats.present,
                   (unsigned long long)g_stats.issued, (unsigned long long)g_stats.filled, (unsigned long long)g_stats.dup,
                   (unsigned long long)g_stats.notrans, (unsigned long long)g_stats.inflight);
  }
  bool enabled() const { return deg > 0 || mode > 0; }

  Drive cycle(const In &in) {
    Drive d;
    if (in.flush) {
      for (auto &s : slot) s.st = FREE;
      pend = false; g_icache_pf_inst.valid = false; scan = 1; nrecent = 0;
    }
    if (mode == 1 && in.utlb_fill) {
      int hit = -1;                                         // the demand path translates every fetch: update, do not duplicate
      for (int i = 0; i < utlb_n; i++) if (utlb[i].v && utlb[i].vpn == in.fill_vpn) hit = i;
      if (hit >= 0) utlb[hit].ppn = in.fill_ppn;
      else { utlb[utlb_rr].v = true; utlb[utlb_rr].vpn = in.fill_vpn; utlb[utlb_rr].ppn = in.fill_ppn; utlb_rr = (utlb_rr + 1) % utlb_n; }
    }
    if (in.refetch) { scan = 1; }
    if (g_icache_pf_inst_done) { g_stats.filled++; g_icache_pf_inst_done = false; }

    // 1. request accepted by the interconnect
    if (in.acc_pf)
      for (int i = 0; i < nslots; i++) if (slot[i].st == REQ && slot[i].id == in.acc_id) { slot[i].st = WAIT; g_stats.issued++; }
    // 2. response: install into the tag arrays when the write port is free; otherwise leave the response on the channel
    if (in.resp_pf) {
      int w = -1;
      for (int i = 0; i < nslots; i++) if (slot[i].st == WAIT && slot[i].id == in.resp_id) w = i;
      if (w < 0) d.resp_ready = true;                       // stale response (after a flush)
      else if (!g_icache_pf_inst.valid) {
        d.resp_ready = true;
        int way;
        if (icache_pf_present(slot[w].line << 6, &way)) g_stats.dup++;   // the demand path brought it in meanwhile
        else {
          g_icache_pf_inst.valid = true;
          g_icache_pf_inst.index = slot[w].line & 63u;
          g_icache_pf_inst.way = uint32_t(way);
          g_icache_pf_inst.tag = (slot[w].line >> 6) & 0xFFFFFu;
          g_icache_pf_inst_done = false;
        }
        slot[w].st = FREE;
      }
    }
    // 3. a new candidate takes a free slot (one request on the channel at a time)
    bool req_pending = false;
    int freeslot = -1;
    for (int i = 0; i < nslots; i++) { if (slot[i].st == REQ) req_pending = true; if (slot[i].st == FREE && freeslot < 0) freeslot = i; }
    if (!req_pending && freeslot >= 0) {
      uint32_t line;
      if (candidate(in, &line)) {
        bool busy = false;
        for (int i = 0; i < nslots; i++) if (slot[i].st != FREE && slot[i].line == line) busy = true;
        if (busy) g_stats.inflight++;
        else { slot[freeslot].st = REQ; slot[freeslot].line = line; g_stats.trig++; }
      }
    }
    // 4. drive the request channel when the demand path does not
    if (!in.demand_axi)
      for (int i = 0; i < nslots; i++) if (slot[i].st == REQ) { d.valid = true; d.addr = slot[i].line << 6; d.id = slot[i].id; break; }
    return d;
  }

 private:
  // candidate generator: returns a missing line (physical line number) or false
  bool candidate(const In &in, uint32_t *out) {
    if (mode == 0) {
      if (in.trig) { pend = true; base = in.trig_addr >> 6; k = 1; left = deg; }
      if (!pend) return false;
      if (left == 0) { pend = false; return false; }
      const uint32_t c = base + uint32_t(k);
      k++; left--;
      if ((c >> 6) != (base >> 6)) { pend = false; return false; }      // stay inside the 4 KiB page
      g_stats.cand++;
      int way;
      if (icache_pf_present(c << 6, &way)) { g_stats.present++; return false; }
      *out = c; return true;
    }
    // FDIP: one queued fetch address per cycle (entry 0 is being fetched now)
    if (scan < 1) scan = 1;
    if (scan >= in.fifo_n || scan > scan_max) { scan = 1; return false; }
    const uint32_t va = in.fifo[scan++];
    const uint32_t vline = va >> 6;
    for (int i = 0; i < 4; i++) if (recent_valid[i] && recent[i] == vline) return false;   // probed lately
    recent[recent_rr] = vline; recent_valid[recent_rr] = true; recent_rr = (recent_rr + 1) % 4;
    g_stats.cand++;
    int e = -1;
    for (int i = 0; i < utlb_n; i++) if (utlb[i].v && utlb[i].vpn == (va >> 12)) { e = i; break; }
    if (e < 0) { g_stats.notrans++; return false; }            // no translation: skip (costs no ITLB port)
    const uint32_t pline = (utlb[e].ppn << 6) | ((va >> 6) & 63u);
    int way;
    if (icache_pf_present(pline << 6, &way)) { g_stats.present++; return false; }
    *out = pline; return true;
  }

  enum St { FREE, REQ, WAIT };
  struct Slot { St st = FREE; uint32_t line = 0; uint8_t id = 0; } slot[kMaxSlots];
  struct UTlb { bool v = false; uint32_t vpn = 0, ppn = 0; } utlb[kMaxUtlb];
  int utlb_rr = 0, utlb_n = 8, scan = 1, scan_max = 15, recent_rr = 0, nrecent = 0;
  uint32_t recent[4] = {0}; bool recent_valid[4] = {false};
  bool pend = false;
  uint32_t base = 0;
  int k = 1, left = 0, deg = 0, mode = 0, nslots = 1;
};

inline Engine &engine() { static Engine e; return e; }

}  // namespace ipf

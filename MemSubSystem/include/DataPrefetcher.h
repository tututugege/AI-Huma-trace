#pragma once
// L1D data prefetcher, frozen design "h1" (EXP-009). This is the specification for the RTL (rtl/rtl/memsys/dcache/dcache_pf.sv).
//
// What it is: a single-entry global stride detector (one stream, no PC) trained on L1D load misses and on the first use of
// a prefetched line, a one-line next-line engine whose lines only go to the LLC (they are not filled into the L1D), a
// request FIFO, and nothing else. No accuracy throttle (it was worth 0.1 points with the LLC engine), no PC from the LSU.
//
// Cycle behaviour (all registers update at the end of the cycle, comb reads only registers and this cycle's inputs):
//   - stage 2 of the L1D produces up to LSU_LDU_COUNT access events (miss / first use of a prefetched line);
//   - seq(): (1) the FIFO head is popped if the L1D took it for a tag probe, (2) the stride generator pushes one candidate,
//     (3) otherwise the LLC engine pushes one candidate, (4) the events registered last cycle train the detector,
//     (5) up to PF_TRAIN_PORTS new events are selected into the input registers (extra events are dropped).
// On by default (EXP-009); the environment variable PF_ENABLE=0 turns it off.
// Circuit rules (pf::Table / pf::Fifo): fixed port counts per cycle, widths truncate on write; violations abort.
#include "Prefetch.h"

constexpr int PF_QUEUE = 8;         // request FIFO depth
constexpr int PF_TRAIN_PORTS = 2;   // training events taken per cycle
constexpr int PF_DEGREE = 8;        // candidates of one stride burst
constexpr int PF_DIST = 2;          // the burst starts DIST strides ahead of the trigger line
constexpr int PF_CONF_THR = 2;      // confidence needed to start a burst
constexpr int PF_STRIDE_BITS = 8;   // signed stride in lines
constexpr int PF_LLC_LINES = 1;     // next-line engine: lines after every load miss that only go to the LLC
constexpr int PF_MSHR_RESERVE = 0;  // a probe allocates an MSHR whenever one is free
constexpr int PF_LINE_BITS = 32 - 6;  // physical line number (64 B lines, 32-bit physical address)

struct PfStats {  // model-only counters, not hardware
  uint64_t issued = 0, useful = 0, late = 0, demand_miss = 0, demand_hit = 0;   // read by the PMU
};
inline PfStats g_pf_stats;

// one load access seen at stage 2 of the L1D pipeline
struct PfAccess {
  bool valid = false;
  uint32_t paddr = 0;
  bool miss = false;     // L1D miss (new, or merged into an in-flight line)
  bool pf_hit = false;   // hit on a line that was brought in by a prefetch and not used before
};

#define PF_REQ_FIELDS(X) X(line, PF_LINE_BITS) X(llc, 1)
PF_ENTRY(PfReqE, PF_REQ_FIELDS)
#undef PF_REQ_FIELDS
#define PF_STRIDE_FIELDS(X) X(valid, 1) X(last, PF_LINE_BITS) X(stride, PF_STRIDE_BITS) X(conf, 2)
PF_ENTRY(PfStrideE, PF_STRIDE_FIELDS)
#undef PF_STRIDE_FIELDS

class DataPrefetcher {
 public:
  // ---- combinational inputs (set by the L1D in stage 2 of the current cycle) ----
  PfAccess acc[LSU_LDU_COUNT];
  // ---- combinational output (stage 1 of the L1D): head of the request FIFO ----
  bool head_valid() const { return enabled_ && !reqq.empty(); }
  uint32_t head_addr() const { return reqq.front().line << 6; }
  bool head_llc_only() const { return reqq.front().llc; }
  // set by the L1D in stage 1: the head was sent to the tag probe this cycle
  bool head_taken = false;
  bool enabled() const { return enabled_; }

  DataPrefetcher()
      : reqq("pf_reqq", 1), stride("pf_stride", PF_TRAIN_PORTS, PF_TRAIN_PORTS) {
    enabled_ = pf::param("ENABLE", 1) != 0;   // default on; PF_ENABLE=0 turns the prefetcher off
  }

  // end of cycle: consume the inputs, update the state
  void seq() {
    if (head_taken) reqq.pop();
    head_taken = false;
    if (!enabled_) { clear_in(); return; }

    // 1. the stride generator pushes one candidate per cycle (page-limited, de-duplicated against the FIFO)
    bool port_used = false;
    if (gen.valid) {
      port_used = true;
      const uint32_t line = gen.next;
      if ((line >> 6) != (gen.base >> 6)) {          // stay inside the 4 KiB page: no translation is needed
        gen.valid = false;
      } else {
        push_candidate(line, /*llc=*/false);
        gen.pushed++;
        gen.next = (gen.next + uint32_t(gen.stride)) & pf::wmask(PF_LINE_BITS);
        if (--gen.left == 0) gen.valid = false;
      }
    }
    // 2. the LLC next-line engine uses the FIFO write port only when the stride generator did not
    if (!port_used && gen2.valid) {
      const uint32_t line = gen2.next;
      if ((line >> 6) != (gen2.base >> 6)) {
        gen2.valid = false;
      } else {
        push_candidate(line, /*llc=*/true);
        gen2.next = (gen2.next + 1) & pf::wmask(PF_LINE_BITS);
        if (--gen2.left == 0) gen2.valid = false;
      }
    }
    // 3. training on the events registered in the previous cycle (keeps the prefetcher off the L1D stage-2 path)
    for (int k = 0; k < PF_TRAIN_PORTS; k++) if (q_acc[k].valid) train(q_acc[k]);

    // 4. input registers: the first PF_TRAIN_PORTS events (in load-port order) that are misses or first uses of prefetched lines
    {
      PfAccess nq[PF_TRAIN_PORTS];
      int taken = 0;
      for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const PfAccess &a = acc[i];
        if (!a.valid || !(a.miss || a.pf_hit)) continue;
        if (taken >= PF_TRAIN_PORTS) continue;
        nq[taken++] = a;
      }
      for (int k = 0; k < PF_TRAIN_PORTS; k++) q_acc[k] = nq[k];
    }
    clear_in();
    reqq.tick(); stride.tick();
  }

 private:
  void clear_in() { for (auto &a : acc) a = PfAccess{}; }

  void push_candidate(uint32_t line, bool llc) {
    const bool dup = reqq.contains([&](const PfReqE &x) { return x.line == line; }, PF_LINE_BITS);
    if (dup) return;
    PfReqE e; e.line = line; e.llc = llc;
    reqq.push(e);   // a full queue drops the candidate
  }

  // A stride burst: the first candidate is DIST strides after the trigger line, then one every stride (a new trigger replaces it).
  void start_gen(uint32_t base_line, int32_t stride_lines) {
    gen.valid = true;
    gen.base = base_line;
    gen.stride = stride_lines;
    gen.next = (base_line + uint32_t(stride_lines * PF_DIST)) & pf::wmask(PF_LINE_BITS);
    gen.left = PF_DEGREE;
    gen.pushed = 0;
  }

  void train(const PfAccess &a) {
    const uint32_t line = (a.paddr >> 6) & pf::wmask(PF_LINE_BITS);
    if (a.miss) {  // LLC engine: the next PF_LLC_LINES lines of every load miss
      gen2.valid = true; gen2.base = line; gen2.next = (line + 1) & pf::wmask(PF_LINE_BITS); gen2.left = PF_LLC_LINES;
    }
    // single global stride entry: allocate on the first event, then compare the line distance with the stored stride
    PfStrideE e = stride.rd(0);
    if (!e.valid) {
      PfStrideE n; n.valid = 1; n.last = line; n.stride = 0; n.conf = 0;
      stride.wr(0, n);
      return;
    }
    const int64_t d = pf::sext((line - e.last) & pf::wmask(PF_LINE_BITS), PF_LINE_BITS);
    if (d == 0) return;                                   // same line again: no update
    const bool in_range = d >= -(int64_t(1) << (PF_STRIDE_BITS - 1)) && d < (int64_t(1) << (PF_STRIDE_BITS - 1));
    const int64_t cur = pf::sext(e.stride, PF_STRIDE_BITS);
    if (in_range && d == cur) {
      if (e.conf < 3) e.conf++;
    } else if (e.conf > 0) {
      e.conf--;
      if (e.conf == 0 && in_range) e.stride = uint32_t(d) & pf::wmask(PF_STRIDE_BITS);
    } else {
      e.stride = in_range ? (uint32_t(d) & pf::wmask(PF_STRIDE_BITS)) : 0;
    }
    e.last = line;
    stride.wr(0, e);
    const int32_t sv = int32_t(pf::sext(e.stride, PF_STRIDE_BITS));
    if (e.conf >= uint32_t(PF_CONF_THR) && sv != 0) start_gen(line, sv);
  }

  PfAccess q_acc[PF_TRAIN_PORTS];   // registered training events (selected in the previous cycle)
  struct Gen { bool valid = false; uint32_t next = 0, base = 0, pushed = 0; int32_t stride = 0; int left = 0; } gen;
  struct Gen2 { bool valid = false; uint32_t next = 0, base = 0; int left = 0; } gen2;
  pf::Fifo<PfReqE, PF_QUEUE> reqq;
  pf::Table<PfStrideE, 1> stride;
  bool enabled_ = false;
};

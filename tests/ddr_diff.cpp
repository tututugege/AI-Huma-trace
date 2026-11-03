// Differential test of the compact SimDDR against the frozen original (tests/ddr_ref) with random AXI masters:
// every output and a state digest are compared each cycle. Usage: ddr_diff [cycles] [seed] [pct knobs...]
#include "config.h"
#include "DdrDigest.h"
#include "SimDDRRef.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>

struct Rng { uint64_t s; uint32_t n() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return uint32_t(s >> 11); } bool c(uint32_t p) { return n() % 100 < p; } uint32_t b(uint32_t k) { return n() % k; } };

int main(int argc, char **argv) {
  const uint64_t cycles = argc > 1 ? strtoull(argv[1], 0, 10) : 2000000;
  Rng r{argc > 2 ? strtoull(argv[2], 0, 10) * 0x9e3779b97f4a7c15ull + 1 : 7};
  const int p_ar = argc > 3 ? atoi(argv[3]) : 30, p_aw = argc > 4 ? atoi(argv[4]) : 20, p_w = argc > 5 ? atoi(argv[5]) : 60;
  const int p_rr = argc > 6 ? atoi(argv[6]) : 70, p_br = argc > 7 ? atoi(argv[7]) : 70, maxlen = argc > 8 ? atoi(argv[8]) : 4, rphase = argc > 9 ? atoi(argv[9]) : 4;
  sim_ddr::SimDDR a; sim_ddr::SimDDRRef b; a.init(); b.init();
  std::deque<int> aw_q;  // lengths (beats) of accepted write bursts still owing W beats
  int w_sent = 0, outstanding = 0;  // writes accepted but not yet answered by B (kept small, like a real master)
  uint64_t reads = 0, writes = 0, bresp = 0, mism = 0;
  for (uint64_t c = 0; c < cycles && !mism; c++) {
    sim_ddr::SimDDR_IO_t in{};
    in.ar.arvalid = r.c(p_ar) && ((c >> 10) % 4) < uint64_t(rphase);  // reads pause in some windows so writes can win the backend
    in.ar.arid = r.b(64); in.ar.arlen = r.b(maxlen); in.ar.araddr = r.n() & ~3u; in.ar.arsize = 5; in.ar.arburst = 1;
    in.aw.awvalid = r.c(p_aw) && outstanding < 8; in.aw.awid = r.b(64); in.aw.awlen = r.b(maxlen); in.aw.awaddr = r.n() & ~3u; in.aw.awsize = 5; in.aw.awburst = 1;
    in.w.wvalid = !aw_q.empty() && r.c(p_w); in.w.wlast = !aw_q.empty() && w_sent + 1 == aw_q.front(); in.w.wdata = 0; in.w.wstrb = 0;
    in.r.rready = r.c(p_rr); in.b.bready = r.c(p_br);
    for (int phase = 0; phase < 2; phase++) {
      a.io = in; b.io = in;
      if (phase == 0) {
        a.comb_outputs(); b.comb_outputs();
        const auto &x = a.io, &y = b.io;
        if (x.ar.arready != y.ar.arready || x.aw.awready != y.aw.awready || x.w.wready != y.w.wready || x.r.rvalid != y.r.rvalid ||
            x.r.rid != y.r.rid || x.r.rlast != y.r.rlast || x.b.bvalid != y.b.bvalid || x.b.bid != y.b.bid) {
          printf("MISMATCH outputs at cycle %llu\n", (unsigned long long)c); mism++; break;
        }
        if (sim_ddr::DdrShadowAccess::digest(a) != b.digest()) { printf("MISMATCH digest at cycle %llu\n", (unsigned long long)c); mism++; break; }
        if (in.aw.awvalid && a.io.aw.awready) { aw_q.push_back(int(in.aw.awlen) + 1); outstanding++; }
        if (a.io.b.bvalid && in.b.bready) outstanding--;
        if (in.w.wvalid && a.io.w.wready) { if (++w_sent == aw_q.front()) { aw_q.pop_front(); w_sent = 0; } }
        in = a.io;  // masters see the slave outputs; valid/ready inputs stay as generated
        reads += x.r.rvalid && in.r.rready; bresp += x.b.bvalid && in.b.bready; writes += x.w.wready && in.w.wvalid;
      } else {
        a.seq(); b.seq();
        if (sim_ddr::DdrShadowAccess::digest(a) != b.digest()) { printf("MISMATCH digest after seq at cycle %llu\n", (unsigned long long)c); mism++; }
      }
    }
  }
  printf("ddr_diff: cycles=%llu r_beats=%llu w_beats=%llu b=%llu mismatches=%llu\n", (unsigned long long)cycles, (unsigned long long)reads,
         (unsigned long long)writes, (unsigned long long)bresp, (unsigned long long)mism);
  return mism ? 1 : 0;
}

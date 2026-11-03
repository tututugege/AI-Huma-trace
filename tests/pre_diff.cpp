// PreIduQueue differential: random front-end blocks, decode consumption, flush/mispredict, commits and PC lookups through
// the live module and the frozen reference (pre_shadow checks every phase's outputs and state).
#include "PreShadow.h"
#include "Wires.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

long long sim_time = 0;
// zeroed heap: the shadow compares output structs bytewise, padding included
void *operator new(size_t n) { void *p = calloc(1, n); if (!p) abort(); return p; }
void operator delete(void *p) noexcept { free(p); }
int main(int argc, char **argv) {
  long cycles = argc > 1 ? atol(argv[1]) : 2000000;
  std::mt19937_64 rng(argc > 2 ? atol(argv[2]) : 1);
  Wires &w = *new Wires;
  ExuIdIO latch = {};
  w.idu_br_latch = &latch;
  PreIduQueue *pre = new PreIduQueue(nullptr);
  pre->wires = &w;
  pre->init();
  auto bit = [&](int pct) { return (int)(rng() % 100) < pct; };
  // mix of regimes so that the FTQ / instruction buffer both fill up and drain
  int p_block = 50, p_consume = 70, p_commit = 40;
  for (long c = 0; c < cycles; c++) {
    if (c % 5000 == 0) { p_block = 20 + rng() % 80; p_consume = rng() % 100; p_commit = rng() % 100; }
    w.in = {};
    if (bit(p_block)) {
      for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
        w.in.valid[i] = bit(70);
        w.in.inst[i] = rng(); w.in.pc[i] = rng(); w.in.predict_dir[i] = bit(50); w.in.page_fault_inst[i] = bit(10);
        w.in.predict_next_fetch_address[i] = rng();
      }
      unsigned char *m = reinterpret_cast<unsigned char *>(&w.in.meta);
      for (size_t k = 0; k < sizeof(w.in.meta); k++) m[k] = rng() & 1 ? rng() : 0;
    }
    w.rob_bcast = {};
    w.rob_bcast.flush = bit(2);
    latch = {};
    latch.mispred = bit(3);
    latch.ftq_idx = rng();
    w.idu_consume = {};
    for (int i = 0; i < DECODE_WIDTH; i++) w.idu_consume.fire[i] = bit(p_consume);
    w.rob_commit = {};
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      auto &e = w.rob_commit.commit_entry[i];
      e.valid = bit(p_commit);
      e.uop.id = 1 + i;
      e.uop->ftq_idx = rng() % FTQ_SIZE;
      e.uop->ftq_offset = rng() % BACKEND_FETCH_WIDTH;
      e.uop.ftq_is_last = bit(30);
    }
    for (auto &r : w.ftq_prf_pc_req.req) { r.valid = bit(50); r.ftq_idx = rng() % FTQ_SIZE; r.ftq_offset = rng() % BACKEND_FETCH_WIDTH; }
    for (auto &r : w.ftq_rob_pc_req.req) { r.valid = bit(50); r.ftq_idx = rng() % FTQ_SIZE; r.ftq_offset = rng() % BACKEND_FETCH_WIDTH; }
    PRE(comb_begin); PRE(comb_accept_front); PRE(comb_ftq_lookup_rob); PRE(comb_commit_info);
    PRE(comb_ftq_lookup_prf); PRE(comb_fire); PRE(seq);
  }
  printf("pre_diff: ok\n");
  return 0;
}

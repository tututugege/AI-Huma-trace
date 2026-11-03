// Self-test of the trace bridge: a run with random wrong paths + rewinds must give
// exactly the same per-instruction facts, final registers and memory as a run without.
#include "../src/TraceBridge.h"
#include "refcpu_api.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <vector>
#include <random>

static const uint32_t RAM_BASE = 0x80000000u, RAM_BYTES = 0x80000000u;

static RefCpuContext *make(const std::vector<uint8_t> &img) {
  RefCpuContext *c = refcpu_init(RAM_BASE, RAM_BYTES);
  std::memcpy(refcpu_get_ram_ptr(c), img.data(), img.size());
  return c;
}
static uint64_t mem_hash(RefCpuContext *c) {
  uint64_t h = 1469598103934665603ull;
  auto mix = [&](uint32_t base, uint32_t bytes) {
    for (uint32_t a = base; a < base + bytes; a += 4) { h ^= refcpu_load_word(c, a); h *= 1099511628211ull; }
  };
  mix(RAM_BASE, 0x20000);              // image + data
  mix(RAM_BASE + 0x6000000u - 0x10000u, 0x10000);  // stack top (link.lds: 96 MB)
  return h;
}

int main(int argc, char **argv) {
  if (argc < 4) { fprintf(stderr, "usage: %s image.bin n_inst wrong_prob_percent [seed]\n", argv[0]); return 2; }
  std::ifstream f(argv[1], std::ios::binary);
  std::vector<uint8_t> img((std::istreambuf_iterator<char>(f)), {});
  const uint64_t N = strtoull(argv[2], 0, 10);
  const int pct = atoi(argv[3]);
  std::mt19937 rng(argc > 4 ? atoi(argv[4]) : 1);

  // reference: plain bridge, no wrong path
  RefCpuContext *c0 = make(img);
  tb::Bridge b0(c0, RAM_BASE, RAM_BYTES);
  std::vector<tb::Fact> ref;
  { RefCpuState s; for (uint64_t i = 0; i < N; i++) {
      refcpu_get_state(c0, &s);
      uint32_t inst = refcpu_load_word(c0, s.pc);
      if (inst == 0x00100073) break;
      ref.push_back(b0.dispatch(i, s.pc, inst));
      b0.commit(i);
  } }
  RefCpuState s0; refcpu_get_state(c0, &s0);

  // noisy: random wrong paths, rewinds, commit lag
  RefCpuContext *c1 = make(img);
  tb::Bridge b1(c1, RAM_BASE, RAM_BYTES);
  uint64_t next_uid = 0, committed = 0, mism = 0;
  std::vector<uint64_t> window;  // right-path uids not yet committed
  std::vector<std::pair<uint64_t,uint64_t>> right;  // (uid, ref index)
  auto text_words = (uint32_t)(img.size() / 4);
  for (size_t i = 0; i < ref.size(); i++) {
    if ((int)(rng() % 100) < pct) {
      RefCpuState snap; refcpu_get_state(c1, &snap);
      uint32_t k = 1 + rng() % 24;
      uint32_t pc = RAM_BASE + 4 * (rng() % (text_words ? text_words : 1));
      if (rng() % 8 == 0) pc = (uint32_t)rng() & ~3u;           // garbage pc
      if (rng() % 8 == 0) pc = RAM_BASE + 0x6000000u - 4 * (rng() % 64);  // stack/data region
      for (uint32_t j = 0; j < k; j++) {
        uint32_t inst = (pc >= RAM_BASE && (uint64_t)pc + 4 <= (uint64_t)RAM_BASE + RAM_BYTES) ? refcpu_load_word(c1, pc) : 0;
        const tb::Fact &w = b1.dispatch(next_uid++, pc, inst);
        pc = w.next_pc;
      }
      b1.rewind_after(window.empty() ? (int64_t)next_uid - (int64_t)k - 1 : (int64_t)window.back());
      { RefCpuState now; refcpu_get_state(c1, &now);
        for (int q = 0; q < 32; q++) if (now.gpr[q] != snap.gpr[q]) { printf("UNDO BUG before right inst %zu: x%d %x -> %x (k=%u)\n", i, q, snap.gpr[q], now.gpr[q], k); break; }
        if (now.pc != snap.pc) printf("UNDO BUG pc %x -> %x\n", snap.pc, now.pc); }
    }
    RefCpuState s; refcpu_get_state(c1, &s);
    if (s.pc != ref[i].pc) { printf("pc mismatch at %zu: bridge pc=0x%x ref pc=0x%x\n", i, s.pc, ref[i].pc); mism++; break; }
    uint64_t uid = next_uid++;
    const tb::Fact &g = b1.dispatch(uid, ref[i].pc, ref[i].inst);
    const tb::Fact &r = ref[i];
    if (g.next_pc != r.next_pc || g.rd_valid != r.rd_valid || (g.rd_valid && g.rd_value != r.rd_value) ||
        g.br_taken != r.br_taken || g.vaddr != r.vaddr || g.store_data != r.store_data || g.blocked != r.blocked) {
      printf("fact mismatch at inst %zu pc=0x%x inst=0x%08x: next %x/%x rd %d/%d val %x/%x br %d/%d vaddr %x/%x blk %d/%d\n", i, r.pc, r.inst, g.next_pc, r.next_pc, g.rd_valid, r.rd_valid, g.rd_value, r.rd_value, g.br_taken, r.br_taken, g.vaddr, r.vaddr, (int)g.blocked, (int)r.blocked); mism++; if (mism > 5) break;
    }
    window.push_back(uid);
    while (window.size() > 16) { b1.commit(window.front()); window.erase(window.begin()); committed++; }
  }
  while (!window.empty()) { b1.commit(window.front()); window.erase(window.begin()); }
  RefCpuState s1; refcpu_get_state(c1, &s1);
  bool regs = std::memcmp(s0.gpr, s1.gpr, sizeof s0.gpr) == 0 && s0.pc == s1.pc && std::memcmp(s0.csr, s1.csr, sizeof s0.csr) == 0;
  bool mem = mem_hash(c0) == mem_hash(c1);
  printf("insts=%zu mismatches=%llu regs_equal=%d csr/pc_equal=%d mem_equal=%d\n", ref.size(), (unsigned long long)mism, (int)regs, (int)regs, (int)mem);
  printf("noisy bridge: dispatched=%llu forced_jumps=%llu rewinds=%llu dropped=%llu trapped=%llu blocked[pc/ebreak/misalign/oor]=%llu/%llu/%llu/%llu\n",
         (unsigned long long)b1.stats.dispatched, (unsigned long long)b1.stats.forced_jumps, (unsigned long long)b1.stats.rewinds,
         (unsigned long long)b1.stats.entries_dropped, (unsigned long long)b1.stats.trapped,
         (unsigned long long)b1.stats.blocked[1], (unsigned long long)b1.stats.blocked[2], (unsigned long long)b1.stats.blocked[3], (unsigned long long)b1.stats.blocked[4]);
  return (mism == 0 && regs && mem) ? 0 : 1;
}

// Fast warm-up replay (EXP-011), see FastWarm.h. Model approximations: committed path only (no wrong-path fetches or
// accesses), page-table walks and the prefetchers are not replayed, caches see line-granular accesses in program order
// (the real L1D sees them out of order and merged by MSHRs), L1I replacement is LRU (the RTL is random), BPU updates are
// applied immediately (no update-queue delay, no speculative history).
#include "FastWarm.h"
#include "xlen_cast.h"
#include "BPU/BPU.h"
#include "Csr.h"
#include "DcacheConfig.h"
#include "AXI_LLC.h"
#include "MemSubsystem.h"
#include "PhysMemory.h"
#include "SimCpu.h"
#include "TlbMmu.h"
#include "config.h"
#include "diff.h"
#include "mmio_address_map.h"
#include "refcpu_api.h"

#include <cstdio>
#include <cstring>
#include <vector>

void front_warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc = false);
void front_warm_finish();
void icache_warm_install(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]);
void icache_warm_install_itlb(const void *entries, int count, int repl_ptr);
extern long long sim_time;

namespace {

constexpr uint32_t kMret = 0x30200073u;

inline uint32_t bits(uint32_t v, int hi, int lo) { return (v >> lo) & ((1u << (hi - lo + 1)) - 1u); }
inline int32_t sext(uint32_t v, int n) { return int32_t(v << (32 - n)) >> (32 - n); }

// Functional Sv32 TLB with the DUT's organisation: 64 entries, global or same-ASID match, round-robin refill.
struct TlbModel {
  std::vector<TlbMmu::TlbEntry> e;
  int ptr = 0;
  explicit TlbModel(int n) : e(n) {}
  void flush() { for (auto &x : e) x = TlbMmu::TlbEntry{}; ptr = 0; }
  static bool match(const TlbMmu::TlbEntry &x, uint32_t va, uint32_t asid) {
    if (!x.valid || !(x.global || x.asid == asid)) return false;
    if (x.vpn1 != ((va >> 22) & 0x3ff)) return false;
    return x.level == 1 || x.vpn0 == ((va >> 12) & 0x3ff);
  }
  // returns false on a page fault (leave untouched); *miss tells whether a refill happened
  bool translate(RefCpuContext *rc, uint32_t satp, uint32_t va, uint32_t &pa, bool &miss) {
    const uint32_t asid = (satp >> 22) & 0x1ff;
    miss = false;
    for (auto &x : e)
      if (match(x, va, asid)) { pa = compose(x, va); return true; }
    const uint32_t root = (satp & 0x3fffffu) << 12;
    uint32_t pte = refcpu_load_word(rc, root + ((va >> 22) & 0x3ff) * 4);
    int level = 1;
    if (!(pte & 1)) return false;
    if (!(pte & 0xe)) {
      pte = refcpu_load_word(rc, ((pte >> 10) << 12) + ((va >> 12) & 0x3ff) * 4);
      level = 0;
      if (!(pte & 1) || !(pte & 0xe)) return false;
    }
    miss = true;
    TlbMmu::TlbEntry n{};
    n.valid = true; n.global = (pte >> 5) & 1; n.asid = asid; n.level = level;
    n.vpn1 = (va >> 22) & 0x3ff; n.vpn0 = (va >> 12) & 0x3ff; n.ppn = (pte >> 10) & 0x3fffff; n.perm = pte & 0xff;
    e[ptr] = n;
    ptr = (ptr + 1) % int(e.size());
    pa = compose(n, va);
    return true;
  }
  static uint32_t compose(const TlbMmu::TlbEntry &x, uint32_t va) {
    if (x.level == 1) return ((x.ppn << 12) & ~((1u << 22) - 1u)) | (va & ((1u << 22) - 1u));
    return (x.ppn << 12) | (va & 0xfff);
  }
};

// L1I: 64 sets x 8 ways, tag = paddr[31:12] (the DUT indexes with the virtual address, same bits), LRU stand-in.
struct L1iModel {
  uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM] = {};
  bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM] = {};
  uint64_t stamp[ICACHE_SET_NUM][ICACHE_WAY_NUM] = {};
  uint64_t now = 0;
  bool access(uint32_t pa) {  // true on a miss
    const uint32_t set = (pa >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1), t = pa >> 12;
    ++now;
    int victim = 0;
    for (int w = 0; w < ICACHE_WAY_NUM; ++w) {
      if (valid[set][w] && tag[set][w] == t) { stamp[set][w] = now; return false; }
    }
    for (int w = 0; w < ICACHE_WAY_NUM; ++w) {
      if (!valid[set][w]) { victim = w; break; }
      if (stamp[set][w] < stamp[set][victim]) victim = w;
    }
    valid[set][victim] = true; tag[set][victim] = t; stamp[set][victim] = now;
    return true;
  }
};

// LLC: line-granular tag state with the DUT's policy (hit or refill leaves the round-robin pointer after the way).
struct LlcModel {
  axi_interconnect::AXI_LLCConfig cfg;
  uint32_t sets = 0, ways = 0;
  std::vector<uint32_t> tag;
  std::vector<uint8_t> valid, dirty;
  std::vector<uint32_t> repl;
  uint64_t dirty_lines = 0;
  void init(const axi_interconnect::AXI_LLCConfig &c) {
    cfg = c; sets = c.set_count(); ways = c.ways;
    tag.assign(size_t(sets) * ways, 0); valid.assign(tag.size(), 0); dirty.assign(tag.size(), 0); repl.assign(sets, 0);
  }
  bool enabled() const { return sets != 0; }
  // read: allocate clean on a miss; write: a dirty line (hit or allocate). Returns true on a miss.
  bool access(uint32_t pa, bool is_write) {
    const uint32_t set = axi_interconnect::AXI_LLC::set_index(cfg, pa), t = axi_interconnect::AXI_LLC::tag_of(cfg, pa);
    const size_t base = size_t(set) * ways;
    int hit = -1, inv = -1;
    for (uint32_t w = 0; w < ways; ++w) {
      if (valid[base + w]) { if (tag[base + w] == t) { hit = int(w); break; } }
      else if (inv < 0) inv = int(w);
    }
    const bool miss = hit < 0;
    const uint32_t way = hit >= 0 ? uint32_t(hit) : (inv >= 0 ? uint32_t(inv) : repl[set] % ways);
    if (miss) {
      if (valid[base + way] && dirty[base + way]) --dirty_lines;
      valid[base + way] = 1; tag[base + way] = t; dirty[base + way] = 0;
    }
    if (is_write && !dirty[base + way]) { dirty[base + way] = 1; ++dirty_lines; }
    repl[set] = (way + 1) % ways;
    return miss;
  }
};

struct Warm {
  TlbModel itlb{ITLB_ENTRIES}, dtlb{DTLB_ENTRIES};
  L1iModel l1i;
  LlcModel llc;
  bool llc_ready = false;
};
Warm *g_warm = nullptr;

inline bool is_ram(uint32_t pa) { return pmem_is_ram_addr(pa, 4); }

}  // namespace

bool fast_warm_replay(SimCpu &cpu, bool skip_until_mret, uint64_t steps, FastWarmStats &st) {
  RefCpuContext *rc = difftest_ref_ctx();
  delete g_warm;
  g_warm = new Warm();
  Warm &W = *g_warm;
  if (cpu.axi_interconnect.llc_enabled()) W.llc.init(cpu.axi_interconnect.get_llc_config());
  difftest_ref_set_uart_print(false);
  difftest_ref_set_ref_only(true);
  difftest_ref_set_device_effects(true);
  difftest_ref_set_interrupt_delivery(true);
  refcpu_set_sim_time(rc, static_cast<uint64_t>(sim_time));

  RefCpuStepInfo info{};
  if (skip_until_mret) {
    for (;;) {
      refcpu_step(rc, 1);
      refcpu_get_step_info(rc, &info);
      ++st.stub_steps;
      if (info.instruction == kMret) break;
      if (info.sim_end || st.stub_steps > 1000000) { st.ended_early = true; return false; }
    }
  }

  front_warm_finish();  // no-op on a fresh BPU; keeps spec == arch
  uint32_t last_iline = 0xffffffffu, last_ipa_line = 0;
  RefCpuState s{}, post{};
  for (uint64_t i = 0; i < steps; ++i) {
    refcpu_get_state(rc, &s);
    const uint32_t pc = narrow_pc(s.pc, "warm-up pc");
    const xlen_t satp = s.csr[csr_satp];
    const bool vm = satp_on(satp) && s.privilege != 3;
    // instruction line: L1I + ITLB (translate once per line)
    const uint32_t iline = pc >> ICACHE_OFFSET_BITS;
    if (iline != last_iline) {
      uint32_t ipa = pc;
      bool ok = true, miss = false;
      if (vm) ok = W.itlb.translate(rc, narrow_pc(satp, "satp"), pc, ipa, miss);   // (the warm-up TLB model is Sv32 only)
      if (miss) ++st.itlb_miss;
      if (ok && is_ram(ipa)) {
        ++st.ifetch_lines;
        if (W.l1i.access(ipa)) {
          ++st.ifetch_miss;
          if (W.llc.enabled()) { ++st.llc_acc; if (W.llc.access(ipa, false)) ++st.llc_miss; }
        }
      }
      last_iline = iline;
    }
    refcpu_step(rc, 1);
    refcpu_get_step_info(rc, &info);
    ++st.steps;
    if (info.sim_end) { st.ended_early = true; return false; }
    const uint32_t inst = info.instruction;
    if (inst == 0x12000073u || (bits(inst, 6, 0) == 0x73 && bits(inst, 31, 25) == 0x09 && bits(inst, 14, 7) == 0)) {
      W.itlb.flush(); W.dtlb.flush();  // sfence.vma
    }
    if (info.is_exception || info.page_fault_inst) { ++st.skipped_exc; continue; }
    const uint32_t op = bits(inst, 6, 0);
    // branch predictor: every committed instruction updates the type predictor, control flow the rest
    if (op == 0x63 || op == 0x6f || op == 0x67) {
      refcpu_get_state(rc, &post);
      int type = BR_DIRECT;
      bool dir = true;
      if (op == 0x63) { dir = info.br_taken; }
      else if (op == 0x6f) type = (bits(inst, 11, 7) == 1) ? BR_CALL : BR_JAL;
      else type = (bits(inst, 19, 15) == 1 && bits(inst, 11, 7) == 0 && bits(inst, 31, 20) == 0) ? BR_RET : (bits(inst, 11, 7) == 1) ? BR_ICALL : BR_IDIRECT;
      front_warm_commit(pc, type, dir, dir ? narrow_pc(post.pc, "warm-up target") : pc + info.inst_len, info.inst_len == 2);
      ++st.branches;
    } else {
      front_warm_commit(pc, BR_NONCTL, false, pc + info.inst_len, info.inst_len == 2);
    }
    // data side
    const bool is_ld = op == 0x03 || (FP && op == 0x07), is_st = op == 0x23 || (FP && op == 0x27), is_amo = op == 0x2f;
    if ((is_ld || is_st || is_amo) && !info.page_fault_load && !info.page_fault_store && !info.is_io) {
      const xlen_t base = s.gpr[bits(inst, 19, 15)];
      const xlen_t va_x = base + (is_ld ? xlen_t(sxlen_t(sext(bits(inst, 31, 20), 12)))
                                : is_st ? xlen_t(sxlen_t(sext((bits(inst, 31, 25) << 5) | bits(inst, 11, 7), 12))) : xlen_t(0));
      const uint32_t va = narrow_pc(va_x, "warm-up address");
      uint32_t pa = va;
      bool ok = true, miss = false;
      if (vm) ok = W.dtlb.translate(rc, narrow_pc(satp, "satp"), va, pa, miss);
      if (miss) ++st.dtlb_miss;
      if (ok && is_ram(pa)) {
        const bool wr = is_st || (is_amo && bits(inst, 31, 27) != 2);
        wr ? ++st.stores : ++st.loads;
        uint32_t wb;
        if (dcache_warm_access(pa, wr, &wb)) {
          ++st.l1d_miss;
          if (W.llc.enabled()) {
            if (wb != 0xffffffffu) W.llc.access(wb, true);
            ++st.llc_acc;
            if (W.llc.access(pa, false)) ++st.llc_miss;
          }
        }
      }
    }
  }
  return true;
}

void fast_warm_install(SimCpu &cpu) {
  Warm &W = *g_warm;
  // The trace bridge steps the core that owns the RAM (pmem_core_ctx()), not the difftest reference the replay ran on:
  // give it the architectural state and the device registers the replay reached (its RAM is the one restore_from_ref
  // already filled).
  RefCpuContext *rc = difftest_ref_ctx();
  if (RefCpuContext *core = pmem_core_ctx(); core != nullptr && core != rc) {
    RefCpuState rs{};
    refcpu_get_state(rc, &rs);
    refcpu_set_state(core, &rs);
    refcpu_clear_io_words(core);
    for (const auto &range : kCheckpointMmioRanges)
      for (uint32_t off = 0; off + 4u <= range.size; off += 4u)
        refcpu_set_io_word(core, range.base + off, refcpu_get_io_word(rc, range.base + off));
    refcpu_sync_mmio_devices_from_backing(core);
  }
  front_warm_finish();
  icache_warm_install(W.l1i.tag, W.l1i.valid);
  icache_warm_install_itlb(W.itlb.e.data(), int(W.itlb.e.size()), W.itlb.ptr);
  cpu.back.warm_dtlb(W.dtlb.e.data(), int(W.dtlb.e.size()), W.dtlb.ptr);
  if (W.llc.enabled()) {
    for (uint32_t set = 0; set < W.llc.sets; ++set) {
      const size_t b = size_t(set) * W.llc.ways;
      cpu.mem_subsystem.llc_warm_set(set, &W.llc.tag[b], &W.llc.valid[b], &W.llc.dirty[b], W.llc.repl[set]);
    }
    cpu.axi_interconnect.warm_set_llc_dirty_lines(uint32_t(W.llc.dirty_lines));
  }
}

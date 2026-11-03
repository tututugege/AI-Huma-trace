// Canonical state digest shared by the reference LSU and the real one (same value iff same logical state).
#pragma once
#include <cstdint>
struct LsuDigest {
  uint64_t h = 1469598103934665603ull;
  void mix(uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; }
  template <class T> void operator()(const T &v) { mix(static_cast<uint64_t>(v)); }
};
// ldq: array of entries with the original field names, stq: array of StqEntry, queues: head/count/entries.
template <class State, class LoadStateEnum>
uint64_t lsu_digest(const State &s, LoadStateEnum empty) {
  LsuDigest d;
  for (int i = 0; i < LDQ_SIZE; i++) {
    const auto &e = s.ldq[i];
    d(static_cast<uint32_t>(e.load_state));
    if (e.load_state == empty) continue;
    d(e.v_addr_valid); d(e.v_addr); d(e.p_addr_valid); d(e.p_addr); d(static_cast<uint32_t>(e.replay_type));
    d(e.func3); d(e.result); d(e.dest_preg); d(e.rob_idx); d(e.rob_flag); d(e.br_mask); d(e.is_mmio);
    d(e.diag_val); d(e.page_fault); d(e.is_lrsc); d(e.stq_snapshot.idx); d(e.stq_snapshot.flag); d(e.cache_miss);
  }
  d(s.stq_head); d(s.stq_commit); d(s.stq_head_flag); d(s.stq_commit_count); d(s.stq_count);
  for (uint32_t i = 0; i < (uint32_t)s.stq_count; i++) {
    const auto &e = s.stq[(s.stq_head + i) % STQ_SIZE];
    d(e.data_valid); d(e.data); d(e.func3); d(e.vaddr_valid); d(e.vaddr); d(e.paddr_valid); d(e.paddr); d(e.page_fault);
    d(e.suppress_write); d(e.is_mmio); d(e.is_lrsc); d(e.sc_pass); d(e.dest_preg); d(static_cast<uint32_t>(e.store_state));
    d(static_cast<uint32_t>(e.replay_type)); d(e.replay_wait_cycles); d(e.br_mask); d(e.rob_idx); d(e.rob_flag); d(e.stq_flag);
  }
  auto q = [&](const auto *arr, uint32_t head, uint32_t count, uint32_t size, auto fn) {
    d(head); d(count);
    for (uint32_t i = 0; i < count; i++) { const auto &e = arr[(head + i) % size]; d(e.valid); fn(e); }
  };
  q(s.wait_mmu_stq, s.wait_mmu_stq_head, s.wait_mmu_stq_count, STQ_SIZE, [&](const auto &e) { d(e.stq_idx); });
  q(s.wait_mmu_ldq, s.wait_mmu_ldq_head, s.wait_mmu_ldq_count, LDQ_SIZE, [&](const auto &e) { d(e.ldq_idx); });
  q(s.mmu_done_stq, s.mmu_done_stq_head, s.mmu_done_stq_count, STQ_SIZE, [&](const auto &e) { d(e.stq_idx); });
  q(s.finish, s.finish_head, s.finish_count, LDQ_SIZE, [&](const auto &e) { d(e.ldq_idx); });
  q(s.stlf_queue, s.stlf_queue_head, s.stlf_queue_count, LDQ_SIZE, [&](const auto &e) { d(e.ldq_idx); });
  q(s.wait_dcache_ldq, s.wait_dcache_ldq_head, s.wait_dcache_ldq_count, LDQ_SIZE,
    [&](const auto &e) { d(e.ldq_idx); d(e.req_gen); d(e.replay_wait_cycles); });
  d(s.uncached_unit.valid); d(s.uncached_unit.is_load); d(s.uncached_unit.addr); d(s.uncached_unit.wdata);
  d(s.uncached_unit.func3); d(s.uncached_unit.idx);
  d(s.lrsc_unit.reserve_valid); d(s.lrsc_unit.reserve_addr); d(s.lrsc_unit.reserve_rob_idx);
  d(s.lrsc_unit.reserve_rob_flag); d(s.lrsc_unit.reserve_br_mask);
  d(s.req_gen);
  return d.h;
}

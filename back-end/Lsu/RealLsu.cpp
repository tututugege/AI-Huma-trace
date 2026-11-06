#include "Wires.h"
#include "RealLsu.h"
#include "DcacheConfig.h"
#include "MemUtils.h"
#include "PhysMemory.h"
#include "SimCpu.h"
#include "TlbMmu.h"
#include "config.h"
#include "trace_hook.h"
#include "util.h"
#include <algorithm>
#include <cstdint>
#include <cstring>

using std::min;

static uint32_t replay_delay_cycles(ReplayType replay) {
  return replay == ReplayType::MSHR_HIT || replay == ReplayType::MSHR_FULL ? LSU_PEPLAY_WAIT_CYCLES : 0;
}

// A DCache fill wakes loads waiting on that line (MSHR_HIT) and a few waiting for a free MSHR (MSHR_FULL).
static void wake_load_replays_on_mshr_fill(const DcacheLsuIO &resp, LsuState &nxt) {
  if (!resp.mshr_fill || !LSU_WAIT_MSHR_FILL) return;
  uint32_t full_wakeups = 0;
  const uint32_t n = nxt.wait_dcache_ldq.window(LSU_LOAD_WINDOW_WIDTH);
  for (uint32_t i = 0; i < n; i++) {
    QWait &w = nxt.wait_dcache_ldq.e[nxt.wait_dcache_ldq.at(i)];
    if (!w.valid || w.replay_wait_cycles == 0) continue;
    const LdqEntry &l = nxt.ldq[w.idx];
    if (l.load_state != LoadState::ReadyToIssue) continue;
    if (l.replay_type == ReplayType::MSHR_HIT && cache_line_match(l.p_addr, resp.mshr_fill_addr)) {
      w.replay_wait_cycles = 0;
    } else if (l.replay_type == ReplayType::MSHR_FULL && full_wakeups < LSU_LDU_COUNT) {
      w.replay_wait_cycles = 0;
      full_wakeups++;
    }
  }
}

static void wake_store_replays_on_mshr_fill(const DcacheLsuIO &resp, LsuState &nxt) {
  if (!resp.mshr_fill || !LSU_WAIT_MSHR_FILL) return;
  uint32_t full_wakeups = 0;
  const uint32_t n = min<uint32_t>(nxt.stq_commit_count, nxt.stq_count);
  for (uint32_t i = 0; i < n; i++) {
    StqEntry &s = nxt.stq[(nxt.stq_head + i) % STQ_SIZE];
    if (s.store_state != StoreState::Committed || s.replay_wait_cycles == 0) continue;
    if (s.replay_type == ReplayType::MSHR_HIT && cache_line_match(s.paddr, resp.mshr_fill_addr)) {
      s.replay_wait_cycles = 0;
    } else if (s.replay_type == ReplayType::MSHR_FULL && full_wakeups < LSU_STA_COUNT) {
      s.replay_wait_cycles = 0;
      full_wakeups++;
    }
  }
}

static STLFResult check_stlf(uint32_t load_addr, uint32_t load_func3, uint32_t store_addr, uint32_t store_func3) {
  const uint32_t s_end = store_addr + get_mem_width(store_func3);
  const uint32_t l_end = load_addr + get_mem_width(load_func3);
  if (store_addr <= load_addr && s_end >= l_end) return STLFResult::Overlap;  // store covers the load
  if (l_end <= store_addr || load_addr >= s_end) return STLFResult::Disjoint;
  return STLFResult::Retry;  // partial overlap: conservatively a conflict
}

static uint64_t load_miss_mask(const LsuState &s) {
  uint64_t m = 0;
  for (const LdqEntry &l : s.ldq)
    if (l.cache_miss && l.load_state != LoadState::Empty) m |= 1ULL << l.rob_idx;
  return m;
}

void RealLsu::comb_cal() {
  ldq_allocated_ = ldq_empty_ = stq_allocated_ = 0;
  for (const LdqEntry &l : cur.ldq) {
    ldq_allocated_ += l.load_state == LoadState::Allocated;
    ldq_empty_ += l.load_state == LoadState::Empty;
  }
  for (int i = 0; i < cur.stq_count; i++)
    stq_allocated_ += cur.stq[(cur.stq_head + i) % STQ_SIZE].store_state == StoreState::Allocated;
}

void RealLsu::comb_lsu2rob() {
  wires->lsu2rob = {};
  wires->lsu2rob.committed_store_pending = cur.stq_commit_count != 0;
  wires->lsu2rob.tma.miss_mask = load_miss_mask(cur);
}

void RealLsu::comb_lsu2dis() {
  wires->lsu2dis = {};
  wires->lsu2dis.stq_tail = stq_idx_after(cur.stq_head, cur.stq_count);
  wires->lsu2dis.stq_tail_flag = stq_tail_flag(cur.stq_head, cur.stq_count, cur.stq_head_flag);

  uint32_t ldq_free = min<uint32_t>(LDQ_SIZE - ldq_allocated_ - cur.wait_mmu_ldq.count, ldq_empty_);
  uint32_t stq_free = STQ_SIZE - stq_allocated_ - std::max<uint32_t>(cur.wait_mmu_stq.count, cur.mmu_done_stq.count);
  stq_free = min<uint32_t>(stq_free, STQ_SIZE - cur.stq_count);
  wires->lsu2dis.stq_free = stq_free;
  wires->lsu2dis.ldq_free = ldq_free;

  uint32_t n = 0;
  for (int i = 0; i < LDQ_SIZE && n < MAX_LDQ_DISPATCH_WIDTH && n < ldq_free; i++)
    if (cur.ldq[i].load_state == LoadState::Empty) {
      wires->lsu2dis.ldq_alloc_idx[n] = i;
      wires->lsu2dis.ldq_alloc_valid[n++] = true;
    }
}

void RealLsu::comb_dis2lsu() {
  const DisLsuIO &d = wires->dis2lsu;
  for (int i = 0; i < MAX_LDQ_DISPATCH_WIDTH; i++) {
    if (!d.ldq_alloc_req[i]) continue;
    LdqEntry &e = nxt.ldq[d.ldq_idx[i]];
    Assert(e.load_state == LoadState::Empty && "LDQ allocate collision");
    e = {};
    e.rob_idx = d.ldq_rob_idx[i];
    e.rob_flag = d.ldq_rob_flag[i];
    e.br_mask = d.ldq_br_mask[i];
    e.load_state = LoadState::Allocated;
  }
  for (int i = 0; i < MAX_STQ_DISPATCH_WIDTH; i++) {
    if (!d.alloc_req[i]) continue;
    Assert(nxt.stq_count < STQ_SIZE && "STQ allocate overflow");
    if (d.stq_flag[i] != stq_tail_flag(nxt.stq_head, nxt.stq_count, nxt.stq_head_flag)) {
      LSU_NON_BSD_ASSERT(0 && "STQ slot flag mismatch");
      Assert(0 && "STQ allocate overflow");
      continue;
    }
    StqEntry &e = nxt.stq[stq_idx_after(nxt.stq_head, nxt.stq_count)];
    e = {};
    e.rob_idx = d.rob_idx[i];
    e.rob_flag = d.rob_flag[i];
    e.stq_flag = d.stq_flag[i];
    e.br_mask = d.br_mask[i];
    e.func3 = d.func3[i];
    e.store_state = StoreState::Allocated;
    nxt.stq_count++;
  }
}

void RealLsu::comb_exe2lsu() {
  const ExeLsuIO &x = wires->exe2lsu;
  for (int i = 0; i < LSU_SDU_COUNT; i++)
    if (x.sdu_req[i].valid) handle_store_data(x.sdu_req[i].uop);
  for (int i = 0; i < LSU_AGU_COUNT; i++)
    if (x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_STA) handle_store_addr(x.agu_req[i].uop);
  for (int i = 0; i < LSU_AGU_COUNT; i++)
    if (x.agu_req[i].valid && x.agu_req[i].uop.op == UOP_LOAD) handle_load_req(x.agu_req[i].uop);
}

void RealLsu::handle_store_data(const MicroOp &inst) {
  if (inst.stq_idx >= STQ_SIZE) return;
  StqEntry &e = nxt.stq[inst.stq_idx];
  if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) {
    LSU_NON_BSD_ASSERT(0 && "Store data mismatch");
    return;
  }
  e.data = inst.result;
  e.data_valid = true;
  if (e.paddr_valid && e.store_state == StoreState::WaitData) {
    e.store_state = StoreState::Ready;
    nxt.mmu_done_stq.push(inst.stq_idx);
  }
}

void RealLsu::handle_store_addr(const MicroOp &inst) {
  if (inst.stq_idx >= STQ_SIZE) return;
  StqEntry &e = nxt.stq[inst.stq_idx];
  if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) {
    LSU_NON_BSD_ASSERT(0 && "Store addr mismatch");
    return;
  }
  e.vaddr_valid = true;
  e.vaddr = inst.result;
  e.paddr_valid = false;
  e.paddr = 0;
  e.func3 = inst->func3;
  e.dest_preg = inst.dest_preg;
  e.store_state = StoreState::WaitTlb;
  nxt.wait_mmu_stq.push(inst.stq_idx);
  e.is_lrsc = inst->is_atomic && ((inst->func7 >> 2) == AmoOp::SC);
}

void RealLsu::handle_load_req(const MicroOp &inst) {
  if (inst.ldq_idx >= LDQ_SIZE) return;
  LdqEntry &e = nxt.ldq[inst.ldq_idx];
  if (e.rob_flag != inst.rob_flag || e.rob_idx != inst.rob_idx) {
    LSU_NON_BSD_ASSERT(e.load_state == LoadState::Allocated && "Load req mismatch with non-empty entry");
    LSU_NON_BSD_ASSERT(0 && "Load req mismatch");
    return;
  }
  e.v_addr_valid = true;
  e.v_addr = inst.result;
  e.dest_preg = inst.dest_preg;
  e.p_addr_valid = false;
  e.p_addr = 0;
  e.func3 = inst->func3;
  e.pc_hash = pf_pc_hash(inst->dbg.pc);   // the PC of the load (debug sideband in the model; hardware reads it from the FTQ)
#ifdef PF_STUDY
  e.pc_full = inst->dbg.pc;
#endif
  e.load_state = LoadState::WaitTlb;
  e.stq_snapshot.idx = inst.stq_idx;
  e.stq_snapshot.flag = inst.stq_flag;
  e.is_lrsc = is_amo_lr_uop(inst);
  nxt.wait_mmu_ldq.push(inst.ldq_idx);
}

void RealLsu::comb_mmio_out() {
  wires->out.peripheral_req = {};
  if (!wires->in.peripheral_resp.ready) return;
  const UncachedUnit &u = nxt.uncached_unit;
  wires->out.peripheral_req.is_mmio = u.valid;
  if (u.valid) {
    wires->out.peripheral_req.wen = !u.is_load;
    wires->out.peripheral_req.mmio_addr = u.addr;
    wires->out.peripheral_req.mmio_wdata = u.wdata;
    wires->out.peripheral_req.mmio_fun3 = u.func3;
  }
}

void RealLsu::comb_mmio_in() {
  if (!(wires->in.peripheral_resp.is_mmio && cur.uncached_unit.valid)) return;
  const uint32_t idx = cur.uncached_unit.idx;
  if (cur.uncached_unit.is_load) {
    if (nxt.finish.count >= LDQ_SIZE) return;
    LdqEntry &e = nxt.ldq[idx];
    if (e.load_state == LoadState::WaitMmioResp) {
      e.result = wires->in.peripheral_resp.mmio_rdata;
      e.load_state = LoadState::ReadyToWb;
      nxt.finish.push(idx);
    }
  } else if (nxt.stq[idx].store_state == StoreState::WaitMmioResp) {
    nxt.stq[idx].store_state = StoreState::Done;
  }
  nxt.uncached_unit.valid = false;
}

void RealLsu::comb_tlb_out() {
  wires->lsu2mmu_io = {};
  wires->lsu2mmu_io.csr_status = wires->csr_status;

  uint32_t issued = 0;
  const uint32_t nl = nxt.wait_mmu_ldq.window(LSU_LOAD_WINDOW_WIDTH);
  for (uint32_t i = 0; i < nl && issued < LSU_LDU_COUNT; i++) {
    const uint32_t wait_idx = nxt.wait_mmu_ldq.at(i);
    const QIdx &w = nxt.wait_mmu_ldq.e[wait_idx];
    const LdqEntry &l = nxt.ldq[w.idx];
    if (w.valid && l.load_state == LoadState::WaitTlb) {
      auto &r = wires->lsu2mmu_io.ldq_req[issued++];
      r.valid = true;
      r.vaddr = l.v_addr;
      r.entry_idx = w.idx;
      r.wait_idx = wait_idx;
    }
  }
  issued = 0;
  const uint32_t ns = nxt.wait_mmu_stq.window(LSU_STORE_WINDOW_WIDTH);
  for (uint32_t i = 0; i < ns && issued < LSU_STA_COUNT; i++) {
    const uint32_t wait_idx = nxt.wait_mmu_stq.at(i);
    const QIdx &w = nxt.wait_mmu_stq.e[wait_idx];
    const StqEntry &s = nxt.stq[w.idx];
    if (w.valid && s.store_state == StoreState::WaitTlb) {
      auto &r = wires->lsu2mmu_io.stq_req[issued++];
      r.valid = true;
      r.vaddr = s.vaddr;
      r.entry_idx = w.idx;
      r.wait_idx = wait_idx;
    }
  }
}

void RealLsu::comb_tlb_in_ldq() {
  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    const auto &resp = wires->mmu2lsu_io.ldq_resp[i];
    if (!resp.valid || resp.wait_idx >= LDQ_SIZE || resp.entry_idx >= LDQ_SIZE) continue;
    QIdx &w = nxt.wait_mmu_ldq.e[resp.wait_idx];
    if (!w.valid || w.idx != resp.entry_idx) continue;
    LdqEntry &l = nxt.ldq[resp.entry_idx];
    if (l.load_state != LoadState::WaitTlb) {
      w.valid = false;
      continue;
    }
    if (resp.result == MMUResultType::HIT) {
      l.p_addr_valid = true;
      l.p_addr = resp.paddr;
      l.is_mmio = lsu_is_mmio_addr(resp.paddr);
      l.load_state = LoadState::WaitSTLF;
    } else if (resp.result == MMUResultType::PAGE_FAULT) {
      l.page_fault = true;
      l.diag_val = l.v_addr;
      l.result = l.v_addr;
      l.load_state = LoadState::WaitFinish;  // the exception is raised once the load reaches the finish queue
      nxt.lrsc_unit.reserve_valid = false;
    } else if (resp.result == MMUResultType::MISS) {
      l.load_state = LoadState::WaitTlbRetry;
    }
  }

  uint32_t stlf_issue = 0, finish_issue = 0;
  const uint32_t n = nxt.wait_mmu_ldq.window(LSU_LOAD_WINDOW_WIDTH);
  const uint32_t head = nxt.wait_mmu_ldq.head;
  for (uint32_t i = 0; i < n; i++) {
    QIdx &w = nxt.wait_mmu_ldq.e[(head + i) % LDQ_SIZE];
    if (!w.valid) continue;
    LdqEntry &l = nxt.ldq[w.idx];
    if (l.load_state == LoadState::WaitSTLF) {
      if (stlf_issue < LSU_LDU_COUNT && nxt.stlf_queue.count < LDQ_SIZE) {
        stlf_issue++;
        l.load_state = LoadState::CheckStlf;
        nxt.stlf_queue.push(w.idx);
        w.valid = false;
      }
    } else if (l.load_state == LoadState::WaitFinish) {
      if (finish_issue < LSU_LDU_COUNT && nxt.finish.count < LDQ_SIZE) {
        finish_issue++;
        l.load_state = LoadState::PageFault;
        nxt.finish.push(w.idx);
        w.valid = false;
      }
    } else if (l.load_state == LoadState::WaitTlbRetry) {
      l.load_state = LoadState::WaitTlb;
    }
  }
  nxt.wait_mmu_ldq.trim(nxt.wait_mmu_ldq.window(LSU_LOAD_WINDOW_WIDTH));
}

void RealLsu::comb_tlb_in_stq() {
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    const auto &resp = wires->mmu2lsu_io.stq_resp[i];
    if (!resp.valid || resp.wait_idx >= STQ_SIZE || resp.entry_idx >= STQ_SIZE) continue;
    const QIdx &w = nxt.wait_mmu_stq.e[resp.wait_idx];
    if (!w.valid || w.idx != resp.entry_idx) continue;
    StqEntry &s = nxt.stq[resp.entry_idx];
    if (s.store_state != StoreState::WaitTlb) continue;
    if (resp.result == MMUResultType::HIT) {
      s.paddr_valid = true;
      s.paddr = resp.paddr;
      s.is_mmio = lsu_is_mmio_addr(resp.paddr);
      s.store_state = s.data_valid ? StoreState::WaitDone : StoreState::WaitData;
    } else if (resp.result == MMUResultType::PAGE_FAULT) {
      s.page_fault = true;
      s.store_state = StoreState::WaitDone;
      nxt.lrsc_unit.reserve_valid = false;
    } else if (resp.result == MMUResultType::MISS) {
      s.store_state = StoreState::WaitTlbRetry;
    }
  }

  const uint32_t n = nxt.wait_mmu_stq.window(LSU_STORE_WINDOW_WIDTH);
  const uint32_t head = nxt.wait_mmu_stq.head;
  for (uint32_t i = 0; i < n; i++) {
    QIdx &w = nxt.wait_mmu_stq.e[(head + i) % STQ_SIZE];
    if (!w.valid) continue;
    StqEntry &s = nxt.stq[w.idx];
    if (s.store_state == StoreState::WaitDone) {
      s.store_state = s.page_fault ? StoreState::PageFault : StoreState::Ready;
      nxt.mmu_done_stq.push(w.idx);
      w.valid = false;
    } else if (s.store_state == StoreState::WaitData) {
      w.valid = false;  // the store data write re-enters the flow
    } else if (s.store_state == StoreState::WaitTlbRetry) {
      s.store_state = StoreState::WaitTlb;
    }
  }
  nxt.wait_mmu_stq.trim(nxt.wait_mmu_stq.window(LSU_STORE_WINDOW_WIDTH));
}

// Store-to-load forwarding check. FIFO style: entries leaving the queue are popped first, then the
// retried ones go back to the tail.
void RealLsu::comb_stlf() {
  auto &q = nxt.stlf_queue;
  const uint32_t issue = q.window(LSU_LOAD_WINDOW_WIDTH);
  wire<LDQ_IDX_WIDTH> retry[LSU_LOAD_WINDOW_WIDTH] = {};
  uint32_t retries = 0, pops = 0;

  for (uint32_t i = 0; i < issue; i++) {
    QIdx &sq = q.e[q.at(i)];
    if (!sq.valid) {
      pops++;
      continue;
    }
    const uint32_t ldq_idx = sq.idx;
    LdqEntry &l = nxt.ldq[ldq_idx];
    bool pop = false, retry_it = false;

    if (l.load_state == LoadState::Empty) {
      pop = true;
    } else if (l.load_state == LoadState::CheckStlf) {
      uint32_t older = 0;
      if (!stq_distance_from_head_to_boundary(nxt.stq_head, nxt.stq_head_flag, nxt.stq_count, l.stq_snapshot, older)) {
        LSU_NON_BSD_ASSERT(0 && "LDQ STQ snapshot boundary is outside active STQ window");
        break;
      }
      if (l.is_mmio) {
        if (older == 0 && lsu_mmio_is_oldest_unfinished(&wires->rob_bcast, l.rob_idx) && !nxt.uncached_unit.valid) {
          l.load_state = LoadState::WaitMmioResp;
          UncachedUnit &u = nxt.uncached_unit;  // wdata is left untouched on purpose (it reaches the bus for loads too)
          u.valid = true;
          u.is_load = true;
          u.addr = l.p_addr;
          u.func3 = l.func3;
          u.idx = ldq_idx;
        } else {
          l.load_state = LoadState::CheckStlfRetry;
        }
      } else {
        uint32_t disjoint = 0;
        for (int j = int(older) - 1; j >= 0; j--) {
          const StqEntry &s = nxt.stq[(nxt.stq_head + j) % STQ_SIZE];
          // A cancelled store cannot forward; keep looking at older stores.
          if (s.suppress_write) { disjoint++; continue; }
          if (s.store_state == StoreState::Allocated || !s.paddr_valid) { l.load_state = LoadState::CheckStlfRetry; break; }
          const STLFResult r = check_stlf(l.p_addr, l.func3, s.paddr, s.func3);
          if (r == STLFResult::Overlap) {
            // An undecided or faulting SC must hold the load; a store without data cannot forward yet.
            const bool hold = (s.is_lrsc && !s.sc_pass) || !s.data_valid;
            l.load_state = hold ? LoadState::CheckStlfRetry : LoadState::WaitFinish;
            break;
          }
          if (r == STLFResult::Retry) { l.load_state = LoadState::CheckStlfRetry; break; }
          disjoint++;
        }
        if (disjoint == older) l.load_state = LoadState::WaitDcache;
      }
    }

    if (l.load_state == LoadState::WaitDcache) {
      if (nxt.wait_dcache_ldq.count >= LDQ_SIZE) break;
      nxt.wait_dcache_ldq.push(sq.idx);
      l.load_state = LoadState::ReadyToIssue;
      pop = true;
    } else if (l.load_state == LoadState::WaitFinish) {
      if (nxt.finish.count >= LDQ_SIZE) break;
      nxt.finish.push(sq.idx);
      l.load_state = LoadState::ReadyToWb;
      pop = true;
    } else if (l.load_state == LoadState::WaitMmioResp) {
      pop = true;
    } else if (l.load_state == LoadState::CheckStlfRetry) {
      retry_it = pop = true;
      l.load_state = LoadState::CheckStlf;
    } else if (!pop) {
      break;
    }

    if (pop) {
      sq.valid = false;
      pops++;
      if (retry_it) retry[retries++] = sq.idx;
    }
  }
  while (pops--) q.pop();
  for (uint32_t i = 0; i < retries; i++) {
    LSU_NON_BSD_ASSERT(q.count < LDQ_SIZE && "STLF retry requeue overflow");
    if (q.count < LDQ_SIZE) q.push(retry[i]);
  }
}

void RealLsu::comb_lsu2dcache_ldq() {
  auto &ports = wires->out.lsu2dcache.req_ports.load_ports;
  for (int i = 0; i < LSU_LDU_COUNT; i++) ports[i].valid = ports[i].replay = false;

  uint32_t issued = 0;
  const uint32_t n = nxt.wait_dcache_ldq.window(LSU_LOAD_WINDOW_WIDTH);
  for (uint32_t scan = 0; scan < n && issued < LSU_LDU_COUNT; scan++) {
    const uint32_t wait_idx = nxt.wait_dcache_ldq.at(scan);
    QWait &w = nxt.wait_dcache_ldq.e[wait_idx];
    if (!w.valid) continue;
    LdqEntry &l = nxt.ldq[w.idx];
    if (l.load_state != LoadState::ReadyToIssue) continue;
    if (w.replay_wait_cycles > 0 && --w.replay_wait_cycles > 0) continue;
    LSU_NON_BSD_ASSERT(!l.is_mmio && "MMIO load must not be issued to DCache");

    const uint32_t gen = normalize_lsu_req_gen(cur.req_gen + issued);
    ports[issued].valid = true;
    ports[issued].addr = l.p_addr;
    ports[issued].req_id = make_lsu_load_req_id(wait_idx, gen);
    ports[issued].replay = l.replay_type != ReplayType::HIT;
    ports[issued].pc_hash = l.pc_hash;
#ifdef PF_STUDY
    ports[issued].pc_full = l.pc_full;
#endif
    w.req_gen = gen;
    l.load_state = LoadState::WaitDcacheResp;
    issued++;
  }
  nxt.req_gen = normalize_lsu_req_gen(cur.req_gen + issued);
}

void RealLsu::comb_dcache2lsu_ldq() {
  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    const auto &resp = wires->in.dcache2lsu.resp_ports.load_resps[i];
    if (!resp.valid) continue;
    if (resp.req_id & (1u << 31)) {
      LSU_NON_BSD_ASSERT(0 && "Invalid load response ID from DCache");
      continue;
    }
    const uint32_t entry_idx = lsu_req_id_wait_idx(resp.req_id);
    const QWait &w = nxt.wait_dcache_ldq.e[entry_idx];
    if (!w.valid) continue;  // stale response after a flush
    if (w.req_gen != lsu_req_id_gen(resp.req_id)) {
      lsu_report_load_req_gen_mismatch(i, resp.req_id, entry_idx, lsu_req_id_gen(resp.req_id), w.req_gen, w.idx,
                                       cur.wait_dcache_ldq.head, cur.wait_dcache_ldq.count);
      LSU_NON_BSD_ASSERT(0 && "LSU load response req_gen mismatch");
      continue;
    }
    LdqEntry &l = nxt.ldq[w.idx];
    if (l.load_state != LoadState::WaitDcacheResp) continue;
    l.replay_type = resp.replay;
    if (resp.replay == ReplayType::HIT) {
      l.load_state = LoadState::ReadyToWb;
    } else {
      l.load_state = LoadState::ReadyToIssue;
      l.cache_miss = true;
      nxt.wait_dcache_ldq.e[entry_idx].replay_wait_cycles = replay_delay_cycles(resp.replay);
    }
  }
  wake_load_replays_on_mshr_fill(wires->in.dcache2lsu, nxt);

  const uint32_t n = nxt.wait_dcache_ldq.window(LSU_LOAD_WINDOW_WIDTH);
  for (uint32_t i = 0; i < n; i++) {
    QWait &w = nxt.wait_dcache_ldq.e[nxt.wait_dcache_ldq.at(i)];
    if (w.valid && nxt.ldq[w.idx].load_state == LoadState::ReadyToWb && nxt.finish.count < LDQ_SIZE) {
      nxt.finish.push(w.idx);
      w.valid = false;
    }
  }
  nxt.wait_dcache_ldq.trim(n);
}

void RealLsu::comb_dcache2lsu_stq() {
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    const auto &resp = wires->in.dcache2lsu.resp_ports.store_resps[i];
    if (!resp.valid) continue;
    if (resp.req_id >= STQ_SIZE) {
      LSU_NON_BSD_ASSERT(0 && "Store response idx out of bound");
      continue;
    }
    StqEntry &s = nxt.stq[resp.req_id];
    if (s.store_state != StoreState::WaitDcacheResp) continue;
    s.replay_type = resp.replay;
    if (resp.replay == ReplayType::HIT) {
      s.store_state = StoreState::Done;
      s.replay_wait_cycles = 0;
    } else {
      s.store_state = StoreState::Committed;  // wait for the fill, then replay
      s.replay_wait_cycles = replay_delay_cycles(resp.replay);
    }
  }
  wake_store_replays_on_mshr_fill(wires->in.dcache2lsu, nxt);
}

void RealLsu::comb_lsu2dcache_stq() {
  auto &ports = wires->out.lsu2dcache.req_ports.store_ports;
  for (int i = 0; i < LSU_STA_COUNT; i++) ports[i].valid = ports[i].replay = false;

  const uint32_t n = min<uint32_t>(nxt.stq_commit_count, LSU_STORE_WINDOW_WIDTH);
  int issued = 0;
  for (uint32_t i = 0; i < n && issued < LSU_STA_COUNT; i++) {
    const uint32_t stq_idx = (nxt.stq_head + i) % STQ_SIZE;
    StqEntry &s = nxt.stq[stq_idx];
    if (!(s.store_state == StoreState::Committed && cur.stq[stq_idx].store_state != StoreState::WaitDcacheResp)) continue;
    if (s.replay_wait_cycles > 0 && --s.replay_wait_cycles > 0) continue;

    const uint8_t strb = get_store_strb(s.paddr, s.func3);
    bool blocked = false;  // an older store to an overlapping address has not finished
    for (uint32_t j = 0; j < i && !blocked; j++) {
      const StqEntry &o = nxt.stq[(nxt.stq_head + j) % STQ_SIZE];
      blocked = CheckAddr(o.paddr, get_store_strb(o.paddr, o.func3), s.paddr, strb) && o.store_state != StoreState::Done;
    }
    if (blocked) continue;

    if (s.is_mmio) {
      if (i == 0 && !nxt.uncached_unit.valid) {
        UncachedUnit &u = nxt.uncached_unit;
        u.valid = true;
        u.is_load = false;
        u.addr = s.paddr;
        u.wdata = s.data;
        u.func3 = s.func3;
        u.idx = stq_idx;
        s.store_state = StoreState::WaitMmioResp;
      }
    } else if (s.suppress_write) {
      s.store_state = StoreState::Done;
      s.replay_wait_cycles = 0;
    } else {
      auto &p = ports[issued++];
      p.valid = true;
      p.addr = s.paddr;
      p.strb = strb;
      p.req_id = stq_idx;
      p.replay = s.replay_type != ReplayType::HIT;
      s.store_state = StoreState::WaitDcacheResp;
    }
  }
}

void RealLsu::comb_lsu2exe() {
  wires->lsu2exe = {};
  uint32_t issue_st = 0, issue_ld = 0;

  const uint32_t nd = nxt.mmu_done_stq.window(LSU_STORE_WINDOW_WIDTH);
  for (uint32_t i = 0; i < nd; i++) {
    QIdx &d = nxt.mmu_done_stq.e[nxt.mmu_done_stq.at(i)];
    if (!d.valid) continue;
    StqEntry &s = nxt.stq[d.idx];
    if (s.store_state != StoreState::Ready && s.store_state != StoreState::PageFault) continue;

    MicroOp wb;
    wb.rob_idx = s.rob_idx;
    wb.rob_flag = s.rob_flag;
    wb.br_mask = s.br_mask;
    wb.diag_val = s.vaddr;
    wb.page_fault_store = s.page_fault;
    wb.dest_en = false;
    wb.is_mmio = s.is_mmio;
    if (s.is_lrsc) {  // SC reports its pass/fail through the load writeback ports
      if (issue_ld >= LSU_LDU_COUNT) continue;
      wb.op = UOP_LOAD;
      if (s.page_fault) {
        wb.result = s.vaddr;
      } else {
        const bool pass = cur.lrsc_unit.reserve_valid && cur.lrsc_unit.reserve_addr == s.paddr;
        s.sc_pass = pass;
        s.suppress_write = !pass;
        nxt.lrsc_unit.reserve_valid = false;
        wb.result = pass ? 0 : 1;
        wb.dest_preg = s.dest_preg;
        wb.dest_en = s.dest_preg != 0;
      }
      wires->lsu2exe.wb_req[issue_ld].valid = true;
      wires->lsu2exe.wb_req[issue_ld++].uop = (wb);
    } else {
      if (issue_st >= LSU_STA_COUNT) continue;
      wb.op = UOP_STA;
      wb.result = s.vaddr;
      wires->lsu2exe.sta_wb_req[issue_st].valid = true;
      wires->lsu2exe.sta_wb_req[issue_st++].uop = (wb);
    }
    s.store_state = StoreState::WaitCommit;
    d.valid = false;
  }
  nxt.mmu_done_stq.trim(nxt.mmu_done_stq.window(LSU_STORE_WINDOW_WIDTH));

  const uint32_t nf = nxt.finish.window(LSU_LOAD_WINDOW_WIDTH);
  for (uint32_t i = 0; i < nf && issue_ld < LSU_LDU_COUNT; i++) {
    QIdx &f = nxt.finish.e[nxt.finish.at(i)];
    if (!f.valid) continue;
    LdqEntry &l = nxt.ldq[f.idx];
    if (l.load_state != LoadState::ReadyToWb && l.load_state != LoadState::PageFault) continue;

    MicroOp wb;
    wb.op = UOP_LOAD;
    wb.rob_idx = l.rob_idx;
    wb.rob_flag = l.rob_flag;
    wb.br_mask = l.br_mask;
    wb.diag_val = l.v_addr;
    wb.result = l.page_fault ? l.diag_val : l.result;
    if (tbhook::poison && !l.page_fault && !l.is_mmio) wb.result ^= 0xA5C3A5C3u;
    wb.dest_preg = l.dest_preg;
    wb.page_fault_load = l.page_fault;
    wb.dest_en = l.dest_preg != 0;  // loads to x0 still complete without a destination
    wb.is_mmio = l.is_mmio;
    wires->lsu2exe.wb_req[issue_ld].valid = true;
    wires->lsu2exe.wb_req[issue_ld++].uop = (wb);

    l.load_state = LoadState::Empty;
    f.valid = false;
    if (l.is_lrsc && !l.page_fault) {
      LrScUnit &r = nxt.lrsc_unit;
      r.reserve_valid = true;
      r.reserve_addr = l.p_addr;
      r.reserve_rob_idx = l.rob_idx;
      r.reserve_rob_flag = l.rob_flag;
      r.reserve_br_mask = l.br_mask;
    }
  }
  nxt.finish.trim(nf);
}

void RealLsu::comb_stq_commit() {
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!wires->rob_commit.commit_entry[i].valid) continue;
    const auto &u = wires->rob_commit.commit_entry[i].uop;
    if (!is_store(u)) continue;
    const int idx = u.stq_idx;
    if (idx != nxt.stq_commit || nxt.stq[idx].store_state != StoreState::WaitCommit) {
      LSU_NON_BSD_ASSERT(0 && "Store commit out of order?");
      continue;
    }
    StqEntry &s = nxt.stq[idx];
    if (static_cast<bool>(wires->rob_bcast.flush) || (s.is_lrsc && !s.sc_pass)) {
      s.suppress_write = true;  // flushed or failed SC: retire without writing
      s.store_state = StoreState::Done;
    } else {
      s.store_state = StoreState::Committed;
    }
    advance_ring_ptr(nxt.stq_commit, STQ_SIZE);
    if (nxt.stq_commit_count < STQ_SIZE) nxt.stq_commit_count++;
  }
}

void RealLsu::comb_check() {  // retire finished stores from the STQ head
  const uint32_t n = min<uint32_t>(min<uint32_t>(nxt.stq_commit_count, nxt.stq_count), LSU_STORE_WINDOW_WIDTH);
  for (uint32_t i = 0; i < n && nxt.stq[nxt.stq_head].store_state == StoreState::Done; i++) {
    nxt.stq_count--;
    if (nxt.stq_commit_count > 0) nxt.stq_commit_count--;
    advance_ring_ptr(nxt.stq_head, nxt.stq_head_flag, STQ_SIZE);
  }
}

void RealLsu::comb_flush() {
  if (wires->rob_bcast.flush) {
    for (LdqEntry &l : nxt.ldq) l.load_state = LoadState::Empty;
    const uint32_t keep = min<uint32_t>(nxt.stq_commit_count, nxt.stq_count);
    nxt.stq_count = nxt.stq_commit_count = keep;
    nxt.stq_commit = stq_idx_after(nxt.stq_head, keep);
    nxt.stlf_queue.clear();
    nxt.wait_dcache_ldq.clear();
    nxt.wait_mmu_ldq.clear();
    nxt.wait_mmu_stq.clear();
    nxt.mmu_done_stq.clear();
    nxt.finish.clear();
    // a committed MMIO store already on the bus survives the flush
    const bool keep_store = nxt.uncached_unit.valid && !nxt.uncached_unit.is_load &&
                            stq_idx_alive_after_flush(nxt.uncached_unit.idx, nxt.stq_head, keep) &&
                            nxt.stq[nxt.uncached_unit.idx].store_state == StoreState::WaitMmioResp;
    if (!keep_store) nxt.uncached_unit.valid = false;
  }

  const DecBroadcastIO &dec = wires->dec_bcast;
  if (dec.mispred) {
    for (LdqEntry &l : nxt.ldq)
      if (l.load_state != LoadState::Empty && (l.br_mask & dec.br_mask) != 0) l.load_state = LoadState::Empty;

    uint32_t keep = nxt.stq_count;
    for (uint32_t i = 0; i < nxt.stq_count; i++)
      if ((nxt.stq[(nxt.stq_head + i) % STQ_SIZE].br_mask & dec.br_mask) != 0) { keep = i; break; }
    nxt.stq_count = keep;

    auto drop_dead_ldq = [&](auto &q) {
      for (uint32_t i = 0; i < q.count; i++) {
        auto &e = q.e[q.at(i)];
        if (nxt.ldq[e.idx].load_state == LoadState::Empty) e.valid = 0;
      }
    };
    auto drop_dead_stq = [&](auto &q) {
      for (uint32_t i = 0; i < q.count; i++) {
        auto &e = q.e[q.at(i)];
        if (!stq_idx_alive_after_flush(e.idx, nxt.stq_head, nxt.stq_count)) e.valid = 0;
      }
    };
    drop_dead_ldq(nxt.wait_mmu_ldq);
    drop_dead_stq(nxt.wait_mmu_stq);
    drop_dead_stq(nxt.mmu_done_stq);
    drop_dead_ldq(nxt.stlf_queue);
    drop_dead_ldq(nxt.finish);
    drop_dead_ldq(nxt.wait_dcache_ldq);

    UncachedUnit &u = nxt.uncached_unit;
    if (u.valid && (u.is_load ? nxt.ldq[u.idx].load_state == LoadState::Empty
                              : !stq_idx_alive_after_flush(u.idx, nxt.stq_head, nxt.stq_count)))
      u.valid = false;
    if (nxt.lrsc_unit.reserve_valid && (nxt.lrsc_unit.reserve_br_mask & dec.br_mask) != 0)
      nxt.lrsc_unit.reserve_valid = false;

    nxt.stq_commit_count = min<uint32_t>(nxt.stq_commit_count, nxt.stq_count);
    nxt.stq_commit = stq_idx_after(nxt.stq_head, nxt.stq_commit_count);
  }

  if (dec.clear_mask) {  // a branch resolved correctly: its mask bit is free again
    for (int i = 0; i < LDQ_SIZE; i++)
      if (cur.ldq[i].load_state != LoadState::Empty) nxt.ldq[i].br_mask &= ~dec.clear_mask;
    for (int i = 0; i < cur.stq_count; i++) nxt.stq[(cur.stq_head + i) % STQ_SIZE].br_mask &= ~dec.clear_mask;
    if (nxt.lrsc_unit.reserve_valid) nxt.lrsc_unit.reserve_br_mask &= ~dec.clear_mask;
  }
}

StqEntry RealLsu::get_stq_entry(int idx, bool flag) {
  if (idx < 0 || idx >= STQ_SIZE) return {};
  const StqEntry &e = cur.stq[idx];
  if (e.stq_flag != flag || (idx + STQ_SIZE - cur.stq_head) % STQ_SIZE >= cur.stq_count) {
    LSU_NON_BSD_ASSERT(0 && "STQ entry flag mismatch");
    return {};
  }
  return e;
}

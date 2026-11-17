#include "BackPorts.h"
#include "RealLsu.h"
#include "config.h"
#include <algorithm>
#include <cstdlib>

// The LSU is the staged pipeline in RealLsuPipe.cpp; this file holds its setup and the commit-time store view.
void RealLsu::init() {
  cur_ = {};
  nxt = {};
  pcur_ = {};
  pnxt = {};
  pw = {};
}

// Commit-time view of a store for the peripheral model (state of the registered STQ).
// The stores of this cycle's commit that reach a device: a committed store (not page-faulting) whose STQ entry has its
// address and data and is not suppressed. The STQ is the registered one (pcur_), so the slot shows what the commit sees.
void RealLsu::comb_commit_store() {
  const bool interrupt = in.rob_bcast->interrupt;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    CommitStore &cs = out.commit_store->slot[i];
    cs = {};
    if (!in.rob_commit->commit_entry[i].valid || interrupt) continue;
    const InstEntry ce = in.rob_commit->commit_entry[i].uop.to_inst_entry(true);
    const InstInfo &inst = ce.uop;
    if (!inst->tma.mem_commit_is_store || inst.page_fault_store) continue;
    const StqEntry e = get_stq_entry(inst->stq_idx, inst->stq_flag);
    const bool suppressed = static_cast<bool>(e.suppress_write) && e.rob_idx == inst->rob_idx && e.rob_flag == inst->rob_flag;
    if (suppressed || !e.paddr_valid || !e.data_valid) continue;
    cs.valid = true;
    cs.paddr = e.paddr;
    cs.data = e.data;
    cs.func3 = e.func3;
  }
}

StqEntry RealLsu::get_stq_entry(int idx, bool flag) {
  const lp::LsuPipeState &pcur = pcur_;
  if (idx < 0 || idx >= STQ_SIZE) return {};
  const uint32_t rank = (idx + STQ_SIZE - pcur.stq_head) % STQ_SIZE;
  if (rank >= pcur.stq_count) return {};
  const bool eflag = pcur.stq_head_flag ^ ((pcur.stq_head + rank) >= STQ_SIZE);
  if (eflag != flag) return {};
  const lp::PStq &p = pcur.stq[idx];
  StqEntry e;
  e.data_valid = p.data_v;
  e.data = p.data;
  e.func3 = p.func3;
  e.paddr_valid = p.pa_v;
  e.paddr = p.paddr;
  e.suppress_write = p.suppress;
  e.rob_idx = p.rob_idx;
  e.rob_flag = p.rob_flag;
  e.stq_flag = eflag;
  return e;
}

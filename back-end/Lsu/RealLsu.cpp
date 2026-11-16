#include "Wires.h"
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

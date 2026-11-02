// Check-mode taps for the trace bridge (enabled with TB_CHECK=1; no-ops otherwise).
#pragma once
#include <cstdint>

namespace tbhook {
extern bool enabled;
void on_dispatch(uint64_t uid, uint32_t pc, uint32_t inst, uint32_t rob_idx);
void on_mispredict(uint64_t branch_uid);          // keep entries with uid <= branch_uid
void on_commit(uint64_t uid);
void on_flush();                                   // global pipeline flush: resync at next dispatch
// Execution results as seen on the exu->rob interface.
struct ExecObs {
  uint32_t rob_idx;          // the LSU path loses dbg.inst_idx: identity is rob_idx -> uid
  uint64_t uid_dbg;          // dbg.inst_idx as carried (0 on the LSU path)
  int op;                    // UopType
  uint32_t result, diag_val;
  bool br_taken, mispred, page_fault;
};
void on_exec(const ExecObs &o);
} // namespace tbhook

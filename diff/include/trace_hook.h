// Check-mode taps for the trace bridge (enabled with TB_CHECK=1; no-ops otherwise).
#pragma once
#include <cstdint>

#include "TraceBridge.h"
namespace tbhook {
extern bool enabled;
extern bool poison;   // TB_POISON=1: corrupt model-computed values that the timing must not read
extern bool inject;   // TB_INJECT=1: facts drive branch/address/store-data/csr-write decisions
// Fact of the in-flight instruction in this ROB slot, nullptr if unknown (killed) or inject is off.
const tb::Fact *fact_by_rob(uint32_t rob_idx);
extern uint64_t inject_miss;
// Bookkeeping of overrides: kind 0 branch dir, 1 jump target, 2 agu addr, 3 store data, 4 csr wdata.
void note_override(int kind, uint32_t computed, uint32_t injected, uint32_t rob_idx);
void on_dispatch(uint64_t uid, uint32_t pc, uint32_t inst, uint32_t rob_idx);
void on_mispredict(uint64_t branch_uid);          // keep entries with uid <= branch_uid
void on_commit(uint64_t uid);
// Global pipeline flush caused by committing instruction `uid` (-1: none valid), or by an interrupt
// (not committed; on_interrupt follows with the cause).
void on_flush(int64_t uid, bool interrupt);
void on_interrupt(uint32_t cause, uint8_t target_priv, uint32_t pending);
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

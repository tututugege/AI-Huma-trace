// Check-mode taps for the trace bridge (enabled with TB_CHECK=1; no-ops otherwise).
#pragma once
#include "xlen_cast.h"
#include <cstdint>

#include "TraceBridge.h"
namespace tbhook {
extern bool enabled;
extern bool poison;   // TB_POISON=1: corrupt model-computed values that the timing must not read
extern bool inject;   // TB_INJECT=1: facts drive branch/address/store-data/csr-write decisions
// Fact of the in-flight instruction in this ROB slot, nullptr if unknown (killed) or inject is off.
const tb::Fact *fact_by_rob(uint32_t rob_idx);
// F/D: does the instruction with this bridge uid turn mstatus.FS to Dirty when it commits? (call before on_commit)
bool fs_dirty_at_commit(uint64_t uid);
extern uint64_t inject_miss;
// Functional memory read (instruction words, page-table entries): the core's memory in trace mode.
uint32_t mem_read(uint32_t paddr);
// Bookkeeping of overrides: kind 0 branch dir, 1 jump target, 2 agu addr, 3 store data, 4 csr wdata.
void note_override(int kind, uint32_t computed, uint32_t injected, uint32_t rob_idx);
void on_dispatch(uint64_t uid, xlen_t pc, uint32_t inst, uint32_t rob_idx);
void on_mispredict(uint64_t branch_uid);          // keep entries with uid <= branch_uid
void on_commit(uint64_t uid);
// Global pipeline flush caused by committing instruction `uid` (-1: none valid), or by an interrupt
// (not committed; on_interrupt follows with the cause).
void on_flush(int64_t uid, bool interrupt);
void on_interrupt(xlen_t cause, uint8_t target_priv, uint32_t pending);
// Execution results as seen on the exu->rob interface.
struct ExecObs {
  uint32_t rob_idx;          // the LSU path loses dbg.inst_idx: identity is rob_idx -> uid
  uint64_t uid_dbg;          // dbg.inst_idx as carried (0 on the LSU path)
  int op;                    // UopType
  xlen_t result, diag_val;
  bool br_taken, mispred, page_fault;
};
void on_exec(const ExecObs &o);
} // namespace tbhook

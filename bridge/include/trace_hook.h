// Taps of the timing model into the trace bridge: dispatch / commit / flush / mispredict / interrupt drive the bridge, the facts
// come back through fact_by_rob; on_exec patches the values only the timing model knows (MMIO loads, CSR reads).
#pragma once
#include "xlen_cast.h"
#include <cstdint>

#include "TraceBridge.h"
namespace tbhook {
// Fact of the in-flight instruction in this ROB slot, nullptr if unknown (killed).
const tb::Fact *fact_by_rob(uint32_t rob_idx);
// F/D: does the instruction with this bridge uid turn mstatus.FS to Dirty when it commits? (call before on_commit)
bool fs_dirty_at_commit(uint64_t uid);
// Functional memory read (instruction words, page-table entries): the core's memory in trace mode.
uint32_t mem_read(uint32_t paddr);
xlen_t arch_gpr(int idx);   // the function model's integer register (the program's exit code: a0 at EBREAK)
// the 32-bit window that starts at a 2-byte aligned address (RVC=1 fetch); the same as mem_read for a 4-byte aligned address
inline uint32_t fetch_window_at(uint32_t paddr) {
  return (paddr & 2u) ? ((mem_read(paddr & ~3u) >> 16) | (mem_read((paddr & ~3u) + 4u) << 16)) : mem_read(paddr);
}
void on_dispatch(uint64_t uid, xlen_t pc, uint32_t inst, uint32_t rob_idx, uint32_t len = 4);
void on_mispredict(uint64_t branch_uid);          // keep entries with uid <= branch_uid
void on_commit(uint64_t uid);
// Global pipeline flush caused by committing instruction `uid` (-1: none valid), or by an interrupt
// (not committed; on_interrupt follows with the cause).
void on_flush(int64_t uid, bool interrupt);
void on_interrupt(xlen_t cause, uint8_t target_priv, uint32_t pending);
// Execution results of loads and CSR reads as seen on the exu->rob interface.
struct ExecObs {
  uint32_t rob_idx;
  int op;                    // UopType
  xlen_t result;
  bool page_fault;
};
void on_exec(const ExecObs &o);
} // namespace tbhook

// Trace bridge: a functional core (librefcpu) that follows the timing model's
// dispatch stream, wrong paths included, and supplies per-instruction facts.
//   dispatch(uid, pc, inst)  in program/fetch order (forced jump when pc != next pc)
//   rewind_after(uid)        mispredict/flush: drop every entry younger than uid
//   commit(uid)              oldest entry leaves the window
// Speculation is undone with per-entry state checkpoints and a store journal.
#pragma once
#include "xlen_cast.h"
#include <cstdint>
#include <deque>

struct RefCpuContext;
struct RefCpuState;

namespace tb {

enum class Blocked : uint8_t {
  None = 0,
  PcOutOfRam,   // cannot fetch
  Ebreak,       // would end the simulation
  MemMisalign,  // refcpu would exit(-1)
  MemOutOfRam,  // MMIO or illegal range: never executed speculatively
};

struct Fact {
  uint64_t uid = 0;
  xlen_t pc = 0, next_pc = 0;
  uint32_t inst = 0;
  bool forced_jump = false;   // pc differed from the core's next pc
  Blocked blocked = Blocked::None;
  bool trapped = false;       // the step took a trap (undone: state restored, pc+4)
  bool rd_valid = false;      // writes a non-x0 register
  uint8_t rd = 0;
  xlen_t rd_value = 0;
  bool is_br = false, br_taken = false;
  bool control_valid = false;  // br_taken/next_pc are meaningful (executed, or evaluated from registers)
  bool is_load = false, is_store = false;
  xlen_t vaddr = 0;
  uint8_t size = 0;            // 1, 2, 4 (rv64: 8)
  xlen_t store_data = 0;       // size-masked
  xlen_t rs2_value = 0;        // full rs2 (what the model carries as store data)
  xlen_t rs1_value = 0;        // full rs1 before the instruction (EXP-020: stale-value evaluation of the speculative window)
  uint32_t mem_old = 0;        // the word at mem_waddr before the store (valid with has_mem_write)
  bool has_mem_write = false;  // an executed store/AMO that changes RAM: word address and the word after it
  uint32_t mem_waddr = 0, mem_wdata = 0;
  xlen_t csr_wdata = 0;        // SYSTEM csrr*: rs1 value or zimm
};

class Bridge {
public:
  Bridge(RefCpuContext *ctx, uint32_t ram_base, uint32_t ram_bytes);
  const Fact &dispatch(uint64_t uid, xlen_t pc, uint32_t inst);
  void rewind_after(int64_t uid);   // keep entries with uid <= given (-1: drop all)
  void commit(uint64_t uid);        // must be the oldest entry
  // Instructions can also be fetched from the boot ROM region (word-backed MMIO in the core).
  void set_boot_region(uint32_t base, uint32_t bytes) { boot_base_ = base; boot_bytes_ = bytes; }
  size_t in_flight() const { return chain_.size(); }
  // A value the core cannot know (MMIO/timer read, mip/time CSR read) arrives from the model:
  // rewind everything younger than uid, write the destination register, replay the younger
  // entries (same uid/pc/inst) so their facts follow the patched value.
  bool patch_rd(uint64_t uid, xlen_t value);
  // Commit of an instruction that flushes the pipeline (trap, mret/sret, ecall, csr, fence...):
  // drop everything younger; if the instruction trapped when executed speculatively, the core takes
  // the state it computed then (handler entry, mret target, ...); then retire it.
  void commit_flush(uint64_t uid);
  // Interrupt taken at a commit boundary (before the oldest uncommitted instruction): drop every
  // in-flight entry, then the core takes the interrupt with the cause the model chose.
  bool take_interrupt(uint32_t cause, uint8_t target_priv, uint32_t pending_snapshot);
  uint64_t oldest_uid() const;
  void drop_oldest();
  // Entries older than `uid` that are still in flight were killed without a rewind (wrong path): undo their effects
  // (registers, memory, pc) and re-execute the entries from `uid` on on top of the restored state.
  size_t kill_older_than(uint64_t uid);  // returns the number of killed entries
  const Fact *find(uint64_t uid) const;
  // Functional word at a physical address as the core sees it now (RAM or boot ROM).
  bool read_word(uint32_t paddr, uint32_t &word) const;
  size_t undo_bytes() const;  // per-entry undo log size (fixed part)

  struct Stats {
    uint64_t dispatched = 0, forced_jumps = 0, rewinds = 0, entries_dropped = 0, patches = 0, replayed = 0, flush_commits = 0, trap_commits = 0, interrupts = 0;
    uint64_t blocked[5] = {0}, trapped = 0;
  } stats;

private:
  struct Entry;
  RefCpuContext *ctx_;
  uint32_t ram_base_, ram_bytes_;
  uint32_t boot_base_ = 0, boot_bytes_ = 0;
  std::deque<Entry *> chain_;
  bool in_ram(uint32_t addr, uint32_t size) const;
};

} // namespace tb

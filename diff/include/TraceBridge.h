// Trace bridge: a functional core (librefcpu) that follows the timing model's
// dispatch stream, wrong paths included, and supplies per-instruction facts.
//   dispatch(uid, pc, inst)  in program/fetch order (forced jump when pc != next pc)
//   rewind_after(uid)        mispredict/flush: drop every entry younger than uid
//   commit(uid)              oldest entry leaves the window
// Speculation is undone with per-entry state checkpoints and a store journal.
#pragma once
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
  uint32_t pc = 0, inst = 0, next_pc = 0;
  bool forced_jump = false;   // pc differed from the core's next pc
  Blocked blocked = Blocked::None;
  bool trapped = false;       // the step took a trap (undone: state restored, pc+4)
  bool rd_valid = false;      // writes a non-x0 register
  uint8_t rd = 0;
  uint32_t rd_value = 0;
  bool is_br = false, br_taken = false;
  bool is_load = false, is_store = false;
  uint32_t vaddr = 0;
  uint8_t size = 0;
  uint32_t store_data = 0;
};

class Bridge {
public:
  Bridge(RefCpuContext *ctx, uint32_t ram_base, uint32_t ram_bytes);
  const Fact &dispatch(uint64_t uid, uint32_t pc, uint32_t inst);
  void rewind_after(int64_t uid);   // keep entries with uid <= given (-1: drop all)
  void commit(uint64_t uid);        // must be the oldest entry
  size_t in_flight() const { return chain_.size(); }
  // A value the core cannot know (MMIO/timer read, mip/time CSR read) arrives from the model:
  // rewind everything younger than uid, write the destination register, replay the younger
  // entries (same uid/pc/inst) so their facts follow the patched value.
  bool patch_rd(uint64_t uid, uint32_t value);
  uint64_t oldest_uid() const;
  void drop_oldest();  // commit-less removal of a killed entry at the head
  const Fact *find(uint64_t uid) const;
  size_t undo_bytes() const;  // per-entry undo log size (fixed part)

  struct Stats {
    uint64_t dispatched = 0, forced_jumps = 0, rewinds = 0, entries_dropped = 0, patches = 0, replayed = 0;
    uint64_t blocked[5] = {0}, trapped = 0;
  } stats;

private:
  struct Entry;
  RefCpuContext *ctx_;
  uint32_t ram_base_, ram_bytes_;
  std::deque<Entry *> chain_;
  bool in_ram(uint32_t addr, uint32_t size) const;
};

} // namespace tb

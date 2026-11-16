#pragma once
#include "IO.h"
#include "config.h"
#include <cstdlib>

struct RobCommitExtra {
  wire<AREG_IDX_WIDTH> src1_areg;
  wire<32> imm;

  RobCommitExtra() { std::memset(this, 0, sizeof(RobCommitExtra)); }
};

struct RobOut {
  RobCommitExtra commit_extra[COMMIT_WIDTH];
};

// Compact ROB (EXP-028): entries are allocated one after another (rob_idx = ring position, stored at bank = rob_idx % 8,
// line = rob_idx / 8), commit takes the longest completed prefix from the head (max COMMIT_WIDTH, cut by the BPU commit
// credits), a mispredict pulls the tail back to the entry after the branch.

struct RobStoredEntry {
  wire<1> valid;
  Uop uop;
};

// Reorder buffer (compact, line-organized). Contract (checked by scripts/comb/comb_graph.py against the code): the Wires
// each phase reads (in) and writes (out), and the outputs only this module drives.
//   CONTRACT comb_commit  in: csr2rob dec_bcast in lsu2rob  out: rob2csr rob2dis rob_bcast rob_commit
//   CONTRACT comb_update  in: dec_bcast dis2rob exu2rob    out: -
//   CONTRACT owns: rob2csr rob2dis rob_bcast rob_commit
class Rob {
public:
  void comb_commit();  // the head: commit and its broadcasts
  void comb_update();  // completions, branch recovery, enqueue, flush
  Rob(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  void init();
  void seq();
  // PMU (EXP-013): the oldest incomplete instruction of the head line (the one that blocks commit), if any
  bool pmu_head_incomplete(Uop &out, int &occupied_entries) const;

  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  const class PreIduQueue *ftq = nullptr;  // FTQ PC read port (PreIduQueue::ftq_pc): the pc of the head
  RobOut out;

  // Registers: q (read in comb through const Regs &q, written only by seq) and d (their next value, written by comb;
  // d == q at the start of every cycle). seq copies d to q (the entries only where written).
  struct Regs {
    RobStoredEntry entry[ROB_BANK_NUM][ROB_LINE_NUM];
    // head entry, number of valid entries, wrap flag of the head
    int c_head = 0, c_count = 0; bool c_hflag = false;
    // control-flow instructions / fetch-block ends committed last cycle (still on their way to the BPU queues)
    int c_prev_ctl = 0, c_prev_blk = 0;
    int stall_cycle = 0;   // cycles without a commit (deadlock watchdog)
    RobStoredEntry &cent(int idx) { return entry[idx % ROB_BANK_NUM][idx / ROB_BANK_NUM]; }
    const RobStoredEntry &cent(int idx) const { return entry[idx % ROB_BANK_NUM][idx / ROB_BANK_NUM]; }
  };
  Regs d;
  const Regs &q() const { return q_; }
  // Slots of d.entry written this cycle; seq() copies only these (d == q at the start of a cycle).
  static constexpr int kDirtyWords = (ROB_BANK_NUM * ROB_LINE_NUM + 63) / 64;
  uint64_t dirty_[kDirtyWords] = {};
  bool dirty_all_ = false;
  void touch(int bank, int line) {
    const int k = bank * ROB_LINE_NUM + line;
    dirty_[k >> 6] |= 1ull << (k & 63);
  }
  void touch_all() { dirty_all_ = true; }

  void ctouch(int idx) { touch(idx % ROB_BANK_NUM, idx / ROB_BANK_NUM); }
  void single_actions(const Uop &u);  // what a single-retired instruction triggers (flush, trap, CSR, fence, ...)

private:
  // phases of one cycle, run in the order of the methods above
  void ready();
  void commit();
  void complete();
  void branch();
  void fire();
  void flush();
  void begin_cycle();   // called by ready(), the first phase of the cycle
  FtqPc head_pc_{};   // the FTQ read of the head (comb_commit)
  Regs q_;
  bool is_empty() const { return q_.c_count == 0; }
  bool is_full() const { return q_.c_count == ROB_NUM; }
};

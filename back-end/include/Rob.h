#pragma once
#include "BackPorts.h"
#include "IO.h"
#include "config.h"
#include <cstdlib>


// Compact ROB (EXP-028): entries are allocated one after another (rob_idx = ring position, stored at bank = rob_idx % 8,
// line = rob_idx / 8), commit takes the longest completed prefix from the head (max COMMIT_WIDTH, cut by the BPU commit
// credits), a mispredict pulls the tail back to the entry after the branch.

struct RobStoredEntry {
  wire<1> valid;
  Uop uop;
};

struct BackIn;

// Reorder buffer (compact, line-organized). Contract (checked by scripts/comb/comb_graph.py against the code): the signals
// each phase reads (in) and writes (out), and the outputs only this module drives.
//   CONTRACT comb_commit  in: in.dec_bcast in.lsu2rob in.top_in.front_stall in.top_in.front_tq_free in.top_in.front_upd_free  out: out.rob2csr out.rob2dis out.rob_bcast out.rob_commit
//   CONTRACT comb_redirect  in: in.dec_bcast in.ftq_commit_info in.idu_br_latch in.pre2front out.top_out.flush out.top_out.redirect_pc out.top_out.rf_slot out.top_out.rf_valid  out: out.rob2csr out.top_out
class Rob {
public:
  void comb_commit();  // the head: commit and its broadcasts
  void comb_redirect();
  Rob(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  void init();
  void seq();
  // PMU (EXP-013): the oldest incomplete instruction of the head line (the one that blocks commit), if any
  bool pmu_head_incomplete(Uop &out, int &occupied_entries) const;

  RobBroadcastIO bc_prev_;   // the broadcast of the previous cycle (CFG_FLUSH_REG)
  struct PendFlush { bool v = false, fence_i = false, itlb = false, interrupt = false; uint64_t pc = 0; int64_t committing = -1; } pend_, pend_nxt_;   // the front-end redirect, delivered one cycle later
  // Outputs: the signals this module drives (it alone).
  struct Out {
    RobCsrIO rob2csr;
    RobDisIO rob2dis;
    RobBroadcastIO rob_bcast;
    RobBroadcastIO rob_bcast_d;   // what the other back-end modules read: CFG_FLUSH_REG = 1 delays the flush / trap fields by a cycle (the head fields stay)
    RobCommitIO rob_commit;
    BackOut *top_out = nullptr;    // the back end's outputs (BackTop's interface), bound by BackTop::init
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const DecBroadcastIO *dec_bcast = nullptr;
    const DisRobIO *dis2rob = nullptr;
    const ExuRobIO *exu2rob = nullptr;
    const BackIn *top_in = nullptr;   // the back end's inputs (SimCpu)
    const LsuRobIO *lsu2rob = nullptr;
    const PreFrontIO *pre2front = nullptr;
    const FtqCommitInfoIO *ftq_commit_info = nullptr;
    const ExuIdIO *idu_br_latch = nullptr;   // the IDU's branch latch (its register): the mispredict target
  } in;
  const class PreIduQueue *ftq = nullptr;

  // Registers (q): read by the comb phases, updated only by seq (the RTL's rob_entry always_ff: enqueue, write-back,
  // branch clear, commit clear, flush).
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
  const Regs &q() const { return q_; }
  void single_actions(const Uop &u);  // what a single-retired instruction triggers (flush, trap, CSR, fence, ...)

private:
  void ready();
  void commit();
  void redirect();
  void report_commit();
  int head_incomplete_ = -1;   // the oldest incomplete entry of the head line (comb_commit's scan; -1: none)
  // the clock edge (seq), in this order: write-back completion bits, mispredict recovery, enqueue, flush
  void complete();
  void branch(int head, int count, int &c_count);
  void fire(int &c_count);
  void flush(int &c_head, int &c_count, bool &c_hflag);
  FtqPc head_pc_{};   // the FTQ read of the head (comb_commit)
  struct CommitWires {   // what comb_commit decided, applied by seq
    int n = 0;                         // entries retiring from the head
    int prev_ctl = 0, prev_blk = 0;    // their control-flow instructions / fetch-block ends (the BPU credits)
    int stall_cycle = 0;               // cycles without a commit, this one included
  } cw_;
  Regs q_;
  bool is_empty() const { return q_.c_count == 0; }
  bool is_full() const { return q_.c_count == ROB_NUM; }
};

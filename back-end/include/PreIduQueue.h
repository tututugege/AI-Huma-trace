#pragma once

#include "FTQ.h"
#include "IO.h"
#include "InstructionBuffer.h"
#include "config.h"

class SimContext;

static_assert(FTQ_SIZE <= 64, "FTQ indices fit the 64-bit masks of the PMU / recovery");

struct BackIn;

// Pre-IDU queue (instruction buffer and FTQ). Contract (checked by scripts/comb/comb_graph.py against the code): the signals each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_accept_front  in: in.idu_br_latch in.top_in.inst in.top_in.is_rvc in.top_in.meta in.top_in.page_fault_inst in.top_in.pc in.top_in.pf_inst_hi in.top_in.predict_dir in.top_in.predict_next_fetch_address in.top_in.valid  out: out.pre2front out.pre_issue
//   CONTRACT comb_commit_info  in: in.rob_commit  out: out.ftq_commit_info
class PreIduQueue {
public:
  // Registers: read by the comb phases, updated only by seq (from the registers and this cycle's wires).
  struct Regs {
    bool flush_q  = false;
    InstructionBuffer ibuf;
    FTQEntry ftq_lookup_entries[FTQ_SIZE];
    FTQTrainMetaEntry ftq_train_meta_fifo[FTQ_SIZE];
    wire<1> ftq_valid[FTQ_SIZE] = {};
    reg<FTQ_IDX_WIDTH> ftq_head  = 0;
    reg<FTQ_IDX_WIDTH> ftq_tail  = 0;
    reg<bit_width_for_count(FTQ_SIZE + 1)> ftq_count  = 0;
  };
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  explicit PreIduQueue(SimContext * = nullptr) {}
  // Outputs: the signals this module drives (it alone).
  struct Out {
    PreFrontIO pre2front;
    PreIssueIO pre_issue;
    FtqCommitInfoIO ftq_commit_info;
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const ExuIdIO *idu_br_latch = nullptr;
    const IduConsumeIO *idu_consume = nullptr;
    const BackIn *top_in = nullptr;   // the back end's inputs (SimCpu)
    const RobBroadcastIO *rob_bcast = nullptr;
    const RobCommitIO *rob_commit = nullptr;
  } in;

  void init();
  void comb_accept_front();
  void comb_commit_info();
  void seq();

  // BPA_REPAIR: BPU block sequence number and slot pc of an FTQ entry (read by BackTop when a branch mispredict redirects the front end)
  uint16_t ftq_blk_id(uint32_t idx, uint32_t off = 0) const { return q_.ftq_train_meta_fifo[idx].meta.blk_id[off]; }
  uint64_t ftq_slot_pc(uint32_t idx, uint32_t off) const { return q_.ftq_lookup_entries[idx].slot_pc[off]; }
  bool ftq_slot_rvc(uint32_t idx, uint32_t off) const { return q_.ftq_lookup_entries[idx].slot_rvc[off]; }
  // FTQ PC read (the PRF's ALU / BRU ports, the ROB head): a combinational read of the registered FTQ array
  FtqPc ftq_pc(uint32_t idx, uint32_t off) const;
private:
  void begin_cycle();   // called by comb_accept_front(), the first phase of the cycle

  void ftq_pop(int pop_cnt);
  void ftq_recover(int new_tail);
  void ftq_flush();

  // wires of the cycle: the block accepted from the front end (comb_accept_front: its FTQ entry and instructions), written
  // into the FTQ and the buffer at seq; with an empty buffer the accepted block goes straight to decode (push_bypass)
  bool accept_ = false;
  FTQEntry accept_block_{};
  decltype(FTQTrainMetaEntry::meta) accept_meta_{};
  InstructionBufferEntry push_entries[BACKEND_FETCH_WIDTH] = {};
  int push_count = 0;
  bool push_bypass = false;


private:
  Regs q_;
};

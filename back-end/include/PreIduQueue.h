#pragma once

#include "FTQ.h"
#include "IO.h"
#include "InstructionBuffer.h"
#include "config.h"

class SimContext;

static_assert(FTQ_SIZE <= 64, "FTQ indices fit the 64-bit masks of the PMU / recovery");

struct BackIn;

// Pre-IDU queue (instruction buffer and Unified FTQ-64). Contract (checked by scripts/comb/comb_graph.py against the code): the signals each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_accept_front  in: in.idu_br_latch in.top_in.ftq_idx in.top_in.inst in.top_in.is_rvc in.top_in.page_fault_inst in.top_in.pc in.top_in.pf_inst_hi in.top_in.valid  out: out.pre2front out.pre_issue
//   CONTRACT comb_commit_info  in: in.rob_commit  out: out.ftq_commit_info
class PreIduQueue {
public:
  // Registers: read by the comb phases (including FrontCore read-only), updated only by seq.
  struct Regs {
    bool flush_q  = false;
    InstructionBuffer ibuf;
    FTQEntry ftq_lookup_entries[FTQ_SIZE];
    FTQTrainMetaEntry ftq_train_meta_fifo[FTQ_SIZE];
    wire<1> ftq_valid[FTQ_SIZE] = {};
    // 5 monotonic pointers on the single 64-entry Unified FTQ ring:
    //   comm_ptr <= pd_ptr <= {ifu_ptr, s2_ptr} <= bpu_ptr,  (bpu_ptr - comm_ptr) mod 64 <= 63
    reg<FTQ_IDX_WIDTH> comm_ptr = 0;   // oldest uncommitted FTQ entry (old ftq_head)
    reg<FTQ_IDX_WIDTH> pd_ptr   = 0;   // next entry for Predecode to check
    reg<FTQ_IDX_WIDTH> s2_ptr   = 0;   // next entry for BPU S2 to write
    reg<FTQ_IDX_WIDTH> ifu_ptr  = 0;   // next entry for ICache to issue
    reg<FTQ_IDX_WIDTH> bpu_ptr  = 0;   // next entry for BPU S0 to allocate
    bool fresh_v = false;              // EXP-044: 1-cycle age gate for ifu_ptr (entry pushed last cycle)
    reg<FTQ_IDX_WIDTH> fresh_slot = 0;
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
  void restart() { ftq_flush(); q_.ibuf.clear(); q_.flush_q = false; }
  void comb_accept_front();
  void comb_commit_info();
  void seq();

  // Read-only registered views for FrontCore (no combinational path from back_top into front_top)
  uint8_t total_count() const { return static_cast<uint8_t>((q_.bpu_ptr - q_.comm_ptr) & (FTQ_SIZE - 1)); }
  uint8_t fa_count() const { return static_cast<uint8_t>((q_.bpu_ptr - q_.ifu_ptr) & (FTQ_SIZE - 1)); }
  bool ftq_full() const { return total_count() >= (FTQ_SIZE - 1) || fa_count() >= 15; }
  bool fa_empty() const { return q_.ifu_ptr == q_.bpu_ptr; }
  bool fa_head_fresh() const { return q_.fresh_v && !fa_empty() && (q_.fresh_slot == q_.ifu_ptr); }
  bool s2_empty() const { return q_.pd_ptr == q_.s2_ptr; }
  const FTQEntry &ifu_head() const { return q_.ftq_lookup_entries[q_.ifu_ptr]; }
  const FTQEntry &pd_head() const { return q_.ftq_lookup_entries[q_.pd_ptr]; }
  const PredMetaT<BACKEND_FETCH_WIDTH> &pd_meta() const { return q_.ftq_train_meta_fifo[q_.pd_ptr].meta; }
  uint32_t pd_idx() const { return q_.pd_ptr; }
  uint32_t ftq_head() const { return q_.comm_ptr; }

  // BPA_REPAIR: BPU block sequence number and slot pc of an FTQ entry (read by BackTop when a branch mispredict redirects the front end)
  uint16_t ftq_blk_id(uint32_t idx, uint32_t off = 0) const { return q_.ftq_train_meta_fifo[idx].meta.blk_id[off]; }
  uint64_t ftq_slot_pc(uint32_t idx, uint32_t off) const { return q_.ftq_lookup_entries[idx].slot_pc[off]; }
  bool ftq_slot_rvc(uint32_t idx, uint32_t off) const { return q_.ftq_lookup_entries[idx].slot_rvc[off]; }
  // FTQ PC read (the PRF's ALU / BRU ports, the ROB head): a combinational read of the registered FTQ array
  FtqPc ftq_pc(uint32_t idx, uint32_t off) const;
private:
  void begin_cycle();   // called by comb_accept_front(), the first phase of the cycle

  void ftq_recover(int new_tail);
  void ftq_flush();

  // wires of the cycle: instructions accepted into ibuf this cycle (FTQ entry already lives at top_in->ftq_idx)
  InstructionBufferEntry push_entries[BACKEND_FETCH_WIDTH] = {};
  int push_count = 0;
  bool push_bypass = false;

private:
  Regs q_;
};

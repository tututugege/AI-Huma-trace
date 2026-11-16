#pragma once

#include "FTQ.h"
#include "IO.h"
#include "InstructionBuffer.h"
#include "config.h"

class SimContext;

static_assert(FTQ_SIZE <= 64, "ftq_alloc_mask is a 64-bit slot mask");

// Pre-IDU queue (instruction buffer and FTQ). Contract (checked by scripts/comb/comb_graph.py against the code): the Wires each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_accept_front  in: idu_br_latch in                                out: pre2front pre_issue
//   CONTRACT comb_commit_info   in: rob_commit                                     out: ftq_commit_info
//   CONTRACT comb_fire          in: idu_br_latch idu_consume rob_bcast rob_commit  out: -
//   CONTRACT owns: pre2front pre_issue ftq_commit_info
class PreIduQueue {
public:
  // Registers: q (read in comb through const Regs &q, written only by seq) and d (their next value, written by comb;
  // d == q at the start of every cycle). seq: q = d.
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
  Regs d;
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  explicit PreIduQueue(SimContext * = nullptr) {}
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

  void init();
  void comb_accept_front();
  void comb_fire();
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

  int ftq_alloc();
  void ftq_pop(int pop_cnt);
  void ftq_recover(int new_tail);
  void ftq_flush();

  // committed (x) == working (x_1) at comb_begin; seq copies only dirty parts.
  bool ibuf_dirty = false;
  bool ftq_dirty = false;
  uint64_t ftq_alloc_mask = 0; // slots allocated this cycle (FTQ_SIZE <= 64)


private:
  Regs q_;
};

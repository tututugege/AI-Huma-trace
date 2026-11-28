#pragma once
#include "BPU/BPU_pipe.h"
#include "fifo/front_fifo.h"
#include "icache/icache_top.h"
#include "train_IO.h"

class PreIduQueue;

// The instruction FIFO port of the front end (Unified FTQ-64 replaces fetch_addr_fifo, ptab, and front2back_fifo).
//   CONTRACT comb_read  in: in.fifo_read_enable in.global_refetch in.global_reset in.rd.fifo_rd_snapshot in.rd.q  out: out.carry_low out.fifo_head out.fifo_in out.fifo_rd out.fifo_req out.grp_in_range out.grp_ptab out.grp_seg_first out.multi out.rvc_n out.rvc_straddle
//   CONTRACT comb_write  in: in.global_refetch in.global_reset in.ic_out in.rd.q  out: out.f1_next_entry out.fifo_in out.fifo_rd out.fifo_req
class InstPort {
public:
  struct In {
    const bool *global_reset = nullptr, *global_refetch = nullptr, *fifo_read_enable = nullptr;
    const FrontReadData *rd = nullptr;
    const ICacheOut *ic_out = nullptr;
  } in;
  ICacheTop *icache = nullptr;           // FE_PAIR: the second block's words (fetch_second, the I-cache's arrays)
  const PreIduQueue *ftq = nullptr;      // read-only Unified FTQ-64 registers (pd_ptr prediction view)
  struct Out {
    InstFifoIn fifo_in{};
    InstFifoEntry f1_next_entry{};    // FE_ICACHE_PIPE=1: F0 assembled entry latched into FrontRegs::ic_f1_entry
    InstFifoReadData fifo_rd{};
    InstFifoOut fifo_head{};          // the group at the head as the checker sees it (fixed: FE_PAIR cut, RVC carry)
    InstructionCombOut fifo_req{};
    bool multi = false;               // FE_PAIR: the group at the head spans two blocks
    wire<1> grp_in_range[FETCH_WIDTH] = {};    // the group's slots
    wire<1> grp_seg_first[FETCH_WIDTH] = {};   // each block's first slot
    int rvc_n = 0;                    // RVC=1: slots of the group (before the head fix), its straddle and carried half
    bool rvc_straddle = false;
    uint16_t carry_low = 0;
    PtabOut grp_ptab{};               // Unified FTQ pd_ptr entry as the checker sees it (RVC=1: head fix moves its first pc)
  } out;
  FrontFifo<InstFifo> *fifo = nullptr;
  void comb_read();    // the head, fixed for the checker
  void comb_write();   // a completed I-cache read, predecoded
};

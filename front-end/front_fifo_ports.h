#pragma once
#include "BPU/BPU_pipe.h"
#include "fifo/front_fifo.h"
#include "icache/icache_top.h"
#include "train_IO.h"

// The four FIFOs of the front end as modules (the RTL's fetch_addr_fifo / inst_fifo / ptab / front2back_fifo). Each owns
// its port state of the cycle (in, read data, head), the request its storage applies at the edge (FrontCore::seq), and
// its read / write phases; the storage is FrontCore's FIFO object (fifo). Port names are the front end's wire names.
// Each module's first phase of the cycle starts its port state anew (in cleared, read data from the registers, the
// request's enables cleared).

//   CONTRACT comb_read   in: in.global_reset in.global_refetch in.pipe_dec in.fetch_addr_read_enable in.rd  out: out.fetch_addr_in out.fetch_addr_rd out.fetch_addr_head out.fetch_addr_fifo_req out.ipf
//   CONTRACT comb_write  in: in.bpu_output in.bpu_can_run in.global_reset  out: out.fetch_addr_in out.fetch_addr_rd out.fetch_addr_fifo_req out.ipf
class FetchAddrPort {
public:
  struct In {
    const bool *global_reset = nullptr, *global_refetch = nullptr, *fetch_addr_read_enable = nullptr, *bpu_can_run = nullptr;
    const bpuf::Decision *pipe_dec = nullptr;
    const FrontReadData *rd = nullptr;
    const bpuf::OutputPayload *bpu_output = nullptr;
  } in;
  struct Out {
    FetchAddrFifoIn fetch_addr_in{};
    FetchAddrFifoReadData fetch_addr_rd{};
    FetchAddrFifoOut fetch_addr_head{};      // the head popped this cycle (the I-cache takes it)
    FetchAddrCombOut fetch_addr_fifo_req{};  // applied at the edge
    PendingIpf ipf{};                        // FE_IPF: the fetch addresses pushed this cycle (prefetch candidates)
  } out;
  FetchAddrFifo *fifo = nullptr;
  void comb_read();
  void comb_write();   // the predicted fetch address enters the FIFO (the I-cache reads it next cycle)
};

//   CONTRACT comb_read   in: in.global_reset in.global_refetch in.fifo_read_enable in.ptab_head in.rd  out: out.fifo_in out.fifo_rd out.fifo_head out.fifo_req out.multi out.grp_in_range out.grp_seg_first out.rvc_n out.rvc_straddle out.carry_low out.grp_ptab
//   CONTRACT comb_write  in: in.global_reset in.global_refetch in.ic_out in.fetch_addr_head in.rd  out: out.fifo_in out.fifo_rd out.fifo_req
class InstPort {
public:
  struct In {
    const bool *global_reset = nullptr, *global_refetch = nullptr, *fifo_read_enable = nullptr;
    const PtabOut *ptab_head = nullptr;
    const FrontReadData *rd = nullptr;
    const ICacheOut *ic_out = nullptr;
    const FetchAddrFifoOut *fetch_addr_head = nullptr;
    ICacheTop *icache = nullptr;   // FE_PAIR: the second block's words (fetch_second)
  } in;
  struct Out {
    InstFifoIn fifo_in{};
    InstFifoReadData fifo_rd{};
    InstFifoOut fifo_head{};          // the group at the head as the checker sees it (fixed: FE_PAIR cut, RVC carry)
    InstructionCombOut fifo_req{};
    bool multi = false;               // FE_PAIR: the group at the head spans two blocks
    wire<1> grp_in_range[FETCH_WIDTH] = {}, grp_seg_first[FETCH_WIDTH] = {};   // the group's slots, each block's first
    int rvc_n = 0;                    // RVC=1: slots of the group (before the head fix), its straddle and carried half
    bool rvc_straddle = false;
    uint16_t carry_low = 0;
    PtabOut grp_ptab{};               // the PTAB head as the checker sees it (RVC=1: the head fix moves its first pc)
  } out;
  FrontFifo<InstFifo> *fifo = nullptr;
  void comb_read();    // the head, fixed for the checker
  void comb_write();   // a completed I-cache read, predecoded
private:
  void fix_head();
};

//   CONTRACT comb_read  in: in.global_reset in.global_refetch in.fifo_read_enable in.ptab_bypass in.ptab_can_write in.bpu_output in.rd  out: out.ptab_in out.ptab_rd out.ptab_head out.ptab_req
class PtabPort {
public:
  struct In {
    const bool *global_reset = nullptr, *global_refetch = nullptr, *fifo_read_enable = nullptr;
    const bool *ptab_bypass = nullptr, *ptab_can_write = nullptr;
    const bpuf::OutputPayload *bpu_output = nullptr;
    const FrontReadData *rd = nullptr;
  } in;
  struct Out {
    PtabIn ptab_in{};
    PtabReadData ptab_rd{};
    PtabOut ptab_head{};
    PtabCombOut ptab_req{};
  } out;
  FrontFifo<Ptab> *fifo = nullptr;
  void comb_read();   // the BPU's block in (written before the read when it bypasses to the checker), the head out
};

//   CONTRACT comb_read   in: in.global_reset in.f2b_read_enable in.in in.rd  out: out.f2b_in out.f2b_rd out.f2b_head out.front2back_fifo_req out.f2b_bypass out.use_f2b_bypass
//   CONTRACT comb_write  in: in.global_reset in.f2b_read_enable in.fifo_read_enable in.fifo_head in.grp_ptab in.checker_out in.rd  out: out.f2b_in out.f2b_rd out.front2back_fifo_req out.f2b_bypass out.use_f2b_bypass
class F2bPort {
public:
  struct In {
    const bool *global_reset = nullptr, *f2b_read_enable = nullptr, *fifo_read_enable = nullptr;
    const FrontTopIn *in = nullptr;   // the redirect (a refetch clears the FIFO)
    const FrontReadData *rd = nullptr;
    const InstFifoOut *fifo_head = nullptr;
    const PtabOut *grp_ptab = nullptr;   // the group's PTAB entry (InstPort)
    const PredecodeCheckerOut *checker_out = nullptr;
  } in;
  struct Out {
    F2bFifoIn f2b_in{};
    F2bFifoReadData f2b_rd{};
    F2bFifoOut f2b_head{};
    Front2BackCombOut front2back_fifo_req{};
    F2bFifoOut f2b_bypass{};          // the checked entry straight to the output when the FIFO is empty
    bool use_f2b_bypass = false;
  } out;
  FrontFifo<F2bFifo> *fifo = nullptr;
  void comb_read();
  void comb_write();   // the checked entry, or the bypass
};

#pragma once
#include "BackPorts.h"
#include "front_IO.h"

// The register between the back end and the front end (the RTL's core_back2front.sv): what the back end decided in a
// cycle (redirect, fence flushes, the stall flag and the BPU training of the commit) is the front end's input in the
// next. comb computes it from back.out; comb_front drives the front end's inputs from q. Fields not refreshed keep their
// registered value.
struct Back2FrontRegs {
  FrontTopIn in{};      // the front end's inputs
  bool stall = false;   // the back end stalled (the front end does not deliver unless a refetch is pending)
};

// Comb phases in its own ports (SimCpu::init binds them):
//   CONTRACT comb_front  in: -  out: out.can_read out.front_in out.front_in.csr_status out.front_in.ctrl out.front_in.redirect
//   CONTRACT comb  in: in.back_out.commit_entry in.back_out.commit_extra in.back_out.commit_ftq in.back_out.fence_i in.back_out.flush in.back_out.itlb_flush in.back_out.mispred in.back_out.redirect_pc in.back_out.rf_blk in.back_out.rf_is_rvc in.back_out.rf_pc in.back_out.rf_slot in.back_out.rf_type in.back_out.rf_valid in.back_out.stall  out: -
class Back2Front {
public:
  struct In {   // bound by SimCpu::init
    const BackOut *back_out = nullptr;
  } in;
  struct Out {
    FrontTopIn *front_in = nullptr;   // the front end's inputs of this cycle (FrontTop::in)
    bool *can_read = nullptr;         // the back end takes a group this cycle (FrontTop::can_read)
  } out;
  CsrStatusIO *csr_status = nullptr;      // carried in the front end's inputs (back.csr_status)

  void init(CsrStatusIO *csr);
  // the register's output: the front end's inputs of this cycle; when the back end did not stall (or a refetch is
  // pending) the fetch-group FIFO is read (can_read: Front2Back then takes the group from the front end's output)
  void comb_front();
  void comb();   // the register's next value from the back end's outputs
  void seq() { q_ = d; }
  void restart_at(pc_t pc);   // between cycles (a switch from fast mode): a refetch at pc, no stall
  const Back2FrontRegs &q() const { return q_; }

private:
  Back2FrontRegs q_, d;
};

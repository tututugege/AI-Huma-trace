#pragma once
#include "BackPorts.h"
#include "front_IO.h"

// The register between the back end and the front end (the RTL's core_back2front.sv): what the back end decided in a
// cycle (redirect, fence flushes, the stall flag and the BPU training of the commit) is the front end's input in the
// next. comb computes it from back.out; the front end reads q (FrontTop::from_back). Fields not refreshed keep their
// registered value.
struct Back2FrontRegs {
  FrontTopIn in{};      // the front end's inputs
  bool stall = false;   // the back end stalled (the front end does not deliver unless a refetch is pending)
};

// SimCpu-level phase (ports in include/SimCpu.h, scripts/comb/gen_top_order.py):
//   EXPORT comb  in: b2f_reg back2front
class Back2Front {
public:
  const BackOut *back_out = nullptr;      // bound by SimCpu::init
  CsrStatusIO *csr_status = nullptr;      // carried in the front end's inputs (back.csr_status)

  void init(CsrStatusIO *csr);
  void comb();
  void seq() { q_ = d; }
  void restart_at(pc_t pc);   // between cycles (a switch from fast mode): a refetch at pc, no stall
  const Back2FrontRegs &q() const { return q_; }

private:
  Back2FrontRegs q_, d;
};

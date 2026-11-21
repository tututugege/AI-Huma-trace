#pragma once
#include "BackPorts.h"
#include "front_IO.h"

// Front end -> back end (the RTL's core_front2back.sv): the fetch group the back end gets this cycle. When the front
// end may deliver (can_read: the back end did not stall last cycle, or a redirect is pending) it is the front end's
// output cut after the first predicted-taken instruction (FE_PAIR: a second block may follow it); otherwise the group
// held from the cycle the back end stalled. The front end's commit credits go through every cycle. The hold register:
// when the back end stalls without a redirect, the group it saw minus the lanes it took.
// Comb phases in its own ports (SimCpu::init binds them):
//   CONTRACT comb  in: in.can_read in.front_out.FIFO_valid in.front_out.commit_stall in.front_out.commit_tq_free in.front_out.commit_upd_free in.front_out.inst_valid in.front_out.instructions in.front_out.is_rvc in.front_out.meta in.front_out.page_fault_inst in.front_out.pc in.front_out.pf_inst_hi in.front_out.predict_dir in.front_out.predict_next_fetch_address out.to_back.valid  out: out.to_back out.to_back.front_stall out.to_back.front_tq_free out.to_back.front_upd_free out.to_back.inst out.to_back.is_rvc out.to_back.meta out.to_back.page_fault_inst out.to_back.pc out.to_back.pf_inst_hi out.to_back.predict_dir out.to_back.predict_next_fetch_address out.to_back.valid
class Front2Back {
public:
  struct Regs {
    FrontPreIO hold{};          // the group held while the back end stalls
    bool hold_valid = false;
  };
  struct In {   // bound by SimCpu::init
    const FrontTopOut *front_out = nullptr;
    const bool *can_read = nullptr;    // FrontTop::can_read
    const BackOut *back_out = nullptr;
  } in;
  struct Out {
    FrontPreIO *to_back = nullptr;     // the fetch-group part of back.in
  } out;

  void init();
  void comb();        // the group to the back end
  void seq();         // after the back end: the hold register
  void restart();     // between cycles (a switch from fast mode): a held group is dropped

private:
  Regs q_;
};

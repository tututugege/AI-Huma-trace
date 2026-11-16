#pragma once
#include "BackPorts.h"
#include "front_IO.h"

// Front end -> back end (the RTL's core_front2back.sv): the fetch group the back end gets this cycle. When the front
// end may deliver (can_read: the back end did not stall last cycle, or a redirect is pending) it is the front end's
// output cut after the first predicted-taken instruction (FE_PAIR: a second block may follow it); otherwise the group
// held from the cycle the back end stalled. The front end's commit credits go through every cycle. The hold register:
// when the back end stalls without a redirect, the group it saw minus the lanes it took.
// SimCpu-level phases (ports in include/SimCpu.h, scripts/comb/gen_top_order.py):
//   EXPORT comb       in: front_out  out: fetch_group
//   EXPORT comb_hold  in: fetch_group back2front
class Front2Back {
public:
  struct Regs {
    FrontPreIO hold{};          // the group held while the back end stalls
    bool hold_valid = false;
  };
  // bound by SimCpu::init
  const FrontTopOut *front_out = nullptr;
  const bool *can_read = nullptr;    // FrontTop::can_read
  const BackOut *back_out = nullptr;
  FrontPreIO *to_back = nullptr;     // the fetch-group part of back.in

  void init();
  void comb();        // the group to the back end
  void comb_hold();   // after the back end: the hold register's next value
  void seq();
  void restart();     // between cycles (a switch from fast mode): a held group is dropped

private:
  Regs q_, d;
};

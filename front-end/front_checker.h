#pragma once
#include "BPU/BPU.h"
#include "front_IO.h"
#include "predecode_checker.h"

// The predecode check of the group at the instruction FIFO head against its PTAB entry (the RTL's predecode_checker):
// the corrected direction / next fetch address, a refetch when the prediction was wrong, the RVC=1 carry of a straddling
// last instruction, and the check's report to the BPU for the next cycle.
//   CONTRACT comb  in: in.carry_low in.fifo_head in.fifo_read_enable in.grp_in_range in.grp_ptab in.grp_seg_first in.multi in.rvc_n in.rvc_straddle  out: out.checker_out out.flush_address out.lh_new_data out.lh_new_pc out.lh_new_valid out.lh_updated out.pd_next out.predecode_flush
class FrontChecker {
public:
  struct In {   // (the field names are the front end's wire names)
    const bool *fifo_read_enable = nullptr;
    const InstFifoOut *fifo_head = nullptr;
    const PtabOut *grp_ptab = nullptr;     // the group's PTAB entry (InstPort: after the head fix)
    const bool *multi = nullptr;
    const wire<1> *grp_in_range = nullptr;    // [FETCH_WIDTH]
    const wire<1> *grp_seg_first = nullptr;   // [FETCH_WIDTH]
    const int *rvc_n = nullptr;
    const bool *rvc_straddle = nullptr;
    const uint16_t *carry_low = nullptr;
  } in;
  struct Out {
    PredecodeCheckerOut checker_out{};
    bool predecode_flush = false;   // the check flushes, to flush_address
    pc_t flush_address = 0;
    bpuf::PdInfo pd_next{};
    bool lh_new_valid = false;      // RVC=1: the "last half" state after the group (set when a head is checked)
    pc_t lh_new_pc = 0;
    uint16_t lh_new_data = 0;
    bool lh_updated = false;
  } out;

  void comb();
};

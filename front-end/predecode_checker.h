#ifndef PREDCODE_CHECKER_H
#define PREDCODE_CHECKER_H

#include "frontend.h"
#include "front_IO.h"
#include "predecode.h"

struct PredecodeCheckerIn {
  // from PTAB
  wire<1> predict_dir[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  // from instFIFO
  predecode_type_t predecode_type[FETCH_WIDTH];
  target_addr_t predecode_target_address[FETCH_WIDTH];
  pc_t seq_next_pc;
  // FE_PAIR: the group may consist of two fetch blocks (a pc discontinuity after a predicted-taken branch)
  wire<1> multi;
  pc_t pc[FETCH_WIDTH];
  wire<1> inst_valid[FETCH_WIDTH];
  // multi-block groups: slots that are inside the group (before RVC=1 drops the non-start ones), slots that begin a block, 16-bit instructions
  wire<1> in_range[FETCH_WIDTH];
  wire<1> seg_first[FETCH_WIDTH];
  wire<1> is_rvc[FETCH_WIDTH];
};

struct PredecodeCheckerOut {
    wire<1> predict_dir_corrected[FETCH_WIDTH];
    fetch_addr_t predict_next_fetch_address_corrected;
    wire<1> predecode_flush_enable;
    uint8_t keep_n;   // multi-block groups: number of leading slots that stay valid (everything after the flushing branch is dropped)
};

// The predecode check of one group (combinational: the checker keeps no state).
void predecode_check(const PredecodeCheckerIn &in, PredecodeCheckerOut &output);

#endif

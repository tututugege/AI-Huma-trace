#ifndef PREDCODE_CHECKER_H
#define PREDCODE_CHECKER_H

#include "frontend.h"
#include "front_IO.h"
#include "predecode.h"

struct predecode_checker_in {
  // from PTAB
  wire1_t predict_dir[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  // from instFIFO
  predecode_type_t predecode_type[FETCH_WIDTH];
  target_addr_t predecode_target_address[FETCH_WIDTH];
  pc_t seq_next_pc;
  // FE_PAIR: the group may consist of two fetch blocks (a pc discontinuity after a predicted-taken branch)
  wire1_t multi;
  pc_t pc[FETCH_WIDTH];
  wire1_t inst_valid[FETCH_WIDTH];
};

struct predecode_checker_out {
    wire1_t predict_dir_corrected[FETCH_WIDTH];
    fetch_addr_t predict_next_fetch_address_corrected;
    wire1_t predecode_flush_enable;
    uint8_t reason, seg;   // multi-block groups: why the checker flushed (1 branch ending a non-last segment leads elsewhere, 2 no branch where one was predicted, 3 last segment mismatch) and in which segment
    uint8_t keep_n;   // multi-block groups: number of leading slots that stay valid (everything after the flushing branch is dropped)
};

struct predecode_checker_read_data {
  predecode_checker_in inp_regs;
};

void predecode_checker_seq_read(struct predecode_checker_in *in,
                                struct predecode_checker_read_data *rd);
void predecode_checker_comb(const predecode_checker_read_data &input,
                            predecode_checker_out &output);
void predecode_checker_seq_write();

#endif

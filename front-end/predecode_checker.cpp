#include "predecode_checker.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>

void predecode_checker_seq_read(struct predecode_checker_in *in,
                                struct predecode_checker_read_data *rd) {
  assert(in);
  assert(rd);
  std::memset(rd, 0, sizeof(predecode_checker_read_data));
  rd->inp_regs = *in;
}

void predecode_checker_comb(const predecode_checker_read_data &input,
                            predecode_checker_out &output) {
  std::memset(&output, 0, sizeof(predecode_checker_out));
  output.keep_n = FETCH_WIDTH;

  for (int i = 0; i < FETCH_WIDTH; i++) {
    switch (input.inp_regs.predecode_type[i]) {
    case PREDECODE_NON_BRANCH:
      output.predict_dir_corrected[i] = false;
      break;
    case PREDECODE_DIRECT_JUMP_NO_JAL:
      output.predict_dir_corrected[i] = input.inp_regs.predict_dir[i];
      break;
    case PREDECODE_JALR:
    case PREDECODE_JAL:
      output.predict_dir_corrected[i] = true;
      break;
    default:
      std::printf("ERROR!!: predecode_type[%d] = %d\n", i,
                  input.inp_regs.predecode_type[i]);
      std::exit(1);
    }
  }

  if (input.inp_regs.multi) {
    // Segment by segment: a segment is a run of valid slots with consecutive pcs. The branch that ends a segment must lead to the pc of the next
    // segment (or, in the last one, to the predicted next fetch address); a branch that does not is a predecode flush and drops everything after it.
    const predecode_checker_in &in = input.inp_regs;
    int i = 0, segno = 0;
    while (i < FETCH_WIDTH && in.inst_valid[i]) {
      int e = i;
      while (e + 1 < FETCH_WIDTH && in.inst_valid[e + 1] && in.pc[e + 1] == in.pc[e] + 4) e++;
      const bool last = !(e + 1 < FETCH_WIDTH && in.inst_valid[e + 1]);
      const fetch_addr_t claimed = last ? in.predict_next_fetch_address : in.pc[e + 1];
      int ft = -1;
      for (int k = i; k <= e; k++) if (output.predict_dir_corrected[k]) { ft = k; break; }
      fetch_addr_t corrected;
      int keep = e + 1;
      if (ft >= 0) {
        const predecode_type_t ty = in.predecode_type[ft];
        corrected = (ty == PREDECODE_DIRECT_JUMP_NO_JAL || ty == PREDECODE_JAL) ? in.predecode_target_address[ft] : claimed;
        keep = ft + 1;
      } else {
        corrected = last ? in.seq_next_pc : in.pc[e] + 4;
      }
      if (corrected != claimed) {
        output.predict_next_fetch_address_corrected = corrected;
        output.predecode_flush_enable = true;
        output.keep_n = keep;
        output.seg = segno;
        output.reason = last ? 3 : (ft >= 0 ? 1 : 2);
        return;
      }
      if (last) { output.predict_next_fetch_address_corrected = corrected; return; }
      i = e + 1; segno++;
    }
    output.predict_next_fetch_address_corrected = in.predict_next_fetch_address;
    return;
  }

  int first_taken_index = -1;
  for (int i = 0; i < FETCH_WIDTH; i++) {
    if (output.predict_dir_corrected[i]) {
      first_taken_index = i;
      break;
    }
  }

  output.predict_next_fetch_address_corrected =
      input.inp_regs.predict_next_fetch_address;

  if (first_taken_index != -1) {
    if (input.inp_regs.predecode_type[first_taken_index] ==
            PREDECODE_DIRECT_JUMP_NO_JAL ||
        input.inp_regs.predecode_type[first_taken_index] == PREDECODE_JAL) {
      output.predict_next_fetch_address_corrected =
          input.inp_regs.predecode_target_address[first_taken_index];
    }
  } else {
    output.predict_next_fetch_address_corrected = input.inp_regs.seq_next_pc;
  }

  output.predecode_flush_enable =
      (input.inp_regs.predict_next_fetch_address !=
       output.predict_next_fetch_address_corrected);
}

void predecode_checker_seq_write() {}

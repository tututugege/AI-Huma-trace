#include "predecode_checker.h"
#include "fatal.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>

void predecode_check(const PredecodeCheckerIn &in, PredecodeCheckerOut &output) {
  output = {};
  output.keep_n = FETCH_WIDTH;

  for (int i = 0; i < FETCH_WIDTH; i++) {
    switch (in.predecode_type[i]) {
    case PREDECODE_NON_BRANCH:
      output.predict_dir_corrected[i] = false;
      break;
    case PREDECODE_DIRECT_JUMP_NO_JAL:
      output.predict_dir_corrected[i] = in.predict_dir[i];
      break;
    case PREDECODE_JALR:
    case PREDECODE_JAL:
      output.predict_dir_corrected[i] = true;
      break;
    default:
      fatal("ERROR!!: predecode_type[%d] = %d\n", i,
                  in.predecode_type[i]);
    }
  }

  if (in.multi) {
    // Segment (= fetch block) by segment: the branch that ends a segment must lead to the pc of the next segment (or, in the last one, to the predicted
    // next fetch address); a branch that does not is a predecode flush and drops everything after it.  RVC=1: a segment may contain slots that are not
    // instruction starts (in_range but not valid), and the sequential continuation of a segment is behind its last instruction.
    int i = 0;
    while (i < FETCH_WIDTH && in.in_range[i]) {
      int e = i;
      while (e + 1 < FETCH_WIDTH && in.in_range[e + 1] && !in.seg_first[e + 1]) e++;
      const bool last = !(e + 1 < FETCH_WIDTH && in.in_range[e + 1]);
      const fetch_addr_t claimed = last ? in.predict_next_fetch_address : in.pc[e + 1];
      int ft = -1, lv = -1;
      for (int k = i; k <= e; k++) { if (in.inst_valid[k]) lv = k; if (ft < 0 && output.predict_dir_corrected[k]) ft = k; }
      fetch_addr_t corrected;
      int keep = e + 1;
      if (ft >= 0) {
        const predecode_type_t ty = in.predecode_type[ft];
        corrected = (ty == PREDECODE_DIRECT_JUMP_NO_JAL || ty == PREDECODE_JAL) ? in.predecode_target_address[ft] : claimed;
        keep = ft + 1;
      } else {
        corrected = last ? in.seq_next_pc : (lv >= 0 ? in.pc[lv] + (in.is_rvc[lv] ? 2 : 4) : in.pc[e] + SLOT_BYTES);
      }
      if (corrected != claimed) {
        output.predict_next_fetch_address_corrected = corrected;
        output.predecode_flush_enable = true;
        output.keep_n = keep;
        return;
      }
      if (last) { output.predict_next_fetch_address_corrected = corrected; return; }
      i = e + 1;
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
      in.predict_next_fetch_address;

  if (first_taken_index != -1) {
    if (in.predecode_type[first_taken_index] ==
            PREDECODE_DIRECT_JUMP_NO_JAL ||
        in.predecode_type[first_taken_index] == PREDECODE_JAL) {
      output.predict_next_fetch_address_corrected =
          in.predecode_target_address[first_taken_index];
    }
  } else {
    output.predict_next_fetch_address_corrected = in.seq_next_pc;
  }

  output.predecode_flush_enable =
      (in.predict_next_fetch_address !=
       output.predict_next_fetch_address_corrected);
}


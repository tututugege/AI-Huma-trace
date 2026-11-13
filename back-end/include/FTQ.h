#pragma once
#include "config.h"
#include "wire_types.h"
#include <cstdint>

struct FTQEntry {
  wire<XLEN> slot_pc[BACKEND_FETCH_WIDTH];
  wire<1> slot_rvc[BACKEND_FETCH_WIDTH];
  wire<XLEN> next_pc; // Predicted Target of the block
  wire<1> pred_taken_mask[BACKEND_FETCH_WIDTH];
  uint8_t nslots = 0; // slots of the fetch group that carry instructions (FE_PAIR: tells a fetch-block boundary after a taken branch)

  FTQEntry() {
    for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) {
      slot_pc[i] = 0;
      slot_rvc[i] = false;
      pred_taken_mask[i] = false;
    }
    next_pc = 0;
    nslots = 0;
  }
};

// Prediction/training sideband metadata that backend execution path does not
// need to random-read. This payload is consumed in FIFO order with FTQ blocks.
struct FTQTrainMetaEntry {
  PredMetaT<BACKEND_FETCH_WIDTH> meta = {};
  wire<1> mid_pred[BACKEND_FETCH_WIDTH] = {}; // For future use (mid-block prediction)
};

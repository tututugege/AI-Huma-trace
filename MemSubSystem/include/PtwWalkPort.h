#pragma once

#include "IO.h"

// The port between a TLB and the shared page-table walker (MemPtwBlock), one per TLB.
// TLB -> walker (written by the TLB's comb, read by the walker's comb_request / comb_feedback, cleared after):
struct PtwWalkPortCombIn {
  bool req_valid = false;      // a walk request (only when the walker showed req_ready)
  PtwWalkReq req = {};
  bool resp_consumed = false;  // the walk response was taken
  bool flush = false;          // the TLB dropped its walk (flush / cancel)
};
// walker -> TLB (registered: the walker's outputs at the last clock edge):
struct PtwWalkPortCombOut {
  bool req_ready = false;
  bool resp_valid = false;
  PtwWalkResp resp = {};
};

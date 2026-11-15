#pragma once
#include "config.h"
#include "Exu.h"
#include "IO.h"

class Prf {
public:
  Prf(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

  void comb_awake();
  void comb_req_ftq();
  void comb_read();
  void comb_pipeline();
  void init();
  void seq();

  ExePrfIO::ExePrfEntry inst_r[ISSUE_WIDTH];

  ExePrfIO::ExePrfEntry inst_r_1[ISSUE_WIDTH];
};

#include "FrontTop.h"
#include "Back2Front.h"
#include "BackTop.h"
#include "config.h"
#include "front_core.h"
#include "front_module.h"
#include "train_IO.h"
#include "util.h"
#include <cstring>
#include <memory>

FrontTop::FrontTop()
    : core(std::make_unique<FrontCore>()), read_data_(std::make_unique<FrontReadData>()) {}



FrontTop::~FrontTop() = default;

void FrontTop::init() {
  CsrStatusIO *bound_csr = in.csr_status;
  core->bind(&in, read_data_.get(), &out, ctx);
  core->icache.set_context(ctx);
  core->icache.bind_walk();
  in = {};
  out = {};
  in.csr_status = bound_csr;
  Assert(ctx != nullptr && "FrontTop::init requires bound SimContext");
  Assert(in.csr_status != nullptr && "FrontTop::init requires bound csr_status");
  in.ctrl.fifo_read_enable = true;
  in.ctrl.reset = true;
  step_bpu();
  in.ctrl.reset = false;
}

void FrontTop::read_core_q() {
  core->read_q(*read_data_);
}

void FrontTop::comb_core() {
  core->comb();
}

void FrontTop::seq_core() {
  core->seq(in);
  core->read_q(*read_data_);   // q and the FIFOs' read data of the next cycle
}

void FrontTop::warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc) {
  core->bpu_pipe.warm_commit(pc, type, dir, target, rvc);
}
void FrontTop::warm_finish() { core->bpu_pipe.warm_finish(); }
void FrontTop::warm_icache(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]) {
  core->icache.warm_install(tag, valid);
}
void FrontTop::warm_itlb(const void *entries, int count, int repl_ptr) { core->icache.warm_install_itlb(entries, count, repl_ptr); }

void FrontTop::restart_at(pc_t pc) {
  in.ctrl.reset = false;
  in.ctrl.fifo_read_enable = false;
  in.redirect.valid = true;
  in.redirect.address = pc;
}

void FrontTop::step_bpu() {
  read_core_q();
  comb_core();
  seq_core();
}



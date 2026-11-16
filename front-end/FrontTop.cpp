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
    : core_(std::make_unique<FrontCore>()), read_data_(std::make_unique<FrontReadData>()),
      update_req_(std::make_unique<FrontUpdateRequest>()) {}

// the I-cache's ports (SimCpu binds them on FrontTop; they take effect at the I-cache's next step)
void FrontTop::bind_ports() {
  ICacheTop::Ports p;
  p.ptw_walk = icache_ptw_walk_port;
  p.probe_req = icache_mem_req_port;
  p.probe_resp = icache_mem_resp_port;
  p.read = icache_mem_read_port;
  p.read_q = icache_mem_read_q;
  core_->icache.set_ports(p);
}

FrontTop::~FrontTop() = default;

void FrontTop::init() {
  CsrStatusIO *bound_csr = in.csr_status;
  bind_ports();
  core_->ctx = ctx;
  core_->icache.set_context(ctx);
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
  bind_ports();
  core_->read_q(*read_data_);
}

void FrontTop::comb_core() {
  *update_req_ = {};
  core_->comb(in, *read_data_, out, *update_req_);
}

void FrontTop::seq_core() { core_->seq(in, *update_req_); }

void FrontTop::warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc) {
  core_->bpu_pipe.warm_commit(pc, type, dir, target, rvc);
}
void FrontTop::warm_finish() { core_->bpu_pipe.warm_finish(); }
void FrontTop::warm_icache(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]) {
  core_->icache.warm_install(tag, valid);
}
void FrontTop::warm_itlb(const void *entries, int count, int repl_ptr) { core_->icache.warm_install_itlb(entries, count, repl_ptr); }

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

// One front-end cycle from the back end's registered outputs (Back2Front): when the back end does not stall (or a refetch is pending) the
// fetch-group FIFO is read (can_read; Front2Back then takes the group from out), then the BPU / FIFOs / I-cache comb.
void FrontTop::comb(CsrStatusIO *csr_status) {
  const bool registered_refetch = static_cast<bool>(from_back->in.redirect.valid);
  can_read = !from_back->stall || registered_refetch;

  in = from_back->in;   // (reset is 0 here: only init() runs the front end with reset)
  in.csr_status = csr_status;
  in.ctrl.fifo_read_enable = can_read;
  if (!can_read) in.redirect.valid = false;
  read_core_q();
  comb_core();
}


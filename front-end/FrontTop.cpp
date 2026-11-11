#include "FrontTop.h"
#include "config.h"
#include "front_module.h"
#include "oracle.h"
#include "train_IO.h"
#include "util.h"
#include <cstring>
#include <memory>

namespace {
inline void sync_icache_ptw_ports(const FrontTop &front) {
  icache_ptw_mem_port = front.icache_ptw_mem_port;
  icache_ptw_walk_port = front.icache_ptw_walk_port;
  icache_mem_req_port = front.icache_mem_req_port;
  icache_mem_resp_port = front.icache_mem_resp_port;
  icache_mem_read_port = front.icache_mem_read_port;
}
} // namespace

FrontTop::FrontTop()
    : read_data_(std::make_unique<FrontReadData>()),
      update_req_(std::make_unique<FrontUpdateRequest>()) {}

FrontTop::~FrontTop() = default;

void FrontTop::init() {
  CsrStatusIO *bound_csr = in.csr_status;
  sync_icache_ptw_ports(*this);
  front_set_context(ctx);
  icache_set_context(ctx);
  std::memset(&in, 0, sizeof(in));
  std::memset(&out, 0, sizeof(out));
  std::memset(&back2front, 0, sizeof(back2front));
  std::memset(&back2front_1, 0, sizeof(back2front_1));
  in.csr_status = bound_csr;
  back2front.csr_status = bound_csr;
  back2front_1.csr_status = bound_csr;
  back2front_stall = false;
  back2front_stall_1 = false;
  Assert(ctx != nullptr && "FrontTop::init requires bound SimContext");
  Assert(in.csr_status != nullptr && "FrontTop::init requires bound csr_status");
  in.FIFO_read_enable = true;
  in.reset = true;
  step_bpu();
  in.reset = false;
}

void FrontTop::seq_read_bpu() {
  sync_icache_ptw_ports(*this);
  front_top_seq_read(in, *read_data_);
}

void FrontTop::comb_bpu() {
  sync_icache_ptw_ports(*this);
  *update_req_ = {};
  front_comb_calc(in, *read_data_, out, *update_req_);
}

void FrontTop::seq_write_bpu() {
  sync_icache_ptw_ports(*this);
  front_top_seq_write(in, *update_req_, in.reset);
}

void FrontTop::seq_back2front() {
  back2front = back2front_1;
  back2front_stall = back2front_stall_1;
}

void FrontTop::force_back2front_refetch(pc_t pc) {
  back2front.refetch = true;
  back2front.refetch_address = pc;
  back2front_1 = back2front;
  back2front_stall = false;
  back2front_stall_1 = false;
}

void FrontTop::step_bpu() {
  seq_read_bpu();
  comb_bpu();
  seq_write_bpu();
}

void FrontTop::step_oracle() {
  sync_icache_ptw_ports(*this);
  get_oracle(in, out);
}


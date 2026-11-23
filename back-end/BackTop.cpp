#include <cstdlib>
#include <cstdio>
#include "trace_hook.h"
#include "BackTop.h"
#include "IO.h"
#include "RealLsu.h"
#include "config.h"
#include "RefCore.h"
#include "mmio_address_map.h"
#include "ref.h"
#include "util.h"

#include <cstdint>
#include <cstring>

BackTop::~BackTop() = default;

void BackTop::init() {
  pre = std::make_unique<PreIduQueue>(ctx);
  idu = std::make_unique<Idu>(ctx, MAX_BR_PER_CYCLE);
  rename = std::make_unique<Ren>(ctx);
  dis = std::make_unique<Dispatch>(ctx);
  isu = std::make_unique<Isu>(ctx);
  exu = std::make_unique<Exu>(ctx);
  rob = std::make_unique<Rob>(ctx);
  lsu = std::make_unique<RealLsu>(ctx);

  exu->ftq = pre.get();
  rob->ftq = pre.get();
  // the modules' outputs that are the back end's interface (kept here)
  lsu->out.lsu2dcache = &out.lsu2dcache;
  lsu->out.peripheral_req = &out.peripheral_req;
  lsu->out.commit_store = &out.commit_store;
  // the modules' inputs: the other modules' outputs, or the back end's interface
  pre->in.idu_br_latch = &idu->q().br_latch;
  pre->in.idu_consume = &idu->out.idu_consume;
  pre->in.top_in = &in;
  pre->in.rob_bcast = &rob->out.rob_bcast_d;
  pre->in.rob_commit = &rob->out.rob_commit;
  idu->in.exu2id = &exu->out.exu2id;
  idu->in.pre_issue = &pre->out.pre_issue;
  idu->in.ren2dec = &rename->out.ren2dec;
  idu->in.rob_bcast = &rob->out.rob_bcast_d;
  rename->in.dec2ren = &idu->out.dec2ren;
  rename->in.dec_bcast = &idu->out.dec_bcast;
  rename->in.dis2ren = &dis->out.dis2ren;
  rename->in.rob_bcast = &rob->out.rob_bcast_d;
  rename->in.rob_commit = &rob->out.rob_commit;
  dis->in.dec_bcast = &idu->out.dec_bcast;
  dis->in.iss2dis = &isu->out.iss2dis;
  dis->in.iss_awake = &isu->out.iss_awake;
  dis->in.lsu2dis = &lsu->out.lsu2dis;
  dis->in.ren2dis = &rename->out.ren2dis;
  dis->in.rob2dis = &rob->out.rob2dis;
  dis->in.rob_bcast = &rob->out.rob_bcast_d;
  isu->in.dec_bcast = &idu->out.dec_bcast;
  isu->in.dis2iss = &dis->out.dis2iss;
  isu->in.exe2iss = &exu->out.exe2iss;
  isu->in.lsu_awake = &lsu->out.lsu_awake;
  isu->in.prf_awake = &exu->out.prf_awake;
  isu->in.rob_bcast = &rob->out.rob_bcast_d;
  exu->in.dec_bcast = &idu->out.dec_bcast;
  exu->in.iss2prf = &isu->out.iss2prf;
  exu->in.lsu2exe = &lsu->out.lsu2exe;
  exu->in.rob_bcast = &rob->out.rob_bcast_d;
  rob->in.dec_bcast = &idu->out.dec_bcast;
  rob->in.dis2rob = &dis->out.dis2rob;
  rob->in.exu2rob = &exu->out.exu2rob;
  rob->in.top_in = &in;
  rob->in.lsu2rob = &lsu->out.lsu2rob;
  rob->in.pre2front = &pre->out.pre2front;
  rob->in.ftq_commit_info = &pre->out.ftq_commit_info;
  rob->in.idu_br_latch = &idu->q().br_latch;
  rob->in.ren2rob = &rename->out.ren2rob;
  rob->out.top_out = &out;
  lsu->in.dec_bcast = &idu->out.dec_bcast;
  lsu->in.dis2lsu = &dis->out.dis2lsu;
  lsu->in.exe2lsu = &exu->out.exe2lsu;
  lsu->in.top_in = &in;
  lsu->in.rob_bcast = &rob->out.rob_bcast_d;
  lsu->in.rob_commit = &rob->out.rob_commit;

  pre->init();
  idu->init();
  rename->init();
  dis->init();
  isu->init();
  exu->init();
  rob->init();
  lsu->init();
}

// ---------------------------------------------------------------------------------------------
// One back-end cycle is evaluated in dependency order: every stage reads only signals that an
// earlier stage (or registered state) already produced. The call order in BackTop::comb() is that order.
// ---------------------------------------------------------------------------------------------


void BackTop::seq() {
  // rename -> isu/stq/rob
  pre->seq();
  rename->seq();
  dis->seq();
  idu->seq();
  isu->seq();
  exu->seq();
  rob->seq();
  lsu->seq();
}

// Architectural state taken from the reference model / a checkpoint that the back end itself uses: the PC and the LR/SC
// reservation (the CSR file: SimCpu::install_arch_state).
void BackTop::install_arch_state(const CPU_state &state) {
  restored_pc = state.pc;
  lsu->cur_mut().lrsc_unit.reserve_valid = lsu->nxt.lrsc_unit.reserve_valid = state.reserve_valid;
  lsu->cur_mut().lrsc_unit.reserve_addr = lsu->nxt.lrsc_unit.reserve_addr = state.reserve_addr;
  // the pipeline starts with a refetch from the restored pc
  out.flush = true;
  out.redirect_pc = state.pc;
}

void BackTop::restart_at(pc_t pc) {
  out.flush = true;
  out.mispred = true;
  out.redirect_pc = pc;
}

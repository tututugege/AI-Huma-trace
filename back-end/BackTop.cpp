#include <cstdlib>
#include <cstdio>
#include "trace_hook.h"
#include "BackTop.h"
#include "Csr.h"
#include "IO.h"
#include "PhysMemory.h"
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
  csr = std::make_unique<Csr>();
  rob = std::make_unique<Rob>(ctx);
  lsu = std::make_unique<RealLsu>(ctx);
  dtlb_mmu =
      std::make_unique<TlbMmu>(ctx, DTLB_ENTRIES, TlbMmu::Kind::DTLB);
  lsu->dtlb = dtlb_mmu.get();

  exu->ftq = pre.get();
  exu->csr = csr.get();
  rob->ftq = pre.get();
  // the modules' outputs that are the back end's interface (kept here)
  csr->out.csr_status = &csr_status;
  lsu->out.lsu2dcache = &out.lsu2dcache;
  lsu->out.peripheral_req = &out.peripheral_req;
  lsu->out.commit_store = &out.commit_store;
  // the modules' inputs: the other modules' outputs, or the back end's interface
  pre->in.idu_br_latch = &idu->q().br_latch;
  pre->in.idu_consume = &idu->out.idu_consume;
  pre->in.top_in = &in;
  pre->in.rob_bcast = &rob->out.rob_bcast;
  pre->in.rob_commit = &rob->out.rob_commit;
  idu->in.exu2id = &exu->out.exu2id;
  idu->in.pre_issue = &pre->out.pre_issue;
  idu->in.ren2dec = &rename->out.ren2dec;
  idu->in.rob_bcast = &rob->out.rob_bcast;
  rename->in.dec2ren = &idu->out.dec2ren;
  rename->in.dec_bcast = &idu->out.dec_bcast;
  rename->in.dis2ren = &dis->out.dis2ren;
  rename->in.rob_bcast = &rob->out.rob_bcast;
  rename->in.rob_commit = &rob->out.rob_commit;
  dis->in.dec_bcast = &idu->out.dec_bcast;
  dis->in.iss2dis = &isu->out.iss2dis;
  dis->in.iss_awake = &isu->out.iss_awake;
  dis->in.lsu2dis = &lsu->out.lsu2dis;
  dis->in.prf_awake = &exu->out.prf_awake;
  dis->in.ren2dis = &rename->out.ren2dis;
  dis->in.rob2dis = &rob->out.rob2dis;
  dis->in.rob_bcast = &rob->out.rob_bcast;
  isu->in.dec_bcast = &idu->out.dec_bcast;
  isu->in.dis2iss = &dis->out.dis2iss;
  isu->in.exe2iss = &exu->out.exe2iss;
  isu->in.lsu_awake = &lsu->out.lsu_awake;
  isu->in.prf_awake = &exu->out.prf_awake;
  isu->in.rob_bcast = &rob->out.rob_bcast;
  exu->in.dec_bcast = &idu->out.dec_bcast;
  exu->in.iss2prf = &isu->out.iss2prf;
  exu->in.lsu2exe = &lsu->out.lsu2exe;
  exu->in.rob_bcast = &rob->out.rob_bcast;
  rob->in.csr2rob = &csr->out.csr2rob;
  rob->in.dec_bcast = &idu->out.dec_bcast;
  rob->in.dis2rob = &dis->out.dis2rob;
  rob->in.exu2rob = &exu->out.exu2rob;
  rob->in.top_in = &in;
  rob->in.lsu2rob = &lsu->out.lsu2rob;
  csr->in.csr_interrupt_inject_io = &csr_interrupt_inject_io;
  csr->in.exe2csr = &exu->out.exe2csr;
  csr->in.rob2csr = &rob->out.rob2csr;
  csr->in.rob_bcast = &rob->out.rob_bcast;
  csr->in.top2csr = &top2csr;
  lsu->in.csr_status = &csr_status;
  lsu->in.dec_bcast = &idu->out.dec_bcast;
  lsu->in.dis2lsu = &dis->out.dis2lsu;
  lsu->in.exe2lsu = &exu->out.exe2lsu;
  lsu->in.top_in = &in;
  lsu->in.rob_bcast = &rob->out.rob_bcast;
  lsu->in.rob_commit = &rob->out.rob_commit;

  pre->init();
  idu->init();
  rename->init();
  dis->init();
  isu->init();
  exu->init();
  csr->init();
  rob->init();
  lsu->init();
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->flush();
    dtlb_mmu->seq();
  }
}

void BackTop::set_lsu_ptw_walk_port(PtwWalkPort *port) {
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->set_ptw_walk_port(port);
  }
}

void BackTop::comb_csr_status() {
  csr->comb_csr_status();
  out.sstatus = csr_status.sstatus;
  out.mstatus = csr_status.mstatus;
  out.satp = csr_status.satp;
  out.privilege = csr_status.privilege;
}

void BackTop::clear_comb_inputs() { in = BackIn{}; }

void BackTop::comb_in_regs() {
  clear_comb_inputs();
  in.peripheral_resp = in_q_.peripheral_resp;
}

// ---------------------------------------------------------------------------------------------
// One back-end cycle is evaluated in dependency order: every stage reads only signals that an
// earlier stage (or registered state) already produced. The call order in BackTop::comb() is that order.
// ---------------------------------------------------------------------------------------------

// Working copies and default outputs of every module.
// Branch/decode results, free resources (rename registers, issue queues, LDQ/STQ) and the wakeups/write-backs completing this cycle.
void BackTop::comb_outputs() {
  for (int i = 0; i < BACKEND_FETCH_WIDTH; i++) out.fire[i] = pre->out.pre2front.fire[i];
  out.flush = rob->out.rob_bcast.flush;
  if (out.flush) {
    int64_t committing = -1;
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      if (rob->out.rob_commit.commit_entry[i].valid) {
        committing = (int64_t)rob->out.rob_commit.commit_entry[i].uop->dbg.inst_idx;
        break;
      }
    }
    tbhook::on_flush(committing, rob->out.rob_bcast.interrupt);
  }
  out.fence_i = rob->out.rob_bcast.fence_i;
  out.itlb_flush = rob->out.rob_bcast.fence;

  // 1. Normal case (No Rob flush)
  if (!rob->out.rob_bcast.flush) {
    out.mispred = idu->out.dec_bcast.mispred;
    out.stall = !pre->out.pre2front.ready;
    out.redirect_pc = idu->q().br_latch.redirect_pc;
    out.rf_valid = idu->out.dec_bcast.mispred;
    if (out.rf_valid) {
      const uint32_t fi = idu->q().br_latch.ftq_idx;
      out.rf_slot = idu->q().br_latch.ftq_off;
      out.rf_blk = pre->ftq_blk_id(fi, out.rf_slot);
      out.rf_type = idu->q().br_latch.br_type;
      out.rf_pc = pre->ftq_slot_pc(fi, out.rf_slot);
      out.rf_is_rvc = pre->ftq_slot_rvc(fi, out.rf_slot);
    }
  } else {
    out.mispred = true;
    if (rob->out.rob_bcast.mret || rob->out.rob_bcast.sret) {
      out.redirect_pc = csr->out.csr2front.epc;
    } else if (rob->out.rob_bcast.exception) {
      out.redirect_pc = csr->out.csr2front.trap_pc;
    } else {
      out.redirect_pc = rob->out.rob_bcast.pc + ilen_of(rob->out.rob_bcast.is_rvc);
    }
  }

  top2csr.fs_dirty = false;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    out.commit_entry[i] =
        rob->out.rob_commit.commit_entry[i].uop.to_inst_entry(
            rob->out.rob_commit.commit_entry[i].valid);
    out.commit_extra[i] = rob->out.rob_commit.commit_extra[i];
    out.commit_ftq[i] = pre->out.ftq_commit_info.resp[i];
    if (rob->out.rob_commit.commit_entry[i].valid &&
        !rob->out.rob_bcast.interrupt) {
      if (tbhook::fs_dirty_at_commit((uint64_t)rob->out.rob_commit.commit_entry[i].uop->dbg.inst_idx)) top2csr.fs_dirty = true;
      tbhook::on_commit(
          (uint64_t)rob->out.rob_commit.commit_entry[i].uop->dbg.inst_idx);
    }
    if (out.commit_entry[i].valid && out.flush) {
      // Flush: the next PC of every committing instruction is the redirect target (trap vector, epc, ...)
      out.commit_entry[i].uop.diag_val = out.redirect_pc;
    }
  }
}

void BackTop::comb() {
  // The CSR status is refreshed by SimCpu::cycle() before the front / back comb (back.comb_csr_status).
  // The phase order follows from the module contracts (the CONTRACT lines above each module's class): a module's
  // phases in header order, a writer before the readers of its outputs, the trace-bridge users in a fixed order.
  // BEGIN GENERATED by scripts/comb/gen_comb_order.py from the module contracts: do not edit
  pre->comb_accept_front();
  rename->comb_rename();
  idu->comb_decode();
  lsu->comb_outputs();
  csr->comb_interrupt();
  rob->comb_commit();
  pre->comb_commit_info();
  lsu->comb_writeback();
  dis->comb_alloc();
  exu->comb_exec();
  lsu->comb_translate();
  csr->comb_commit();
  isu->comb_issue();
  dis->comb_dispatch();
  rename->comb_fire();
  idu->comb_fire();
  pre->comb_fire();
  lsu->comb_requests();
  lsu->comb_commit_store();
  exu->comb_pipeline();
  isu->comb_enq();
  comb_outputs();
  rob->comb_update();
  // END GENERATED
}

// The back end's phases that read an input another top drives after BackTop::comb() (the CSR's interrupt lines, from
// the commit's device stores: SimCpu::comb_commit_store); gen_comb_order.py puts them here.
void BackTop::comb_late() {
  // BEGIN GENERATED (late) by scripts/comb/gen_comb_order.py from the module contracts: do not edit
  csr->comb_interrupt_inject();
  // END GENERATED
  pmu_sample();
}

void BackTop::seq() {
  in_q_ = in_d;
  // rename -> isu/stq/rob
  pre->seq();
  rename->seq();
  dis->seq();
  idu->seq();
  isu->seq();
  exu->seq();
  rob->seq();
  csr->seq();
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->seq();
  }
  lsu->seq();
}

// Architectural state taken from the reference model / a checkpoint that the timing model itself uses: PC, privilege,
// CSRs (address translation, interrupts, traps) and the LR/SC reservation.
void BackTop::install_arch_state(const CPU_state &state, uint8_t privilege) {
  restored_pc = state.pc;
  csr->q_mut().privilege = csr->d.privilege = privilege;
  for (int i = 0; i < CSR_NUM; i++) csr->q_mut().CSR_RegFile[i] = csr->d.CSR_RegFile[i] = state.csr[i];
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
  comb_csr_status();
}

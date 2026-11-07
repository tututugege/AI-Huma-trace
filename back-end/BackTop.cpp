#include "trace_hook.h"
#include "BackTop.h"
#include "Csr.h"
#include "IO.h"
#include "PhysMemory.h"
#include "RealLsu.h"
#include "ShadowHooks.h"
#include "config.h"
#include "diff.h"
#include "mmio_address_map.h"
#include "oracle.h"
#include "ref.h"
#include "util.h"

#include <cstdint>
#include <cstring>

void BackTop::init() {
  pre = new PreIduQueue(ctx);
  idu = new Idu(ctx, MAX_BR_PER_CYCLE);
  rename = new Ren(ctx);
  dis = new Dispatch(ctx);
  isu = new Isu(ctx);
  prf = new Prf(ctx);
  exu = new Exu(ctx);
  csr = new Csr();
  rob = new Rob(ctx);
  lsu = new RealLsu(ctx);
  dtlb_mmu =
      std::make_unique<TlbMmu>(ctx, nullptr, DTLB_ENTRIES, TlbMmu::Kind::DTLB);

  out.fire = pre2front.fire;

  pre->wires = this;
  idu->wires = this;
  rename->wires = this;
  dis->wires = this;
  isu->wires = this;
  prf->wires = this;
  exu->wires = this;
  rob->wires = this;
  csr->wires = this;
  lsu->wires = this;
  idu_br_latch = &idu->br_latch;

  pre->init();
  idu->init();
  rename->init();
  dis->init();
  isu->init();
  prf->init();
  exu->init();
  csr->init();
  rob->init();
  lsu->init();
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->flush();
    dtlb_mmu->seq();
  }
}

void BackTop::set_lsu_ptw_mem_port(PtwMemPort *port) {
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->set_ptw_mem_port(port);
  }
}

void BackTop::set_lsu_ptw_walk_port(PtwWalkPort *port) {
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->set_ptw_walk_port(port);
  }
}

void BackTop::comb_lsu_mmu() {
  mmu2lsu_io = {};
  if (dtlb_mmu == nullptr) {
    return;
  }

  if (rob_bcast.fence) {
    dtlb_mmu->flush();
    return;
  }
  if (rob_bcast.flush || dec_bcast.mispred) {
    dtlb_mmu->cancel_pending_walk();
    return;
  }

  dtlb_mmu->translate_lsu_ports(lsu2mmu_io, mmu2lsu_io);
}

void BackTop::comb_csr_status() {
  CSR(comb_csr_status);
  out.sstatus = csr_status.sstatus;
  out.mstatus = csr_status.mstatus;
  out.satp = csr_status.satp;
  out.privilege = csr_status.privilege;
}

void BackTop::clear_comb_inputs() { in = Back_in{}; }

// ---------------------------------------------------------------------------------------------
// One back-end cycle is evaluated in dependency order: every stage reads only signals that an
// earlier stage (or registered state) already produced. The call order in BackTop::comb() is that order.
// ---------------------------------------------------------------------------------------------

// Working copies and default outputs of every module.
void BackTop::comb_begin_all() {
  PRE(comb_begin);
  IDU(comb_begin);
  REN(comb_begin);
  DIS(comb_begin);
  ISU(comb_begin);
  prf->comb_begin();
  EXU(comb_begin);
  ROB(comb_begin);
  CSR(comb_begin);
}

// Branch/decode results, free resources (rename registers, issue queues, LDQ/STQ) and the wakeups/write-backs completing this cycle.
void BackTop::comb_stage_decode_and_resources() {
  IDU(comb_branch);
  PRE(comb_accept_front);
  IDU(comb_decode);
  CSR(comb_interrupt);
  REN(comb_alloc);
  prf->comb_complete();
  prf->comb_awake();
  prf->comb_write();
  ISU(comb_ready);
  LSU(comb_cal);
  LSU(comb_lsu2dis);
  LSU(comb_lsu2rob);
}

// ROB head -> commit group; committed stores release their STQ entries in the LSU.
void BackTop::comb_stage_commit() {
  ROB(comb_ready);
  ROB(comb_ftq_pc_req);
  PRE(comb_ftq_lookup_rob);
  ROB(comb_commit);
  PRE(comb_commit_info);
  LSU(comb_stq_commit);
}

// MMIO/DCache responses finish loads and stores and produce write-backs; CSR read data.
void BackTop::comb_stage_memory_responses() {
  DIS(comb_alloc);
  LSU(comb_mmio_in);
  LSU(comb_dcache2lsu_ldq);
  LSU(comb_dcache2lsu_stq);
  LSU(comb_lsu2exe);
  EXU(comb_to_csr);
  CSR(comb_csr_read);
}

// Functional units execute; their results complete ROB entries.
void BackTop::comb_stage_execute() {
  EXU(comb_exec);
  ROB(comb_complete);
}

// Issue selection, then LSU address translation (the MMU sits between the LSU's request and response phases).
void BackTop::comb_stage_issue_and_translate() {
  EXU(comb_ready);
  ISU(comb_issue);
  LSU(comb_exe2lsu);
  LSU(comb_tlb_out);
  comb_lsu_mmu();
  LSU(comb_tlb_in_ldq);
  LSU(comb_tlb_in_stq);
  LSU(comb_stlf);
}

// Register reads, next-cycle wakeups, exceptions, rename and dispatch of the instructions leaving decode.
void BackTop::comb_stage_operands_and_dispatch() {
  prf->comb_req_ftq();
  PRE(comb_ftq_lookup_prf);
  prf->comb_read();

  ISU(comb_awake);
  ISU(comb_calc_latency_next);
  CSR(comb_exception);
  CSR(comb_csr_write);
  DIS(comb_wake);
  REN(comb_rename);

  DIS(comb_dispatch);
}

// Handshakes that move instructions (fire), the LSU's DCache/MMIO requests, queue enqueue, and recovery of ROB/IQs from a mispredict or global flush (flush wins).
void BackTop::comb_stage_fire_and_recover() {
  DIS(comb_fire);
  LSU(comb_dis2lsu);
  REN(comb_fire);
  IDU(comb_fire);
  // Kill flushed LSU work before driving new DCache/MMIO requests.
  LSU(comb_flush);
  LSU(comb_lsu2dcache_ldq);
  LSU(comb_lsu2dcache_stq);
  LSU(comb_mmio_out);
  LSU(comb_check);
  ISU(comb_enq);
  // Rob recovery was split out of comb_fire(); keep the original priority:
  // branch rollback happens before enqueue, and global flush overrides both.
  ROB(comb_branch);
  ROB(comb_fire);
  ROB(comb_flush);
  ISU(comb_flush);

  PRE(comb_fire);
  prf->comb_pipeline();
  EXU(comb_pipeline);
  DIS(comb_pipeline);
  REN(comb_pipeline);
  CSR(comb_interrupt_inject);
}

// Diagnostic mode (CONFIG_BE_IO_CLEAR_AT_COMB_BEGIN): clear the stage IOs to expose hidden dependence on
// previous-cycle combinational values.
void BackTop::clear_stage_ios() {
#if CONFIG_BE_IO_CLEAR_AT_COMB_BEGIN
  // Diagnostic mode: clear backend internal stage IOs to expose hidden
  // dependence on previous-cycle combinational values.
  pre2front = {};
  pre_issue = {};
  ftq_prf_pc_req = {};
  ftq_prf_pc_resp = {};
  ftq_rob_pc_req = {};
  ftq_rob_pc_resp = {};
  ftq_commit_info = {};

  dec2ren = {};
  dec_bcast = {};

  ren2dec = {};
  idu_consume = {};
  ren2dis = {};

  dis2ren = {};
  dis2iss = {};
  dis2rob = {};
  dis2lsu = {};

  iss_awake = {};
  iss2prf = {};
  iss2dis = {};

  prf2exe = {};
  prf_awake = {};

  exu2id = {};
  // Known issue path: ROB consumes exu2rob before EXU refreshes it in current
  // comb order. Keep it out of global clear for now and handle EXU separately.
  exe2prf = {};
  exe2iss = {};
  exe2lsu = {};
  exe2csr = {};

  rob2dis = {};
  rob2csr = {};
  rob_bcast = {};
  rob_commit = {};

  csr2exe = {};
  csr2rob = {};
  csr2front = {};

  lsu2exe = {};
  lsu2dis = {};
  lsu2rob = {};
  out.peripheral_req = {};
  out.lsu2dcache = {};
  lsu2mmu_io = {};
  mmu2lsu_io = {};
#endif
}

// Outputs to the front end / difftest: redirect (mispredict or flush target), commit group.
void BackTop::comb_outputs() {
  // 用于调试
  // 修正pc_next 以及difftest对应的pc_next
  out.flush = rob_bcast.flush;
  if (out.flush && tbhook::enabled) {
    int64_t committing = -1;
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      if (rob_commit.commit_entry[i].valid) {
        committing = (int64_t)rob_commit.commit_entry[i].uop->dbg.inst_idx;
        break;
      }
    }
    tbhook::on_flush(committing, rob_bcast.interrupt);
  }
  out.fence_i = rob_bcast.fence_i;
  out.itlb_flush = rob_bcast.fence;

  // 1. Normal case (No Rob flush)
  if (!rob_bcast.flush) {
    out.mispred = dec_bcast.mispred;
    out.stall = !pre2front.ready;
    out.redirect_pc = idu->br_latch.redirect_pc;
    out.rf_valid = dec_bcast.mispred;
    if (out.rf_valid) {
      const uint32_t fi = idu->br_latch.ftq_idx;
      out.rf_blk = pre->ftq_blk_id(fi);
      out.rf_slot = idu->br_latch.ftq_off;
      out.rf_type = idu->br_latch.br_type;
      out.rf_pc = pre->ftq_slot_pc(fi, out.rf_slot);
    }
  } else {
    out.mispred = true;
    if (rob_bcast.mret || rob_bcast.sret) {
      out.redirect_pc = csr2front.epc;
    } else if (rob_bcast.exception) {
      out.redirect_pc = csr2front.trap_pc;
    } else {
      out.redirect_pc = rob_bcast.pc + 4;
    }
    BE_LOG("flush redirect_pc=0x%08x", (uint32_t)out.redirect_pc);
  }

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    out.commit_entry[i] =
        rob_commit.commit_entry[i].uop.to_inst_entry(
            rob_commit.commit_entry[i].valid);
    if (tbhook::enabled && rob_commit.commit_entry[i].valid &&
        !rob_bcast.interrupt) {
      tbhook::on_commit(
          (uint64_t)rob_commit.commit_entry[i].uop->dbg.inst_idx);
    }
    if (out.commit_entry[i].valid && out.flush) {
      // Flush: override extra_data.pc_next with redirect_pc for ALL
      // instructions so Difftest can read the correct next-PC (trap vector,
      // epc, etc.)
      out.commit_entry[i].uop.diag_val = out.redirect_pc;
      rob_commit.commit_entry[i].uop.diag_val = out.redirect_pc;
    }
  }
}

void BackTop::comb() {
  // CSR 状态由 SimCpu::cycle() 在进入 front/back 组合逻辑前统一刷新。
  clear_stage_ios();

  comb_begin_all();
  comb_stage_decode_and_resources();
  comb_stage_commit();
  comb_stage_memory_responses();
  comb_stage_execute();
  comb_stage_issue_and_translate();
  comb_stage_operands_and_dispatch();

  comb_outputs();
  comb_stage_fire_and_recover();
  pmu_sample();
}

void BackTop::seq() {
  // rename -> isu/stq/rob
  // exu -> prf
  PRE(seq);
  REN(seq);
  DIS(seq);
  IDU(seq);
  ISU(seq);
  EXU(seq);
  prf->seq();
  ROB(seq);
  CSR(seq);
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->seq();
  }
  LSU(seq);
}


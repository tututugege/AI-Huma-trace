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

void BackTop::init() {
  pre = new PreIduQueue(ctx);
  idu = new Idu(ctx, MAX_BR_PER_CYCLE);
  rename = new Ren(ctx);
  dis = new Dispatch(ctx);
  isu = new Isu(ctx);
  exu = new Exu(ctx);
  csr = new Csr();
  rob = new Rob(ctx);
  lsu = new RealLsu(ctx);
  dtlb_mmu =
      std::make_unique<TlbMmu>(ctx, DTLB_ENTRIES, TlbMmu::Kind::DTLB);
  lsu->dtlb = dtlb_mmu.get();

  out.fire = pre2front.fire;

  pre->wires = this;
  idu->wires = this;
  rename->wires = this;
  dis->wires = this;
  isu->wires = this;
  exu->wires = this;
  rob->wires = this;
  exu->ftq = pre;
  exu->csr = csr;
  rob->ftq = pre;
  csr->wires = this;
  lsu->wires = this;
  idu_br_latch = &idu->br_latch;

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

void BackTop::clear_comb_inputs() { in = Back_in{}; }

// ---------------------------------------------------------------------------------------------
// One back-end cycle is evaluated in dependency order: every stage reads only signals that an
// earlier stage (or registered state) already produced. The call order in BackTop::comb() is that order.
// ---------------------------------------------------------------------------------------------

// Working copies and default outputs of every module.
// Branch/decode results, free resources (rename registers, issue queues, LDQ/STQ) and the wakeups/write-backs completing this cycle.
void BackTop::comb_stage_decode_and_resources() {
  idu->comb_branch();
  pre->comb_accept_front();
  idu->comb_decode();
  csr->comb_interrupt();
  rename->comb_alloc();
  exu->comb_load_wake();
  isu->comb_ready();
  lsu->comb_cal();
  lsu->comb_lsu2dis();
  lsu->comb_lsu2rob();
}

// ROB head -> commit group; committed stores release their STQ entries in the LSU.
void BackTop::comb_stage_commit() {
  rob->comb_ready();
  rob->comb_commit();
  pre->comb_commit_info();
  lsu->comb_stq_commit();
}

// MMIO/DCache responses finish loads and stores and produce write-backs; CSR read data.
void BackTop::comb_stage_memory_responses() {
  dis->comb_alloc();
  lsu->comb_mmio_in();
  lsu->comb_dcache2lsu();
  lsu->comb_lsu2exe();
}

// Functional units execute; their results complete ROB entries.
void BackTop::comb_stage_execute() {
  exu->comb_exec();
  rob->comb_complete();
}

// Issue selection, then LSU address translation (the MMU sits between the LSU's request and response phases).
void BackTop::comb_stage_issue_and_translate() {
  exu->comb_ready();
  isu->comb_issue();
  lsu->comb_exe2lsu();
  lsu->comb_tlb();
  lsu->comb_stlf();
}

// Register reads, next-cycle wakeups, exceptions, rename and dispatch of the instructions leaving decode.
void BackTop::comb_stage_operands_and_dispatch() {

  isu->comb_awake();
  isu->comb_calc_latency_next();
  csr->comb_exception();
  csr->comb_csr_write();
  dis->comb_wake();
  rename->comb_rename();

  dis->comb_dispatch();
}

// Handshakes that move instructions (fire), the LSU's DCache/MMIO requests, queue enqueue, and recovery of ROB/IQs from a mispredict or global flush (flush wins).
void BackTop::comb_stage_fire_and_recover() {
  dis->comb_fire();
  lsu->comb_dis2lsu();
  rename->comb_fire();
  idu->comb_fire();
  // Kill flushed LSU work before driving new DCache/MMIO requests.
  lsu->comb_flush();
  lsu->comb_lsu2dcache();
  lsu->comb_mmio_out();
  lsu->comb_check();
  isu->comb_enq();
  // Rob recovery was split out of comb_fire(); keep the original priority:
  // branch rollback happens before enqueue, and global flush overrides both.
  rob->comb_branch();
  rob->comb_fire();
  rob->comb_flush();
  isu->comb_flush();

  pre->comb_fire();
  exu->comb_pipeline();
  dis->comb_pipeline();
  rename->comb_pipeline();
  csr->comb_interrupt_inject();
}

// Diagnostic mode (CONFIG_BE_IO_CLEAR_AT_COMB_BEGIN): clear the stage IOs to expose hidden dependence on
// previous-cycle combinational values.
// Outputs to the front end / difftest: redirect (mispredict or flush target), commit group.
void BackTop::comb_outputs() {
  // 用于调试
  // 修正pc_next 以及difftest对应的pc_next
  out.flush = rob_bcast.flush;
  if (out.flush) {
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
      out.rf_slot = idu->br_latch.ftq_off;
      out.rf_blk = pre->ftq_blk_id(fi, out.rf_slot);
      out.rf_type = idu->br_latch.br_type;
      out.rf_pc = pre->ftq_slot_pc(fi, out.rf_slot);
      out.rf_is_rvc = pre->ftq_slot_rvc(fi, out.rf_slot);
    }
  } else {
    out.mispred = true;
    if (rob_bcast.mret || rob_bcast.sret) {
      out.redirect_pc = csr2front.epc;
    } else if (rob_bcast.exception) {
      out.redirect_pc = csr2front.trap_pc;
    } else {
      out.redirect_pc = rob_bcast.pc + ilen_of(rob_bcast.is_rvc);
    }
  }

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    out.commit_entry[i] =
        rob_commit.commit_entry[i].uop.to_inst_entry(
            rob_commit.commit_entry[i].valid);
    if (rob_commit.commit_entry[i].valid &&
        !rob_bcast.interrupt) {
      if (tbhook::fs_dirty_at_commit((uint64_t)rob_commit.commit_entry[i].uop->dbg.inst_idx)) csr->fs_dirty_req = true;
      tbhook::on_commit(
          (uint64_t)rob_commit.commit_entry[i].uop->dbg.inst_idx);
    }
    if (out.commit_entry[i].valid && out.flush) {
      // Flush: the next PC of every committing instruction is the redirect target (trap vector, epc, ...)
      out.commit_entry[i].uop.diag_val = out.redirect_pc;
      rob_commit.commit_entry[i].uop.diag_val = out.redirect_pc;
    }
  }
}

void BackTop::comb() {
  // CSR 状态由 SimCpu::cycle() 在进入 front/back 组合逻辑前统一刷新。
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


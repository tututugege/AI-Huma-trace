#include "Csr.h"
#include "Pmu.h"
#include "Idu.h"
#include "RISCV.h"
#include "BackPorts.h"
#include "config.h"
#include "ref.h"
#include "util.h"
#include "InstDecode.h"
#include <cstdint>
#include <cstdlib>

void Idu::init() {
  for (int i = 0; i < MAX_BR_NUM; i++) {
    q_.tag_vec[i] = true;
    q_.br_mask_cp[i] = 0;
  }
  q_.tag_vec[0] = false;  // tag 0 is "no branch"
  q_.now_br_mask = 0;
  q_.br_latch = {};
}

// Decode the instructions of the pre-decode queue and give every branch a tag/mask. A younger instruction depends on all
// older unresolved branches (its br_mask); a branch itself does not depend on its own bit. When the free tags run out
// the group is cut at that branch.
void Idu::decode() {
  const Regs &q = q_;
  bool alloc_valid[DECODE_WIDTH];
  int tags = 0;
  for (int i = 0; i < MAX_BR_NUM && tags < max_br_per_cycle; i++)
    if (q.tag_vec[i]) {
      alloc_tag[tags] = i;
      alloc_valid[tags++] = true;
    }
  for (int i = tags; i < DECODE_WIDTH; i++) {
    alloc_tag[i] = 0;
    alloc_valid[i] = false;
  }

  for (int i = 0; i < DECODE_WIDTH; i++) {
    out.dec2ren.valid[i] = false;
    out.dec2ren.uop[i] = {};
  }
  for (int i = 0; i < DECODE_WIDTH; i++) {
    const InstructionBufferEntry &e = in.pre_issue->entries[i];
    if (!e.valid) continue;
    out.dec2ren.valid[i] = true;
    Uop &u = out.dec2ren.uop[i];
    u = {};
    u.id = e.uop_id;
    if (e.page_fault_inst) {  // a faulting fetch becomes a NOP that carries the fault
      u.diag_val = e.inst;
      u.page_fault_inst = true;
      u->type = encode_inst_type(NOP);
      u->src1_en = u->src2_en = false;
      u.dest_en = false;
      u->dbg.instruction = e.inst;
    } else {
      decode(u, e.inst);
    }
    u->dbg.pc = e.pc;
    u->is_rvc = e.is_rvc;
    u->ftq_idx = e.ftq_idx;
    u->ftq_offset = e.ftq_offset;
    u.ftq_is_last = e.ftq_is_last;
  }

  // branches resolved last cycle (q.br_latch) are already gone from the mask the new instructions see
  wire<BR_MASK_WIDTH> running = q.now_br_mask & ~q.br_latch.clear_mask;
  int br_num = 0, i = 0;
  bool stall = false;
  for (; i < DECODE_WIDTH; i++) {
    Uop &u = out.dec2ren.uop[i];
    u->br_id = 0;
    u.br_mask = running;
    if (!out.dec2ren.valid[i] || !is_branch(u->type)) continue;
    if (!alloc_valid[br_num]) { stall = true; break; }
    u->br_id = alloc_tag[br_num];
    running |= wire<BR_MASK_WIDTH>(1) << alloc_tag[br_num];
    br_num++;
  }
  g_pmu_back.idu_no_brtag = stall;
  if (stall)
    for (; i < DECODE_WIDTH; i++) {
      out.dec2ren.valid[i] = false;
      out.dec2ren.uop[i]->br_id = 0;
      out.dec2ren.uop[i].br_mask = 0;
    }
}

// Broadcast last cycle's branch resolution (mispredict, tag, clear bits) to everyone; only comb_fire updates local state.
void Idu::branch() {
  const Regs &q = q_;
  DecBroadcastIO &b = out.dec_bcast;
  b.mispred = q.br_latch.mispred;
  b.br_id = q.br_latch.mispred ? q.br_latch.br_id : 0;
  b.br_mask = q.br_latch.mispred ? wire<BR_MASK_WIDTH>(1) << q.br_latch.br_id : 0;
  if (q.br_latch.mispred) b.redirect_rob_idx = q.br_latch.redirect_rob_idx;
  b.clear_mask = q.br_latch.clear_mask;  // includes the mispredicted branch's own bit; receivers flush first, then clear
}

// The consume handshake with the pre-decode queue: every decoded slot fires when rename takes the group (nothing in a
// flush or mispredict cycle).
void Idu::comb_fire() {
  for (int i = 0; i < DECODE_WIDTH; i++) out.idu_consume.fire[i] = false;
  if (in.rob_bcast->flush || q_.br_latch.mispred) return;
  for (int i = 0; i < DECODE_WIDTH; i++) out.idu_consume.fire[i] = out.dec2ren.valid[i] && in.ren2dec->ready;
}

// The clock edge: branch tag bookkeeping. Flush resets everything; resolved branches return their tags; a mispredict
// frees the tags younger than the branch (and allocates nothing); otherwise fired branches take their tag and
// checkpoint the mask.
void Idu::seq() {
  const Regs q = q_;   // the registers of this cycle
  Regs &d = q_;        // updated in place
  d.br_latch = in.rob_bcast->flush ? ExuIdIO{} : (*in.exu2id);
  if (in.rob_bcast->flush) {
    for (int i = 1; i < MAX_BR_NUM; i++) d.tag_vec[i] = true;
    d.tag_vec[0] = false;
    d.now_br_mask = 0;
    return;
  }

  const wire<BR_MASK_WIDTH> clear = q.br_latch.clear_mask;
  for (int i = 1; i < MAX_BR_NUM; i++)
    if ((clear >> i) & 1) {
      d.tag_vec[i] = true;
      d.now_br_mask &= ~(wire<BR_MASK_WIDTH>(1) << i);
    }
  if (clear != 0)  // checkpoints must not keep bits of resolved branches (tags get reused)
    for (int i = 1; i < MAX_BR_NUM; i++) d.br_mask_cp[i] &= ~clear;

  if (q.br_latch.mispred) {
    const wire<BR_MASK_WIDTH> younger = q.now_br_mask & ~q.br_mask_cp[q.br_latch.br_id];
    d.now_br_mask &= ~younger;
    for (int i = 1; i < MAX_BR_NUM; i++)
      if ((younger >> i) & 1) d.tag_vec[i] = true;
    return;
  }

  int br_num = 0;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (out.idu_consume.fire[i] && is_branch(out.dec2ren.uop[i]->type)) {
      const auto tag = alloc_tag[br_num++];
      d.tag_vec[tag] = false;
      d.now_br_mask |= wire<BR_MASK_WIDTH>(1) << tag;
      d.br_mask_cp[tag] = d.now_br_mask;
    }
  }
}



// One instruction into a Uop: the ISA view from the shared decoder (decode_inst, bridge/InstDecode.h), then the decode
// stage's own rules (the top-down-analysis flags, the AMO destination, x0).
void Idu::decode(Uop &uop, uint32_t inst) {
  const DecodedInst r = decode_inst(inst);
  uop->dbg.instruction = inst;
  uop.diag_val = inst;
  uop->type = encode_inst_type(r.type);
  uop.illegal_inst = r.illegal;
  uop.page_fault_inst = false;
  uop->dest_areg = r.rd;
  uop->src1_areg = r.rs1;
  uop->src2_areg = r.rs2;
  uop->src3_areg = r.rs3;
  uop.dest_en = r.dest_en;
  uop->src1_en = r.src1_en;
  uop->src2_en = r.src2_en;
  uop->src3_en = r.src3_en;
  uop->dest_fp = r.dest_fp;
  uop->src1_fp = r.src1_fp;
  uop->src2_fp = r.src2_fp;
  uop->src3_fp = r.src3_fp;
  uop->src1_is_pc = r.src1_is_pc;
  uop->src2_is_imm = r.src2_is_imm;
  uop->is_word = r.is_word;
  uop->is_atomic = r.is_atomic;
  uop->fp_cls = r.fp_cls;
  uop->func3 = r.f3;
  uop->func7 = r.f7;
  uop->csr_idx = r.csr;
  uop->imm = r.imm;
  uop.tma.is_ret = uop.tma.mem_commit_is_load = uop.tma.mem_commit_is_store = false;
  static uint64_t global_inst_idx = 0;   // the instruction's identity in the trace bridge
  uop->dbg.inst_idx = global_inst_idx++;

  const InstType type = decode_inst_type(uop->type);
  const uint32_t amo = uop->func7 >> 2;
  uop.tma.is_ret = type == JALR && uop->src1_areg == 1 && uop->dest_areg == 0 && uop->imm == 0;
  uop.tma.mem_commit_is_load = type == LOAD || (type == AMO && amo != AmoOp::SC);
  uop.tma.mem_commit_is_store = type == STORE || (type == AMO && amo != AmoOp::LR);
  // an AMO that discards its result still writes through a dedicated register (it keeps the load's destination)
  if (type == AMO && uop->dest_areg == 0 && amo != AmoOp::LR && amo != AmoOp::SC) uop->dest_areg = 32;
  if (uop->dest_areg == 0 && !uop->dest_fp) uop.dest_en = false;   // x0 is not written (f0 is a register)
}

// branch resolution (mispredict broadcast), then decode
void Idu::comb_decode() {
  branch();
  decode();
}

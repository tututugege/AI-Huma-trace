#include "Csr.h"
#include "Pmu.h"
#include "Idu.h"
#include "RISCV.h"
#include "BackPorts.h"
#include "config.h"
#include "DseCap.h"
#include "ref.h"
#include "util.h"
#include "InstDecode.h"
#include <cstdint>
#include <cstdlib>

void Idu::init() {
  q_.br_latch = {};
}

// Decode the instructions of the pre-decode queue. Branch recovery uses circular 9-bit ROB-pointer age comparison
// ({rob_flag, rob_idx}), so IDU allocates no branch tags or masks.
void Idu::decode() {
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
      u.diag_val = 0;
      u.page_fault_inst = true;
      u.pf_inst_hi = e.pf_inst_hi;
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
  g_pmu_back.idu_no_brtag = false;
}

// Broadcast last cycle's branch resolution (mispredict + 9-bit ROB pointer) to everyone; only comb_fire updates local state.
void Idu::branch() {
  const Regs &q = q_;
  DecBroadcastIO &b = out.dec_bcast;
  b.mispred = q.br_latch.mispred;
  b.redirect_rob_idx = q.br_latch.mispred ? q.br_latch.redirect_rob_idx : 0;
  b.redirect_rob_flag = q.br_latch.mispred ? q.br_latch.redirect_rob_flag : 0;
}

// The consume handshake with the pre-decode queue: every decoded slot fires when rename takes the group (nothing in a
// flush or mispredict cycle).
void Idu::comb_fire() {
  for (int i = 0; i < DECODE_WIDTH; i++) out.idu_consume.fire[i] = false;
  if (in.rob_bcast->flush || q_.br_latch.mispred) return;
  for (int i = 0; i < DECODE_WIDTH; i++) out.idu_consume.fire[i] = out.dec2ren.valid[i] && in.ren2dec->ready;
}

void Idu::seq() {
  q_.br_latch = in.rob_bcast->flush ? ExuIdIO{} : (*in.exu2id);
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
  uop->tma.is_ret = uop->tma.mem_commit_is_load = uop->tma.mem_commit_is_store = false;
  static uint64_t global_inst_idx = 0;   // the instruction's identity in the trace bridge
  uop->dbg.inst_idx = global_inst_idx++;

  const InstType type = decode_inst_type(uop->type);
  const uint32_t amo = uop->func7 >> 2;
  uop->tma.is_ret = type == JALR && uop->src1_areg == 1 && uop->dest_areg == 0 && uop->imm == 0;
  uop->tma.mem_commit_is_load = type == LOAD || (type == AMO && amo != AmoOp::SC);
  uop->tma.mem_commit_is_store = type == STORE || (type == AMO && amo != AmoOp::LR);
  // Canonicalize `add rd, x0, rs2` (including every expanded `c.mv rd, rs2`) into `addi rd, rs2, 0` so move elimination sees RVC moves.
  if (type == ADD && !uop->src2_is_imm && !uop->is_word && uop->func3 == 0 && uop->func7 == 0 &&
      uop->src1_en && uop->src2_en && uop->src1_areg == 0 && uop->src2_areg != 0 &&
      uop.dest_en && uop->dest_areg != 0) {
    uop->src1_areg = uop->src2_areg;
    uop->src2_areg = 0;
    uop->src2_en = false;
    uop->src2_is_imm = true;
    uop->imm = 0;
  }
  // an AMO that discards its result still writes through a dedicated register (it keeps the load's destination)
  if (type == AMO && uop->dest_areg == 0 && amo != AmoOp::LR && amo != AmoOp::SC) uop->dest_areg = 32;
  if (uop->dest_areg == 0 && !uop->dest_fp) uop.dest_en = false;   // x0 is not written (f0 is a register)
}

// branch resolution (mispredict broadcast), then decode
void Idu::comb_decode() {
  branch();
  decode();
}

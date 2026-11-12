#include "BeginCheck.h"
#include "Csr.h"
#include "Pmu.h"
#include "Idu.h"
#include "RISCV.h"
#include "Wires.h"
#include "config.h"
#include "ref.h"
#include "util.h"
#include <cstdint>
#include <cstdlib>

// Branch tags picked in comb_decode and committed in comb_fire when the instruction actually moves on.
static wire<BR_TAG_WIDTH> alloc_tag[DECODE_WIDTH];

void Idu::init() {
  for (int i = 0; i < MAX_BR_NUM; i++) {
    tag_vec[i] = tag_vec_1[i] = true;
    br_mask_cp[i] = br_mask_cp_1[i] = 0;
  }
  tag_vec[0] = tag_vec_1[0] = false;  // tag 0 is "no branch"
  now_br_mask = now_br_mask_1 = 0;
  br_latch = br_latch_1 = {};
}

// The working copies already equal the registered state (seq() copies them back).
void Idu::comb_begin() {
  BEGIN_EQ(tag_vec_1, tag_vec);
  BEGIN_EQ(br_mask_cp_1, br_mask_cp);
  BEGIN_EQ(now_br_mask_1, now_br_mask);
  BEGIN_EQ(br_latch_1, br_latch);
}

// Decode the instructions of the pre-decode queue and give every branch a tag/mask. A younger instruction depends on all
// older unresolved branches (its br_mask); a branch itself does not depend on its own bit. When the free tags run out
// the group is cut at that branch.
void Idu::comb_decode() {
  Wires &w = *wires;
  bool alloc_valid[DECODE_WIDTH];
  int tags = 0;
  for (int i = 0; i < MAX_BR_NUM && tags < max_br_per_cycle; i++)
    if (tag_vec[i]) {
      alloc_tag[tags] = i;
      alloc_valid[tags++] = true;
    }
  for (int i = tags; i < DECODE_WIDTH; i++) {
    alloc_tag[i] = 0;
    alloc_valid[i] = false;
  }

  for (int i = 0; i < DECODE_WIDTH; i++) {
    w.dec2ren.valid[i] = false;
    w.dec2ren.uop[i] = {};
  }
  for (int i = 0; i < DECODE_WIDTH; i++) {
    const InstructionBufferEntry &e = w.pre_issue.entries[i];
    if (!e.valid) continue;
    w.dec2ren.valid[i] = true;
    Uop &u = w.dec2ren.uop[i];
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
    u->ftq_idx = e.ftq_idx;
    u->ftq_offset = e.ftq_offset;
    u.ftq_is_last = e.ftq_is_last;
  }

  // branches resolved last cycle (br_latch) are already gone from the mask the new instructions see
  wire<BR_MASK_WIDTH> running = now_br_mask & ~br_latch.clear_mask;
  int br_num = 0, i = 0;
  bool stall = false;
  for (; i < DECODE_WIDTH; i++) {
    Uop &u = w.dec2ren.uop[i];
    u->br_id = 0;
    u.br_mask = running;
    if (!w.dec2ren.valid[i] || !is_branch(u->type)) continue;
    if (!alloc_valid[br_num]) { stall = true; break; }
    u->br_id = alloc_tag[br_num];
    running |= wire<BR_MASK_WIDTH>(1) << alloc_tag[br_num];
    br_num++;
  }
  g_pmu_back.idu_no_brtag = stall;
  if (stall)
    for (; i < DECODE_WIDTH; i++) {
      w.dec2ren.valid[i] = false;
      w.dec2ren.uop[i]->br_id = 0;
      w.dec2ren.uop[i].br_mask = 0;
    }
}

// Broadcast last cycle's branch resolution (mispredict, tag, clear bits) to everyone; only comb_fire updates local state.
void Idu::comb_branch() {
  DecBroadcastIO &b = wires->dec_bcast;
  b.mispred = br_latch.mispred;
  b.br_id = br_latch.mispred ? br_latch.br_id : 0;
  b.br_mask = br_latch.mispred ? wire<BR_MASK_WIDTH>(1) << br_latch.br_id : 0;
  if (br_latch.mispred) b.redirect_rob_idx = br_latch.redirect_rob_idx;
  b.clear_mask = br_latch.clear_mask;  // includes the mispredicted branch's own bit; receivers flush first, then clear
}

// Branch tag bookkeeping and the consume handshake with the pre-decode queue.
// Flush resets everything; resolved branches return their tags; a mispredict frees the tags younger than the branch
// (and allocates nothing this cycle); otherwise fired branches take their tag and checkpoint the mask.
void Idu::comb_fire() {
  Wires &w = *wires;
  for (int i = 0; i < DECODE_WIDTH; i++) w.idu_consume.fire[i] = false;

  br_latch_1 = w.rob_bcast.flush ? ExuIdIO{} : w.exu2id;
  if (w.rob_bcast.flush) {
    for (int i = 1; i < MAX_BR_NUM; i++) tag_vec_1[i] = true;
    tag_vec_1[0] = false;
    now_br_mask_1 = 0;
    return;
  }

  const wire<BR_MASK_WIDTH> clear = br_latch.clear_mask;
  for (int i = 1; i < MAX_BR_NUM; i++)
    if ((clear >> i) & 1) {
      tag_vec_1[i] = true;
      now_br_mask_1 &= ~(wire<BR_MASK_WIDTH>(1) << i);
    }
  if (clear != 0)  // checkpoints must not keep bits of resolved branches (tags get reused)
    for (int i = 1; i < MAX_BR_NUM; i++) br_mask_cp_1[i] &= ~clear;

  if (br_latch.mispred) {
    const wire<BR_MASK_WIDTH> younger = now_br_mask & ~br_mask_cp[br_latch.br_id];
    now_br_mask_1 &= ~younger;
    for (int i = 1; i < MAX_BR_NUM; i++)
      if ((younger >> i) & 1) tag_vec_1[i] = true;
    return;
  }

  int br_num = 0;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    const bool fire = w.dec2ren.valid[i] && w.ren2dec.ready;  // every fired slot leaves the pre-decode queue
    w.idu_consume.fire[i] = fire;
    if (fire && is_branch(w.dec2ren.uop[i]->type)) {
      const auto tag = alloc_tag[br_num++];
      tag_vec_1[tag] = false;
      now_br_mask_1 |= wire<BR_MASK_WIDTH>(1) << tag;
      br_mask_cp_1[tag] = now_br_mask_1;
    }
  }
}

void Idu::seq() {
  now_br_mask = now_br_mask_1;
  for (int i = 0; i < MAX_BR_NUM; i++) {
    tag_vec[i] = tag_vec_1[i];
    br_mask_cp[i] = br_mask_cp_1[i];
  }
  br_latch = br_latch_1;
}

namespace {
// CSRs the core implements; accessing any other one raises an illegal-instruction exception (like the function model:
// OpenSBI probes menvcfg / mcounteren / stimecmp / ... by trapping, and must see them absent). The floating-point CSRs
// are executed by the function model but have no timing effect here, so they stay NOPs.
bool is_known_csr(uint32_t c) {
  switch (c) {
  case number_mtvec: case number_mepc: case number_mcause: case number_mie: case number_mip: case number_mtval:
  case number_mscratch: case number_mstatus: case number_mideleg: case number_medeleg: case number_sepc:
  case number_stvec: case number_scause: case number_sscratch: case number_stval: case number_sstatus:
  case number_sie: case number_sip: case number_satp: case number_mhartid: case number_misa:
    return true;
  default:
    return false;
  }
}
}  // namespace

// RISC-V decode into a Uop: operand enables, instruction class, immediate, and the top-down-analysis flags.
void Idu::decode(Uop &uop, uint32_t inst) {
  const uint32_t opcode = BITS(inst, 6, 0), f3 = BITS(inst, 14, 12), f7 = BITS(inst, 31, 25);
  const uint32_t rd = BITS(inst, 11, 7), ra = BITS(inst, 19, 15), rb = BITS(inst, 24, 20), csr = inst >> 20;

  uop->dbg.instruction = inst;
  uop.diag_val = inst;
  uop->dest_areg = rd;
  uop->src1_areg = ra;
  uop->src2_areg = rb;
  uop->src1_is_pc = false;
  uop->src2_is_imm = true;
  uop->func3 = f3;
  uop->func7 = f7;
  uop->is_word = false;
  uop->dest_fp = uop->src1_fp = uop->src2_fp = uop->src3_fp = false;
  uop->src3_en = false;
  uop->src3_areg = 0;
  uop->fp_cls = FP_NONE;
  uop->csr_idx = csr;
  uop.page_fault_inst = false;
  uop.illegal_inst = false;
  uop->type = encode_inst_type(NOP);
  uop.tma.is_cache_miss = uop.tma.is_ret = uop.tma.mem_commit_is_load = uop.tma.mem_commit_is_store = false;
  static uint64_t global_inst_idx = 0;
  uop->dbg.inst_idx = global_inst_idx++;

  // destination written / source 1 read / source 2 read, and the instruction class
  auto set = [&](bool dest, bool s1, bool s2, InstType type) {
    uop.dest_en = dest;
    uop->src1_en = s1;
    uop->src2_en = s2;
    uop->type = encode_inst_type(type);
  };

  switch (opcode) {
  case number_0_opcode_lui:  // reads x0
    set(true, true, false, ADD);
    uop->src1_areg = 0;
    uop->func3 = 0;
    uop->imm = immU(inst);
    break;
  case number_1_opcode_auipc:
    set(true, false, false, ADD);
    uop->src1_is_pc = true;
    uop->func3 = 0;
    uop->imm = immU(inst);
    break;
  case number_2_opcode_jal:
    set(true, false, false, JAL);
    uop->src1_is_pc = true;
    uop->func3 = 0;
    uop->imm = immJ(inst);
    break;
  case number_3_opcode_jalr:
    set(true, true, false, JALR);
    uop->src1_is_pc = true;
    uop->func3 = 0;
    uop->imm = immI(inst);
    break;
  case number_4_opcode_beq:  // beq, bne, blt, bge, bltu, bgeu
    set(false, true, true, BR);
    uop->imm = immB(inst);
    break;
  case number_5_opcode_lb:  // lb, lh, lw, lbu, lhu
    set(true, true, false, LOAD);
    uop->imm = immI(inst);
    break;
  case number_6_opcode_sb:  // sb, sh, sw
    set(false, true, true, STORE);
    uop->imm = immS(inst);
    break;
  case number_7_opcode_addi:  // register-immediate ALU ops, including the Zbb/Zbs immediate forms
    set(true, true, false, ADD);
    uop->imm = immI(inst);
    break;
  case number_8_opcode_add:  // register-register ALU ops (Zba/Zbb/Zbc/Zbs too); funct7 == 1 is M: mul/div
    set(true, true, true, f7 == 1 ? ((f3 & 0b100) ? DIV : MUL) : ADD);
    uop->src2_is_imm = false;
    break;
#if XLEN == 64
  case 0b0011011:  // OP-IMM-32: addiw, slliw, srliw, sraiw, slli.uw, clzw, ctzw, cpopw, roriw (rv64)
    set(true, true, false, ADD);
    uop->imm = immI(inst);
    uop->is_word = true;
    break;
  case 0b0111011:  // OP-32: addw, subw, sllw, srlw, sraw, add.uw, shNadd.uw, rolw, rorw, zext.h; funct7 == 1 is mulw/divw/divuw/remw/remuw
    set(true, true, true, f7 == 1 ? ((f3 & 0b100) ? DIV : MUL) : ADD);
    uop->src2_is_imm = false;
    uop->is_word = true;
    break;
#endif
  case number_9_opcode_fence:  // funct3 001 is fence.i
    set(false, false, false, f3 == 0b001 ? FENCE_I : FENCE);
    break;
  case number_10_opcode_ecall: {  // csrr*, ecall, ebreak, mret, sret, wfi, sfence.vma
    uop->src2_is_imm = (f3 & 0b100) && ((f3 & 0b001) || (f3 & 0b010));
    if ((f3 & 0b001) || (f3 & 0b010)) {
      if (!is_known_csr(csr)) {
        set(false, false, false, NOP);
        if (csr < 0x001 || csr > 0x003) uop.illegal_inst = true;   // not fflags / frm / fcsr
      } else {
        set(true, true, !uop->src2_is_imm, CSR);
        uop->imm = ra;  // zimm for the immediate forms
      }
    } else if (inst == INST_ECALL) set(false, false, false, ECALL);
    else if (inst == INST_EBREAK) set(false, false, false, EBREAK);
    else if (inst == INST_MRET) set(false, false, false, MRET);
    else if (inst == INST_WFI) set(false, false, false, WFI);
    else if (inst == INST_SRET) set(false, false, false, SRET);
    else if (f7 == 0b0001001 && f3 == 0 && rd == 0) set(false, true, true, SFENCE_VMA);
    else set(false, false, false, NOP);
    break;
  }
  case number_11_opcode_lrw:  // atomics; LR has no second source
    set(true, true, (f7 >> 2) != AmoOp::LR, AMO);
    uop->imm = 0;
    uop->is_atomic = true;
    break;
#if FP
  // F/D (rv64): the function model decides what traps (reserved rounding modes, mstatus.FS = Off): an executed FP uop whose fact
  // says "trapped with illegal instruction" becomes the exception in Fu::compute.  Statically reserved encodings trap here.
  case 0x07:  // flw / fld
    if (f3 != 2 && f3 != 3) { set(false, false, false, NOP); uop.illegal_inst = true; break; }
    set(true, true, false, LOAD);
    uop->dest_fp = true;
    uop->imm = immI(inst);
    break;
  case 0x27:  // fsw / fsd
    if (f3 != 2 && f3 != 3) { set(false, false, false, NOP); uop.illegal_inst = true; break; }
    set(false, true, true, STORE);
    uop->src2_fp = true;
    uop->imm = immS(inst);
    break;
  case 0x43: case 0x47: case 0x4b: case 0x4f:  // fmadd / fmsub / fnmsub / fnmadd: three FP sources
    if ((f7 & 3) > 1) { set(false, false, false, NOP); uop.illegal_inst = true; break; }
    set(true, true, true, FPOP);
    uop->src2_is_imm = false;
    uop->dest_fp = uop->src1_fp = uop->src2_fp = uop->src3_fp = true;
    uop->src3_en = true;
    uop->src3_areg = BITS(inst, 31, 27);
    uop->fp_cls = FP_C_FMA;
    break;
  case 0x53: {  // OP-FP: fmt (funct7[1:0]) 0 = S, 1 = D; the class comes from funct5
    const uint32_t f5 = f7 >> 2;
    if ((f7 & 3) > 1) { set(false, false, false, NOP); uop.illegal_inst = true; break; }
    uop->src2_is_imm = false;
    switch (f5) {
    case 0: case 1: case 2:  // fadd fsub fmul
      set(true, true, true, FPOP); uop->dest_fp = uop->src1_fp = uop->src2_fp = true; uop->fp_cls = FP_C_FMA; break;
    case 3:                  // fdiv
      set(true, true, true, FPOP); uop->dest_fp = uop->src1_fp = uop->src2_fp = true; uop->fp_cls = FP_C_FDIV; break;
    case 0x0b:               // fsqrt
      set(true, true, false, FPOP); uop->dest_fp = uop->src1_fp = true; uop->fp_cls = FP_C_FDIV; break;
    case 4: case 5:          // fsgnj*, fmin / fmax
      set(true, true, true, FPOP); uop->dest_fp = uop->src1_fp = uop->src2_fp = true; uop->fp_cls = FP_C_MISC; break;
    case 8:                  // fcvt.s.d / fcvt.d.s
      set(true, true, false, FPOP); uop->dest_fp = uop->src1_fp = true; uop->fp_cls = FP_C_MISC; break;
    case 0x14:               // feq / flt / fle: integer result
      set(true, true, true, FPOP); uop->src1_fp = uop->src2_fp = true; uop->fp_cls = FP_C_MISC; break;
    case 0x18: case 0x1c:    // fcvt.{w,wu,l,lu}.fmt, fmv.x.w / fmv.x.d / fclass: integer result
      set(true, true, false, FPOP); uop->src1_fp = true; uop->fp_cls = FP_C_MISC; break;
    case 0x1a: case 0x1e:    // fcvt.fmt.{w,wu,l,lu}, fmv.w.x / fmv.d.x: integer source
      set(true, true, false, FPOP); uop->dest_fp = true; uop->fp_cls = FP_C_I2F; break;
    default:
      set(false, false, false, NOP); uop.illegal_inst = true; break;
    }
    break;
  }
#endif
  default:
    set(false, false, false, NOP);
    uop.illegal_inst = true;
    break;
  }

  const InstType type = decode_inst_type(uop->type);
  const uint32_t amo = uop->func7 >> 2;
  uop.tma.is_ret = type == JALR && uop->src1_areg == 1 && uop->dest_areg == 0 && uop->imm == 0;
  uop.tma.mem_commit_is_load = type == LOAD || (type == AMO && amo != AmoOp::SC);
  uop.tma.mem_commit_is_store = type == STORE || (type == AMO && amo != AmoOp::LR);
  // an AMO that discards its result still writes through a dedicated register (it keeps the load's destination)
  if (type == AMO && uop->dest_areg == 0 && amo != AmoOp::LR && amo != AmoOp::SC) uop->dest_areg = 32;
  if (uop->dest_areg == 0 && !uop->dest_fp) uop.dest_en = false;   // x0 is not written (f0 is a register)
}

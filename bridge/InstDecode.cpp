// One pass over a RISC-V instruction word (decode_inst, InstDecode.h). The ISA view moved here from the decode stage
// (Idu) so that the decode stage and the trace bridge read one decoder.
#include "InstDecode.h"
#include "Csr.h"
#include "RISCV.h"
#include "ref.h"

namespace {
// CSRs the core implements; accessing any other one raises an illegal-instruction exception (like the function model:
// OpenSBI probes menvcfg / mcounteren / stimecmp / ... by trapping, and must see them absent). F/D: fflags / frm / fcsr
// are real CSR instructions (serialised like the others: the RTL keeps fflags in the CSR file, accrued at commit).
bool is_known_csr(uint32_t c) {
  switch (c) {
  case number_mtvec: case number_mepc: case number_mcause: case number_mie: case number_mip: case number_mtval:
  case number_mscratch: case number_mstatus: case number_mideleg: case number_medeleg: case number_sepc:
  case number_stvec: case number_scause: case number_sscratch: case number_stval: case number_sstatus:
  case number_sie: case number_sip: case number_satp: case number_mhartid: case number_misa:
    return true;
  case number_fflags: case number_frm: case number_fcsr:
    return true;
  default:
    return false;
  }
}
}  // namespace

DecodedInst decode_inst(uint32_t inst) {
  const uint32_t opcode = BITS(inst, 6, 0), f3 = BITS(inst, 14, 12), f7 = BITS(inst, 31, 25);
  const uint32_t rd = BITS(inst, 11, 7), ra = BITS(inst, 19, 15), rb = BITS(inst, 24, 20), csr = inst >> 20;
  DecodedInst r;
  r.rd = rd;
  r.rs1 = ra;
  r.rs2 = rb;
  r.f3 = f3;
  r.f7 = f7;
  r.csr = csr;

  // destination written / source 1 read / source 2 read, and the instruction class
  auto set = [&](bool dest, bool s1, bool s2, InstType type) {
    r.dest_en = dest;
    r.src1_en = s1;
    r.src2_en = s2;
    r.type = type;
  };

  switch (opcode) {
  case number_0_opcode_lui:  // reads x0
    set(true, true, false, ADD);
    r.rs1 = 0;
    r.f3 = 0;
    r.imm = immU(inst);
    break;
  case number_1_opcode_auipc:
    set(true, false, false, ADD);
    r.src1_is_pc = true;
    r.f3 = 0;
    r.imm = immU(inst);
    break;
  case number_2_opcode_jal:
    set(true, false, false, JAL);
    r.src1_is_pc = true;
    r.f3 = 0;
    r.imm = immJ(inst);
    break;
  case number_3_opcode_jalr:
    set(true, true, false, JALR);
    r.src1_is_pc = true;
    r.f3 = 0;
    r.imm = immI(inst);
    break;
  case number_4_opcode_beq:  // beq, bne, blt, bge, bltu, bgeu
    set(false, true, true, BR);
    r.imm = immB(inst);
    break;
  case number_5_opcode_lb:  // lb, lh, lw, lbu, lhu
    set(true, true, false, LOAD);
    r.imm = immI(inst);
    break;
  case number_6_opcode_sb:  // sb, sh, sw
    set(false, true, true, STORE);
    r.imm = immS(inst);
    break;
  case number_7_opcode_addi:  // register-immediate ALU ops, including the Zbb/Zbs immediate forms
    set(true, true, false, ADD);
    r.imm = immI(inst);
    break;
  case number_8_opcode_add:  // register-register ALU ops (Zba/Zbb/Zbc/Zbs too); funct7 == 1 is M: mul/div
    set(true, true, true, f7 == 1 ? ((f3 & 0b100) ? DIV : MUL) : ADD);
    r.src2_is_imm = false;
    break;
  case 0b0011011:  // OP-IMM-32: addiw, slliw, srliw, sraiw, slli.uw, clzw, ctzw, cpopw, roriw (rv64)
    set(true, true, false, ADD);
    r.imm = immI(inst);
    r.is_word = true;
    break;
  case 0b0111011:  // OP-32: addw, subw, sllw, srlw, sraw, add.uw, shNadd.uw, rolw, rorw, zext.h; funct7 == 1 is mulw/divw/divuw/remw/remuw
    set(true, true, true, f7 == 1 ? ((f3 & 0b100) ? DIV : MUL) : ADD);
    r.src2_is_imm = false;
    r.is_word = true;
    break;
  case number_9_opcode_fence:  // funct3 001 is fence.i
    set(false, false, false, f3 == 0b001 ? FENCE_I : FENCE);
    break;
  case number_10_opcode_ecall: {  // csrr*, ecall, ebreak, mret, sret, wfi, sfence.vma
    r.src2_is_imm = (f3 & 0b100) && ((f3 & 0b001) || (f3 & 0b010));
    if ((f3 & 0b001) || (f3 & 0b010)) {
      if (!is_known_csr(csr)) {
        set(false, false, false, NOP);
        r.illegal = true;
      } else {
        set(true, true, !r.src2_is_imm, CSR);
        r.imm = ra;  // zimm for the immediate forms
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
    r.imm = 0;
    r.is_atomic = true;
    break;
  // F/D (rv64): the function model decides what traps (reserved rounding modes, mstatus.FS = Off): an executed FP instruction whose fact
  // says "trapped with illegal instruction" becomes the exception in Fu::compute.  Statically reserved encodings trap here.
  case 0x07:  // flw / fld
    if (f3 != 2 && f3 != 3) { set(false, false, false, NOP); r.illegal = true; break; }
    set(true, true, false, LOAD);
    r.dest_fp = true;
    r.imm = immI(inst);
    break;
  case 0x27:  // fsw / fsd
    if (f3 != 2 && f3 != 3) { set(false, false, false, NOP); r.illegal = true; break; }
    set(false, true, true, STORE);
    r.src2_fp = true;
    r.imm = immS(inst);
    break;
  case 0x43: case 0x47: case 0x4b: case 0x4f:  // fmadd / fmsub / fnmsub / fnmadd: three FP sources
    if ((f7 & 3) > 1) { set(false, false, false, NOP); r.illegal = true; break; }
    set(true, true, true, FPOP);
    r.src2_is_imm = false;
    r.dest_fp = r.src1_fp = r.src2_fp = r.src3_fp = true;
    r.src3_en = true;
    r.rs3 = BITS(inst, 31, 27);
    r.fp_cls = FP_C_FMA;
    break;
  case 0x53: {  // OP-FP: fmt (funct7[1:0]) 0 = S, 1 = D; the class comes from funct5
    const uint32_t f5 = f7 >> 2;
    if ((f7 & 3) > 1) { set(false, false, false, NOP); r.illegal = true; break; }
    r.src2_is_imm = false;
    switch (f5) {
    case 0: case 1: case 2:  // fadd fsub fmul
      set(true, true, true, FPOP); r.dest_fp = r.src1_fp = r.src2_fp = true; r.fp_cls = FP_C_FMA; break;
    case 3:                  // fdiv
      set(true, true, true, FPOP); r.dest_fp = r.src1_fp = r.src2_fp = true; r.fp_cls = FP_C_FDIV; break;
    case 0x0b:               // fsqrt
      set(true, true, false, FPOP); r.dest_fp = r.src1_fp = true; r.fp_cls = FP_C_FDIV; break;
    case 4: case 5:          // fsgnj*, fmin / fmax
      set(true, true, true, FPOP); r.dest_fp = r.src1_fp = r.src2_fp = true; r.fp_cls = FP_C_MISC; break;
    case 8:                  // fcvt.s.d / fcvt.d.s
      set(true, true, false, FPOP); r.dest_fp = r.src1_fp = true; r.fp_cls = FP_C_MISC; break;
    case 0x14:               // feq / flt / fle: integer result
      set(true, true, true, FPOP); r.src1_fp = r.src2_fp = true; r.fp_cls = FP_C_MISC; break;
    case 0x18: case 0x1c:    // fcvt.{w,wu,l,lu}.fmt, fmv.x.w / fmv.x.d / fclass: integer result
      set(true, true, false, FPOP); r.src1_fp = true; r.fp_cls = FP_C_MISC; break;
    case 0x1a: case 0x1e:    // fcvt.fmt.{w,wu,l,lu}, fmv.w.x / fmv.d.x: integer source
      set(true, true, false, FPOP); r.dest_fp = true; r.fp_cls = FP_C_I2F; break;
    default:
      set(false, false, false, NOP); r.illegal = true; break;
    }
    break;
  }
  default:
    set(false, false, false, NOP);
    r.illegal = true;
    break;
  }


  // the function model's view (the trace bridge's undo log and memory facts)
  const uint32_t f5 = inst >> 27;
  r.writes_fpr = opcode == 0x07 || opcode == 0x43 || opcode == 0x47 || opcode == 0x4b || opcode == 0x4f ||
                 (opcode == 0x53 && f5 != 0x14 && f5 != 0x18 && f5 != 0x1c);
  r.touches_fcsr = opcode == 0x53 || opcode == 0x43 || opcode == 0x47 || opcode == 0x4b || opcode == 0x4f || opcode == 0x73;
  r.writes_xreg = opcode != 0x63 && opcode != 0x23 && opcode != 0x27 && opcode != 0x0f && !r.writes_fpr;
  const bool is_amo = opcode == 0x2f;
  r.is_lr = is_amo && BITS(inst, 31, 27) == 2;
  r.mem_load = opcode == 0x03 || opcode == 0x07 || r.is_lr;
  r.mem_store = !r.mem_load && (opcode == 0x23 || opcode == 0x27 || is_amo);
  return r;
}

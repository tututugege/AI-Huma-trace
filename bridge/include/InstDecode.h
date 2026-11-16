#pragma once
// One pass over a RISC-V instruction word (rv64gc; a compressed instruction comes expanded): its class and the
// registers / memory it touches. A pure function of the word: the decode stage (Idu: what the pipeline renames and
// issues) and the trace bridge (what the function model may change, for its undo log) both read it, each with its
// own rules on top.
#include "types.h"
#include <cstdint>

struct DecodedInst {
  // the ISA view (the decode stage)
  InstType type = NOP;
  bool illegal = false;              // a statically reserved encoding / an unknown CSR
  uint32_t rd = 0, rs1 = 0, rs2 = 0, rs3 = 0;
  bool dest_en = false, src1_en = false, src2_en = false, src3_en = false;
  bool dest_fp = false, src1_fp = false, src2_fp = false, src3_fp = false;
  bool src1_is_pc = false, src2_is_imm = true, is_word = false, is_atomic = false;
  uint32_t fp_cls = FP_NONE;         // FpCls
  uint32_t f3 = 0, f7 = 0, csr = 0;
  int64_t imm = 0;
  // the function model's view (an executed instruction): register files it may write, fcsr, memory access kind
  bool writes_fpr = false, touches_fcsr = false, writes_xreg = false;
  bool mem_load = false, mem_store = false, is_lr = false;
};

DecodedInst decode_inst(uint32_t inst);

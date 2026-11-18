#pragma once
#include "types.h"
#include <cstdio>
#include <cstdlib>

#define LOOP_INC(idx, length) idx = (idx + 1) % (length)
extern long long sim_time;

// Custom Assert Macro to avoid WSL2 issues.
// Respect NDEBUG so release builds do not pay hot-path assertion cost.
#ifdef NDEBUG
#define Assert(cond)                                                           \
  do {                                                                         \
    (void)sizeof(cond);                                                        \
  } while (0)
#else
#define Assert(cond)                                                           \
  do {                                                                         \
    if (__builtin_expect(!(cond), 0)) {                                        \
      printf("\033[1;31mAssertion failed: %s, file %s, line %d, cycle "        \
             "%lld\033[0m\n",                                                  \
             #cond, __FILE__, __LINE__, sim_time);                             \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)
#endif

inline int get_rob_line(uint32_t rob_idx) { return rob_idx / ROB_BANK_NUM; }
inline int get_rob_bank(uint32_t rob_idx) { return rob_idx % ROB_BANK_NUM; }

inline uint8_t rob_cplt_popcount(wire<ROB_CPLT_MASK_WIDTH> mask) {
  return static_cast<uint8_t>(__builtin_popcount(static_cast<unsigned>(mask)));
}

// The queue a uop of kind `kind` (UopKind) is enqueued in: the loads and the store addresses go to the memory queue.
// With two INT queues (CFG_INT_CLUSTERS=2) an integer uop goes to the queue of its position: CSR to IQ_INT, I2F to IQ_INT1, the others by
// the parity of the ROB index (even / odd lane of the dispatch group).
inline int route_iq(int kind, int op = 0, uint32_t rob_idx = 0) {
  switch (kind) {
  case UK_INT:
    if (CFG_INT_CLUSTERS == 2) return op == UOP_CSR ? IQ_INT : op == UOP_I2F ? IQ_INT1 : (rob_idx & 1 ? IQ_INT1 : IQ_INT);
    return IQ_INT;
  case UK_LD:
  case UK_STA: return IQ_MEM;
  case UK_STD: return IQ_STD;
  case UK_BR:  return IQ_BR;
  default:     return IQ_FP;
  }
}

inline wire<ROB_CPLT_MASK_WIDTH> rob_cplt_mask_from_kind(int kind) {
  switch (kind) {
  case UK_INT:
  case UK_LD:
  case UK_FP:
    return ROB_CPLT_G0;
  case UK_BR:
  case UK_STA:
    return ROB_CPLT_G1;
  case UK_STD:
    return ROB_CPLT_G2;
  default:
    Assert(0 && "Unknown uop kind for ROB completion mask");
    return 0;
  }
}

inline wire<ROB_CPLT_MASK_WIDTH> rob_cplt_mask_from_issue_port(int port_idx) {
  if (port_idx >= IQ_ALU_PORT_BASE && port_idx < IQ_ALU_PORT_BASE + ALU_NUM) {
    return ROB_CPLT_G0;
  }
  if (port_idx >= IQ_LD_PORT_BASE &&
      port_idx < IQ_LD_PORT_BASE + LSU_LOAD_WB_WIDTH) {
    return ROB_CPLT_G0;
  }
  if (port_idx >= IQ_FP_PORT_BASE && port_idx < IQ_FP_PORT_BASE + GLOBAL_IQ_CONFIG[IQ_FP].port_num) {
    return ROB_CPLT_G0;
  }
  if (port_idx >= IQ_BR_PORT_BASE && port_idx < IQ_BR_PORT_BASE + BRU_NUM) {
    return ROB_CPLT_G1;
  }
  if (port_idx >= IQ_STA_PORT_BASE &&
      port_idx < IQ_STA_PORT_BASE + LSU_STA_COUNT) {
    return ROB_CPLT_G1;
  }
  if (port_idx >= IQ_STD_PORT_BASE &&
      port_idx < IQ_STD_PORT_BASE + GLOBAL_IQ_CONFIG[IQ_STD].port_num) {
    return ROB_CPLT_G2;
  }
  Assert(0 && "Unknown issue port for ROB completion mask");
  return 0;
}

inline bool is_branch(InstType type) { return type == BR || type == JALR; }

constexpr InstType encode_inst_type(InstType type) { return type; }
inline InstType decode_inst_type(InstType type) { return type; }

inline InstType decode_inst_type(wire<INST_TYPE_WIDTH> bits) {
  Assert((uint32_t)bits < INST_TYPE_COUNT);
  return static_cast<InstType>((uint32_t)bits);
}

inline UopType decode_uop_type(UopType op) { return op; }

inline UopType decode_uop_type(wire<UOP_TYPE_WIDTH> bits) {
  Assert((uint32_t)bits < MAX_UOP_TYPE);
  return static_cast<UopType>((uint32_t)bits);
}

inline bool is_store(InstInfo uop) {
  return uop->type == STORE ||
         (uop->type == AMO && (uop->func7 >> 2) != AmoOp::LR);
}

inline bool is_load(InstInfo uop) {
  return uop->type == LOAD || (uop->type == AMO && (uop->func7 >> 2) != AmoOp::SC);
}

// Move elimination (EXP-012): `addi rd, rs, 0` with rd and rs both real registers is renamed away: rd takes over rs's
// physical register, no uop is issued, the ROB entry completes at dispatch.
inline bool is_move_shape(const InstInfo &u) {
  return decode_inst_type(u->type) == ADD && u->src2_is_imm && !u->is_word && u->func3 == 0 && u->imm == 0 && u->src1_en &&
         !u->src1_is_pc && u->src1_areg != 0 && u.dest_en && u->dest_areg != 0;
}
// After rename: an eliminated move shares its source's physical register.
inline bool is_eliminated_move(const InstInfo &u) {
  return is_move_shape(u) && u->dest_preg == u->src1_preg;
}

static inline bool is_amo_lr_uop(const MicroOp &uop) {
  return uop->is_atomic && ((uop->func7 >> 2) == AmoOp::LR);
}

inline bool is_CSR(InstType type) {
  return (type == CSR || type == MRET || type == ECALL || type == EBREAK);
}
inline bool is_branch(wire<INST_TYPE_WIDTH> type) {
  return is_branch(decode_inst_type(type));
}
inline bool is_CSR(wire<INST_TYPE_WIDTH> type) {
  return is_CSR(decode_inst_type(type));
}

inline bool is_branch_uop(UopType op) { return op == UOP_BR || op == UOP_JUMP; }
inline bool is_branch_uop(wire<UOP_TYPE_WIDTH> op) {
  return is_branch_uop(decode_uop_type(op));
}

inline bool cmp_inst_age(InstInfo inst1, InstInfo inst2) {
  if (inst1->rob_flag == inst2->rob_flag) {
    return inst1->rob_idx > inst2->rob_idx;
  } else {
    return inst1->rob_idx < inst2->rob_idx;
  }
}

inline bool is_exception(InstInfo uop) {
  return uop.page_fault_inst || uop.page_fault_load || uop.page_fault_store ||
         uop.illegal_inst || uop->type == ECALL;
}


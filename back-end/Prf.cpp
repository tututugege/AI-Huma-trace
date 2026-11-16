#include "Wires.h"
#include "Prf.h"
#include "PreIduQueue.h"
#include "IO.h"
#include "config.h"
#include "util.h"
#include <cstring>

namespace {
static inline bool is_killed(const Uop &uop,
                             const DecBroadcastIO *db) {
  if (!db->mispred) return false;
  return (uop.br_mask & db->br_mask) != 0;
}

static inline int get_prf_ftq_slot(int p_idx, const Uop &uop,
                                   uint64_t req_bit) {
  if (p_idx >= IQ_ALU_PORT_BASE && p_idx < IQ_ALU_PORT_BASE + ALU_NUM &&
      (req_bit & OP_MASK_ALU) && uop->src1_is_pc) {
    return p_idx - IQ_ALU_PORT_BASE;
  }
  if (p_idx >= IQ_BR_PORT_BASE && p_idx < IQ_BR_PORT_BASE + BRU_NUM &&
      (req_bit & OP_MASK_BR)) {
    return ALU_NUM + (p_idx - IQ_BR_PORT_BASE);
  }
  return -1;
}

static inline bool is_load_wb(const Uop &uop) {
  return decode_uop_type(uop.op) == UOP_LOAD;
}
} // namespace

void Prf::init() {
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    inst_r[i] = {};
    inst_r_1[i] = {};
  }
}


// Register-read stage: there is no register file (operand values are facts of the trace), so the issued uops pass through
// to the EXU and the ALU/BRU ports that need the instruction PC take it from the FTQ response.
void Prf::comb_read() {
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    wires->prf2exe.iss_entry[i].valid = wires->iss2prf.iss_entry[i].valid;
    if (!wires->prf2exe.iss_entry[i].valid)
      continue;

    auto &entry = wires->prf2exe.iss_entry[i];
    entry.uop = (wires->iss2prf.iss_entry[i].uop);
    
    // Set PC and FTQ fields from FTQ response
    uint64_t req_bit = (1ULL << wires->iss2prf.iss_entry[i].uop.op);
    if (get_prf_ftq_slot(i, wires->iss2prf.iss_entry[i].uop, req_bit) >= 0) {   // the ALU (pc operand) and BRU ports
      const FtqPc r = ftq->ftq_pc(entry.uop->ftq_idx, entry.uop->ftq_offset);
      entry.uop->pc = r.pc;
      entry.uop->ftq_pred_taken = r.pred_taken;
      entry.uop->ftq_next_pc = r.next_pc;
    } else {
      entry.uop->pc = 0;
      entry.uop->ftq_pred_taken = false;
      entry.uop->ftq_next_pc = 0;
    }
  }
}

// 功能：从写回流水寄存器中提取可见的 Load 完成事件并生成唤醒广播。
// 输入依赖：inst_r[]、&wires->dec_bcast（mispred/br_mask）。
// 输出更新：wires->prf_awake.wake[]。
// 约束：仅 Load 且未被 squash 的目的寄存器参与唤醒；端口数受 LSU_LOAD_WB_WIDTH 限制。
void Prf::comb_awake() {
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    wires->prf_awake.wake[i].valid = false;
  }

  int awake_idx = 0;
  // 遍历寻找有效的 Load 唤醒 (支持多端口)
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    if (inst_r[i].valid && inst_r[i].uop.dest_en && is_load_wb(inst_r[i].uop)) {
      bool is_squashed = is_killed(inst_r[i].uop, &wires->dec_bcast);
      if (is_squashed) {
        continue;
      }
      if (inst_r[i].uop.early_woken) continue;  // the LSU already woke the consumers from its L3 register

      if (awake_idx < LSU_LOAD_WB_WIDTH) {
        wires->prf_awake.wake[awake_idx].valid = true;
        wires->prf_awake.wake[awake_idx].preg = inst_r[i].uop.dest_preg;
        awake_idx++;
      }
    }
  }
}

// 功能：推进 PRF 写回流水寄存器，并处理 flush/mispred/clear_mask。
// 输入依赖：wires->rob_bcast.flush、wires->exe2prf.entry[]、wires->dec_bcast.{mispred,br_mask,clear_mask}。
// 输出更新：inst_r_1[]（valid/uop 及 br_mask 清理）。
// 约束：flush 优先清空；mispred 命中分支掩码的条目被杀；存活条目需清除 clear_mask。
void Prf::comb_pipeline() {
  bool global_flush = wires->rob_bcast.flush;
  wire<BR_MASK_WIDTH> clear = wires->dec_bcast.clear_mask;
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    if (global_flush) {
      inst_r_1[i].valid = false;
    } else if (wires->exe2prf.entry[i].valid) {
      inst_r_1[i].valid = true;
      inst_r_1[i].uop = wires->exe2prf.entry[i].uop;
      if (is_killed(inst_r_1[i].uop, &wires->dec_bcast)) {
        inst_r_1[i].valid = false;
      } else if (inst_r_1[i].valid && clear) {
        inst_r_1[i].uop.br_mask &= ~clear;
      }
    } else {
      inst_r_1[i].valid = false;
    }
  }
}

void Prf::seq() {
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    inst_r[i] = inst_r_1[i];
  }
}

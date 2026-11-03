#pragma once
// Frozen copy of the original Isu (lockstep reference, ISU_SHADOW builds).
#include "IssueQueue.h"
#include "config.h"
#include "IO.h"
#include <cstdint>

// MUL 固定延迟唤醒条目（移位寄存器）
struct MulWakeEntryRef {
  wire<1> valid = 0;
  wire<PRF_IDX_WIDTH> dest_preg = 0;
  wire<BR_MASK_WIDTH> br_mask;
};

// DIV/FP 迭代唤醒条目（计数器槽位）
struct IterWakeEntryRef {
  wire<1> valid = 0;
  int countdown = 0;
  wire<PRF_IDX_WIDTH> dest_preg = 0;
  wire<BR_MASK_WIDTH> br_mask;
};

constexpr int REF_ISU_MUL_WAKE_UNIT_NUM = count_ports_with_mask(OP_MASK_MUL);
constexpr int REF_ISU_DIV_WAKE_UNIT_NUM = count_ports_with_mask(OP_MASK_DIV);
constexpr int REF_ISU_FP_WAKE_UNIT_NUM = count_ports_with_mask(OP_MASK_FP);
constexpr int REF_ISU_MUL_WAKE_DEPTH = (MUL_MAX_LATENCY > 1) ? (MUL_MAX_LATENCY - 1) : 1;
constexpr int REF_ISU_MUL_WAKE_SLOT_NUM =
    (REF_ISU_MUL_WAKE_UNIT_NUM > 0) ? REF_ISU_MUL_WAKE_UNIT_NUM : 1;
constexpr int REF_ISU_DIV_WAKE_SLOT_NUM =
    (REF_ISU_DIV_WAKE_UNIT_NUM > 0) ? REF_ISU_DIV_WAKE_UNIT_NUM : 1;
constexpr int REF_ISU_FP_WAKE_SLOT_NUM =
    (REF_ISU_FP_WAKE_UNIT_NUM > 0) ? REF_ISU_FP_WAKE_UNIT_NUM : 1;

class IsuRef {
public:
  friend struct IsuDigestAccess;
private:
  SimContext *ctx;

  // 1. 发射队列 (固定顺序: INT, LD, STA, STD, BR...)
  // 直接使用 vector，但依靠 init 的顺序保证下标对应枚举
  std::vector<IssueQueue> iqs;
  std::vector<IssueQueueConfig> configs;

  // 2. 延迟唤醒
  // MUL: 固定延迟，使用移位寄存器模型（深度 = MUL_MAX_LATENCY - 1）
  MulWakeEntryRef mul_wake_pipe[REF_ISU_MUL_WAKE_DEPTH][REF_ISU_MUL_WAKE_SLOT_NUM];
  MulWakeEntryRef mul_wake_pipe_1[REF_ISU_MUL_WAKE_DEPTH][REF_ISU_MUL_WAKE_SLOT_NUM];
  // DIV / FP: 迭代单元，使用 countdown 槽位（槽位数 = 对应 FU 个数）
  IterWakeEntryRef div_wake_slots[REF_ISU_DIV_WAKE_SLOT_NUM];
  IterWakeEntryRef div_wake_slots_1[REF_ISU_DIV_WAKE_SLOT_NUM];
  IterWakeEntryRef fp_wake_slots[REF_ISU_FP_WAKE_SLOT_NUM];
  IterWakeEntryRef fp_wake_slots_1[REF_ISU_FP_WAKE_SLOT_NUM];

public:
  uint32_t port_attributes[ISSUE_WIDTH];
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  IsuRef(SimContext *ctx);
  void init();
  void comb_begin(); // 默认保持寄存器状态（*_1 <- *）

  // 组合逻辑
  void comb_ready(); // 更新 iss2dis->free_slots
  void comb_enq();   // 处理 dispatch 入队
  void comb_issue(); // 调度 + 发射 + 延迟处理
  void comb_awake(); // 汇总所有唤醒源
  void comb_calc_latency_next();

  // 辅助逻辑
  void comb_flush(); // 全局流水线清空

  // 时序逻辑
  void seq();

private:
  void add_iq(const IssueQueueConfig &cfg);
  int get_latency(UopType uop_type, wire<7> func7);
  void apply_wakeup_to_uop(Uop &uop) const;
};

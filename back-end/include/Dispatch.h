#pragma once
#include "IO.h"
#include "config.h"

class BackTop;
class Ren;

struct UopPacket {
  int iq_id;               // 目标 IQ
  Uop uop; // Dispatch 内部直接使用发往 Issue 的线网格式
};

class Dispatch {
private:

  Uop make_dis_iss_uop(const Uop &inst, int k) const;
  bool is_preg_woken(wire<PRF_IDX_WIDTH> preg) const;
  void apply_wakeup_to_uop(Uop &uop) const;
  int decompose_inst(const Uop &original_inst, UopPacket *out_uops);

  Uop inst_alloc[DECODE_WIDTH];

  // 记录每条指令 Dispatch 是否成功 (comb_dispatch -> comb_fire)
  bool dispatch_success_flags[DECODE_WIDTH];

  // 记录每个端口分配到的指令槽位，-1 表示该端口本拍未分配。
  int stq_port_owner[MAX_STQ_DISPATCH_WIDTH];
  int ldq_port_owner[MAX_LDQ_DISPATCH_WIDTH];

  struct DispatchCache {
    int count;                     // 拆分数量
    int iq_ids[MAX_UOPS_PER_INST]; // 仅保存目标 IQ 的 ID
  };

  // 用于在 comb_dispatch 和 comb_fire 之间传递数据
  DispatchCache dispatch_cache[DECODE_WIDTH];

public:
  friend struct DisDigestAccess;  // lockstep test hook
  Dispatch(SimContext *ctx) { this->ctx = ctx; }
  SimContext *ctx;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

  void init();
  void comb_begin(); // 默认保持寄存器状态（*_1 <- *）
  void comb_alloc();
  void comb_dispatch();
  void comb_wake();
  void comb_cancel();  // LSU_PIPE_L2: speculative load wake-up of the previous cycle was wrong
  void comb_fire();
  void comb_pipeline();
  void seq();
  // PMU (EXP-013): classify this cycle's DECODE_WIDTH dispatch slots into the event vector (read-only, after comb_fire)
  void pmu_slots(struct PmuVec &ev, bool recovering, bool recovering_by_flush, int &be_slots) const;
  Uop inst_r[DECODE_WIDTH];
  Uop inst_r_1[DECODE_WIDTH];
  reg<1> inst_valid[DECODE_WIDTH];
  wire<1> inst_valid_1[DECODE_WIDTH];
  reg<1> busy_table[PRF_NUM];
  wire<1> busy_table_1[PRF_NUM];
};

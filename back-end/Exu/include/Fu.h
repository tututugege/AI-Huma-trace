#pragma once
#include "AbstractFU.h" // for __builtin_clz
#include "IO.h"
#include "config.h"
#include "trace_hook.h"
// #include <cassert>
#include <climits>

// ==========================================
// ALU: 算术逻辑单元
// ==========================================
class AluUnit : public SingleCycleFU {
  static constexpr int SUB = 0b000;
  static constexpr int SLL = 0b001;
  static constexpr int SLT = 0b010;
  static constexpr int SLTU = 0b011;
  static constexpr int XOR = 0b100;
  static constexpr int SRL = 0b101;
  static constexpr int OR = 0b110;
  static constexpr int AND = 0b111;

public:
  AluUnit(std::string name = "ALU", int port_idx = 0)
      : SingleCycleFU(name, port_idx) {}

protected:
  void impl_compute(ExuInst &inst) override {
    // Value datapath removed (P3): results come from the trace bridge facts; timing does not read them.
    (void)inst;
  }
};

// ==========================================
// AGU: 地址生成单元 (IO 精确修改版)
// ==========================================
class AguUnit : public SingleCycleFU {
public:
  AguUnit(std::string name, int port_idx) : SingleCycleFU(name, port_idx) {}

protected:
  void impl_compute(ExuInst &inst) override {
    // 1. 计算虚拟地址 (Common logic)
    // Load 和 Store (STA) 都需要计算地址： Base + Offset
    uint64_t vaddr = 0;  // the address is a fact of the trace (rs1 + imm is not computed)
    if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) {
      vaddr = f->vaddr;
    }
    inst.result =
        vaddr; // 记录结果，如果是 Load，这个值是地址；如果是 STA，也是地址
  }
};

// ==========================================
// SDU: 存数数据单元 (IO 精确修改版)
// ==========================================
class SduUnit : public SingleCycleFU {
public:
  SduUnit(std::string name, int port_idx) : SingleCycleFU(name, port_idx) {}

protected:
  void impl_compute(ExuInst &inst) override {
    // The store data is a fact of the trace (rs2); the register value datapath no longer exists.
    uint32_t result_data = 0;
    if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) {
      result_data = f->rs2_value;
    }
    inst.result = result_data;  // result carries the store data to the LSU
  }
};

// ==========================================
// MUL: 乘法单元
// ==========================================
class MulUnit : public FixedLatencyFU {
public:
  MulUnit(std::string name = "MUL", int port_idx = 0, int lat = MUL_MAX_LATENCY)
      : FixedLatencyFU(name, port_idx, lat) {}

protected:
  void impl_compute(ExuInst &inst) override {
    // Value datapath removed (P3): results come from the trace bridge facts; timing does not read them.
    (void)inst;
  }
};

// ==========================================
// BRU: 分支单元
// ==========================================
class BruUnit : public SingleCycleFU {
  static constexpr int BEQ = 0b000;
  static constexpr int BNE = 0b001;
  static constexpr int BLT = 0b100;
  static constexpr int BGE = 0b101;
  static constexpr int BLTU = 0b110;
  static constexpr int BGEU = 0b111;
public:
  BruUnit(std::string name, int port_idx) : SingleCycleFU(name, port_idx) {}

protected:
  void impl_compute(ExuInst &inst) override {
    // Direction and target are facts of the trace (the compare on register values is gone).
    uint32_t inst_pc = inst->pc;
    uint32_t pc_br = inst_pc + inst->imm;
    bool br_taken = false;
    if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) {
      if (inst.op == UOP_BR) {
        br_taken = f->br_taken;
      } else if (inst.op == UOP_JUMP) {
        br_taken = true;
        pc_br = f->next_pc;  // jal/jalr target
      }
    } else if (inst.op == UOP_JUMP) {
      br_taken = true;  // killed instruction: no fact, outcome unobservable
    }

#if defined(CONFIG_ORACLE_STEADY_FETCH_WIDTH) && !defined(CONFIG_BPU)
    // Oracle steady-fetch stress mode intentionally relaxes FTQ block semantics.
    // In no-BPU builds, treat branch prediction as always-correct and avoid
    // recovery redirects from BRU.
    inst.mispred = false;
#else
    // FTQ lookup
    bool pred_taken = inst->ftq_pred_taken;
    uint32_t pred_target = 0;

    if (pred_taken) {
      pred_target = inst->ftq_next_pc;
    } else {
      pred_target = inst_pc + 4;
    }

    // Verify
    if ((br_taken && pred_taken && pred_target == pc_br) ||
        (!br_taken && !pred_taken)) {
      inst.mispred = false;
    } else {
      inst.mispred = true;
    }
#endif

    inst.br_taken = br_taken;
    inst.diag_val = br_taken ? pc_br : (inst_pc + 4);
  }
};

// ==========================================
// DIV: 除法单元 (Fixed Latency for simulation correctness)
// Note: While real dividers are non-pipelined (IterativeFU), using
// FixedLatencyFU eliminates race conditions with register recycling in this
// implementation
// ==========================================
class DivUnit : public IterativeFU {
public:
  DivUnit(std::string name, int port_idx, int lat)
      : IterativeFU(name, port_idx, lat) {}

protected:
  void impl_compute(ExuInst &inst) override {
    // Value datapath removed (P3): results come from the trace bridge facts; timing does not read them.
    (void)inst;
  }

};

// ==========================================
// CSR: 控制状态寄存器单元
// ==========================================
class CsrUnit : public SingleCycleFU {
public:
  CsrUnit(std::string name, int port_idx) : SingleCycleFU(name, port_idx) {}

protected:
  void impl_compute(ExuInst &inst) override { (void)inst; }
};

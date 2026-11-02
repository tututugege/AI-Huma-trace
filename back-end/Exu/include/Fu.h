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
    uint64_t vaddr = inst.src1_rdata + inst.imm;
    if (tbhook::inject) {
      if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) { tbhook::note_override(2, (uint32_t)vaddr, f->vaddr, inst.rob_idx); vaddr = f->vaddr; }
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
    uint32_t result_data = 0;

    // === 1. 获取操作数 & 计算 Store Data ===
    uint32_t op1 = inst.src1_rdata;
    uint32_t op2 = inst.src2_rdata;
    bool is_atomic = inst.is_atomic;

    if (!is_atomic) {
      // 普通 Store: src2_rdata 是要存的数据 (对应 RS2)
      // 注意：根据你的架构定义，这里可能需要确认数据在 src1 还是 src2
      // 通常 Store: Mem[RS1+Imm] = RS2. 所以数据在 src2_rdata.
      // 但你的旧代码用了 src1_rdata 作为 op1 赋值给 result_data，请根据你的
      // decode 逻辑确认。 假设 decode 将 rs2 的值放到了 src2_rdata：
      result_data = inst.src2_rdata;
    } else {
      // AMO 运算逻辑
      switch (inst.func7 >> 2) {
      case AmoOp::SWAP:
        result_data = op2;
        break;
      case AmoOp::ADD:
        result_data = op1 + op2;
        break;
      case AmoOp::XOR:
        result_data = op1 ^ op2;
        break;
      case AmoOp::AND:
        result_data = op1 & op2;
        break;
      case AmoOp::OR:
        result_data = op1 | op2;
        break;
      case AmoOp::MIN:
        result_data = ((int32_t)op1 < (int32_t)op2) ? op1 : op2;
        break;
      case AmoOp::MAX:
        result_data = ((int32_t)op1 > (int32_t)op2) ? op1 : op2;
        break;
      case AmoOp::MINU:
        result_data = (op1 < op2) ? op1 : op2;
        break;
      case AmoOp::MAXU:
        result_data = (op1 > op2) ? op1 : op2;
        break;
      default:
        result_data = op2;
        break;
      }
    }

    if (tbhook::inject && !is_atomic) {
      if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) { tbhook::note_override(3, result_data, f->rs2_value, inst.rob_idx); result_data = f->rs2_value; }
    }
    // 更新 uop 中的 result，以便传输给 LSU
    inst.result = result_data; // 这里 result 存的是 Data
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
    uint32_t operand1 = inst.src1_rdata;
    uint32_t operand2 = inst.src2_rdata;
    uint32_t inst_pc = inst.pc;
    uint32_t pc_br = inst_pc + inst.imm;
    bool br_taken = true;

    if (inst.op == UOP_BR) {
      switch (inst.func3) {
      case BEQ:
        br_taken = (operand1 == operand2);
        break;
      case BNE:
        br_taken = (operand1 != operand2);
        break;
      case BGE:
        br_taken = ((signed)operand1 >= (signed)operand2);
        break;
      case BLT:
        br_taken = ((signed)operand1 < (signed)operand2);
        break;
      case BGEU:
        br_taken = ((unsigned)operand1 >= (unsigned)operand2);
        break;
      case BLTU:
        br_taken = ((unsigned)operand1 < (unsigned)operand2);
        break;
      }
    }

    switch (inst.op) {
    case UOP_BR:
      break;
    case UOP_JUMP:
      br_taken = true;
      if (inst.src1_en)
        pc_br = (inst.src1_rdata + inst.imm) & (~0x1);
      break;
    default:
      br_taken = false;
    }
    if (tbhook::inject && (inst.op == UOP_BR || inst.op == UOP_JUMP)) {
      if (const tb::Fact *f = tbhook::fact_by_rob(inst.rob_idx)) {
        if (inst.op == UOP_BR) { tbhook::note_override(0, br_taken, f->br_taken, inst.rob_idx); br_taken = f->br_taken; }
        else { tbhook::note_override(1, pc_br, f->next_pc, inst.rob_idx); pc_br = f->next_pc; }  // jal/jalr target
      }
    }

#if defined(CONFIG_ORACLE_STEADY_FETCH_WIDTH) && !defined(CONFIG_BPU)
    // Oracle steady-fetch stress mode intentionally relaxes FTQ block semantics.
    // In no-BPU builds, treat branch prediction as always-correct and avoid
    // recovery redirects from BRU.
    inst.mispred = false;
#else
    // FTQ lookup
    bool pred_taken = inst.ftq_pred_taken;
    uint32_t pred_target = 0;

    if (pred_taken) {
      pred_target = inst.ftq_next_pc;
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

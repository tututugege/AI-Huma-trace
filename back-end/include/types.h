#pragma once
#include "config.h"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

// Standard library usings (kept for compatibility)
using std::cin;
using std::cout;
using std::dec;
using std::endl;
using std::hex;
using std::string;

// Legacy branch-mask alias used across LSU/dispatch paths.
using mask_t = wire<BR_MASK_WIDTH>;

// ==========================================
// [Instruction & Pipeline Definitions]
// ==========================================

enum InstType {
  NOP,
  JAL,
  JALR,
  ADD,
  BR,
  LOAD,
  STORE,
  CSR,
  FENCE,
  ECALL,
  EBREAK,
  WFI,
  SFENCE_VMA,
  FENCE_I,
  MRET,
  SRET,
  MUL,
  DIV,
  AMO,
  FP
};

constexpr int INST_TYPE_COUNT = FP + 1;
constexpr int INST_TYPE_WIDTH = bit_width_for_count(INST_TYPE_COUNT);

// AMO Operations (funct7[6:2])
namespace AmoOp {
constexpr uint8_t ADD = 0b00000;
constexpr uint8_t SWAP = 0b00001;
constexpr uint8_t LR = 0b00010;
constexpr uint8_t SC = 0b00011;
constexpr uint8_t XOR = 0b00100;
constexpr uint8_t OR = 0b01000;
constexpr uint8_t AND = 0b01100;
constexpr uint8_t MIN = 0b10000;
constexpr uint8_t MAX = 0b10100;
constexpr uint8_t MINU = 0b11000;
constexpr uint8_t MAXU = 0b11100;
} // namespace AmoOp

// ==========================================
// [Structs & Classes]
// ==========================================

// Debug sideband shared by InstInfo/MicroOp. Keep diag_val in hardware path.
struct DebugMeta {
  wire<32> instruction;
  wire<32> pc;
  bool is_mmio;
  int64_t inst_idx;
};

struct TmaMeta {
  bool is_cache_miss;
  bool is_ret;
  bool mem_commit_is_load;
  bool mem_commit_is_store;
};

struct InstEntry;
// One instruction record for every pipeline stage (decode, rename, dispatch, issue, execute, ROB, commit).
// The pipeline registers hold a Uop = id + the few fields that change after creation. Everything written when a
// uop is created (decode, rename, dispatch) lives in a pool (UopS), indexed by the id: `u.imm`, `u.pc`...
// Dynamic fields stay in the entry (`u.br_mask`, `u.result`, ...), so the comb/seq double buffering keeps its
// meaning: a field that changes while the uop is in flight is never shared between the two copies.
struct UopS {
  wire<AREG_IDX_WIDTH> dest_areg;
  wire<AREG_IDX_WIDTH> src1_areg;
  wire<AREG_IDX_WIDTH> src2_areg;
  wire<FTQ_IDX_WIDTH> ftq_idx;
  wire<FTQ_OFFSET_WIDTH> ftq_offset;
  wire<BR_TAG_WIDTH> br_id;
  wire<CSR_IDX_WIDTH> csr_idx;
  InstType type;
  wire<32> imm;
  wire<PRF_IDX_WIDTH> src1_preg;
  wire<PRF_IDX_WIDTH> src2_preg;
  wire<1> src1_en;
  wire<1> src2_en;
  wire<1> src1_is_pc;
  wire<1> src2_is_imm;
  wire<3> func3;
  wire<7> func7;
  wire<1> is_atomic;
  wire<PRF_IDX_WIDTH> old_dest_preg;
  wire<32> pc;
  wire<32> ftq_next_pc;
  wire<1> ftq_entry_valid;
  wire<1> ftq_resp_valid;
  wire<1> ftq_pred_taken;
  DebugMeta dbg;
};

constexpr int UOP_POOL_SIZE = 65535;     // ring; far more than ROB + queues + front-end buffers + wrong-path waves in flight
constexpr int UOP_BLOCK = 4;             // an instruction owns [id] (decode/ROB) and [id+1..id+3] (its up to three dispatched uops, AMOs)
extern UopS uop_pool[UOP_POOL_SIZE];
uint16_t uop_alloc();                // fresh zeroed block of UOP_BLOCK pool entries (ring; entry 0 is the shared null entry)

struct Uop {
  uint16_t id;

  wire<32> diag_val; // Hardware: Shared field for instruction or pc_next
  wire<32> result;
  wire<PRF_IDX_WIDTH> dest_preg;
  wire<1> dest_en;

  wire<1> mispred;
  wire<1> br_taken;
  wire<1> src1_busy, src2_busy;
  wire<BR_MASK_WIDTH> br_mask;

  wire<ROB_IDX_WIDTH> rob_idx;
  wire<STQ_IDX_WIDTH> stq_idx;
  wire<1> stq_flag;
  wire<LDQ_IDX_WIDTH> ldq_idx;

  wire<ROB_CPLT_MASK_WIDTH> expect_mask;
  wire<ROB_CPLT_MASK_WIDTH> cplt_mask;
  wire<1> rob_flag; // 用于对比指令年龄

  wire<1> page_fault_inst;
  wire<1> page_fault_load;
  wire<1> page_fault_store;
  wire<1> illegal_inst;
  wire<1> flush_pipe;
  wire<1> ftq_is_last;  // set on the mispredicting branch inside the ROB
  wire<1> is_mmio;      // set when the LSU executes an MMIO access

  UopType op;
  TmaMeta tma;


  Uop() { std::memset(this, 0, sizeof(Uop)); }

  UopS *operator->() const { return &uop_pool[id]; }

  // Converters of the old per-stage structs: identity.
  static Uop from_ren_dis_inst(const Uop &s) { return s; }
  static Uop from_dec_ren_inst(const Uop &s) { return s; }
  static Uop from_dis_iss_uop(const Uop &s) { return s; }
  static Uop from_iss_prf_uop(const Uop &s) { return s; }
  static Uop from_prf_exe_uop(const Uop &s) { return s; }
  static Uop from_dis_rob_inst(const Uop &s) { return s; }
  static Uop from_micro_op(const Uop &s) { return s; }
  const Uop &to_dis_rob_inst() const { return *this; }
  const Uop &to_iss_prf_uop() const { return *this; }
  const Uop &to_exe_prf_wb_uop() const { return *this; }
  const Uop &to_exu_rob_uop() const { return *this; }
  const Uop &to_exe_lsu_req_uop() const { return *this; }
  const Uop &to_commit_inst() const { return *this; }
  const Uop &to_micro_op() const { return *this; }
  InstEntry to_inst_entry(wire<1> valid) const;
};
using InstInfo = Uop;
using MicroOp = Uop;

struct InstEntry {
  wire<1> valid;
  InstInfo uop;
};

inline InstEntry Uop::to_inst_entry(wire<1> valid) const {
  InstEntry e;
  e.valid = valid;
  e.uop = *this;
  return e;
}

struct UopEntry {
  wire<1> valid;
  MicroOp uop;
};

struct WakeInfo {
  wire<1> valid;
  wire<PRF_IDX_WIDTH> preg;
};

#include "PerfCount.h"

// Added to support Remote icache
enum class ExitReason { NONE, EBREAK, WFI, SIMPOINT };
class SimCpu;

class SimContext {
public:
  PerfCount perf;
  ExitReason exit_reason = ExitReason::NONE;
  bool is_ckpt = false;
  uint64_t ckpt_warmup_commit_target = 0;
  uint64_t ckpt_measure_commit_target = 0;
  SimCpu *cpu = nullptr;
  void run_commit_inst(InstEntry *inst_entry, int commit_slot);
  void run_interrupt_hook();
};

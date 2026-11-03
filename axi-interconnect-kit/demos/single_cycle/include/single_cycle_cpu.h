#pragma once
#include <cstdint>

#define RISCV_MODE_U 0b00
#define RISCV_MODE_S 0b01
#define RISCV_MODE_M 0b11

#define BITMASK(bits) ((1ull << (bits)) - 1)
#define BITS(x, hi, lo) \
  (((x) >> (lo)) & BITMASK((hi) - (lo) + 1)) // similar to x[hi:lo] in verilog
#define SEXT(x, len)           \
  ({                           \
    struct {                   \
      int64_t n : len;         \
    } __x = {.n = (int64_t)x}; \
    (uint64_t) __x.n;          \
  })

#define immI(i) SEXT(BITS(i, 31, 20), 12)
#define immU(i) (SEXT(BITS(i, 31, 12), 20) << 12)
#define immS(i) ((SEXT(BITS(i, 31, 25), 7) << 5) | BITS(i, 11, 7))
#define immJ(i)                                                 \
  ((SEXT(BITS(i, 31, 31), 1) << 20) | (BITS(i, 19, 12) << 12) | \
   (BITS(i, 20, 20) << 11) | (BITS(i, 30, 21) << 1))
#define immB(i)                                               \
  ((SEXT(BITS(i, 31, 31), 1) << 12) | (BITS(i, 7, 7) << 11) | \
   (BITS(i, 30, 25) << 5) | (BITS(i, 11, 8) << 1))

// ================= CSR Bit Masks (Standard RISC-V) =================
#define MSTATUS_MIE (1 << 3)
#define MSTATUS_MPIE (1 << 7)
#define MSTATUS_SIE (1 << 1)
#define MSTATUS_SPIE (1 << 5)
#define MSTATUS_MPP (3 << 11) // Bits 11-12
#define MSTATUS_SPP (1 << 8)  // Bit 8

#define MIP_SSIP (1 << 1)
#define MIP_MSIP (1 << 3)
#define MIP_STIP (1 << 5)
#define MIP_MTIP (1 << 7)
#define MIP_SEIP (1 << 9)
#define MIP_MEIP (1 << 11)

// 获取 MPP 的值 (0=U, 1=S, 3=M)
#define GET_MPP(x) ((x >> 11) & 0x3)
// 获取 SPP 的值
#define GET_SPP(x) ((x >> 8) & 0x1)

// SV32 Page Table Entry (PTE) Bits
#define PTE_V (1 << 0) // Valid
#define PTE_R (1 << 1) // Read
#define PTE_W (1 << 2) // Write
#define PTE_X (1 << 3) // Execute
#define PTE_U (1 << 4) // User
#define PTE_G (1 << 5) // Global
#define PTE_A (1 << 6) // Accessed
#define PTE_D (1 << 7) // Dirty

// MSTATUS bits needed for translation
#define MSTATUS_MXR (1 << 19)
#define MSTATUS_SUM (1 << 18)
#define MSTATUS_MPRV (1 << 17)
#define MSTATUS_MPP_SHIFT 11

typedef struct CPU_state {
  uint32_t gpr[32];
  uint32_t csr[21];
  uint32_t pc;

  uint32_t store_addr;
  uint32_t store_data;
  uint32_t store_strb;
  bool store;
} CPU_state;

class SingleCycleCpu {
public:
  uint32_t *memory;
  uint32_t Instruction;
  CPU_state state;
  uint8_t privilege;
  bool asy;
  bool page_fault_inst;
  bool page_fault_load;
  bool page_fault_store;
  bool illegal_exception;
  bool translation_pending;

  bool M_software_interrupt;
  bool M_timer_interrupt;
  bool M_external_interrupt;
  bool S_software_interrupt;
  bool S_timer_interrupt;
  bool S_external_interrupt;

  bool sim_end;

  bool fast_run = false;

  void init(uint32_t reset_pc);
  void exec();
  void RISCV();
  void RV32IM();
  void RV32A();
  void RV32CSR();
  void RV32Zfinx();
  void exception(uint32_t trap_val);
  void store_data();
  bool va2pa(uint32_t &p_addr, uint32_t v_addr, uint32_t type);

  bool is_br;
  bool br_taken;

  bool is_csr;
  bool is_exception;

  static constexpr uint32_t kPtwCacheSize = 512;
  uint32_t ptw_cache_tag[kPtwCacheSize];
  uint32_t ptw_cache_data[kPtwCacheSize];
  bool ptw_cache_valid[kPtwCacheSize];
  void ptw_cache_reset();
  bool ptw_cache_read(uint32_t paddr, uint32_t *data);
  void ptw_cache_fill(uint32_t paddr, uint32_t data);
  void ptw_cache_invalidate_word(uint32_t paddr);
  void ptw_cache_flush();
};

enum CpuMemReadResult : uint8_t {
  CPU_MEM_READ_OK = 0,
  CPU_MEM_READ_PENDING = 1,
  CPU_MEM_READ_FAULT = 2,
};

// Physical-memory read hook used by va2pa page-table walk (supports pending).
extern CpuMemReadResult (*g_cpu_mem_read32_hook)(uint32_t paddr,
                                                 uint32_t *data);

// Immediate read/write hooks used by instruction/load/store/amo paths.
// These hooks must be provided by the embedding runtime.
extern bool (*g_cpu_mem_read32_now_hook)(uint32_t paddr, uint32_t *data);
extern bool (*g_cpu_mem_write32_now_hook)(uint32_t paddr, uint32_t data,
                                          uint32_t wstrb);

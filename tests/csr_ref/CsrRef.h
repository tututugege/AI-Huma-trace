#pragma once
// Frozen copy of the original Csr (lockstep reference, CSR_SHADOW builds).
#include "Csr.h"
class CsrRef {
public:
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  CsrIn in;
  CsrOut out;

  void init();
  void comb_begin(); // 默认保持寄存器状态（*_1 <- *）

  void comb_csr_status();
  void comb_csr_read();
  void comb_csr_write();
  void comb_interrupt();
  void comb_interrupt_inject();
  void comb_exception();
  void seq();

  reg<32> CSR_RegFile[CSR_NUM];
  reg<2> privilege = 0b11;

  // 执行时存下需要写入的idx，cmd和data
  // 提交时才写入
  // 否则csr指令未提交，其更改就已经生效
  reg<12> csr_idx;
  reg<32> csr_wdata;
  reg<2> csr_wcmd;
  reg<1> csr_we;

  wire<32> CSR_RegFile_1[CSR_NUM];
  wire<2> privilege_1 = 0b11;

  wire<12> csr_idx_1;
  wire<32> csr_wdata_1;
  wire<2> csr_wcmd_1;
  wire<1> csr_we_1;
};

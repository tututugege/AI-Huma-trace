#pragma once
#include "IO.h"
#include "config.h"
#include <cstdlib>
#include <cstdio>

#define CSR_S 0b10
#define CSR_C 0b11

enum enum_number_csr_code {
  number_fflags = 0x001,   // F/D: fflags / frm / fcsr are three views of one register (value-free here)
  number_frm = 0x002,
  number_fcsr = 0x003,
  number_mtvec = 0x305,
  number_mepc = 0x341,
  number_mcause = 0x342,
  number_mie = 0x304,
  number_mip = 0x344,
  number_mtval = 0x343,
  number_mscratch = 0x340,
  number_mstatus = 0x300,
  number_mideleg = 0x303,
  number_medeleg = 0x302,
  number_sepc = 0x141,
  number_stvec = 0x105,
  number_scause = 0x142,
  number_sscratch = 0x140,
  number_stval = 0x143,
  number_sstatus = 0x100,
  number_sie = 0x104,
  number_sip = 0x144,
  number_satp = 0x180,
  number_mhartid = 0xf14,
  number_misa = 0x301,
  number_time = 0xc01,
  number_timeh = 0xc81,
};

enum enum_csr {
  csr_mtvec,
  csr_mepc,
  csr_mcause,
  csr_mie,
  csr_mip,
  csr_mtval,
  csr_mscratch,
  csr_mstatus,
  csr_mideleg,
  csr_medeleg,
  csr_sepc,
  csr_stvec,
  csr_scause,
  csr_sscratch,
  csr_stval,
  csr_sstatus,
  csr_sie,
  csr_sip,
  csr_satp,
  csr_mhartid,
  csr_misa,
  csr_fcsr,   // F/D (CSR_NUM = 22)
};

inline int cvt_number_to_csr(int csr_idx) {
  switch (csr_idx) {
  case number_mtvec:
    return csr_mtvec;
  case number_mepc:
    return csr_mepc;
  case number_mcause:
    return csr_mcause;
  case number_mie:
    return csr_mie;
  case number_mip:
    return csr_mip;
  case number_mtval:
    return csr_mtval;
  case number_mscratch:
    return csr_mscratch;
  case number_mstatus:
    return csr_mstatus;
  case number_mideleg:
    return csr_mideleg;
  case number_medeleg:
    return csr_medeleg;
  case number_sepc:
    return csr_sepc;
  case number_stvec:
    return csr_stvec;
  case number_scause:
    return csr_scause;
  case number_sscratch:
    return csr_sscratch;
  case number_stval:
    return csr_stval;
  case number_sstatus:
    return csr_sstatus;
  case number_sie:
    return csr_sie;
  case number_sip:
    return csr_sip;
  case number_satp:
    return csr_satp;
  case number_mhartid:
    return csr_mhartid;
  case number_misa:
    return csr_misa;
  case number_fflags:
  case number_frm:
  case number_fcsr:
    return csr_fcsr;
  case number_time:
  case number_timeh:
    std::fprintf(stderr,
                 "Fatal: CSR 0x%x (time/timeh) is not implemented in Csr RegFile\n",
                 csr_idx);
    std::abort();
  default:
    std::fprintf(stderr, "Fatal: unknown CSR index 0x%x\n", csr_idx);
    std::abort();
  }
}

typedef struct {
} CsrIn;

typedef struct {
} CsrOut;

// CSR file. Contract (checked by scripts/comb/comb_graph.py against the code): the Wires each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_interrupt         in: -                          out: csr2rob
//   CONTRACT comb_commit            in: exe2csr rob2csr rob_bcast  out: csr2front
//   CONTRACT comb_interrupt_inject  in: csr_interrupt_inject_io top2csr    out: -
//   CONTRACT owns: csr2rob csr2front
class Csr {
public:
  // Registers: q (read in comb through const Regs &q, written only by seq) and d (their next value, written by comb;
  // d == q at the start of every cycle). seq: q = d.
  struct Regs {
    reg<XLEN> CSR_RegFile[CSR_NUM];
    reg<2> privilege  = 0b11;
    reg<12> csr_idx;
    reg<XLEN> csr_wdata;
    reg<2> csr_wcmd;
    reg<1> csr_we;
  };
  Regs d;
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  void comb_commit();  // exceptions and the CSR write of the committing instruction
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  CsrIn in;
  CsrOut out;

  void init();

  void comb_csr_status();
  void comb_interrupt();
  void comb_interrupt_inject();
  void seq();
  reg<XLEN> read(int idx) const { return q_.CSR_RegFile[cvt_number_to_csr(idx)]; }   // the read port (a combinational read of the register file)

  // The CSR write (idx, cmd, data) is held at execute and applied at commit:
  // otherwise an uncommitted CSR instruction would already take effect



private:
  // phases of one cycle, run in the order of the methods above
  void exception();
  void csr_write();
  Regs q_;
};

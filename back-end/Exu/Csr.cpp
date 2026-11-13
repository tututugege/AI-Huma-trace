#include "Wires.h"
#include "BeginCheck.h"
#include "Csr.h"
#include "DebugPtwTrace.h"
#include "config.h"
#include "ref.h"
#include <cstdio>
#include <cstdlib>

namespace {
// TB_TRACE_TRAP=1 prints every trap the CSR model takes (cause, epc, tval, target)
bool trace_traps() { static const bool on = std::getenv("TB_TRACE_TRAP") != nullptr; return on; }
// mstatus bits visible through sstatus / writable (as in the function model: rv64 has no SD tracking, UXL = SXL = 2 are read-only)
#if XLEN == 64
#if FP
constexpr xlen_t kSstatusMask = 0x80000003000DE133ull;   // SD included (FS = Dirty)
#else
constexpr xlen_t kSstatusMask = 0x00000003000DE133ull;
#endif
constexpr xlen_t kSstatusWriteMask = 0x00000000000DE133ull;
constexpr xlen_t kMstatusWriteMask = 0x00000000007FF9BBull;
constexpr xlen_t kMstatusReset = 0x0000000A00000000ull;
#else
constexpr uint32_t kSstatusMask = 0x800DE133;
#if FP
constexpr uint32_t kSstatusWriteMask = 0x000DE133;   // SD (bit 31) follows FS: read-only
constexpr uint32_t kMstatusWriteMask = 0x007FF9BB;
#else
constexpr uint32_t kSstatusWriteMask = 0x800DE133;
constexpr uint32_t kMstatusWriteMask = 0x807FF9BB;
#endif
constexpr uint32_t kMstatusReset = 0;
#endif
// Interrupt CSRs per the privileged spec (no N extension, same as the function model): mie holds SSIE/MSIE/STIE/MTIE/SEIE/MEIE;
// the supervisor views sie / sip show only the bits delegated by mideleg (mideleg holds SSI/STI/SEI), and sip can only write SSIP.
constexpr uint32_t kMipMask = 0x00000aaa, kMieMask = 0x00000aaa, kDelegBits = 0x00000222;

// Interrupts that can be taken now, split by the privilege level that handles them.
struct IrqState {
  bool m_sw, m_timer, m_ext;
  bool s_sw, s_timer, s_ext;
  bool to_m() const { return m_sw || m_timer || m_ext; }
  bool to_s() const { return s_sw || s_timer || s_ext; }
};

IrqState eval_interrupts(xlen_t mstatus, xlen_t mie, xlen_t mip, xlen_t mideleg, uint32_t priv) {
  const bool m_enable = priv < 3 || (mstatus & MSTATUS_MIE);
  const bool s_enable = priv < 2 && (priv < 1 || (priv == 1 && (mstatus & MSTATUS_SIE)));
  // pending and enabled, not delegated -> machine level
  auto to_m = [&](uint32_t bit) { return (mip & bit) && (mie & bit) && !(mideleg & bit) && m_enable; };
  // pending and enabled and delegated, or the supervisor's own pending bit -> supervisor level
  auto to_s = [&](uint32_t m_bit, uint32_t s_bit) {
    return (((mip & m_bit) && (mie & m_bit) && (mideleg & m_bit)) || ((mip & s_bit) && (mie & s_bit))) && s_enable;
  };
  return {to_m(MIP_MSIP), to_m(MIP_MTIP), to_m(MIP_MEIP),
          to_s(MIP_MSIP, MIP_SSIP), to_s(MIP_MTIP, MIP_STIP), to_s(MIP_MEIP, MIP_SEIP)};
}
}  // namespace

void Csr::init() {
#if XLEN == 64
#if FP
  CSR_RegFile[csr_misa] = CSR_RegFile_1[csr_misa] = (xlen_t(2) << 62) | (0x14112B | (kRvc ? 4 : 0));   // MXL = 2: U/S/M, RV64 I/A/M/B/F/D
#else
  CSR_RegFile[csr_misa] = CSR_RegFile_1[csr_misa] = (xlen_t(2) << 62) | (0x141103 | (kRvc ? 4 : 0));   // MXL = 2: U/S/M, RV64 I/A/M/B
#endif
#else
#if FP
  CSR_RegFile[csr_misa] = CSR_RegFile_1[csr_misa] = 0x4014112B | (kRvc ? 4 : 0);                    // U/S/M, RV32 I/A/M/B/F/D
#else
  CSR_RegFile[csr_misa] = CSR_RegFile_1[csr_misa] = 0x40141103 | (kRvc ? 4 : 0);                    // U/S/M, RV32 I/A/M/B
#endif
#endif
  CSR_RegFile[csr_mstatus] = CSR_RegFile_1[csr_mstatus] = kMstatusReset;
  CSR_RegFile[csr_sstatus] = CSR_RegFile_1[csr_sstatus] = kMstatusReset & kSstatusMask;
}

// The working copies already equal the registered state (seq() copies them back).
void Csr::comb_begin() {
  BEGIN_EQ(CSR_RegFile_1, CSR_RegFile);
  BEGIN_EQ(privilege_1, privilege);
  BEGIN_EQ(csr_idx_1, csr_idx);
  BEGIN_EQ(csr_wdata_1, csr_wdata);
  BEGIN_EQ(csr_wcmd_1, csr_wcmd);
  BEGIN_EQ(csr_we_1, csr_we);
}

void Csr::comb_csr_status() {
  wires->csr_status.mstatus = CSR_RegFile[csr_mstatus];
  wires->csr_status.sstatus = CSR_RegFile[csr_sstatus];
  wires->csr_status.satp = CSR_RegFile[csr_satp];
  wires->csr_status.privilege = privilege;
}

void Csr::comb_csr_read() {
  if (wires->exe2csr.re) wires->csr2exe.rdata = CSR_RegFile[cvt_number_to_csr(wires->exe2csr.idx)];
}

void Csr::comb_interrupt() {
  const IrqState irq = eval_interrupts(CSR_RegFile[csr_mstatus], CSR_RegFile[csr_mie], CSR_RegFile[csr_mip],
                                       CSR_RegFile[csr_mideleg], privilege);
  wires->csr2rob.interrupt_req = irq.to_m() || irq.to_s();
}

// External interrupt lines set/clear their mip bit; sip mirrors the supervisor-visible bits.
void Csr::comb_interrupt_inject() {
  const CsrInterruptInjectIO &io = wires->csr_interrupt_inject_io;
  const struct { bool valid, pending; uint32_t bit; } lines[] = {
      {io.external_irq_pending_valid, io.external_irq_pending, MIP_MEIP},
      {io.supervisor_external_irq_pending_valid, io.supervisor_external_irq_pending, MIP_SEIP},
      {io.timer_irq_pending_valid, io.timer_irq_pending, MIP_MTIP},
  };
  for (const auto &l : lines) {
    if (!l.valid) continue;
    CSR_RegFile_1[csr_mip] = l.pending ? (CSR_RegFile_1[csr_mip] | l.bit) : (CSR_RegFile_1[csr_mip] & ~l.bit);
    CSR_RegFile_1[csr_sip] = CSR_RegFile_1[csr_mip] & (CSR_RegFile_1[csr_mideleg] & kDelegBits);
  }
}

// Trap entry / xRET: next-cycle status, privilege, cause, tval and the redirect target. Machine traps take priority
// over supervisor traps; a trap is delegated to S according to medeleg (interrupts: mideleg inside eval_interrupts).
void Csr::comb_exception() {
  const RobBroadcastIO &rb = wires->rob_bcast;
  xlen_t mstatus = CSR_RegFile[csr_mstatus];
  xlen_t sstatus = CSR_RegFile[csr_sstatus];
  const xlen_t medeleg = CSR_RegFile[csr_medeleg];
  const xlen_t mtvec = CSR_RegFile[csr_mtvec], stvec = CSR_RegFile[csr_stvec];

  const bool bp = rb.breakpoint, ecall = rb.ecall, pf_inst = rb.page_fault_inst, pf_load = rb.page_fault_load,
             pf_store = rb.page_fault_store, illegal = rb.illegal_inst;
  const bool pf = pf_inst || pf_load || pf_store;
  auto deleg = [&](int bit) { return (medeleg >> bit) & 1; };
  const bool d_uecall = deleg(8), d_secall = deleg(9), d_pf_inst = deleg(12), d_pf_load = deleg(13),
             d_pf_store = deleg(15), d_illegal = deleg(2),
             d_bp = deleg(3) && privilege != 3;  // a trap taken in M mode is never delegated

  const IrqState irq = eval_interrupts(mstatus, CSR_RegFile[csr_mie], CSR_RegFile[csr_mip], CSR_RegFile[csr_mideleg], privilege);
  const bool resp = wires->rob2csr.interrupt_resp;
  const bool m_irq = irq.to_m() && resp, s_irq = irq.to_s() && resp;

  const bool u_ecall = ecall && privilege == 0, s_ecall = ecall && privilege == 1, m_ecall = ecall && privilege == 3;
  const bool m_trap = m_irq || (u_ecall && !d_uecall) || (s_ecall && !d_secall) || m_ecall ||
                      (pf_inst && !d_pf_inst) || (pf_load && !d_pf_load) || (pf_store && !d_pf_store) ||
                      (illegal && !d_illegal) || (bp && !d_bp);
  const bool s_trap = s_irq || (u_ecall && d_uecall) || (s_ecall && d_secall) || (pf_inst && d_pf_inst) ||
                      (pf_load && d_pf_load) || (pf_store && d_pf_store) || (illegal && d_illegal) || (bp && d_bp);

  // Vectored mode: interrupts jump to base + 4 * cause.
  auto trap_target = [&](xlen_t tvec, xlen_t cause) -> xlen_t {
    const xlen_t msb = xlen_t(1) << (XLEN - 1);   // interrupt flag of mcause / scause
    if ((tvec & 1) && (cause & msb)) return (tvec & ~xlen_t(3)) + 4 * (cause & ~msb);
    return tvec;
  };

  if (m_trap) {
    CSR_RegFile_1[csr_mepc] = rb.pc;
    const bool is_irq = irq.to_m();  // same selection as the hardware: the cause reports the raw interrupt state
    xlen_t cause = is_irq ? xlen_t(1) << (XLEN - 1) : xlen_t(0);
    cause += irq.m_sw ? 3
             : irq.m_timer ? 7
             : (irq.m_ext || (m_ecall && !d_uecall)) ? 11
             : (u_ecall && !d_uecall) ? 8
             : (s_ecall && !d_secall) ? 9
             : (pf_inst && !d_pf_inst) ? 12
             : (pf_load && !d_pf_load) ? 13
             : (pf_store && !d_pf_store) ? 15
             : (bp && !d_bp) ? 3
             : illegal ? 2
                       : 0;
    CSR_RegFile_1[csr_mcause] = cause;
    wires->csr2front.trap_pc = trap_target(mtvec, cause);
    if (trace_traps()) std::fprintf(stderr, "[TRAP] M cause=0x%llx epc=0x%llx tval=0x%llx target=0x%llx priv=%u\n", (unsigned long long)cause, (unsigned long long)rb.pc, (unsigned long long)rb.trap_val, (unsigned long long)wires->csr2front.trap_pc, (unsigned)privilege);

    mstatus = (mstatus & ~MSTATUS_MPP) | ((privilege & 0x3) << 11);  // MPP = privilege
    mstatus = (mstatus & MSTATUS_MIE) ? (mstatus | MSTATUS_MPIE) : (mstatus & ~MSTATUS_MPIE);  // MPIE = MIE
    mstatus &= ~MSTATUS_MIE;
    CSR_RegFile_1[csr_mstatus] = mstatus;
    CSR_RegFile_1[csr_sstatus] = mstatus & kSstatusMask;
    privilege_1 = RISCV_MODE_M;
    CSR_RegFile_1[csr_mtval] = (pf || illegal || bp) ? rb.trap_val : 0;
  } else if (s_trap) {
    CSR_RegFile_1[csr_sepc] = rb.pc;
    xlen_t cause = irq.to_s() ? xlen_t(1) << (XLEN - 1) : xlen_t(0);
    cause += (irq.s_ext || (s_ecall && d_secall)) ? 9
             : irq.s_timer ? 5
             : (u_ecall && d_uecall) ? 8
             : irq.s_sw ? 1
             : (pf_inst && d_pf_inst) ? 12
             : (pf_load && d_pf_load) ? 13
             : (pf_store && d_pf_store) ? 15
             : (bp && d_bp) ? 3
             : (illegal && d_illegal) ? 2
                                      : 0;
    CSR_RegFile_1[csr_scause] = cause;
    wires->csr2front.trap_pc = trap_target(stvec, cause);
    if (trace_traps()) std::fprintf(stderr, "[TRAP] S cause=0x%llx epc=0x%llx tval=0x%llx target=0x%llx priv=%u\n", (unsigned long long)cause, (unsigned long long)rb.pc, (unsigned long long)rb.trap_val, (unsigned long long)wires->csr2front.trap_pc, (unsigned)privilege);

    sstatus = privilege == 1 ? (sstatus | MSTATUS_SPP) : (sstatus & ~MSTATUS_SPP);  // SPP = privilege
    sstatus = (sstatus & MSTATUS_SIE) ? (sstatus | MSTATUS_SPIE) : (sstatus & ~MSTATUS_SPIE);  // SPIE = SIE
    sstatus &= ~MSTATUS_SIE;
    CSR_RegFile_1[csr_sstatus] = sstatus;
    CSR_RegFile_1[csr_mstatus] = (CSR_RegFile[csr_mstatus] & ~kSstatusWriteMask) | (sstatus & kSstatusWriteMask);
    privilege_1 = 1;
    CSR_RegFile_1[csr_stval] = (pf || illegal || bp) ? rb.trap_val : 0;
  } else if (rb.mret) {
    mstatus = (mstatus & MSTATUS_MPIE) ? (mstatus | MSTATUS_MIE) : (mstatus & ~MSTATUS_MIE);  // MIE = MPIE
    privilege_1 = GET_MPP(mstatus);
    mstatus |= MSTATUS_MPIE;
    mstatus &= ~MSTATUS_MPP;  // MPP = U
    CSR_RegFile_1[csr_mstatus] = mstatus;
    CSR_RegFile_1[csr_sstatus] = mstatus & kSstatusMask;
    wires->csr2front.epc = CSR_RegFile[csr_mepc];
  } else if (rb.sret) {
    sstatus = (sstatus & MSTATUS_SPIE) ? (sstatus | MSTATUS_SIE) : (sstatus & ~MSTATUS_SIE);  // SIE = SPIE
    privilege_1 = GET_SPP(sstatus);
    sstatus |= MSTATUS_SPIE;
    sstatus &= ~MSTATUS_SPP;  // SPP = U
    CSR_RegFile_1[csr_sstatus] = sstatus;
    CSR_RegFile_1[csr_mstatus] = (CSR_RegFile[csr_mstatus] & ~kSstatusWriteMask) | (sstatus & kSstatusWriteMask);
    wires->csr2front.epc = CSR_RegFile[csr_sepc];
  }
}

// A CSR instruction latches its write at execute and performs it when it commits (so a squashed instruction has no
// effect). The aliases (sie/sip/sstatus) update the machine register and the derived supervisor view.
void Csr::comb_csr_write() {
  if (wires->exe2csr.we) {
    csr_we_1 = wires->exe2csr.we;
    csr_idx_1 = wires->exe2csr.idx;
    csr_wcmd_1 = wires->exe2csr.wcmd;
    csr_wdata_1 = wires->exe2csr.wdata;
  }
  if (!(wires->rob2csr.commit && csr_we)) return;

  const xlen_t old_val = CSR_RegFile[cvt_number_to_csr(csr_idx)];
  if (csr_wcmd == CSR_S) csr_wdata = (csr_wdata | old_val);
  else if (csr_wcmd == CSR_C) csr_wdata = (~csr_wdata & old_val);

  // machine register `m`, written through its supervisor alias with `s_mask` or directly with `m_mask`
  auto write_pair = [&](int m, int s, bool via_s, xlen_t m_mask, xlen_t s_mask, xlen_t s_view) {
    CSR_RegFile_1[m] = via_s ? (CSR_RegFile[m] & ~s_mask) | (csr_wdata & s_mask) : (m == csr_mstatus ? (CSR_RegFile[m] & ~m_mask) | (csr_wdata & m_mask) : csr_wdata & m_mask);
    CSR_RegFile_1[s] = CSR_RegFile_1[m] & s_view;   // the supervisor view: sstatus shows UXL (read-only) that a write cannot change
  };
  const xlen_t deleg = CSR_RegFile[csr_mideleg] & kDelegBits;
  if (csr_idx == number_mie || csr_idx == number_sie) {
    write_pair(csr_mie, csr_sie, csr_idx == number_sie, kMieMask, deleg, deleg);
  } else if (csr_idx == number_mip || csr_idx == number_sip) {
    write_pair(csr_mip, csr_sip, csr_idx == number_sip, kMipMask, csr_idx == number_sip ? (MIP_SSIP & deleg) : deleg,
               csr_idx == number_sip ? (MIP_SSIP & deleg) : deleg);
    CSR_RegFile_1[csr_sip] = CSR_RegFile_1[csr_mip] & deleg;
  } else if (csr_idx == number_mideleg) {
    CSR_RegFile_1[csr_mideleg] = csr_wdata & kDelegBits;
    CSR_RegFile_1[csr_sie] = CSR_RegFile[csr_mie] & CSR_RegFile_1[csr_mideleg];
    CSR_RegFile_1[csr_sip] = CSR_RegFile[csr_mip] & CSR_RegFile_1[csr_mideleg];
  } else if (csr_idx == number_mstatus || csr_idx == number_sstatus) {
    write_pair(csr_mstatus, csr_sstatus, csr_idx == number_sstatus, kMstatusWriteMask, kSstatusWriteMask, kSstatusMask);
#if FP
    {  // SD follows FS
      const xlen_t sd = xlen_t(1) << (XLEN - 1);
      if (((CSR_RegFile_1[csr_mstatus] >> 13) & 3) == 3) CSR_RegFile_1[csr_mstatus] |= sd; else CSR_RegFile_1[csr_mstatus] &= ~sd;
      CSR_RegFile_1[csr_sstatus] = CSR_RegFile_1[csr_mstatus] & kSstatusMask;
    }
#endif
  } else if (csr_idx == number_satp) {
    const xlen_t old_satp = CSR_RegFile[csr_satp];
    // rv64: only Bare (0) and Sv39 (8) are implemented; any other mode leaves satp unchanged (as in the function model)
    if (!(XLEN == 64 && (csr_wdata >> 60) != 0 && (csr_wdata >> 60) != 8)) CSR_RegFile_1[csr_satp] = csr_wdata;
    if (old_satp != csr_wdata)
      debug_ptw_trace::record_satp_write(static_cast<uint32_t>(old_satp), static_cast<uint32_t>(csr_wdata), static_cast<uint8_t>(privilege));
  } else {
    CSR_RegFile_1[cvt_number_to_csr(csr_idx)] = csr_wdata;
  }
  csr_we_1 = false;
}

void Csr::seq() {
#if FP
  if (fs_dirty_req) {   // FS = Dirty, SD: applied on top of whatever else changed mstatus this cycle
    CSR_RegFile_1[csr_mstatus] |= (xlen_t(3) << 13) | (xlen_t(1) << (XLEN - 1));
    CSR_RegFile_1[csr_sstatus] = CSR_RegFile_1[csr_mstatus] & kSstatusMask;
    fs_dirty_req = false;
  }
#endif
  for (int i = 0; i < CSR_NUM; i++) CSR_RegFile[i] = CSR_RegFile_1[i];
  csr_idx = csr_idx_1;
  csr_wdata = csr_wdata_1;
  csr_wcmd = csr_wcmd_1;
  csr_we = csr_we_1;
  privilege = privilege_1;
}

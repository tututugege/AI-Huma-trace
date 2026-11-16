#include "BackPorts.h"
#include "Csr.h"
#include "config.h"
#include "ref.h"
#include <cstdio>
#include <cstdlib>

namespace {
// TB_TRACE_TRAP=1 prints every trap the CSR model takes (cause, epc, tval, target)
// mstatus bits visible through sstatus / writable (as in the function model: rv64 has no SD tracking, UXL = SXL = 2 are read-only)
constexpr xlen_t kSstatusMask = 0x80000003000DE133ull;   // SD included (FS = Dirty)
constexpr xlen_t kSstatusWriteMask = 0x00000000000DE133ull;
constexpr xlen_t kMstatusWriteMask = 0x00000000007FF9BBull;
constexpr xlen_t kMstatusReset = 0x0000000A00000000ull;
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
  q_.CSR_RegFile[csr_misa] = d.CSR_RegFile[csr_misa] = (xlen_t(2) << 62) | (0x14112B | (kRvc ? 4 : 0));   // MXL = 2: U/S/M, RV64 I/A/M/B/F/D
  q_.CSR_RegFile[csr_mstatus] = d.CSR_RegFile[csr_mstatus] = kMstatusReset;
  q_.CSR_RegFile[csr_sstatus] = d.CSR_RegFile[csr_sstatus] = kMstatusReset & kSstatusMask;
}

void Csr::comb_csr_status() {
  const Regs &q = q_;
  out.csr_status->mstatus = q.CSR_RegFile[csr_mstatus];
  out.csr_status->sstatus = q.CSR_RegFile[csr_sstatus];
  out.csr_status->satp = q.CSR_RegFile[csr_satp];
  out.csr_status->privilege = q.privilege;
}

void Csr::comb_interrupt() {
  const Regs &q = q_;
  const IrqState irq = eval_interrupts(q.CSR_RegFile[csr_mstatus], q.CSR_RegFile[csr_mie], q.CSR_RegFile[csr_mip],
                                       q.CSR_RegFile[csr_mideleg], q.privilege);
  out.csr2rob.interrupt_req = irq.to_m() || irq.to_s();
}

// External interrupt lines set/clear their mip bit; sip mirrors the supervisor-visible bits.
void Csr::comb_interrupt_inject() {
  const CsrInterruptInjectIO &io = (*in.csr_interrupt_inject_io);
  const struct { bool valid, pending; uint32_t bit; } lines[] = {
      {io.external_irq_pending_valid, io.external_irq_pending, MIP_MEIP},
      {io.supervisor_external_irq_pending_valid, io.supervisor_external_irq_pending, MIP_SEIP},
      {io.timer_irq_pending_valid, io.timer_irq_pending, MIP_MTIP},
  };
  for (const auto &l : lines) {
    if (!l.valid) continue;
    d.CSR_RegFile[csr_mip] = l.pending ? (d.CSR_RegFile[csr_mip] | l.bit) : (d.CSR_RegFile[csr_mip] & ~l.bit);
    d.CSR_RegFile[csr_sip] = d.CSR_RegFile[csr_mip] & (d.CSR_RegFile[csr_mideleg] & kDelegBits);
  }
  // FS = Dirty, SD (an FP instruction committed: top2csr.fs_dirty from BackTop::comb_outputs): on top of whatever else
  // changed mstatus this cycle; this is the CSR's last phase of the cycle
  if (in.top2csr->fs_dirty) {
    d.CSR_RegFile[csr_mstatus] |= (xlen_t(3) << 13) | (xlen_t(1) << (XLEN - 1));
    d.CSR_RegFile[csr_sstatus] = d.CSR_RegFile[csr_mstatus] & kSstatusMask;
  }
}

// Trap entry / xRET: next-cycle status, privilege, cause, tval and the redirect target. Machine traps take priority
// over supervisor traps; a trap is delegated to S according to medeleg (interrupts: mideleg inside eval_interrupts).
void Csr::exception() {
  const Regs &q = q_;
  const RobBroadcastIO &rb = (*in.rob_bcast);
  xlen_t mstatus = q.CSR_RegFile[csr_mstatus];
  xlen_t sstatus = q.CSR_RegFile[csr_sstatus];
  const xlen_t medeleg = q.CSR_RegFile[csr_medeleg];
  const xlen_t mtvec = q.CSR_RegFile[csr_mtvec], stvec = q.CSR_RegFile[csr_stvec];

  const bool bp = rb.breakpoint, ecall = rb.ecall, pf_inst = rb.page_fault_inst, pf_load = rb.page_fault_load,
             pf_store = rb.page_fault_store, illegal = rb.illegal_inst;
  const bool pf = pf_inst || pf_load || pf_store;
  auto deleg = [&](int bit) { return (medeleg >> bit) & 1; };
  const bool d_uecall = deleg(8), d_secall = deleg(9), d_pf_inst = deleg(12), d_pf_load = deleg(13),
             d_pf_store = deleg(15), d_illegal = deleg(2),
             d_bp = deleg(3) && q.privilege != 3;  // a trap taken in M mode is never delegated

  const IrqState irq = eval_interrupts(mstatus, q.CSR_RegFile[csr_mie], q.CSR_RegFile[csr_mip], q.CSR_RegFile[csr_mideleg], q.privilege);
  const bool resp = in.rob2csr->interrupt_resp;
  const bool m_irq = irq.to_m() && resp, s_irq = irq.to_s() && resp;

  const bool u_ecall = ecall && q.privilege == 0, s_ecall = ecall && q.privilege == 1, m_ecall = ecall && q.privilege == 3;
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
    d.CSR_RegFile[csr_mepc] = rb.pc;
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
    d.CSR_RegFile[csr_mcause] = cause;
    out.csr2front.trap_pc = trap_target(mtvec, cause);

    mstatus = (mstatus & ~MSTATUS_MPP) | ((q.privilege & 0x3) << 11);  // MPP = q.privilege
    mstatus = (mstatus & MSTATUS_MIE) ? (mstatus | MSTATUS_MPIE) : (mstatus & ~MSTATUS_MPIE);  // MPIE = MIE
    mstatus &= ~MSTATUS_MIE;
    d.CSR_RegFile[csr_mstatus] = mstatus;
    d.CSR_RegFile[csr_sstatus] = mstatus & kSstatusMask;
    d.privilege = RISCV_MODE_M;
    d.CSR_RegFile[csr_mtval] = (pf || illegal || bp) ? rb.trap_val : 0;
  } else if (s_trap) {
    d.CSR_RegFile[csr_sepc] = rb.pc;
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
    d.CSR_RegFile[csr_scause] = cause;
    out.csr2front.trap_pc = trap_target(stvec, cause);

    sstatus = q.privilege == 1 ? (sstatus | MSTATUS_SPP) : (sstatus & ~MSTATUS_SPP);  // SPP = q.privilege
    sstatus = (sstatus & MSTATUS_SIE) ? (sstatus | MSTATUS_SPIE) : (sstatus & ~MSTATUS_SPIE);  // SPIE = SIE
    sstatus &= ~MSTATUS_SIE;
    d.CSR_RegFile[csr_sstatus] = sstatus;
    d.CSR_RegFile[csr_mstatus] = (q.CSR_RegFile[csr_mstatus] & ~kSstatusWriteMask) | (sstatus & kSstatusWriteMask);
    d.privilege = 1;
    d.CSR_RegFile[csr_stval] = (pf || illegal || bp) ? rb.trap_val : 0;
  } else if (rb.mret) {
    mstatus = (mstatus & MSTATUS_MPIE) ? (mstatus | MSTATUS_MIE) : (mstatus & ~MSTATUS_MIE);  // MIE = MPIE
    d.privilege = GET_MPP(mstatus);
    mstatus |= MSTATUS_MPIE;
    mstatus &= ~MSTATUS_MPP;  // MPP = U
    d.CSR_RegFile[csr_mstatus] = mstatus;
    d.CSR_RegFile[csr_sstatus] = mstatus & kSstatusMask;
    out.csr2front.epc = q.CSR_RegFile[csr_mepc];
  } else if (rb.sret) {
    sstatus = (sstatus & MSTATUS_SPIE) ? (sstatus | MSTATUS_SIE) : (sstatus & ~MSTATUS_SIE);  // SIE = SPIE
    d.privilege = GET_SPP(sstatus);
    sstatus |= MSTATUS_SPIE;
    sstatus &= ~MSTATUS_SPP;  // SPP = U
    d.CSR_RegFile[csr_sstatus] = sstatus;
    d.CSR_RegFile[csr_mstatus] = (q.CSR_RegFile[csr_mstatus] & ~kSstatusWriteMask) | (sstatus & kSstatusWriteMask);
    out.csr2front.epc = q.CSR_RegFile[csr_sepc];
  }
}

// A CSR instruction latches its write at execute and performs it when it commits (so a squashed instruction has no
// effect). The aliases (sie/sip/sstatus) update the machine register and the derived supervisor view.
void Csr::csr_write() {
  const Regs &q = q_;
  if (in.exe2csr->we) {
    d.csr_we = in.exe2csr->we;
    d.csr_idx = in.exe2csr->idx;
    d.csr_wcmd = in.exe2csr->wcmd;
    d.csr_wdata = in.exe2csr->wdata;
  }
  if (!(in.rob2csr->commit && q.csr_we)) return;

  const xlen_t old_val = q.CSR_RegFile[cvt_number_to_csr(q.csr_idx)];
  xlen_t wdata = q.csr_wdata;   // the value written (CSRRS / CSRRC: combined with the old value)
  if (q.csr_wcmd == CSR_S) wdata = (wdata | old_val);
  else if (q.csr_wcmd == CSR_C) wdata = (~wdata & old_val);

  // machine register `m`, written through its supervisor alias with `s_mask` or directly with `m_mask`
  auto write_pair = [&](int m, int s, bool via_s, xlen_t m_mask, xlen_t s_mask, xlen_t s_view) {
    d.CSR_RegFile[m] = via_s ? (q.CSR_RegFile[m] & ~s_mask) | (wdata & s_mask) : (m == csr_mstatus ? (q.CSR_RegFile[m] & ~m_mask) | (wdata & m_mask) : wdata & m_mask);
    d.CSR_RegFile[s] = d.CSR_RegFile[m] & s_view;   // the supervisor view: sstatus shows UXL (read-only) that a write cannot change
  };
  const xlen_t deleg = q.CSR_RegFile[csr_mideleg] & kDelegBits;
  if (q.csr_idx == number_mie || q.csr_idx == number_sie) {
    write_pair(csr_mie, csr_sie, q.csr_idx == number_sie, kMieMask, deleg, deleg);
  } else if (q.csr_idx == number_mip || q.csr_idx == number_sip) {
    write_pair(csr_mip, csr_sip, q.csr_idx == number_sip, kMipMask, q.csr_idx == number_sip ? (MIP_SSIP & deleg) : deleg,
               q.csr_idx == number_sip ? (MIP_SSIP & deleg) : deleg);
    d.CSR_RegFile[csr_sip] = d.CSR_RegFile[csr_mip] & deleg;
  } else if (q.csr_idx == number_mideleg) {
    d.CSR_RegFile[csr_mideleg] = wdata & kDelegBits;
    d.CSR_RegFile[csr_sie] = q.CSR_RegFile[csr_mie] & d.CSR_RegFile[csr_mideleg];
    d.CSR_RegFile[csr_sip] = q.CSR_RegFile[csr_mip] & d.CSR_RegFile[csr_mideleg];
  } else if (q.csr_idx == number_mstatus || q.csr_idx == number_sstatus) {
    write_pair(csr_mstatus, csr_sstatus, q.csr_idx == number_sstatus, kMstatusWriteMask, kSstatusWriteMask, kSstatusMask);
    {  // SD follows FS
      const xlen_t sd = xlen_t(1) << (XLEN - 1);
      if (((d.CSR_RegFile[csr_mstatus] >> 13) & 3) == 3) d.CSR_RegFile[csr_mstatus] |= sd; else d.CSR_RegFile[csr_mstatus] &= ~sd;
      d.CSR_RegFile[csr_sstatus] = d.CSR_RegFile[csr_mstatus] & kSstatusMask;
    }
  } else if (q.csr_idx == number_satp) {
    // rv64: only Bare (0) and Sv39 (8) are implemented; any other mode leaves satp unchanged (as in the function model)
    if (!((wdata >> 60) != 0 && (wdata >> 60) != 8)) d.CSR_RegFile[csr_satp] = wdata;
  } else {
    d.CSR_RegFile[cvt_number_to_csr(q.csr_idx)] = wdata;
  }
  d.csr_we = false;
}

void Csr::seq() {
  q_ = d;
}

// exceptions and the CSR write of the committing instruction
void Csr::comb_commit() {
  exception();
  csr_write();
}

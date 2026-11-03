// Csr differential: random machine state and stimulus through the new Csr and the frozen reference; outputs and state must agree every cycle.
#include "Csr.h"
#include "CsrRef.h"
#include "Wires.h"
#include <cstdio>
#include <cstring>
#include <random>

long long sim_time = 0;
static bool same_status(const CsrStatusIO &x, const CsrStatusIO &y) { return x.mstatus == y.mstatus && x.sstatus == y.sstatus && x.satp == y.satp && x.privilege == y.privilege; }  // field-wise: padding is not initialised
int main(int argc, char **argv) {
  long cycles = argc > 1 ? atol(argv[1]) : 20000000;
  std::mt19937_64 rng(argc > 2 ? atol(argv[2]) : 1);
  Wires wa, wb;
  Csr a; CsrRef b;
  a.wires = &wa; b.wires = &wb;
  a.init(); b.init();
  static const int nums[] = {number_mtvec, number_mepc, number_mcause, number_mie, number_mip, number_mtval, number_mscratch, number_mstatus,
    number_mideleg, number_medeleg, number_sepc, number_stvec, number_scause, number_sscratch, number_stval, number_sstatus, number_sie,
    number_sip, number_satp, number_mhartid, number_misa};
  auto bit = [&](int pct) { return (int)(rng() % 100) < pct; };
  // dense-bit words: the interesting CSR bits are scattered, so bias towards all-ones / all-zeros / random
  auto word = [&]() -> uint32_t { switch (rng() % 4) { case 0: return 0; case 1: return ~0u; default: return (uint32_t)rng(); } };
  long bad = 0;
  auto cmp = [&](const char *ph) {
    bool ok = same_status(wa.csr_status, wb.csr_status) && !memcmp(&wa.csr2exe, &wb.csr2exe, sizeof(wa.csr2exe)) &&
              !memcmp(&wa.csr2rob, &wb.csr2rob, sizeof(wa.csr2rob)) && !memcmp(&wa.csr2front, &wb.csr2front, sizeof(wa.csr2front)) &&
              !memcmp(a.CSR_RegFile, b.CSR_RegFile, sizeof(a.CSR_RegFile)) && !memcmp(a.CSR_RegFile_1, b.CSR_RegFile_1, sizeof(a.CSR_RegFile_1)) &&
              a.privilege == b.privilege && a.privilege_1 == b.privilege_1 && a.csr_idx == b.csr_idx && a.csr_wdata == b.csr_wdata &&
              a.csr_wcmd == b.csr_wcmd && a.csr_we == b.csr_we && a.csr_idx_1 == b.csr_idx_1 && a.csr_wdata_1 == b.csr_wdata_1 &&
              a.csr_wcmd_1 == b.csr_wcmd_1 && a.csr_we_1 == b.csr_we_1;
    if (!ok && bad++ < 5) {
      printf("MISMATCH in %s: status=%d c2e=%d c2r=%d c2f=%d reg=%d reg1=%d priv=%d/%d w=%d/%d\n", ph, same_status(wa.csr_status, wb.csr_status), !memcmp(&wa.csr2exe, &wb.csr2exe, sizeof(wa.csr2exe)), !memcmp(&wa.csr2rob, &wb.csr2rob, sizeof(wa.csr2rob)), !memcmp(&wa.csr2front, &wb.csr2front, sizeof(wa.csr2front)), !memcmp(a.CSR_RegFile, b.CSR_RegFile, sizeof(a.CSR_RegFile)), !memcmp(a.CSR_RegFile_1, b.CSR_RegFile_1, sizeof(a.CSR_RegFile_1)), a.privilege == b.privilege, a.privilege_1 == b.privilege_1, a.csr_wdata == b.csr_wdata, a.csr_wdata_1 == b.csr_wdata_1);
    }
  };
  for (long c = 0; c < cycles && bad == 0; c++) {
    if (bit(5)) {  // fresh machine state
      for (int i = 0; i < CSR_NUM; i++) a.CSR_RegFile[i] = a.CSR_RegFile_1[i] = b.CSR_RegFile[i] = b.CSR_RegFile_1[i] = word();
      a.privilege = a.privilege_1 = b.privilege = b.privilege_1 = (int[]){0, 1, 3, 3}[rng() % 4];
    }
    // stimulus: sparse, so that single events (one fault, one ret) are the common case
    wa.exe2csr = {}; wa.rob2csr = {}; wa.rob_bcast = {}; wa.csr_interrupt_inject_io = {};
    wa.exe2csr.re = bit(30); wa.exe2csr.we = bit(40);
    wa.exe2csr.idx = nums[rng() % 21]; wa.exe2csr.wdata = word(); wa.exe2csr.wcmd = rng() % 4;
    wa.rob2csr.commit = bit(60); wa.rob2csr.interrupt_resp = bit(50);
    auto &r = wa.rob_bcast;
    r.ecall = bit(10); r.page_fault_inst = bit(6); r.page_fault_load = bit(6); r.page_fault_store = bit(6); r.illegal_inst = bit(6);
    r.mret = bit(10); r.sret = bit(10); r.trap_val = word(); r.pc = word();
    auto &io = wa.csr_interrupt_inject_io;
    io.external_irq_pending_valid = bit(20); io.external_irq_pending = bit(50);
    io.supervisor_external_irq_pending_valid = bit(20); io.supervisor_external_irq_pending = bit(50);
    io.timer_irq_pending_valid = bit(20); io.timer_irq_pending = bit(50);
    wb.exe2csr = wa.exe2csr; wb.rob2csr = wa.rob2csr; wb.rob_bcast = wa.rob_bcast; wb.csr_interrupt_inject_io = wa.csr_interrupt_inject_io;
    a.comb_csr_status(); b.comb_csr_status(); cmp("status");
    a.comb_begin(); b.comb_begin();
    a.comb_interrupt(); b.comb_interrupt(); cmp("interrupt");
    a.comb_csr_read(); b.comb_csr_read(); cmp("read");
    a.comb_exception(); b.comb_exception(); cmp("exception");
    a.comb_csr_write(); b.comb_csr_write(); cmp("write");
    a.comb_interrupt_inject(); b.comb_interrupt_inject(); cmp("inject");
    a.seq(); b.seq(); cmp("seq");
  }
  printf("csr_diff: %ld mismatches\n", bad);
  return bad != 0;
}

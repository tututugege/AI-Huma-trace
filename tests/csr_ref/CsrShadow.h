// Lockstep hook: every Csr phase also runs on the frozen original (own Wires, same inputs); outputs and state must match.
#pragma once
#include "Csr.h"
#define CSR_PHASES(X) X(comb_begin) X(comb_csr_status) X(comb_interrupt) X(comb_csr_read) X(comb_exception) X(comb_csr_write) X(comb_interrupt_inject) X(seq)
enum class CsrPhase {
#define X(n) n,
  CSR_PHASES(X)
#undef X
};
void csr_shadow(Csr &x, CsrPhase p, const char *name);
#define CSR(f) do { csr->f(); csr_shadow(*csr, CsrPhase::f, #f); } while (0)

// Lockstep hook: every Exu phase also runs on the frozen original (own FUs, own Wires, same inputs); outputs and state digests must match.
#pragma once
#include "Exu.h"
#define EXU_PHASES(X) X(comb_begin) X(comb_ready) X(comb_to_csr) X(comb_pipeline) X(comb_exec) X(seq)
enum class ExuPhase {
#define X(n) n,
  EXU_PHASES(X)
#undef X
};
void exu_shadow(Exu &x, ExuPhase p, const char *name);
#define EXU(f) do { exu->f(); exu_shadow(*exu, ExuPhase::f, #f); } while (0)

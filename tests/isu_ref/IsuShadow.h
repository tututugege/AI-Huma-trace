// Lockstep hook: every Isu phase also runs on the frozen original (own issue queues, own Wires, same inputs); outputs and state digests must match.
#pragma once
#include "Isu.h"
#define ISU_PHASES(X) X(comb_begin) X(comb_ready) X(comb_enq) X(comb_issue) X(comb_awake) X(comb_calc_latency_next) X(comb_flush) X(seq)
enum class IsuPhase {
#define X(n) n,
  ISU_PHASES(X)
#undef X
};
void isu_shadow(Isu &x, IsuPhase p, const char *name);
#define ISU(f) do { isu->f(); isu_shadow(*isu, IsuPhase::f, #f); } while (0)

// Lockstep hook: every Dispatch phase also runs on the frozen original (own Wires, same inputs); outputs, shared uop-pool writes and a state digest must match.
#pragma once
#include "Dispatch.h"
#define DIS_PHASES(X) X(comb_begin) X(comb_alloc) X(comb_wake) X(comb_dispatch) X(comb_fire) X(comb_pipeline) X(seq)
enum class DisPhase {
#define X(n) n,
  DIS_PHASES(X)
#undef X
};
void dis_shadow(Dispatch &d, DisPhase p, const char *name);
#define DIS(f) do { dis->f(); dis_shadow(*dis, DisPhase::f, #f); } while (0)

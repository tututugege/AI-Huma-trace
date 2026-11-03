// Lockstep hook: every Ren phase also runs on the frozen original (own Wires, same inputs); outputs, the uop-pool slots the renamer wrote and the state must match.
#pragma once
#include "Ren.h"
#define REN_PHASES(X) X(comb_begin) X(comb_alloc) X(comb_rename) X(comb_fire) X(comb_pipeline) X(seq)
enum class RenPhase {
#define X(n) n,
  REN_PHASES(X)
#undef X
};
void ren_shadow(Ren &x, RenPhase p, const char *name);
#define REN(f) do { rename->f(); ren_shadow(*rename, RenPhase::f, #f); } while (0)

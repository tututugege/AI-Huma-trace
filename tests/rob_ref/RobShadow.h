// Lockstep hook: every Rob phase also runs on the frozen original Rob (with its own copy of the wires); outputs and state digests must match.
#pragma once
#include "Rob.h"
#define ROB_PHASES(X) X(comb_begin) X(comb_ready) X(comb_ftq_pc_req) X(comb_commit) X(comb_complete) X(comb_branch) X(comb_fire) X(comb_flush) X(seq)
enum class RobPhase {
#define X(n) n,
  ROB_PHASES(X)
#undef X
};
void rob_shadow(Rob &rob, RobPhase p, const char *name);
#define ROB(f) do { rob->f(); rob_shadow(*rob, RobPhase::f, #f); } while (0)

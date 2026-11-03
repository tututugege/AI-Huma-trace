// Lockstep hook: every PreIduQueue phase also runs on the frozen original (own Wires, same inputs); outputs and state must match (uop ids are masked: the reference does not allocate).
#pragma once
#include "PreIduQueue.h"
#define PRE_PHASES(X) X(comb_begin) X(comb_accept_front) X(comb_ftq_lookup_rob) X(comb_commit_info) X(comb_ftq_lookup_prf) X(comb_fire) X(seq)
enum class PrePhase {
#define X(n) n,
  PRE_PHASES(X)
#undef X
};
void pre_shadow(PreIduQueue &x, PrePhase p, const char *name);
#define PRE(f) do { pre->f(); pre_shadow(*pre, PrePhase::f, #f); } while (0)

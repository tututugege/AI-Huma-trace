// Lockstep hook: every Idu phase also runs on the frozen original (own Wires, same inputs); outputs, the uop-pool slots the decoder wrote and the state must match.
#pragma once
#include "Idu.h"
#define IDU_PHASES(X) X(comb_begin) X(comb_branch) X(comb_decode) X(comb_fire) X(seq)
enum class IduPhase {
#define X(n) n,
  IDU_PHASES(X)
#undef X
};
void idu_shadow(Idu &x, IduPhase p, const char *name);
#define IDU(f) do { idu->f(); idu_shadow(*idu, IduPhase::f, #f); } while (0)

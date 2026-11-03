// Lockstep hook: every LSU phase also runs on the frozen reference LSU; outputs and a state digest must match.
#pragma once
#include "RealLsu.h"
#define LSU_PHASES(X) X(comb_cal) X(comb_lsu2dis) X(comb_lsu2rob) X(comb_stq_commit) X(comb_mmio_in) X(comb_dcache2lsu_ldq) \
  X(comb_dcache2lsu_stq) X(comb_lsu2exe) X(comb_exe2lsu) X(comb_tlb_out) X(comb_tlb_in_ldq) X(comb_tlb_in_stq) X(comb_stlf) \
  X(comb_dis2lsu) X(comb_flush) X(comb_lsu2dcache_ldq) X(comb_lsu2dcache_stq) X(comb_mmio_out) X(comb_check) X(seq)
enum class LsuPhase {
#define X(n) n,
  LSU_PHASES(X)
#undef X
};
void lsu_shadow(RealLsu &lsu, LsuPhase p, const char *name);
#define LSU(f) do { lsu->f(); lsu_shadow(*lsu, LsuPhase::f, #f); } while (0)

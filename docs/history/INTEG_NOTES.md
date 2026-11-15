> Historical record of the `exp/integ` merge (2026-11-09; memory speculation EXP-018/020 + front end EXP-019). It is in main; the defaults listed below are still the defaults, but later merges (EXP-024 write buffer, EXP-025 ITT, EXP-027 uncached windows, EXP-028 compact ROB) changed the cycle numbers, so those here are no longer current: see `docs/MAIN_RECORDS.md` and [docs/PROJECT.md](docs/PROJECT.md). Branch names, commit ids and server paths are of that session.

# exp/integ: main (7f8456e) + memory speculation (exp/memspec-refs ef02c4c) + front end (exp/fe-refs b02eaf6)

## Merge
- Branch `exp/integ` from main `7f8456e` (EXP-016 DSE + EXP-017 IQ widths). `git merge exp/memspec-refs` (clean), then `git merge exp/fe-refs` (one conflict, `include/config.h`: both sides appended a block after CFG_LSU_MEMSPEC; resolved by keeping both). exp/fe-refs carried the stale EXP-018 memspec (2196889), which is an ancestor of exp/memspec-refs, so the newest LSU code wins with no manual work.
- Verified: `git diff exp/fe-refs -- front-end` is empty; the LSU/ROB/Dispatch/MsData/diff files equal exp/memspec-refs except `RealLsuPipe.cpp` (only the three `CFG_*` defaults below) and `TlbMmu.{cpp,h}` (`peek_translate` of fe-refs, used by FE_IPF only).
- Commits: merges 8ee4c9e, 207957a, baseline 8d647a8, docs 451c9dc (+ this notes file).

## Defaults (include/config.h, env overrides kept)
CFG_LSU_MEMSPEC=1, CFG_LSU_MS_DATA=1 (value path, active only with MEMSPEC), CFG_LSU_MDP_BITS=6, CFG_LSU_MDP_CLR=2000, new CFG_LSU_MDP_HW=2, CFG_LSU_MDP_MODE=0, CFG_LSU_MS_SLOTS=0, CFG_FE_RFAST=15, CFG_FE_BFAST=0. FE_PAIR and FE_IPF stay env-only, default 0 (not in the SPEC run).
RTL config export (`gen_cfg_pkg.py` dumps every upper-case `constexpr int`): LSU_MEMSPEC=1, LSU_MDP_BITS=6, LSU_MDP_CLR=2000, FE_RFAST=15, FE_BFAST=0. MDP_MODE/HW/SLOTS/MS_DATA are macros (not constexpr), so they are not exported; nothing else was added.

## Dhrystone (64 runs)
default 114,299 | `LSU_MEMSPEC=0 FE_RFAST=0` 123,519 (= main) | memspec only (FE_RFAST=0) 116,217 (= exp/memspec-refs binary at ef02c4c; the 115,896 of the notes is an older HW mode) | FE only (LSU_MEMSPEC=0) 119,644. Other big programs, default / old config: coremark 283,280 / 293,286; memory-stress 2,355,011 / 2,355,257; smc-div 10,781 / 10,990; smc-addi 10,221 / 10,432; memory-stress-small 328,606 (main 331,844); pf-stress 615,590 (main 617,730).

## Model gate
- Default config: 64 programs (gate 63 + load-store), all difftest Success, no crash, 0 TB_CHECK mismatches. 63 of 64 cycle counts changed (expected); `results/baseline/baseline.csv` regenerated. Biggest changes: vm_asid_test 84,239 -> 41,006 (-51%), vm_sfence_perm 45,942 -> 25,210 (-45%), vm_pagefault 32,973 -> 18,918 (-43%), max -9.3%; the rest -0.0% to -9%.
- Old-config regression: `LSU_MEMSPEC=0 FE_RFAST=0` with main's baseline: 63/63 equal. Big programs dhrystone 123,519 and coremark 293,286 equal main.
- trace-model Linux: boots from reset to `rv32 login:` (default config).
- Not run: small-LLC check (not requested), RTL side.

## Full SPEC (580 points, FASTWARM=1000000, PF_ENABLE=1, PMU=1, server 106.14.7.26)
Both runs from this session, same points/env/server, 580/580 success, no fail.txt. Baseline = main 7f8456e built on the server (`/root/integbase/res`), because the existing full results are EXP-015 main (lsubpu/full_new, no PMU) or scattered partial EXP-017 runs; the new baseline vs lsubpu/full_new geomean is 1.0000 (consistency check). Integ: `/root/integ/res`. Script: `scripts/integ/spec_cmp.py` (SimPoint-weighted CPI per program, 12-program geomean).

| program | pts | IPC main | IPC integ | change |
|---|---|---|---|---|
| 429.mcf | 79 | 0.5394 | 0.5523 | +2.40% |
| 403.gcc | 55 | 1.9366 | 2.0591 | +6.33% |
| 462.libquantum | 51 | 2.2475 | 2.4931 | +10.92% |
| 473.astar | 35 | 1.2500 | 1.5314 | +22.51% |
| 445.gobmk | 50 | 1.6986 | 1.9150 | +12.74% |
| 401.bzip2 | 76 | 1.5146 | 2.0895 | +37.95% |
| 458.sjeng | 28 | 1.7251 | 2.0656 | +19.73% |
| 483.xalancbmk | 51 | 2.0868 | 2.1600 | +3.51% |
| 464.h264ref | 37 | 1.5882 | 1.6465 | +3.67% |
| 471.omnetpp | 40 | 1.3886 | 1.4624 | +5.32% |
| 400.perlbench | 44 | 2.1725 | 2.3979 | +10.38% |
| 456.hmmer | 34 | 3.4736 | 3.5441 | +2.03% |

Geomean +11.02%. Estimated score 9.42 (last recorded, EXP-015) x 1.1102 = 10.46 points/GHz (SimPoint weights; main itself is -0.0% vs the EXP-015 full run so 9.42 is a valid start; reference times unverified). Unweighted per-point geomean +11.63%, min -4.3%, max x2.77. Points below -5%: none (worst: 401.bzip2 sp20 -4.3%).

Gain split (cheap, from existing 25/30-point precise-warmup sets, not additive): FE_RFAST=15 about +2.6% (ferefs 25 points; +BFAST 3 +5.0%, not in the defaults); memory speculation +5.35% (30 points, with LSU_MS_DATA=1). The full-run total (+11.0%) is larger than the sum, mainly bzip2/astar/sjeng.

## TMA level 1 (highest-weight point of each program, FASTWARM, PMU=1; mean of 12, % of slots)
Retiring 41.7 -> 43.2, BadSpec 9.3 -> 7.3, Frontend 29.7 -> 31.5, Backend 19.3 -> 18.0. Largest bad-speculation drops: gobmk 23.7 -> 18.3, sjeng 21.2 -> 15.3, astar 14.7 -> 11.7, mcf 18.5 -> 15.0; sjeng backend 20.0 -> 11.9. Raw: `/root/integ/tma`, `/root/integbase/tma` on the server.

## Notes
- Server memory: every simulator holds ~3.1 GB RSS, not only gcc/mcf, so 40 jobs nearly exhaust 121 GB; an initial 80-job start was cut to ~32-40.
- PMU=1 in `run_eval.sh` writes `pmu_*.csv` in the cwd (overwritten by concurrent points), so per-point TMA is not available from the full run; TMA came from separate top-weight runs (`/root/tma.sh`). The stray `/root/pmu_*.csv` was overwritten by my launch (cwd /root).

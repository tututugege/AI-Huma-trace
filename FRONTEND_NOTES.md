# Front-end exploration notes (branch exp/frontend, trace-model only)

Baseline = main cefaee4 (PF_ENABLE=1 PMU=1, precise warm-up, 10M + 10M, points in scripts/pf/points_fe.lst, 25 points).
Raw results: ~/personal/AI-Huma-data/frontend/srv/<tag> (server runs under /root/fe/res), comparison script
~/personal/AI-Huma-data/frontend/cmp.py (`python3 -I cmp.py <tag>`). Baselines: iqenq_local/p30_C (identical cycles to a local run of this
build, checked on gobmk sp3) and srv/base_c (server run of this build for xalancbmk, bzip2).

## Diagnostics added (env, no effect on cycles)
- FE_DIAG=1: histogram of recovery durations (redirect -> first valid instruction at dispatch) and per-cycle "why did the front end deliver nothing".
- FE_TRACE=1: per-cycle trace of the refetch -> dispatch path (first 24 cycles of the first redirect after cycle 40000).
- FE_FUSE_DIAG=1: instructions per fetch cycle of the committed stream under three fetch rules (limit study).

## Findings
- Redirect -> first instruction at dispatch is 8 cycles (all mispredicts, measured, no I$ miss). Chain: back2front register (1), BPU refetch cycle without S0 issue (1),
  S0 -> fetch-address FIFO -> ICache (1; ICache also drops ready for one cycle after a refetch), PTAB written at S_D (D=2) and only readable the cycle after (1),
  pre-IDU buffer (1), rename reg (1), dispatch reg (1).
- Fetch limit study (committed stream, perfect prediction, no bubbles): insts per fetch cycle with one basic block per cycle 4.7-5.0; continuing across a taken branch
  whose target is in the same 64 B line (17-26% of taken branches) 5.0-5.4 (+6%); two basic blocks per cycle 6.5-6.8 (+35%).

(results table below)

## Results (25 points, precise warm-up, IPC change vs main cefaee4; TMA FE / BadSp averages in the cmp.py output)
| switch | perl | gcc | gobmk | sjeng | astar | h264 | hmmer | xalan | bzip2 | geomean |
|---|---|---|---|---|---|---|---|---|---|---|
| FE_RFAST=15 (redirect path) | +3.1 | +0.6 | +3.2 | +5.4 | +5.2 | +0.3 | +0.5 | +1.7 | +0.5 | +2.3 |
| FE_BFAST=3 (same-cycle issue after L1/S_D override) | +0.2 | +4.9 | +4.2 | +4.5 | +1.7 | +0.1 | +0.2 | +3.6 | +0.2 | +2.1 |
| both | +5.1 | +5.6 | +6.3 | +10.1 | +7.1 | +0.5 | +0.8 | +3.0 | +0.7 | +4.3 |
Gate (switches off) 63/63; with FE_RFAST=15 FE_BFAST=3 all 63 difftest Success.
Remaining front-end stall (gobmk, after both): I$ miss busy 22% of front cycles, IFIFO empty 5%, refill 7%.

## FE_PAIR: dual-block fetch (2 blocks per cycle across a predicted-taken branch)
Design (everything stays one entry per cycle in every FIFO; a fetch group is the first block up to its taken branch followed by a second block, <= 8 slots):
- BPU S0: when the uBTB guess of block A is a taken branch (same 4 KiB page, target != fall-through), the block at the target is looked up in the same cycle
  (second uBTB read with the history / RAS after A, second main-predictor index set); it is paired only if it fits into the slots A leaves. S1 reads both,
  S_D checks both (A first; if A changes, or the final B does not fit, the pair is dropped = override of A; if B differs, override of B). One PTAB entry carries
  both blocks (explicit pc per slot, per-slot meta, B has its own block id / repair record).
- fetch-address FIFO entry carries the second block (addr, lengths). ICache reads A normally and B functionally in the same cycle (same page, tag resident, no state change); if B cannot be
  read the group is the first block only and the predecode checker flushes to B (never needed in the runs: failures 0 after the same-page rule).
- predecode checker works segment by segment (a segment ends where the pc is not consecutive); back end: FTQ entry per group, the branch ending a segment has the next segment's first pc as its predicted target.
- ICache: fetch_paddr exported; icache_fetch_second(), icache_peek_hit().
Findings: a cond-branch-ended first block raises committed cond mispredicts 2.5x on sjeng (see results), other first-block types are neutral.

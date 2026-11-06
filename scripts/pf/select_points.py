#!/usr/bin/env python3
"""Pick the prefetch evaluation set from a finished SPEC replay (EXP-009).
usage: select_points.py <spec-dir> <res-dir> > points.txt
Per benchmark, take the points with the largest weight*CPI (share of run time); lines: <split> <bench> <ckpt-name> <weight> <ipc>.
split = train | hold (hold-out points are only used for confirmation)."""
import glob, os, re, sys
spec, res = sys.argv[1], sys.argv[2]
QUOTA = [("429.mcf", 5, 2), ("462.libquantum", 4, 1), ("473.astar", 3, 1), ("471.omnetpp", 2, 0),
         ("483.xalancbmk", 2, 1), ("401.bzip2", 2, 0), ("403.gcc", 2, 1)]  # (bench, points, how many are hold-out)
for bench, n, nh in QUOTA:
    w = {}
    for l in open(f"{spec}/sp/{bench}.weights"):
        p = l.split()
        if len(p) >= 2: w[p[1]] = float(p[0])
    rows = []
    for f in glob.glob(f"{res}/{bench}/*.ok"):
        name, ins, cyc = open(f).read().split()[:3]
        sp = re.search(r"sp(\d+)_", name).group(1)
        ipc = int(ins) / int(cyc)
        rows.append((w.get(sp, 0) / ipc, name, w.get(sp, 0), ipc))
    rows.sort(reverse=True)
    for k, (score, name, wt, ipc) in enumerate(rows[:n]):
        split = "hold" if k % 2 == 1 and k // 2 < nh else "train"
        print(split, bench, name, f"{wt:.4f}", f"{ipc:.3f}")

#!/usr/bin/env python3
"""variant_report.py <dse dir> <ref run> <run>...: compare several fork-DSE runs (one per prefetcher variant, EXP-048).
Each <dse dir>/<run>/ has jobs.txt ("<bench> <ckpt> <weight>"), cfg.txt and <bench>/<ckpt>/<cfg>.res. Per benchmark the
weighted CPI over its points; rows = run x cfg, columns = IPC change per benchmark against <ref run>/base, plus the geomean."""
import math
import os
import sys
from collections import defaultdict

top, ref = sys.argv[1], sys.argv[2]
runs = sys.argv[2:]


def load(run):
    W = os.path.join(top, run)
    pts = defaultdict(list)
    for line in open(os.path.join(W, "jobs.txt")):
        f = line.split()
        if len(f) >= 3:
            pts[f[0]].append((f[1], float(f[2])))
    cfgs = [l.split()[0] for l in open(os.path.join(W, "cfg.txt")) if l.strip() and not l.lstrip().startswith("#")]
    cpi = {}
    for c in cfgs:
        for b, pl in pts.items():
            acc = ws = 0.0
            for ck, w in pl:
                p = os.path.join(W, b, ck, c + ".res")
                if not os.path.exists(p):
                    acc = None
                    break
                _, ins, cyc = open(p).read().split()[:3]
                acc += w * int(cyc) / int(ins)
                ws += w
            if acc is not None and ws:
                cpi[(c, b)] = acc / ws
    return cfgs, cpi


data = {r: load(r) for r in runs}
base = data[ref][1]
benches = sorted({b for (c, b) in base if c == "base"})
print(f"# IPC change against {ref}/base; {len(benches)} benchmarks")
print(f"{'run/cfg':<20} {'geo%':>7} " + " ".join(f"{b.split('.')[1][:8]:>8}" for b in benches))
for r in runs:
    cfgs, cpi = data[r]
    for c in cfgs:
        if not all((c, b) in cpi for b in benches):
            print(f"{r + '/' + c:<20} incomplete")
            continue
        d = [base[("base", b)] / cpi[(c, b)] for b in benches]
        g = math.exp(sum(math.log(x) for x in d) / len(d))
        print(f"{r + '/' + c:<20} {100 * (g - 1):+7.2f} " + " ".join(f"{100 * (x - 1):+8.2f}" for x in d))

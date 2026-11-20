#!/usr/bin/env python3
"""fork_report.py <run dir> <jobs file> [--bench]: EXP-047 DSE table.
<run dir>/<bench>/<ckpt>/<cfg>.res = "<ckpt> <insts> <cycles>"; jobs file lines "<bench> <ckpt> <weight>".
Per benchmark: CPI = weighted mean of its points' CPI (weights renormalised over the points that have all configs);
then the IPC geomean over the benchmarks, as a change against config `base`.  --bench adds the per-benchmark changes."""
import math
import os
import sys
from collections import defaultdict

run, jobs = sys.argv[1], sys.argv[2]
per_bench = "--bench" in sys.argv
pts = defaultdict(list)
for line in open(jobs):
    f = line.split()
    if len(f) >= 3:
        pts[f[0]].append((f[1], float(f[2])))

cfgs = [l.split()[0] for l in open(os.path.join(run, "cfg.txt")) if l.strip() and not l.lstrip().startswith("#")]
cpi = defaultdict(dict)  # cpi[cfg][bench]
missing = 0
for b, plist in pts.items():
    acc = defaultdict(float)
    wsum = 0.0
    for ck, w in plist:
        r = {}
        for c in cfgs:
            p = os.path.join(run, b, ck, c + ".res")
            if os.path.exists(p):
                _, ins, cyc = open(p).read().split()[:3]
                r[c] = int(cyc) / int(ins)
        if len(r) != len(cfgs):
            missing += 1
            continue
        wsum += w
        for c in cfgs:
            acc[c] += w * r[c]
    if wsum:
        for c in cfgs:
            cpi[c][b] = acc[c] / wsum

benches = sorted(cpi["base"]) if "base" in cpi else []
if not benches:
    sys.exit("no complete benchmark yet")
print(f"# {len(benches)} benchmarks, {sum(len(v) for v in pts.values()) - missing} complete points ({missing} incomplete)")
def gm_ipc(c):
    return math.exp(sum(math.log(1 / cpi[c][b]) for b in benches) / len(benches))
base = gm_ipc("base")
print(f"# base IPC geomean {base:.4f}")
hdr = f"{'config':<14} {'IPC':>7} {'d%':>7}"
if per_bench:
    hdr += " " + " ".join(f"{b.split('.')[1][:7]:>7}" for b in benches)
print(hdr)
for c in cfgs:
    g = gm_ipc(c)
    row = f"{c:<14} {g:7.4f} {100 * (g / base - 1):+7.2f}"
    if per_bench:
        row += " " + " ".join(f"{100 * (cpi['base'][b] / cpi[c][b] - 1):+7.2f}" for b in benches)
    print(row)

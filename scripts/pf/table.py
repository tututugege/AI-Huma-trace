#!/usr/bin/env python3
"""table.py <exp-root> <split> <cfg>...: speedup table (geomean per benchmark and overall) of configs vs <exp-root>/base (EXP-009)."""
import math, os, re, sys
here = os.path.dirname(os.path.abspath(__file__))
root, split, cfgs = sys.argv[1], sys.argv[2], sys.argv[3:]
def cyc(d, b, n):
    f = f"{root}/{d}/{b}/{n}.res"
    if not os.path.exists(f): return None
    nm, ins, c = open(f).readline().split()
    return int(ins) / int(c)
pts = [l.split() for l in open(f"{here}/" + os.environ.get("POINTS", "points.txt"))]
pts = [p for p in pts if split == "all" or p[0] == split]
benches = []
for p in pts:
    if p[1] not in benches: benches.append(p[1])
print(f"{'config':10s}" + "".join(f"{b.split('.')[1][:8]:>9s}" for b in benches) + f"{'GEOMEAN':>9s}  n")
for c in cfgs:
    per = {}; allv = []
    c, _, bd = c.partition("@")          # "cfg@basedir" compares against another baseline (e.g. a different MSHR count)
    for s, b, n, w, ipc in pts:
        a, t = cyc(bd or "base", b, n), cyc(c, b, n)
        if a and t: per.setdefault(b, []).append(t / a); allv.append(t / a)
    if not allv: continue
    row = "".join(f"{(math.exp(sum(map(math.log, per[b]))/len(per[b])) if b in per else float('nan')):9.3f}" for b in benches)
    print(f"{c:10s}{row}{math.exp(sum(map(math.log, allv))/len(allv)):9.4f}  {len(allv)}")

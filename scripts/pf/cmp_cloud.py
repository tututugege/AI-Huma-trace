#!/usr/bin/env python3
"""cmp_cloud.py <local-root> <cloud-root> <cfg>...: (1) cloud cl_<cfg> vs local <cfg> cycle identity on the points both have;
(2) speedup of cl_<cfg> over the local base on train / hold / all (EXP-009)."""
import math, os, sys
here = os.path.dirname(os.path.abspath(__file__))
L, C, cfgs = sys.argv[1], sys.argv[2], sys.argv[3:]
def cyc(root, d, b, n):
    f = f"{root}/{d}/{b}/{n}.res"
    if not os.path.exists(f): return None
    _, i, c = open(f).readline().split(); return int(i), int(c)
pts = [l.split() for l in open(f"{here}/points.txt")]
for cfg in cfgs:
    same = diff = 0; bad = []
    for s, b, n, w, ipc in pts:
        a, t = cyc(L, cfg, b, n), cyc(C, "cl_" + cfg, b, n)
        if a and t:
            if a == t: same += 1
            else: diff += 1; bad.append((b, n[:18], a[1], t[1]))
    print(f"{cfg}: cloud vs local identical {same}, different {diff}", bad[:3] if bad else "")
    for split in ("train", "hold", "all"):
        v = []
        for s, b, n, w, ipc in pts:
            if split != "all" and s != split: continue
            a, t = cyc(L, "base", b, n), cyc(C, "cl_" + cfg, b, n)
            if a and t: v.append((t[0] / t[1]) / (a[0] / a[1]))
        if v: print(f"   {split:5s} n={len(v):2d} geomean speedup {math.exp(sum(map(math.log, v))/len(v)):.4f}")

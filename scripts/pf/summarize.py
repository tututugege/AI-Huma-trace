#!/usr/bin/env python3
"""summarize.py <base-dir> <test-dir> [train|hold|all]: per-point and per-benchmark IPC change of test vs base (EXP-009)."""
import math, os, re, sys
here = os.path.dirname(os.path.abspath(__file__))
base, test = sys.argv[1], sys.argv[2]
split = sys.argv[3] if len(sys.argv) > 3 else "all"
def read(d, b, n):
    f = f"{d}/{b}/{n}.res"
    if not os.path.exists(f): return None
    L = open(f).read().splitlines()
    name, ins, cyc = L[0].split()
    st = {}
    for l in L[1:]:
        for k, v in re.findall(r"(\w+)=([0-9.]+)", l): st[k] = float(v)
    return int(ins), int(cyc), st
rows, per = [], {}
for l in open(f"{here}/points.txt"):
    s, b, n, w, ipc0 = l.split()
    if split != "all" and s != split: continue
    a, t = read(base, b, n), read(test, b, n)
    if not a or not t: continue
    sp = (a[1] / a[0]) and (t[0] / t[1]) / (a[0] / a[1])
    rows.append((b, n, sp, t[2])); per.setdefault(b, []).append(sp)
    st = t[2]
    print(f"{b:15s} {n[:22]:22s} ipc_base {a[0]/a[1]:.3f} speedup {sp:.4f}  "
          f"miss {st.get('demand_miss',0):.0f} useful {st.get('useful',0):.0f} late {st.get('late',0):.0f} issued {st.get('issued',0):.0f} "
          f"useless {st.get('useless_evict',0):.0f}")
if rows:
    g = math.exp(sum(math.log(r[2]) for r in rows) / len(rows))
    print("per-benchmark geomean speedup:", {b: round(math.exp(sum(math.log(x) for x in v)/len(v)), 4) for b, v in per.items()})
    print(f"GEOMEAN speedup ({split}, {len(rows)} points): {g:.4f}")

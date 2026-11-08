#!/usr/bin/env python3
"""spec_cmp.py <base-res-root> <test-res-root> <points_full.txt>: per-program and per-point comparison of two full SPEC runs.
Each root contains full/<bench>/*.res and full_mcf/<bench>/*.res (first line: "<ckpt> <instructions> <cycles>").
Program IPC = sum(w) / sum(w * cycles / instructions) over the SimPoints present in both runs (SimPoint weights, column 4 of the points file);
estimated score change = geomean of the per-program IPC ratios."""
import glob, math, os, sys
base, test, pts = sys.argv[1:4]
def load(root):
    d = {}
    for f in glob.glob(f"{root}/full*/*/*.res"):
        b = os.path.basename(os.path.dirname(f)); n = os.path.basename(f)[:-4]
        p = open(f).readline().split()
        if len(p) == 3: d[(b, n)] = (int(p[1]), int(p[2]))
    return d
A, T = load(base), load(test)
W = {}; order = []
for l in open(pts):
    s, b, n, w = l.split()[:4]
    W[(b, n)] = float(w)
    if b not in order: order.append(b)
rows = []; per = {}
for k, w in W.items():
    if k in A and k in T:
        ia, ca = A[k]; it, ct = T[k]
        rows.append((k[0], k[1], w, (it / ct) / (ia / ca), ia, ca, it, ct)); per.setdefault(k[0], []).append(rows[-1])
print(f"points: total {len(W)}, base {sum(k in A for k in W)}, test {sum(k in T for k in W)}, common {len(rows)}")
print(f"{'program':16} {'pts':>4} {'IPC base':>9} {'IPC test':>9} {'speedup':>8}")
sp = []
for b in order:
    r = per.get(b)
    if not r: continue
    sw = sum(x[2] for x in r)
    cb = sum(x[2] * x[5] / x[4] for x in r) / sw; ct = sum(x[2] * x[7] / x[6] for x in r) / sw
    s = cb / ct; sp.append(s)
    print(f"{b:16} {len(r):4d} {1/cb:9.4f} {1/ct:9.4f} {100*(s-1):+7.2f}%")
g = math.exp(sum(map(math.log, sp)) / len(sp))
print(f"GEOMEAN over {len(sp)} programs: {g:.4f} ({100*(g-1):+.2f}%)   estimated score: 9.42 x {g:.4f} = {9.42*g:.2f} points/GHz")
pg = math.exp(sum(math.log(r[3]) for r in rows) / len(rows))
print(f"unweighted per-point geomean speedup over {len(rows)} points: {pg:.4f}; min {min(r[3] for r in rows):.3f} max {max(r[3] for r in rows):.3f}")
print("points below -5%:")
for r in sorted(rows, key=lambda x: x[3]):
    if r[3] < 0.95: print(f"  {r[0]:16} {r[1][:28]:28} w={r[2]:.4f} {100*(r[3]-1):+.1f}%")

#!/usr/bin/env python3 -I
"""EXP-011: compare fast-warmup SPEC results with the exact ones.
usage: fw_compare.py <spec> <cov> <exact_base_dir> <fast_base_dir> [<name>=<exact_dir>,<fast_dir> ...]
Per program weighted IPC (weights of the SimPoints covering <cov>), geomean over programs; for every named configuration the
speedup over the baseline (exact: exact_cfg/exact_base; fast: fast_cfg/fast_base) per program and geomean, the error of
the fast ratio in percentage points, the configuration ranking, and the absolute IPC error of the baseline per program."""
import sys, glob, math, os
spec, cov, ebase, fbase = sys.argv[1], float(sys.argv[2]), sys.argv[3], sys.argv[4]
cfgs = [(a.split('=')[0], *a.split('=')[1].split(',')) for a in sys.argv[5:]]
B = ['400.perlbench', '403.gcc', '429.mcf', '456.hmmer', '458.sjeng', '445.gobmk', '473.astar', '464.h264ref']

def points(b):
    pts = sorted(((float(l.split()[0]), int(l.split()[1])) for l in open(f'{spec}/sp/{b}.weights') if l.strip()), reverse=True)
    acc, sel = 0, []
    for w, sp in pts:
        sel.append((w, sp)); acc += w
        if acc >= cov - 1e-9: break
    return sel

def load(d, b, sp):
    f = glob.glob(f'{d}/{b}/ckpt_sp{sp}_target*.ok')
    if not f: return None
    _, i, c, *_ = open(f[0]).read().split()
    return int(i), int(c)

def prog_ipc(d, b, only=None):
    sel = [s for s in points(b) if only is None or s[1] in only]
    W = sum(w for w, _ in sel); cpi = 0
    for w, sp in sel:
        r = load(d, b, sp)
        cpi += w / W * r[1] / r[0]
    return 1 / cpi

def common(*dirs):
    out = {}
    for b in B:
        out[b] = {sp for _, sp in points(b) if all(load(d, b, sp) for d in dirs)}
    return out

alld = [ebase, fbase] + [x for c in cfgs for x in c[1:]]
com = common(*alld)
n = sum(len(v) for v in com.values())
print(f'points present in every directory: {n}')
def geo(v): return math.prod(v) ** (1 / len(v))
use = [b for b in B if com[b]]
print('\nbaseline IPC per program (weighted over the common points)   exact   fast   err%')
errs = []
for b in use:
    e, f = prog_ipc(ebase, b, com[b]), prog_ipc(fbase, b, com[b])
    errs.append(abs(f / e - 1) * 100)
    print(f'  {b:14s} {e:7.3f} {f:7.3f} {100 * (f / e - 1):+6.2f}   ({len(com[b])} pts)')
ge, gf = geo([prog_ipc(ebase, b, com[b]) for b in use]), geo([prog_ipc(fbase, b, com[b]) for b in use])
print(f'  {"geomean":14s} {ge:7.3f} {gf:7.3f} {100 * (gf / ge - 1):+6.2f}   mean |err| per program {sum(errs) / len(errs):.2f}%')
pe = []
for b in use:
    for sp in com[b]:
        a, c = load(ebase, b, sp), load(fbase, b, sp)
        pe.append(abs((c[0] / c[1]) / (a[0] / a[1]) - 1) * 100)
pe.sort()
print(f'  per-point |IPC error| of the baseline: mean {sum(pe) / len(pe):.2f}%  median {pe[len(pe) // 2]:.2f}%  max {pe[-1]:.2f}%')
rank_e, rank_f = [], []
for name, ec, fc in cfgs:
    print(f'\nconfiguration {name}: speedup over baseline   exact   fast   diff(pp)')
    for b in use:
        re_ = prog_ipc(ec, b, com[b]) / prog_ipc(ebase, b, com[b])
        rf = prog_ipc(fc, b, com[b]) / prog_ipc(fbase, b, com[b])
        print(f'  {b:14s} {100 * (re_ - 1):+7.2f}% {100 * (rf - 1):+7.2f}% {100 * (rf - re_):+6.2f}')
    ce = geo([prog_ipc(ec, b, com[b]) / prog_ipc(ebase, b, com[b]) for b in use])
    cf = geo([prog_ipc(fc, b, com[b]) / prog_ipc(fbase, b, com[b]) for b in use])
    print(f'  {"geomean":14s} {100 * (ce - 1):+7.2f}% {100 * (cf - 1):+7.2f}% {100 * (cf - ce):+6.2f}   <- acceptance: |diff| <= 0.5 pp')
    rank_e.append((ce, name)); rank_f.append((cf, name))
print('\nranking (best first) exact:', ' > '.join(n for _, n in sorted(rank_e, reverse=True)))
print('ranking (best first) fast :', ' > '.join(n for _, n in sorted(rank_f, reverse=True)))

def times(d):
    t = []
    for f in glob.glob(f'{d}/*/*.time'):
        p = open(f).read().split()
        if len(p) == 3: t.append(tuple(map(float, p)))
    return t
for tag, d in (('exact baseline', ebase), ('fast baseline', fbase)):
    t = times(d)
    if t: print(f'{tag}: {len(t)} timed points, mean wall {sum(x[0] for x in t) / len(t):.1f}s user {sum(x[1] for x in t) / len(t):.1f}s')

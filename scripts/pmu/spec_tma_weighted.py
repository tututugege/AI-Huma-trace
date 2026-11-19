#!/usr/bin/env python3
"""SimPoint-weighted top-down (TMA) of full SPEC runs (rv64_one.sh results: <res>/<bench>/<ckpt>.pmu_events.csv).

  spec_tma_weighted.py <sp-dir> <res-dir> [<res-dir-2>]

Per benchmark the event counts of its points are summed with the SimPoint weights (every point measures the same 10M
instructions, so this is the weighted average per instruction) and turned into % of slots (DECODE_WIDTH = 8 per cycle) with
the formulas of docs/PMU.md (rename / IDU stalls in Backend Bound, column BE.ren; BE.mem / BE.core split the dispatch-blocked
slots by the ROB head). With a second result directory each row shows run 2 and the change
(percentage points). Only points present in every given directory are used. The last row is the plain mean over benchmarks.
"""
import csv, glob, os, re, sys

SLOTS_PER_CYCLE = 8


def weights(sp, b):
    w = {}
    for line in open(os.path.join(sp, b + '.weights')):
        t = line.split()
        if len(t) >= 2:
            w[int(t[1])] = float(t[0])
    return w


def events(f):
    return {r['event']: int(r['count']) for r in csv.DictReader(open(f))}


def ren_stall(e):
    """Empty dispatch slots while rename / the IDU wait for a physical register / branch tag (Backend Bound) and whether the
    file is from before 2026-11-19, when these were SLOT_FE_OTHER_NOPREG / _NOBRTAG, a part of SLOT_FE_OTHER."""
    if 'SLOT_REN_NOPREG' in e:
        return e['SLOT_REN_NOPREG'] + e['SLOT_IDU_NOBRTAG'], False
    return e.get('SLOT_FE_OTHER_NOPREG', 0) + e.get('SLOT_FE_OTHER_NOBRTAG', 0), True


def tma(e):
    sl = e['CYCLES'] * SLOTS_PER_CYCLE
    rec = e['SLOT_FE_RECOV_MISPRED'] + e['SLOT_FE_RECOV_FLUSH']
    ren, old = ren_stall(e)
    wrong = max(0, e['SLOT_FIRE'] - e['INSTRET'])
    p = lambda x: 100.0 * x / sl
    return {
        'IPC': e['INSTRET'] / e['CYCLES'],
        'Ret': p(e['INSTRET']),
        'BadSp': p(wrong + rec + e['SLOT_KILLED']),
        'FE': p(e['SLOT_FE_EMPTY'] - rec - ren),
        'FE.lat': p(e['SLOT_FE_ICACHE'] + e['SLOT_FE_ITLB']),
        'FE.bw': p(e['SLOT_FE_OTHER'] - (ren if old else 0)),
        'BE': p(e['SLOT_BE'] + ren),
        'BE.ren': p(ren),
        'BE.mem': p(e['SLOT_BE_HEAD_MISS'] + e['SLOT_BE_HEAD_MEM']),
        'BE.core': p(e['SLOT_BE_HEAD_CORE'] + e['SLOT_BE_HEAD_DONE']),
        'L1Dmpki': 1000.0 * e['DC_LD_MISS'] / e['INSTRET'],
        'MLP': e['DC_LD_MISS_OUT'] / max(1, e['CYC_LD_MISS_OUT']),
        'MissLat': e['DC_LD_MISS_OUT'] / max(1, e['DC_LD_MISS']),
    }


COLS = ['IPC', 'Ret', 'BadSp', 'FE', 'FE.lat', 'FE.bw', 'BE', 'BE.ren', 'BE.mem', 'BE.core', 'L1Dmpki', 'MLP', 'MissLat']


def bench_tma(sp, dirs, b):
    w = weights(sp, b)
    pts = None
    for d in dirs:
        s = {int(re.search(r'ckpt_sp(\d+)_', f).group(1)): f for f in glob.glob(os.path.join(d, b, 'ckpt_sp*.pmu_events.csv'))}
        pts = set(s) if pts is None else pts & set(s)
    pts = sorted(p for p in (pts or ()) if p in w)
    if not pts:
        return None, 0
    out = []
    for d in dirs:
        tot = {}
        for p in pts:
            f = glob.glob(os.path.join(d, b, 'ckpt_sp%d_*.pmu_events.csv' % p))[0]
            for k, v in events(f).items():
                tot[k] = tot.get(k, 0.0) + w[p] * v
        out.append(tma(tot))
    return out, len(pts)


def main():
    sp, dirs = sys.argv[1], sys.argv[2:]
    benches = sorted(os.path.basename(p)[:-8] for p in glob.glob(os.path.join(sp, '*.weights')))
    hdr = '%-15s %4s ' % ('bench', 'pts') + ' '.join('%8s' % c for c in COLS)
    print('% of slots (8 per cycle); ' + ('run 2 and change vs run 1 (pp)' if len(dirs) > 1 else 'one run'))
    print(hdr)
    acc = [dict() for _ in dirs]
    n = 0
    for b in benches:
        r, k = bench_tma(sp, dirs, b)
        if r is None:
            continue
        n += 1
        for i, t in enumerate(r):
            for c in COLS:
                acc[i][c] = acc[i].get(c, 0.0) + t[c]
        if len(dirs) == 1:
            print('%-15s %4d ' % (b, k) + ' '.join('%8.2f' % r[0][c] for c in COLS))
        else:
            print('%-15s %4d ' % (b, k) + ' '.join('%8.2f' % r[1][c] for c in COLS))
            print('%-15s %4s ' % ('  change', '') + ' '.join('%+8.2f' % (r[1][c] - r[0][c]) for c in COLS))
    if n:
        m = [{c: a[c] / n for c in COLS} for a in acc]
        print('%-15s %4s ' % ('mean', '') + ' '.join('%8.2f' % m[-1][c] for c in COLS))
        if len(dirs) > 1:
            print('%-15s %4s ' % ('  change', '') + ' '.join('%+8.2f' % (m[1][c] - m[0][c]) for c in COLS))
            print('%-15s %4s ' % ('run 1 mean', '') + ' '.join('%8.2f' % m[0][c] for c in COLS))


if __name__ == '__main__':
    main()

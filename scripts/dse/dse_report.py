#!/usr/bin/env python3
"""dse_report.py [res-dir] [points.txt]: per-config weighted IPC (geomean over benchmarks) vs base + TMA/occupancy columns.
Per benchmark: CPI = weight-averaged over the benchmark's points; configs missing any point of a benchmark are flagged (n/12)."""
import csv, math, os, sys, collections
res = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/personal/AI-Huma-data/dse/res")
pts = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(__file__), "../pf/points_dse.txt")
W = collections.defaultdict(dict)
for l in open(pts):
    p = l.split()
    if len(p) >= 4: W[p[1]][p[2]] = float(p[3])
BENCH = sorted(W)
ROUND = lambda x: f"{x:.3f}"
def load(cfg):
    out = {}
    for b in BENCH:
        rows = []
        for n, w in W[b].items():
            f = f"{res}/{cfg}/{b}/{n}.res"
            if not os.path.exists(f): continue
            ins, cyc = map(int, open(f).read().split()[1:3])
            ev = {r["event"]: int(r["count"]) for r in csv.DictReader(open(f"{res}/{cfg}/{b}/{n}.events.csv"))}
            rows.append((w, ins, cyc, ev))
        if len(rows) == len(W[b]): out[b] = rows
    return out
def bench_ipc(rows):  # weighted CPI
    tw = sum(r[0] for r in rows); return tw / sum(r[0] * r[2] / r[1] for r in rows)
def tma(rows):  # slot shares, cycle-weighted over points of one benchmark (sum of events)
    s = collections.Counter()
    for _, _, _, ev in rows:
        for k, v in ev.items(): s[k] += v
    sl = 8 * s["CYCLES"]; g = lambda k: s[k] / sl
    rec = s["SLOT_FE_RECOV_MISPRED"] + s["SLOT_FE_RECOV_FLUSH"]
    return dict(ret=s["INSTRET"] / sl, bad=(max(0, s["SLOT_FIRE"] - s["INSTRET"]) + rec + s["SLOT_KILLED"]) / sl,
                fe=(s["SLOT_FE_EMPTY"] - rec) / sl, be=g("SLOT_BE"), rob=g("SLOT_BE_ROB_FULL"), iq=g("SLOT_BE_IQ_FULL"),
                ldq=g("SLOT_BE_LDQ_FULL"), stq=g("SLOT_BE_STQ_FULL"), preg=g("SLOT_FE_OTHER_NOPREG"), brt=g("SLOT_FE_OTHER_NOBRTAG"),
                occrob=s["OCC_ROB"] / s["CYCLES"], occldq=s["OCC_LDQ"] / s["CYCLES"], occstq=s["OCC_STQ"] / s["CYCLES"])
cfgs = sorted(d for d in os.listdir(res) if os.path.isdir(f"{res}/{d}"))
data = {c: load(c) for c in cfgs}
base = {b: bench_ipc(r) for b, r in data.get("base", {}).items()}
def gm(xs): return math.exp(sum(math.log(x) for x in xs) / len(xs))
print("| config | benches | geomean IPC | vs base | " + " | ".join(b.split('.')[1][:6] for b in BENCH) + " |")
print("|---|---|---|---|" + "---|" * len(BENCH))
summ = {}
for c in cfgs:
    d = data[c]; ok = [b for b in BENCH if b in d and b in base]
    if not ok: continue
    ratios = {b: bench_ipc(d[b]) / base[b] for b in ok}
    g = gm([bench_ipc(d[b]) for b in ok]); gb = gm([base[b] for b in ok])
    summ[c] = (g, g / gb)
    print(f"| {c} | {len(ok)}/12 | {g:.3f} | {100*(g/gb-1):+.2f}% | " + " | ".join(f"{100*(ratios[b]-1):+.1f}" if b in ratios else "-" for b in BENCH) + " |")
print("\n### TMA / occupancy (all-benchmark mean of per-benchmark slot shares, %)\n")
print("| config | Ret | BadSp | FE | BE | ROBfull | IQfull | LDQfull | STQfull | noPREG | noBRtag | occROB | occLDQ | occSTQ |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for c in cfgs:
    d = data[c]; ok = [b for b in BENCH if b in d]
    if len(ok) < 12: continue
    t = [tma(d[b]) for b in ok]; m = lambda k: sum(x[k] for x in t) / len(t)
    print(f"| {c} | " + " | ".join(f"{100*m(k):.1f}" for k in ("ret","bad","fe","be","rob","iq","ldq","stq","preg","brt")) + " | " + " | ".join(f"{m(k):.1f}" for k in ("occrob","occldq","occstq")) + " |")

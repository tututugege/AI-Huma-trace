#!/usr/bin/env python3
"""One-table TMA summary of the runs of spec_tma.sh:  spec_summary.py <outdir>"""
import csv, sys
from pathlib import Path

out = Path(sys.argv[1])
rows = []
for d in sorted(p for p in out.iterdir() if p.is_dir()):
    f = d / "pmu_events.csv"
    if not f.exists():
        rows.append((d.name, None)); continue
    e = {r["event"]: int(r["count"]) for r in csv.DictReader(open(f))}
    cyc = e["CYCLES"]; sl = cyc * 8
    rec = e["SLOT_FE_RECOV_MISPRED"] + e["SLOT_FE_RECOV_FLUSH"]
    wrong = max(0, e["SLOT_FIRE"] - e["INSTRET"])
    bad = wrong + rec + e["SLOT_KILLED"]
    fe = e["SLOT_FE_EMPTY"] - rec
    be = e["SLOT_BE"]
    mem = e["SLOT_BE_HEAD_MISS"] + e["SLOT_BE_HEAD_MEM"]
    ki = e["INSTRET"] / 1000
    rows.append((d.name, dict(
        ipc=e["INSTRET"] / cyc, ret=e["INSTRET"] / sl, bad=bad / sl, fe=fe / sl, be=be / sl,
        mis=e["SLOT_FE_RECOV_MISPRED"] / sl, wrong=wrong / sl, flat=(e["SLOT_FE_ICACHE"] + e["SLOT_FE_ITLB"]) / sl,
        fbw=e["SLOT_FE_OTHER"] / sl, mem=mem / sl, l1m=e["SLOT_BE_HEAD_MISS"] / sl,
        core=(e["SLOT_BE_HEAD_CORE"] + e["SLOT_BE_HEAD_DONE"]) / sl, rob=e["SLOT_BE_ROB_FULL"] / sl,
        iq=e["SLOT_BE_IQ_FULL"] / sl, lsq=(e["SLOT_BE_LDQ_FULL"] + e["SLOT_BE_STQ_FULL"]) / sl,
        brmpki=(e["COMMIT_MISPRED_BR"] + e["COMMIT_MISPRED_JALR"] + e["COMMIT_MISPRED_RET"]) / ki,
        l1mpki=e["DC_LD_MISS"] / ki, imp=e["ICACHE_MISS"] / ki,
        mlp=e["DC_LD_MISS_OUT"] / max(1, e["CYC_LD_MISS_OUT"]))))
P = lambda x: f"{100 * x:5.1f}"
print(f"{'benchmark':16} {'IPC':>5} | {'Ret':>5} {'BadSp':>5} {'FE':>5} {'BE':>5} | {'mispr':>5} {'wrong':>5} | {'fLat':>5} {'fBW':>5} |"
      f" {'Mem':>5} {'L1m':>5} {'Core':>5} | {'ROB':>5} {'IQ':>5} {'LSQ':>5} | {'brMPKI':>6} {'L1dMPKI':>7} {'iMPKI':>5} {'MLP':>4}")
for n, r in rows:
    if r is None:
        print(f"{n:16} (no result)"); continue
    print(f"{n:16} {r['ipc']:5.2f} | {P(r['ret'])} {P(r['bad'])} {P(r['fe'])} {P(r['be'])} | {P(r['mis'])} {P(r['wrong'])} |"
          f" {P(r['flat'])} {P(r['fbw'])} | {P(r['mem'])} {P(r['l1m'])} {P(r['core'])} | {P(r['rob'])} {P(r['iq'])} {P(r['lsq'])} |"
          f" {r['brmpki']:6.1f} {r['l1mpki']:7.1f} {r['imp']:5.2f} {r['mlp']:4.1f}")
print("\ncolumns (% of slots): Ret/BadSp/FE/BE = level 1; mispr = mispredict recovery bubbles, wrong = wrong-path dispatches (both in BadSp);"
      "\n fLat = ICache+ITLB miss bubbles, fBW = other frontend bubbles (in FE); Mem/L1m/Core = by ROB head (Mem = head is a load/store, "
      "L1m = that load missed L1D);\n ROB/IQ/LSQ = by the full resource (in BE)")

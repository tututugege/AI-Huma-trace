#!/usr/bin/env python3
"""Analysis of the CSV files written by `PMU=1 PMU_OUT=<prefix> simulator ...` (EXP-013).

  pmu_report.py <prefix> [--top N] [--elf prog.elf]

Prints (1) the commit-blame profile: which static instructions the machine waited for (stall slots = dispatch slots
lost while the instruction was the oldest incomplete one), grouped by instruction type and by 4 KiB code page, with
branch mispredict hot spots; (2) the phase view from <prefix>_series.csv (PMU_INTERVAL set): IPC and the top-down
level-1 split per interval; (3) the histograms. --elf adds the nearest symbol of every pc (needs riscv nm: uses
`nm` found in PATH, e.g. riscv64-unknown-elf-nm).
"""
import argparse, bisect, csv, shutil, subprocess, sys
from collections import defaultdict
from pathlib import Path

TYPES = ["NOP", "JAL", "JALR", "ADD", "BR", "LOAD", "STORE", "CSR", "FENCE", "ECALL", "EBREAK", "WFI", "SFENCE_VMA",
         "FENCE_I", "MRET", "SRET", "MUL", "DIV", "AMO", "FP"]  # include/types.h InstType order


def load_symbols(elf):
    nm = shutil.which("riscv64-unknown-elf-nm") or shutil.which("riscv32-unknown-elf-nm") or shutil.which("nm")
    if not elf or not nm:
        return [], []
    out = subprocess.run([nm, "-n", "--defined-only", elf], capture_output=True, text=True).stdout
    addrs, names = [], []
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3 and p[1] in "tTwW":
            addrs.append(int(p[0], 16)); names.append(p[2])
    return addrs, names


def sym(addrs, names, pc):
    if not addrs:
        return ""
    i = bisect.bisect_right(addrs, pc) - 1
    return f"{names[i]}+{pc - addrs[i]:#x}" if i >= 0 else ""


def events(prefix):
    return {r["event"]: int(r["count"]) for r in csv.DictReader(open(f"{prefix}_events.csv"))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix"); ap.add_argument("--top", type=int, default=15); ap.add_argument("--elf")
    a = ap.parse_args()
    ev = events(a.prefix)
    addrs, names = load_symbols(a.elf)
    cyc = ev["CYCLES"]; slots = cyc * 8

    pcs = []
    p = Path(f"{a.prefix}_pc.csv")
    if p.exists():
        for r in csv.DictReader(open(p)):
            pcs.append({k: (int(v, 16) if k == "pc" else int(v)) for k, v in r.items()})
        total_stall = sum(r["stall_slots"] for r in pcs) or 1
        print(f"== who blocked the machine (oldest incomplete instruction while dispatch was stalled; {total_stall} slots = "
              f"{100 * total_stall / slots:.1f}% of all slots)")
        print(f"{'pc':>10} {'type':>6} {'commits':>9} {'head cyc':>9} {'miss cyc':>9} {'stall slots':>11} {'%stall':>6}  symbol")
        for r in pcs[:a.top]:
            print(f"{r['pc']:#10x} {TYPES[r['type']]:>6} {r['commits']:>9} {r['head_cycles']:>9} {r['head_miss_cycles']:>9} "
                  f"{r['stall_slots']:>11} {100 * r['stall_slots'] / total_stall:>5.1f}%  {sym(addrs, names, r['pc'])}")
        by_type = defaultdict(lambda: [0, 0, 0])
        by_page = defaultdict(lambda: [0, 0])
        for r in pcs:
            t = by_type[TYPES[r["type"]]]; t[0] += r["commits"]; t[1] += r["head_cycles"]; t[2] += r["stall_slots"]
            g = by_page[r["pc"] >> 12]; g[0] += r["head_cycles"]; g[1] += r["stall_slots"]
        print("\n== stall slots by instruction type (which instruction kind the machine waits for)")
        for k, v in sorted(by_type.items(), key=lambda kv: -kv[1][2])[:8]:
            print(f"  {k:>6}: commits {v[0]:>10}  head cycles {v[1]:>9}  stall slots {v[2]:>10} ({100 * v[2] / total_stall:.1f}%)")
        print("\n== stall slots by 4 KiB code page (top 5)")
        for k, v in sorted(by_page.items(), key=lambda kv: -kv[1][1])[:5]:
            print(f"  page {k << 12:#010x}: head cycles {v[0]:>9}  stall slots {v[1]:>10} ({100 * v[1] / total_stall:.1f}%)")
        mp = sorted((r for r in pcs if r["mispreds"]), key=lambda r: -r["mispreds"])
        tot_mp = sum(r["mispreds"] for r in mp) or 1
        print(f"\n== mispredict hot spots (committed mispredicts {tot_mp}; top {min(a.top, len(mp))})")
        for r in mp[:a.top]:
            print(f"  {r['pc']:#10x} {TYPES[r['type']]:>5}  mispredicts {r['mispreds']:>7} / {r['commits']:>8} commits "
                  f"({100 * r['mispreds'] / max(1, r['commits']):.1f}%)  {sym(addrs, names, r['pc'])}")
        for n in (1, 5, 20):
            print(f"  top {n:>2} branches = {100 * sum(r['mispreds'] for r in mp[:n]) / tot_mp:.1f}% of mispredicts")

    s = Path(f"{a.prefix}_series.csv")
    if s.exists():
        rows = list(csv.DictReader(open(s)))
        print(f"\n== phases ({len(rows)} intervals): IPC and top-down level 1 per interval")
        print(f"{'cycle':>10} {'IPC':>5} {'ret%':>5} {'badsp%':>6} {'fe%':>5} {'be%':>5} {'ldmiss/cyc':>10}")
        step = max(1, len(rows) // 20)
        prev = 0
        for i, r in enumerate(rows):
            c = int(r["cycle"]); d = c - prev; prev = c
            if i % step and i != len(rows) - 1:
                continue
            g = lambda k: int(r[k]); sl = d * 8
            retire = g("INSTRET")
            rec = g("SLOT_FE_RECOV_MISPRED") + g("SLOT_FE_RECOV_FLUSH")
            bad = max(0, g("SLOT_FIRE") - retire) + rec + g("SLOT_KILLED")
            fe = g("SLOT_FE_EMPTY") - rec
            be = sum(g(k) for k in ("SLOT_BE_ROB_FULL", "SLOT_BE_FLUSH_WAIT", "SLOT_BE_SERIAL", "SLOT_BE_LDQ_FULL",
                                    "SLOT_BE_STQ_FULL", "SLOT_BE_IQ_FULL"))
            print(f"{c:>10} {retire / d:>5.2f} {100 * retire / sl:>5.1f} {100 * bad / sl:>6.1f} {100 * fe / sl:>5.1f} "
                  f"{100 * be / sl:>5.1f} {g('DC_LD_MISS_OUT') / d:>10.2f}")

    h = Path(f"{a.prefix}_hist.csv")
    if h.exists():
        hist = defaultdict(list)
        for r in csv.DictReader(open(h)):
            hist[r["hist"]].append(int(r["cycles"]))
        print("\n== histograms (% of cycles)")
        for k, v in hist.items():
            t = sum(v) or 1
            print(f"  {k:>20}: " + " ".join(f"{100 * x / t:.0f}" for x in v))


if __name__ == "__main__":
    main()

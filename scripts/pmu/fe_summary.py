#!/usr/bin/env python3
"""Frontend delivery split (EXP-025) of PMU runs.

usage: python3 -I scripts/pmu/fe_summary.py <outdir>      (<outdir>/<bench>/pmu_events.csv, written by PMU=1 PMU_OUT=<outdir>/<bench>/pmu)

The FEO_* events are sampled at the front-end -> back-end interface in fetch-group slots (FETCH_WIDTH per cycle), only in cycles where the
back end could read, so they do not depend on backend stalls. Every column after the first block is a share of those slots.
"""
import csv, glob, os, sys

LOSS = [  # (event, column header)
    ("FEO_LOSS_TAKEN", "taken"), ("FEO_LOSS_LINE", "line"), ("FEO_LOSS_SHORTOTH", "shOth"),
    ("FEO_LOSS_REFETCH", "refch"), ("FEO_LOSS_ICMISS", "icMiss"), ("FEO_LOSS_PDFLUSH", "pdFl"),
    ("FEO_LOSS_OVR_SD", "ovrSD"), ("FEO_LOSS_OVR_L1", "ovrL1"), ("FEO_LOSS_PTABSTALL", "ptabSt"),
    ("FEO_LOSS_BPUOTHER", "bpuOth"), ("FEO_LOSS_ICNOTRDY", "icNRdy"), ("FEO_LOSS_IFIFO", "ififo"),
    ("FEO_LOSS_PTABLAG", "ptabLag"), ("FEO_LOSS_F2B", "f2b"),
]


def load(path):
    d = {}
    with open(path) as f:
        for row in csv.reader(f):
            if len(row) == 3 and row[2].isdigit():
                d[row[0]] = int(row[2])
    return d


def main():
    root = sys.argv[1]
    files = sorted(glob.glob(os.path.join(root, "*", "pmu_events.csv")))
    if not files:
        sys.exit("no pmu_events.csv under " + root)
    hdr = f"{'benchmark':<16}{'IPC':>5} | {'FE%':>5}{'fBW%':>6} | {'inst/grp':>8}{'grp/cyc':>8}{'inst/cyc':>9} |"
    hdr += "".join(f"{h:>8}" for _, h in LOSS)
    print(hdr)
    for p in files:
        e = load(p)
        b = os.path.basename(os.path.dirname(p))
        cyc, ret = e.get("CYCLES", 0), e.get("INSTRET", 0)
        slots = 8 * cyc if cyc else 1
        fe = (e.get("SLOT_FE_EMPTY", 0) - e.get("SLOT_FE_RECOV_MISPRED", 0) - e.get("SLOT_FE_RECOV_FLUSH", 0)) * 100.0 / slots
        fbw = e.get("SLOT_FE_OTHER", 0) * 100.0 / slots
        fc, fg, fi = e.get("FEO_CYC", 0), e.get("FEO_GROUPS", 0), e.get("FEO_INSTS", 0)
        tot = max(1, 8 * fc)
        line = f"{b:<16}{ret / cyc if cyc else 0:>5.2f} | {fe:>5.1f}{fbw:>6.1f} | {fi / fg if fg else 0:>8.2f}{fg / fc if fc else 0:>8.2f}{fi / fc if fc else 0:>9.2f} |"
        line += "".join(f"{e.get(k, 0) * 100.0 / tot:>8.1f}" for k, _ in LOSS)
        print(line)
    print("\nshares are % of FEO slots (8 per cycle in which the back end could read); the rest is delivered instructions (inst/cyc / 8).")
    print("taken/line/shOth = lost slots inside a delivered short group; the others = cycles with no group at all (x8 slots).")


if __name__ == "__main__":
    main()

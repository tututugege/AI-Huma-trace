#!/usr/bin/env python3
"""wb_cmp.py <base.csv> <new.csv>: per-program cycle change of two run_gate_c.sh WRITE tables (test,status,wall_s,cycles) and the geomean."""
import csv, math, sys
def load(f): return {r['test']: r for r in csv.DictReader(open(f))}
a, b = load(sys.argv[1]), load(sys.argv[2])
lg = []; ch = []
for t, r in a.items():
    if not r['cycles'].isdigit() or t not in b or not b[t]['cycles'].isdigit(): continue
    x, y = int(r['cycles']), int(b[t]['cycles']); lg.append(math.log(y / x))
    if x != y: ch.append((t, x, y, 100 * (y / x - 1)))
for t, x, y, p in sorted(ch, key=lambda z: z[3]): print(f'{t:28s} {x:8d} {y:8d} {p:+.2f}%')
print('n', len(lg), 'changed', len(ch), 'geomean %+.3f%%' % (100 * (math.exp(sum(lg) / len(lg)) - 1)))

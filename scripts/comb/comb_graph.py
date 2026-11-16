#!/usr/bin/env python3
"""The comb checks of the whole CPU.

  * modules.py: every module's contracts against its code (its ports' reads / writes per phase);
  * SimCpu::cycle() as it runs (comb_top.py): each phase's reads / writes as SimCpu paths, and no phase reads a signal a
    later phase writes (a hidden register), except the reviewed cases below;
  * gen_top_order.py --check: the generated order is the code's.
usage: comb_graph.py [--check]   (--check: exit 1 on any mismatch or hazard)
"""
import re, sys, os, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ASSIGN = r'\s*(?:\[[^\]]*\]|\.\w+|\(\))*\s*(?:=(?!=)|\|=|&=|\+=|-=|\^=|<<=|>>=|\+\+|--)'


def overlap(a, b):
    """keys a (set) and b (set) name a common signal: equal, or one is the whole of the other"""
    out = set()
    for x in a:
        for y in b:
            if x == y or y.startswith(x + '.') or x.startswith(y + '.'): out.add(x if len(x) >= len(y) else y)
    return out


all_hazards = []
import modules
merr = modules.check()
print(f'modules: {len(modules.MODULES)} modules, {len(modules.phases())} phases, {len(merr)} contract mismatches')
for e in merr: print('  contract: ' + e)
all_hazards += merr

# top level: SimCpu::cycle() (see comb_top.py)
from comb_top import top_domain
trows, tnotes, toverlap, texcl, tprior = top_domain(ROOT, ASSIGN)
# reviewed: the AXI interconnect keeps some of its own last-cycle outputs in the shared port structs (the downstream
# AR / AW / W valid it drove, the write-port req.ready) and reads them in comb_outputs: registers inside the AXI kit,
# to become explicit when the AXI kit is reworked (docs/REG_WIRE.md)
# reviewed: the AXI kit keeps a write port's req.ready of the last cycle in the port struct (comb_inputs sets it, the
# next comb_outputs reads it): a register inside the AXI kit
TOP_KNOWN = {('axi.comb_outputs', 'axi.interconnect.write_ports.req.ready', 'axi.comb_inputs'): 'AXI kit'}
# reviewed: comb_outputs reads a write port's addr / total_size only under its req.valid, which axi.comb_begin has
# cleared at that point (its contract: pre; the memory subsystem drives the request later): the stale value is never used
for f in ('addr', 'total_size'):
    for wr in ('mem_subsystem.wb_.comb_dcache_to_axi', 'mem_subsystem.peripheral_axi_.comb'):
        TOP_KNOWN[('axi.comb_outputs', f'axi.interconnect.write_ports.req.{f}', wr)] = 'under valid'
thaz, tstaged = [], 0
for i, (n, r, w) in enumerate(trows):
    for j in range(i + 1, len(trows)):
        if texcl(i, j): continue
        for f in sorted(toverlap(r, trows[j][2])):
            if (n, f, trows[j][0]) in TOP_KNOWN: continue
            if tprior(i, f): tstaged += 1; continue     # written earlier in the cycle too: not a hidden register
            thaz.append((n, f, trows[j][0]))
tnotes = set(tnotes) | {f'{tstaged} staged reads (also written by an earlier phase of the cycle)'}
print(f'\n== top: phases: {len(trows)}, read-before-later-write hazards: {len(thaz)}')
for n, r, w in trows: print(f'{n:40s} R: {" ".join(sorted(r))} | W: {" ".join(sorted(w))}')
print('hazards (reader, signal, later writer):')
for h in thaz: print('  %s reads %s, written later by %s' % h)
for x in sorted(tnotes): print('  note: ' + x)
all_hazards += thaz
# SimCpu::cycle() and FrontCore::comb() must be the order generated from the modules' contracts
if '--check' in sys.argv:
    p = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gen_top_order.py'), '--check'],
                       capture_output=True, text=True)
    print(p.stdout.rstrip())
    if p.returncode != 0: all_hazards.append('generated order')
if '--check' in sys.argv and all_hazards: sys.exit(1)

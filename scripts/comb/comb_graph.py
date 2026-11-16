#!/usr/bin/env python3
"""Static read/write sets of the back-end comb phases over the back end's signals (the modules' In / Out ports, back-end/include/BackPorts.h).

For every phase called in BackTop::comb() (in call order) the body of the phase function is scanned for `wires->F` /
`w.F` / `wires_->F` (F = a Wires field); an access is a write when the field expression is followed by an assignment
operator (=, |=, &=, +=, ...), or is passed to a function as an output (`&w.F` / non-const reference parameters are
not seen: the tool under-reports such writes).  Prints the table, the order hazards (a phase reading a field that a
LATER phase writes) and a DOT graph.  usage: comb_graph.py [--dot out.dot] [--check]   (--check: exit 1 on a hazard)
"""
import re, sys, os, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
def rd(p): return open(os.path.join(ROOT, p), errors='ignore').read()

wires_h = rd('back-end/include/BackPorts.h')
m = re.search(r'struct BackPorts \{(.*?)\n\};', wires_h, re.S)
FIELDS = re.findall(r'^\s*[\w:<>, ]+?\s\*?(\w+)(?:\s*=[^;]*)?;', m.group(1), re.M)
# the back end's signals: those in BackTop (Wires: its interface and its own) and each module's outputs (struct Out)
OUTS = {}   # signal -> owning module class
for d0, _, fs in os.walk(os.path.join(ROOT, 'back-end')):
    for f0 in fs:
        if not f0.endswith('.h'): continue
        t0 = open(os.path.join(d0, f0), errors='ignore').read()
        for cm in re.finditer(r'\nclass (\w+)[^;{]*\{', t0):
            om = re.search(r'struct Out \{(.*?)\n\s*\} out;', t0[cm.end():cm.end() + 20000], re.S)
            if not om or re.search(r'\nclass \w+', t0[cm.end():cm.end() + om.start()]): continue
            for fm in re.finditer(r'^\s*[\w:<>, ]+?\s(\w+);', om.group(1), re.M): OUTS[fm.group(1)] = cm.group(1)
FIELDS = [f for f in FIELDS if f not in ('in', 'out')] + sorted(OUTS) + ['idu_br_latch', 'in', 'out']   # (idu_br_latch: Idu's
#   br_latch register, a port of the pre-decode queue)
EXT_PORT = {'lsu2dcache': 'out.lsu2dcache', 'peripheral_req': 'out.peripheral_req', 'commit_store': 'out.commit_store',
            'csr_status': 'csr_status'}   # module
#   output ports bound to BackTop's interface

def as_wires(cls, b):
    """a body in the module-port form, rewritten to the Wires form the scan reads: a module's in.F-> / out.F, BackTop's
    obj->out.F (the scan rules stay as they were)"""
    if b is None: return b
    if cls == 'BackTop':
        return re.sub(r'\b\w+->out\.(' + '|'.join(sorted(OUTS, key=len, reverse=True)) + r')\b', r'\1', b)
    b = re.sub(r'\bin\.top_in->', 'wires->in.', b)
    b = re.sub(r'\(\*in\.top_in\)', 'wires->in', b)
    for p, sig in EXT_PORT.items():
        b = re.sub(r'\bout\.' + p + r'->', 'wires->' + sig + '.', b)
        b = re.sub(r'\(\*out\.' + p + r'\)', 'wires->' + sig, b)
    names = '|'.join(sorted(set(FIELDS) - {'in', 'out'}, key=len, reverse=True))
    b = re.sub(r'(?<![\w.>])in\.(' + names + r')->', r'wires->\1.', b)
    b = re.sub(r'\(\*in\.(' + names + r')\)', r'wires->\1', b)
    b = re.sub(r'(?<![\w.>])in\.(' + names + r')\b', r'wires->\1', b)
    b = re.sub(r'(?<![\w.>])out\.(' + names + r')\b', r'wires->\1', b)
    return b

srcs = {}
for d, _, fs in os.walk(os.path.join(ROOT, 'back-end')):
    for f in fs:
        if f.endswith('.cpp'): srcs[os.path.join(d, f)] = open(os.path.join(d, f), errors='ignore').read()

def body_of(cls, fn):
    pat = re.compile(r'\b' + cls + r'::' + fn + r'\s*\([^)]*\)\s*(?:const\s*)?\{')
    for p, t in srcs.items():
        mm = pat.search(t)
        if not mm: continue
        i = mm.end() - 1; d = 0
        for j in range(i, len(t)):
            if t[j] == '{': d += 1
            elif t[j] == '}':
                d -= 1
                if d == 0: return as_wires(cls, t[i:j + 1])
    return None

# phases in BackTop::comb() order (expanding the comb_stage_* helpers)
bt = rd('back-end/BackTop.cpp')
member_cls = {'pre': 'PreIduQueue', 'rename': 'Ren', 'dis': 'Dispatch', 'idu': 'Idu', 'isu': 'Isu', 'exu': 'Exu',
              'rob': 'Rob', 'csr': 'Csr', 'lsu': 'RealLsu', 'dtlb_mmu': 'TlbMmu'}
def expand(fn, out):
    b = re.sub(r'//[^\n]*', '', body_of('BackTop', fn))   # (a call named in a comment is not a phase)
    for call in re.finditer(r'(?:(\w+)->)?(\w+)\(\)\s*;', b):
        obj, f = call.groups()
        if obj in member_cls: out.append((obj, member_cls[obj], f))
        elif obj is None and f.startswith(('comb_stage', 'comb_begin_all')): expand(f, out)
        elif obj is None and f.startswith('comb_'): out.append(('top', 'BackTop', f))
phases = []
expand('comb', phases)
N_EARLY = len(phases)          # BackTop::comb(); the rest: BackTop::comb_late()
expand('comb_late', phases)

ASSIGN = r'\s*(?:\[[^\]]*\]|\.\w+|\(\))*\s*(?:=(?!=)|\|=|&=|\+=|-=|\^=|<<=|>>=|\+\+|--)'
def accesses(body, unqualified=False):
    """read / write sets of `field` or `field.sub` (one level below the Wires field)"""
    r, w = set(), set()
    if not body: return r, w
    def key(f, tail):   # in / out: two levels (their sub-structs are the real interfaces)
        m2 = re.match(r'\.(\w+)(\.\w+)?', tail)
        if not m2: return f
        return f + '.' + m2.group(1) + ((m2.group(2) or '') if f in ('in', 'out') else '')
    for f in FIELDS:
        pref = (r'(?:(?<![\w.>:])|\bwires->|\bw\.|\bwires_->|\bwires\.)' if unqualified else r'(?:\bwires->|\bw\.|\bwires_->|\bwires\.)') + f + r'\b'
        for mm in re.finditer(pref, body):
            tail = body[mm.end():mm.end() + 200]
            k = key(f, tail)
            if re.match(r'(?:\.\w+)?' + ASSIGN, tail) or re.match(r'\s*=\s*\{\s*\}', tail): w.add(k)
            elif re.match(r'\.\w+\s*(?:\[[^\]]*\])+\s*(?:\.\w+)*' + ASSIGN, tail) or re.match(r'(?:\.\w+)?(?:\[[^\]]*\])+(?:\.\w+)*' + ASSIGN, tail): w.add(k)
            else: r.add(k)
        # local aliases: `auto &x = w.F[.G];` (const aliases only read); `for (auto &x : w.F.G)` likewise
        for am in re.finditer(r'(const\s+)?[\w:<>]+\s*&\s*(\w+)\s*[=:]\s*' + pref + r'((?:\.\w+){0,2})[^;)]*[;)]', body):
            alias = am.group(2)
            k = f + ('.'.join(am.group(3).split('.')[:3 if f in ('in', 'out') else 2]) if am.group(3) else '')
            if am.group(1): r.add(k); continue
            wrote = False
            scope = body[am.end():am.end() + 600]   # (an alias is used right after its declaration)
            for um in re.finditer(r'\b' + alias + r'\b', scope):
                tail = body[am.end() + um.end():am.end() + um.end() + 200]
                if re.match(r'(?:\[[^\]]*\]|\.\w+)*' + ASSIGN, tail) or re.match(r'\s*=\s*\{\s*\}', tail): wrote = True
                else: r.add(k)
            if wrote: w.add(k)
    return r, w

def overlap(a, b):
    """field keys a (set) and b (set) name a common signal: equal, or one is the whole field of the other"""
    out = set()
    for x in a:
        for y in b:
            if x == y or y.startswith(x + '.') or x.startswith(y + '.'): out.add(x if len(x) >= len(y) else y)
    return out

def body_closure(cls, fn, depth=4, seen=None):
    """the phase body plus the bodies of the same-class member functions it calls (to `depth` levels)"""
    seen = seen if seen is not None else set()
    b = body_of(cls, fn)
    if not b or (cls, fn) in seen: return ''
    seen.add((cls, fn))
    out = b
    if depth > 0:
        for c in set(re.findall(r'\b(\w+)\s*\(', b)):
            if c != fn and body_of(cls, c): out += body_closure(cls, c, depth - 1, seen)
    return out

rows = []
for obj, cls, fn in phases:
    r, w = accesses(body_closure(cls, fn), unqualified=(cls == 'BackTop'))   # BackTop is the Wires: unqualified fields
    rows.append((f'{obj}.{fn}', r, w))

# reviewed false positives of the static scan (reader, field, writer): reason
KNOWN = {
    ('lsu.comb_outputs', 'lsu_awake.wake', 'lsu.comb_writeback'): 'cal() clears the wake slots (a write through a range-for alias)',
}
hazards, staged = [], 0
for i, (n, r, w) in enumerate(rows):
    for j in range(i + 1, len(rows)):
        for f in overlap(r, rows[j][2]):
            if (n, f, rows[j][0]) in KNOWN: continue
            # an earlier phase of the cycle writes it too: the read sees this cycle's value (staged), not a register
            if any(overlap({f}, rows[k][2]) for k in range(i)): staged += 1; continue
            hazards.append((n, f, rows[j][0]))
edges = collections.Counter()
for i, (n, r, w) in enumerate(rows):
    for j, (n2, r2, w2) in enumerate(rows):
        if i != j:
            if overlap(w, r2): edges[(n.split('.')[0], n2.split('.')[0])] += 1
modx = sum(1 for (a, b) in edges if a != b)
reentry = collections.Counter(n.split('.')[0] for n, _, _ in rows)

out = []
out.append(f'phases: {len(rows)}, module-to-module wire edges: {modx}, read-before-later-write hazards: {len(hazards)}')
out.append('phases per module: ' + ', '.join(f'{k} {v}' for k, v in reentry.most_common()))
for n, r, w in rows: out.append(f'{n:32s} R: {" ".join(sorted(r))} | W: {" ".join(sorted(w))}')
out.append(f'hazards (reader, field, later writer; {staged} staged reads not listed):')
for h in hazards: out.append('  %s reads %s, written later by %s' % h)
print('\n'.join(out))
all_hazards = list(hazards)

# Module contracts: "// CONTRACT <phase> in: <fields|-> out: <fields|->" and "// CONTRACT owns: <fields>" in a back-end header.
# A phase's declared in / out must equal the Wires fields it reads / writes (reads of the module's own outputs are not
# inputs); an owned field is written by no other module.
contract_errors = []
def cfield(k):   # a contract names a Wires field; BackTop's shared in / out structs one level deeper
    p = k.split('.')
    return '.'.join(p[:2]) if p[0] in ('in', 'out') and len(p) > 1 else p[0]
hdr_text = {}
for d0, _, fs in os.walk(os.path.join(ROOT, 'back-end')):
    for f0 in fs:
        if f0.endswith('.h'): hdr_text[os.path.join(d0, f0)] = open(os.path.join(d0, f0), errors='ignore').read()
cls_of = {o: c for o, c in member_cls.items()}
cls_of['top'] = 'BackTop'
row_of = {n: (r, w) for n, r, w in rows}
writers = collections.defaultdict(set)
for n, r, w in rows:
    for k in w: writers[cfield(k)].add(n.split('.')[0])
n_contract = 0
for obj, cls in cls_of.items():
    t = None
    for txt in hdr_text.values():   # the comment block directly above the class definition
        m = re.search(r'((?:^//[^\n]*\n)+)class ' + cls + r'\s*(?::[^{;]*)?\{', txt, re.M)
        if m and 'CONTRACT' in m.group(1): t = m.group(1); break
    if not t: continue
    owns = set()
    for m in re.finditer(r'CONTRACT owns:([^\n]*)', t): owns |= set(m.group(1).split())
    for m in re.finditer(r'CONTRACT (\w+)\s+in:(.*?)\s+out:([^\n]*)', t):
        ph, din, dout = m.group(1), set(m.group(2).split()) - {'-'}, set(m.group(3).split()) - {'-'}
        name = f'{obj}.{ph}'
        if name not in row_of: contract_errors.append(f'{name}: declared but not a phase'); continue
        r, w = row_of[name]
        sin = {cfield(k) for k in r} - owns
        sout = {cfield(k) for k in w}
        if sin != din: contract_errors.append(f'{name}: reads {sorted(sin)}, declared in {sorted(din)}')
        if sout != dout: contract_errors.append(f'{name}: writes {sorted(sout)}, declared out {sorted(dout)}')
        n_contract += 1
    for f0 in owns:
        other = writers[f0] - {obj}
        if other: contract_errors.append(f'{obj} owns {f0}, also written by {sorted(other)}')
# the trace-bridge users (side effects outside the Wires) must be the ones gen_comb_order.py orders by TB_ORDER
import gen_comb_order
tb_users = set()
for (obj, cls, fn) in phases:
    if re.search(r'tbhook::|g_bridge', re.sub(r'//[^\n]*', '', body_closure(cls, fn))): tb_users.add(f'{obj}.{fn}')
if tb_users != set(gen_comb_order.TB_ORDER):
    contract_errors.append(f'trace-bridge users {sorted(tb_users)} != TB_ORDER {sorted(gen_comb_order.TB_ORDER)}')
# BackTop::comb() must be the order generated from the contracts
import subprocess
if subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gen_comb_order.py'), '--check'],
                  capture_output=True).returncode != 0:
    contract_errors.append('BackTop::comb() differs from the order generated from the contracts (gen_comb_order.py --write)')
print(f'contracts: {n_contract} phases declared, {len(contract_errors)} mismatches')
for e in contract_errors: print('  contract: ' + e)
all_hazards += contract_errors

# =====================================================================================================================
# Front end and memory subsystem.  Their phases are member functions of one class (FrontComb) or the statements of
# MemSubsystem::comb_outputs / comb_inputs; their signals are that class's data members (accessed unqualified).
# A sub-block's comb method reads the signals its `in.F` pointers are bound to and writes those of its `out.F`
# pointers (bindings parsed from MemSubsystem::init).  A member passed to a function as a non-const reference /
# pointer argument counts as written and read (parameter constness from the function definition; unresolved
# callees count as reads).
# =====================================================================================================================
from comb_domains import front_domain, mem_domain
for name, (drows, dhaz, notes) in (('front', front_domain(ROOT, ASSIGN, overlap)), ('mem', mem_domain(ROOT, ASSIGN, overlap))):
    print(f'\n== {name}: phases: {len(drows)}, read-before-later-write hazards: {len(dhaz)}')
    for n, r, w in drows: print(f'{n:40s} R: {" ".join(sorted(r))} | W: {" ".join(sorted(w))}')
    print('hazards (reader, signal, later writer):')
    for h in dhaz: print('  %s reads %s, written later by %s' % h)
    for x in notes: print('  note: ' + x)
    all_hazards += dhaz
# top level: SimCpu::cycle() (see comb_top.py)
from comb_top import top_domain
trows, tnotes, toverlap, texcl, tprior = top_domain(ROOT, ASSIGN, rows, back_cls=cls_of, back_early=N_EARLY)
# reviewed: the AXI interconnect keeps some of its own last-cycle outputs in the shared port structs (the downstream
# AR / AW / W valid it drove, the write-port req.ready) and reads them in comb_outputs: registers inside the AXI kit,
# to become explicit when the AXI kit is reworked (docs/REG_WIRE.md)
TOP_KNOWN = {('axi.interconnect.comb_outputs', f'axi.interconnect.{io}.{ch}.{ch}valid', 'axi.interconnect.comb_inputs'): 'AXI kit'
             for io in ('axi_ddr_io', 'axi_mmio_io') for ch in ('ar', 'aw', 'w')}
TOP_KNOWN[('axi.interconnect.comb_outputs', 'axi.interconnect.write_ports[].req.ready', 'axi.interconnect.comb_inputs')] = 'AXI kit'
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
# all levels in one dependence graph (global_graph.py)
from comb_top import TOP_DETAIL
import global_graph
gtext, gsame = global_graph.report(TOP_DETAIL, toverlap)
print(gtext)
if not gsame: all_hazards.append('global schedule')
# SimCpu::cycle() must be the order generated from the tops' EXPORT declarations, and the declarations must match the code
if '--check' in sys.argv:
    p = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gen_top_order.py'), '--check'],
                       capture_output=True, text=True)
    print(p.stdout.rstrip())
    if p.returncode != 0: all_hazards.append('top exports')
    p = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gen_mem_order.py'), '--check'],
                       capture_output=True, text=True)
    print(p.stdout.rstrip())
    if p.returncode != 0: all_hazards.append('mem contracts')
    p = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gen_front_order.py'), '--check'],
                       capture_output=True, text=True)
    print(p.stdout.rstrip())
    if p.returncode != 0: all_hazards.append('front contracts')
if '--check' in sys.argv and all_hazards: sys.exit(1)
if '--dot' in sys.argv:
    with open(sys.argv[sys.argv.index('--dot') + 1], 'w') as fdot:
        fdot.write('digraph comb {\n  rankdir=LR;\n')
        for (a, b), c in edges.items():
            if a != b: fdot.write(f'  "{a}" -> "{b}" [label="{c}"];\n')
        fdot.write('}\n')

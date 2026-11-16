"""Global comb graph (comb_graph.py prints it): all levels' phases in one dependence graph.

The rows of SimCpu::cycle() (comb_top.py) with each summarised sub-object call (back.comb, front.comb_core,
mem_subsystem.comb_outputs / comb_inputs) replaced by that domain's own phases, as SimCpu paths. The current order is
one valid schedule; the edges that any other schedule must keep are derived from it:
  * data: a write before a read of the same signal, a read before a write, two writes (read-after-write,
    write-after-read, write-after-write; rows in different branches of one `if` are exclusive);
  * state inside an object: phases of one unit keep their order (a back-end module, the front core, one memory block,
    one AXI object; their registers / members are not signals in the scans);
  * global side effects: the trace bridge (tbhook::, the functional memory) and each PMU event bus (g_pmu_*) are
    pseudo-signals every user reads and writes, so their users keep their order.
Any topological order of this graph computes the same values. The minimal segmentation is the number of contiguous runs
per top module (FrontTop, BackTop, MemSubsystem, AxiSubsystem, SimCpu's glue) that a schedule needs. derive_cuts()
schedules the graph with each module's phases in their current order (one chain per module) and staying in a module
while it can; where another module has to run in between, the module is cut. comb_graph.py --check requires this derived
schedule to be the order of cycle(): the levels' generators plus the exported splits are what a global schedule gives.
"""
import re, collections

TOPS = ('back', 'front', 'mem_subsystem', 'axi', 'f2b')


def pseudo(text):
    t = re.sub(r'//[^\n]*', '', text or '')
    s = set()
    if re.search(r'tbhook::|g_bridge', t): s.add('global.trace_bridge')
    for m in re.finditer(r'\bg_pmu_(\w+)', t): s.add('global.pmu_' + m.group(1))
    return s


def module_of(label, group):
    for h in (label.split(':')[0].split('.')[0], group.split('.')[0]):
        if h in TOPS: return h
    if group.startswith('axi_'): return 'axi'
    return 'simcpu'


def unit_of(label, group, inner_of):
    """the object whose state a row's phase touches beyond the scanned signals"""
    base = label.split(':')[0].split('#')[0]
    p = base.split('.')
    if inner_of in ('back.comb', 'back.comb_late'): return 'back.' + (p[1] if p[1] != 'pmu_sample' else 'top')
    if inner_of == 'front.comb_core': return 'front.core'
    if inner_of and inner_of.startswith('mem_subsystem.'): return 'mem_subsystem.' + (p[1] if len(p) > 2 else 'top')
    if base in ('back.comb_csr_status',): return 'back.csr'
    if base in ('front.read_core_q', 'front.comb_core'): return 'front.core'
    if base.startswith(('axi.interconnect', 'axi.ddr', 'axi.mmio')): return '.'.join(p[:2])
    if base.startswith('mem_subsystem.llc'): return 'mem_subsystem.llc'
    if base == 'mem_subsystem.comb_commit_store': return 'mem_subsystem.peripheral'
    return 'group:' + group                          # SimCpu / FrontTop / AxiSubsystem glue: its statements in order


def build(detail, overlap):
    """rows: (label, R, W, module, tag, unit, node). node: what a schedule moves as one piece: a phase of an expanded
    domain on its own, else the whole top-level call of cycle() it belongs to (SimCpu / FrontTop / AxiSubsystem glue
    and sub-object calls stay together; a top-level call is contiguous in the current order)."""
    expanded = {g for n, r, w, g, t, inner in detail if inner}
    rows = []
    for n, r, w, g, t, inner in detail:
        if inner:
            suffix = n[n.index('#'):] if '#' in n else ''
            for ln, lr, lw, txt in inner:
                ps = pseudo(txt)
                rows.append((ln + suffix, set(lr) | ps, set(lw) | ps, module_of(ln, g), t, unit_of(ln, g, n.split('#')[0]),
                             'phase:' + ln + suffix))
        elif g in expanded:
            rows.append((n, set(r), set(w), module_of(n, g), t, unit_of(n, g, None), 'row:%d' % len(rows)))
        else:
            rows.append((n, set(r), set(w), module_of(g, g), t, unit_of(n, g, None), 'call:' + g))

    def exclusive(i, j):
        di, dj = dict(rows[i][4]), dict(rows[j][4])
        return any(k in dj and dj[k] != b for k, b in di.items())

    succ = collections.defaultdict(set)
    data = []   # (i, j, kind, signals) for cross-module data edges
    for i in range(len(rows)):
        for j in range(i + 1, len(rows)):
            if exclusive(i, j): continue
            ni, ri, wi = rows[i][0], rows[i][1], rows[i][2]
            rj, wj = rows[j][1], rows[j][2]
            raw, war, waw = overlap(wi, rj), overlap(ri, wj), overlap(wi, wj)
            if raw or war or waw or rows[i][5] == rows[j][5]:
                succ[i].add(j)
            if raw and rows[i][3] != rows[j][3]: data.append((i, j, sorted(raw)))
    return rows, succ, data


def segments(order, rows):
    seg = []
    for i in order:
        m = rows[i][3]
        if not seg or seg[-1][0] != m: seg.append([m, 0])
        seg[-1][1] += 1
    return seg


def greedy(rows, succ):
    """a topological order that stays in the current module while it can (ties: the current position)"""
    indeg = collections.Counter()
    for i in succ:
        for j in succ[i]: indeg[j] += 1
    ready = {i for i in range(len(rows)) if indeg[i] == 0}
    out, cur = [], None
    while ready:
        same = [i for i in ready if rows[i][3] == cur]
        i = min(same) if same else min(ready)
        cur = rows[i][3]
        ready.discard(i); out.append(i)
        for j in succ[i]:
            indeg[j] -= 1
            if indeg[j] == 0: ready.add(j)
    if len(out) != len(rows): raise SystemExit('global graph: cycle')
    return out


def report(detail, overlap):
    rows, succ, data = build(detail, overlap)
    cur = segments(range(len(rows)), rows)
    g = derive_cuts(rows, succ)
    gs = segments(g, rows)
    same = g == list(range(len(rows)))
    out = [f'\n== global: phases: {len(rows)} (all levels), dependence edges: {sum(len(v) for v in succ.values())}, '
           f'cross-module data edges: {len(data)}']
    out.append('current order:  ' + ' | '.join(f'{m} {n}' for m, n in cur) + f'   ({len(cur)} runs)')
    out.append('derived order:  ' + ' | '.join(f'{m} {n}' for m, n in gs) + f'   ({len(gs)} runs)')
    out.append('derived schedule: ' + ('identical to cycle() (each module cut where the global graph needs it)' if same else
                                       'DIFFERS from cycle()'))
    for m, sgs in sorted(cuts_of(g, rows).items()):
        out.append(f'  {m}: ' + ' | '.join(f'{s[0]} .. {s[-1]} ({len(s)})' if len(s) > 1 else s[0] for s in sgs))
    pairs = collections.defaultdict(list)
    for i, j, sig in data: pairs[(rows[i][3], rows[j][3])].append((rows[i][0], rows[j][0], sig))
    out.append('cross-module data edges (writer -> reader: signals):')
    for (a, b), es in sorted(pairs.items()):
        out.append(f'  {a} -> {b}: {len(es)}')
        for wn, rn, sig in es:
            out.append(f'    {wn} -> {rn}: {" ".join(sig[:4])}{" ..." if len(sig) > 4 else ""}')
    side = collections.defaultdict(list)
    for row in rows:
        for s in sorted(x for x in row[1] if x.startswith('global.')): side[s].append(row[0])
    out.append('global side effects (users keep their order):')
    for s, us in sorted(side.items()): out.append(f'  {s}: {" ".join(us)}')
    return '\n'.join(out), same


def derive_cuts(rows, succ):
    """A schedule of the nodes (build()) in which each top module's nodes keep their current order (one chain per
    module: the levels' own generators order them) and that stays in the current module while it can. The places where
    another module has to run in between are the module's cut points. Returns the row order."""
    node_ids = {}
    for row in rows: node_ids.setdefault(row[6], len(node_ids))
    nn = len(node_ids)
    of = [node_ids[row[6]] for row in rows]
    nmod = [None] * nn
    for i, row in enumerate(rows): nmod[of[i]] = row[3]
    nsucc = collections.defaultdict(set)
    for i, js in succ.items():
        for j in js:
            if of[i] != of[j]: nsucc[of[i]].add(of[j])
    last = {}
    for k in range(nn):
        if nmod[k] in last: nsucc[last[nmod[k]]].add(k)
        last[nmod[k]] = k
    norder = greedy([(None, None, None, m) for m in nmod], nsucc)
    members = collections.defaultdict(list)
    for i in range(len(rows)): members[of[i]].append(i)
    return [i for k in norder for i in members[k]]


def cuts_of(order, rows):
    """per module: its segments as lists of row labels, in schedule order"""
    seg = collections.defaultdict(list)
    prev = None
    for i in order:
        m = rows[i][3]
        if m != prev: seg[m].append([])
        seg[m][-1].append(rows[i][0])
        prev = m
    return seg

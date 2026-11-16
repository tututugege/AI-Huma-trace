#!/usr/bin/env python3
"""Back-end comb order with merged phases.

Takes the back-end phases of BackTop::comb() and their read / write sets from comb_graph.py, the ordering constraints
between two phases i < j of the current order (W_i & R_j: j needs i's value; W_i & W_j: the later writer decides the
value; R_i & W_j: i must see the value before j overwrites it), contracts the given groups (a group = phases of one
module merged into one call) and prints a topological order that keeps the current order where it can.

usage: comb_order.py --schedule          a whole order with few module switches (each run of one module = one phase)
       comb_order.py [--groups]            the dependency groups per module (greedy, current order)
       comb_order.py MOD [MOD ...]         the new order with all phases of each MOD merged by its dependency groups
"""
import os, sys, io, contextlib, heapq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def load():
    g = {'__file__': os.path.join(HERE, 'comb_graph.py')}
    src = open(os.path.join(HERE, 'comb_graph.py')).read().split('# top level: SimCpu::cycle()')[0]
    argv = sys.argv
    sys.argv = ['comb_graph.py']
    with contextlib.redirect_stdout(io.StringIO()):
        exec(src, g)
    sys.argv = argv
    # what each phase touches outside the Wires: other modules' objects, and the trace bridge (global side effects)
    import re
    touch = []
    for obj, cls, fn in g['phases']:
        b = re.sub(r'//[^\n]*', '', g['body_closure'](cls, fn))
        t = set(re.findall(r'\b(pre|idu|rename|dis|isu|exu|rob|csr|lsu)\s*->', b))
        if re.search(r'\bftq\s*->', b): t.add('pre')
        if re.search(r'\bcsr\s*->', b): t.add('csr')
        if 'tbhook::' in b or 'g_bridge' in b: t.add('tb')
        t.discard(obj)
        touch.append(t)
    return g['rows'], g['overlap'], touch


def edges(rows, overlap, touch):
    n = len(rows)
    e = [set() for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            ri, wi = rows[i][1], rows[i][2]
            rj, wj = rows[j][1], rows[j][2]
            if overlap(wi, rj) or overlap(wi, wj) or overlap(ri, wj): e[i].add(j)
    # a module's own phases keep their order: they also communicate through the module's private state, which the
    # Wires scan does not see
    mod = [x[0].split('.')[0] for x in rows]
    for i in range(n):
        nxt = next((j for j in range(i + 1, n) if mod[j] == mod[i]), None)
        if nxt is not None: e[i].add(nxt)
    # a phase that touches module M's object keeps its order against M's phases; trace-bridge users keep their order
    for i in range(n):
        for j in range(i + 1, n):
            if (mod[j] in touch[i]) or (mod[i] in touch[j]) or ('tb' in touch[i] and 'tb' in touch[j]): e[i].add(j)
    return e


def reach_from(e, n):
    r = [set() for _ in range(n)]
    for i in range(n - 1, -1, -1):
        for j in e[i]:
            r[i].add(j)
            r[i] |= r[j]
    return r


def dep_groups(rows, e, r, module):
    """greedy: consecutive phases of `module` share a group unless a path leaves the group through another phase"""
    n = len(rows)
    mod = [x[0].split('.')[0] for x in rows]
    ph = [i for i in range(n) if mod[i] == module]
    groups = [[ph[0]]]
    for p in ph[1:]:
        cur = groups[-1]
        # p joins if no path g -> x -> p with x outside the group (contracting stays acyclic)
        bad = any(x not in cur and x < p and p in (r[x] | e[x]) for g in cur for x in (r[g] | e[g]))
        if bad: groups.append([p])
        else: cur.append(p)
    return groups


def order(rows, e, groups):
    """topological order of the contracted graph; ties broken by the smallest original index"""
    n = len(rows)
    node = list(range(n))
    for gi, g in enumerate(groups):
        for p in g: node[p] = n + gi
    nodes = sorted(set(node))
    key = {v: min(p for p in range(n) if node[p] == v) for v in nodes}
    succ = {v: set() for v in nodes}
    indeg = {v: 0 for v in nodes}
    for i in range(n):
        for j in e[i]:
            a, b = node[i], node[j]
            if a != b and b not in succ[a]:
                succ[a].add(b)
                indeg[b] += 1
    h = [(key[v], v) for v in nodes if indeg[v] == 0]
    heapq.heapify(h)
    out = []
    while h:
        _, v = heapq.heappop(h)
        out.append(v)
        for w in succ[v]:
            indeg[w] -= 1
            if indeg[w] == 0: heapq.heappush(h, (key[w], w))
    if len(out) != len(nodes): sys.exit('cycle: the groups cannot be merged')
    return [(v, [p for p in range(n) if node[p] == v]) for v in out]


def best_schedule(rows, e, trials=4000, seed=1):
    """a topological order with few module switches: list scheduling that keeps the current module while it has a
    ready phase (ties: random, many trials); returns the runs (module, [phases]) of the best order found"""
    import random
    n = len(rows)
    mod = [x[0].split('.')[0] for x in rows]
    pred = [0] * n
    for i in range(n):
        for j in e[i]: pred[j] += 1
    rnd = random.Random(seed)
    best = None
    for t in range(trials):
        indeg = pred[:]
        ready = [i for i in range(n) if indeg[i] == 0]
        seq, last = [], None
        while ready:
            same = [i for i in ready if mod[i] == last]
            if same: pick = min(same)
            elif t == 0: pick = min(ready)                      # trial 0: the current order where possible
            else:
                # prefer a module that has many phases ready now
                cnt = {}
                for i in ready: cnt[mod[i]] = cnt.get(mod[i], 0) + 1
                top = max(cnt.values())
                pick = rnd.choice([i for i in ready if cnt[mod[i]] == top]) if rnd.random() < 0.7 else rnd.choice(ready)
            ready.remove(pick)
            seq.append(pick)
            last = mod[pick]
            for j in e[pick]:
                indeg[j] -= 1
                if indeg[j] == 0: ready.append(j)
        runs = []
        for p in seq:
            if runs and runs[-1][0] == mod[p]: runs[-1][1].append(p)
            else: runs.append((mod[p], [p]))
        if best is None or len(runs) < len(best): best = runs
    return best


MEM_MODULE = {'dcache_stage2': 'dcache_', 'dcache_stage1': 'dcache_', 'ptw_request': 'ptw_block', 'ptw_feedback': 'ptw_block'}


def mem_rows(group):
    """the mem rows of MemSubsystem::comb_outputs / comb_inputs, labelled <block>.<phase>"""
    g = {'__file__': os.path.join(HERE, 'comb_graph.py')}
    src = open(os.path.join(HERE, 'comb_graph.py')).read().split('# top level: SimCpu::cycle()')[0]
    argv = sys.argv
    sys.argv = ['comb_graph.py']
    with contextlib.redirect_stdout(io.StringIO()):
        exec(src, g)
    sys.argv = argv
    import comb_domains
    rows = []
    for n, r, w in comb_domains.MEM_GROUPS[group]:
        m = MEM_MODULE.get(n, n.split('.')[0] if '.' in n else n)
        rows.append((f'{m}.{n.split(".")[-1]}', r, w))
    return rows, g['overlap'], [set() for _ in rows]


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--mem':
        rows, overlap, touch = mem_rows(sys.argv[2])
        e = edges(rows, overlap, touch)
        runs = best_schedule(rows, e)
        print(f'{len(rows)} phases -> {len(runs)} runs')
        for m, ps in runs: print('  ' + ' + '.join(rows[p][0] for p in ps))
        return
    rows, overlap, touch = load()
    e = edges(rows, overlap, touch)
    r = reach_from(e, len(rows))
    mods = sorted({x[0].split('.')[0] for x in rows})
    if len(sys.argv) > 1 and sys.argv[1] == '--schedule':
        runs = best_schedule(rows, e)
        per = {}
        for m, ps in runs: per[m] = per.get(m, 0) + 1
        print(f'{len(rows)} phases -> {len(runs)} runs: ' + ', '.join(f'{m} {c}' for m, c in sorted(per.items(), key=lambda x: -x[1])))
        for m, ps in runs: print('  ' + ' + '.join(rows[p][0] for p in ps))
        return
    if len(sys.argv) < 2 or sys.argv[1] == '--groups':
        tot = 0
        for m in mods:
            gs = dep_groups(rows, e, r, m)
            tot += len(gs)
            print(f'{m:7s} ' + ' | '.join(' '.join(rows[p][0].split('.')[1] for p in g) for g in gs))
        print(f'{len(rows)} phases -> {tot}')
        return
    groups = []
    for m in sys.argv[1:]:
        groups += [g for g in dep_groups(rows, e, r, m) if len(g) > 1]
    res = order(rows, e, groups)
    print(f'{len(rows)} phases -> {len(res)}')
    for v, ps in res:
        print('  ' + ' + '.join(rows[p][0] for p in ps))


if __name__ == '__main__':
    main()

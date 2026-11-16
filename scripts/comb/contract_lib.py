"""Shared by the order generators (gen_comb_order / gen_top_order / gen_mem_order / gen_front_order): the scan of the
code (comb_graph.py, run once), the topological order with a fixed tie-break, the generated-block rewrite."""
import os, sys, io, heapq, contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
_scanned = False


def scan():
    """run comb_graph.py once (without --check): fills comb_top.TOP_PHASES, comb_domains.MEM_GROUPS / MEM_BLOCKS /
    MEM_BIND / FRONT_ROWS; returns (comb_top, comb_domains)"""
    global _scanned
    sys.path.insert(0, HERE)
    if not _scanned:
        argv = sys.argv; sys.argv = [os.path.join(HERE, 'comb_graph.py')]
        g = {'__file__': os.path.join(HERE, 'comb_graph.py'), '__name__': 'comb_graph_scan'}
        with contextlib.redirect_stdout(io.StringIO()):
            exec(open(os.path.join(HERE, 'comb_graph.py')).read(), g)
        sys.argv = argv
        _scanned = True
    import comb_top, comb_domains
    return comb_top, comb_domains


def topo(names, succ, rank, what='the contracts'):
    """names in an order where a -> b (succ) holds; ready nodes taken by rank; a cycle exits with the nodes on it"""
    indeg = {n: 0 for n in names}
    for a in names:
        for b in succ[a]: indeg[b] += 1
    ready = [(rank[n], n) for n in names if indeg[n] == 0]
    heapq.heapify(ready)
    order = []
    while ready:
        _, n = heapq.heappop(ready)
        order.append(n)
        for b in sorted(succ[n]):
            indeg[b] -= 1
            if indeg[b] == 0: heapq.heappush(ready, (rank[b], b))
    if len(order) != len(names): sys.exit(f'{what} have a cycle: ' + ' '.join(n for n in names if indeg[n] > 0))
    return order


def block_span(src, begin, end, start=0):
    """(a, b): the generated block from its BEGIN line through its END line, the first one at or after start"""
    a = src.find(begin, start)
    if a < 0: return None
    return a, src.index(end, a) + len(end)

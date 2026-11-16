"""Top domain of comb_graph.py: SimCpu::cycle() up to the first seq call.

Every access is a path from SimCpu (`back.out.lsu2dcache`, `axi.interconnect.read_ports[MASTER_ICACHE].req.ready`,
`mem_subsystem.mshr_axi_in.req_ready`); an index that is a constant (MASTER_*) is kept, any other index is the
wildcard `[]`.  SimCpu methods and free functions called from cycle() are inlined statement by statement (parameters
and local `auto &` aliases substituted).  A call into a sub-object is one phase whose read / write set is a summary:
  back.comb                        the back-end rows (Wires field F -> back.F; BackTop is the Wires)
  front.comb_core                   the front rows (in.F -> front.in.F, out.F -> front.out.F) + the I-cache ports
                                   (icache_top.cpp: bind_.read -> the AXI ICACHE read port, probe_req / probe_resp ->
                                   SimCpu::icache_req / icache_resp) + the ITLB walk port
  mem_subsystem.comb_outputs / comb_inputs   the mem rows of that function, pointer members through SimCpu::init()
  axi.interconnect.* / axi.ddr.* / axi.mmio.* / mem_subsystem.llc_*   a scan of that class's member function closure
FrontTop's comb / comb_from_back and AxiSubsystem's comb phases are inlined like SimCpu's methods (members -> front.* / axi.*).
A PtwWalkPort is two signals: <port>.in (the TLB's request / consume, read by MemSubsystem::comb_inputs) and
<port>.out (the response the TLB reads, written by sync_ptw_port_outputs in comb_inputs and in MemSubsystem::seq).
"""
import re, os
from comb_domains import Src, strip_comments, statements, split_top, enclosing_call, closure, MEM_GROUPS, FRONT_ROWS

ROOTS = ('back', 'front', 'mem_subsystem', 'axi', 'icache_req',
         'icache_resp', 'icache_resp_q', 'icache_axi_q', 'q_', 'd', 'ctx')
SUBOBJ = ('back', 'front', 'mem_subsystem', 'axi', 'axi.interconnect', 'axi.ddr', 'axi.mmio')
SEQ_CALLS = ('front.seq_core',)
# methods of a sub-object inlined like SimCpu's: call -> (class, member pattern prefixed with the object)
INLINE = {'front.comb': ('FrontTop', r'back2front_stall_1|back2front_stall|back2front_1|back2front|in|out'),
          'front.comb_from_back': ('FrontTop', r'back2front_stall_1|back2front_stall|back2front_1|back2front|in|out'),
          'axi.comb_begin': ('AxiSubsystem', r'interconnect|ddr|mmio|uart'),
          'axi.comb_outputs': ('AxiSubsystem', r'interconnect|ddr|mmio|uart'),
          'axi.comb_inputs': ('AxiSubsystem', r'interconnect|ddr|mmio|uart')}
BOUND = {'AxiSubsystem': [(r'\bllc->', 'mem_subsystem.')]}   # pointer members of an inlined class -> the object bound
TOP_PHASES = []   # filled by top_domain: the SimCpu-level phases with their read / write sets
TOP_DETAIL = []   # filled by top_domain: per row of cycle() (label, R, W, group, tag, inner); inner: for a sub-object call
                  # summarised from a domain, that domain's phases [label, R, W, source text] as SimCpu paths, else None
DEPTH = 4                                   # path components kept below the root


def norm_path(p):
    p = re.sub(r'\s+', '', p).replace('->', '.')
    p = re.sub(r'^\(\*?(\w+)\)', r'\1', p)
    p = re.sub(r'\[(?!MASTER_)[^\]]*\]', '[]', p)
    p = re.sub(r'^(?:cpu|this)\.', '', p)
    parts = re.findall(r'\w+(?:\[[^\]]*\])?', p)
    return '.'.join(parts[:DEPTH + 1])


def comps(k):
    return k.split('.')


def same(a, b):
    """two path components name the same thing ([] matches any index)"""
    na, ia = re.match(r'(\w+)(\[[^\]]*\])?', a).groups()
    nb, ib = re.match(r'(\w+)(\[[^\]]*\])?', b).groups()
    if na != nb: return False
    if not ia or not ib or ia == '[]' or ib == '[]': return True
    return ia == ib


def overlap(ra, wb):
    out = set()
    for x in ra:
        cx = comps(x)
        for y in wb:
            cy = comps(y)
            n = min(len(cx), len(cy))
            if all(same(cx[i], cy[i]) for i in range(n)): out.add(x if len(cx) >= len(cy) else y)
    return out


PATH = r'(?<![\w.>:])(%s)\b((?:\s*\[[^\]]*\]|\s*(?:\.|->)\s*\w+)*)'


def scan_paths(body, ASSIGN, src, notes, owner, roots=ROOTS, prefix='', types=None):
    """read / write sets of the paths that start at one of `roots` (each key gets `prefix`)"""
    r, w = set(), set()
    body = strip_comments(body)
    for m in re.finditer(PATH % '|'.join(roots), body):
        full = m.group(1) + m.group(2)
        tail = body[m.end():m.end() + 200]
        if re.search(r'auto\s*\*\s*\w+\s*=\s*&?\s*$', body[max(0, m.start() - 40):m.start()]) and re.match(r'\s*;', tail):
            continue                    # `auto *p = <path>;` takes an address, it reads nothing
        if re.match(r'\s*\(', tail):    # a method call inside an expression: a read of <object>.<method>
            if not m.group(2): continue
            mm = re.match(r'(.*?)\s*(?:\.|->)\s*(clear|fill|reset|assign)$', full, re.S)
            if mm: w.add(prefix + norm_path(mm.group(1))); continue
        k = prefix + norm_path(full)
        if re.match(ASSIGN, tail) or re.match(r'\s*=\s*\{\s*\}', tail) or \
           re.match(r'\s*\.\s*(?:clear|fill|reset|assign)\s*\(', tail) or re.search(r'memset\s*\(\s*&?\s*$', body[max(0, m.start() - 12):m.start()]):
            w.add(k); continue
        call = enclosing_call(body, m.start())
        if call and re.search(r'&\s*$', body[max(0, m.start() - 3):m.start()]):
            r.add(k); continue      # an address passed: a binding (what the callee does is in its own summary)
        if call:
            if call[2]:
                cls = (types or {}).get(call[2]); pw = src.params_written(call[0], cls) if cls else None
            else:
                pw = src.params_written(call[0])
            if pw is not None and call[1] < len(pw) and pw[call[1]]:
                w.add(k)
                if pw[call[1]] == 'rw': r.add(k)
                continue
        r.add(k)
    return r, w


def top_domain(root, ASSIGN, back_rows, back_cls=None, back_early=None):
    if back_early is None: back_early = len(back_rows)
    src = Src(root, ['back-end', 'front-end', 'MemSubSystem', 'axi-interconnect-kit', 'include'])
    for fn in os.listdir(root):
        if fn.endswith('.cpp'): src.t[os.path.join(root, fn)] = open(os.path.join(root, fn), errors='ignore').read()
    notes = set()
    sim = strip_comments(src.t[os.path.join(root, 'SimCpu.cpp')])
    init = src.body('SimCpu', 'init')[1]

    # pointer members of MemSubsystem bound in SimCpu::init(): mem member -> top path
    membind = {m.group(1): norm_path(re.sub(r'\[\w+::(MASTER_\w+)\]', r'[\1]', m.group(2)))
               for m in re.finditer(r'\bmem_subsystem\.(\w+)\s*=\s*&\s*([\w.\[\]:>-]+)\s*;', init)}
    WALK = {'dtlb_walk_port_inst': 'mem_subsystem.dtlb_walk', 'itlb_walk_port_inst': 'mem_subsystem.itlb_walk'}

    # ---- summaries of the sub-object calls ---------------------------------------------------------------------------
    def from_rows(rows, mapk):
        r, w = set(), set()
        for _, rr, ww in rows:
            for k in rr:
                mk = mapk(k)
                if mk: r |= mk if isinstance(mk, set) else {mk}
            for k in ww:
                mk = mapk(k, True)
                if mk: w |= mk if isinstance(mk, set) else {mk}
        return r, w

    def back_key(k, write=False): return 'back.' + k
    def front_key(k, write=False):
        head = k.split('.')[0]
        if head == 'in':
            if k.startswith('in.csr_status'): return None if write else 'back.csr_status'   # a pointer to back.csr_status (read)
            return 'front.' + k
        if head == 'out': return 'front.' + k
        return None                                                    # FrontComb locals / front-internal state
    def mem_key(k, write=False):
        head = k.split('.')[0]
        if head in WALK: return None                                   # the walk ports: added explicitly below
        if head in membind: return membind[head] + k[len(head):]
        return 'mem_subsystem.' + k

    S = {}
    S['back.comb'] = from_rows(back_rows[:back_early], back_key)
    S['back.comb_late'] = from_rows(back_rows[back_early:], back_key)
    S['back.clear_comb_inputs'] = (set(), {'back.in'})
    S['back.comb_csr_status'] = ({'back.csr'}, {'back.csr_status', 'back.out.sstatus', 'back.out.mstatus', 'back.out.satp', 'back.out.privilege'})
    S['front.read_core_q'] = (set(), set())
    fr, fw = from_rows(FRONT_ROWS, front_key)
    # the I-cache ports (icache_top.cpp, ICacheTop's methods over bind_)
    ict = src.t[os.path.join(root, 'front-end', 'icache', 'icache_top.cpp')]
    ic = ''
    for fn in ('comb', 'step', 'comb_calc', 'invalidate_step', 'clear_outputs', 'read_ready'):
        mm = re.search(r'\n[\w:<>&*\s]*?\b' + fn + r'\s*\([^)]*\)\s*(?:const\s*)?\{', ict)
        if mm: ic += Src.block(ict, mm.end() - 1)
        else: notes.add(f'icache_top.cpp: no {fn}()')
    fb_ = {m.group(1): norm_path(m.group(2)) for m in re.finditer(r'\bfront\.(icache_mem_\w+)\s*=\s*&\s*([\w.\[\]:>-]+)\s*;', init)}
    fb_ = {k: re.sub(r'\[\w+::(MASTER_\w+)\]', r'[\1]', v) for k, v in fb_.items()}
    ic = re.sub(r'(?:const\s+)?auto\s*\*\s*\w+\s*=\s*bind_\.\w+\s*;', '', ic)   # pointer aliases: no access
    ic = re.sub(r'\bif\s*\(\s*(?:rp|rq|pr|bind_\.\w+)\s*\)', 'if (1)', ic)          # null tests of the port pointers
    ic = re.sub(r'\b(?:rp|rq|pr|bind_\.\w+)\s*\?', '1 ?', ic)
    ic = re.sub(r'\bbind_\.read_q\b|\brq\b', fb_.get('icache_mem_read_q', '?read_q'), ic)
    ic = re.sub(r'\bbind_\.read\b|\brp\b', 'axi.interconnect.read_ports[MASTER_ICACHE]', ic)
    ic = re.sub(r'\bbind_\.probe_resp\b|\bpr\b', fb_.get('icache_mem_resp_port', '?probe_resp'), ic)
    ic = re.sub(r'\bbind_\.probe_req\b', fb_.get('icache_mem_req_port', '?probe_req'), ic)
    ir, iw = scan_paths(ic, ASSIGN, src, notes, 'icache')
    fr |= ir | {'mem_subsystem.itlb_walk.out'}
    fw |= iw | {'mem_subsystem.itlb_walk.in'}
    S['front.comb_core'] = (fr, fw)
    br, bw = S['back.comb']
    S['back.comb'] = (br | {'mem_subsystem.dtlb_walk.out'}, bw | {'mem_subsystem.dtlb_walk.in'})
    S['mem_subsystem.comb_outputs'] = from_rows(MEM_GROUPS['comb_outputs'], mem_key)
    mr, mw = from_rows(MEM_GROUPS['comb_inputs'], mem_key)
    syncs = any(n == 'sync_ptw_port_outputs' for n, _, _ in MEM_GROUPS['comb_inputs'])   # (else only MemSubsystem::seq sets it)
    S['mem_subsystem.comb_inputs'] = (mr | {'mem_subsystem.dtlb_walk.in', 'mem_subsystem.itlb_walk.in'},
                                      mw | ({'mem_subsystem.dtlb_walk.out', 'mem_subsystem.itlb_walk.out'} if syncs else set()))

    # ---- the same calls phase by phase (global_graph.py): each domain's rows as SimCpu paths, with their source text ----
    SUB = {}
    def mapped(rows, mapk, prefix):
        out = []
        for n, rr, ww in rows:
            r, w = set(), set()
            for k in rr:
                mk = mapk(k)
                if mk: r |= mk if isinstance(mk, set) else {mk}
            for k in ww:
                mk = mapk(k, True)
                if mk: w |= mk if isinstance(mk, set) else {mk}
            out.append([prefix + n, r, w, ''])
        return out
    SUB['back.comb'] = mapped(back_rows[:back_early], back_key, 'back.')
    SUB['back.comb_late'] = mapped(back_rows[back_early:], back_key, 'back.')
    for row in SUB['back.comb'] + SUB['back.comb_late']:
        obj, fn = row[0].split('.')[1:3]
        if back_cls and obj in back_cls: row[3] = closure(src, back_cls[obj], fn) or ''
        if row[0] == 'back.lsu.comb_translate':
            row[1].add('mem_subsystem.dtlb_walk.out'); row[2].add('mem_subsystem.dtlb_walk.in')
    SUB['back.comb_late'].append(['back.pmu_sample', set(), set(), closure(src, 'BackTop', 'pmu_sample') or ''])
    for row in SUB['back.comb'] + SUB['back.comb_late']:   # calls into the other back-end modules (obj->f(...)): their text too
        for o2, f2 in set(re.findall(r'\b(\w+)->(\w+)\s*\(', row[3])):
            if back_cls and o2 in back_cls: row[3] += closure(src, back_cls[o2], f2) or ''
    def front_key2(k, write=False):   # FrontComb locals: front.c.*
        if k.split('.')[0] in ('in', 'out'): return front_key(k, write)
        return 'front.c.' + k
    SUB['front.comb_core'] = mapped(FRONT_ROWS, front_key2, 'front.')
    IC_PHASE = {'comb': 'run_icache', 'step': 'run_icache', 'comb_calc': 'run_icache', 'clear_outputs': 'run_icache',
                'invalidate_step': 'finish', 'read_ready': 'read_fifos'}
    def ic_ports(t1):
        t1 = re.sub(r'(?:const\s+)?auto\s*\*\s*\w+\s*=\s*bind_\.\w+\s*;', '', t1)
        t1 = re.sub(r'\bif\s*\(\s*(?:rp|rq|pr|bind_\.\w+)\s*\)', 'if (1)', t1)
        t1 = re.sub(r'\b(?:rp|rq|pr|bind_\.\w+)\s*\?', '1 ?', t1)
        t1 = re.sub(r'\bbind_\.read_q\b|\brq\b', fb_.get('icache_mem_read_q', '?read_q'), t1)
        t1 = re.sub(r'\bbind_\.read\b|\brp\b', 'axi.interconnect.read_ports[MASTER_ICACHE]', t1)
        t1 = re.sub(r'\bbind_\.probe_resp\b|\bpr\b', fb_.get('icache_mem_resp_port', '?probe_resp'), t1)
        t1 = re.sub(r'\bbind_\.probe_req\b', fb_.get('icache_mem_req_port', '?probe_req'), t1)
        return scan_paths(t1, ASSIGN, src, notes, 'icache')
    for row in SUB['front.comb_core']:
        ph = row[0].split('.', 1)[1]
        txt = closure(src, 'FrontComb', ph) or ''
        for cm2 in set(re.findall(r'\bicache\.(\w+)\s*\(', txt)): txt += closure(src, 'ICacheTop', cm2) or ''
        row[3] = txt
        for fn, phx in IC_PHASE.items():
            if phx != ph: continue
            mm = re.search(r'\n[\w:<>&*\s]*?\b' + fn + r'\s*\([^)]*\)\s*(?:const\s*)?\{', ict)
            if not mm: continue
            r1, w1 = ic_ports(Src.block(ict, mm.end() - 1))
            row[1] |= r1; row[2] |= w1
        if ph == 'run_icache':
            row[1].add('mem_subsystem.itlb_walk.out'); row[2].add('mem_subsystem.itlb_walk.in')
    memh = src.t.get(os.path.join(root, 'MemSubSystem', 'include', 'MemSubsystem.h'), '')
    def mem_text(n):
        if '.' in n:
            obj, meth = n.split('.', 1)
            tm = re.search(r'([\w:<>]+)\s+' + re.escape(obj) + r'\s*[;{=]', memh)
            return (closure(src, tm.group(1).split('::')[-1], meth) or '') if tm else ''
        return closure(src, 'MemSubsystem', n) or ''
    for grp in ('comb_outputs', 'comb_inputs'):
        SUB['mem_subsystem.' + grp] = mapped(MEM_GROUPS[grp], mem_key, 'mem_subsystem.')
        for row in SUB['mem_subsystem.' + grp]: row[3] = mem_text(row[0].split('.', 1)[1])
    S['mem_subsystem.comb_commit_store'] = ({'back.out.commit_store', 'mem_subsystem.peripheral_model_'},
                                            {'back.csr_interrupt_inject_io', 'mem_subsystem.peripheral_model_'})
    mr2, mw2 = S['mem_subsystem.comb_inputs']
    S['mem_subsystem.comb_inputs'] = (mr2 | {'mem_subsystem.peripheral_model_'}, mw2)
    for row in SUB['mem_subsystem.comb_inputs']:
        if row[0] == 'mem_subsystem.peripheral_axi_.comb': row[1].add('mem_subsystem.peripheral_model_')
        if row[0] == 'mem_subsystem.ptw_request': row[1] |= {'mem_subsystem.dtlb_walk.in', 'mem_subsystem.itlb_walk.in'}

    def class_summary(obj, cls, meth, members):
        b = closure(src, cls, meth)
        if not b:
            ic = src.in_class(cls, meth)
            if not ic: notes.add(f'no body for {cls}::{meth}'); return set(), set()
            b = ic[1]
        return scan_paths(b, ASSIGN, src, notes, f'{obj}.{meth}', roots=members, prefix=obj + '.')

    ax_members = ('read_ports', 'write_ports', 'axi_ddr_io', 'axi_mmio_io', 'llc_lookup_in_', 'llc_lookup_in')
    for meth in ('comb_outputs', 'comb_inputs', 'set_llc_lookup_in'):
        S['axi.interconnect.' + meth] = class_summary('axi.interconnect', 'AXI_Interconnect', meth, ax_members)
    for obj, cls in (('axi.ddr', 'SimDDR'), ('axi.mmio', 'MMIO_Bus_AXI4')):
        for meth in ('comb_outputs', 'comb_inputs'):
            S[obj + '.' + meth] = class_summary(obj, cls, meth, ('io',))
    S['mem_subsystem.llc_comb_outputs'] = class_summary('mem_subsystem', 'MemSubsystem', 'llc_comb_outputs', ('axi_kit_runtime',))
    S['mem_subsystem.llc_lookup_in'] = ({'mem_subsystem.axi_kit_runtime'}, set())

    # ---- phases: cycle() inlined -------------------------------------------------------------------------------------
    rows = []
    stop = [False]

    tags = []          # per row: the (if id, branch) pairs it is inside (rows in different branches of one if are exclusive)
    cur_tag = []
    n_if = [0]

    groups = []        # per row: the top-level call of cycle() it belongs to (a SimCpu-level phase)
    cur_group = ['']

    inners = []        # per row: the sub-object's phases (SUB) or None

    def emit(label, r, w, inner=None):
        rows.append((label, r, w)); tags.append(tuple(cur_tag)); groups.append(cur_group[0]); inners.append(inner)

    def subst(text, mp):
        for a, e in mp.items(): text = re.sub(r'(?<![\w.>])' + a + r'\b', e, text)
        return text

    def walk_body(body, where, mp, top=False):
        for st in statements(body):
            if stop[0]: return
            if top:   # a statement of cycle(): one SimCpu-level phase
                cm0 = re.match(r'([\w.]+?)\s*(?:\.|->)\s*(\w+)\s*\(', st) or re.match(r'(\w+)\s*\(', st)
                cur_group[0] = (norm_path(cm0.group(1)) + '.' + cm0.group(2)) if cm0 and cm0.lastindex == 2 else (cm0.group(1) if cm0 else 'cycle')
            st = subst(st, mp)
            st = re.sub(r'\bcpu\s*\.\s*', '', st)              # free functions: cpu.X -> X
            st = re.sub(r'\b(?:std|axi_interconnect|sim_ddr|mmio)::(?=\w)', '', st)   # namespace qualifiers
            # a local alias: auto &x = <path>;  (also const auto &x = <path>.f;)
            am = re.match(r'(?:const\s+)?auto\s*&\s*(\w+)\s*=\s*([^;]+);$', st)
            if am and re.match(PATH % '|'.join(ROOTS), am.group(2).strip()):
                mp[am.group(1)] = '(' + am.group(2).strip() + ')' if '(' in am.group(2) else am.group(2).strip()
                continue
            cm = re.match(r'([\w.]+?)\s*(?:\.|->)\s*(\w+)\s*\(([^;]*)\)\s*;$', st)
            if cm and norm_path(cm.group(1)) in SUBOBJ:
                key = norm_path(cm.group(1)) + '.' + cm.group(2)
                if key in SEQ_CALLS: stop[0] = True; return
                if key in INLINE:   # a FrontTop / AxiSubsystem method: inlined, its members prefixed with the object
                    cls, members = INLINE[key]
                    obj = norm_path(cm.group(1))
                    fb = src.body(cls, cm.group(2))
                    params = [re.search(r'(\w+)\s*$', p.strip()).group(1) for p in split_top(fb[0], ',') if p.strip()]
                    args = [a.strip() for a in split_top(cm.group(3), ',') if a.strip()]
                    body = strip_comments(fb[1])
                    for pat, rep in BOUND.get(cls, []): body = re.sub(pat, rep, body)
                    body = re.sub(r'(?<![\w.>])(' + members + r')\b(?!\s*\()', obj + r'.\1', body)
                    body = re.sub(r'(?<![\w.>])(read_core_q|comb_core)\s*\(\)', r'front.\1()', body)
                    body = re.sub(r'(?<![\w.>:])(connect_slave_outputs|connect_master_outputs)\s*\(', r'\1(', body)
                    mp2 = dict(mp)
                    for p, a in zip(params, args): mp2[p] = a
                    walk_body(body, key, mp2); continue
                if key in S:
                    r, w = S[key]
                    ra, wa = scan_paths(cm.group(3), ASSIGN, src, notes, key)   # arguments
                    emit(key, r | ra, w | wa, SUB.get(key)); continue
                notes.add(f'no summary for {key}()')
                emit(key, *scan_paths(st, ASSIGN, src, notes, key)); continue
            fm = re.match(r'(\w+)\s*\(([^;]*)\)\s*;$', st)
            if fm:
                fn, args = fm.group(1), [a.strip() for a in split_top(fm.group(2), ',') if a.strip()]
                if src.body('SimCpu', fn):
                    if fn.endswith('_seq'): continue        # a clock edge (register load), not a comb phase
                    walk_body(src.body('SimCpu', fn)[1], fn, dict(mp)); continue
                fb = src.body('AxiSubsystem', fn) or src.body(None, fn)
                if fb and (fb[1] in src.t[os.path.join(root, 'SimCpu.cpp')] or fb[1] in src.t[os.path.join(root, 'AxiSubsystem.cpp')]):
                    params = [re.search(r'(\w+)\s*$', p.strip()).group(1) for p in split_top(fb[0], ',') if p.strip()]
                    mp2 = dict(mp)
                    for p, a in zip(params, args):
                        mp2[p] = a.replace('*this', 'cpu')
                    walk_body(fb[1], fn, mp2); continue
            # if (cond) { ... } [else { ... }]: the condition is one phase, the branches are inlined
            im = re.match(r'if\s*\(', st)
            if im:
                i = st.index('('); d = 0
                for j in range(i, len(st)):
                    if st[j] == '(': d += 1
                    elif st[j] == ')':
                        d -= 1
                        if d == 0: break
                cond, rest = st[i + 1:j], st[j + 1:].strip()
                emit(f'{where}:if', *scan_paths(cond, ASSIGN, src, notes, where))
                if rest.startswith('{'):
                    then = Src.block(rest, 0); rest = rest[len(then):].strip()
                else:
                    k = rest.index(';') + 1; then = '{' + rest[:k] + '}'; rest = rest[k:].strip()
                branches = [then]
                if rest.startswith('else'):
                    rest = rest[4:].strip()
                    branches.append(rest if rest.startswith('{') else '{' + rest + '}')
                n_if[0] += 1; my = n_if[0]
                for bi, part in enumerate(branches):
                    cur_tag.append((my, bi)); walk_body(part, where, mp); cur_tag.pop()
                continue
            # for (...) { ... }: the header is one phase, the body is walked (aliases inside it substituted)
            fm2 = re.match(r'for\s*\(', st)
            if fm2 and '{' in st:
                i = st.index('('); d = 0
                for j in range(i, len(st)):
                    if st[j] == '(': d += 1
                    elif st[j] == ')':
                        d -= 1
                        if d == 0: break
                hdr, rest = st[i + 1:j], st[j + 1:].strip()
                if rest.startswith('{'):
                    if re.search(PATH % '|'.join(ROOTS), hdr): emit(f'{where}:for', *scan_paths(hdr, ASSIGN, src, notes, where))
                    walk_body(Src.block(rest, 0), where, dict(mp)); continue
            if not re.search(PATH % '|'.join(ROOTS), st): continue
            emit(f'{where}:glue', *scan_paths(st, ASSIGN, src, notes, where))

    walk_body(src.body('SimCpu', 'cycle')[1], 'cycle', {}, top=True)
    # number repeated labels
    seen = {}
    out = []
    for n, r, w in rows:
        seen[n] = seen.get(n, 0) + 1
        out.append((n if seen[n] == 1 else f'{n}#{seen[n]}', r, w))
    def prior_write(i, key):
        """an earlier phase (not exclusive with i) writes key: the read sees this cycle's value"""
        for j in range(i):
            if not exclusive(j, i) and overlap({key}, rows[j][2]): return True
        return False

    def exclusive(i, j):
        di = dict(tags[i]); dj = dict(tags[j])
        return any(k in dj and dj[k] != b for k, b in di.items())
    TOP_DETAIL[:] = [(n, r, w, g, t, inn) for (n, r, w), g, t, inn in zip(out, groups, tags, inners)]
    TOP_PHASES[:] = []   # (phase, R, W) per SimCpu-level phase, in the order of cycle()
    for g, (n, r, w) in zip(groups, rows):
        if not TOP_PHASES or TOP_PHASES[-1][0] != g: TOP_PHASES.append((g, set(), set()))
        TOP_PHASES[-1][1].update(r); TOP_PHASES[-1][2].update(w)
    return out, notes, overlap, exclusive, prior_write

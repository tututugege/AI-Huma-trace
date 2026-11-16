#!/usr/bin/env python3
"""The modules: one form for every module of the CPU (docs/MODULE_CONTRACT.md).

A module has `struct In` (const pointers: what it reads), `struct Out` (what it drives: its own fields, or pointers bound
to a signal its parent owns), its comb phases as methods, and their contracts above its class, in its own port names:
    //   CONTRACT <phase>  in: in.<port>[.<field>] ... | -  out: out.<port>[.<field>] ... | -
Its parent binds the ports, one line each: `<module>.in.<port> = &<signal>;` (`<module>->in...` for a unique_ptr). A
port therefore resolves to a SimCpu path (a module's out field, a top's struct, a register `<module>.q.<field>`), and
gen_top_order.py orders the phases from these paths. A read of another module's registers (q, read-only during comb)
goes through a plain pointer member, not a port.

The scan (`check()`): every `in.X` / `out.X` access of a phase (its own method and the methods of its class it calls)
is a read or a write of that port; `in.X->f` / `out.X->f` name the field f when the port points at a top's whole
interface struct (TOP_STRUCTS). A declared item covers the accesses at or below it; the declaration must cover every
scanned access, and each declared item must be scanned.

usage: modules.py           print every module's contracts as scanned (in the declared form)
       modules.py --check   exit 1 when a declaration and the code differ
"""
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from comb_domains import Src, strip_comments, split_top, enclosing_call, KEYWORDS

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ASSIGN = r'\s*(?:\[[^\]]*\]|\.\w+|\(\))*\s*(?:=(?!=)|\|=|&=|\+=|-=|\^=|<<=|>>=|\+\+|--)'
MUTATE = r'(?:\[[^\]]*\]|\.\w+)*\.(?:fill|reset|clear|assign|push_back|emplace_back|pop_front|pop_back)\s*\('

# The module instances: SimCpu path, class, the call prefix in SimCpu::cycle(), the parent's binding code.
TOPS = {
    'back': dict(cls='BackTop', bind=[('BackTop', 'init')], prefix='back.', call='back.{m}->', own='{m}->',
                 alias={'in': 'back.in', 'out': 'back.out', 'csr_status': 'back.csr_status', 'top2csr': 'back.top2csr',
                        'csr_interrupt_inject_io': 'back.csr_interrupt_inject_io'}),
    'front': dict(cls='FrontCore', bind=[('FrontCore', 'FrontCore'), ('FrontCore', 'bind')], prefix='front.core.',
                  call='front.core->{m}.', own='{m}.', alias={'in': 'front.in', 'rd': 'front.read_data', 'out': 'front.out'}),
    'cpu': dict(cls='SimCpu', bind=[('SimCpu', 'init')], prefix='', call='{m}.', own='{m}.', alias={}),
    'mem_subsystem': dict(cls='MemSubsystem', bind=[('MemSubsystem', 'init')], prefix='mem_subsystem.',
                          call='mem_subsystem.{m}.', own='{m}.', alias=None),   # alias: its pointer members (SimCpu::init)
}
MODULES = [   # (path, class), in each top's tie-break order
    ('back.pre', 'PreIduQueue'), ('back.rename', 'Ren'), ('back.idu', 'Idu'), ('back.lsu', 'RealLsu'),
    ('back.csr', 'Csr'), ('back.rob', 'Rob'), ('back.dis', 'Dispatch'), ('back.exu', 'Exu'), ('back.isu', 'Isu'),
    ('front.core.ctrl', 'FrontControl'), ('front.core.bpu_pipe', 'BpuPipe'), ('front.core.fa_port', 'FetchAddrPort'),
    ('front.core.ptab_port', 'PtabPort'), ('front.core.inst_port', 'InstPort'), ('front.core.f2b_port', 'F2bPort'),
    ('front.core.icache', 'ICacheTop'), ('front.core.checker', 'FrontChecker'),
    ('mem_subsystem.wb_', 'WriteBuffer'), ('mem_subsystem.mshr_', 'MSHR'), ('mem_subsystem.dcache_', 'RealDcache'),
    ('mem_subsystem.mem_route_block', 'MemRouteBlock'), ('mem_subsystem.ptw_block', 'MemPtwBlock'),
    ('mem_subsystem.peripheral_axi_', 'PeripheralAxi'), ('mem_subsystem.peripheral_model_', 'PeripheralModel'),
    ('b2f', 'Back2Front'), ('f2b', 'Front2Back'),
]
TOP_STRUCTS = {'back.in', 'back.out', 'front.in', 'front.out', 'front.read_data'}   # a port at one of them: per field
FIELD_PORTS = {'mem_subsystem.dcache_mshr_io_', 'mem_subsystem.dcache_wb_io_'}   # written by two phases: per field (also
#   an AXI master port of the interconnect: per channel field, req.<f> / resp.<f>)

_src = None
def src():
    global _src
    if _src is None:
        _src = Src(ROOT, ['back-end', 'front-end', 'include', 'MemSubSystem'])
        for f in os.listdir(ROOT):   # SimCpu.cpp, AxiSubsystem.cpp, ...
            if f.endswith('.cpp'): _src.t[os.path.join(ROOT, f)] = open(os.path.join(ROOT, f), errors='ignore').read()
    return _src


def class_text(cls):
    for t in src().t.values():
        m = re.search(r'((?:^//[^\n]*\n)*)(?:class|struct) ' + cls + r'\s*(?::[^{;]*)?\{', t, re.M)
        if m: return m.group(1), Src.block(t, m.end() - 1)
    sys.exit(f'class {cls}: not found')


def contracts(cls):
    """[(phase, in-items, out-items)] in header order"""
    head, _ = class_text(cls)
    out = []
    for m in re.finditer(r'CONTRACT (\w+)\s+in:(.*?)\s+out:([^\n]*)', head):
        out.append((m.group(1), set(m.group(2).split()) - {'-'}, set(m.group(3).split()) - {'-'}))
    return out


def ports(cls):
    """{'in': {port: is_array}, 'out': {port: is_pointer}}"""
    _, body = class_text(cls)
    res = {}
    for d, name in (('in', 'In'), ('out', 'Out')):
        m = re.search(r'struct ' + name + r'\s*\{', body)
        res[d] = {}
        if not m: continue
        blk = strip_comments(Src.block(body, m.end() - 1)[1:-1])
        for st in blk.split(';'):
            st = re.sub(r'\{[^{}]*\}\s*$', '', st.split('=')[0].strip()).strip()   # (a {} initializer)
            if not st or '(' in st and not re.search(r'\(\s*&', st): continue
            ptr = '*' in st
            for part in split_top(re.sub(r'^.*?(?=[\w*&(]+\s*(?:\[[^\]]*\])*\s*$)', '', st) if ',' not in st else st, ','):
                mm = re.search(r'(\w+)\s*(?:\[[^\]]*\])*\s*$', part.strip())
                if mm: res[d][mm.group(1)] = ptr or '*' in part
    return res


def norm(e):
    """a C++ access path as a SimCpu path: -> as ., an accessor x() as x, a master index kept ([MASTER_X]), others dropped"""
    e = re.sub(r'\s+', '', e.strip().lstrip('&')).replace('->', '.')
    e = re.sub(r'\[(?:\w+::)*(MASTER_\w+)\]', r'<\1>', e)
    e = re.sub(r'\[[^\]]*\]', '', e).replace('()', '')
    return re.sub(r'<(MASTER_\w+)>', r'[\1]', e)


def sim_bindings():
    """SimCpu::init's bindings of the tops' pointer members: {'<top>.<member>': path}"""
    b = src().body('SimCpu', 'init')
    return {m.group(1): norm(m.group(2)) for m in re.finditer(r'\b(mem_subsystem\.\w+)\s*=\s*&([^;]+);', strip_comments(b[1]))}


def resolve(expr, top):
    """a binding's right-hand side as a SimCpu path"""
    e = norm(expr)
    head = e.split('.')[0]
    al = TOPS[top]['alias']
    if al is None:   # the top's pointer members bound by SimCpu::init
        sb = sim_bindings()
        al = {k.split('.', 1)[1]: v for k, v in sb.items() if k.startswith(TOPS[top]['prefix'])}
    if head in al: return al[head] + e[len(head):]
    return TOPS[top]['prefix'] + e


def bindings():
    """{(path, 'in'|'out', port): SimCpu path}"""
    out = {}
    for top, t in TOPS.items():
        for cls, fn in t['bind']:
            b = src().body(cls, fn)
            if not b: sys.exit(f'{cls}::{fn}: no binding code')
            for m in re.finditer(r'\b(\w+)\s*(?:->|\.)\s*(in|out)\.(\w+)\s*=\s*([^;]+);', strip_comments(b[1])):
                out[(TOPS[top]['prefix'] + m.group(1), m.group(2), m.group(3))] = resolve(m.group(4), top)
    return out


def path_of(mod, item, bind):
    """a contract item (in.X[.f] / out.X[.f]) of module mod as a SimCpu path"""
    d, port, *rest = item.split('.')
    b = bind.get((mod, d, port))
    base = b if b else f'{mod}.{d}.{port}'
    return '.'.join([base] + rest)


def closure(cls, fn, depth=3, seen=None):
    seen = seen if seen is not None else set()
    s = src()
    b = s.body(cls, fn) or s.in_class(cls, fn)
    if not b or fn in seen: return ''
    seen.add(fn)
    out = b[1]
    if depth > 0:
        for c in set(re.findall(r'(?<![\w.>])(\w+)\s*\(', strip_comments(b[1]))):
            cb = None if c in KEYWORDS or c == fn else (s.body(cls, c) or s.in_class(cls, c))
            # (a method with a parameter named in / out works on its arguments, not on the ports)
            if cb and not re.search(r'\b(?:in|out)\s*(?:,|$)', cb[0]): out += closure(cls, c, depth - 1, seen)
    return out


def member_types(cls):
    """{member: class} of cls's data members (to resolve calls through them)"""
    _, body = class_text(cls)
    out = {}
    for m in re.finditer(r'^\s*(?:const\s+)?([\w:]+)(?:<[^>]*>)?\s*\*?\s*(\w+)\s*(?:=[^;]*)?;', strip_comments(body), re.M):
        out[m.group(2)] = m.group(1).split('::')[-1]
    return out


def callee_fields(fn, idx, cls):
    """the fields <a>.<b> a function writes through its parameter idx (when it writes only through <param>.<a>.<b> = ...)"""
    b = src().body(cls, fn) or src().body(None, fn)
    if not b: return None
    params = [x for x in split_top(b[0], ',') if x.strip()]
    if idx >= len(params): return None
    pn = re.search(r'(\w+)\s*$', params[idx].strip()).group(1)
    body = strip_comments(b[1])
    uses = re.findall(r'(?<![\w.>])' + pn + r'\b([^;]*)', body)
    out = set()
    for u in uses:
        m = re.match(r'\.(\w+)\.(\w+)\s*=(?!=)', u)
        if not m: return None   # another use: not a field-wise write
        out.add(f'{m.group(1)}.{m.group(2)}')
    return out


def scan(mod, cls, phase, bind):
    """read / write items of one phase, in port names"""
    body = strip_comments(closure(cls, phase))
    s = src()
    types = member_types(cls)
    pt = ports(cls)
    r, w = set(), set()
    def item(d, port, tail):
        bp = bind.get((mod, d, port)) or ''
        if bp in TOP_STRUCTS or bp in FIELD_PORTS or bp.startswith('axi.interconnect.'):
            m = re.match(r'\s*(?:->|\)\s*\.)\s*(\w+)(?:\s*(?:\[[^\]]*\])*\s*\.\s*(\w+))?', tail)
            if m:
                two = bp.startswith('axi.interconnect.') and m.group(2)   # an AXI master port: req.<f> / resp.<f>
                return f'{d}.{port}.{m.group(1)}' + (f'.{m.group(2)}' if two else '')
        return f'{d}.{port}'
    # local reference aliases of a port: auto &x = out.X...; (const: read only)
    aliases = {}
    for am in re.finditer(r'(const\s+)?[\w:<>]+\s*&\s*(\w+)\s*[=:]\s*\*?\s*(?<![\w.>])(in|out)\.(\w+)((?:\s*->\s*\w+)?)', body):
        k = item(am.group(3), am.group(4), am.group(5))
        aliases[am.group(2)] = (k, bool(am.group(1)))
    for m in re.finditer(r'(\*\s*)?(?<![\w.>])(in|out)\s*\.\s*(\w+)\b(?!\s*\()', body):
        star, d, port = m.group(1), m.group(2), m.group(3)
        tail = body[m.end():m.end() + 200]
        k = item(d, port, tail)
        pre = body[max(0, m.start() - 40):m.start()]
        if re.search(r'(?:const\s+)?[\w:<>]+\s*&\s*\w+\s*[=:]\s*\*?\s*$', pre): continue   # an alias declaration
        if re.match(r'(?:\s*->\s*\w+)?' + ASSIGN, tail) or re.match(r'\s*=\s*\{\s*\}', tail) or \
           re.match(r'(?:\s*->\s*\w+)?' + MUTATE, tail) or re.search(r'memset\(\s*&?\s*$', pre) or \
           (star and re.match(r'(?:\[[^\]]*\])*\s*=(?!=)', tail)):
            w.add(k); continue
        call = enclosing_call(body, m.start())
        if call:
            if call[2]: pw = s.params_written(call[0], types[call[2]]) if call[2] in types else None
            else: pw = s.params_written(call[0], cls) or s.params_written(call[0])
            if pw is not None and call[1] < len(pw) and pw[call[1]] and not re.match(r'\s*->', tail):
                fine = callee_fields(call[0], call[1], cls) if k.count('.') == 1 and (bind.get((mod, d, port)) or '').startswith('axi.interconnect.') else None
                w |= {f'{k}.{f}' for f in fine} if fine else {k}   # (a helper that writes some fields of a master port)
                if pw[call[1]] == 'rw': r.add(k)
                continue
            if call[0] in ('memset', 'memcpy', 'memmove'):   # the destination is written, the source read
                (w if call[1] == 0 else r).add(k); continue
            if call[0] == 'swap': w.add(k); r.add(k); continue
        r.add(k)
    for a, (k, const) in aliases.items():
        d0, p0 = k.split('.')[:2]
        per_field = k.count('.') == 1 and (bind.get((mod, d0, p0)) or '') in TOP_STRUCTS   # an alias of a whole top struct
        for um in re.finditer(r'(?<![\w.>])' + a + r'\b', body):
            tail = body[um.end():um.end() + 200]
            fm = re.match(r'\s*\.\s*(\w+)', tail) if per_field else None
            kk = f'{k}.{fm.group(1)}' if fm else k
            if not const and (re.match(r'(?:\[[^\]]*\]|\.\w+)*' + ASSIGN, tail) or re.match(r'\s*=\s*\{\s*\}', tail)): w.add(kk)
            elif not re.match(r'\s*=[^=]', tail): r.add(kk)
    # a read of its own output field is internal (a pointer output, bound to a shared signal, is not; a class whose
    # Out struct is declared elsewhere has pointer outputs)
    owned = pt['out'] if pt['out'] else {}
    r = {k for k in r if not (k.startswith('out.') and k.split('.')[1] in owned and not owned[k.split('.')[1]])}
    return r, w


def top_of(mod): return max((t for t, d in TOPS.items() if mod.startswith(d['prefix'])), key=lambda t: len(TOPS[t]['prefix']))


def call(mod, phase):
    """the call of a module phase in SimCpu::cycle() / in its top's own code"""
    t = TOPS[top_of(mod)]
    m = mod[len(t['prefix']):]
    return t['call'].format(m=m) + phase + '()', t['own'].format(m=m) + phase + '()'


def phases():
    """every module phase: (name <path>.<phase>, path, class, in-items, out-items)"""
    return [(f'{mod}.{ph}', mod, cls, i, o) for mod, cls in MODULES for ph, i, o in contracts(cls)]


def tb_users():
    """the module phases with side effects outside the ports: the trace bridge (tbhook::, the functional model)"""
    return {f'{mod}.{ph}' for mod, cls in MODULES for ph, _, _ in contracts(cls) if re.search(r'tbhook::|g_bridge', strip_comments(closure(cls, ph)))}


def covers(d, k): return k == d or k.startswith(d + '.')


def check():
    bind = bindings()
    errs = []
    for mod, cls in MODULES:
        for ph, di, do in contracts(cls):
            r, w = scan(mod, cls, ph, bind)
            for kind, dec, got in (('in', di, r), ('out', do, w)):
                for k in sorted(got):
                    if not any(covers(x, k) for x in dec): errs.append(f'{mod}.{ph}: {"reads" if kind == "in" else "writes"} {k}, not declared {kind}')
                for x in sorted(dec):
                    if not any(covers(x, k) or covers(k, x) for k in got): errs.append(f'{mod}.{ph}: {kind} {x} declared, not {"read" if kind == "in" else "written"}')
        if not contracts(cls): errs.append(f'{mod} ({cls}): no CONTRACT lines')
    return errs


def emit():
    bind = bindings()
    for mod, cls in MODULES:
        print(f'{mod} ({cls})')
        for ph, _, _ in contracts(cls) or [(p, 0, 0) for p in phase_names(cls)]:
            r, w = scan(mod, cls, ph, bind)
            print(f'//   CONTRACT {ph}  in: {" ".join(sorted(r)) or "-"}  out: {" ".join(sorted(w)) or "-"}')


def phase_names(cls):
    _, body = class_text(cls)
    return [m for m in re.findall(r'void (comb\w*|control|enables|finish)\(\)\s*;', body)]


if __name__ == '__main__':
    if '--check' in sys.argv:
        e = check()
        for x in e: print('  ' + x)
        if e: sys.exit(1)
        print(f'modules: {len(MODULES)} modules, contracts match the code')
    else:
        emit()

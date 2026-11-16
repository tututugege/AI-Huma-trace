"""Front-end and memory-subsystem domains of comb_graph.py (see there for the rules)."""
import re, os

KEYWORDS = {'if', 'for', 'while', 'switch', 'return', 'sizeof', 'static_cast', 'reinterpret_cast', 'const_cast', 'assert',
            'Assert', 'defined', 'decltype', 'alignof', 'int', 'unsigned', 'bool', 'uint32_t', 'uint64_t'}


class Src:
    def __init__(self, root, dirs):
        self.t = {}
        for d0 in dirs:
            for d, _, fs in os.walk(os.path.join(root, d0)):
                for f in fs:
                    if f.endswith(('.cpp', '.h')): self.t[os.path.join(d, f)] = open(os.path.join(d, f), errors='ignore').read()
        self.cache = {}

    @staticmethod
    def block(t, i):
        """t[i] == '{': the text up to the matching '}'"""
        d = 0
        for j in range(i, len(t)):
            if t[j] == '{': d += 1
            elif t[j] == '}':
                d -= 1
                if d == 0: return t[i:j + 1]
        return t[i:]

    def body(self, cls, fn):
        """body of cls::fn (cls None: a free function); also finds in-class definitions of a struct cls"""
        k = (cls, fn)
        if k in self.cache: return self.cache[k]
        res = None
        pats = []
        if cls: pats.append(re.compile(r'\b' + cls + r'::' + fn + r'\s*\(([^)]*)\)\s*(?:const\s*)?\{'))
        else:
            pats.append(re.compile(r'(?:^|\n)[\w:<>&*\s,]*?\b' + fn + r'\s*\(([^)]*)\)\s*(?:const\s*)?\{'))
            pats.append(re.compile(r'\b\w+::' + fn + r'\s*\(([^)]*)\)\s*(?:const\s*)?\{'))
        for p, t in self.t.items():
            for pat in pats:
                m = pat.search(t)
                if m: res = (m.group(1), self.block(t, m.end() - 1)); break
            if res: break
        self.cache[k] = res
        return res

    def in_class(self, cls, fn):
        for t in self.t.values():
            m = re.search(r'\b(?:struct|class)\s+' + cls + r'\b[^;{]*\{', t)
            if not m: continue
            blk = self.block(t, m.end() - 1)
            mm = re.search(r'\b' + fn + r'\s*\(([^)]*)\)\s*(?:const\s*)?\{', blk)
            if mm: return (mm.group(1), self.block(blk, mm.end() - 1))
        return None

    def params_written(self, fn, cls=None):
        """per parameter, True when it is a non-const reference / pointer (cls: a member function of that class)"""
        b = (self.body(cls, fn) or self.in_class(cls, fn)) if cls else self.body(None, fn)
        if not b: return None
        out = []
        for p in split_top(b[0], ','):
            if not p.strip() or p.strip() == 'void': continue
            if not re.search(r'[&*]', p) or re.match(r'\s*const\b', p): out.append(False); continue
            # 'w': the callee assigns / clears the whole parameter before any other use (a pure output); else 'rw'
            nm = re.search(r'(\w+)\s*(?:\[[^\]]*\])*\s*$', p)
            mode = 'rw'
            if nm and b[1]:
                first = re.search(r'(memset\s*\(\s*&?\s*)?\b' + nm.group(1) + r'\b(\s*=(?!=)\s*)?', b[1])
                if first and (first.group(1) or first.group(2)): mode = 'w'
            out.append(mode)
        return out


def split_top(s, sep):
    out, d, cur = [], 0, ''
    for ch in s:
        if ch in '([{<': d += 1
        elif ch in ')]}>': d -= 1
        if ch == sep and d == 0: out.append(cur); cur = ''
        else: cur += ch
    out.append(cur)
    return out


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
    return re.sub(r'//[^\n]*', '', t)


def enclosing_call(body, pos):
    """(callee name, argument index) when body[pos] is inside the argument list of a call, else None"""
    d, commas = 0, 0
    for i in range(pos - 1, -1, -1):
        c = body[i]
        if c in ')]': d += 1
        elif c in '([':
            if d == 0:
                if c == '[': return None
                m = re.search(r'(?:(\w+)\s*(?:\.|->)\s*)?([\w:]+)\s*$', body[:i])
                if not m: return None
                name = m.group(2).split('::')[-1]
                if name in KEYWORDS: return None
                return name, commas, m.group(1)
            d -= 1
        elif c in '{};' and d == 0: return None
        elif c == ',' and d == 0: commas += 1
    return None


def scan(body, names, ASSIGN, src, notes, owner, types=None):
    """read / write sets over the unqualified data members `names` (key: member or member.sub)"""
    r, w = set(), set()
    body = strip_comments(body)
    for nm in names:
        for m in re.finditer(r'(?<![\w.>:])' + nm + r'\b(?!\s*\()', body):
            tail = body[m.end():m.end() + 200]
            m2 = re.match(r'\s*(?:\[[^\]]*\])*\.(\w+)', tail)
            k = nm + ('.' + m2.group(1) if m2 else '')
            if re.match(r'(?:\[[^\]]*\]|\.\w+|->\w+)*' + ASSIGN, tail) or re.match(r'\s*=\s*\{\s*\}', tail):
                w.add(k); continue
            if re.match(r'(?:\[[^\]]*\]|\.\w+)*\.(?:fill|reset|clear|assign|push_back|emplace_back|pop_front|pop_back)\s*\(', tail):
                w.add(k); continue
            call = enclosing_call(body, m.start())
            if call:
                if call[2]:
                    cls = (types or {}).get(call[2])
                    pw = src.params_written(call[0], cls) if cls else None
                else:
                    pw = src.params_written(call[0])
                if pw is not None and call[1] < len(pw) and pw[call[1]]:
                    w.add(k)
                    if pw[call[1]] == 'rw': r.add(k)
                    continue
                if call[0] in ('memset', 'memcpy', 'swap') : w.add(k); continue
                if pw is None and call[0] not in ('min', 'max', 'Assert', 'printf', 'fprintf'):
                    notes.add(f'{owner}: {nm} passed to unresolved {(call[2] + ".") if call[2] else ""}{call[0]}()')
            r.add(k)
    return r, w


def closure(src, cls, fn, depth=3, seen=None):
    seen = seen if seen is not None else set()
    b = src.body(cls, fn)
    if not b or (cls, fn) in seen: return ''
    seen.add((cls, fn))
    out = b[1]
    if depth > 0:
        for c in set(re.findall(r'(?<![\w.>])(\w+)\s*\(', b[1])):
            if c != fn and src.body(cls, c): out += closure(src, cls, c, depth - 1, seen)
    return out


def struct_members(text, name):
    m = re.search(r'struct ' + name + r'\s*\{', text) or re.search(r'class ' + name + r'\s*[:{]', text)
    blk = Src.block(text, text.index('{', m.end() - 1))
    blk = strip_comments(blk[1:-1])
    # drop member function bodies / nested blocks
    flat, d = '', 0
    for ch in blk:
        if ch == '{': d += 1; continue
        if ch == '}': d -= 1; continue
        if d == 0: flat += ch
    out = []
    for st in flat.split(';'):
        st = st.strip()
        if not st or '(' in st.split('=')[0] or st.startswith(('using', 'friend', 'typedef', 'static', 'public', 'private', 'enum')): continue
        st = re.sub(r'^(?:public|private|protected)\s*:\s*', '', st)
        for part in split_top(st.split('=')[0], ','):
            mm = re.search(r'(\w+)\s*(?:\[[^\]]*\])*\s*$', part.strip())
            if mm: out.append(mm.group(1))
    return [x for x in dict.fromkeys(out) if x not in ('const', 'int', 'bool')]


def hazards_of(rows, overlap, known):
    hz = []
    for i, (n, r, w) in enumerate(rows):
        for j in range(i + 1, len(rows)):
            for f in overlap(r, rows[j][2]):
                if (n, f, rows[j][0]) not in known: hz.append((n, f, rows[j][0]))
    return hz


# ---------------------------------------------------------------------------------------------------------------------
# front end: FrontComb phases in front_comb_calc() order
# ---------------------------------------------------------------------------------------------------------------------
FRONT_KNOWN = {
}


def front_domain(root, ASSIGN, overlap):
    """FrontComb is constructed for every front_comb_calc() call and `req` is cleared before it: a read of one of its
    members before a later phase writes it sees the initial value (a staged input, reported, not a hazard).  Only the
    file-scope state of front_top.cpp lives across cycles: reading it before a later phase writes it is a hazard."""
    src = Src(root, ['front-end', 'include'])
    ft = src.t[os.path.join(root, 'front-end', 'front_top.cpp')]
    names = struct_members(ft, 'FrontComb')
    decls = list(re.finditer(r'^static\s+(?!const\b)([\w:<>, ]+?)\s\*?(\w+)\s*(?:=[^;]*)?;', strip_comments(ft), re.M))
    persistent = [m.group(2) for m in decls]
    types = {m.group(2): m.group(1).split('::')[-1].strip() for m in decls}
    calc = src.body(None, 'front_comb_calc')[1]
    phases = re.findall(r'\bc\.(\w+)\(', calc)
    rows, notes = [], set()
    for p in phases:
        r, w = scan(closure(src, 'FrontComb', p), names + persistent, ASSIGN, src, notes, p, types)
        rows.append((p, r, w))
    FRONT_ROWS[:] = rows
    hz, staged = [], []
    for h in hazards_of(rows, overlap, FRONT_KNOWN):
        (hz if h[1].split('.')[0] in persistent else staged).append(h)
    notes = sorted(notes) + [f'persistent state scanned: {" ".join(persistent)}',
                             f'{len(staged)} staged reads (a member read before a later phase writes it: the initial value)']
    return rows, hz, notes


# ---------------------------------------------------------------------------------------------------------------------
# memory subsystem: the statements of MemSubsystem::comb_outputs() then comb_inputs()
# ---------------------------------------------------------------------------------------------------------------------
MEM_GROUPS = {}   # entry function -> its rows (used by the top domain)
MEM_BLOCKS = {}   # block phase label -> (class, reads, writes in the block's own port names)
MEM_BIND = {}     # (block object, 'in' | 'out', field) -> MemSubsystem signal (MemSubsystem::init)
FRONT_ROWS = []
MEM_EXPAND = ('dcache_stage2', 'dcache_stage1', 'ptw_request', 'ptw_feedback')   # walked statement by statement, one row each
MEM_KNOWN = {
}


def statements(body):
    """top-level statements of a { } body ({ } blocks and if/for with their bodies are one statement)"""
    t = strip_comments(body)[1:-1]
    out, cur, d = [], '', 0
    i = 0
    while i < len(t):
        ch = t[i]; cur += ch
        if ch in '({': d += 1
        elif ch in ')}':
            d -= 1
            if ch == '}' and d == 0:
                rest = t[i + 1:].lstrip()
                if not rest.startswith(('else', ';')): out.append(cur); cur = ''
        elif ch == ';' and d == 0: out.append(cur); cur = ''
        i += 1
    return [s.strip() for s in out if s.strip()]


def mem_domain(root, ASSIGN, overlap):
    src = Src(root, ['MemSubSystem', 'back-end/include', 'include'])
    hdr = src.t[os.path.join(root, 'MemSubSystem', 'include', 'MemSubsystem.h')]
    names = struct_members(hdr, 'MemSubsystem')
    # member blocks and their classes
    blocks = dict((m.group(2), m.group(1)) for m in re.finditer(r'^\s*(RealDcache|MSHR|WriteBuffer|PeripheralAxi|PeripheralModel|MemPtwBlock|MemRouteBlock)\s+(\w+)\s*;', hdr, re.M))
    sig_names = [n for n in names if n not in blocks]
    # bindings blk.in.F / blk.out.F = &sig / sig / &sig->x
    init = strip_comments(src.body('MemSubsystem', 'init')[1])
    bind = {}
    for m in re.finditer(r'\b(\w+)\.(in|out)\.(\w+)(?:\[\w+\])?\s*=\s*&?(\w+)', init):
        if m.group(1) in blocks: bind[(m.group(1), m.group(2), m.group(3))] = m.group(4)
    rows, notes = [], set()
    MEM_BIND.clear(); MEM_BIND.update(bind); MEM_BLOCKS.clear()

    def block_phase(blk, meth, label):
        cls = blocks[blk]
        b = strip_comments(closure(src, cls, meth))
        r, w = set(), set()
        rr, rw = set(), set()   # the block's own port names (in.F / out.F)
        for m in re.finditer(r'(\*\s*)?(?<![\w.>])(in|out)\s*\.\s*(\w+)', b):
            star, d, fld = m.group(1), m.group(2), m.group(3)
            sig = bind.get((blk, d, fld))
            k = sig if sig else f'{blk}.{d}.{fld}'
            tail = b[m.end():m.end() + 200]
            sub = re.match(r'\s*->\s*(\w+)(?:\s*(?:\[[^\]]*\])*\s*\.\s*(\w+))?', tail)   # a pointer to a whole struct: two levels below it
            if sig and sub: k = sig + '.' + sub.group(1) + ('.' + sub.group(2) if sub.group(2) else '')
            head = b[max(0, m.start() - 12):m.start()]
            if re.match(r'(?:\[[^\]]*\]|\.\w+|->\w+)*' + ASSIGN, tail) or re.search(r'memset\(\s*&?\s*$', head) or \
               (star and re.match(r'(?:\[[^\]]*\])*\s*=(?!=)', tail)):
                w.add(k); rw.add(f'{d}.{fld}')
            else:
                r.add(k); rr.add(f'{d}.{fld}')
        rows.append((label, r, w))
        MEM_BLOCKS[label] = (cls, rr, rw)

    def walk(fn):
        glue = []
        def flush_glue():
            if glue:
                r, w = scan('{' + ';'.join(glue) + ';}', sig_names + list(blocks), ASSIGN, src, notes, fn)
                # a direct access of a block member from glue: key blk.in.F / blk.out.F
                rows.append((f'{fn}:glue{len([x for x in rows if x[0].startswith(fn + ":glue")])}', r, w))
                glue.clear()
        for st in statements(src.body('MemSubsystem', fn)[1]):
            m = re.match(r'(\w+)\.(\w+)\s*\(\s*\)\s*;?$', st) or re.match(r'(\w+)\.(\w+)\s*\(', st)
            m_self = re.match(r'(\w+)\s*\(([^;]*)\)\s*;?$', st)
            if m and m.group(1) in blocks and st.startswith(m.group(1) + '.' + m.group(2)):
                flush_glue(); block_phase(m.group(1), m.group(2), f'{m.group(1)}.{m.group(2)}')
            elif m_self and m_self.group(1) in MEM_EXPAND:
                flush_glue()
                n0 = len(rows)
                walk(m_self.group(1))
                sub = rows[n0:]
                del rows[n0:]
                rows.append((m_self.group(1), set().union(*[x[1] for x in sub]) if sub else set(),
                             set().union(*[x[2] for x in sub]) if sub else set()))
            elif m_self and src.body('MemSubsystem', m_self.group(1)):
                flush_glue()
                r, w = scan(closure(src, 'MemSubsystem', m_self.group(1)), sig_names + list(blocks), ASSIGN, src, notes, m_self.group(1))
                rows.append((m_self.group(1), r, w))
            elif m_self and src.params_written(m_self.group(1)) is not None:
                flush_glue()
                r, w = scan('{' + st + '}', sig_names, ASSIGN, src, notes, m_self.group(1))
                rows.append((m_self.group(1), r, w))
            else:
                glue.append(st)
        flush_glue()
    walk('comb_outputs')
    n_out = len(rows)
    walk('comb_inputs')
    MEM_GROUPS.clear()
    MEM_GROUPS.update({'comb_outputs': rows[:n_out], 'comb_inputs': rows[n_out:]})
    return rows, hazards_of(rows, overlap, MEM_KNOWN), sorted(notes)

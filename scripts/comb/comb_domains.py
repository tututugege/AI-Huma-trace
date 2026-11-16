"""Source scanning helpers of the comb tools (modules.py, comb_top.py)."""
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
            pats.append(re.compile(r'(?:^|\n)(?:\[\[\w+\]\]\s*)?[\w:<>&*\s,]*?\b' + fn + r'\s*\(([^)]*)\)\s*(?:const\s*)?\{'))
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
        if not st or ('(' in st.split('=')[0] and not re.search(r'\(\s*&\s*\w+\s*\)', st.split('=')[0])) or st.startswith(('using', 'friend', 'typedef', 'static', 'public', 'private', 'enum')): continue
        st = re.sub(r'^(?:public|private|protected)\s*:\s*', '', st)
        for part in split_top(st.split('=')[0], ','):
            mm = re.search(r'\(\s*&\s*(\w+)\s*\)\s*(?:\[[^\]]*\])*\s*$', part.strip()) or \
                 re.search(r'(\w+)\s*(?:\[[^\]]*\])*\s*$', part.strip())   # (a reference to an array: (&name)[N])
            if mm: out.append(mm.group(1))
    return [x for x in dict.fromkeys(out) if x not in ('const', 'int', 'bool')]


def hazards_of(rows, overlap, known):
    hz = []
    for i, (n, r, w) in enumerate(rows):
        for j in range(i + 1, len(rows)):
            for f in overlap(r, rows[j][2]):
                if (n, f, rows[j][0]) not in known: hz.append((n, f, rows[j][0]))
    return hz


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

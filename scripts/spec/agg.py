import sys, glob, math
spec, out, cov, benches = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4:]
ipcs = []
for b in benches:
    pts = sorted(((float(l.split()[0]), int(l.split()[1])) for l in open(f'{spec}/sp/{b}.weights') if l.strip()), reverse=True)
    acc = 0; sel = []
    for w, sp in pts:
        sel.append((w, sp)); acc += w
        if acc >= cov - 1e-9: break
    W = sum(w for w, _ in sel); cpi = 0; n = 0; mm = 0
    for w, sp in sel:
        f = glob.glob(f'{out}/{b}/ckpt_sp{sp}_target*.ok')
        if not f: print('MISSING', b, sp); continue
        _, i, c, *m = open(f[0]).read().split(); n += 1
        mm += any(not x.endswith('=0') for x in m); cpi += w / W * int(c) / int(i)
    ipc = 1 / cpi; ipcs.append(ipc)
    print(f'{b:16s} points={n:3d}/{len(sel):3d} weighted IPC={ipc:.3f} mismatch-points={mm}')
print('geomean IPC = %.3f' % math.prod(ipcs) ** (1 / len(ipcs)))

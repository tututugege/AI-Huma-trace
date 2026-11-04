import sys, glob
spec, cov, benches = sys.argv[1], float(sys.argv[2]), sys.argv[3:]
for b in benches:
    pts = sorted(((float(l.split()[0]), int(l.split()[1])) for l in open(f'{spec}/sp/{b}.weights') if l.strip()), reverse=True)
    acc = 0
    for w, sp in pts:
        print(glob.glob(f'{spec}/ckpt/{b}/ckpt_sp{sp}_target*.gz')[0]); acc += w
        if acc >= cov - 1e-9: break

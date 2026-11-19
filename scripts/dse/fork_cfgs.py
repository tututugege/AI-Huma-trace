#!/usr/bin/env python3
"""fork_cfgs.py: the EXP-047 sweep (one parameter at a time around the default) as a DSE_CONFIGS file on stdout.
Every line spells out all caps, because a key left out means the binary's compiled size (sim64_dsebig: the maxima)."""
D = dict(rob=128, prf=128, fprf=96, br=32, ldq=64, stq=64, mshr=8, iq_int=32, iq_mem=64, iq_std=16, iq_br=32, iq_fp=32)
C = [("base", {})]
C += [(f"rob{v}", dict(rob=v)) for v in (64, 96)]
C += [(f"prf{v}", dict(prf=v)) for v in (96, 112, 160, 192)]
C += [(f"rob{v}prf{v}", dict(rob=v, prf=v)) for v in (160, 192, 256)]
C += [(f"ldq{v}", dict(ldq=v)) for v in (16, 24, 32, 48)]
C += [(f"stq{v}", dict(stq=v)) for v in (16, 24, 32, 48)]
C += [(f"mshr{v}", dict(mshr=v)) for v in (4, 6, 12, 16)]
C += [(f"iqint{v}", dict(iq_int=v)) for v in (16, 24, 48, 64)]
C += [(f"iqmem{v}", dict(iq_mem=v)) for v in (32, 48, 96)]
C += [(f"iqstd{v}", dict(iq_std=v)) for v in (8, 32)]
C += [(f"iqbr{v}", dict(iq_br=v)) for v in (16, 48)]
C += [(f"br{v}", dict(br=v)) for v in (16, 24, 48, 64)]
for name, o in C:
    c = dict(D, **o)
    print(name, " ".join(f"{k}={v}" for k, v in c.items()))

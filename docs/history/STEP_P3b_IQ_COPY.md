# P3b: IssueQueue / PreIduQueue double-buffer copies (exact)

> Historical record of the pre-P3 migration (2026-11-04); may be outdated, see [README.md](README.md).

After P3a the top of stack was still memmove (~20%): IssueQueue vector assigns, PreIduQueue::comb_begin/seq.
Pattern: committed copy `x` and working copy `x_1`; comb_begin did `x_1 = x`, seq did `x = x_1`, although the two are
equal at comb_begin (seq just copied them) and a cycle usually changes a few slots.

Change (patches/0002-iq-preidu-dirty-copy.patch):
- IssueQueue: no copy in comb_begin; writes to entry_1 set `entry_dirty`; writes to the wake matrices record the
  touched preg row (dirty row list, all-dirty on flush_all); seq copies only dirty parts.
- PreIduQueue: no copy in comb_begin (scalars still copied); `ibuf_dirty` / `ftq_dirty` set at the write sites;
  seq copies only dirty arrays.
- Invariant "committed == working at comb_begin" is checked by -DIQ_SELFCHECK builds (abort on difference).

Checks:
- IQ_SELFCHECK build: 56/56 cycle counts equal, logs byte-identical, no abort.
- Normal build: 56/56 cycle counts equal, logs byte-identical.
- Negative control (-DIQ_NEG_CONTROL, enqueue does not mark dirty, with IQ_SELFCHECK): aborts on prime
  ("committed != working at seq").
- Limit: the invariant is proven only on the paths the 56 tests execute.

Speed (timer-mmio, 206,120 cycles; CPU time = user+sys, wall is noisy on this machine):
- baseline: 202.3 G instr, 17.3 s CPU -> 11.9 k cycles/s
- after P3a: 97 G, ~6.2 s CPU -> ~33 k cycles/s
- after P3b: 92.1 G, ~6.0 s CPU -> ~34 k cycles/s (2.9x vs baseline; includes ~0.3 s startup)
Remaining top of stack (3.0k samples): memmove 376, memset 222, copy_regs_live, malloc/free (~200: vectors built
per comb call), icache_compact_kernel, IssueQueue::schedule, LLC comb paths.

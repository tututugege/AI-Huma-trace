# P3a: LLC register state copy (exact)

Finding (P0, docs/P0_PROFILE.md): ~68% of host time was memmove in AXI_LLC::comb / AXI_Interconnect::seq.
Cause (measured): the AXI subsystem ticks CONFIG_AXI_SUBSYSTEM_FREQ_DIV=16 times per core cycle; per core cycle
AXI_LLC::comb ran 32x and seq 16x (counted: 121,760 / 60,880 over 3,805 cycles of timer-interrupt-mask).
Each did a full copy of AXI_LLC_Regs_t (60,376 bytes).

Change (patches/0001-llc-live-slot-copy.patch, in axi-interconnect-kit):
- Regs members that hold line payloads (read_resp data/queue data, victim write-back queue, write_ctx, write_q,
  victim_wb data/strobe, mshr) moved to the end of the struct; `copy_regs_live` memcpy's the scalar prefix and copies
  a bulk slot only when it is live (valid flag or inside head/count of its queue).
- Dead slots still get their `valid` flag restored: comb() runs several times per tick and an earlier pass may have
  allocated a slot in dst. (First attempt without this deadlocked: "卡死了".)

Checks:
- Gate: 56/56 cycle counts equal to baseline, and the full log of every test is byte-identical.
- Poison build (-DLLC_POISON_DEAD: dead slots of dst filled with 0xA5 on every copy): 56/56 identical -> no gate test
  reads a dead slot's payload.
- Negative control (-DLLC_NEG_CONTROL: live mshr not copied): timer-interrupt-mask and prime deadlock -> the gate and the
  poison build are sensitive.
- Limits: the poison proof covers only what the 56 tests execute.

Result (timer-mmio, 206,120 cycles, two runs):
- host instructions retired 202.3 G -> 96.8 / 97.3 G (2.1x)
- wall 17.3 s -> 6.6 / 7.7 s (2.2-2.6x)
- sum wall of the 56 tests 43.3 s -> 27.0 s (tiny tests are dominated by ~0.35 s startup, 1 GB memory init)
- top of stack now: memmove 20% (IssueQueue vector assign, PreIduQueue::comb_begin/seq, front_comb_calc), memset 5%.

# P3a:LLC 寄存器状态拷贝(exact)

> 历史记录:P3 迁移之前的状态(2026-11-04),可能已过时,现状见 [README.md](README.md)。

发现(P0,docs/history/P0_PROFILE.md):约 68% 的 host 时间是 AXI_LLC::comb / AXI_Interconnect::seq 中的 memmove。
原因(实测):AXI 子系统每个 core cycle tick CONFIG_AXI_SUBSYSTEM_FREQ_DIV=16 次;每个 core cycle 中
AXI_LLC::comb 运行 32 次,seq 运行 16 次(计数:timer-interrupt-mask 的 3,805 个 cycle 中为 121,760 / 60,880)。
每次都完整拷贝一份 AXI_LLC_Regs_t(60,376 字节)。

改动(patches/0001-llc-live-slot-copy.patch,在 axi-interconnect-kit 中):
- 持有 line payload 的 Regs 成员(read_resp data/queue data、victim write-back 队列、write_ctx、write_q、
  victim_wb data/strobe、mshr)移到 struct 末尾;`copy_regs_live` 对标量前缀做 memcpy,并且只在
  bulk slot 为 live(有 valid 标志,或位于其队列的 head/count 之内)时才拷贝它。
- 死 slot 仍然要恢复它的 `valid` 标志:comb() 在一个 tick 内会运行多次,较早的一遍可能已经
  在 dst 中分配了一个 slot。(没有这一点的第一次尝试发生了死锁:"卡死了"。)

检查:
- Gate:56/56 的 cycle 数与基线相等,且每个测试的完整 log 逐字节相同。
- 毒化构建(-DLLC_POISON_DEAD:每次拷贝时 dst 的死 slot 都填 0xA5):56/56 相同 -> 没有 gate 测试
  读取死 slot 的 payload。
- 负对照(-DLLC_NEG_CONTROL:不拷贝 live mshr):timer-interrupt-mask 和 prime 死锁 -> gate 和
  毒化构建都是敏感的。
- 限制:毒化证明只覆盖这 56 个测试所执行的内容。

结果(timer-mmio,206,120 cycles,两次运行):
- host 退休指令 202.3 G -> 96.8 / 97.3 G(2.1x)
- wall 17.3 s -> 6.6 / 7.7 s(2.2-2.6x)
- 56 个测试的 wall 总和 43.3 s -> 27.0 s(小测试主要被约 0.35 s 的启动和 1 GB 内存初始化主导)
- 现在的栈顶:memmove 20%(IssueQueue vector 赋值、PreIduQueue::comb_begin/seq、front_comb_calc),memset 5%。

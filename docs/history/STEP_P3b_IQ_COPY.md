# P3b:IssueQueue / PreIduQueue 双缓冲拷贝(exact)

> 历史记录:P3 迁移之前的状态(2026-11-04),可能已过时,现状见 [README.md](README.md)。

P3a 之后栈顶仍然是 memmove(约 20%):IssueQueue vector 赋值、PreIduQueue::comb_begin/seq。
模式:已提交的拷贝 `x` 和工作拷贝 `x_1`;comb_begin 做 `x_1 = x`,seq 做 `x = x_1`,尽管这两者在
comb_begin 时是相等的(seq 刚刚拷贝过它们),而且一个 cycle 通常只改变少数几个 slot。

改动(patches/0002-iq-preidu-dirty-copy.patch):
- IssueQueue:comb_begin 中不拷贝;对 entry_1 的写入设置 `entry_dirty`;对 wake 矩阵的写入记录
  被触及的 preg 行(dirty 行列表,flush_all 时全部 dirty);seq 只拷贝 dirty 的部分。
- PreIduQueue:comb_begin 中不拷贝(标量仍然拷贝);在写入位置设置 `ibuf_dirty` / `ftq_dirty`;
  seq 只拷贝 dirty 的数组。
- "comb_begin 时已提交 == 工作"这一不变量由 -DIQ_SELFCHECK 构建检查(不同则 abort)。

检查:
- IQ_SELFCHECK 构建:56/56 cycle 数相等,log 逐字节相同,没有 abort。
- 普通构建:56/56 cycle 数相等,log 逐字节相同。
- 负对照(-DIQ_NEG_CONTROL,enqueue 不标记 dirty,配合 IQ_SELFCHECK):在 prime 上 abort
  ("committed != working at seq")。
- 限制:该不变量只在 56 个测试所执行的路径上得到证明。

速度(timer-mmio,206,120 cycles;CPU 时间 = user+sys,这台机器上 wall 噪声较大):
- 基线:202.3 G 指令,17.3 s CPU -> 11.9 k cycles/s
- P3a 之后:97 G,约 6.2 s CPU -> 约 33 k cycles/s
- P3b 之后:92.1 G,约 6.0 s CPU -> 约 34 k cycles/s(相对基线 2.9x;包含约 0.3 s 启动)
剩余的栈顶(3.0k 个样本):memmove 376,memset 222,copy_regs_live,malloc/free(约 200:每次 comb 调用构建的 vector),
icache_compact_kernel,IssueQueue::schedule,LLC comb 路径。

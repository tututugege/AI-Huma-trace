# P2a:由 fact 驱动模型中与时序相关的决策(TB_INJECT=1)

> 历史记录:P3 迁移之前的状态(2026-11-04),可能已过时,现状见 [README.md](README.md)。

模型自己的数值计算仍然存在;只要存在 fact,下列位置就不再读取它:
| 位置 | fact | 56 个测试上的 n |
|---|---|---|
| BruUnit 分支方向 | br_taken | 61,200 |
| BruUnit jal/jalr 目标 | next_pc | 42,641 |
| AguUnit 地址(load/STA) | vaddr | 107,846 |
| SduUnit store data(非原子) | rs2_value | 46,030 |
| Exu CSR 写数据 | csr_wdata | 323 |
与模型自己计算值不同的 override:0(fact == 计算结果);fact miss:0。
模型的 latency 不依赖数据(MUL/DIV 固定 latency),所以不需要 latency fact。

发现必须在 bridge 中补充的内容:
- Boot ROM:core 从 0x0..0x100000 的 word-backed MMIO 启动;bridge 的 core 由 pmem 初始化,并且可能从那里取指。
- 未执行指令(被 block 的取指、被撤销的 trap)的控制流根据 bridge 寄存器求值
  (分支比较、jal/jalr 目标),因为错误路径上分支的 mispredict 同样会驱动恢复。

检查:
1. fact 驱动的 gate(TB_INJECT=1):56/56 的 cycle 数与基线相等。
2. 正对照(扰动 fact,--no-diff):每 50 个翻转一个分支 fact:prime 11,760 -> 2,923,
   quick-sort 2,779 -> 2,103;地址扰动(每 20 个):quick-sort 2,779 -> 2,784(prime 不变:只有 10 个 load)。
3. 毒化对照(TB_POISON=1 TB_INJECT=1 --no-diff):模型的 ALU/MUL/DIV/FP 结果和非 MMIO load 数据
   都与 0xA5C3A5C3 异或。56 个测试中有 45 个 cycle 数仍与基线相等(results/poison_all.txt),
   即模型的时序不读取这些值。另外 11 个(csr-alias-boundary、fence-i-self-modify、
   illegal-instruction-trap、interrupt-vectored、software-interrupt-vectored、supervisor-external-interrupt-vectored、
   supervisor-illegal-delegation、supervisor-trap-delegation、timer-interrupt-mask、trap-boundary、trap-precise-store)
   挂死:flush 之后 bridge 的状态取自 difftest reference,而 reference 在 --no-diff 下不会步进。

## 下一步(P2b)
在 bridge 内部做 commit 时执行(trap、mret/sret、CSR 副作用、带有来自模型的 cause 的中断),
使 difftest reference 不再是状态来源;然后这 11 个测试加入毒化对照。
之后 P3 就可以删除数值 datapath。

# P2b:bridge 内部的 commit 时事件(不再以 difftest reference 作为状态来源)

- 异常(page fault、illegal、ecall、mret/sret 在 core 里都是 `exception()`):当一条指令在
  推测执行时 trap,bridge 保存它到达的状态(`trap_after`),再撤销它以便流继续,
  并在该指令的 commit-flush(`commit_flush`)时丢弃所有更年轻的指令并采用 `trap_after`。
  trap 是指令 + 架构状态的确定性函数,所以它是逐指令的 fact。
- 中断:异步。由模型决定何时发生(timer compare / pending 位,是 cycle 的函数);在 commit
  边界,tap(SimContext::run_difftest_interrupt)传入 cause、目标 privilege 和 pending 位;
  bridge 丢弃所有 in-flight 条目并调用 refcpu_take_forced_interrupt。发生中断的 cycle 不会
  commit 队首指令(interrupt_nop),所以那里跳过 commit tap。
- TB_RESYNC_REF=1 选择旧的 P1b 行为(状态来自 difftest reference),用于对比。

结果(56 个测试):check 模式下 rd/branch/address/target 为 0 个不一致(169,538 / 61,200 / 103,293 / 103,841),
cycle 相等;普通 gate 和 TB_INJECT gate 56/56 cycle 相等;毒化对照(TB_POISON,--no-diff)56/56 cycle
相等(P2a 中是 45/56),8 个 job 下耗时 18 s。
负对照(TB_NEG_NO_TRAP_STATE:不采用 trap_after):不一致计数器仍为 0,因为 handler 中读到的错误 CSR
值会被模型自动 patch;信号在于 csr_read patch 次数
(illegal-instruction-trap 0 -> 3,trap-boundary 0 -> 8,supervisor-trap-delegation 0 -> 7)。
由此得出的 gate 规则:除 mip/time 读之外,csr_read patch 必须为 0(它们是唯一合理地无法预知的 CSR;
timer-interrupt-mask 中有 2 次)。

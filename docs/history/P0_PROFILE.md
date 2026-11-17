# P0:基线的 host 时间花在哪里(实测)

> 历史记录:P3 迁移之前的状态(2026-11-04),可能已过时,现状见 [README.md](README.md)。

二进制:upstream 2f31e8d,默认 profile,-O3,macOS arm64,用 `sample`(1 ms)采样,带 get-task-allow 签名的副本。
workload:timer-mmio(206,120 cycles,17.3 s;启动 3 s 后采样 10 s)和 matmul_fpu(42,253 cycles;2 s)。
原始采样:results/baseline/p0_*.sample.txt。

## 结论
- timer-mmio:206,120 cycles 共退休 202.3 G 条 host 指令 = 每个模拟 cycle 约 981k 条指令;RSS 4.8 GB。
- 启动约 0.3 s(wfi-boundary:151 cycles,0.34 s),所以开销在每个 cycle 上,而不是启动。

## 栈顶分布(timer-mmio,约 8.2k 个样本)
| 占比 | 函数 |
|---|---|
| ~68% | `_platform_memmove`(5567 个样本) |
| 2.6% | `_platform_memset` |
| 各 ~1% | AXI_LLC::comb、prepare_llc_inputs、llc_path_quiescent、icache_compact_kernel、IssueQueue::comb_begin ... |

memmove 按调用者:AXI_LLC::comb() 3498,AXI_Interconnect::seq() 1564(合计占 memmove 的 91%);
其余是 IssueQueue 的 vector 赋值(109)、PreIduQueue(154)、其他(各 <50)。

## 按子系统的 inclusive 时间(SimCpu::cycle 的子调用,timer-mmio)
| 子系统 | 样本数 | 占比 |
|---|---|---|
| 共享 AXI interconnect + LLC + DDR(comb_outputs、LLC::comb、seq、llc_seq、comb_inputs 等) | ~6,400 | ~78% |
| backend(BackTop comb+seq) | ~970 | ~12% |
| frontend(front comb+seq) | ~615 | ~7.5% |
| dcache 快速路径 | ~220 | ~2.7% |

matmul_fpu 的分布相同(栈顶 memmove 占 71%;调用者相同)。FPU/softfloat 函数:0 个样本。

## 原因
`AXI_LLC::comb()` 以 `io.ext_out = {}; io.table_out = {}; io.reg_write = io.regs;` 开头,而 `AXI_Interconnect::seq()`
会把已寄存的状态拷贝回去。AXI_LLC_Regs_t 含有宽数据队列数组(read-response 队列、write 队列、
victim write-back 队列),table 请求还带有字节 vector,所以每个 cycle 都要拷贝/清零数十 KB(未精确测量),
而这些状态很少变化。与 RSD 的"tick 中数组旧值 memcpy"是同一种模式。

## 对计划的影响
1. 删除数值逻辑(FPU、ALU、PRF 数据)在这里几乎不减少 host 时间(~0%):P3 的第 1-2 步不是提速的杠杆。
2. 第一个提速步骤是 LLC/AXI 状态:停止整体状态拷贝(原地更新 / 只拷 dirty 部分),去掉数据 payload
   (时序只需要 tag、valid、repl、MSHR、队列占用和地址,不需要 line 数据)。exactness 由 gate 保证。
3. 之后剩下的约 25%:backend 12%、frontend 7.5%、IssueQueue/PreIduQueue 的 vector 拷贝。
4. 注意事项:只有两个 workload,每个只有一个采样窗口;采样会把 inline 的代码算到调用者头上;访存更少的
   workload 上 LLC 占比可能不同。每一步之后都要用退休指令数重新测量。

## ICache S1-S3 + AXI idle skip 之后的 profile(timer-mmio,`sample` 2 s,1,178 个样本;一次运行)

Inclusive,SimCpu::cycle 的子调用:front_comb_before_back 38.4%,BackTop::comb 38.2%,BackTop::seq 15.3%,AXI comb_outputs 7.0%(其中 LLC::comb 5.6%),AXI seq 3.1%,comb_inputs 2.2%,front_seq_write 3.3%。
栈顶:`_platform_memmove` 303(26%),`_platform_memset` 116(10%),copy_regs_live,front_comb_calc 52,IssueQueue::comb_begin 40。

前端(front_comb_calc 之下 415 个样本):BPU_TOP::bpu_comb_calc 245(占总量 20.8%;bpu_core_comb_calc 146,memset 30,bpu_submodule_seq_read 27,memmove 18),bpu_seq_read 32,icache_comb_calc 20(1.7%),PTAB/FIFO 约 20。BPU 现在是最大的单个模块;ICache 很小。
后端:Isu::comb_issue 89(7.6%),TlbMmu::translate_lsu_ports 50(4.2%),Isu::comb_begin 40,Exu::comb_begin/comb_exec/fu2exu_collect 32/30/24,Rob 和 Dispatch 各约 10。
memmove 调用者:PreIduQueue::seq 88(占总量 7.5%),IssueQueue vector `assign` 41,front_comb_calc 31,SimCpu::cycle 24,TAGE/BTB/BPU 拷贝约 40。memset 调用者:BPU(bpu_comb_calc 30,bpu_seq_read 27,comb_bpu 23,core 14)。

要点:(1) 整个 struct 的拷贝/清零(memmove+memset 约占栈顶 36%)再次成为主要开销,现在分散在 BPU payload struct、PreIduQueue::seq 和 IssueQueue 赋值上;(2) BPU inclusive 约 30%;(3) AXI+LLC 从约 78% 降到约 15%。

## 前端重写若干轮之后的 profile(timer-mmio,2 s 采样,1,313 个样本;一次运行)

Inclusive(SimCpu::cycle 的子调用):BackTop::comb 50.1%,front_comb_before_back 30.5%,AXI comb_outputs 7.3%(LLC::comb 4.3%),BackTop::seq 4.6%,front_seq_write 3.7%,AXI comb_inputs/seq 2.7/2.6%。
后端:Isu::comb_issue 114(IssueQueue::comb_issue,含 schedule),TlbMmu::translate_lsu_ports 73(全部在 bit-vector TLB kernel 的往返中),Isu::comb_begin 71(IssueQueue::comb_begin),Exu::comb_exec/begin/collect 37/33/25,Dispatch::comb_dispatch 25,Isu::comb_enq 24,Rob::seq 21。
前端:BPU core 151(11.5%),bpu_submodule_seq_read 35,icache 28,bpu_seq_read 27(memset)。
栈顶:memmove 173(13%),memset 93(7%)。

根据此 profile 已完成的工作(patch 0026):IssueQueue 调度每个 IQ 每 cycle 每次调用使用三个堆上 vector,并在 comb_begin 中把整个 enqueue/grant payload 清零 -> 改为 scratch buffer + 只清 `valid`(-3.3%);TLB 顶层:当没有有效 port 需要翻译(M-mode 或 satp bare)且没有 flush/cancel 时,跳过 bit-vector kernel(encode -> generator -> decode),直接返回恒等结果(-6.4%)。该快速路径用 TLB_FAST_VERIFY=1 与 kernel 做了对比:gate 上共 456,429 次检查,0 次不一致(逐字段比较)。注意:56 个测试的 gate 没有分页测试,所以开启翻译的路径是未改动的代码,并没有被重新验证。
合计:timer-mmio user 时间 2.271 -> 2.063 s(-9%),约 100k cycles/s。

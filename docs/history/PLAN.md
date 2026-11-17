# 蒸馏计划:execution-driven 模型 -> trace-driven 时序模型

> 历史记录:P3 迁移之前的状态(2026-11-04),可能已过时,现状见 [README.md](README.md)。

目标:一个独立的时序模型,在 gate 集合上与 execution-driven 基线 cycle 完全一致,且快得多。
之后:生成的 RTL + 本模型,所以 event/fact 接口不能依赖旧模型的内部实现。

## Gate
- 56 个测试(scripts/build_tests.sh,LLVM riscv32),results/baseline/baseline.csv,总计 311,725 cycles,其中 13 个是 interrupt/exception/CSR/fence.i。
- 通过 = 每个测试的 cycle 数都与基线相等(不检查流水线细节)。
- 排除:memory-stress(太长)、load-store(clang 下非对齐 UB)、matmul_soft/mersenne/string(需要 libc)。
- 弱点:测试很小(大多 <1k cycles)。以后再加 2-3 个更长的程序(timer-mmio 最长,206k)。
- 每个剪枝步骤都需要负对照:扰动被删除的值 -> cycle 必须保持相等;
  扰动保留的 fact(分支结果、地址)-> cycle 必须变化。否则 gate 可能是空的。

## P0 对基线做 profile(在删除任何东西之前)
host 时间花在哪里:FPU/ALU 数值、PRF/DCache 数据数组、PTW/MMU、AXI/DDR、BPU、comb/seq 调用开销?
按占比排序,先删最大的,并了解上限。使用 `sample`(需要 get-task-allow)和退休指令计数器;
不要只相信 wall-clock(对布局敏感)。

P0 结果(docs/history/P0_PROFILE.md):约 68% 的 host 时间是 AXI_LLC::comb/AXI_Interconnect::seq 中的 memmove(整体状态拷贝);
数值逻辑(FPU/ALU)约 0%。所以第一个提速步骤是下面的 P3a,先于 bridge。

## P3a(提前到最前)LLC/AXI 状态:不做整体状态拷贝,去掉 line 数据 payload;gate 56/56 相等。

## P1 check 模式下的 Bridge(不注入)
- 一个功能 core(librefcpu)跟随模型取到的 PC,包括错误路径:
  当 decode 出的 pc 与 core 的下一个 pc 不同时强制跳转;每条指令一份 checkpoint/undo log;
  mispredict -> 回滚到比该分支更年轻的第一条指令;flush -> 回滚到最老的 in-flight 指令;
  推测性 store 放在 store buffer 中,commit 时发布;core 无法得知的值(timer、MMIO)做 patch。
- 标识:每条被取指的指令一个 uid,放在指令记录里携带。
- 检查:对每条被执行的指令(包括错误路径)比较 core 的 fact 与模型自己的值:
  结果、branch taken/target、load/store vaddr/paddr/size、exception cause、div/FP latency 的输入。
- 完成标准:56/56 个 0 不一致,且 cycle 不变。
- 待定:librefcpu 的 API(获取/设置状态、undo)。如果它缺少 rewind,就在 function-model 中添加(那是用户自己的代码)。

## P2 注入 fact
每条指令一个 Fact 记录(来自 bridge,以 uid 为键),在使用点替换计算:
branch resolve(taken/target -> mispred)、AGU(vaddr、paddr、size)、exception/page-fault、CSR 读值
类别、mul/div/FP latency、MMIO 读。保留时序结构(ROB、IQ、rename、LSU 队列、BPU、cache)。
完成标准:Fact 驱动一切、旧的数值路径仍在但不被读取时,56/56 相等。

## P3 删除(最大的先删,每次 gate 运行一步)
1. FPU(fadd.h 12k + fmul.h 9.7k 行):只保留 latency。
2. ALU/mul/div 结果 datapath、PRF 数据数组、bypass 数据。
3. CSR 文件的值、store data、STQ data、DCache/ICache 数据数组(保留 tag、MSHR、replacement)。
4. TlbMmu/PTW:翻译作为 fact 提供;保留 walk 时序(PTE 地址来自 bridge)。
5. bit 级:通过修改 struct 来缩减字段宽度;用 taint 找出死 bit,用 gate 验证。
顺序按依赖关系排:模型中 comb 顺序已经固定,所以字段改动是编译器检查的编辑。

## P4 提速
再次 profile。预计数值消失后,结构本身(comb/seq 阶段、22 个 issue port、带 memset 的 wire<N> struct、AXI/DDR
kit)会成为主要开销;把热点簇(frontend、ROB+LSU、IQ)重构为朴素 C++,
cycle 数由 gate 保证不变。

## P5 为 RTL 配对冻结接口
Event + Fact 协议(decode fire、dispatch、commit、mispredict、flush、fact 字段)记录在 docs/ 中;
同一个 bridge 必须能在 RTL tap 上工作。gate 的来源从旧模型切换为 RTL。

## 风险
- 中断/timer 时序基于 cycle:随机 IRQ 延迟(seed 1,8..64 cycles)必须精确复现。
- 错误路径的 load 会污染 cache:它们的地址需要 bridge 提供的错误路径值(P1)。
- gate 很小:负对照是必须的。
- 提速幅度在 P0 之前未知:如果数值只占 host 时间的一小部分,收益来自 P4,而不是 P3。

## 状态表(在 P3 结束时更新)
| 步骤 | 状态 |
|---|---|
| P0 profile | 完成(docs/history/P0_PROFILE.md) |
| P3a LLC 拷贝,P3b IQ/PreIdu 拷贝 | 完成,exact(patches 0001、0002) |
| P1 bridge(check 模式) | 完成(0003) |
| P2 fact 注入 + bridge 中的 commit 时事件 | 完成(0004、0005) |
| P3 删除数值 | 完成:FU 计算+FPU(0006)、PRF(0007)、cache 数据(0008)、统一 Uop(0009)、宽 payload(0010) |
| P4 提速(idle-tick skip、前端、后端) | 未开始 |
| P5 为 RTL 冻结接口 | 未开始 |

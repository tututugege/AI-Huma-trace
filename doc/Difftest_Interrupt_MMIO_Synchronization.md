# Difftest 中断与 MMIO 同步改造说明

## 1. 版本位置

本文记录上一轮已经跑通的中断与 MMIO difftest 改造。

- 顶层分支：`icache-dcache-retry-mmio`
- 顶层提交：`a362ae6 Synchronize interrupt and MMIO difftest state`
- Ref 子模块提交：`d85a62b Support externally synchronized difftest interrupts`
- 验证：`timer-csr-test` 通过

当前 `main` 仍位于 `origin/main` 的 `7b783e4`，不包含上述实现。本文用于说明
该提交的设计边界、具体修改和后续演进方向。

## 2. 改造目标

原来的 difftest 依赖 DUT 和 Ref 各自运行设备模型、各自决定中断何时到达。
这种方式只在设备时序严格一致时可靠。一旦中断触发时刻具有随机性，或者 DUT
和 Ref 的 MMIO 执行时机不同，就会出现以下问题：

1. DUT 已经置位 `mip`，Ref 尚未置位，普通 CSR 对比失败。
2. DUT 尚未接受中断，Ref 已经自主进入 trap，PC 和特权级失配。
3. 同一个 MMIO load 在两个设备模型中得到不同结果。
4. MMIO store 在 DUT 和 Ref 中重复产生 UART、timer 或 OCSDC 副作用。
5. OCSDC DMA 只修改 DUT RAM，Ref RAM 随后与 DUT 分叉。

本轮采用的原则是：

> DUT 是设备状态和中断时序的唯一真源，Ref 只执行指令语义；设备观测值、DMA
> 写入和最终中断接受事件由 difftest 显式同步给 Ref。

这不是每次遇到 MMIO 或中断就复制整套 DUT 状态。普通指令仍由 Ref 独立执行并
逐条对比，只有异步输入和外部副作用通过专门接口同步。

## 3. Interrupt commit 的架构语义

### 3.1 ROB 仍然提交一个有效 entry

中断被接受时，ROB 仍输出一个有效的 single-commit entry，并将该 ROB 项出队。
这样流水线内部仍能完成提交边界推进、ROB 回收和 flush 处理。

中断不是一条真实退休的架构指令。这里必须区分已有 DUT 行为和本轮新增行为。

`origin/main` 在本轮修改之前就已经包含以下 DUT 行为：

- interrupt commit 不更新 `arch_RAT`。
- interrupt commit 不按普通提交释放 `old_dest_preg`。
- interrupt flush 随后回滚 speculative rename 状态。

这些条件不是提交 `a362ae6` 新增的。本轮没有修改上述 rename/free-list 硬件状态
转换，也没有修改 ROB、STQ 或 LDQ 的回收信号。

本轮新增的是模拟器统计、commit-side 回调和 difftest 对该 entry 按 NOP 处理：

- 不增加 `commit_num`、load/store retire 计数。
- 不调用普通 `run_commit_inst()`。
- 不执行普通 `run_difftest_inst()`。
- 不通过 `commit_sync()` 产生模拟器 MMIO store 副作用。
- 改走独立的 interrupt difftest 事件，不暴露当前 entry 的 store/page-fault sideband。

寄存器和队列资源仍由 ROB 出队及 interrupt flush/rename rollback 路径处理。这里的
“架构 NOP”准确来说是模拟器 commit-side/difftest 语义，不等价于“不输出 commit
entry”，也不是本轮重新设计了 DUT 的 rename 提交逻辑。

### 3.2 中断使用独立 difftest 事件

`Ren::comb_fire()` 在看到 `rob_bcast->interrupt` 时，不再把当前 ROB entry 当成一条
普通指令送入 Ref，而是调用：

```text
SimContext::run_difftest_interrupt()
  -> SimCpu::difftest_prepare_interrupt()
  -> difftest_take_interrupt()
  -> refcpu_take_forced_interrupt()
```

DUT 提供：

- 实际接受的 `mcause/scause`
- trap 目标特权级
- 接受时刻的 `mip` snapshot
- DUT trap 后的 PC、GPR 和 CSR 状态

Ref 根据自身的 `mie/mip/mideleg/mstatus` 检查该中断在架构上是否确实可接受，检查
通过后独立执行 trap 状态转换，再与 DUT 对比。它不是直接复制 DUT trap 后的 CSR。

### 3.3 清除 Ref 的旧指令 sideband

强制中断前会清除 Ref 上一条指令留下的临时状态：

- `state.store`、store address/data/strb
- page fault 标志
- `Instruction`
- `is_io`
- `force_sync`

这修复了长时间运行中出现的以下假失配：DUT 已经进入中断，没有 store；Ref 的
架构寄存器和 PC 都正确，但仍残留上一条 store trace，最终报告
`Reference store=1, DUT store=0`。

## 4. Ref 不再自主驱动设备和中断

O3 difftest 模式下，Ref 配置为：

```text
ref_only = true
device_effects_enable = false
interrupt_delivery_enable = false
```

含义如下：

- Ref 不根据自己的 timer/UART/OCSDC 状态产生设备副作用。
- Ref 不根据自己的模拟时间自主置位并接受中断。
- Ref 的普通 RAM load/store 仍正常执行。
- Ref 的设备地址 store 只完成指令语义记录，不修改外设状态。
- 中断只在 DUT 实际接受时由 `refcpu_take_forced_interrupt()` 触发。

这样中断是否随机、设备响应延迟是多少，都只影响 DUT。Ref 不再需要复制相同的
随机数序列或设备周期。

## 5. MMIO load 同步

MMIO load 的返回值属于外部观测，不能要求两个设备模型独立算出同一个结果。

MMIO 指令提交时，difftest 根据已有的 `DebugMeta::is_mmio` 判定同步点，调用
`difftest_sync_from_dut()`，把该提交点的 GPR、CSR、PC、特权级等完整架构状态
写入 Ref。Ref 不再独立执行这条 MMIO 指令，也不需要 LSU 经 writeback 和 ROB
携带 MMIO load 返回值。

普通指令仍由 Ref 独立执行并与 DUT 比较。只有 MMIO 指令使用提交点状态同步，
因此设备响应随机性不会传入 Ref，也不会关闭后续指令的 difftest 检查。

## 6. `mip/sip` CSR 读取同步

`mip/sip` 是设备 pending 状态在架构上的可见窗口。即使 difftest 暂时忽略 CSR
数组中的异步 pending bit，程序执行 `csrr` 后，读到的值仍会写入 GPR，必须对齐。

因此提交 `mip/sip` CSR 读指令前，DUT 将该指令实际写入目标 GPR 的值通过
`refcpu_set_next_csr_read()` 注入 Ref。Ref 用这个值完成 CSR 指令的读语义和后续
CSR set/clear/write 计算。

普通寄存器对比时，以下异步 pending bit 不直接比较：

- `MTIP`
- `MEIP`
- `SEIP`

这是为了允许“DUT pending 已置位，但尚未接受中断”的合法窗口。忽略 pending bit
不意味着忽略软件可见结果，显式 `csrr mip/sip` 仍通过读值注入严格对齐。

## 7. MMIO store 和设备副作用

MMIO store 只允许 DUT 设备模型产生真实副作用。Ref 执行对应 store 指令时保持
设备 dry-run，不再重复修改设备。

按设备分类：

### 7.1 UART

- DUT commit MMIO store 时更新 UART 状态并输出字符。
- Ref 不重复输出，也不更新独立 UART 状态。
- UART status/data load 使用 DUT 的 MMIO load 结果注入 Ref。
- UART 中断是否 pending、何时被接受，以 DUT 为准。

### 7.2 Timer

- DUT timer 根据模拟器设备时间更新 pending。
- Ref timer 不自主置位 `mip`，也不自主进入 trap。
- 软件读取 timer MMIO 时使用 DUT 返回值。
- 软件读取 `mip/sip` 时使用 DUT 观察值。
- DUT 真正接受 timer interrupt 时，才强制 Ref 执行同类型 trap。

### 7.3 OCSDC

- DUT 执行 OCSDC 命令和 SD 卡数据搬运。
- Ref 不重复执行 OCSDC 命令副作用。
- OCSDC 寄存器 load 使用 DUT 返回值。
- OCSDC/XPS INTC 产生的外部中断以 DUT 的 pending 和接受时刻为准。

## 8. DMA RAM 同步

OCSDC DMA 会绕过 CPU store 指令直接修改 RAM。仅同步寄存器和 MMIO 返回值不足以
保持 Ref 一致，因此 `PeripheralModel` 在 DMA 写入后记录：

- DMA 目标物理地址
- DMA 写入长度

DUT 处理完产生 DMA 的 commit-side MMIO store 后，通过
`MemSubsystem::consume_dma_write()` 取得范围，再由
`difftest_sync_ram_from_dut()` 把该范围从 DUT RAM 复制到 Ref RAM。

这里只复制实际被 DMA 修改的区间，不做整块 RAM 同步。同步函数会检查范围必须
完全位于 RAM 内，错误的 DMA 地址会直接触发断言。

## 9. MMIO 不再强制 ROB single commit

曾经加入过 `rob_is_mmio()`，使任意 MMIO 指令在 ROB 提交时强制 single commit。
最终已经撤销该策略：

- MMIO 本身不再触发 ROB single commit。
- flush、fence、WFI 和 interrupt 原有的 single-commit 条件保留。
- MMIO 同步依赖 commit entry 已有的 `DebugMeta::is_mmio`，不依赖 single commit。
- MMIO store 副作用仍受 commit 边界约束，不需要把整组提交退化成单提交。

MMIO load 不再向 ROB 增加返回值旁带；提交点直接使用已经抓取的 DUT 架构状态
覆盖 Ref。

## 10. PC 计算修正

普通 difftest 指令准备阶段不再用统一的 `rob_bcast->flush` 判断 next PC，而是区分：

- `mret/sret`：使用 CSR 输出的 `epc`
- exception：使用 CSR 输出的 `trap_pc`
- branch/JAL：使用指令计算出的目标地址
- 普通指令：`pc + 4`

interrupt 不走这条普通指令 next-PC 路径，而走独立的 forced-interrupt 流程。

## 11. 当前时序边界

本轮没有实现中断触发随机化。UART、timer 和 OCSDC 的 pending 信号仍按当前模拟器
设备模型立即更新，并在满足 CSR enable/delegation 条件后由 ROB 接受。

本轮工作的价值是先解除 difftest 对“Ref 必须在同一周期自行产生同一中断”的
依赖。后续加入随机延迟时，只需要在 DUT 模拟器的设备 pending/注入路径增加随机
调度；Ref 和 difftest 协议不需要跟着复制随机时序。

## 12. 验证结果

完成上述修改并撤销 MMIO single commit 后，执行：

```bash
./build/simulator baremetal/timer-csr-test/build/timer_csr_test.bin
```

测试成功收到 machine timer interrupt：

```text
[TIMER-CSR] trap_count=1 mcause=0x80000007 ... mip=0x00000080
[TIMER-CSR] PASS
```

完整 Linux 启动测试也已由人工长跑验证通过，随后将工作状态保存为顶层提交
`a362ae6` 和 Ref 子模块提交 `d85a62b`。

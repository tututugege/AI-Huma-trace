# P3：删除值数据通路（value datapath）

> 历史记录:P3 迁移之前的状态(2026-11-04),可能已过时,现状见 [README.md](README.md)。

顺序依据证据（P0 profile + poison control）而定，而不是按规模大小。每一步都要满足：56/56 个测试的 cycle 数与 baseline 相同，
fact miss 为 0，csr_read patch 仅用于 mip/time（timer-interrupt-mask 中共 2 处）。在 facts 为强制的情况下，模型运行时不带
difftest（bridge 开启时 main.cpp 会设置 disable_difftest）；check 模式（TB_CHECK，P1b）是 P3 之前的工具，一旦值被删除就失去意义。

## P3.1（patch 0006）：ALU / MUL / DIV / FP 计算、FPU softfloat、生成的 FADD/FMUL
- Fu.h 498 -> 255 行，FPU.h 380 -> 59 行（延迟仅由操作类型决定），fadd.h（12,409）和 fmul.h（9,673）被移除。
- MUL/DIV 延迟固定；FPU 延迟只取决于操作类型：不需要延迟 fact。
- 速度：约 0（timer-mmio 上宿主指令数 92.1 G -> 92.0 G），与 P0 的预测一致；bridge 的开销已计入其中。

## P3.2（patch 0007）：PRF 寄存器堆与操作数字段
- 移除了 reg_file/reg_file_1、read_operand_with_bypass、PrfExeUop/MicroOp 的 src1_rdata/src2_rdata、PRF 写端口。
  写回级 `inst_r` 保留（它驱动 load 唤醒和 bypass 时序）。
- AGU、BRU、SDU 和 CSR 的写数据现在只读取 fact（被 kill 的指令回退为 0，其结果不可观察）。
- 寄存器的 ckpt restore 和 get_reg() 不复存在（值保存在 bridge 中）。
- 速度：同样约 0（profile 中 PRF 仅占宿主时间的 0.1%）。

## 顺带发现的精确优化（在 patch 0007 中）
MemSubsystem::AxiLlcTableRuntime::make_write_req 在每个 cycle 的 16 次 AXI tick 中，对 4 张 LLC 表的每一张都会分配并清零一个行 payload
（以及一个 chunk-enable 向量），即使该写操作未启用；而 DynamicStorage::write_chunks 会忽略被禁用的请求。
改为在分配之前返回：timer-mmio 宿主指令数 92.0 G -> 78.8 G（-14%），cycle 不变。

## 现在时间花在哪里（timer-mmio，2,260 个采样，baseline 时为 8,200）
前端 27.7%（icache_comb_calc、BPU），BackTop::comb 24.4%（IssueQueue/Isu ~94、TlbMmu 60），AXI+LLC 约 32%
（Interconnect comb_outputs 12.4%、LLC::comb 10%），dcache 6.1%。栈顶：memmove 16%（copy_regs_live 前缀、
PreIduQueue::seq、IssueQueue assign），memset 5%。
目前的速度：11.9 k -> 约 42 k cycles/s（timer-mmio 206,120 个 cycle，约 4.9 s CPU）。

## 尚未完成（cache 数据）
L1D/LLC 数据删除的设计：store 在 ROB commit 时依据 fact（字地址 + step 之后的字）写 pmem，
SimDDR 不再写 pmem，LLC/ifetch/PTW 从 pmem 读取数据；之后 L1D/MSHR/WB/LLC 的 payload 成员即可删除。
预期对速度的影响很小（空闲 slot 的 payload 拷贝本来就已被跳过）；价值在于减少代码/状态。

# P3.3：cache 数据（patch 0008、0008b）

设计：功能内存（pmem）是唯一保存值的地方；其他所有路径只携带 tag/时序。
- store 在 ROB commit 时依据 fact（字地址 + step 之后的字）更新 pmem；包含 AMO，失败的 SC
  不产生 store。SimDDR 不再写 pmem（axi_kit_ddr_functional_writes = false）。
- LLC：4 MB 的数据表被移除；查找时利用 meta/valid 表从 pmem 合成该 set 的数据行；
  每一个读响应（hit、refill、转发）都由 pmem 构造（extract_line_response）。
- L1D：数据阵列被移除（DcacheConfig.cpp）。可能被写回的 victim 快照从 pmem 读取。
- MemRouteBlock：经由 L1D 读端口返回的指令数据和页表项现在来自 pmem
  （方法是把 L1D 读数据标记为 0xCAFEF00D，然后观察它作为指令字出现）。
- 结果：56/56 个 cycle 数与 baseline 相同；timer-mmio 仍为 78.4 G 宿主指令数不变（AXI/LLC/MSHR/write-buffer 结构体中的
  payload 成员仍然存在，也仍在被拷贝）。

对照实验（Controls）
- 新增了有区分度的测试（tests/extra）：smc-div / smc-addi（交替执行 `addi` 与 `div` 的自修改代码）。
  原始模型：8,996 / 8,866 cycle。Trace 构建：8,996 / 8,866。关闭 commit 时内存写入
  （TB_NEG_NO_MEMWRITE）后：smc-div 为 8,867（陈旧指令），smc-addi 不变。在这一步之前该对照没有差异，
  因为数据 cache 仍然携带着值。
- 原始模型作为 baseline 二进制保留：third_party/tututugege/baseline-sim（2f31e8d 的 worktree，带有 P3a 的完整
  kit 改动）；它能复现 baseline.csv，并可给出任何新测试的 cycle 数。

# 回归限制（scripts/run_gate.sh）
过去一次挂死会消耗一次外部超时。现在：每个测试的 SIM_MAX_CYCLES（main.cpp，退出码 3）= max(3000, 4 x baseline
cycle)，8 个任务并行，另有一个 20 s 的硬性截止时间作为最后手段。56 个测试的 gate 耗时约 24 s。

## cache 数据删除的已知限制
- LLC 模式 "mapped scratchpad"（LLC 作为 0x30000000 处的 RAM）只在 LLC 表中保存数据：不支持。
- 页表遍历器（page-table walker）写入的 A/D 位不会镜像到 pmem。
- payload 成员（AXI 结构体中的 WideReadData/WideWriteData、LLC 队列、MSHR/WB 行）仍然存在：留给 P3.4。

# P3.4a：所有流水级共用一个指令记录（patch 0009）

之前：19 个按流水级划分的结构体（InstInfo、MicroOp、DecRenInst、RenDisInst、DispatchInst、DisRobInst、DisIssUop、IqStoredUop、
IssPrfUop、PrfExeUop、ExuInst、ExePrfWbUop、ExuRobUop、ExeLsuReqUop、LsuExeRespUop、RobStoredInst、RobCommitInst，...），
以及约 17 个逐字段的转换函数。它们合计共有 49 个不同的字段；只有 `op`（UopType 与编码后的 wire）和
`type`（InstType 与编码后的 wire）的声明方式不同。
现在：`struct Uop`（back-end/include/types.h，112 字节，49 个字段）；每个流水级结构体都是 `using X = Uop;`，
转换函数是恒等成员，op/type 到处都是枚举（encode/decode 是恒等的）。
- 重复的辅助函数重载（is_store/is_load/is_exception/is_flush_inst/is_page_fault/cmp_inst_age 曾最多存在 6 份，
  分别针对 InstInfo、MicroOp 和四个 IO 结构体）现在各只有一份。MicroOp（基于 op）的变体没有调用者。
- RobCommitExtra（src1_areg、imm）直接在 Rob.cpp 中填充。
- 代码：9 个文件，+60 / -1031 行。56/56 个 cycle 数与 baseline 相同；smc-div/smc-addi 与原始模型一致。
- 速度：timer-mmio 78.4 G -> 79.6 G 宿主指令数（+1.6%）：每次双缓冲拷贝都要搬运完整的 112 字节。
  为了简洁而接受；下面的字段裁剪可以把它赢回来。
未完成：缩减那些只承载值的字段（result/diag_val/ftq_next_pc/imm 仍然需要：MMIO store 数据和
load 结果、地址、目标），以及 AXI/LLC/MSHR/write-buffer 的 payload 成员。

# P3.4b：AXI/LLC 宽数据 payload（patch 0010）
- WideReadData_t：256 B -> 64 B（一条 cache line，16 个字）；WideWriteData_t：64 B -> 4 B（一个字）。operator[] 会钳制
  下标（超出保留字的部分是垃圾别名），成员是 private 的，所以每一处直接的 `.words[]` 访问都必须被复查
  （AXI_Interconnect.cpp / AXI_LLC.cpp 中的 20 处，现在都用 operator[]）。
- WideWriteStrb_t 不是 payload，保持 64 B：字节使能决定 LLC 中是整行写还是部分行写
  （read-for-ownership），也就是说它们驱动时序。
- 第一次尝试读方向只保留一个字：所有测试都挂死（32 s 内 SIM_MAX_CYCLES 触发 rc=3，dispatch=flush=3333）：
  CPU 从 0x0 处的 boot ROM 启动，那是一个 MMIO 区域，它的指令字是作为 MMIO 读响应的一整行到达的。
  一条 cache line 就能覆盖这一点。
- 56/56 个 cycle 数与 baseline 相同（gate 17 s）；smc-div/smc-addi 与原始模型一致。
- 速度：timer-mmio 79.6 G -> 76.0 G 宿主指令数（-4.6%），约 4.8 s CPU，约 43 k cycles/s。
- 剩余类似 payload 的成员：AXI_LLC_Bytes_t 向量（表写 payload，构造后丢弃）、MSHR / write-buffer / FILLWrite / FillIn 中的
  DCACHE_WORD_NUM 数组。profile 显示 AXI/LLC 的占比（约 33%）主要是逻辑
  （comb_outputs、LLC::comb、静默检查）每个 cycle 运行 16 次，而不是 payload 拷贝。

# P3.5：以 id 索引的 uop（patch 0011）
流水线寄存器和 IO 端口不再携带整个指令记录。现在一个 `Uop` 是 `id` + uop 在飞行期间会变化的字段
（56 B，原先 112 B）；uop 创建时写入的一切内容都放在一个池 `UopS`（每项 64 B，`uop_pool[id]`）中，
通过 `u->imm`、`u->pc`、`u->type`、`u->dbg` ... 读取（23 个字段：pc、imm、ftq_*、dest/src areg、
src preg、old_dest_preg、enables/flags、func3/func7、br_id、csr_idx、is_atomic、type、dbg）。
为什么不是"只放 id"：模型是 comb/seq 双缓冲的（`x` 为已提交副本，`x_1` 为下一个副本）。如果每个字段都放在
一张共享表里，`entry` 和 `entry_1` 会互相别名，而下一状态副本中改变的字段（br_mask、src busy 位、
完成掩码、结果、page-fault 位、ftq_is_last、is_mmio，...）会被当前状态的读者看到。
这些字段仍留在 entry 中；它们是动态部分。
- id：前端在指令进入指令缓冲时分配一块 4 个池项（停顿的指令每个 cycle 都会重新运行 decode，因此它不能分配）；
  指令拥有 [id]，它分派出的 uop 使用 [id+1+k]（k <= 2：STA/STD、JAL/JALR、AMO），在 Dispatch 中根据指令的 entry 填充。
  分配是确定性的，所以重新求值一个 comb 级不会分配任何东西。环大小为 65,535 项（约 16 k 条指令）；存活时间
  超过这个长度的指令会被覆盖（未见发生：死锁检测器在 50,000 个 cycle 时触发）。
- 0 号项是共享的空项，供默认构造的句柄使用（LSU 写回 uop）；如果它被写入，析构函数会 abort。
- 通过二分发现的 bug（所有字段都为动态时通过；再把字段分组移回）：Rename 为所有 slot 写 old_dest_preg / src preg，
  包括无效的 slot，而这些 slot 的 `inst_r[i]` 仍持有一条已在 ROB 中的指令的句柄，于是写入命中了一个活跃的 entry
  （过去它命中的是一个丢弃的副本）。修复：只写有效的 slot。要警惕的就是：对所有 slot 都做计算、并通过陈旧句柄
  写静态字段的流水级。
- 这次重写由编译器驱动：静态字段只能通过 `->` 访问，因此对已移动字段的每一次访问都是一个编译错误，其位置
  由脚本改写（/tmp 脚本，未保留）。
- 结果：56/56 个 cycle 数与 baseline 相同；timer-mmio 76.0 G -> 74.0 G 宿主指令数（-2.6%），约 45 k cycles/s。
  收益很小，与早期的 RSD 实验一样；这种结构现在使流水线更易于推理
  （创建一次 vs 可变），并且是进一步裁剪可变部分的基础。

# 清理 1：profile 基础设施与统计（patch 0012）
- 移除了 host_profile.{h,cpp}、frontend_stats.{h,cpp} 以及 47 处 FRONTEND_HOST_PROFILE_SCOPE 的使用。
- PerfCount.h 1,404 -> 33 行：只保留 cycle / commit 计数器以及 gate 读取的 `sim-time(cycle)=` 行。372 处计数器使用
  是由编译器驱动移除的（在每个 "no member named" 错误处删除该语句；多行的语句和统计辅助函数手工处理）：MemSubsystem::sync_llc_perf、L1D 的 COUNT_* 宏、LSU 的 count_*/sample_* 辅助函数
  （RealLsuPerf.cpp 保留行为部分：dcache 响应超时重放）、Dispatch/Ren/Idu/PreIdu 的 TMA 块。
- 看起来像统计但实际是行为的东西：RealLsuPerf.cpp 中的 CONFIG_LSU_DRESP_TIMEOUT_CYCLES 重放。保留。
- 56/56 个 cycle 数与 baseline 相同，smc-div/smc-addi 相同；模型源码 60.9 k -> 58.1 k 行；timer-mmio 74.0 G -> 72.7 G
  宿主指令数（-1.8%）。
清单中的下一项（见讨论）：固定默认的前端/后端 profile（去掉 small/medium/large 以及死的 #if
分支），然后把 ICache 重写为只有 tag + miss/refill 时序（它的数据阵列已经是死的，因为指令字来自
pmem；约 5.5 k 行的 bool 数组 kernel + glue）。

# 清理 2：单一配置（patch 0013）
- 只保留默认 profile：移除了 include/config.h.{small,medium,large,default} 和
  front-end/config/frontend_feature_config.h.{small,medium,large,default}；当前生效的头文件是 include/config.h 和
  front-end/config/frontend_feature_config.h（它们与 default 文件逐字节相同）。Makefile 去掉了 PROFILE 以及
  default/small/medium/large 目标。
- 这四个 profile 的差异不止数值：规模（宽度、ROB/PRF、队列、预测器表、延迟）、issue 端口表
  （default 中 23 个端口，其他为 11/13/16）、TAGE 子组件（ENABLE_TAGE_SC_LITE / SC_L / SC_PATH /
  LOOP_PRED 在 default 中为 0，其他为 1）以及 CONFIG_ORACLE_STEADY_FETCH_WIDTH（只在 default 中定义）。
- 死的 #if 分支用 `unifdef -m` 剪除（对配置头文件中定义且未在别处重新定义的 136 个整数宏用 -D，对条件中被引用但从未定义的宏用 -U，例如 LOG_*_ENABLE、BSD_CONFIG、SRAM_DELAY_ENABLE、
  FRONTEND_ENABLE_*_BYPASS、CONFIG_LSU_DRESP_TIMEOUT_CYCLES）。这些符号不能完全决定的条件保持不动；
  本项目的调试宏（IQ_SELFCHECK、TB_NEG_*）保留。27 个文件，-1,390 行（TAGE_top.h 1,893 -> 1,519，
  front_top.cpp 2,095 -> 1,711）。
- 更正：CONFIG_LSU_DRESP_TIMEOUT_CYCLES 在任何地方都没有定义，因此 RealLsuPerf.cpp 中的 dcache 响应超时重放
  是死代码（清理 1 中出于谨慎保留了它，现在已删除）。
- 56/56 个 cycle 数相同，smc-div/smc-addi 相同，宿主指令数不变（72.7 G）：死代码本来就不执行。
- 模型源码现在为 56.7 k 行（原为 83.9 k，-32%）。

## ICache S1：移除生成的 kernel 以及 pack/unpack 层

- 删除了 ICacheCompactKernel.h（2373 行）、ICacheLookupMetaPackKernel.h、ICacheTopTrueGlueKernel.h 以及 bool 数组的 pack/unpack glue；直接使用 `comb_core` / `eval_true_glue`（native 路径）。
- front-end/icache：约 7.9k -> 3.3k 行。
- Gate 56/56 与 baseline 相同；smc-div 8,996，smc-addi 8,866（不变）。Patch：patches/0014-icache-s1.patch。
- 下一步：S2 去掉 ICache 数据阵列（从 pmem 取指令字），S3 简化 comb_core。

## ICache S2：去掉数据表

- icache.cpp：移除了 DataTable（每个 set 每路 64 B）及其读/写/复位路径和 focus-dump 辅助函数。tag 命中时，line 的字从 pmem 的 (tag << 12 | index << offset) 处读取。
- Gate 56/56 与 baseline 相同；smc-div 8,996，smc-addi 8,866。Patch：patches/0015-icache-s2.patch。
- 下一步：S3 简化 comb_core；refill 仍在 icache_module 内部携带 line 字（finish_refill / table_write.data），可在之后裁剪。

## ICache S3：refill 不再携带 line 数据

- finish_refill 从 pmem 读取 line（单字的 uncached 响应仍使用 AXI 字）；移除了 `table_write.data`、`dcache_probe_data_r`、`DcacheProbeStep::line` 以及空的拷贝循环。D-cache probe 只保留其时序（done/hit/字计数器）。
- front-end/icache：3,150 行。Gate 56/56，smc-div 8,996，smc-addi 8,866。Patch：patches/0016-icache-s3.patch。
- 剩余：`mem_resp_data_r` / `mem_resp_data_w` 仍然保存一整条 line（只有 uncached 字才需要）；可缩减为一个字。comb_core 的控制流（状态机、perf_state）尚未简化。

## AXI 空闲 tick 跳过

- 每个核 cycle 额外的 15 次 AXI 子系统 tick（FREQ_DIV 16，加上 L1D 快速域）被截断：在第 2 个 tick 之后，如果 interconnect+LLC、DDR、MMIO 总线以及 L1D 快速域（RealDcache 的流水级寄存器、MSHR、write buffer、响应 meta）全部处于静默状态，其余 tick 都是不动点，直接 `break`。判断谓词：AXI_Interconnect/SimDDR/MMIO_Bus/MSHR/WriteBuffer/RealDcache 中的 `idle_quiescent()`，MemSubsystem 中的 `dcache_domain_quiescent()`。
- 精确性：gate 56/56，smc-div 8,996，smc-addi 8,866。`AXI_IDLE_VERIFY=1` 会仍然运行所有 tick，并检查静默状态持续存在且边界状态不变：整个 gate 上共 249,766 次检查，0 次不一致。（verify 只检查静默性 + CPU 可见的边界，不检查完整内部状态；gate 才是完整检查。）
- 速度（timer-mmio，206,120 个 cycle）：4.06 s（所有 tick）-> 2.89 s，即约 51k -> 约 71k cycles/s，1.4 倍。Patch：0017-axi-idle-skip.patch、0017b-kit-idle-quiescent.patch。

## BPU 拷贝/清零清理

- bpu_comb_calc 不再构造 BpuCombOut（10.6 KB 清零外加两次 10 KB 拷贝）；核心直接写 OutputPayload/UpdateRequest，front_top 就地传递 `bpu_seq_txn_req.req`。
- BpuNlpCombIn 之前按值持有 ReadData（每 cycle 拷贝 13 KB）；现在持有 const 引用。未使用的 BpuCombIn/BpuCombOut 被移除。
- GHR/FH 历史更新就地完成（`tage_hist_update_inplace`，memmove 移位），而不是每条分支拷贝 6 份 256 字节的 GHR。
- 结果：gate 56/56，smc 8,996 / 8,866 不变。timer-mmio user 时间（5 次交替运行的最小值）：2.673 s -> 2.554 s（-4.5%）；宿主指令数 38.26 G -> 37.27 G。比 profile 占比所暗示的要小：BPU 的 memset/memmove 降到约占采样的 7%；BPU 其余开销是真正的预测器逻辑。Patch：0018-bpu-copies.patch。
- 下一个最大的拷贝：PreIduQueue::seq（约占采样的 6%）、IssueQueue vector assign（约 3%）、SimCpu::cycle 拷贝（约 2%）。

## PreIduQueue：按 slot 同步而非整个队列拷贝

- seq() 过去只要有任何变化就会拷贝整个 128 项的指令缓冲（约 3 KB）以及全部 64 个 FTQ 项 + train-meta（数十 KB）。现在 `InstructionBuffer::sync_from` 只拷贝被 push/pop 触碰过的 slot（clear 会触碰所有 slot），FTQ 只拷贝本 cycle 分配的 slot（`ftq_alloc_mask`）加上 64 字节的 valid 数组。项的 payload 只在分配时改变，所以这是精确的。
- Gate 56/56，smc 8,996 / 8,866。timer-mmio user 时间（5 次交替的最小值）：2.560 s -> 2.414 s（-5.7%）。未运行：IQ_SELFCHECK 构建（committed==working 的 memcmp）。Patch：0019-preidu-sync.patch。
- 下一批拷贝目标：IssueQueue vector `assign`（约占采样的 3%）、SimCpu::cycle 拷贝（约 2%）、front2back_FIFO_seq_write、Ren::seq。

## IssueQueue：按 slot 的 entry 同步

- seq() 过去只要任何 slot 变化就会赋值整个 `std::vector<IqStoredEntry>`。现在对 working 副本的写入会标记该 slot（`mark_slot`：wakeup、enqueue、issue/flush 移除、br_mask 清除）；seq 只拷贝被标记的 slot（flush_all 拷贝全部）。
- Gate 56/56，smc 8,996 / 8,866。IQ_SELFCHECK 构建（每个 cycle 对 IssueQueue 和 PreIduQueue 做 committed==working 的 memcmp）同样 56/56，没有 abort。timer-mmio user 时间 2.414 s -> 2.343 s（-3%）。Patch：0020-iq-slot-sync.patch。
- 自 AXI 跳过以来的累计（profile 时为 2.89 s wall / 2.658 s user）：2.343 s user。
- 剩余的拷贝目标：SimCpu::cycle 拷贝、front2back_FIFO_seq_write、Ren::seq、BPU comb_bpu/seq_read 的 memset、AXI copy_regs_live。

## 前端裁剪（"紧凑地重写，只保留功能 + cycle" 的第 1 轮）

- 用有效宏值运行 unifdef 移除了死分支（2-ahead/NLP、TAGE-SC-L/loop/path、理想 ICache、SRAM 延迟模型）。
- 移除了 falcon/front 统计计数器（其汇总本来就是空桩）、DEBUG_LOG* 调用、SIM_DEBUG_PRINT 块和 focus-dump 函数、从未被实例化的 SimpleICacheTop 与 BitWriter/BitReader，以及未使用的 train_IO.h 别名/结构体。
- 前端 13,678 -> 12,644 行。每一步之后 gate 56/56，smc 8,996 / 8,866。Patch（3 个 commit）：0021-frontend-prune.patch。
- 估计复核：我之前"-2-3k"的猜测过于乐观；死代码比设想的少。剩下的规模都是活的结构：BPU 5.5k（算法 + comb 包装）、front_top 1.4k、ICache 约 3k、各流水级的 IO 结构体。

## BPU 包装层（紧凑重写的第 2 轮）

- 2-ahead / NLP 预测器已被编译掉（ENABLE_2AHEAD 未定义），但它的状态、表、comb 函数以及 saved_2ahead / mini_flush 的连线仍然存在：已从 BPU 移除（状态、ReadData/UpdateRequest 字段、bpu_nlp_comb），并且恒为 false 的 mini-flush / 2-ahead 输出也沿 front_top、front_IO、PTAB（dummy 项）和 train_IO 一路移除。这是精确的，因为这些值在生效的构建中都是常量（false / 0）。
- 历史和更新队列的 comb 包装被内联：`bpu_hist_comb(inp, rd, type_out, req)` 和 `bpu_queue_comb(inp, rd, req, out)` 现在直接读取输入/快照并直接写 `req`（由脚本根据 pack/unpack 行生成），移除了 4 个包装结构体以及约 150 行 memcpy/字段拷贝。
- BPU.h 2000 -> 1439 行，前端 12,644 -> 12,095。每一步 gate 56/56，smc 8,996 / 8,866。timer-mmio user 时间 2.328 -> 2.291 s（-1.6%，很小但方向正确）。Patch（3 个 commit）：0022-bpu-wrappers.patch。
- BPU 中剩余：PreReadReq / PostReadReq / SubmoduleBind / PredictMain 包装（它们的输出会扇出到 ReadData 和 UpdateRequest，所以映射不是 1:1，需要手工合并），以及 TAGE_top.h/BTB_top.h 中每个 bank 的 TAGE/BTB comb/seq 包装。

## BPU 包装层，剩余部分

- 将 `bpu_pre_read_req_comb` + `bpu_data_seq_read` 合并为 `bpu_data_read(inp, rd)`（地址/RAS/队列读阶段直接写 rd）；内联了 post-read、submodule-bind 和 predict-main 包装：这些函数现在接收 `inp`/`rd` 加上真正的中间值（type_out、tage_out、btb_out），并直接写 `out`/`req`。又移除了 8 个包装结构体及其 pack/unpack 代码（hist/queue 已在第 2 轮完成）。输入映射由旧的 pack 代码机械推导；predict-main 的输出映射为 `out.out.X -> out.X`，其余全部 -> `req`。
- BPU.h 1439 -> 1239 行；前端 12,095 -> 11,886。每一步 gate 56/56，smc 8,996 / 8,866。Patch（3 个 commit）：0023-bpu-wrappers2.patch。
- BPU 中剩余：TAGE_top.h（1.5k）/ BTB_top.h（1.4k）/ TypePredictor.h（0.4k）带有同样的 In/Out/CombResult 包装风格，它们本身就是预测器算法。

## front_top glue

- BPU 输入过去被拷贝四次（front_top_in -> BPU_in 种子 -> control-in -> BPU_in -> BPU_TOP::InputPayload）；现在只构建一次。global-control、read-enable 和 read-stage-input 的 glue 函数及其 6 个 In/Out 结构体被内联为普通的布尔表达式。front_top.cpp 1,318 -> 1,137 行，前端 11,886 -> 11,643。Gate 56/56，smc 8,996 / 8,866。Patch：0024-front-top-glue.patch。
- 未完成及原因：ptab_write / checker_input / front2back_write / output comb 函数会在近乎相同的结构体之间（BPU 输出、PTAB_in/out、front2back_in/out、front_top_out、后端中的 FTQ 项）逐字段拷贝*预测元数据包*（pcpn、altpcpn、tage_idx/tag[4]、sc_*、loop_*）。内联它们只能省约 60 行。真正的收益是把这个包只定义一次（`PredMeta`），让 FIFO 把它作为一个整体存储/拷贝，这会涉及 PTAB/front2back FIFO 模块和后端接口（用 gate 一步一步来）。

## PredMeta：预测元数据成为一个结构体

- `PredMeta`（wire_types.h）包含随取指组一起传递的 14 个每 lane 字段（alt_pred、altpcpn、pcpn、tage_idx/tag[TN_MAX]、sc_used/pred/sum/idx、loop_used/hit/pred/idx/tag），外加用于硬件位宽的 `mask()`。它取代了 PTAB_in/PTAB_out/PTAB_entry、front2back_FIFO_in/out/entry 和 BPU::OutputPayload（重命名 out_* -> meta.*）中逐字段的声明，并且 PTAB.cpp、front2bank_FIFO.cpp 和 front_top.cpp 中的每个拷贝循环现在都是 `dst.meta = src.meta`（旧代码做过 mask 的地方加上 `mask()`；mask 是幂等的）。同时移除了死的 BPU_in / BPU_out 结构体和 lane 宽度的 static_assert。
- 前端 11,643 -> 11,379 行。Gate 56/56，smc 8,996 / 8,866。timer-mmio user 时间 2.309 -> 2.204 s（-4.5%，结构体拷贝取代了逐字段循环）。Patch：0025-predmeta.patch。
- 未统一：BACKEND_FETCH_WIDTH 宽的 front_top_out（平铺数组，在 front_output_comb 中拷贝一次）、COMMIT_WIDTH 宽的更新路径（front_top_in、BPU InputPayload、QueueEntry）以及后端 FTQ 的 train-meta；字段列表相同，但宽度/方向不同，需要一个宽度模板。

## 分页测试：bridge 不感知地址转换（已发现并修复）

- 新增的 gate 成员：四个上游分页测试（vm-asid / vm-pagefault / vm-sfence-perm / vm-sum-fetch），由 scripts/build_vm_tests.sh 构建到 build/vm/（clang，-fgnu89-inline）；baseline cycle 为 68,737 / 29,412 / 38,695 / 2,694（baseline 模型：全部 Success）。run_gate.sh 现在运行 60 个测试。
- 修复前 trace model 有三个失败、第四个挂死（4 个中有 3 个出现 "translated store did not update backing physical word"，rc=3）。原因：TraceBridge 把它的 RAM 范围检查和 store undo 地址应用在了*虚拟*地址（以及虚拟 pc）上，所以任何经由恒等范围之外映射的访问都被当作"blocked"而从不执行，undo/redo 也使用了错误的字。
- 修复：新增核心 API `refcpu_translate_data(ctx, vaddr, type, &paddr)`（无副作用，决策与 exec.cpp 相同：satp 为 bare 或有效特权级为 M 时为恒等映射，否则走 va2pa；type 0 = fetch 忽略 MPRV）。bridge 对 in_ram / misalign / undo 使用物理地址，并把 page fault 视为"执行并让核心抛出异常"，而不是 blocked。fact 的 `vaddr` 保持为虚拟地址，因为模型自己的 TLB/PTW 会对它做转换。
- 之后：四个分页测试全部 Success，cycle 数与 baseline 相同；原有的 56 个测试、smc-div/addi（8,996 / 8,866）、dhrystone（3,911,612）和 coremark x3（323,962）均不变。Patch：0027-bridge-translation.patch、0027b-refcpu-translate.patch。
- 对 TLB/PTW 重写计划的影响：gate 现在覆盖了开启地址转换的路径（page walk、ASID、sfence、SUM、权限 fault），因此可以验证 TlbMmuKernel / PtwCompactCoreKernel 的原生重写。

## TLB / PTW 位向量 kernel 被移除
- TLB：TlbMmu 已经有原生的 `core_comb()` / `top_comb()`；运行时却走 `encode_*_bits -> tlb_kernel_*_io_generator -> decode_*_bits`（TlbMmuKernel.h，1,547 行，外加 TlbMmu.cpp 中约 300 行的 codec 以及 TlbMmu.h 中的宽度常量）。先用一个临时的 TLB_NATIVE_VERIFY 在全部 60 个 gate 测试上对比 native 与 kernel（9,440 次非平凡调用，其中四个分页测试中有 2,684 次，0 次不一致），然后把 `run_top_via_io_generator` 切换为 `top_comb(in, entries)`，并删除了 kernel、codec 和常量。TlbMmu.cpp 1,551 -> 1,080，TlbMmu.h 410 -> 307，TlbMmuKernel.h 被删除：合计 -2.1k 行。bare 模式的快速路径及其 verify 代码一并移除（native 路径已足够便宜：相对 profile 二进制，timer-mmio user 时间 2.226 -> 2.061 s）。
- PTW：`PtwCompactCoreKernel.h`（1,157 行）以及 MemPtwBlock.h 中打包的 encode/decode + generator 包装在运行时不可达（头文件之外没有调用者）；已移除。MemPtwBlock.h 1,236 -> 674：合计 -1.7k 行。
- 之后：gate 60/60（56 个原有测试 + 4 个分页测试），smc 8,996 / 8,866，dhrystone 3,911,612，coremark x3 323,962，全部与 baseline 相同。Patch：0028-tlb-native.patch、0029-ptw-kernel-removed.patch。

## 顶层与后端 IO.h

- rv_simu_mmu_v2.cpp 1,365 -> 1,051：移除了模型侧的 difftest hook（自 P3 起已死：没有寄存器堆，trace 模式下 enable_difftest 始终关闭；为 tbhook::on_interrupt 提供输入的中断 hook 保留为 `run_interrupt_hook`）、配置 banner、未使用的内部 tick 辅助函数，以及 commit_sync 中空的 TMA/统计分支。main.cpp / harness 对参考核心的使用不变。
- back-end/include/IO.h 1,025 -> 711：44 个 IO 结构体的构造函数只是用循环把每个字段清零；替换为 `= {}` 成员初始化器（相同的零状态）。Gate 60/60，smc 8,996 / 8,866，dhrystone 3,911,612，coremark x3 323,962；timer-mmio user 时间不变（2.073 s）。Patch：0030-top-cleanup.patch、0031-io-initializers.patch。8 个带有非平凡构造函数的 IO 结构体保持原样。

## 后端：按宽度的 PredMeta

- `PredMetaT<N>`（wire_types.h；`PredMeta = PredMetaT<FETCH_WIDTH>`）及用于窄到宽拷贝的 `assign_prefix()` 现在被 front_top_out、FrontPreIO 和 FTQ train meta（BACKEND_FETCH_WIDTH = 16）使用：rv_simu 中前端到后端的 glue、PreIduQueue 的 FTQ 写入以及 front_output_comb 的 lane 拷贝都变成了结构体拷贝（约 -150 行）。Gate 60/60，smc、dhrystone、coremark 相同；timer-mmio 2.035 s。Patch：0032-predmeta-width.patch。
- 对后端其余部分的排查（脚本：字段数 >= 8 的结构体，共享 >= 8 个字段名的结构体对）：仅剩的重叠是 COMMIT_WIDTH 的 commit 侧包（front_top_in 与 FtqCommitInfoResp，同样的 14 个名字，但前者是每 lane 标量、后者是 lane 数组；约 50 行）以及 StqEntry/LdqEntry（9 个名字）。后端的主体是以 RTL 风格写成的真实逻辑（Dispatch 714、Rob 761、RealLsu 1,529、Lsu 总计 5.6k、Exu 1.8k），而不是重复的声明。

## Exu：表驱动的 FU

- AbstractFU / SingleCycleFU / FixedLatencyFU / IterativeFU 以及 AluUnit、MulUnit、DivUnit、AguUnit、SduUnit、BruUnit、CsrUnit 和 FPUSoftfloat（impl_compute 对 ALU/MUL/DIV/CSR/FPU 是空操作）合并为一个类 `Fu`，带有一个 `FuKind`（ALU、MUL、DIV、AGU、SDU、BRU、CSR、FPU）和三种时序模式：SINGLE（一个 cycle）、PIPE（固定延迟流水线：MUL）、ITER（一次一个，延迟取决于 op：DIV、FPU）。剩下的计算只有 facts 所取代不了的部分：AGU 地址、SDU store 数据，以及 BRU 对照 FTQ 预测的 taken/target 检查。`Exu::init` 根据一张 (kind, op-mask) 表构建这些单元，而不是九个复制粘贴的块。
- Exu 目录 1,804 -> 1,514 行（AbstractFU.h、FPU.h 被删除，Fu.h 167 -> 220，含全部内容）。Gate 60/60，smc 8,996 / 8,866，dhrystone 3,911,612，coremark x1/3/10 相同；timer-mmio user 时间 2.228 -> 2.020 s（vtable 调用没有了）。Patch：0033-exu-fu-table.patch。
- Exu 中剩余的部分：Csr（586 行：trap 进入/返回、委托、向量化 trap、中断；真实状态，决定重定向 pc 以及 TLB 看到的 satp/mstatus）和 Exu.cpp（约 600 行：issue 端口 / FU 编排、写回收集、分支仲裁）。

## Lsu 一轮：统计/调试遗留与死值

- 第 1 步（patch 0034）：移除 RealLsuPerf.{h,cpp}（内存延迟/等待起点统计只是空壳；8 个很小的 replay setter 被内联；`load_miss_mask` 作为静态辅助函数保留；perf 字段和 *_PERF_FIELDS 宏被移除）。所有 dump_debug 函数被移除（RealLsu 160 行、TlbMmu、MemSubsystem/RealDcache/MSHR/WriteBuffer/MemRouteBlock/MemPtwBlock、front_dump_debug_state、icache_dump_debug_state），以及死锁/diff trace 基础设施（DeadlockDebug、DeadlockReplayTrace.h、DiffMemTrace.h：没有任何生产者）和 TLB trace 环（record_trace、trace_watch_vaddr、ITLB focus 打印）。ROB 死锁 Assert 保留。-1.15k 行。Gate 60/60，大程序相同。
- 第 2 步（patch 0035）：LSU 的值通路已经是死的：STLF 的 `extract_data(stq data)`、dcache load 响应数据、dcache store 端口的 `align_store_data`、`LoadResp.data` 以及两个辅助函数都已移除；STQ 的 `data` 保留，因为 MMIO store（外设模型）和 MMIO load patch 要用它，`LdqEntry.result` 保留用于 page-fault tval 和 MMIO。Cycle 在 gate 60/60、smc、dhrystone、coremark x1/3/10 上不变；TB_POISON 运行仍然通过。
- 未完成（下一个候选）：dcache 的 line 数据链仍然存在，但实际上已死：StoreReq.data -> dcache store slot / bank-retry 数据 -> MSHR merged_store_data + fill_write.data 合并（apply_strobe）-> WriteBuffer 项数据 -> AXI 写 wdata。现在没有任何东西再读取这些值（cache 数据阵列和 LLC 数据已在 P3 中移除），因此它们可以在一次由编译器引导的扫荡中全部去掉。
- 本轮之后模型总计：43,437 行。

## dcache line 数据链被移除

- 移除了 LSU 以下所有携带 cache-line / store 值的载体：StoreReq.data、dcache store slot 和 bank-retry 数据、MergeReq/MSHRReq 的 data+strb、FillReqMerge、MSHR 的 merged_store_data/strb 与 axi_resp_hold_data 以及 MSHR fill 数据（`MSHR_FILLReq.data`、`FILLWrite.data`、`MshrAxiIn.resp_data`）、DirtyInfo/FillIn 的 data_snap（从 pmem 读取 victim line）、DcacheLineReadResp.data、BypassResp.data、WriteBufferEntry.data 和 WbAxiOut.req_wdata、DcacheRespMeta 的 data/strb、`apply_strobe`、`write_dcache_line` 的 data 参数，以及 rv_simu 中把 R 数据拷入 MSHR、把 WB 数据拷入 AXI 写端口的拷贝。保留的有：tag/valid/dirty/PLRU、store strobe（字节重叠检查）、`merged_store_dirty`、merge/bypass 的 valid + busy 握手，以及外设（MMIO）写数据。
- 精确性：gate 60/60（56 + 4 个分页），smc 8,996 / 8,866，dhrystone 3,911,612，coremark x1/3/10 相同；memory-stress（不在 gate 中，约 1.5M cycle）1,541,195 cycle = baseline 模型的 1,541,195（针对这次改动在两边各运行一次）。模型总计 43,293 行。Patch：0036-line-data-chain.patch。

## memory-stress-small 加入 gate（61 个测试）

- tests/extra/memory-stress-small.c：上游 memory-stress 的 ROUNDS 16 -> 2（同样是 3 x 64 KiB 数组，所以有同样的 L1D 冲突 / dirty 写回 / MSHR 压力），期望值在宿主上重新计算。Baseline 模型和 trace 模型都是 189,128 cycle（1,081,439 条指令，147,460 个 load，114,688 个 store）；trace 模型耗时 3 s。gate 的匹配模式现在只排除大的 `memory-stress/` 目录。baseline.csv 增加了新的一行；gate = 61 个测试，全部通过。
- 对照实验：gate 比较的是 cycle，所以这个测试的灵敏度取决于它的时序。TB_PERTURB_ADDR=3（每第 3 个 load/store 地址翻转 bit 12）给出 189,167 cycle 而不是 189,128，也就是说 dcache/内存时序是被观察到的（每 50 个一次则太弱：不变）。TB_NEG_NO_MEMWRITE 不会使这个测试失败：数据值来自功能核心，commit 时的 pmem 写入只供给模型自己的取指/PTW 读取，所以坏掉的 store 通路会表现为 cycle 变化，而不是校验和错误。

## 模型对于代码、页表或 line 数据不再有自己的内存

- 新增 `tbhook::mem_read(paddr)`（Bridge::read_word）：核心所看到的功能字（RAM 通过 refcpu_load_word，boot ROM 通过核心的 io words）。指令字（ICache glue、MemRouteBlock 的 icache 响应）和页表项（MemRouteBlock 的 PTW 事件）现在从核心的内存而不是 pmem 读取。Gate 61/61，包括 smc-div/addi（8,996 / 8,866）和分页测试：没有 cycle 变化，所以先前的担心（核心内存比 commit "超前"）在这些测试中没有出现；只有当一个 PTE/指令 store 在被读取时仍在飞行中，才可能有影响，而在没有 sfence.vma / fence.i 的情况下，这在架构上本来就是无序的。
- commit 时依据 facts 的 `pmem_write` 已删除；不再有任何东西从 pmem 读取 RAM。AXI/LLC 读数据（`extract_line_response`）、LLC 数据行和 ICache 的 line 字（`rd_data`、`data_resp_line`、`mem_resp_data*`、`lookup_hit_data_w`）此前是零填充的，现已移除；ICache 模块现在报告 `ifu_resp_paddr`，由 glue 根据 `mem_read` 构建取指组。
- 检查：gate 61/61，smc 8,996 / 8,866，dhrystone 3,911,612，coremark x1/3/10 相同，memory-stress 1,541,195 相同；timer-mmio user 时间 2.02 s。模型 43,235 行。Patch：0037-core-memory-reads.patch（2 个 commit）。
- 仍在使用 pmem 的部分：程序加载器（镜像 -> pmem -> 启动时拷入核心，`refcpu_sync_ram_from_dut`）、外设 / MMIO 设备寄存器（它们位于 pmem 的 IO 窗口）、BackTop 中的 boot-ROM 桩和 UART 初始化写。SD 卡 DMA 写只落入 pmem（过去负责转发它们的 difftest 同步在 trace 模式下本来就已关闭）：对读取 DMA 数据的工作负载是一个缺口，gate 中没有测试这样做。

## 单一 RAM：pmem 就是核心的内存

- `pmem_init()` 创建核心（`refcpu_init`），而 `p_memory` / SDRAM 指针就是核心自己的数组；因此对 RAM 地址的 `pmem_read/write/memcpy` 直接作用于核心内存。加载器（`cpu.back.load_image` -> `pmem_memcpy_to_ram`）和 SD 卡 DMA 写（`pmem_write`）直接落入核心，暂存拷贝（前 1 MiB 的 `refcpu_sync_ram_from_dut`）已去掉，`tbhook::ensure()` 从 `pmem_core_ctx()` 取得核心。只有 IO 窗口（设备寄存器）仍是模型侧的状态（`g_io_words`）；boot ROM 字像以前一样被写入核心的 io words。这同时也弥补了上一步所提到的 DMA 缺口。
- 当开启注入时，RUN 模式不再创建 difftest 参考核心（跳过 `difftest_init_run_state`）：它是另一份从未被使用的完整 RAM + SDRAM。
- 效果：一个很小的测试的常驻内存 4.8 GB -> 0.54 GB，`add` 的启动时间 0.24 s -> 0.04 s；timer-mmio user 时间 2.01 -> 1.90 s。Gate 61/61，smc 8,996 / 8,866，dhrystone 3,911,612，coremark x1/3/10 和 memory-stress 1,541,195 不变。Patch：0038-single-ram.patch。
- 未触及：CKPT / REF_ONLY / fast-forward 模式仍像以前一样使用 difftest 参考核心（gate 没有覆盖它们）。

## 功能内存遗留物被移除

- SimDDR 不再读写 pmem（`do_memory_read/write` 和 `axi_kit_ddr_functional_writes` 被删除；它的读数据为零），`PeripheralModel` / `MemSubsystem` 失去了 `memory` RAM 指针（它只是一个空指针保护），AXI 读 payload 是一个字（`kWideReadKeptWords` 16 -> 1：只有 MMIO / 外设读值会传递；boot-ROM 字现在来自 `tbhook::mem_read`）。写 payload 本来就已经是一个字。Gate 61/61，smc、dhrystone、coremark x1/3/10、memory-stress 不变。Patch：0039-memory-leftovers.patch。
- 剩下的看起来像内存但不是 RAM 的东西：设备寄存器（IO 窗口、`g_io_words`，由 PeripheralModel / MMIO 总线的遗留 backing line 使用）、STQ 的 `data` / LDQ 的 `result`（MMIO store 数据和 MMIO load patch），以及 LLC 的逐 line 字节管线（`AXI_LLC_Bytes_t` 行、`refill_data`、`apply write to line`：全都携带零）。最后一项贯穿在 AXI_LLC.cpp（3.3k 行）之中，随 LLC 重写一起消失，不会更早。

## AXI/LLC line 数据管线被移除（patch 0042）

LLC 的 line 字节辅助函数、表数据 payload 赋值、`victim_data` / `victim_wb_data_r` 载体被移除
（MMIO bypass 的 1 字数据保留）。时序信号（表 enable/write/index/way）保留。Kit 的 axi_interconnect
现在约 18.1k 行（含头文件，cpp+h）。Gate 61/61；smc-div 8,996，smc-addi 8,866，dhrystone 3,911,612，
coremark10 976,652，memory-stress 1,541,195 —— 全部与 baseline 相同。

## AXI/LLC/DDR 由覆盖率驱动的裁剪（patch 0043）

方法：用 `-fprofile-instr-generate -fcoverage-mapping` 构建（BUILD_DIR=build_cov），运行 66 个 gate/vm/dhrystone/coremark3
程序，用 `llvm-cov export` 导出 JSON，然后 (1) 删除计数为零且无人引用的函数（根据未声明/未定义符号错误迭代），(2) 清空计数为零的 `{...}` 块（区域起点 `{`，终点 `}`），(3) 重新运行覆盖率，
删除因 (2) 而成为孤儿的函数，(4) 移除 `if (pure-cond) {}` 遗留物（含调用的条件保留：移除有副作用的
`if` 会让全部 61 个测试挂掉）。结果：AXI_Interconnect.cpp 3406->~2.3k，AXI_LLC.cpp 2588->~1.5k，SimDDR
~416，route_shape.h 757->364；kit 源码+头文件约 5.9k 行（原为约 10.9k）。Gate 61/61，smc/dhrystone/coremark10/memory-stress
与 baseline 相同；timer-mmio 1.83 s user（不变）。注意：裁剪的安全性仅取决于测试覆盖；这 66 个程序
没有执行到的功能（UART MMIO 路径、prefetch、invalidate、mapped window）已被删除/清空。

## BPU 重写 + 差分测试（patch 0044）

基于覆盖率的裁剪被否决了（它只能说"这些程序没有执行到"），因此改为重写 BPU，
并用差分测试证明其等价。
- 分析：TAGE/BTB/TypePredictor 在实践中是单周期的。S_STAGE2 状态、SRAM 延迟 PRNG、输入/流水线锁存、tage_done/btb_done、SC/loop 预测器（输出被硬接为 0）、path history 以及 256 项的 GHR 拷贝，对可观察行为而言都是死的。有意保留的怪癖：BTB 拿到的是 bank 内的 pc（所以它的回退 target 是 bank_pc+4），TAGE/BTB 从队列头更新，而 type predictor 从 commit 流更新，更新队列会丢弃（每个 bank 每 cycle 一次 push，update_queue_full 恒为 0），复位不触碰 TAGE/BTB 阵列。
- 新的 front-end/BPU/BPU.h（615 行，命名空间 bpuf）：Tage/Btb/TypePred（predict/update -> 写集合，commit）、Hist
  （128 位 GHR + 折叠历史）、Ras、Bpu::comb()（纯函数，返回 Txn）/ commit() / reset()。BPU 目录 4,754 -> 764 行。
- tests/bpu_ref = 冻结的旧 RTL 风格 BPU_TOP；tests/bpu_diff.cpp 用相同的结构化随机激励驱动两者（合成的分支池、由先前预测重放的真实元数据、罕见的 reset/refetch），并在每个 cycle 对每个输出做 memcmp；scripts/bpu_diff.sh <sim> [cycles] [seeds]。6 种配置（池 100..30000 个 pc，跨度 1k..64k 字）共 30M cycle：0 次不一致。
- Harness 有效性：对新代码做 18 处故意变异；捕获 16 处，2 个幸存者是等价/不可观察的（type conf 只
  供给一个未使用的输出；一个 8 位 BHT 值 >>18 与 >>19 都是 0）。
- Gate 61/61；smc/dhrystone/coremark10/memory-stress 与 baseline 相同；timer-mmio 1.83 -> 1.35 s user（-26%）。

## Lsu 重写 + lockstep 验证（patch 0045）

RealLsu.cpp/.h 1,617 -> 971 行，外部可见行为相同（所有 phase，BackTop 的调用顺序相同）。
- 六个手写的索引队列（wait_mmu_ldq/stq、mmu_done_stq、finish、stlf_queue、wait_dcache_ldq）加上约 14 个
  入队/出队/trim 辅助函数变成了一个 `Ring<E,N>` 模板（push/pop/trim/clear/window）。空洞（位于有效队头之后、已被消费的项）被有意保留：它们占用窗口 slot 并改变时序。未使用的 load 状态、FenceUnit、byte_mask、
  stq_wait、alloc_* 包装、仅用于诊断的代码被移除；枚举值保持稳定。
- 验证用的是 lockstep 影子，而不是覆盖率：`LSU_SHADOW=1` 时，构建会链接 tests/lsu_ref（冻结的原始 LSU，
  重命名为 RealLsuRef），BackTop 的 `LSU(phase)` 宏在两者上以相同输入运行每个 phase，比较该 phase 的
  输出结构体和一个规范化的状态摘要（所有 LDQ/STQ 字段、每个队列包括环的 head/count、uncached/LR-SC 单元、
  req_gen）。scripts/lsu_shadow.sh 构建它并运行 gate。旧对旧给出 61/61（无误报）；新对旧：gate 61/61，
  dhrystone、coremark3、memory-stress(+small)、smc-*：无不一致，每个 phase 的摘要都相等。
- 对验证器的变异检查（在新代码中植入 21 个故意的 bug）：捕获 17 个（包括用于 TLB retry 的 vm 分页测试），1 个等价
  （`x-- > 1` == `--x > 0`），3 个幸存是因为没有程序能到达该路径（超过 LSU_LDU_COUNT=4 的 MSHR_FULL 唤醒、被重叠 load 挡住的 SC、flush 时仍在总线上的 MMIO store）。为它们构造的裸机测试始终没有到达（探测：最多 4 个 MSHR_FULL
  等待者；SC-hold 和 flush-with-MMIO-on-bus 计数器保持为 0），因此 LsuShadow.cpp 里有定向的状态注入场景
  （LSU_SHADOW_DIRECTED=1）：相同的手工构建状态被送入新 LSU 和参考 LSU，在两者上运行一个 phase，摘要
  必须匹配。全部 5 个场景匹配，3 个幸存变异现在都被捕获（21 个中捕获 20 个，1 个等价）。
- Gate 61/61，dhrystone/coremark10/memory-stress/smc 与 baseline 相同；速度不变（A/B timer-mmio 1.34 vs 1.34 s）。

## 后端提速一轮（patch 0046）

Profile（dhrystone，`sample`，约 10k 个采样，self time）：后端约 48%（IssueQueue 9.4、Dispatch 6.9、Exu 6.6、Rob 6.1、Isu 5.2、Lsu 3.4、Fu 3.0 ...），
memmove+memset 约 13%，分散在许多调用者中（SimCpu::cycle、front_comb_calc、FIFO seq、Ren）。
- Rob：comb_begin/seq 每个 cycle 把整个 128 项数组拷贝两次；现在用一个 dirty 位图（在每个 entry_1 写入点调用 touch()，
  flush 时调用 touch_all()），只拷回被写过的 slot。Rob 占采样 6.1% -> 1.6%；timer-mmio 1.33 -> 1.27 s，coremark3 2.64 -> 2.56 s。
- IssueQueue：ready 位图在 seq() 中为 dirty slot 刷新；schedule() 遍历置位的位，而不是扫描所有 slot（顺序相同）。约快 0.5%。
- 每次改动之后 Gate 61/61，dhrystone/coremark10/memory-stress/smc 与 baseline 相同。
- 剩下的是零散的开销：IssueQueue::comb_issue（3.9%）、AXI copy_regs_live（3.5%+1%，每个 tick 多次批量拷贝 LLC 寄存器）、
  Exu::comb_begin（2.6%，逐 FU slot 拷贝）、Dispatch::is_preg_woken（2%）、每 cycle 1-5 KB 的结构体拷贝（FrontUpdateRequest 4.6 KB、
  FrontPreIO 0.9 KB、LsuState 8.6 KB cur=nxt）——每一项值 1-2%。
- LLC（提速工作的第 2 轮）：AXI_LLC::comb 中的 `io.table_out = {}` 每次调用都会释放/分配 8 个 vector；改为保留存储的 clear()
  （状态与 `= {}` 相同），并且 MemSubsystem 不再为每次 LLC 查找填充一个 1 KiB 的零数据行（只读取 `data_valid`）。
  timer-mmio 1.26 -> 1.24 s，coremark3 2.56 -> 2.49 s。后端提速工作的累计：timer-mmio 1.33 -> 1.24 s（-7%），
  coremark3 2.64 -> 2.49 s（-6%）。未完成：Dispatch::is_preg_woken 的 wake 位图（需要 prf/iss wake 输入的精确 comb 顺序）、
  Exu::comb_begin 逐 FU 拷贝、每 cycle 1-5 KB 的前端/后端结构体拷贝（每项 <=1-2%）。

## comb_begin 拷贝被移除（patch 0047）

Idu/Ren/Dispatch/Isu/Prf/Exu/Csr::comb_begin 中的每一个 `X_1 = X` 拷贝都是冗余的：seq() 会把 X_1 拷回 X，所以两者在下一次
begin 时是相等的。back-end/include/BeginCheck.h 把每个拷贝变成 `BEGIN_EQ(X_1, X)`，它平时是空操作，在
`-DBEGIN_SELFCHECK` 下则是 memcmp+abort。带 self-check 的构建运行了 gate 61/61、dhrystone、coremark10、memory-stress、smc 和 AXI_IDLE_VERIFY，没有违例，因此
这些拷贝被删除。速度影响约 0（timer-mmio 1.24 -> 1.23 s）：这些结构体很小；这是简化，而不是提速。
未触及：PreIduQueue（ftq 头）、逐 FU 的 `units[i]->comb_begin()` 以及 Exu::comb_begin 中的输出默认值清零（这些是必需的）。

## 更正：覆盖率驱动的清空被回退（kit commit c707481、237bdf6、652645e）

0043 这一轮清空了每一个没有测试执行到的块。这 66 个程序从不会从 8 MiB 的 LLC 中驱逐一条 dirty line，所以这也掏空了
DDR 写路径（AW/W/B 处理）、victim 写回处理等等。在 64 KiB LLC（`-DCONFIG_AXI_LLC_SIZE_BYTES=65536`）下，被清空的
kit 永远无法完成 memory-stress-small（挂死），而回退后的 kit 给出 189,639 cycle = 用同样方式构建的原始模拟器
（在 dcache-conflict-writeback 868 和 dcache-mshr-pressure 193 上也相同）。回退前后 gate/大程序的数字是相同的，
这正是 gate 看不出问题的原因。0043 中保留下来的：删除无人引用的函数
（2bbb1a3）以及不分配内存的 `table_out` 复位。教训：未被执行的分支不等于死分支。scripts/llc_small_check.sh 用小 LLC 构建两个
模拟器并比较 cycle；AXI 重写将对照这个完整的 kit 验证，而不是被清空的那个。

## SimDDR 重写（patch 0048；对照完整 kit 验证，而不是被清空的那个）

SimDDR.cpp+.h 771 -> 257 行，行为完全相同。写数据 FIFO 只保留 beat 计数（没有 addr/data），WriteTransaction/ReadTransaction 去掉了
未使用的地址字段，三个都选择 Read 的 Read 仲裁分支合并为一个，w_active/w_current（与队列冗余）被移除。
- 独立的差分测试（tests/ddr_diff.cpp、scripts/ddr_diff.sh）：随机 AXI master（协议正确的 W burst、有界的 outstanding 写、让写占据 backend 的读暂停窗口）驱动新的 SimDDR 和冻结的原版（tests/ddr_ref）；每个 cycle 比较每个输出和一个状态摘要。
  11 种配置 x 1.5M cycle（数十万个读写 beat，10^5 B 的响应）：0 次不一致。8 个故意变异（drain keep、turnaround、
  B aging、WREADY 门控、retire 条件、round-robin、ARREADY、drain-enter 水位）：全部被捕获。
- 模拟器内的 lockstep（`gmake DDR_SHADOW=1`，tests/ddr_ref/DdrShadow.*）：gate 61/61 和小 LLC 运行都匹配，无不一致。仅靠集成系统无法到达若干 DDR 路径
  （outstanding 32、写水位、turnaround），所以独立测试才是真正的覆盖。
- Gate 61/61，大程序与 baseline 相同；scripts/llc_small_check.sh（64 KiB LLC，真实 DDR 写）与原始模拟器相同。
- 速度：回退清空之后 timer-mmio 1.23 -> 1.29 s（被恢复的分支又会运行了）；DDR 本身只占很小一部分。

## 后端整理，第 1 轮（patch 0049，cycle 精确，无行为变化）

- 移除了恒等的 `Uop` 转换函数（`from_*`、`to_*`）和 14 个各流水级专用的别名（`DecRenInst`、`DisIssUop`、`ExuRobUop`，...）：每个流水级
  传递的都是同一个 `Uop`。冻结的 Lsu 参考被机械地更新；lockstep gate 仍然通过。
- `BackTop::comb()` 现在是七个具名流水级加 fire/recover 级的简短列表，每个都是一个小方法，并有一条注释说明它
  产生什么；调用顺序不变，就是依赖顺序。
- Gate 61/61，dhrystone/coremark10/memory-stress/smc 与 baseline 相同。

## 后端整理，第 2 轮：Wires（patch 0050，cycle 精确）

后端所有模块间信号（46 个 IO 对象加顶层的 in/out）现在放在一个 `struct Wires`（back-end/include/Wires.h）中；`BackTop` 继承自它，
每个模块都通过 `wires->name.field` 读写它们。消失的有：BackTop::init 中约 100 行 `module->in.x = &x` / `->out.x = &x`、十个各模块
只含指针的 `In`/`Out` 结构体及其成员；访问由脚本改写（`in.x->f` -> `wires->x.f`），并由编译器检查；唯一一个由
某个模块拥有的 wire（Idu 的分支锁存）在 Wires 中是一个指针。Lsu lockstep 参考和定向场景已适配（它们自己构建一个 Wires）；lockstep gate
61/61 和 5 个定向场景仍然匹配。后端 11.0k -> 10.9k 行（节省的更多是间接层次而不是行数）。Gate 61/61，dhrystone/coremark10/
memory-stress/smc 与 baseline 相同。

## Rob 重写 + lockstep（patch 0051）

Rob.cpp 761 -> 364 行，phase 相同，成员相同（entry/entry_1/指针、dirty-slot 同步），行为相同。
- commit 仲裁被重构：队头 line 扫描合并为一个循环，恒为真的 `group_commit_mask`/`head_line_empty` 条件被去掉（mask 等于
  valid 位），单条 commit 的决策分为三个短步骤，特殊指令广播改为一条 if/else 链，"如果 line 已排空则推进 head" 做成一个 lambda，
  由 group 路径和 single 路径共用；comb_complete 使用端口范围而不是逐端口循环；约 110 行调试 dump（DUP-CPLT、deadlock、DBG_PRINTF）变成了带
  简短消息的 Assert。
- 验证：`gmake ROB_SHADOW=1` 链接 tests/rob_ref（冻结的原始 Rob，trace-bridge 调用被禁用，因此不会重复上报），BackTop 的 `ROB(phase)` 宏在两者上运行
  每个 phase，各自使用自己的 Wires（每个 phase 之前把输入从真实的那份拷贝过来，比较输出，外加指针和每个有效 entry 的摘要）。
  旧对旧 61/61（rob_commit 只在 comb_commit 中比较，因为后面的流水级会就地修补 diag_val）。新对旧：gate 61/61，dhrystone、coremark3、
  memory-stress(+small)、smc：无不一致。
- 变异检查（14 个故意的 bug）：10 个被程序捕获，另外 2 个（flush_pipe 累积、STA store-fault 清除 load fault）被定向场景捕获
  （`ROB_SHADOW_DIRECTED=1`），1 个在此处不可观察（`front_stall` 恒为 0，因为 BPU 更新队列从不报告满），1 个被 gate 自身捕获（EBREAK 退出）；
  没有任何程序或场景执行到的：队头是 fence 而一个已 commit 的 store 仍在排空（fence 门控）。
- Gate 61/61，dhrystone/coremark10/memory-stress/smc 与 baseline 相同。

## Dispatch 重写 + lockstep（patch 0052）

Dispatch.cpp 712 -> 372 行，phase/成员/行为相同。
- `decompose_inst`（约 240 行逐指令的 case 块）变成了一张小表：每个 uop 是 {queue, op, src1/src2/dest enable 的覆盖项, imm=4, atomic}；唯一一个依赖数据的
  情况（非 SWAP AMO 的 store 数据读取 load 的目的寄存器）在循环之后修补。comb_fire 失去了死的"阻塞原因"记账（从未被读取）；
  CSR/atomic 串行化检查是一个条件；回滚循环和端口所有者循环更短。
- 验证：`gmake DIS_SHADOW=1` 链接 tests/dis_ref（冻结的原版），BackTop 的 `DIS(phase)` 宏在两者上运行每个 phase，使用各自独立的 Wires；比较输出（dis2ren/iss/rob/lsu）、
  已寄存状态与 working 状态的摘要，以及真实 Dispatch 为已发射 uop 写入的共享 uop-pool slot（参考版会覆盖它们）。
  旧对旧 61/61；新对旧：gate 61/61，dhrystone、coremark3、memory-stress(+small)、smc：无不一致。
- 变异检查（15 个故意的 bug）：10 个被程序捕获，另外 2 个被定向场景捕获（`DIS_SHADOW_DIRECTED=1`：写 x0 的较老 slot 不得让某个 source 保持 busy；满的 issue
  queue 会阻止该组），3 个是等价的（LDQ 限制 `<=` 被端口 valid 检查所遮蔽，JALR 跳转的 src1 本来就已使能，LSU 的 br_mask 已在 comb_alloc 中被清除）。
- Gate 61/61，dhrystone/coremark10/memory-stress/smc 与 baseline 相同。

## Exu 重写 + lockstep（patch 0053）

Exu.cpp 577 -> 306 行（ExuIn/ExuOut 指针结构体消失：FU 请求/响应总线是普通成员），phase 和行为相同。
- FU 规格表、`comb_begin` 默认值、逐端口 ready mask、CSR 请求提取、流水线 slot 填充、四个执行子阶段以及结果收集，
  是同样的逻辑，只是去掉了冗余的脚手架：`fu_global_idx`（= 循环下标）、重述不变量的逐阶段 assert、winner 指针记账、重复的
  flush/kill 条件（一个 `dead` 标志）、两个复制粘贴的 LSU 写回块（一个 lambda）、作为一个条件的误预测仲裁。
- 验证：`gmake EXU_SHADOW=1` 链接 tests/exu_ref（冻结的原版，带自己的 FU；它的 trace-bridge exec 观察被禁用，因此不会重复上报），BackTop 的
  `EXU(phase)` 宏在两者上运行每个 phase，使用各自独立的 Wires；比较六条输出总线（exe2csr/prf/iss/lsu、exu2id、exu2rob）以及 FU 输入 slot 和 FU 请求/响应
  总线的摘要。旧对旧 61/61；新对旧：gate 61/61，dhrystone、coremark3、memory-stress(+small)、smc：无不一致。
- 变异检查（15 个故意的 bug）：捕获 12 个；3 个幸存：用 `== br_mask` 的 `is_br_killed`（在广播 mask 为 one-hot 时等价）、从
  端口 ready mask 中去掉 `|| fu_ready`，以及被 squash 的分支仍然驱动恢复（需要一个分支在其被 flush 的同一 cycle 完成）。这些路径是逐行转写的，没有经过动态证明。
- Gate 61/61，dhrystone/coremark10/memory-stress/smc 与 baseline 相同。

## Isu 重写 + lockstep（patch 0054）

Isu.cpp 525 -> 247 行，phase/成员/行为相同。
- 三个延迟唤醒结构（MUL 流水线、DIV 和 FP slot）通过共享辅助函数处理：`clear_wake_state()`、一个 `for_each_wake_slot_1` 访问器（用于误预测 kill 和
  分支位清除）、一个用于 DIV/FP 倒计时的 `age` lambda、以及一对用于分配的 `put`/`free_slot`；`comb_awake` 的唤醒通道通过一个 `drive` lambda 驱动。
  slot 放置保持在 `Assert` 之外（它必须在 NDEBUG 构建中运行）。
- 验证：`gmake ISU_SHADOW=1` 链接 tests/isu_ref（冻结的原版，带自己的 issue queue），BackTop 的 `ISU(phase)` 宏在两者上运行每个 phase，使用各自独立的 Wires；比较 iss2dis/iss2prf/iss_awake
  以及每个队列（已提交和 working 的 entry、计数）加唤醒状态的摘要。旧对旧 61/61；新对旧：gate 61/61，dhrystone、coremark3、memory-stress(+small)、smc：无不一致。
- 变异检查（15 个故意的 bug）：8 个被程序捕获，另外 2 个被定向场景捕获（`ISU_SHADOW_DIRECTED=1`：flush 会清除待处理的 DIV 唤醒；没有目的寄存器的 DIV 不启动任何唤醒），5 个幸存，并且在此配置下看起来
  是等价的（MUL 流水线深度为 1，所以移位和最后一级的变异体什么也不改变；STA 端口不是 ALU 端口；Dispatch 已经对进入的 uop 应用了唤醒）。
- Gate 61/61，dhrystone/coremark10/memory-stress/smc 与 baseline 相同。

## Idu 重写 + lockstep（patch 0055）
Idu.cpp 505 -> 302 行：`decode()` 对每个 opcode 使用一个 `set(dest,s1,s2,type)` 辅助函数和一个 CSR 已知列表函数；comb_decode/comb_fire 被压缩。参考在 tests/idu_ref（IduRef + IduShadow，`IDU_SHADOW=1`），比较 dec2ren、dec_bcast、idu_consume、已解码的 uop-pool slot 以及 now_br_mask / br_mask_cp / tag_vec / br_latch 的摘要。
- 验证：lockstep 下 gate 61/61 无不一致；smc-div、smc-addi、dhrystone、coremark3、memory-stress(-small) lockstep 干净；普通二进制：coremark10 976652，dhrystone 3911612，memory-stress 1541195，smc 8866/8996。
- 变异检查（16 个变异体）：lockstep 捕获 9 个（tag 清除、flush 复位、mispred 释放、checkpoint 清除、LR src2、DIV/MUL、FENCE.I、bcast mask、LUI src1）。6 个 decode 幸存者（CSR 列表中的 sip、time/timeh 非法、AMO rd=0 -> areg 32、fp unary 0x70、is_ret imm、sfence rd）是没有任何测试程序包含的指令形式；`tests/idu_diff.cpp` 在所有 opcode x funct3 x funct7 x 寄存器模式、全部 4096 个 CSR 编号以及 20M 个随机字上比较新旧 `decode()`：0 次不一致，并且捕获了全部 6 个变异体。
- Tag 耗尽停顿：在每 cycle 8 个 tag 的情况下无法到达，因此两份副本都被修改为每 cycle 最多分配 2 个 tag（此时会发生停顿，quick-sort 等）；lockstep 干净。`page_fault_inst=false` 变异体是等价的（u = {} 已经将其清除）。

## Ren 重写 + lockstep（patch 0056）
Ren.cpp 400 -> 220 行：单循环 free-list 分配并按序停顿，用一个 `lookup()` lambda 处理 src1/src2/old-dest 以及同组 bypass（取代九个逐 slot 数组），提交统计放在 `count_commit()` 中，用 memcpy 做 checkpoint，comb_pipeline 精简。参考在 tests/ren_ref（RenRef，其 commit 副作用——perf 计数器、commit/interrupt hook、日志——已被移除以免运行两次；RenShadow）。每个 phase 比较 ren2dis（valid、uop、共享 uop-pool slot）、ren2dec 以及完整状态（inst_r、RAT、checkpoint、free list、指针），包括 working 和 committed。
- 验证：lockstep 下 gate 61/61，smc/dhrystone/coremark3/memory-stress(-small) lockstep 干净；普通二进制的 cycle 数不变（coremark10 976652，dhrystone 3911612，memory-stress 1541195，smc 8866/8996）。
- 变异检查（19 个变异体）：捕获 13 个。幸存者：分配失败后的 `stall` 和来自 x0 的 bypass（等价：decoder 会为 x0 清除 dest_en）、从 `_1` 副本读取 checkpoint（等价：一个 tag 在使用它的同一 cycle 内不会被再次 checkpoint）、ready 路径中的 clear_mask（冗余：Idu 已将其清除），以及两条没有测试到达的路径（已通过对全部 66 个测试 + 大程序的探测验证）：commit 与误预测同时发生时丢弃回收（`free_tail_1 = free_tail`），以及带 dest_en 的异常指令（旧的 dest 未被释放）。这两处都逐语句保留了原版的写法。

## Csr 重写 + lockstep（patch 0057）
Csr.cpp 420 -> 222 行：中断评估使用两个小 lambda（to_m / to_s），表驱动的中断注入，comb_exception 中有具名的委托标志和一个 `trap_target` lambda，mie/mip/mstatus 别名使用一个 `write_pair` 辅助函数。参考在 tests/csr_ref（CsrRef 去掉了 satp trace 记录，CsrShadow：每个 phase 比较输出 csr_status/csr2exe/csr2rob/csr2front 和完整状态；机器状态在第一个 phase 采纳一次，因为它也会从外部加载）。
- 验证：lockstep 下 gate 61/61，smc/dhrystone/coremark3/memory-stress lockstep 干净，普通 cycle 数不变。
- 测试只到达很少的 trap/S-mode 路径，因此 `tests/csr_diff.cpp` 用随机的机器状态（稠密位字、所有特权级）和随机事件（ecall、fault、illegal、mret/sret、中断响应与注入、各种 CSR 写）驱动新旧两者 20M 个 cycle：0 次不一致。28 个变异体中有 27 个被它捕获（幸存者在 comb_csr_read 中读取 `_1`，而此时它等于已提交值：等价）。

## PreIduQueue 重写 + lockstep（patch 0058）
PreIduQueue.cpp 351 -> 245 行：移除了空的 `if` 体和未使用的 ctx hook，FTQ 辅助函数被压缩，共享 PC 查找路径，commit-info 元数据通过局部引用拷贝，`ftq_alloc() < 0` 分支（可证明是死的：ready 已经要求有空闲 slot）变成了 Assert。参考在 tests/pre_ref（PreIduQueueRef，其 uop 分配是桩，PreShadow 作为活动类的 `friend` 以访问私有状态）：每个 phase 比较 pre_issue（uop id 被屏蔽）、pre2front、FTQ 的 PC/commit-info 响应、指令缓冲和 FTQ 状态。
- 验证：lockstep 下 gate 61/61，大程序 lockstep 干净，普通 cycle 数不变。
- lockstep 子集上的 26 个变异体：6 个幸存（page_fault_inst、next_pc、sc_idx 偏移、rob 查找 entry_valid、FTQ-full ready、元数据游标失效），因为程序从不产生这些情形。`tests/pre_diff.cpp` 用随机的块、消费、flush/误预测、commit 和查找（数百万 cycle，FTQ/缓冲区满和空的情形）驱动活动版本与参考：干净，并且捕获了全部六个。另有两个变异体是等价的（head 指针空操作、死的 alloc 分支）。

## BackTop 整理（patch 0059）
BackTop.cpp 850 -> 366 行。镜像/flash/SD 加载和 checkpoint 恢复移到 BackTopLoad.cpp（422 行，逻辑不变）；九个 `<MOD>_SHADOW` 宏块移到 include/ShadowHooks.h；`restore_from_ref` 和 `restore_checkpoint` 共享 `install_arch_state()`；诊断 IO 清零以及输出/重定向/commit 组的代码变成 `clear_stage_ios()` 和 `comb_outputs()`。BackTop 是胶水，所以没有 lockstep：通过 cycle 相等来验证（gate 61/61，coremark10/dhrystone/memory-stress/smc 不变），并通过一次同时开启全部九个 shadow 的构建验证，61/61 无不一致（DEADLINE=300，很慢）。Checkpoint 恢复本身不被 gate 覆盖（除了共享的辅助函数之外，已编译且未改变）。

## LLC 重写 + 随机差分（patch 0060，kit 仓库 commit 67a6b81）
AXI_LLC.cpp 2538 -> 1467 行（头文件的 private 部分被重新组织）。三个巨型函数按情形拆分：`try_complete_lookup` -> `complete_direct_lookup / complete_hit_lookup / complete_write_miss / complete_read_miss`；写路径 -> `on_mem_write_resp`、`try_commit_write_after_victim`；refill commit 共享 `externalize_read_victim` / `release_mshr`。重复的块变成了辅助函数：`start_lookup`、`end_lookup`、`retry_lookup`、`write_line_tables`、`put_meta / put_repl / begin_write`、`promote_read_response`、`arm_victim_writeback`、`any_read_victim`。移除的有：debug-focus 宏和 trace hook（全部被编译掉）、空的 perf-count 函数、空的 `if` 体，以及被 `pick_new_read_master` / `pick_new_write_master` 变得冗余的检查（相同的寄存器集合，相同的条件）。
行为说明：`end_lookup` 清除所有 lookup 标志，原版在某些路径中只清除其中一个子集；这些标志只在 `lookup_valid_r` 时才被读取，并且每个 `start_lookup` 都会重写全部标志，所以没有任何可观察的变化（差分对无效 lookup 会屏蔽它们）。
- LLC 是一个独立的状态机（上游端口、内存端口、表命令/应答），所以验证是闭环差分，`tests/llc_diff.cpp`（`scripts/llc_diff.sh`）：活动类和冻结的原版（`tests/llc_ref`）各自运行同一个随机模拟（每个 master 的 sticky 请求，包含 bypass / direct-mapped / 整行和部分写，带写响应码的随机 ready/延迟内存模型，表模型，invalidate-all（也在 drain 窗口中）和 invalidate-line，训练 prefetcher 的顺序流，单 MSHR 配置，1-4 路，很小的 set 使冲突和 dirty 驱逐持续发生）。每个 cycle 所有 ext_out 字段、每个表命令以及活动寄存器状态的哈希都必须相同。Harness 对原版源码的覆盖：约 91% 的行。
- 结果：6 个种子共 12M cycle，另加 `CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC=1`（no-allocate 路径）下 6M：0 次不一致。对新代码的变异检查（38 个变异体）：捕获 32 个；6 个幸存，它们要么等价，要么按协议不可达（内存应答一个无效的 MSHR 索引；第二次队列 promote 以及静默检查中的读队列项，队列不变量使其冗余；post-victim commit 的表 busy 保护，在该点从不 busy；hit-write victim 循环中的同 line 跳过，被 lookup 的 MSHR 检查所排除；victim 写回按地址查找 owner 的回退，原样保留自原版）。
- 模拟器：gate 61/61，coremark10 976652，dhrystone 3911612，memory-stress 1541195，smc 8866/8996 不变；小 LLC 检查（64 KiB，dirty 驱逐）：memory-stress-small 189639、dcache-conflict-writeback 868、dcache-mshr-pressure 193、memory-boundary 323 与 baseline 相同。

## Interconnect 整理 + 随机差分（patch 0061，kit 仓库）
AXI_Interconnect.cpp 3373 -> 2771 行。这是一个协议桥，带有大量相互耦合的手写状态，所以是分步验证式地整理，而不是整体重写：(1) 移除了 debug-focus 宏、trace hook、空操作的 `if` 体、读超时监视器（它只碰自己的计数器）以及十一个只写不读的成员（`ar_*_c`、`aw_port_c`、`r_current_master`、`req_drop_warned`；模拟器的 `read_req_drop_warned` 快照也随之去掉）；(2) `seq()`（900 行）被拆分为 `seq_read_ar / seq_llc_dispatch / seq_read_data / seq_llc_write / seq_direct_write / seq_finish`，`seq` 之前的采样放在 `SeqPrev` 中；两个相同的尾部变成 `seq_finish`；(3) 共享辅助函数：`to_llc_read_req / to_llc_write_req`、`start_downstream_write`（当前事务 + AW 锁存设置的两份拷贝）、`issue_master_read`（AR meta 设置的三份拷贝）、`first_free_id`、`mode2_ddr_aligned`；(4) 端口访问器、同 line 冒险、外部冒险、静默谓词、id 冲突检查和待处理队列查找器的紧凑形式。一个冗余项（mode-2 对齐标志里的 `!request_in_mapped_window`，已被 `request_uses_ddr_port` 排除）被去掉。
- 验证：闭环差分 `tests/ic_diff.cpp`（`scripts/ic_diff.sh`）：活动版本和冻结的原版（`tests/ic_ref`）运行同一个随机模拟——四个读 master 和两个写 master，带 sticky 请求（DDR、direct-mapped 窗口、MMIO 含不支持的大小、bypass、部分/整行写），每个下游端口一个随机化的 AXI slave（随机 ready/延迟、多 beat burst、每端口内存镜像），LLC 表，invalidate-all/line 以及运行中途的模式切换（0/1/2，会触发重配置 flush）。每个 cycle 所有上游和下游输出、LLC 表命令、accepted/ready 脉冲、当前模式以及 `idle_quiescent()` 都必须相同。对原版源码的行覆盖约 90%。结果：6 个种子共 9M cycle，0 次不一致。
- 对整理后代码的变异检查（22 个变异体，每个 600k cycle，对幸存者再用 6 个种子）：15 个立即被捕获；幸存者在此处是冗余或不可达的（每个 master 待处理读上限 32 从未达到，已锁存 AW 的 AW id 排除已被活动写覆盖，`w_current` 同粒度检查在 LLC 路径之外被 `w_pending` 覆盖，对同 line 写阶段的 line-invalidate 门控，模式切换期间非 LLC 的静默项）；第一个变异脚本把一个 AW 锁存大小的变异体放错了位置，重新正确运行后在 cycle 166 被捕获。
- 模拟器：gate 61/61，coremark10 976652，dhrystone 3911612，memory-stress 1541195，smc 8866/8996，小 LLC 检查与 baseline 相同。

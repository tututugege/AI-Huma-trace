# trace-model 环境变量

`build/simulator`（和 `build_shadow/simulator`）运行时读取的全部环境变量，共 103 个（模型本体 99 个 + 影子构建 4 个），按用途分组。默认值取自源码，括号里是源文件。带「编译期默认」的变量，默认值来自 `include/config.h` 里同名的 `CFG_*` 宏。

> **和 RTL 比周期必须设 `LSU_PIPE_NOWD=1`。** RTL 的参考周期是关掉 63 周期看门狗标定的；不设时模型会在 load / store 停住 63 周期后重发，`crc32` 这类程序会差几十周期。

写法：`NAME=值 build/simulator prog.bin`。「开关」类变量：未设 = 关；设成 `0` 的行为按各行说明（有的变量设成 `0` 仍算「已设」，用 `unset` 关掉）。

## 1. 运行控制

| 变量 | 默认 | 作用 | 位置 |
|---|---|---|---|
| `SIM_MAX_CYCLES` | 内部上限 | 周期数到了就停（退出码 3），防止不终止的程序卡住 | `main.cpp` |
| `FAST_WARMUP` | 关 | 设 `1`：检查点运行时把（检查点间隔 − 预热）条指令用函数模型快速回放进 cache / TLB / BPU，预热目标改为至多 100 万条提交（EXP-011）；SPEC 全量跑法用它 | `main.cpp` |

## 2. 性能计数（PMU）

| 变量 | 默认 | 作用 | 位置 |
|---|---|---|---|
| `PMU` | 关 | 设为非 `0` 的值打开 PMU / TMA 统计，结束时打印并写 `pmu_*.csv` | `back-end/Pmu.cpp` |
| `PMU_OUT` | `pmu` | 输出文件名前缀 | 同上 |
| `PMU_INTERVAL` | 0（不采样） | 每隔这么多周期记一行时间序列 | 同上 |
| `PMU_PCPROF` | 开 | 设 `0` 关掉按 PC 的统计 | 同上 |
| `FE_COMMIT_TRACE` | 关 | （需 `PMU=1`）每条提交打印 `[CT] pc result diag_val c=<周期>`；误预测 / flush 时打印 `[CT] EV ...`。和 RTL 的 `SOC_TB_COMMIT` 逐提交对周期时只取 `pc` 和 `c=` | `back-end/PmuSample.cpp` |

## 3. 与函数模型对拍、调试（`diff/`）

| 变量 | 默认 | 作用 | 位置 |
|---|---|---|---|
| `TB_CHECK` | 关 | 打开逐条值比较（rd / 分支 / 地址 / 目标），结束时打印 `[TB_CHECK]` 统计（不一致数应为 0） | `diff/trace_hook.cpp` |
| `TB_NO_INJECT` | 关 | 关闭事实注入（P3 之后事实是必须的，只用来调试老版本） | 同上 |
| `TB_POISON` | 关 | 把时序模型不该读的、模型自己算出的值破坏掉，用来证明时序没有偷读这些值 | 同上 |
| `TB_RESYNC_REF` | 关 | 用 P1b 的重同步行为（对照用） | 同上 |
| `TB_TRACE_COMMIT` | 关 | 每条提交打印 `[COMMIT] cyc pc inst rd v`；`rtl/scripts/first_diff.sh` 用它和 RTL 的 `SOC_TB_COMMIT` 找第一处分歧 | 同上 |
| `TB_TRACE_TRAP` | 关 | 打印每个 trap（`[TRAP] M/S cause epc tval target priv`）和注入给函数模型的中断（`[IRQ]`） | `back-end/Exu/Csr.cpp`、`diff/trace_hook.cpp` |
| `TB_SAMPLE_PC` | 关 | 设 N：每 N 条提交打印一次 `[SAMPLE] cyc commits pc inst`，看长程序的时间花在哪里 | `diff/trace_hook.cpp` |
| `TB_PERTURB_BR` | 关 | 阳性对照：每 N 个分支翻转一次分支事实，之后的运行必须和基线不同（验证检查本身有效） | 同上 |
| `TB_PERTURB_ADDR` | 关 | 同上，对象是访存地址 | 同上 |
| `TB_NEG_NO_REWIND` | 关 | 阴性对照：误预测后不回滚事实链，检查必须失败 | 同上 |
| `TB_NEG_NO_MEMWRITE` | 关 | 阴性对照：提交的 store 不写内存，检查必须失败 | 同上 |
| `TB_NEG_NO_TRAP_STATE` | 关 | 阴性对照：trap 后不恢复函数模型状态，检查必须失败 | `diff/TraceBridge.cpp` |
| `ME_CHECK` | 关 | 每周期检查 move elimination 的引用计数（`arch_cnt` 与 arch RAT 一致；空闲 + 在途 + 不同的架构寄存器 = `PRF_NUM`，无泄漏） | `back-end/Ren.cpp` |

## 4. 前端、分支预测

BPU 的变量都是模型实验用的运行期参数，默认值就是 RTL 对应的设计；`BPU_PIPE` 取值含义见 `front-end/BPU/BPU_pipe.h` 头注释，BPU 优化开关见 `front-end/BPU/` 的文档。

| 变量 | 默认 | 作用 | 位置 |
|---|---|---|---|
| `FE_RFAST` | 编译期默认（15） | 重定向快路径位掩码：bit0 S0 在重取指周期发射、bit1 PTAB 写到读旁路、bit2 重定向后 I$ 立即 ready（需 bit0）、bit3 PreIdu 旁路（EXP-023） | `BPU_pipe.h`、`icache_module.cpp`、`PreIduQueue.cpp` |
| `FE_BFAST` | 编译期默认（0） | bit0：S1（L1）覆盖后同拍发射修正块；bit1：S_D 覆盖后同拍发射（少 1 拍气泡） | `BPU_pipe.h` |
| `FE_PAIR` | 0 | 设 `1`：一拍取两块（S0 猜测为 taken 时同拍查目标块） | 同上 |
| `FE_IPF` | 0 | 设 `1`：I$ 预取（只对此前被需求 miss 填过的行预取，不走真实 LLC/AXI/DRAM 路径，是模型局限） | `front-end/front_top.cpp` |
| `FE_IPF_LAT` | 12 | I$ 预取的固定延迟（周期） | 同上 |
| `FE_TRACE` | 关 | 打印前端 refetch 附近的流水状态（周期 > 40000 之后，最多 24 行） | 同上 |
| `BPU_PIPE` | 3 | BPU 建模层级（3 = 带更新队列、与 S1 读仲裁的单口阵列，RTL 对应） | `BPU_pipe.h` |
| `BPU_S0_ENTRIES` / `BPU_S0_WAYS` | 128 / 4 | S0（uBTB）项数 / 路数 | 同上 |
| `BPU_MAIN_LAT` | 2 | 主预测器延迟 | 同上 |
| `BPU_UPDQ` / `BPU_UPD_PORTS` | 8 / 2 | 更新队列深度 / 写口数 | 同上 |
| `BPU_S0_TRAIN` | 1 | `0`：uBTB 从不训练（调试） | 同上 |
| `BPU_BUBBLE` | 0 | predicted-taken 后 BPU 空转的拍数（EXP-010） | `BPU.h` |
| `BPU_UBTB` | 0 | N 项直接映射 1 拍目标表，隐藏气泡 | 同上 |
| `BPU_ICALL` | 1 | `rd=ra` 的 jalr 当作调用（`0` = 旧设计，当 IDIRECT） | 同上 |
| `BPO_BANKBLK` | 4 | 块 bank 数（0 = 按 slot 的 bank 口模型） | `BPU_pipe.h` |
| `BPO_UPD_MERGE` | 1 | （配合 `BANKBLK`）同一行的更新合并成一次写 | 同上 |
| `BPO_UB_MAIN` | 0 | `1`：也用 S_D 结果训练 uBTB / L1；`2`：只用 S_D | 同上 |
| `BPO_L1_ENTRIES` / `BPO_L1_WAYS` | 512 / 2 | L1 预测器大小 / 路数 | 同上 |
| `BPO_UB_TAG` | 10 | uBTB 标签位数 | 同上 |
| `BPO_L1_HBITS` / `BPO_UB_HBITS` | 0 / 0 | L1 / uBTB 的索引和标签用多少位近期历史 | 同上 |
| `BPO_LOOP` | 0 | S0 单块循环退出预测（需 `BPO_HIST=1`） | 同上 |
| `BPO_PDCKPT` | 1 | 每个 PTAB 项一个检查点，预解码 flush 时精确恢复历史 / RAS | 同上 |
| `BPO_TQ` | 4 | 训练队列深度 | 同上 |
| `BPO_HIST` | 1 | 全局历史每个 taken 分支压入一个符号（`0` = 每条条件分支一位方向） | `BPU.h` |
| `BPO_HSYM` | 3 | 历史符号位数 | 同上 |
| `BPA_HLEN` | `8,13,32,119` | TAGE 各表历史长度（逗号分隔，≤255 位） | 同上 |
| `BPA_ITTH` | 编译期默认（`1,2,3`） | 间接跳转表（ITT）复用的 TAGE 表序号；设为空串 = 关 ITT | 同上 |
| `BPA_ITTN` / `BPA_ITTCONF` | 64 / 0 | 每 bank 每表项数（2 的幂，≤1024）/ 最小置信度 | 同上 |
| `BPA_REPAIR` | 3 | 误预测时精确修复历史 / RAS（位掩码；`0` = 从已提交状态拷贝） | 同上 |

## 5. 访存（LSU 分级流水、dcache、预取）

LSU 的位掩码开关含义详见 `docs/LSU_PIPE.md` 和 `RealLsuPipe.cpp` 里各变量上方的注释。除 `LSU_PIPE`、`LSU_MEMSPEC`、`LSU_MS_DATA` 外都只对分级 LSU 生效。

| 变量 | 默认 | 作用 | 位置 |
|---|---|---|---|
| `LSU_PIPE` | 编译期默认（1） | 非 `0` 用分级 LSU，`0` 用旧的非分级 LSU | `include/config.h` |
| `LSU_PIPE_NOWD` | 关（**对周期必须设**） | 设了就关掉 63 周期看门狗（只打印 `[WD]`，不重发） | `RealLsuPipe.cpp` |
| `LSU_PIPE_CUT` | 编译期默认（127） | 关键路径实验位掩码：B/R/D/F/FM/G/H（见源码注释，RTL 已按默认实现） | 同上 |
| `LSU_PIPE_L2` | 编译期默认（115） | L2 阶段位掩码：L3 寄存器、唤醒时机、MSHR-full、推测唤醒等 | 同上 |
| `LSU_PIPE_RP` | 编译期默认（4） | 重放寄存器个数 | `RealLsu.cpp` |
| `LSU_PIPE_BOUNCE` | 0 | 老的重放寄存器在 N 周期后顶掉最后一个新 load（0 = 关，最大 7） | 同上 |
| `LSU_PIPE_GATE` | 1 | DCache 读使能由部分 CAM 限定 | 同上 |
| `LSU_PIPE_RBLK` | 1 | 端口上有待处理的 DCache 隐藏重试时不发射 | 同上 |
| `LSU_PIPE_RPRIO` | 开 | 设 `0` 关掉 DCache 对重放 load 的优先级 | `MemSubSystem/RealDcache.cpp` |
| `LSU_PIPE_DRAIN` / `LSU_PIPE_DCAND` | 编译期默认（2 / 4） | store 每周期排出的口数 / 候选扫描数 | `RealLsuPipe.cpp` |
| `LSU_PIPE_WBUF` | 编译期默认（4） | store write buffer 项数（0 = 关；EXP-024；需 `LSU_PIPE_MGRAN=64`） | 同上 |
| `LSU_PIPE_WBINS` / `LSU_PIPE_WBSCAN` / `LSU_PIPE_WBMAX` | 编译期默认（4 / 4 / 2） | 每周期插入的 store 数 / 扫描的项数 / 每项最多合并的 store 数 | 同上 |
| `LSU_PIPE_MERGE` | 1（关） | N：同一 cache 行的最多 N 个已提交 store 共用一个 store 口（写合并上界） | 同上 |
| `LSU_PIPE_MGRAN` | 64 | 合并粒度（字节，2 的幂，4..64） | 同上 |
| `LSU_PIPE_MADJ` | 0 | 跟随者只能加入至多 N 个扫描位置之前的领队（0 = 任意更早的领队） | 同上 |
| `LSU_PIPE_MSPEC` | 0 | 合并配对在重叠 CAM 之前决定（RTL 的做法） | 同上 |
| `LSU_MEMSPEC` | 编译期默认（1） | 访存推测：load 越过地址未知的老 store（EXP-018/019） | `RealLsuPipe.cpp`、`MsData.h` |
| `LSU_MS_DATA` | 编译期默认（1） | 访存推测的值路径位掩码：1 分支按旧值解析、2 污染地址（未建模，只计数）、4 污染的 store 数据进 STQ、8 评估器自检（EXP-020） | `MsData.h` |
| `LSU_MS_SLOTS` | 编译期默认（0） | 同时处于推测状态的 load 上限（0 = 不限） | `RealLsuPipe.cpp` |
| `LSU_MDP_MODE` | 编译期默认（0） | 访存依赖预测器：0 = 等待表，1 = store sets | 同上 |
| `LSU_MDP_BITS` / `LSU_MDP_CLR` | 编译期默认（6 / 2000） | 等待表 2^N 项（0 = 关预测器）/ 每多少周期清一次（0 = 不清） | 同上 |
| `LSU_MDP_HW` | 编译期默认（2） | 等待表训练的可实现程度：0 = 立即、1 = 最低 LDQ 索引、2 = 违例先标记、下一拍从寄存器训练（RTL 的做法） | 同上 |
| `LSU_MDP_NTR` | 2 | 等待表写口数（每周期训练个数） | 同上 |
| `LSU_MDP_THR` / `LSU_MDP_DEC` | 1 / 0 | 置信度阈值 / 无违例时的递减（1/0 = 原始粘性 1 位） | 同上 |
| `LSU_PIPE_STATS` | 关 | 非 `0`：结束时打印 `[LSU_PIPE_STATS]`（和 `[MS_DATA]`）统计 | `RealLsuPipe.cpp`、`MsData.h` |
| `LSU_PIPE_EVT` | 关 | 每周期打印 LSU 边界事件（`E t=...`），格式和 RTL 测试平台的 `SOC_TB_LSUEVT` 相同，用来逐周期对 RTL | `RealLsuPipe.cpp` |
| `LSU_PIPE_STQ` | 关 | 设 STQ 项号：事件行里额外打印这一项的寄存器 | 同上 |
| `LSU_PIPE_DUMP` | 关 | 设周期数：在这个周期转储 LDQ / STQ | 同上 |
| `LSU_PIPE_WBTRACE` | 关 | 设周期数 N：从第 N 周期起打印写回缓冲的响应 / 发射事件（`WB t=...`，stderr） | 同上 |
| `LSU_MS_TRACE` | 关 | 打印访存推测事件（与 RTL 的 `MS_DBG` 对应） | 同上 |
| `LSU_MS_DTRACE` | 关 | 打印每次旧值 / 翻转 | `MsData.h` |
| `PF_ENABLE` | 1 | 设 `0` 关掉 L1D 预取器（`PF_<参数名>` 是预取器参数的统一入口，目前源码只读 `PF_ENABLE`） | `MemSubSystem/include/DataPrefetcher.h`、`Prefetch.h` |
| `PF_STATS` | 关 | 结束时打印预取统计（issued / useful / late） | `DataPrefetcher.h` |

## 6. DDR / AXI

| 变量 | 默认 | 作用 | 位置 |
|---|---|---|---|
| `DS3_CONFIG` | 内置 `DS3_DEFAULT_CONFIG` | DRAMsim3 配置文件（需用 DRAMsim3 构建时有效） | `axi-interconnect-kit/sim_ddr/include/SimDDR_DS3.h` |
| `DS3_OUT` | `ds3_out` | DRAMsim3 输出目录（运行时创建） | 同上 |
| `DS3_EXTRA_NS` | 构造参数 | 在 DRAMsim3 之外额外加的固定延迟（ns） | 同上 |
| `CPU_FREQ_MHZ` | 1000（`CONFIG_CPU_FREQ_MHZ`） | 运行时覆盖 CPU 频率（影响 ns → 周期换算） | 同上 |
| `DS3_STATS` | 关 | 结束时打印 `[DS3]` 读写延迟和停顿统计 | 同上 |
| `AXI_SUBMODULE_MODE` | 内置 | AXI 互连的子模块模式，只取低 2 位 | `axi_interconnect/AXI_Interconnect.cpp` |
| `AXI_SUBMODULE_OFFSET` | 内置 | 覆盖 LLC 映射偏移 | 同上 |

## 7. 影子（参考实现）测试

只在 `tests/*_ref/` 的影子构建（`build_shadow/simulator`，见 `scripts/lsu_shadow.sh`）里读取：`ISU_SHADOW_DIRECTED`、`DIS_SHADOW_DIRECTED`、`ROB_SHADOW_DIRECTED`、`LSU_SHADOW_DIRECTED`，设任意值就在运行时额外跑一组定向的状态注入场景（模型和参考实现各跑一遍，状态摘要必须一致）。

## 已删除（2026-11-12）

没人用的临时调试开关：`CKPT_TRACE_PC`（EXP-011 窗口位置检查，已完成）、`TB_DEBUG_DDR`、`TB_DEBUG_FLUSH`、`TB_DEBUG_PATCH`（P1 桥接调试打印）、`LSU_PIPE_RPREG`（变量只被设置、从不读取）、`FE_IPF_MAX`（改成常量 4）。`include/config.h` 里 `ROB_COMPACT` 注释提到的同名环境变量源码里并不存在，已改正。

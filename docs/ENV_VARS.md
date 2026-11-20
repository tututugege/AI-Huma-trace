# trace-model 环境变量

`build/simulator` 运行时读取的全部环境变量，共 16 个（exp/tm-simplify，2026-11-16；之前是 77 个）。写法：`NAME=值 build/simulator prog.bin`。

> **模型的默认配置就是 RTL 配置**：和 RTL 比周期不需要设任何环境变量。（2026-11-19 删掉了 RTL 没有的 DCache 响应看门狗和它的开关 `LSU_PIPE_NOWD`；以前忘了设它会让 crc32、memory-stress-small 这类程序差几十周期。）

| 变量 | 默认 | 作用 | 位置 |
|---|---|---|---|
| `SIM_MAX_CYCLES` | 内部上限 | 周期数到了就停（退出码 3） | `driver/main.cpp` |
| `FAST_WARMUP` | 关 | 设 `1`：检查点运行时用快速预热（同 `--fast-warmup`） | 同上 |
| `PMU` | 开（2026-11-19 起） | `0` 关掉 PMU / TMA 统计；开着时结束时打印并写 `pmu_*.csv`（不改变周期，耗时几乎不变） | `back-end/Pmu.cpp` |
| `PMU_OUT` | `pmu` | PMU 输出文件名前缀 | 同上 |
| `PMU_INTERVAL` | 0 | 每隔这么多周期记一行时间序列 | 同上 |
| `PMU_PCPROF` | 开 | 设 `0` 关掉按 PC 的统计 | 同上 |
| `FE_PAIR` | 0 | 设 `1`：一拍取两块（实验特性） | `front-end/BPU/BPU_pipe.h` |
| `FE_IPF` | 0 | 设 `1`：I$ 预取（实验特性） | `front-end/front_top.cpp` |
| `FE_IPF_LAT` | 12 | I$ 预取的固定延迟（周期） | 同上 |
| `PF_ENABLE` | 1 | 设 `0` 关掉 L1D 预取器（nopf 门禁 / RTL 的 nopf 表用） | `MemSubSystem/include/DataPrefetcher.h` |
| `FPR` | 关 | 设 bin 周期数：打印 BPU 的对齐指纹（`FPR PT/UP bin= n= h=`），`rtl/scripts/align_check.py` 和 RTL 的同名输出比对 | `front-end/BPU/fpr.h` |
| `TB_TRACE_COMMIT` | 关 | 每条提交打印 `[COMMIT] cyc pc inst rd v`；`rtl/scripts/first_diff.sh` 用它和 RTL 的 `SOC_TB_COMMIT` 找第一处分歧 | `bridge/trace_hook.cpp` |
| `LSU_PIPE_EVT` | 关 | 每拍打印 LSU 边界事件（`E t=..`），与 RTL tb 的 `SOC_TB_LSUEVT` 同格式，逐拍对齐 LSU 用 | `back-end/Lsu/RealLsuPipe.cpp` |
| `FPR_DUMP` | 关 | `<from>:<to>`：再逐条打印这段周期里的事件（`FPRRAW`） | 同上 |
| `DS3_OUT` | `ds3_out` | DRAMsim3 输出目录（并行运行时每个运行一个） | `axi-interconnect-kit/sim_ddr/include/SimDDR_DS3.h` |
| `DS3_CONFIG` | 编译期默认 `.ini` | DRAMsim3 配置文件路径覆盖（跨机静态二进制在集群运行时使用） | `axi-interconnect-kit/sim_ddr/include/SimDDR_DS3.h` |

## 已删除（2026-11-16，exp/tm-simplify）

全部冻结为当时的默认值，非默认分支删除（git 历史里有）：
- BPU：`BPU_S0_ENTRIES` / `BPU_S0_WAYS` / `BPU_MAIN_LAT` / `BPU_UPDQ` / `BPU_UPD_PORTS`、`BPO_BANKBLK` / `BPO_L1_ENTRIES` / `BPO_L1_WAYS` / `BPO_UB_TAG` / `BPO_LOOP`（loop 出口预测器删除）/ `BPO_TQ` / `BPO_HSYM`、`BPA_HLEN` / `BPA_ITTH` / `BPA_ITTN`、`FE_RFAST`（15）/ `FE_BFAST`（0，连同 config.h 的 `CFG_FE_*`）；调试打印 `BPU_OVR_DBG` / `BPU_OVR_DBG2`、`FE_TRACE`（对齐指纹 `FPR` / `FPR_DUMP` 曾误删，已恢复；同样恢复了 RTL 对齐用的 `TB_TRACE_COMMIT`、`LSU_PIPE_EVT`）。
- LSU：`LSU_MEMSPEC`（1）、`LSU_MS_DATA`（7，含自检位 8）连同 `CFG_LSU_MEMSPEC` / `CFG_LSU_MS_DATA`；统计 `LSU_PIPE_STATS`（含 `[MS_PENALTY]` / `[MS_DATA]`）；调试 `LSU_PIPE_DUMP` / `LSU_PIPE_STQ` / `LSU_PIPE_WBTRACE` / `LSU_MS_TRACE` / `LSU_MS_DTRACE`。
- 桥接：`TB_CHECK`（逐条值比较）、`TB_TRACE_PATCH` / `TB_TRACE_TRAP` / `TB_SAMPLE_PC` / `TB_PCTRACE`、阳性 / 阴性对照 `TB_PERTURB_BR` / `TB_PERTURB_ADDR` / `TB_NEG_NO_REWIND` / `TB_NEG_NO_MEMWRITE` / `TB_NEG_NO_TRAP_STATE`。
- 其它：`ME_CHECK`、`RVC_ILL_DBG`、`FE_COMMIT_TRACE`、`FP_WB_PORTS`（2）、`PF_STATS`、`TB_TIMER_DBG`、`DS3_CONFIG` / `DS3_EXTRA_NS` / `CPU_FREQ_MHZ`（运行时覆盖；编译期 `gmake CPU_FREQ_MHZ=` 仍在）/ `DS3_STATS`。
- 编译期变体：`FP=0`（F/D 永远开）、`CFG_ROB_COMPACT=0`（行 ROB）、`CFG_MOVE_ELIM=0`、Makefile 的 `DS3=0`（固定延迟 SimDDR）、LLC 关闭模式。

## 已删除（2026-11-15）

`TB_POISON`（P2 时用来证明时序不读模型自算的值，值通路早已删除）；`TB_NO_INJECT`（关掉事实注入，P3 之后无法运行）；命令行 `--no-diff`（提交时 difftest 已不存在）；`LSU_PIPE`（旧 LSU 删除，只剩分级 LSU）；`TB_RESYNC_REF`（从第二个参考核重同步，第二个核已删除）；LSU 的消融开关 `LSU_PIPE_RPRIO` / `WB_RESERVE` / `LSU_PIPE_RP` / `LSU_PIPE_BOUNCE` / `LSU_PIPE_GATE` / `LSU_PIPE_RBLK` / `LSU_MDP_BITS` / `LSU_MDP_CLR` / `LSU_MDP_MODE` / `LSU_MDP_HW` / `LSU_MS_SLOTS` / `LSU_MDP_NTR` / `LSU_MDP_THR` / `LSU_MDP_DEC` / `LSU_PIPE_CUT` / `LSU_PIPE_L2` / `LSU_PIPE_DRAIN` / `LSU_PIPE_DCAND` / `LSU_PIPE_MERGE` / `LSU_PIPE_MADJ` / `LSU_PIPE_MSPEC` / `LSU_PIPE_WBUF` / `LSU_PIPE_WBINS` / `LSU_PIPE_WBSCAN` / `LSU_PIPE_WBMAX` / `LSU_PIPE_MGRAN`（冻结为默认值，非默认分支删除）；AXI 互连的 `AXI_SUBMODULE_MODE` / `AXI_SUBMODULE_OFFSET`（LLC 运行时模式 0 / 2 和映射窗口删除，固定为模式 1）；命令行 `--mode fast` / `--mode ref` / `-f` / `--flash-image` / `--sdcard-image` / `--direct-opensbi`（没有脚本使用）；影子构建的 `ISU_SHADOW_DIRECTED`、`DIS_SHADOW_DIRECTED`、`ROB_SHADOW_DIRECTED`、`LSU_SHADOW_DIRECTED`（`tests/*_ref` 和 `<MOD>_SHADOW` 构建一起删除）。

## 已删除（2026-11-12）

没人用的临时调试开关：`CKPT_TRACE_PC`（EXP-011 窗口位置检查，已完成）、`TB_DEBUG_DDR`、`TB_DEBUG_FLUSH`、`TB_DEBUG_PATCH`（P1 桥接调试打印）、`LSU_PIPE_RPREG`（变量只被设置、从不读取）、`FE_IPF_MAX`（改成常量 4）。`include/config.h` 里 `ROB_COMPACT` 注释提到的同名环境变量源码里并不存在，已改正。

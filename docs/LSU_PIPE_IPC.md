# LSU_PIPE 对 legacy LSU 的 IPC 差分解（2026-11-07；第 8 节是 lsu-pipe-fix 分支的实施结果，其余是研究阶段的原文）

> **状态说明（2026-11-12）**：本文是 2026-11-07 的研究记录，不随后续合入更新。第 2 节表里的“现默认 CUT=63 L2=115”是当时的默认；main 现在的默认是 CUT=127、DRAIN=2 + 4 项写缓冲（`CFG_LSU_PIPE_WBUF`）、RP=4，并默认开启访存推测（`CFG_LSU_MEMSPEC` 1），见 [LSU_PIPE.md](LSU_PIPE.md) 第 11、12 节和文件头。第 7 节的 `ipc-scratch` 分支、scratchpad 路径和服务器目录是一次性的，可能已不存在；`LSU_PIPE_DRAINSKIP/DRAINCAND/CAMBITS` 等 scratch 分支开关不在 main（main 里对应的是 `LSU_PIPE_DCAND`）。环境变量见 [ENV_VARS.md](ENV_VARS.md)。

范围：30 点 exact-warmup SPEC（同 LSU_PIPE.md 10.1），模型提交 472b937（默认 LSU_PIPE=1 CUT=63 L2=115）。**服务器上重编的 Linux 二进制对 legacy 的周期与 Mac 结果逐点相等**（如 perlbench sp5 3,183,317）。每个配置一次运行（模型确定性）。
IPC 变化 = cycles(legacy)/cycles(配置) - 1，几何平均。全部数字是 30 点，除非标注“只有 8 点”（x_/y_/z_ 配置）。**主扫描最后几个配置（尾部若干点）在写本文时仍在跑，ALL 列里 n<30 的列已标出；缺失的不影响结论。**

## 1. 先排除的假设：load-to-use 不是原因
微基准（`tests/ubench`，本地 build_lm2，legacy=`LSU_PIPE=0`）：lat_ptr 4.0/4.0，ld_alu 4.92/4.92，ld_chain4 1.0/1.0，ld_unk_addr 8/8，lat_stlf 5/5。**legacy 的 load-to-use 也是 4 拍**，分级 LSU 并没有多一拍。
有差别的：stlf_partial 9 → 12（部分重叠转发多 3 拍），bank_diff 0.75 → 0.84 拍/load（每个 load 都过一次 RDY→RP→L1 重放，`rblk`），st_ld_alias 2.007 → 2.081。

## 2. 消融表（ALL = 30 点几何平均，对 legacy）
| 步骤 | ALL | 备注 |
|---|---|---|
| staged，不切（CUT=0 L2=0） | **-2.4%** | perlbench -7.5，gcc -5.7，hmmer -8.1，h264ref -2.2，gobmk -2.2，sjeng -2.6，astar -3.2，mcf +3.4 |
| + B(1) | -2.4% | 0 |
| + R(2) 单独 | -2.7% | h264 sp17 -3.4 |
| + D(4) 单独 | **-3.8%** | perlbench -12.6（单点 -6），gobmk -4.2，sjeng -4.2，astar -5.2 |
| + F(8) 单独 | **-3.2%** | h264ref -12.4（sp9 -13.4，sp17 -4.8） |
| + G(16) 单独 | -2.4% | 0 |
| + H(32) 单独 | -2.6% | |
| CUT=63（B R D F G H） | **-4.8%** | |
| CUT=63 去掉 D（59） | -3.4% | 回收 1.4 |
| CUT=63 去掉 F（55） | -4.1% | 回收 0.7，h264ref -15.5 → -6.0 |
| CUT=63 去掉 R/G/B/H | -4.7/-4.8/-4.8/-4.9 | 无差别 |
| CUT=63 + L2 a（1） | -8.1% | load-to-use 5 |
| + a+a2（3） | -4.9% | |
| **CUT=63 + L2=115（现默认）** | **-5.0%** | 与 10.1 的 -4.98% 一致 |
| L2=115 去 S/F/M（51/99/83） | -5.0/-4.9/-4.9 | F、M、S 都几乎免费 |
| CUT=63 + L2=123（推测唤醒 d） | **-3.0%** | 推测唤醒只回收 2.0 个点 |
| CUT=63 + L2=8 / 4（无 L3，d / oracle） | -2.6 / -2.9% | |
| CUT=0 + L2=115 | -5.4%（部分点） | |
| CUT=0 + L2=123 | -2.2%（部分点） | |

逐段：legacy→staged 不切 -2.4；+L1 切 -2.4（D 1.4 + F 0.7 + R/H 0.3，互相重叠）；+L2 115 -0.2；推测唤醒回收 +2.0。

## 3. 根因（按 IPC 贡献）
### 根因 1：committed store 排空太窄（不是切断造成的，staged 不切就有）
证据（PMU=1，测量窗口，legacy → CUT=0）：
* hmmer sp11：`SLOT_BE_STQ_FULL` 607k → 1,644k 槽，`OCC_STQ` 70M → 103M；h264 sp17 STQ_FULL +2,905k 槽；统计 `conflict_block` ≈ `drain`（hmmer 2.9M/2.86M）。
* 机制：排空只取前 `DRAIN_PORTS=2` 个 ready 候选，候选被更老的同字节 store 挡住（`CheckAddr`）时**占着端口空转**，队头两个都被挡就整拍不排；legacy 是 3 口、被挡的不占口、窗口 15。
* 消融（8 点，不切）：`LSU_PIPE_DRAINSKIP=1`（被挡的不占口，2 口）hmmer -7.5 → **+1.5**；`DRAIN=3` 不 skip：gcc sp42 -14.5 → -6.1，h264 sp17 -6.6 → -0.7；`DRAIN=3+SKIP`：hmmer +2.2，gcc42 -6.1，h264 sp17 -0.7。2 口、只看前 4 个候选（`DRAINCAND=4`，`z_c4`）：hmmer +2.1 但 gcc42/h264 sp17 无改善（需要第 3 口）。
* 加在现默认（63/115）上：hmmer -8.5 → +0.5～+2.5，gcc42 -15.0 → -6.0，h264 sp17 -16.0 → -11.3。
* 8 点内 hmmer(4 点)整个差、gcc sp42 的 8.4 点、h264 sp17 的 6 点都来自这里。

### 根因 2：重放路径窄 + 两条切断（D、F）的代价集中在 load 重放/写回
* load 延迟分布（`LSU_PIPE_STATS` 新增 latAGUWB 直方图，AGU 结果→写回请求）：hmmer legacy 90% 在 1 拍、staged 80%，另有 8.5% 在 3 拍（12 位偏移 CAM 假命中 `cand_false` 558k，重放一圈）；perlbench sp3 平均 9.0 → 11.1 拍，13～64 拍的尾巴 24% → 30%（等老 store 地址 `unk_addr_wait_disjoint` 3.5M 次，一个 load 平均重放 >1 次）。
* 重放寄存器只有 2 个：`RP=4`（模型 `LSU_PIPE_RP`，RP_MAX 改 4）perlbench -7.5 → **-5.4%**（sp5 -10.1 → -7.3，sp3 -4.9 → -3.5），gobmk +0.5，对其他程序 ≈ 0。
* **D 切断**（重放寄存器只从寄存器里的 RDY 选，晚醒的晚一拍）：perlbench -5（sp5 -4，sp3 -6.1），gobmk -2.0，sjeng -1.6，astar -2.0。
* **F 切断**（WBP 选择只用寄存器位）：h264ref sp9 -16，sp17 -4.2；`mmio_ld`/`fwd_wait` 计数都很小，**为什么 1 拍延迟变成 16% 我没有查清**（统计只显示 `SLOT_BE_HEAD_MEM` +11M 槽，没有拿到 WBP 占用；`occ` 计数器已加入但这一点没有来得及跑 c08）。结论按实测，不按机理。
* 12 位 CAM 假命中：`CAMBITS=32`（`y_cam32`）全部点 ≈ 0（-0.0～-0.2），**不是**原因（hmmer 的 3 拍重放只是延迟，不是瓶颈）。
* `RBLK=0 / RPRIO=0`（恢复 legacy 的隐藏重试语义）更差（gcc42 -14.5 → -22.3，hmmer -8 → -11），**现有的 RBLK/RPRIO 是对的，不要动**。

### 其他
* 推测唤醒（L2=123）总共 +2.0 个点（-5.0 → -3.0），单靠它到不了 legacy 水平；统计 58% 被取消（LSU_PIPE.md 7.3）。
* L2 的 a/a2/F/M/S（115）相对 CUT=63 只多 -0.2%。

## 4. 各切断的建议（深度数字取自 rtl-lsupipe docs/analysis/lsu-pipe.md 9.2/10.3）
| 位 | 省的层数 | 30 点 IPC 代价 | 建议 |
|---|---|---|---|
| B、G | 小 | 0 | 留 |
| R | 小 | -0.3%（h264 -3.4 单点） | 留（便宜）或在 h264 敏感时删 |
| H | 3（166←169） | -0.2% | 留 |
| **D** | 24（350→326，之后重放选择叠在 st9 后，不做 D 路径回到 230+） | **-1.4%，perlbench -5、astar -2、gobmk -2** | 深度价值大，**留，但要补偿**：RP 加到 4（回收约 2 点 perlbench），未测 D+RP4 的组合 |
| **F** | 约 6（142←136 里的 F/W 相关） | **-0.7% 总体，h264ref -9.5%（sp9 -16）** | **性价比最差：对 h264ref 太伤，建议删（或查清原因再改）** |
| L2 a+a2 | 155→（含 W/P/T）124 | -0.2% | 留 |
| L2 F、M、S | 各 ~6～7 | ≈ 0 | 留 |
| 推测唤醒 d | 121 | +2.0% | **不够**：即使加上也是 -3.0%，且 58% 取消，硬件代价未测 |

## 5. 便宜且不拉长关键路径的修复（模型已验证的部分）
1. **排空**：被挡的候选不占口 + 3 个排空口（STA 口本来就是 3 个）：8 点上 hmmer 到 legacy 以上、gcc42/h264 sp17 大部分回收。RTL：排空 CAM 从 2×64 到 3×64（或前 4 候选 CAM + 4→3 压缩），排空本身 73 层，不在关键路径（关键路径是 L2 段 124 层）。**优先级最高**，对 8 点上 -2.4% 的主要来源。
2. **RP 2→4**（perlbench +2～3 点）：多两个 PRp 寄存器 + 两级 first-set，与 D 同一块逻辑，层数需要综合确认。
3. **F 切断改写**（或删）。
全部合起来的 30 点数字**没有跑**（只有 8 点：`x_new_d3s` 即 63/115+DRAIN3+SKIP：hmmer +0.5/+2.5，gcc42 -6.0，perlbench -11.6，gobmk -7.3，h264 sp17 -11.3，sp9 -17.3，因为 F 仍在）。

## 6. 没做 / 没确认
* 30 点上的“存储修复 + RP4 + 去 F”组合没有跑（时间）。
* F 在 h264ref 上 16% 的机理；D+RP4 的交互；perlbench 去掉 D 与 RP 之后剩下的 -5% 没有再拆。
* gcc sp42 在 DRAIN=3 后仍 -6.0%，原因未查。
* 统计计数包含 10M 预热；PMU 事件是测量窗口，二者不要混着算比例。

## 7. 复现
* 服务器：`fleet.sh rtlsrc`（RTL_SRC=模型 worktree，RTL_DST=/root/lsuipc/tm）推源码，`make BUILD_DIR=build` 在 `/root/lsuipc/tm*/` 里编；任务 `/root/lsuipc/job.sh <tag> "<ENV>" <bench> <ckpt>`（`PMU=1 LSU_PIPE_STATS=1`，`-c 10000000`，stub 在 `/root/lsuipc/stubs`）；结果 `/root/lsuipc/res/<tag>/<bench>/<ckpt>.{ok,log,pmu_events.csv}`。
* 配置标签：`leg`=`LSU_PIPE=0`；`u00`=`CUT=0 L2=0`；`c<n>`=`CUT=n L2=0`；`n63_l<m>`=`CUT=63 L2=m`；`x_/y_/z_*` 在 scratch 分支用 `LSU_PIPE_DRAIN` `LSU_PIPE_DRAINSKIP` `LSU_PIPE_DRAINCAND` `LSU_PIPE_CAMBITS` `LSU_PIPE_RP=4`。
* scratch 分支 `ipc-scratch`（3 个提交，作者 tututugege，**不在任何 main**）：工作区 `/private/tmp/claude-501/-Users-ruyaoh-personal-AI-Huma/76c7eb6c-d693-45f6-9fc2-5e97a09d3da1/scratchpad/tm_ipc`，含上述环境开关、load AGU→写回直方图（`LSU_PIPE_STATS=1` 的 latAGUWB 行，legacy 也有）、LDQ 状态占用计数。默认行为不变。
* 本地数据：`scratchpad/{r1,ry,rx,all}.jsonl`（每运行一行：周期、PMU 事件、统计），`tab2.py <jsonl> <base> <tag,tag,...>` 出表。

## 8. 实施结果（分支 lsu-pipe-fix，LSU_PIPE.md 第 11 节）
30 点几何平均，对 legacy：当前默认 -4.98% → +排空修复（3 口、4 候选、被挡的不占口）-2.94% → +RP4 -2.18% → +去 F（CUT=55）-1.52%（RTL 综合：去 F 使 lp_core 131 → 167 层，不采用）；**新默认 CUT=127（保留 F + MMIO 豁免 FM）-1.39%**；去 F 再加 L2=123（推测唤醒，仅供参考）+1.34%。
排空修复单独 +2.0 点（hmmer -8.7 → +1.8，gcc -6.3 → -2.2），RP4 +0.8，去 F / FM +0.7～0.8。
**F 的 16% 已查清（第 3 节“没有查清”作废）**：h264ref sp9 在 20M 周期里有 278 万个 MMIO load（占其 load 近 30%）；MMIO load 一次一个，下一个要等上一个完成（必须是最老未完成指令）。F 切断让 MMIO 响应的写回晚一拍，于是**每个 MMIO load 串行多一拍**：16.28M → 18.87M 周期（差 2.6M ≈ MMIO load 数，WBP 占用 2.70M 对 2.8K）。
这是该 checkpoint 的特征（MMIO load 这么多不是典型 SPEC 行为），不是 WBP 端口压力。不删 F 的做法（采用）：CUT 位 64（FM，RTL LP_CUT 位 17）让 MMIO 响应当拍写回，F 保留：ALL -1.39%、h264ref -4.9%，与删 F 的 -1.52% 相当，而 RTL 上不用付 36 层。
剩余 -1.52%：perlbench -10.5%、gobmk/sjeng/astar 约 -4%（D 切断与重放路径，未再拆）。
RP=4 后“load stuck in DCW”看门狗事件变多（30 个运行 4,267 次，RP=2 时 403 次）：都是错误路径上请求无响应的 load（NOWD 下 12 个事件最多的点全部正常结束，周期差在 ±0.2% 内）；RTL 没有看门狗，所以 RTL 参考周期用 `LSU_PIPE_NOWD=1` 生成。根因（DCache 请求路径上丢响应）没有分析。

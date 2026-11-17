# 共用 AGU / DTLB 口（EXP-TLBPORTS）

分支 `exp/tlb-ports`（trace-model，从 main 5b95b35 切出）。全部是运行时开关，默认关，默认周期不变（dhrystone 100,983）。

## 开关

| 环境变量 | 含义 |
|---|---|
| `TLB_HIST_OUT=<文件>` | 退出时写出每周期 DTLB 请求数直方图（load 数 × store 数） |
| `TLB_LD_PORTS` / `TLB_ST_PORTS` / `TLB_TOT_PORTS` | DTLB 入口每周期最多服务的 load / STA / 总请求数，其余走已有的“无响应”重试路径 |
| `AGU_TOT=N` | load 与 STA 共用 N 个 AGU（发射端计数上限，load 队列先调度所以 load 赢） |
| `AGU_CFG=nL,nS,nM` | 专用 load / 专用 store / 共用口（load 优先）的计数上限版，发射端和 DTLB 入口同形状 |
| `AGU_PORTS=nL,nS,nM` | **端口精确版**：物理 AGU 口 nL+nM+nS 个；LD 队列的口 k 就是物理口 k（k < nL+nM），STA 队列的口依次是专用 store 口、再是共用口；STA 在共用口上的授权要求该口没被 load 占用；重发的 load / STA 需要 DTLB 时占一个自己能用的空闲口，没有就留在重发寄存器里。要求构建的 `CFG_LD_PORTS = nL+nM`、`CFG_STA_PORTS = nS+nM`（212 正好是 4 / 3） |

实现：`back-end/include/AguCfg.h`（形状）、`Isu.cpp` `grant_gate`（发射授权）、`Lsu/LsuPipeTranslate.cpp`（新请求带物理口，重发占口）。

## 测量方法

eda 上 rv64gc SPEC06 检查点（`/nfs_global/S/houruyao/spec06_rv64gc`），8 个访存重的程序各取权重最高的 2 个点，共 16 个；stub 前门恢复（`ckpt2stub.py` + `--restore-stub`，不加 `-w`，不能用 `--fast-warmup`，它的预热 TLB 只有 Sv32），10M 条详细预热加 4M 条测量。

## 结果（IPC 相对 4 load + 3 STA = 7 口的变化，16 点几何平均）

| 配置（专用 load / 专用 store / 共用） | AGU 数 | 几何平均 | 最差的点 |
|---|---|---|---|
| 共用 6 | 6 | -0.08% | -0.65% |
| 共用 5 | 5 | -0.42% | -3.43% |
| 共用 4 | 4 | -1.51% | -6.13% |
| 3/1/1（计数版） | 5 | -0.54% | xalancbmk.5 -3.43% |
| **2/1/2（计数版）** | 5 | -0.47% | xalancbmk.5 -3.48% |
| **2/1/2（端口精确版）** | 5 | **-0.49%** | xalancbmk.5 -3.29% |
| 2/1/1（计数版） | 4 | -1.27% | h264ref.6 -10.1% |
| 3/1/0（计数版，不共用） | 4 | -4.68% | h264ref.6 -33.9% |
| 3/0/1（计数版） | 4 | -5.17% | h264ref.6 -37.4% |
| 分离 3 load / 2 STA | 5 | -0.78% | bzip2.3 -3.6% |
| 共用 5 个 DTLB 口（不限发射） | 7 | -0.03% | -0.52% |

要点：必须给 store 留专用口（没有专用 store 口的 3/0/1、2/0/2 在 h264ref.6 这类 store 突发点上掉 10% 以上）；同样 5 个口，“共用”比“分离”好（-0.47% 对 -0.78%）。端口精确版和计数版几乎一样（-0.49% 对 -0.47%），默认配置的周期与未改动的版本逐点相同。

## 选定：212（2 专用 load、1 专用 store、2 共用，load 优先）

AGU 7 → 5，DTLB 口 7 → 5，PRF 的 AGU 读口 7 → 5。
RTL 改动清单（还没做）：

1. `isu`：LD 队列绑物理口 0、1、3、4，STA 队列绑 2、3、4；两个队列在共用口 3、4 上加冲突与门（STA 授权要求 LD 没占用）；口号映射代替“每个队列占连续区间”的假设（`isu.sv` 的布局断言）。
2. `exu` / `exu_ctrl_*`：AGU 7 → 5，共用口的输出按 uop 类型分给 `ldu` 或 `sta`（`exu_ctrl_lsu_out.sv` 现在按下标固定映射）；FU 列表和写回映射跟着改。
3. PRF：AGU 读口 7 → 5。
4. `lp_rp` / `lp_l1`：load 重发和 STA 重发需要 DTLB 时找自己能用的空闲口。
5. `tlb_mmu`：lane 7 → 5；`tlb_mmu_core` 里的 lane 类型（load / store）由静态的 `lane < LD_LANES` 改为每个请求带类型，响应再按类型分给 load 或 STA。
6. 其余 LSU 流水线不变（load 4 条、STA 3 条通道，替换 AGU / DTLB 口之后各自保留）。

AMO 作为特殊 store（EXP-AMO，另一分支）之后，LD/STA 队列合并时也不再需要为 AMO 留双写通路；本方案保持两个队列分开，不依赖它。

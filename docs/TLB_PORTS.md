# 共用 AGU / DTLB 口（EXP-TLBPORTS）

分支 `exp/tlb-ports`（trace-model，从 main 5b95b35 切出）。全部是运行时开关，默认关，默认周期不变（dhrystone 100,983）。

## 开关

只有一个编译期开关：`CFG_AGU_SHARED = NM`（默认 0，周期不变；`make EXTRA_CXXFLAGS=-DCFG_AGU_SHARED=2`）。load 队列和 STA 队列的口数不变（`CFG_LD_PORTS`、`CFG_STA_PORTS`），AGU / DTLB 的物理口数是 `LSU_AGU_PHYS = LD + STA - NM`。形状定义在 `back-end/include/AguPorts.h`：

- 口 0 到 NL-1 只给 load，NL 到 NL+NM-1 两者共用（load 优先），再往后只给 store 地址（NL = LD - NM）。
- load 队列的发射口 k 就是物理口 k；STA 队列的口依次是专用 store 口、再是共用口。
- 发射（`Isu::grant_gate`）：STA 授权要求它的物理口这一拍没被 load 占用，输了就留在队列里。
- DTLB（`LsuPipeTranslate.cpp`）：每个物理口一个请求，请求在自己 AGU 的口上，重发的 load / STA 需要 DTLB 时占自己能用的空闲口（没有就留在重发寄存器里），请求自带类型，walk 优先级按口序。

测量时（下面的结果表）用过一批环境变量版本的实验开关（`AGU_TOT`、`AGU_CFG`、`AGU_PORTS`、`TLB_*_PORTS`、`TLB_HIST_OUT`）：它们在分支历史里（`b1e3b90` 及更早），清理后的代码不再有，只保留上面的编译期形状；清理前后 67 个程序的周期完全相同。

## 测量方法

eda 上 rv64gc SPEC06 检查点（`/nfs_global/S/houruyao/spec06_rv64gc`），8 个访存重的程序各取权重最高的 2 个点，共 16 个；stub 前门恢复（`ckpt2stub.py` + `--restore-stub`，不加 `-w`，当时不能用 `--fast-warmup`(预热 TLB 只有 Sv32;2026-11-19 起已改为 Sv39 并验证,见 PROJECT.md)），10M 条详细预热加 4M 条测量。

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
RTL（`rtl` 仓库 `exp/agu-share`）：三个阶段都已做完，fm_gate 67/67、周期和 `scripts/ref_cycles_agu212.txt` 逐个相等、lint_chip 0：

1. 发射：STA 在共用口上的授权要求对应 load 口没被授权（`isu.sv`）。
2. `lp_l1`：重发占自己能用的空闲物理口；请求按物理口输出、响应按口分回槽。
3. PRF：共用的 STA 口借用 load 口的 src1 读口（`prf.sv`）。
4. DTLB：`tlb_mmu` 5 个口（`LSU_AGU_PHYS`），类型随请求走（`tlb_mmu_lane` 的访问类型由参数改为输入）。
未做：AGU 加法器合并（每个约 200 个单元）、LD/STA 队列合并。时序影响（AGU 到 TLB 的路径、PRF 读口前的两路选择）还没有综合量过。

原来的改动清单（供参考）：

1. `isu`：LD 队列绑物理口 0、1、3、4，STA 队列绑 2、3、4；两个队列在共用口 3、4 上加冲突与门（STA 授权要求 LD 没占用）；口号映射代替“每个队列占连续区间”的假设（`isu.sv` 的布局断言）。
2. `exu` / `exu_ctrl_*`：AGU 7 → 5，共用口的输出按 uop 类型分给 `ldu` 或 `sta`（`exu_ctrl_lsu_out.sv` 现在按下标固定映射）；FU 列表和写回映射跟着改。
3. PRF：AGU 读口 7 → 5。
4. `lp_rp` / `lp_l1`：load 重发和 STA 重发需要 DTLB 时找自己能用的空闲口。
5. `tlb_mmu`：lane 7 → 5；`tlb_mmu_core` 里的 lane 类型（load / store）由静态的 `lane < LD_LANES` 改为每个请求带类型，响应再按类型分给 load 或 STA。
6. 其余 LSU 流水线不变（load 4 条、STA 3 条通道，替换 AGU / DTLB 口之后各自保留）。

AMO 作为特殊 store（EXP-AMO，另一分支）之后，LD/STA 队列合并时也不再需要为 AMO 留双写通路；本方案保持两个队列分开，不依赖它。

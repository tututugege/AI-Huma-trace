# Linux 对齐（EXP-LINUXALIGN）：trace-model 对 RTL 的指纹/PMU 对齐

分支 `exp/tlb-ports`（trace-model）+ `exp/agu-share`（RTL，`6d7ed85`：两个 INT 发射队列）。2026-11-18 夜间工作记录。

## 目标与方法

两个 INT 发射队列合入 RTL 后，`fm_gate` 67/67、周期与模型参考表完全一致，但 Linux 从复位启动 1.4 亿周期的 PMU 序列/指纹对比（`scripts/linux_gate.sh`）
在 406 万周期处就出现差别。逐个定位、修模型（RTL 没有改），目标是全程对齐。

流程（每一类偏差都是这样找到的）：

1. `align_check.py` 找第一个不同的 PT / UP 指纹箱；
2. 在该窗口里用 `FPR_DUMP` 取两边的原始事件，找第一个不同的 PT 事件；
3. 模型打开 `MSD_TRACE=1`（陈旧 load、分支翻转、误预测目标、每个 BRU / AGU 操作）和 `BRIDGE_TRACE=1`（每次派发），RTL 测试台打开
   `SOC_TB_LSUEVT=1 SOC_TB_COMMIT=1 SOC_TB_EVT_RANGE=<from>:<to>`（LSU 事件、误预测来源 `R t=`、各发射口操作数 `P t=`），对齐"模型周期 = RTL 周期 − 1"；
4. 找出 RTL 比模型多（或少）的那次重定向，顺着它的操作数往回追到产生不同值的那条指令。

检查点窗口（`seg_cmp.sh`，Linux 检查点冷启动、两边各跑 400 万周期、比对指纹）比等 6840 万周期快得多，能同时暴露多类偏差。**RTL 没改时 RTL 的输出
只跑一次、缓存复用**（`build/segcache/<seg>/`），改模型后只重跑模型。

## 已修的三类模型偏差（都在"错误路径上的语义"）

| 提交 | 偏差 | 现象 |
| --- | --- | --- |
| `7e19121` | 错误路径上的**非对齐整数 load** 被 bridge 拦成"不执行，rd 不变"；RTL 不处理非对齐，按 `extract_data`（对齐 dword 右移 `8*offset`，按 funct3 扩展）取数写回 | 模型的 `s1` 保持旧值，后面一条 `bne s0` 判成不跳，RTL 判跳并误预测。第一处指纹分歧 406 万 → 6470 万周期 |
| `3724146` | 真实地址非法（模型里 page fault / blocked）但**陈旧基址寄存器给了合法地址**的 load：RTL 真的取数并写回，模型 `plain=false` 不打 taint | `s1` 在 RTL 里是 `…600888`，模型是 `…3160`。6470 万 → 6840 万周期 |
| `cc30321` | 推测 load 的**陈旧值取的是第一次写回时**的内存映像，而 RTL 是**读 DCache 那一拍**（与 L1 CAM 同拍）；更老的 store 在两者之间出了 STQ，陈旧值就等于真值被当成静默 | `ld a5,88(sp)` 在 RTL 里第一次写回 0，模型是真值，后面 `beqz a5` RTL 误预测。s06 检查点窗口复现并修复 |

三处改动之后 67 个程序（默认配置和两个 INT 队列配置）的周期都没有变。

## 诊断开关（默认关闭）

- `BRIDGE_CHECK=1`：FM 的寄存器 = 已提交寄存器 + 链上各条在飞指令的写入，每次派发前检查（8M 次派发 0 违例，说明 bridge 的撤销日志没有问题）；
- `BRIDGE_TRACE=1`：每次派发 / 被拦截的派发；
- `MSD_TRACE=1`：陈旧 load（`[MSL]`）、污染（`[MSD]`）、分支（`[BRU]`）、AGU（`[AGU]`）、误预测目标（`[MP]`）；
- `MSD_DBG_PC=<pc>`：该 pc 的 load 计算陈旧值时 STQ 的视图。

## 当前对齐状态

- Linux 检查点窗口（12 个，各 400 万周期）：s00–s07、s10 **PASS**（PT / UP 指纹相等，逐条函数模型检查 0 不一致）；
- **s08**：已修（见下，PASS，RTL 侧的修复）；
- **s09**：已修（见下，PASS，uBTB 目标全宽，模型和 RTL 都改）；
- **s11**：已修（见下，PASS）；
- 从复位的 1.4 亿周期：第一处 PT 分歧 6840 万周期（PMU 第 136 个区间），UP 指纹到 1.3 亿周期全部相等。

## 工具

`rtl-agu/build/segtools/seg_cmp.sh <seg...>`（RTL 缓存 + 模型）、`drill.sh <seg> <fp0> <fp1> <abs0> <abs1>`（带打印的下钻）。测试台的事件打印在 `rtl-i2`
（`6d7ed85` + `tb/soc/soc_tb.cpp`、`soc_public.vlt` 的打印补丁，不改 RTL）。注意：`rtl-agu` 工作区被另一个会话（fork-2）同时修改，RTL 对照一律用 `rtl-i2` 这个独立副本。

## s11：flush 型提交指令的 FS-dirty 丢失（已修）

用户态 `csrw fflags,sp`（`00171073`）在函数模型里把 mstatus.FS 置为 Dirty，时序模型本应在提交时把 FS/SD 写进自己的 CSR 文件。
CSR 写指令提交时带 flush，`commit_flush` 在决策那一拍就把它的 fact 退休了，之后提交报告里的 `fs_dirty_at_commit(uid)` 找不到 fact，
FS-dirty 丢了。结果：陷入内核后 `csrrc s1,sstatus,t0` 在模型 CSR 文件里读到 FS=Clean（`0x200004020`），与函数模型（`0x8000000200006020`）不同，
`patch_rd` 把错值写回函数模型的 s1，随后 `sd s1,256(sp)` 的数据错，RTL 与模型出现不同的预测更新。修复：`on_flush` 先记下提交指令的 fs_dirty，
`fs_dirty_at_commit` 在 fact 已退休时返回它（bridge/trace_hook.cpp）。s11 窗口 PASS，67 程序周期不变。诊断开关 `REWIND_TRACE=1` 打印 `[REWIND]`、`[PATCH]`、`[FSD]`、`[FSC]`。

## s08：LDQ 应答选择没比 epoch（RTL bug，已修）

RTL 的 `lp_ldq.sv` 对每个 LDQ 项按"最后一个匹配的 load 口"选 DCache 应答，选择时不比 epoch。被冲刷的旧 load（epoch 6）的 HIT 应答和同一
LDQ 位置新分配的 load（epoch 8）的 CONFLICT 应答在同一拍到达（端口 2 对端口 0）时，过期应答赢，`lp_ldq_ent` 再按 epoch 拒绝，新应答丢了，
load 一直停在 DCW（RTL 没有 watchdog）。模型对每个口分别检查 epoch，所以立刻重发，多占一拍 L1 口，之后 replay 相位差 3 拍，队首 load 提交晚 6 拍，
UP 指纹差 1 拍。修复：应答匹配条件里加 epoch 相等（rtl `b69f661`，分支 `exp/ldq-resp-epoch`）。取证用 `SOC_TB_LDQ=1`（逐拍 LDQ 状态、replay 寄存器），
模型 `PARK_TRACE=1`。

## s09：uBTB 目标读出位宽（RTL bug + 模型残留，已修）

uBTB 条目的目标在 RTL 里存 63 位（`UB_TGT_W`），读出却只用低 32 位，在 64 位上下文里左移（低 32 位最高位为 1 时进位到第 32 位），高位取自块地址。
内核目标 `0xffffffff80045212` 从用户块 `2ae50d52e0` 读出得到 `0x2b80045212`（模型 `ub_expand` 截成 32 位得 `0x2a80045212`），S0 推进的 3 位历史符号不同
（RTL 7，模型 5），之后的 UP 的 idx / tag 差 1～2 位。这是被 OVR 覆盖的错误路径 S0 预测，PT 流不受影响。取证用 RTL 波形（`SOC_TB_VCD`，`--trace` 编译）
读 `u_ub_look`，模型 `HP_TRACE=1`（每次推测历史 push 的位置 / pc / 目标 / 位、S0 读的整组 uBTB）。

两边统一为全宽：模型 `UbEnt::target` 改成 `pc_t`、`UB_TGT_BITS = XLEN - SLOT_SHIFT`、`ub_expand` 直接移位；RTL 读出直接用存储的目标（rtl `bpu_ub_look.sv`）。
主 BTB（`bta`）两边本来就是全宽。67 个程序周期不变（低地址程序）。索引 / tag 的哈希输入、`hist_sym`、更新队列窗口比较仍只用低 32 位（哈希宽度，两边一致，不影响值）。

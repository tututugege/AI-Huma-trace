# AI-Huma / trace-model:项目记录

AI-Huma 是整个项目(AI 与人协作;也是对 Rem 魔法的致意),由三部分组成:
- `function-model`(`~/personal/AI-Huma/function-model`):功能级 RISC-V core(rv64;rv32 只在分支 `rv32-legacy`;`XLEN` 是编译期参数,默认 64);
  编成 `librefcpu.a`。这里通过符号链接 `function-model/` -> `../function-model`(AI-Huma 布局中的同级仓库)使用。
- `trace-model`(本仓库):项目的时序模型与探索平台,事实(fact)由 function model 提供。
- `rtl`(`~/personal/AI-Huma/rtl`):SystemVerilog 实现;其配置包由本仓库的 `include/config.h` 生成(`rtl/scripts/gen_cfg.sh`),
  合入 main 时 RTL 必须逐程序对上本模型的周期数。
全项目规则(分支、合并流程、记录、TODO)在 AI-Huma 仓库:`docs/WORKFLOW.md`、`docs/GUIDE.md`;
每次合并的性能/验证表是那边的 `docs/MAIN_RECORDS.md`。本仓库入口:[../README.md](../README.md);
全部运行期环境变量:[ENV_VARS.md](ENV_VARS.md)。

两个阶段。(1) 基础阶段,2026-11-04 完成:从 tututugege/simulator(RTL 风格、execution-driven、cycle-accurate 的 OoO RISC-V)蒸馏出 trace-driven 时序模型,
与原模型周期完全一致,代码量约一半且更快(计划与步骤记录:`docs/history/`)。(2) 此后是我们自己的优化:架构改动先在这里探索
(探索记录 EXP-xxx 在 AI-Huma 仓库的 `docs/EXPLORATION.md`),合入后再搬到 RTL。
正确性:function model 是 trace bridge 的核心并提供每一个值(fact);运行必须跑到程序结尾("Success"),`TB_CHECK=1` 比较模型仍然自己决定的部分(地址、分支、异常)。没有提交级 difftest,也只有一个 function-model core。Gate:周期数等于 `results/baseline/baseline.csv`,
只有合入 main 的有意配置变更才会改它(其余任何改动都必须保持周期数不变)。

## 架构概述
模型不再计算值(ALU/MUL/DIV/FP、PRF、L1D 数据、LLC 数据表都已删除)。分支方向/目标、地址、store 数据和 CSR 写数据来自 trace bridge(`bridge/TraceBridge.*`)提供的 fact;
bridge 背后是跟随模型 dispatch 流(含错误路径)的功能 core(librefcpu,function-model)。功能内存在 ROB commit 时按 fact 写入;
指令/PTE/LLC 读数据来自 pmem。

当前微架构(`include/config.h` 的默认值;RTL 包取相同的值):
- 前端(`front-end/`):8 宽取指,流水 BPU(S0 uBTB、S1、S_D;TAGE/ITT/L1 预测器、update queue;`BPU_PIPE=3`,源码注释在
  `front-end/BPU/BPU_pipe.h`,RTL 描述 `rtl/docs/analysis/bpu-pipe-rtl.md`),取指地址 FIFO / PTAB / 指令 FIFO 各 16 项,pre-IDU buffer,
  32 KiB 8 路 I-cache(64 B 行),redirect 快速路径 `CFG_FE_RFAST` 15(EXP-019/023;`CFG_FE_BFAST` 0)。
- 后端(`back-end/`):8 宽 decode / rename / dispatch / commit,move elimination(见下),`CFG_PRF_NUM` 128,`CFG_ROB_NUM` 128 且为 compact ROB
  (`CFG_ROB_COMPACT` 1,EXP-028),`CFG_MAX_BR_NUM` 32,发射队列 INT 64 / LD 64 / STA 64 / STD 16 / BR 32,入队宽度 7/6/4/4/5(EXP-017)。
- 发射口:6 个整数 ALU 口(其中 4 个带 MUL/DIV/FP/CSR 能力,`CFG_PLAIN_ALU_PORTS` 2 个为纯 ALU),`CFG_BR_PORTS` 2,`CFG_LD_PORTS` 4,`CFG_STA_PORTS` 3,`CFG_STD_PORTS` 3。
  乘法器 2 周期(XLEN=64 为 3),除法器 16(32)。
- LSU(`back-end/Lsu/RealLsuPipe.cpp`,[LSU_PIPE.md](LSU_PIPE.md)):分级 load/store 流水(唯一的 LSU;`RealLsu.cpp` 放它的 setup),LDQ/STQ 各 64 项,访存依赖推测(`CFG_LSU_MEMSPEC` 1)与值通路 `CFG_LSU_MS_DATA` 1(wait-table 预测器
  `CFG_LSU_MDP_BITS` 6),4 个 replay 寄存器,已提交 store 经 4 项 write buffer 排空(`CFG_LSU_PIPE_WBUF` 4,EXP-024)。
- 存储系统(`MemSubSystem/`、`axi-interconnect-kit/`):64 KiB 4 路 L1D(64 B 行,8 个 MSHR)配 L1D 预取器 h1(`PF_ENABLE` 1),共享 LLC
  `CONFIG_AXI_LLC_SIZE_BYTES` 4 MiB 16 路,8 个 MSHR,查找延迟 3,SimDDR 之后是 DRAMsim3 DDR4-3200(EXP-006)外加 60 ns 的固定非核延迟,核频 1 GHz,
  无 AXI 子系统分频(`CONFIG_AXI_SUBSYSTEM_FREQ_DIV` 1)。可缓存区:0x40000000-0x4fffffff 与 0x80000000-0xffffffff,其余都是 MMIO。
- PMU / TMA(`back-end/Pmu*.cpp`,`PMU=1`,EXP-013;AI-Huma 仓库的 `docs/PMU.md`)。

## 配置开关(include/config.h)
尺寸是 `CFG_*` 宏,可用 `-D` 覆盖(`gmake EXTRA_CXXFLAGS="-D..." BUILD_DIR=build_area/<name>`);RTL 包生成器
(`rtl/scripts/gen_cfg.sh`)用相同的默认值并接受相同的覆盖。多数开关还有运行期环境变量覆盖,见 [ENV_VARS.md](ENV_VARS.md)。`XLEN` 由 Makefile 设置(默认 64:`gmake XLEN=64`)并导出为 `XLEN_BITS`。
- 宽度与窗口:`CFG_BACKEND_FETCH_WIDTH`(8)、`CFG_MAX_BR_NUM`(32)、`CFG_ROB_NUM`(128)、`CFG_PRF_NUM`(128)、`CFG_IBUF_SIZE`(64);
- ROB 组织:`CFG_ROB_COMPACT`(1;仅编译期)——0:每个 8 项 ROB 行放一个 dispatch 组,整行一起提交;1:compact ROB(EXP-028):在同样的 8 bank x 16 行上顺序分配,
  从队头起最长的已完成前缀可提交(最多 8 条),并受已寄存的 BPU commit credit 限制
  (BPU update-queue / uBTB train-queue 占用减去上一拍的提交数);flush、fence、WFI 和中断在队头单独提交;
- 发射队列:`CFG_INT_IQ_SIZE` 64、`CFG_LD_IQ_SIZE` 64、`CFG_STA_IQ_SIZE` 64、`CFG_STD_IQ_SIZE` 16、`CFG_BR_IQ_SIZE` 32(也导出为 `IQ_*_SIZE`);
  入队宽度 `CFG_INT_IQ_ENQ_WIDTH` 7、`CFG_LD_IQ_ENQ_WIDTH` 6、`CFG_STA_IQ_ENQ_WIDTH` 4、`CFG_STD_IQ_ENQ_WIDTH` 4、`CFG_BR_IQ_ENQ_WIDTH` 5;
- 发射口:`CFG_PLAIN_ALU_PORTS` 2(排在四个 ALU+MUL/DIV/FP/CSR 口之后)、`CFG_LD_PORTS` 4、`CFG_STA_PORTS` 3、`CFG_STD_PORTS` 3、`CFG_BR_PORTS` 2;
- 队列:`CFG_LDQ_SIZE` 64、`CONFIG_STQ_SIZE` 64;D-cache `CFG_DCACHE_MSHR` 8;
- move elimination:`CFG_MOVE_ELIM` 1、`CFG_MOVE_ELIM_REFS` 7(见下);
- LSU:`CFG_LSU_PIPE` 1、`CFG_LSU_PIPE_CUT` 127、`CFG_LSU_PIPE_L2` 115、`CFG_LSU_MEMSPEC` 1、`CFG_LSU_MS_DATA` 1、`CFG_LSU_MDP_BITS` 6、`CFG_LSU_MDP_CLR` 2000、
  `CFG_LSU_MDP_MODE` 0、`CFG_LSU_MDP_HW` 2、`CFG_LSU_MS_SLOTS` 0、`CFG_LSU_PIPE_WBUF` 4,以及 `_WBMAX` 2、`_WBINS` 4、`_WBSCAN` 4、`_DRAIN` 2、`_DCAND` 4、`_RP` 4;
- 前端:`CFG_FE_RFAST` 15、`CFG_FE_BFAST` 0;FIFO 在 `front-end/config/frontend_feature_config.h`:`FRONT2BACK_FIFO_SIZE`、`PTAB_SIZE` 16、`INSTRUCTION_FIFO_SIZE` 16、`FETCH_ADDR_FIFO_SIZE` 16;
- 存储:`CONFIG_CPU_FREQ_MHZ` 1000(`gmake CPU_FREQ_MHZ=...`)、`CONFIG_AXI_LLC_*`、`CONFIG_SIM_DDR_LATENCY` 60(固定非核延迟,单位 ns:SOC 20 + CDC 12 + CTL 15 + PHY 13;运行期可用 `DS3_EXTRA_NS` 覆盖)。

## 来源与历史
- 从 tututugege/simulator 的 2f31e8d 分叉(`baseline.lock`)。这里的 `main` 就是原来的 `trace-distill` 分支。
- `axi-interconnect-kit/`(来自 dyh1807/axi-interconnect-kit,本地提交到 382cb6f)以普通目录方式 vendored。`function-model/` 是指向 `../function-model` 的符号链接
  (与本仓库同级检出的 function-model 仓库;2026-11-04 之前它是 git submodule)。
- 合并前的 patch-queue 仓库(补丁 0001..0061、步骤文档)是 `../tututugege-trace`(如果存在);其文档在 `docs/history/`,
  其测试/脚本在 `tests/` 和 `scripts/`。
- 原始模型保留为基线可执行文件(`~/personal/llm4trace/third_party/tututugege/baseline-sim/src`,2f31e8d + 精确的 kit 补丁);
  2026-11-04 原模型的周期数已不再是目标(每次合入 main 的配置变更都会重新生成基线,见 `docs/MAIN_RECORDS.md`)。

## rv64(EXP-029,2026-11-12 合入)
`XLEN` 是 function-model、trace-model 和 RTL 包共同的编译期参数(`XLEN_BITS`;现在默认 64)。
`XLEN=64` 覆盖 rv64imab:寄存器宽度的 PC 与地址、Sv39 页表遍历 / TLB、64 位 CSR、8 字节访存、OP-IMM-32 / OP-32,乘法器 3 周期、除法器 32 周期(`*W` 除法 16);
`addiw rd, rs, 0` 不是 move(不做消除)。构建:见 [../README.md](../README.md)。通过 RTL gate 验证(cpu-tests、Linux 到 `rv64 login:`;`docs/MAIN_RECORDS.md`,2026-11-12)。
未做:SPEC rv64 流程。FastWarm 重放的 TLB 只支持 Sv39(不再有 Sv32);重放期间参考核打开设备和中断投递并推进 mtime(否则内核收不到时钟中断,重放出的是另一条指令流,预热结果完全不对)。验证(2026-11-19,同一二进制):本地 rv64 Linux 检查点 +0.7% 到 +6.1%;eda 上 SPEC06 rv64gc 的 bzip2 / gcc / mcf / hmmer 各 2 个点,fast-warmup 与精确预热(10M 详细预热)的周期差在 -1.1% 到 +0.5%。

## Gate(每次改动后运行)
所有脚本都以仓库根作为 `<simulator-repo>`,例如 `scripts/run_gate.sh build/simulator results/gate`。
1. `gmake LDFLAGS="-lz" -j4`(macOS 没有 -lstdc++fs,所以要覆盖 `LDFLAGS`)。DRAMsim3 默认开启(EXP-006):它是固定版本的 git submodule `third_party/DRAMsim3`(克隆后执行 `git submodule update --init third_party/DRAMsim3`),由 Makefile 编进 `build_dramsim3/libdramsim3.a`(`gmake dramsim3`;不用 cmake,没有动态库)。`gmake DS3=0` 构建旧的固定延迟 SimDDR。核频:`gmake CPU_FREQ_MHZ=...`(默认 1000)或运行期的 `CPU_FREQ_MHZ` 环境变量。改过编译选项后用干净的 `BUILD_DIR`。
1b. 新检出后先 `gmake -C function-model librefcpu.a LDFLAGS="-lz"`(构建 `../function-model`;构建产物,不入库)。需要 function-model 仓库与本仓库同级。
2. `scripts/build_tests.sh . build/tests`,再加 `scripts/build_tests.sh . build/tests memory-stress-small smc-addi smc-div pf-stress`
   (Homebrew LLVM,riscv64;额外程序默认不构建)。
3. `scripts/run_gate.sh build/simulator results/gate`:62 个测试(cpu-tests 与 `tests/extra` 的额外程序;`memory-stress`、`load-store` 和 `smc-*` 被脚本排除,见其 `grep -v`;`baseline.csv` 也列出它们),通过 = 周期数等于 `results/baseline/baseline.csv`
   (2026-11-19 以新默认配置(= RTL 配置:两个 INT 队列、合并访存队列、`FLUSH_REG` 等 13 个 `CFG_*` 默认值,EXP-044)重新生成,62 个程序周期都变了,几何平均 +1.96%;更早:2026-11-11 以 compact ROB(EXP-028)为默认,2026-11-09 带 write buffer(EXP-024);`baseline_nopf.csv` 是同一基线的关预取版本:`PF_ENABLE=0` 可复现,2026-11-19 同时重新生成)。
   全部以 difftest Success 结束,TB_CHECK 不一致数为 0。gate 不需要 `LSU_PIPE_NOWD=1`(基线就是模型自己的周期数);与 RTL 做任何周期比较时才需要。
   **旧设计回归**(旧 LSU + 旧 BPU):两者都已删除(BPU_PIPE 0..2 在 exp/rvc-carry,旧 LSU / `LSU_PIPE=0` 与 `legacy_lsu_baseline*.csv` 于 2026-11-15 删除);要用请查历史。
4. 大程序(源码和二进制在 `AI-Huma/rtl/programs/big`;clang + 桩 libc 头文件;缩短版构建自 2026-11-04 起固定为标准)。**当前默认值(2026-11-12,开启 compact ROB EXP-028;`docs/MAIN_RECORDS.md`):**
   dhrystone 64 次 116,774;coremark 2 次迭代 284,902;memory-stress 2,308,184;smc-div 10,687;smc-addi 10,176;memory-stress-small 325,458;pf-stress 621,069;itt-dispatch 222,439。
   行式 ROB(`-DCFG_ROB_COMPACT=0`):dhrystone 114,685;coremark 283,240;memory-stress 2,358,268;smc-div 10,875;smc-addi 10,315;memory-stress-small 329,424;pf-stress 630,916;itt-dispatch 222,445
   (EXP-028 之前的 baseline.csv;`software-interrupt-vectored` 2254 -> 2356 来自被删除的 uncached 窗口)。
   更早的配置列在 `docs/MAIN_RECORDS.md`(每次合并一行);它们的开关(`LSU_PIPE_WBUF=0 LSU_PIPE_DRAIN=3` = EXP-024 之前,`LSU_MEMSPEC=0 FE_RFAST=0` = EXP-019 之前,`LSU_PIPE=0` = 旧 LSU)仍然可用,但数字已不能精确复现。
   这些周期数以 EXP-006 的存储系统为前提:DRAMsim3 DDR4-3200 加 60 ns 固定非核延迟,核频 1 GHz。
5. 小 LLC 检查(`scripts/llc_small_check.sh $PWD <baseline-sim/src 的绝对路径>`(必须用绝对路径)):默认 4 MiB LLC 在小测试里从不发生替换,所以脚本用 64 KiB LLC 构建两个模拟器,在会发生替换的程序上比较周期。
   (它与原模型比较,因此只有在 trace-model 运行的配置与原模型一致时才吻合;把它当作 DDR 写 / victim 写回路径的检查工具,而不是 gate。)
   `load-store` 显示 "base= FAIL",因为原模型因 "Misaligned Load" 中止(历史遗留);gate 里用 `load-store-aligned`(tests/extra,用对齐的字节 load 取代未对齐的 `lw`)代替它。

## 验证手段
- 周期 gate(`scripts/run_gate.sh` 配 `baseline.csv`,`scripts/run_gate_c.sh` 配 `baseline_c.csv`)是回归检查:凡是要保持时序的改动,
  每个程序的周期都必须完全一致。逐模块的 lockstep shadow 以及与原模块冻结副本的随机 differential(`tests/*_ref`、`<MOD>_SHADOW` 构建、`*_diff`)
  曾证明每个模块的重写正确,已于 2026-11-15(exp/tm-cleanup)删除;它们在那次提交之前的历史里。
- 对照 function model 与 RTL:`TB_CHECK=1`(值比较,`scripts/check_run.sh`)、`TB_TRACE_COMMIT=1` 配 RTL 的 `SOC_TB_COMMIT`、`LSU_PIPE_EVT=1` 配 `SOC_TB_LSUEVT`(见 [ENV_VARS.md](ENV_VARS.md))。
- 教训:测试覆盖不到许多 trap / S 态 / FTQ 满的路径,所以 lockstep 要配随机激励 harness。
- 只按覆盖率删代码的做法被否决了(它曾把 DDR 写路径删掉);应优先重写加 differential 测试。

## 状态
2026-11-03 的蒸馏数字(84k -> 43k 行、提速手段)在 `docs/history/`。当前状态:见 `docs/MAIN_RECORDS.md`(最新几行)以及 AI-Huma 仓库的 TODO / EXPLORATION。
蒸馏以来没动过:MemSubSystem 外设(PeripheralModel.h、MemPtwBlock.h;没有测试覆盖)、旧格式的 checkpoint 恢复。

## 常见坑
- 时序对布局敏感;每次改动跑一次时序就够(与已知数字比较),以周期为准。
- zsh 不对未加引号的变量做分词:用 bash 脚本。
- BSD sed 没有 `\|`;本环境里 `git commit` 需要显式指定身份。
- 长时间运行放后台并设置 `SIM_MAX_CYCLES`。
- 与 RTL 比较周期需要 `LSU_PIPE_NOWD=1`。
- 本地批量仿真:最多 4 个并行任务(每个模拟器最多占约 3 GB RSS)。

## Move elimination(EXP-012;config.h 里的 `CFG_MOVE_ELIM`、`CFG_MOVE_ELIM_REFS`)
- `addi rd, rs, 0`(rd、rs 为真实寄存器)在 Ren 里被改名消掉:rd 直接取 rs 的物理寄存器,不占 free-list 槽位,不产生 uop,
  ROB 项在 dispatch 时即完成。当没有任何 arch_RAT 项指向某物理寄存器时才释放它(`arch_cnt`,在 commit 时计数,
  因此 flush 时从不需要回滚计数器)。恢复机制不变:被消除的 move 不分配任何东西,所以 free-list
  头部 checkpoint 与 `free_head_commit`(跳过被消除的 move)保持一致。
- 计数器上界(`Ren::elim_refs_ok`):只有当 move 的源直接来自推测 RAT(同组内没有更老的指令写它),并且(指向它的推测 RAT 项数)+(同组内更老的有目的寄存器的指令数)+ 1 <= `CFG_MOVE_ELIM_REFS`
  (默认 7 = 3 位计数器)时才消除。move 提交后的体系结构状态等于它 rename 之后立刻的推测 RAT,所以体系结构计数不会超过上界。
- `ME_CHECK=1`(运行期环境变量):每个周期检查 arch_cnt 等于指向各寄存器的 arch_RAT 项数,且没有物理寄存器泄漏;打印见到的最大引用数。
- rv64:`addiw rd, rs, 0` 是符号扩展而不是 move,不消除。
- 未处理:`add rd, rs, x0`(工具链不会生成压缩的 `c.mv`)。带目的寄存器的提交与 mispredict 在同一周期发生时,在 ME 模式下会断言
  (基线的 free-list 自检已把它视为不可达;gate、大程序、Linux 和 SPEC quick 里从未触发)。

## F/D 浮点(EXP-030;默认开启;Zfinx 已删除)
F/D 是默认构建(`FP=1`;function-model 的 `librefcpu.a` 必须用相同的 `FP` 构建:`RefCpuState` 带 `fpr[32]` 和 `fcsr`)。`FP=0` 是完全没有浮点的 core(也没有 Zfinx:旧的 `UOP_FP` / `FuKind::FPU` 路径以及测试 `zfinx*`、`matmul_fpu` 都已删除;`tests/extra/fd-*` 是它们的 F/D 版本)。整数代码在有无 F/D 时周期完全相同。
rv32:FLEN 也是 64;`fld` / `fsd` 是 8 字节访存(LSU 的宽度辅助函数在 F/D 下对 funct3 3 返回 8),`fcvt.l/lu` 和 `fmv.x.d/fmv.d.x` 不存在(function model 会 trap 它们,所以模型也如此),`mstatus.SD` 是第 31 位。
- **寄存器与 tag。** 独立的 FP 物理寄存器堆(`CFG_FPRF_NUM` 96,体系结构寄存器 32),`Ren` 里有 FP RAT / free list / 每个分支的 RAT checkpoint。模型里物理寄存器 tag 是同一个数字空间(整数 `0..PRF_NUM-1`,FP `FP_TAG_BASE..`;`TAG_NUM`、`PRF_IDX_WIDTH` 随之),所以发射队列项、
  dispatch busy 表、唤醒总线和访存推测的 taint 表都不变;RTL 有两个寄存器堆、一个域位和分域的唤醒总线。
  `UopS` 有 `dest_fp` / `src{1,2,3}_fp`、`src3_en/areg/preg`(融合乘加)、`fp_cls`。`InstType::FP` 改名为 `FPOP`(现在这个名字是构建宏)。
- **发射。** 新增 `IQ_FP`(`CFG_FP_IQ_SIZE` 32,入队宽度 4),带第三个唤醒矩阵;三个 FP 发射口(FMA+MISC、FMA+MISC、FDIV+MISC;每个单元是独立的 FU,
  所以长除法不会阻塞其他单元),以及整数口 2 和 3 上的 `I2F`。单元:`UOP_FMA`(fadd/fsub/fmul/fmadd 系列,流水)、`UOP_FMISC`(fsgnj、
  fmin/fmax、fcmp、fclass、FP 内部的 fcvt、FP -> 整数的 move/转换,流水)、`UOP_FDIV`(fdiv/fsqrt,迭代,延迟按 S/D)、`UOP_I2F`(整数 -> FP)。
  `CFG_FP_*_LAT` 是**相关指令的发射间隔**(4 = 相关的 fadd.d 在前一条之后 4 个周期发射;延迟唤醒在单元延迟之外再多花一个周期,与 MUL/DIV 相同,所以模型延迟是 CFG - 1):FMA 4、MISC 3、I2F 3、fdiv S/D 12/20、fsqrt S/D 14/24(起始值,未校准)。
- **访存。** `flw/fld` 是 load(LD 队列,FP 目的 tag),`fsw/fsd` 是 STA(整数基址)+ STD,STD 的数据源是 FP tag(目前所有 STD 口都能接收;
  原计划只用 0 号口,留给 RTL 决定)。LSU 没有任何改动。
- **Trap 与状态。** 什么会 trap(mstatus.FS = Off、保留的舍入模式……)由 function model 裁决:fact 标明"trapped, cause 2"的已执行 FP 类 uop
  变成非法指令异常(`Fu::fp_trap`);静态保留编码在 decode 阶段 trap。mstatus.FS = Dirty / SD 与 misa F/D 在 `Csr` 里建模
  (fact 表明把 FS 变成 Dirty 的已提交指令,在该周期末置位)。fflags / frm / fcsr 的 `csrr`/`csrw` 仍是 NOP,没有
  串行化(模型不计算值,所以动态舍入模式没有效果;fcsr 写通过同一个 fact 弄脏 FS)。
- **接口。** `Fact` 增加 `rd_is_fp`、`fs_dirty`、`trap_cause`;bridge 解码 opcode 0x07 / 0x27(地址、大小、store 数据取自 `fpr`),撤销 FP 寄存器和 fcsr 写,
  `FastWarm` 重放 FP load/store,checkpoint 格式 5(`CPU_state.fpr`,其后是 fcsr)由 function model 读取(`refcpu_restore_checkpoint`,由 `driver/Load.cpp` 调用)。
- **检查。** `scripts/fp_check.sh <sim> <workdir>`:function model 的定向 FP 测试、trap/状态测试、编译器生成的 kernel 和 23 个官方 riscv-tests
  `rv64uf`/`rv64ud` 都跑到结尾(difftest Success,每条指令都提交);`tests/fp/fpmicro.c` 微基准显示配置的延迟(4 条相关 fadd.d:16 周期,
  8 条独立:4 周期 = 两个 FMA 流水都用上,相关 fdiv.d 20,fcvt.l.d + addi + fcvt.d.l 7)。小配置 `CFG_FPRF_NUM=36`、`CFG_FP_IQ_SIZE=8`、`CFG_FP_FMA_LAT=8` 下无死锁运行
  (FP free-list 不变式每周期断言)。`TB_TRACE_PATCH=1` 打印模型打补丁回 function model 的每一次 CSR 读(FP 测试里为 0)。
- **未做。** 串行化的 fcsr/frm 访问、FP 域的 move elimination、除定向测试外的 FS-off trap 覆盖、RTL、校准的延迟、用于调这些开关的 FP workload(PolyBench / SPECfp)、FP 消费者的 `MsData` taint(未建模)。

### FP 寄存器堆端口研究(EXP-032,分支 `exp/fp-wb`)
`FP_WB_PORTS=<n>`(环境变量,默认 2,0 = 不限):每周期 FP 单元(FMA、带 FP 目的的 MISC、FDIV、I2F)最多写 n 次 FP 寄存器。写周期在发射时已知
(`issue + latency`),所以 `Isu::comb_issue` 在一张 busy 表里预留它(`back-end/include/FpStats.h`,类似 XiangShan 的 WbFuBusyTable;一个 `IssueQueue::grant_gate` 钩子),
发现该周期已满的 uop 留在发射队列里(被 squash 的 uop 的预留不释放,略偏悲观)。FP load 通过 LSU 写,不受限制,只计数。退出时的统计 `[FP_PORTS]`:每周期 FU 写次数、每周期 FP load 写回次数、
FP 队列已发射 uop 每周期的 FP 读次数、每周期 FP store-data 读次数。`scripts/fp_bench.sh <sim> <work> [ports...]` 构建 `tests/fp/fpbench.c` 的六个 kernel 并扫描该限制。

## rv32 已移除(2026-11-16)

main 只支持 rv64。`XLEN` 默认 64(Makefile、`include/config.h`),测试脚本默认 `riscv64 / rv64ima / lp64 / medany`,四个 Sv32
分页测试(`baremetal/vm-*`)及其脚本、rv32 预编译 ELF 和旧 LSU 基线都已删除,`results/baseline/baseline.csv`、
`baseline_nopf.csv`、`baseline_c.csv` 已在 rv64 上重新生成(62 / 62 / 67 个程序,全部 difftest Success;`fdoff`、`matmul_soft` 和 `string` 不属于
gate)。`rv32-legacy` 分支保留之前的 main。上面各文档里引用的旧 rv32 周期数是历史数据。

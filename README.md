# trace-model

AI-Huma 项目(function-model + trace-model + rtl)的时序模型与架构探索平台。
它是 cycle-accurate、RTL 风格的乱序 RISC-V core 模型(8 宽前端 / decode / commit,128 项 ROB,分级 LSU,
流水 BPU,L1I/L1D,共享 LLC,DRAMsim3 DDR)。想法先在这里测量(周期、IPC、队列/端口使用、经 PMU 的 TMA),
再搬到 RTL;合入 `main` 时 RTL 必须逐程序复现本模型的周期数。

模型不计算值。分支方向/目标、地址、store 数据和 CSR 写数据来自 trace bridge(`bridge/TraceBridge.*`)提供的 fact;
bridge 背后是跟随模型 dispatch 流(含错误路径)的功能 core(同级的 `function-model` 仓库,编成 `librefcpu`;
这里的 `function-model/` 是指向 `../function-model` 的符号链接)。
目录布局:时序模型是 `front-end/`、`back-end/`、`MemSubSystem/`、`axi-interconnect-kit/` 和 `SimCpu.cpp`(core 顶层,`SimCpu::cycle`);
`bridge/` 是它的输入(trace bridge 以及模型接入它的 tap);`driver/` 负责运行(命令行与运行模式在 `main.cpp`,镜像 / checkpoint 加载在 `Load.cpp`(checkpoint 文件由 function model 读取),
参考 core 的设置与 prewarm 在 `RefCore.cpp`,cache / TLB / BPU 的功能性预热在 `FastWarm.cpp`)。
每次改动都必须保持 gate 的周期数(`results/baseline/baseline.csv`),除非是合入 main 的有意配置变更。全项目规则(分支、合并流程、`MAIN_RECORDS.md`、`EXPLORATION.md`、TODO)
在包含这三部分的 AI-Huma 仓库(`docs/WORKFLOW.md`、`docs/GUIDE.md`)。

ISA:rv64imafd(F/D 自 2026-11-12,EXP-030;Zfinx 已删除;`RVC=1` 加入 C,EXP-038);`FP=0` 构建无浮点的 core。rv32 已从 main 移除(分支 `rv32-legacy`);源码里残留的 `XLEN==32` 条件是死代码。未做:C,SPEC rv64 流程,
FastWarm 重放里的 Sv39。

## 建模方式

* **只算时序，不算值。** 指令的结果、分支方向和目标、访存地址、store 数据、CSR 写入值都是功能模型给出的事实（trace bridge）。
  模型只决定"哪一拍发生什么"。例外是 RTL 的时序确实依赖、而事实给不出的值：访存推测时 load 读到的旧值（LSU_MS_DATA，
  `back-end/include/MsData.h`），以及设备寄存器的值（外设模型，`IoBacking`）。
* **形状照着 RTL。** 模块和 RTL 的模块一一对应，信号有声明的位宽（`wire<N>` / `reg<N>`）；合入 main 时 RTL 必须逐程序复现本模型的周期数。
  整理代码时周期必须完全不变（门禁、FPR、TB / LSU 逐拍轨迹、Linux 检查点和 Linux 窗口逐字节比较）。
* **一拍 = comb 阶段 + 时钟沿。** comb 阶段的顺序由各模块声明的读写关系自动生成（`SimCpu::cycle()` 的生成块），
  寄存器只在时钟沿更新。详见 [docs/SCHEDULING.md](docs/SCHEDULING.md)。
* **uop 只带 id。** 一条指令的静态和只写一次的字段（译码结果、物理寄存器、ROB / LSQ 位置）在 uop 池 `uop_pool[id]`，
  流水线里传的是 id 加上每份拷贝自己的动态位（busy、`br_mask`、`dest_en`、结果）。
* **CPU 级共享状态。** CSR 文件是 `SimCpu::csr`，全 CPU 通过 `csr_state()` 只读访问、只在时钟沿改；功能内存就是功能模型的 RAM
  （`driver/RefCore.cpp`），模型自己只保存设备寄存器（`IoBacking`）。

## 代码风格

* **模块的统一写法**：`struct In`（只读的 const 指针）、`struct Out`（自己驱动的信号），头文件里每个 comb 阶段一行
  `CONTRACT`（用自己的端口名写读写哪些端口），父模块一行绑定一个端口。工具检查契约和代码是否一致，并据此生成调度。
* **comb 算输出和线信号，seq 原地更新寄存器**：comb 只读寄存器、写自己的输出和几个具名的线信号（成员名以 `_` 结尾），
  seq 用这些线信号和本拍输入原地改寄存器。D-cache、MSHR、写缓冲、PTW、LSU（`pcur_` / `pnxt`）和 FrontControl
  仍是 q / d 两份，因为它们的 d 在同一拍里被多个步骤读写；BPU 的寄存器在 `comb_front` 里原地写（这一拍之后没有阶段再读它）。
* **每拍默认值只清 valid / 使能位**，其它字段只在 valid 下读；写 valid 的一方写满整个条目。
* **没有文件级的可变状态**，例外和它们为什么顺序安全列在 docs/SCHEDULING.md 第 5 节；环境变量只经 `sim_env()` 读。
* **命名**：类型 CamelCase，函数和变量 snake_case，comb 阶段叫 `comb_<做什么>`，时钟沿叫 `seq`，辅助函数按它算什么命名。
* **错误**：模型不变量用 `Assert` / `fatal`；代码注释用英文，项目文档用中文；不留注释掉的代码。

完整规则：[docs/STYLE.md](docs/STYLE.md)。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/PROJECT.md](docs/PROJECT.md) | 架构、配置开关(`include/config.h`)、gate、验证手段、常见坑 |
| [docs/SCHEDULING.md](docs/SCHEDULING.md) | 一拍的调度：模块契约、自动生成的 `SimCpu::cycle()`、改了模块之后怎么做、工具看不见的共享状态 |
| [docs/STYLE.md](docs/STYLE.md) | 代码风格的完整规则 |
| [docs/NEXT_STATE.md](docs/NEXT_STATE.md), [docs/MODULE_CLEANUP.md](docs/MODULE_CLEANUP.md), [docs/UNIFIED_MODULES.md](docs/UNIFIED_MODULES.md) | 现行模块结构的由来：统一写法、下一状态放到 seq、各模块的整理 |
| [docs/ENV_VARS.md](docs/ENV_VARS.md) | 全部运行期环境变量及默认值(**与 RTL 比较周期时用 `LSU_PIPE_NOWD=1`**) |
| [docs/LSU_PIPE.md](docs/LSU_PIPE.md), [docs/LSU_PIPE_IPC.md](docs/LSU_PIPE_IPC.md) | 分级 LSU:规格、近似、结果、IPC 研究 |
| [docs/history/](docs/history/) | 历史记录:2026-11-04 的蒸馏(P0-P3)、FRONTEND_NOTES / INTEG_NOTES / MS_DATA_NOTES(前端快速路径、memspec 集成、memspec 值通路);可能已过时 |
| [front-end/](front-end/), [axi-interconnect-kit/](axi-interconnect-kit/) | 前端编码规范,vendored 的 AXI/LLC/DDR kit |

## 构建

需要 C++20 编译器、zlib,以及与本仓库同级的 `function-model` 仓库(`function-model` 符号链接)。DRAMsim3 是 git
submodule `third_party/DRAMsim3`(`git submodule update --init third_party/DRAMsim3`);Makefile 把它编进
`build_dramsim3/libdramsim3.a`(`gmake DS3=0` 改用旧的固定延迟 SimDDR)。

rv64(main 上唯一的配置;rv32 在分支 `rv32-legacy`):

```bash
gmake -C function-model librefcpu.a RVC=1 LDFLAGS="-lz"   # 构建 ../function-model(构建产物,不入库);RVC 要与 trace-model 一致(默认 1)
gmake -j4 LDFLAGS="-lz"                               # build/simulator;macOS 没有 -lstdc++fs,所以要覆盖 LDFLAGS
```

寄存器宽度是两个仓库共同的编译期参数(`XLEN`,默认 64);目标文件不带它的标记,
所以切换 `XLEN` / `FP` / `RVC` 之后,要在 function-model 的另一份拷贝里构建 `librefcpu.a`,并使用单独的 `BUILD_DIR`。

改过编译选项后用干净的 `BUILD_DIR`。配置宏(`CFG_*`,见 `include/config.h`)用
`gmake EXTRA_CXXFLAGS="-DCFG_...=..." BUILD_DIR=build_x` 覆盖。核频:`gmake CPU_FREQ_MHZ=...` 或环境变量 `CPU_FREQ_MHZ`。

## 运行

```bash
./build/simulator path/to/program.bin                  # 裸二进制加载到 0x80000000,结束时打印 sim-time(cycle)= ...
PMU_OUT=out/prog ./build/simulator prog.bin            # PMU 默认开:TMA / 事件统计,写 <PMU_OUT>_*.csv(默认当前目录 pmu_*.csv);PMU=0 关掉
LSU_PIPE_NOWD=1 ./build/simulator prog.bin             # 周期可与 RTL 比较
SIM_MAX_CYCLES=1000000 ./build/simulator prog.bin      # N 个周期后停止(退出码 3);任何可能挂死的运行都要设
./build/simulator --mode ckpt checkpoint/mcf/ckpt_sp1.gz   # SimPoint checkpoint(./build/simulator -h)
```

全部变量:[docs/ENV_VARS.md](docs/ENV_VARS.md)。

## Gate(每次改动后运行)

```bash
scripts/build_tests.sh . build/tests                   # Homebrew LLVM,riscv64;额外程序:... build/tests memory-stress-small smc-addi smc-div pf-stress
bash scripts/run_gate.sh build/simulator results/gate  # 并行;通过 = 周期数等于 results/baseline/baseline.csv
```

`scripts/run_gate.sh` 接受模拟器、输出目录和可选的任务数(默认 8;本地最多用 4),任何失败都以非零退出;`BASELINE=<csv>` 选择另一个基线。`scripts/run_gate_on.sh <sim> <out> . [jobs]` 只检查 difftest Success(用于改变时序的配置),`scripts/check_run.sh`
以 `TB_CHECK=1` 运行程序,`scripts/gate.sh` 把一组二进制的结果跑进 csv,`scripts/compare.sh` 比较两个结果目录。细节(旧设计回归开关、大程序周期数、小 LLC 检查、lockstep shadow 与 differential 测试):
[docs/PROJECT.md](docs/PROJECT.md)。

### C 扩展(`RVC=1`,EXP-038)

`RVC=1`(2026-11-19 起默认,与 RTL 一致)让前端按 2 字节位置取指(每 32 字节块 16 个槽)并接受 16 位压缩指令;`RVC=0` 是旧的 4 字节前端,
即使程序里没有压缩指令周期也不同。它需要用相同的 `XLEN`、`FP`、`RVC` 从 `function-model` 构建的 `librefcpu.a`(`make FP=1 RVC=1 librefcpu.a`)。
`results/baseline/baseline.csv`、`baseline_nopf.csv` 是 RVC=1 构建在 `build_tests.sh` 程序(rv64ima,不含压缩指令)上的周期,`baseline_c.csv` 是 C 编译的程序。

```bash
make BUILD_DIR=build_rvc REFCPU_LIB=<用 RVC=1 构建的 librefcpu>
scripts/build_rvc_tests.sh                                           # build_c/tests(用 C 编译的 cpu-tests),build_c/gen(function-model 生成的 C 压力测试)
bash scripts/run_gate_c.sh build_rvc/simulator results/gate_c        # 周期数等于 results/baseline/baseline_c.csv;FULL=1 加上长的生成测试
```

设计(XiangShan 风格,见顶层仓库 `docs/EXPLORATION.md` 的 EXP-038):逐槽预解码并做 RVC 展开,指令起点来自两条
链(`start_a` / `start_b`),由上一组的"最后半字"进位选择,带空洞的有效槽掩码,每条指令向后端传 `is_rvc`。
`RVC_DBG=1` / `BPU_OVR_DBG=1` 打印预解码 flush / uBTB 覆盖的诊断信息。

## 与其他仓库的关系

- `function-model`:功能 core `librefcpu`(rv64,XLEN 编译期确定),fact 的来源。
- `rtl`:SystemVerilog 实现。它的配置包由 `include/config.h` 生成(`rtl/scripts/gen_cfg.sh`,
  默认值与覆盖方式相同);RTL 的 `fm_gate.sh` 把它的周期与本模型的比较(使用 `LSU_PIPE_NOWD=1`)。
- AI-Huma(上层仓库):`docs/MAIN_RECORDS.md`(每次合入 main 的性能与验证),`docs/EXPLORATION.md`(EXP-xxx)。

## 来源

从 tututugege/simulator 的 2f31e8d 分叉(`baseline.lock`),2026-11-04 蒸馏成与原模型周期完全一致的 trace-driven 模型
(`docs/history/`);此后模型是我们自己的设计。上游 README、原(execution-driven)模块的设计文档、上游 CI 与 review 工作流、SPEC 打分脚本和 vendored 的 riscv-tests
副本都已于 2026-11-04 删除;它们保留在 git 历史里。

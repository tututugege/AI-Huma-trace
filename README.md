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

## 文档

| 文档 | 内容 |
|---|---|
| [docs/PROJECT.md](docs/PROJECT.md) | 架构、配置开关(`include/config.h`)、gate、验证手段、常见坑 |
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
gmake -C function-model librefcpu.a LDFLAGS="-lz"     # 构建 ../function-model(构建产物,不入库)
gmake -j4 LDFLAGS="-lz"                               # build/simulator;macOS 没有 -lstdc++fs,所以要覆盖 LDFLAGS
```

寄存器宽度是两个仓库共同的编译期参数(`XLEN`,默认 64);目标文件不带它的标记,
所以切换 `XLEN` / `FP` / `RVC` 之后,要在 function-model 的另一份拷贝里构建 `librefcpu.a`,并使用单独的 `BUILD_DIR`。

改过编译选项后用干净的 `BUILD_DIR`。配置宏(`CFG_*`,见 `include/config.h`)用
`gmake EXTRA_CXXFLAGS="-DCFG_...=..." BUILD_DIR=build_x` 覆盖。核频:`gmake CPU_FREQ_MHZ=...` 或环境变量 `CPU_FREQ_MHZ`。

## 运行

```bash
./build/simulator path/to/program.bin                  # 裸二进制加载到 0x80000000,结束时打印 sim-time(cycle)= ...
PMU=1 ./build/simulator prog.bin                       # TMA / 事件统计,在当前目录写 pmu_*.csv
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

`RVC=1` 让前端按 2 字节位置取指(每 32 字节块 16 个槽)并接受 16 位压缩指令;`RVC=0`(默认)不变
(gate 64/64)。它需要用相同的 `XLEN`、`FP`、`RVC` 从 `function-model` 构建的 `librefcpu.a`(`make FP=1 RVC=1 librefcpu.a`)。

```bash
make BUILD_DIR=build_rvc RVC=1 REFCPU_LIB=<用 RVC=1 构建的 librefcpu>
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

# Simulator 脚本说明

本目录只保留可复用的运行、算分、分析、调试和测试脚本。默认路径面向当前
Qimeng1 SPEC2006 checkpoint 环境，但关键路径均可通过参数或环境变量覆盖。

## 目录结构

```text
script/
├── README.md
├── run_all.sh
├── score_spec2006.sh
├── run_local_short_tests.sh
├── run_special_ckpts_1mcommit.sh
├── analysis/
│   ├── cal_spec.py
│   ├── plot_perf_topdown.py
│   ├── plot_perf_topdown_detail.py
│   ├── plot_perf_topdown_memory.py
│   ├── plot_spec2006_frontend.py
│   └── plot_spec2006_frontend_all.py
├── debug/
│   ├── gdb_repro.sh
│   ├── sim.gdb
│   └── sim_auto.gdb
└── tests/
    └── frontend_param_stress_test.sh
```

## 常用流程

### 1. 全量运行 629 个 SPEC2006 checkpoint

```bash
SIMULATOR=/absolute/path/to/build/simulator \
RESULT_DIR=/absolute/path/to/results \
script/run_all.sh
```

默认配置：

- checkpoint：`/share/personal/S/houruyao/simpoint_20260719/ckpts`
- SD 卡：`/share/personal/S/houruyao/simpoint_20260719/images/sdcard-ref.img`
- 10M committed instructions warmup
- 10M committed instructions measure
- difftest 开启
- 自动选择每个物理核的一个逻辑 CPU，避免使用 SMT sibling
- 使用共享动态任务队列；任一 worker 完成后立即领取下一个 checkpoint，避免静态
  分片在尾部留下空闲核心
- 预期 checkpoint 数量为 629
- 不设置超时，避免长 checkpoint 被提前终止

重要环境变量：

| 变量 | 作用 | 默认值 |
|---|---|---|
| `SIMULATOR` | simulator 可执行文件 | `./build/simulator` |
| `CKPT_ROOT` | checkpoint 根目录 | 当前 SPEC2006 ref checkpoint 目录 |
| `SDCARD_IMAGE` | SD 卡镜像；设为空字符串可禁用 | 当前 ref SD 卡镜像 |
| `RESULT_DIR` | 日志和状态输出目录 | NFS results 下的时间戳目录 |
| `CKPT_WARMUP` | O3 warmup commit 数 | `10000000` |
| `CKPT_MAX_COMMIT` | measure commit 数 | `10000000` |
| `MAX_JOBS` | 并行 worker 数；`auto` 使用全部物理核 | `auto` |
| `NO_DIFF` | `1` 关闭 difftest | `0` |
| `RESUME` | `1` 跳过已有 PASS | `1` |
| `TIMEOUT_SECS` | 单 checkpoint 超时；`0` 禁用 | `0` |
| `DRY_RUN` | `1` 只校验和打印任务分配 | `0` |

输出包括：

- 每个 checkpoint 的 `.log`
- 每个 checkpoint 的 `.status`
- 每个 worker 的 TSV 结果
- `progress.txt`：实时的 claimed/completed/pass/fail/running/pending 计数
- `run_config.txt`：simulator 哈希及本轮关键参数，防止断点续跑时混入其他版本
- `summary.tsv`
- `summary.txt`

脚本只有在 simulator 返回成功并确认达到 measure commit 上限时才标记 PASS。
同一个 `RESULT_DIR` 同时只允许一个 runner；收到中断时会终止对应 worker 的整个
simulator 进程组，避免遗留孤儿进程。最终 `summary.tsv` 按 benchmark/checkpoint
稳定排序。

### 2. 一键计算 SPECint2006 估算分数

自动选择最新的完整结果：

```bash
script/score_spec2006.sh
```

指定结果目录和 CPU 频率：

```bash
script/score_spec2006.sh /absolute/path/to/results 1.0
```

算分前会校验：

- 629 个 `.status`
- 629 PASS、0 FAIL
- 12 个 SimPoint 权重文件

输出文件：

- `perf_report.txt`：完整加权性能报告
- `log_status_report.txt`：日志完整性诊断
- `spec_score_summary.txt`：IPC、CPI 和 SPEC Ratio 精简表

分数依赖传入的 CPU 频率，默认是 1 GHz，并与频率线性缩放。例如 1 GHz 下
16.14 对应 100 MHz 下约 1.614。

### 3. 生成 Top-Down 图

主图：

```bash
python3 script/analysis/plot_perf_topdown.py \
  -i /path/to/results/perf_report.txt \
  -o /path/to/results/perf_topdown.png
```

详细图：

```bash
python3 script/analysis/plot_perf_topdown_detail.py \
  -i /path/to/results/perf_report.txt \
  -o /path/to/results/perf_topdown_detail.png
```

内存分析图：

```bash
python3 script/analysis/plot_perf_topdown_memory.py \
  -i /path/to/results/perf_report.txt \
  -o /path/to/results/perf_topdown_memory.png
```

## 顶层运行脚本

### `run_all.sh`

全量 SPEC2006 checkpoint runner。负责物理核绑定、动态任务调度、difftest、
SD 卡、断点续跑、运行配置校验、实时进度、严格 PASS 判断和最终汇总。它是普通
checkpoint 回放的统一入口，不再保留其他功能重叠的通用 checkpoint runner。

### `score_spec2006.sh`

SPEC2006 一键算分入口。负责选择结果目录、校验运行完整性，并调用
`analysis/cal_spec.py`。正常使用时优先运行这个脚本，不需要直接设置
`cal_spec.py` 的环境变量。

### `run_local_short_tests.sh`

初始化必要子模块、编译 default simulator，并运行 `baremetal/test/cpu-tests`
中的本地短测试。默认跳过耗时明显更长的 `memory-stress`。

```bash
script/run_local_short_tests.sh
script/run_local_short_tests.sh --include-long
script/run_local_short_tests.sh --list
```

### `run_special_ckpts_1mcommit.sh`

回放 12 个包含 MMIO/设备状态的特殊 checkpoint，默认每个运行到 1M committed
instructions。用于验证 checkpoint 设备状态恢复，不用于普通 629 checkpoint
性能算分。

服务器上使用时必须显式指定本次要验证的 simulator，避免误用旧部署：

```bash
SIMULATOR=/absolute/path/to/build/simulator \
script/run_special_ckpts_1mcommit.sh
```

## 分析脚本

### `analysis/cal_spec.py`

读取 checkpoint 日志和 SimPoint 权重，按 benchmark 计算加权 CPI、IPC、缓存、
分支、内存延迟、资源阻塞、Top-Down 指标和估算 SPEC Ratio。由
`score_spec2006.sh` 调用。

### `analysis/plot_perf_topdown.py`

从 `perf_report.txt` 生成主要 Top-Down、IPC 和 SPEC Ratio 图，也支持通过
`--compare` 对比多份报告。

### `analysis/plot_perf_topdown_detail.py`

生成二级、三级 Top-Down 分解图，适合定位 frontend、backend、bad speculation
内部的具体瓶颈。

### `analysis/plot_perf_topdown_memory.py`

生成偏内存方向的 Top-Down 图，包括 L1、外部内存、缓存命中率、MPKI 和内存
延迟等信息。

### `analysis/plot_spec2006_frontend.py`

解析一套结果中的 FALCON frontend 统计，输出瓶颈图和 CSV。默认读取当前目录，
也可以使用 `--input` 指定结果目录。

```bash
python3 script/analysis/plot_spec2006_frontend.py --input /path/to/results
```

### `analysis/plot_spec2006_frontend_all.py`

批量扫描一个根目录下的多套实验配置，对每套包含完整 frontend 字段的结果生成
图和 CSV。用于参数对比，不参与 SPEC 算分。

## 调试脚本

### `debug/gdb_repro.sh`

统一的 GDB 复现入口，可运行普通 binary、checkpoint、fast-forward 和 ref 模式，
并支持限制 warmup/commit 缩短复现窗口。

```bash
script/debug/gdb_repro.sh --mode ckpt -w 500000 -c 2000000 /path/to/ckpt.gz
```

### `debug/sim.gdb`

交互式 GDB 配置，设置断点、信号策略，并提供 `simstate`、`simbt` 辅助命令。

### `debug/sim_auto.gdb`

非交互式崩溃采集配置。捕获 SIGSEGV、SIGABRT、`abort()` 和 assertion 后自动
打印完整 backtrace、寄存器、参数及局部变量。

## 测试脚本

### `tests/frontend_param_stress_test.sh`

对 frontend 的参数组合进行编译矩阵测试，既检查合法配置可以通过，也检查非法
配置能够被静态约束拒绝。日志写入 `build/frontend_param_stress_logs`。

## 已删除的旧脚本

| 脚本 | 删除原因 |
|---|---|
| `plot_perf_cache_hit.py` | 仅跳转到其他用户的绝对 NFS 路径，本仓库没有实现 |
| `run_load_window_width_sweep.sh` | 使用已删除的 `LOAD_WINDOWS_WIDTH` 名称，而且依赖旧 runner 参数 |
| `plot_frontend.py` | 功能已被 `plot_spec2006_frontend.py` 覆盖 |
| `plot_perf_topdown_all.py` | checkpoint 等权汇总，忽略 SimPoint 权重，SPEC 结果不可靠 |
| `run_parent_ckpts_8phys.sh` | 功能是新版 `run_all.sh` 的不完整子集 |

## 依赖

- Shell runner：Bash、`taskset`、`lscpu`、`flock`、`setsid`、`sha256sum`
- 分析和绘图：Python 3、NumPy、Matplotlib
- GDB 复现：GDB
- SPEC 算分：对应结果日志和 12 个 SimPoint `.weights` 文件

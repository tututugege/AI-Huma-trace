# 调试 / 实验用环境变量（部分）

只列这里补过文档的 8 个。LSU 分级流水的开关见 [LSU_PIPE.md](LSU_PIPE.md)，其余环境变量在各自源文件里有注释。全部在运行 `build/simulator` 时设置。**注意：和 RTL 比周期必须设 `LSU_PIPE_NOWD=1`**（RTL 的参考周期是关掉 63 周期看门狗标定的）。

| 变量 | 取值 | 作用 | 定义位置 |
|---|---|---|---|
| `FAST_WARMUP` | `1` | 检查点运行：把（检查点间隔 − 预热）条指令用函数模型快速回放进 cache / TLB / BPU，预热目标改为至多 100 万条提交（EXP-011）。SPEC 全量跑法用它 | `main.cpp` |
| `FE_COMMIT_TRACE` | 任意值（需同时 `PMU=1`） | 每条提交打印 `[CT] pc result diag_val c=<周期>`；分支误预测 / flush 时打印 `[CT] EV ...`。和 RTL 的 `SOC_TB_COMMIT` 逐提交对周期用（取 `pc` 和 `c=`） | `back-end/PmuSample.cpp` |
| `LSU_PIPE_WBTRACE` | 周期数 N | 从第 N 个周期起，打印写回缓冲的响应 / 发射事件（`WB t=... resp/issue ...`，写到 stderr） | `back-end/Lsu/RealLsuPipe.cpp` |
| `TB_NO_INJECT` | 任意值 | 关闭事实注入（P3 之后事实是必须的，只用来调试老版本） | `diff/trace_hook.cpp` |
| `TB_PERTURB_BR` | N | 阳性对照：每 N 个分支翻转一次分支事实，之后的运行必须和基线不同（验证检查本身有效）。地址版本是 `TB_PERTURB_ADDR` | `diff/trace_hook.cpp` |
| `TB_SAMPLE_PC` | N | 每 N 条提交打印一次 `[SAMPLE] cyc commits pc inst`，看长程序的时间花在哪里 | `diff/trace_hook.cpp` |
| `TB_TRACE_COMMIT` | 任意值 | 每条提交打印 `[COMMIT] cyc pc inst rd v`；`rtl/scripts/first_diff.sh` 用它和 RTL 的 `SOC_TB_COMMIT` 找第一处分歧 | `diff/trace_hook.cpp` |
| `TB_TRACE_TRAP` | 任意值 | 打印 CSR 模型处理的每个 trap（`[TRAP] M/S cause epc tval target priv`）和注入给函数模型的中断（`[IRQ]`） | `back-end/Exu/Csr.cpp`、`diff/trace_hook.cpp` |

## 已删除（2026-11-12）

没人用的临时调试开关：`CKPT_TRACE_PC`（EXP-011 窗口位置检查，已完成）、`TB_DEBUG_DDR`、`TB_DEBUG_FLUSH`、`TB_DEBUG_PATCH`（P1 桥接调试打印）、`LSU_PIPE_RPREG`（变量只被设置、从不读取）、`FE_IPF_MAX`（改成常量 4）。

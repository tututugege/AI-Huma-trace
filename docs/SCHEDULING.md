# 一拍的调度：模块契约与自动生成的 `SimCpu::cycle()`

这是现行做法的说明，改模块之前读这一份。过程记录见文末"相关文档"。

## 1. 一拍怎么跑

`SimCpu::cycle()`（SimCpu.cpp）分三段：

1. **拍头**：I-cache 端口寄存器（`icache_port_regs_seq`）、周期计数。
2. **comb 阶段**：`// BEGIN GENERATED ... // END GENERATED` 之间的 48 行，每行是一个模块的一个 comb 阶段（`back.rob->comb_commit()`、`front.core->icache.comb_fetch()`……）。这一段由 `scripts/comb/gen_top_order.py` 从各模块的契约生成，**不要手改**。前端的 `FrontCore::comb()` 也是同样生成的。随后是 `back.pmu_sample()`。
3. **时钟沿**：各 top 的 `seq()`（`front.seq_core`、`b2f`、`f2b`、`back`、`csr`、`mem_subsystem`、`axi`）。

comb 阶段只算本拍的输出和线信号；寄存器在 seq 里更新（写法见 README 的"代码风格与建模方式"）。

## 2. 被调度的前提：模块写法

```cpp
// <模块是什么>
//   CONTRACT comb_x  in: in.a in.b.f  out: out.c
//   CONTRACT comb_y  in: in.d  pre: in.e  out: out.c out.f
class M {
public:
  struct In  { const A *a = nullptr; const B *b = nullptr; ... } in;   // 它读的：const 指针
  struct Out { C c{}; F *f = nullptr; } out;                             // 它驱动的：自己的字段，或父模块拥有的信号的指针
  void comb_x();
  void comb_y();
  void seq();
};
```

* 每个阶段一行 `CONTRACT`，用**自己的端口名**写：`in.<端口>[.<字段>]`、`out.<端口>[.<字段>]`，没有就写 `-`。
* 父模块在 init 里绑定端口，一行一个：`rob->in.dis2rob = &dis->out.dis2rob;`（BackTop::init、MemSubsystem::init、FrontCore、SimCpu::init）。工具沿这些绑定把端口解析成 SimCpu 的路径（如 `back.dis.out.dis2rob`）。
* 一个模块的几个阶段按头文件里列出的顺序执行。
* 模块登记在 `scripts/comb/modules.py` 的 `MODULES` 表里（路径、类名）。新模块要加进去。

契约的四种类型：

| 类型 | 含义 | 例子 |
|---|---|---|
| `in:` | 读这个信号的本拍值：写它的阶段必须排在前面 | 大多数输入 |
| `out:` | 写这个信号 | 大多数输出 |
| `pre:` | 读的是本拍其它模块写之前的值（相当于读寄存器）：排在那些写者前面 | `axi.comb_outputs` 读存储主口 |
| `default:` | 每拍最先写一个默认值：排在所有用这个信号的阶段前面 | `axi.comb_begin` 清各主口的 `req.valid` |

## 3. 顺序规则（gen_top_order.py）

* 同一模块的阶段按声明顺序；
* 一个路径的写者（`out:`）排在其它模块的读者（`in:`）之前；路径按分量比较，一个是另一个的前缀就算重叠；`[MASTER_X]` 这样的下标保留，不带下标的覆盖所有下标；
* `pre:` 读者排在其它写者之前，`default:` 写者排在所有使用者之前；
* 有 trace bridge 副作用的阶段（查事实、提交 / flush 更新 bridge）按 `TB_ORDER` 列表的先后排，这一条不是扫描出来的，是手工声明的；
* 多个阶段同时可排时：优先留在上一个阶段所在的 top（让每个 top 的阶段尽量连成段），再按 `TOP_PRIORITY`，再按 `MODULES` 表顺序。前端内部再按 `FRONT_RANK`。

契约之间成环时工具报错。正确做法是**把一个阶段拆成两个**（比如 LSU 的 `comb_outputs` 和 `comb_writeback`），不能手工排顺序。

## 4. 改了代码之后

```bash
python3 scripts/comb/comb_graph.py --check     # 契约与代码是否一致、有没有顺序隐患、生成块是否最新
python3 scripts/comb/gen_top_order.py          # 打印生成的顺序（看它变没变）
python3 scripts/comb/gen_top_order.py --write  # 写回 SimCpu.cpp / front_top.cpp 的生成块
```

`comb_graph.py --check` 的输出怎么处理：

| 输出 | 意思 | 怎么办 |
|---|---|---|
| `contract: <阶段>: reads X, not declared in` | 代码读了契约没写的端口 | 加进 `in:`（或 `pre:`） |
| `contract: <阶段>: in X declared, not read` | 契约写了、代码没读 | 从契约删掉 |
| `contract: <阶段>: writes X, not declared out` | 代码写了契约没写的端口 | 加进 `out:` |
| `<读者> reads S, written later by <写者>` | 读的时候 S 本拍还没被写：读到的是上一拍留下的值，等于一个没声明的寄存器 | 多半是契约漏了（补上后写者会排到前面）；本意就是读旧值的，用 `pre:` 声明；确认无害的放进 `comb_graph.py` 的 `TOP_KNOWN` 并写明理由 |
| `trace-bridge users not in TB_ORDER: [...]` | 这个阶段新调用了 bridge（`tbhook::...`） | 按它在调度里的位置加进 `gen_top_order.py` 的 `TB_ORDER` |
| `note: N staged reads` | 读的信号本拍更早也有阶段写过 | 只是提示，正常 |
| 顺序成环 | 契约互相依赖 | 拆阶段 |

**生成的顺序变了，周期就可能变。** 纯整理（周期应当不变）时，`gen_top_order.py --check` 必须报"没变"；有意改变顺序时，跑门禁和完整比对（AI-Huma-data 的 `vbin.sh`），再看 Linux 窗口。提交前 `comb_graph.py --check` 必须通过。

## 5. 工具看不见的东西

扫描器只认 `in.X` / `out.X` 端口。下面这些不经过端口，调度工具不知道谁在读写，靠约定保证顺序正确。**新增一处这样的读写，要么改成端口，要么登记在这里并写明为什么顺序安全。**

| 共享状态 | 谁写 | 为什么顺序安全 |
|---|---|---|
| CSR 文件 `csr_state()`（SimCpu::csr） | 只有 `csr.seq()`（时钟沿）和检查点恢复 | comb 里任何时候读都是本拍开头的值 |
| uop 池 `uop_pool[id]` | 静态字段：译码、Rename、Dispatch 各写一次，只写有效槽 | 写在前面的流水级，读在后面的流水级；id 0 是公用空条目，写它会在退出时报错 |
| trace bridge（`tbhook::*`） | ROB 提交 / flush、mispredict；各处查事实 | `TB_ORDER` 声明先后 |
| `msd::taint`（LSU_MS_DATA 旧值表） | LSU 写回、Rename 分配、LSU 的 flush 处理 | 写回和读者（EXU）之间的先后按 RTL 的一拍延迟建模（`release` 的 `until`） |
| 设备寄存器 `IoBacking`（`io_read / io_write`） | 加载器、外设模型、AXI 的 MMIO 路径和 UART | 运行中由外设模型（`PeripheralAxi` / `PeripheralModel`）和 AXI 互连的旧 MMIO 路径读写；两者之间的先后**没有**被工具检查，改这两处时要人工确认 |
| PMU 事件总线 `g_pmu_*` | 各模块置位 | 只有拍末的 `pmu_sample` 读 |
| 模块直接读别的模块的寄存器（指针成员：`Rob::ftq`、`FrontControl::bpu_pipe`、`Exu::ftq`） | 被读模块的 seq | 读的是寄存器；BPU 的 `State` 在 `comb_front` 里原地写，`decide()` 在同一拍更早（`ctrl.control`）读，这一先后由契约保证（`ctrl.control` 产出 `pipe_dec` 给 `comb_front`） |

子模块：LSU 里的 DTLB、I-cache 里的 ITLB、`AxiSubsystem` 里的互连、`PeripheralAxi` 里的外设模型不单独调度，由父模块调用。`modules.py` 的 `SUBMODULES` 表把子模块的成员映射到父模块的端口（例如 TLB 的 `walk_req` → LSU 的 `out.dtlb_walk`），扫描父模块时顺带扫描子模块被调用的方法。给子模块新加一个"端口"成员，要在这张表里补映射。

## 6. 例子

**给模块加一个输入。** `In` 里加 `const X *x = nullptr;` → 父模块 init 里加一行绑定 → 用到它的阶段在 `CONTRACT` 的 `in:` 里加 `in.x` → `comb_graph.py --check` → `gen_top_order.py --write`。如果 `x` 的写者原来排在这个阶段后面，生成的顺序会变。

**一个阶段新查了事实。** ROB 的 CSR 写入改在提交时（`comb_commit` 里 `tbhook::fact_by_rob`）之后，检查报 `trace-bridge users not in TB_ORDER: ['back.rob.comb_commit']`。它在调度里位于 `back.lsu.comb_outputs` 和 `back.lsu.comb_writeback` 之间，就按这个位置加进 `TB_ORDER`；生成的顺序不变。

**读另一个模块"上一拍"的输出。** TLB 读页表遍历器的输出。遍历器的输出结构在它自己的 comb 阶段里会被重写，TLB 如果直接读它，就是一个"读在写之前"的隐患。做法是让遍历器在时钟沿把给 TLB 的那部分锁存一份（`MemPtwBlock::dtlb_q / itlb_q`，只在 seq 写），TLB 读这一份：它是寄存器，对调度没有约束。

## 7. 工具文件

| 文件 | 作用 |
|---|---|
| `scripts/comb/modules.py` | 模块表、契约解析、端口绑定解析、每个阶段的读写扫描；`--check` 对比契约 |
| `scripts/comb/gen_top_order.py` | 唯一的生成器：写 `SimCpu::cycle()` 和 `FrontCore::comb()`；`TOP_PRIORITY`、`TB_ORDER`、`FRONT_RANK` 在这里 |
| `scripts/comb/comb_graph.py` | `--check`：上面两个的检查，加上按 `cycle()` 实际顺序扫描隐患（`TOP_KNOWN`：审过的例外） |
| `scripts/comb/comb_top.py`、`comb_domains.py`、`contract_lib.py` | 隐患扫描和拓扑排序用到的源码解析 |

## 相关文档

* 历史过程记录（[archive/FRONT_MODULES.md](archive/FRONT_MODULES.md) 及已入 git 历史的 `UNIFIED_MODULES.md`、`NEXT_STATE.md`、`MODULE_CLEANUP.md`、`MODULE_CONTRACT.md`、`REG_WIRE.md`）仅作备查，不再维护。

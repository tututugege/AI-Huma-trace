# AXI Interconnect Kit（中文说明）

这是从原模拟器中抽离出的独立 AXI4 内存子系统。

## 当前主线

当前主线是 integrated dual external AXI port：

- CPU core 侧仍使用 `read_ports[4]` / `write_ports[2]` 简化上游接口。
- 下游直接拆成 DDR/SDRAM AXI 口和 MMIO AXI 口。
- DDR/SDRAM 口使用 256-bit beat，支持 64B cacheline 的 2 beat 传输。
- MMIO 口使用 32-bit beat，只支持 4B / 1 beat read/write。
- 读 outstanding 和写 outstanding 各自共享全局预算，dual-port profile 目标值均为 32。

旧单 AXI 口和 `AXI_Router_AXI4` 仍保留给历史测试、demo 和 bring-up；当前 C++ 接口见
[docs/interfaces_CN.md](docs/interfaces_CN.md)。

## 范围

- 一套 AXI4 interconnect，对外提供简化上游端口：
  - `read_ports[4]`：`icache`、`dcache_r`、`uncore_lsu_r`、`extra_r`
  - `write_ports[2]`：`dcache_w`、`uncore_lsu_w`
- 双外部 AXI 口、AXI4 router legacy/demo、SimDDR 下游内存模型、MMIO bus、UART16550
- AXI4 路径上的可选共享统一 LLC

本仓库已经删除 AXI3 支持，现在是 AXI4-only。

## 拓扑

```text
读/写主设备
      |
      v
+----------------------+
| AXI_Interconnect     |
+----------------------+
      |
      v
+----------------------+
| AXI_LLC（可选）      |
+----------------------+
      |
      +---------------------+
      |                     |
      v                     v
+--------------+     +---------------------+
| DDR/SDRAM AXI|     | MMIO AXI            |
| 256-bit beat |     | 32-bit beat         |
+--------------+     +---------------------+
      |                     |
      v                     v
+----------+         +---------------------+
| SimDDR   |         | MMIO_Bus + UART16550|
+----------+         +---------------------+
```

当前 integrated dual-port 主线在 interconnect/bridge 内完成 DDR/MMIO 地址分流。
`AXI_Router_AXI4` 是历史单口/示例路径中的显式层次：interconnect 负责仲裁和上游响应
路由，router 负责 AXI 侧地址译码。

本仓库中的术语约定：

- `upstream`：连接到 `read_ports[]` / `write_ports[]` 的请求来源，以及返回给这些
  master 的响应路径
- `downstream`：位于 interconnect 之下、面向 DDR 或 MMIO 的接口与设备，包括
  `axi_ddr_io`、`axi_mmio_io`、legacy `AXI_Router_AXI4`、`SimDDR`、MMIO 设备

## LLC 概要

`AXI_LLC` 位于 AXI4 interconnect 之后，建模共享统一缓存。

默认配置：

- `8MB`
- `64B` cache line
- `16-way`
- 默认 `8` 个 MSHR
- lookup latency `3`
- `PIPT`、`unified`、`NINE`
- 默认关闭 prefetch

当前语义：

- cacheable read 通过父模拟器提供的外部 SRAM 风格 `data/meta/repl` 表进行
  分配与回填。
- `CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC` 用来控制 DCache demand read miss
  是否在 LLC 中分配：
  - `0`（默认）：miss 回填后安装进 LLC
  - `1`：保持 no-allocate 路径，只把回填数据返回给上游，不在 LLC 中安装
- AXI4 interconnect 的读上游侧支持 multiple outstanding：
  - dual-port profile 全局上限 `32`
  - dual-port profile 单个读 master 上限 `32`
  - standalone CMake 默认值可较小，需按目标 profile 显式配置
- LLC 内部对 cacheable demand miss 的推进仍然更严格：
  - 同一个读 master 同时最多只能有一个由 LLC 正在处理的 cacheable miss
  - 其它 master 仍然可以继续占用剩余的全局读资源
  - bypass read 不受这个 same-master cacheable-miss 串行限制
- cacheable write 由 LLC 路径接管。
- cacheable 的 partial write miss 采用“先回填旧 line，再按字节 merge，最后以
  dirty line 安装”的语义，不会直接在全零 line 上 merge。
- `bypass read` 会先查 LLC：
  - hit 直接返回 resident line 最新值
  - miss 下发到下游且不分配 LLC
- `bypass write` 是 write-through maintenance：
  - hit 更新 resident line，但不设置 `DIRTY`
  - miss 直接下发到下游，不分配 LLC
- 子行写按 `addr % line_bytes` 合并。
- `invalidate_all` 当前采用保守语义：
  - 只有在不存在 dirty resident line、dirty victim writeback、以及写侧 hazard
    时才接受
  - 调用方应持续保持请求，直到观察到 `invalidate_all_accepted`
  - `invalidate_all` pending 期间，已捕获的 clean LLC 路径请求可继续排空
  - 一旦接受，会通过 epoch 丢弃 stale clean refill install
  - 外部表当前把独立 `valid` 表建模为每个 way 一字节的 regfile；
    `invalidate_all` 只清这张 `valid` 表，不清 data/meta/repl
  - 不会静默丢弃 dirty resident 数据
- interconnect 还带有 submodule 原型用运行时控制输入：
  - `mode=1`：LLC_ON
  - `mode=2`：`[offset, offset + 4MB)` 映射为 direct-mapped 的本地 LLC 存储窗口；
    窗口内访问不分配 MSHR、也不会从 DDR refill，窗口外访问强制 `bypass`
    并继续按 dual-port 地址分类下发
  - `mode=2` 窗口外访问分类：
    - `addr >= 0x4000_0000` 走 DDR/SDRAM 口
    - 其它地址走 MMIO 口
    - DDR/SDRAM 口使用 `256-bit` beat；小于等于 `32B` 的读写对齐到单
      beat，`64B` cacheline 读写拆成同一事务的 2 个 beat
    - MMIO 口只支持 `4B / 1 beat` 读写
  - `mode=0`：LLC_OFF
  - 编码值 `mode=3` 会先规范化为 `mode=0`，不是独立运行模式
  - `mode` / `offset` 变化遵循 `drain -> invalidate_all -> activate`：
    切换期间不再接收新的上游请求，旧模式下已在途的 LLC/AXI 工作先排空，
    只有在 `invalidate_all_accepted` 之后才切换到新配置

### AXI4 写并发

当前 AXI4 写路径设计是：

- interconnect 上游侧最多接收 `MAX_WRITE_OUTSTANDING` 个 pending write。
- 非 LLC 路径可直接按 AXI ID 将 B 响应路由回上游。
- LLC 路径内部具备：
  - 按 master 划分的 pending write queue
  - 每个写 master 一个 active write context
  - 共享 lookup engine
  - 共享 victim writeback 资源
  - 共享下游 memory write port
- 同一个 master 的后继写，仍然要等前一个写响应槽被消费后才继续提升。

因此，`MAX_WRITE_OUTSTANDING` 表示的是 interconnect 上游侧的排队上界，
不是“interconnect 状态 + LLC 内部状态”合并后的总写状态上界。

这就是当前阶段写路径 correctness 的目标边界。后续如果还要继续追性能，重点会是
进一步提高内部写资源并行度，而不是再回头修基本的一致性与顺序语义。

## SimDDR 建模说明

- `AXI_KIT_SIM_DDR_BEAT_BYTES` 目前支持 `4/8/16/32B`；其中 `32B` 是
  AXI4 256-bit single-beat 传输所需配置。
- `AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY` 的语义是：最后一个 `W` beat 完成
  握手后，再额外等待多少个完整周期，`B` 通道才首次可见。
- 集成到父 simulator 时，`AXI_KIT_*` 参数应来自 parent `config.h`；
  若缺少必需定义，submodule 会直接编译报错，而不是静默回落到本地默认值。
- `SimDDR` 仍然是固定延迟的功能模型，不是精确 DDR controller 时序模型；
  当前也还没有细化建模更真实的 `AW/W` backpressure 与写通道调度行为。

## 测试分层

- `P0`：纯组件级确定性单测
  - LLC 读写 hit/miss
  - bypass read/write
  - victim writeback
  - maintenance
  - stale refill 保护
  - write queue 边界
- `P1`：AXI4 + LLC + SimDDR 小型集成确定性回归
  - cacheable+bypass 共存
  - queued write
  - maintenance interlock
  - invalidate-all epoch 行为
- `P2`：固定种子混合压力
  - 混合一致性压力
  - refill / maintenance / writeback 竞争

已验证编译器：

- `qm` 环境默认工具链
- 系统默认 `g++`
- `qm` 环境中的 Conda 版 `x86_64-conda-linux-gnu-c++`

## 接口文档

- [docs/submodule_architecture_CN.md](docs/submodule_architecture_CN.md)
- [docs/interfaces.md](docs/interfaces.md)
- [docs/interfaces_CN.md](docs/interfaces_CN.md)
- [docs/llc_design_CN.md](docs/llc_design_CN.md)

## 主要文件

```text
.
├── CMakeLists.txt
├── README.md / README_CN.md
├── axi_interconnect/
│   ├── include/
│   ├── AXI_Interconnect.cpp
│   ├── AXI_LLC.cpp
│   ├── AXI_Router_AXI4.cpp
│   ├── axi_interconnect_dual_port_test.cpp
│   ├── axi_interconnect_issue_probe_test.cpp
│   ├── axi_interconnect_test.cpp
│   ├── axi_interconnect_llc_axi4_test.cpp
│   └── axi_llc_test.cpp
├── sim_ddr/
│   ├── include/
│   ├── SimDDR.cpp
│   └── sim_ddr_test.cpp
├── mmio/
│   ├── include/
│   ├── MMIO_Bus_AXI4.cpp
│   ├── UART16550_Device.cpp
│   └── mmio_router_axi4_test.cpp
└── demos/
    └── axi4_smoke.cpp
```

## 当前收尾结论

本仓库当前已经在 AXI4-only LLC/interconnect 的 correctness 范围上显著稳
定，并对 maintenance 采用保守语义。最终 closure 口径仍有意保留到外部
review 完成之后。父模拟器层面的联调问题，应优先在主模拟器中单独定位，
除非根因能够明确指回本子模块。

# AXI Interconnect Kit 接口规格（中文）

本文档列出：

1. Interconnect 上游接口
2. 双外部 AXI4 五通道信号

架构总览、地址映射和约束说明见
[submodule_architecture_CN.md](submodule_architecture_CN.md)。

## 1) Interconnect 上游接口

定义位置：`axi_interconnect/include/AXI_Interconnect_IO.h`。

### 1.1 读请求（`ReadMasterReq_t`）

| 信号 | 位宽 | 相对 interconnect 方向 | 说明 |
|---|---:|---|---|
| `valid` | 1 | 输入 | 请求有效 |
| `ready` | 1 | 输出 | interconnect 接受请求 |
| `addr` | 32 | 输入 | 字节地址 |
| `total_size` | 8 | 输入 | 传输字节数减 1（`0=1B`, `255=256B`） |
| `id` | 4 | 输入 | 上游事务 ID |
| `bypass` | 1 | 输入 | 绕过 LLC 分配/所有权规则 |

### 1.2 读响应（`ReadMasterResp_t`）

| 信号 | 位宽 | 方向 | 说明 |
|---|---:|---|---|
| `valid` | 1 | 输出 | 响应有效 |
| `ready` | 1 | 输入 | 主设备接受响应 |
| `data` | 2048（`64x32`） | 输出 | 宽数据响应（最多 256B） |
| `id` | 4 | 输出 | 返回上游 ID |

### 1.3 写请求（`WriteMasterReq_t`）

| 信号 | 位宽 | 方向 | 说明 |
|---|---:|---|---|
| `valid` | 1 | 输入 | 请求有效 |
| `ready` | 1 | 输出 | interconnect 接受请求 |
| `addr` | 32 | 输入 | 字节地址 |
| `wdata` | 默认 512（`16x32`） | 输入 | 宽写数据（由 `AXI_KIT_MAX_WRITE_TRANSACTION_BYTES` 决定） |
| `wstrb` | 64 | 输入 | 字节写掩码（每字节 1bit） |
| `total_size` | 8 | 输入 | 传输字节数减 1（`0=1B`，默认最大 `63=64B`） |
| `id` | 4 | 输入 | 上游事务 ID |
| `bypass` | 1 | 输入 | write-through maintenance / bypass 语义 |

### 1.4 写响应（`WriteMasterResp_t`）

| 信号 | 位宽 | 方向 | 说明 |
|---|---:|---|---|
| `valid` | 1 | 输出 | 响应有效 |
| `ready` | 1 | 输入 | 主设备接受响应 |
| `id` | 4 | 输出 | 返回上游 ID |
| `resp` | 2 | 输出 | AXI 响应码 |

### 1.5 LLC maintenance 控制面

AXI4 interconnect 通过 `AXI_Interconnect` 上的方法暴露 LLC maintenance 控制面：

- `set_llc_invalidate_all(bool)`
- `set_llc_invalidate_line(bool, uint32_t line_addr)`
- `llc_invalidate_all_accepted()`
- `llc_invalidate_line_accepted()`

约定如下：

1. 调用方必须持续保持 `invalidate_all` / `invalidate_line` 请求，直到观测到
   对应的 `*_accepted()` 脉冲。
2. `invalidate_all` 采用保守语义：
   - pending 期间会阻止新的上游 LLC 请求进入
   - 已经捕获的 clean LLC 路径请求允许继续排空
   - 只有在不存在 dirty resident line、dirty victim writeback、以及写侧
     hazard 时才会被接受
3. `invalidate_line` 是按 line 的精确 maintenance：
   - 当同一条 line 在上游请求路径、LLC 内部写队列/写响应状态、或下游写路径
     中仍存在任一写侧 hazard 时，该请求会被拒绝
   - 具体包括 inflight miss、active write context、queued write、write
     lookup、victim writeback、active/pending bypass write、以及 same-cycle
     上游写 accept/capture 冲突
4. `invalidate_all` 被接受后，旧 epoch 的 stale clean refill install 会被丢弃，
   不会重新写回 LLC。
5. 当前原型中，line 的真实有效性来自独立 `valid` 表；`meta.flags.VALID`
   只保留为 shadow / 调试兼容位。

### 1.6 submodule 运行时 mode 控制面

`AXI_Interconnect` 当前还暴露一组面向 submodule 原型验证的运行时控制输入：

- `mode[1:0]`
- `llc_mapped_offset[31:0]`

当前语义：

1. `mode=1`
   - LLC_ON 模式
   - 请求继续走共享 LLC datapath
   - 上游本来的 `bypass` 语义保持不变
2. `mode=2`
   - 地址映射模式
   - `[llc_mapped_offset, llc_mapped_offset + 4MB)` 内的请求会被翻译到
     direct-mapped 的本地 LLC 存储窗口
   - 窗口内访问不做 tag compare / replacement，不分配 MSHR，也不会从 DDR refill
   - 窗口外请求被强制转换为 `bypass`，仍经由 DDR/MMIO 下游路径访问
   - 窗口外地址按 dual-port route helper 分类：
     - `addr >= 0x4000_0000` 走 DDR/SDRAM 口
     - 其它地址走 MMIO 口
     - DDR/SDRAM 口使用 `256-bit` beat；小于等于 `32B` 的读写对齐到单
       beat，`64B` cacheline 读写拆成同一事务的 2 个 beat
     - MMIO 口只支持 `4B / 1 beat` 读写
3. `mode=0`
   - LLC_OFF 模式
   - 因此不会在 LLC 中新分配 line

编码值 `mode=3` 会先规范化为 `mode=0`，不是独立运行模式。

当前切换合同：

1. `mode` 或 `llc_mapped_offset` 变化时，interconnect 先进入 reconfiguration
   阶段
2. reconfiguration 期间：
   - 不再接收新的上游请求
   - 旧模式下已在途的 LLC/AXI 工作先排空到 quiescent point
3. 只有到达 quiescent point 后，interconnect 才会请求 `invalidate_all`
4. 只有在 `invalidate_all_accepted` 之后，新的运行时 `mode/offset` 才真正生效

实现备注：

- 当前本地 simulator harness 还支持通过环境变量
  `AXI_SUBMODULE_MODE` / `AXI_SUBMODULE_OFFSET` 在 `init()` 时给这两个输入赋上电
  默认值；这只是仿真注入手段，不改变模块按输入信号工作的硬件语义。

## 2) 双外部 AXI4 五通道信号

定义位置：`sim_ddr/include/SimDDR_IO.h`。

当前主线有两组外部 AXI master 口：

- DDR/SDRAM 口：`axi_ddr_io`
  - `ID`：6 位 profile
  - 数据位宽：`256-bit (32B beat)`
  - 小于等于 32B 的请求对齐到 32B 单 beat；64B cacheline 为 2 beat
- MMIO 口：`axi_mmio_io`
  - `ID`：6 位 profile
  - 数据位宽：`32-bit (4B beat)`
  - 只允许 4B / 1 beat read/write

旧 `axi_io` 是 C++ 侧 legacy alias，指向 DDR 口；不要把它理解为当前唯一外部 AXI 口。

### 2.1 写地址通道（`AW`）

| 信号 | 位宽 | 方向（主->从） | 说明 |
|---|---:|---|---|
| `awvalid` | 1 | M->S | 地址有效 |
| `awready` | 1 | S->M | 地址就绪 |
| `awid` | DDR 6 / MMIO 6 | M->S | 写事务 ID |
| `awaddr` | 32 | M->S | 字节地址 |
| `awlen` | 8 | M->S | 突发长度减 1 |
| `awsize` | 3 | M->S | `log2(bytes_per_beat)` |
| `awburst` | 2 | M->S | 突发类型 |

### 2.2 写数据通道（`W`）

| 信号 | 位宽 | 方向 | 说明 |
|---|---:|---|---|
| `wvalid` | 1 | M->S | 数据有效 |
| `wready` | 1 | S->M | 数据就绪 |
| `wdata` | DDR 256 / MMIO 32 | M->S | 写数据 |
| `wstrb` | DDR 32 / MMIO 4 | M->S | 字节写使能 |
| `wlast` | 1 | M->S | 最后一个 beat |

### 2.3 写响应通道（`B`）

| 信号 | 位宽 | 方向 | 说明 |
|---|---:|---|---|
| `bvalid` | 1 | S->M | 响应有效 |
| `bready` | 1 | M->S | 响应就绪 |
| `bid` | DDR 6 / MMIO 6 | S->M | 响应 ID |
| `bresp` | 2 | S->M | 响应码 |

### 2.4 读地址通道（`AR`）

| 信号 | 位宽 | 方向 | 说明 |
|---|---:|---|---|
| `arvalid` | 1 | M->S | 地址有效 |
| `arready` | 1 | S->M | 地址就绪 |
| `arid` | DDR 6 / MMIO 6 | M->S | 读事务 ID |
| `araddr` | 32 | M->S | 字节地址 |
| `arlen` | 8 | M->S | 突发长度减 1 |
| `arsize` | 3 | M->S | `log2(bytes_per_beat)` |
| `arburst` | 2 | M->S | 突发类型 |

### 2.5 读数据通道（`R`）

| 信号 | 位宽 | 方向 | 说明 |
|---|---:|---|---|
| `rvalid` | 1 | S->M | 数据有效 |
| `rready` | 1 | M->S | 数据就绪 |
| `rid` | DDR 6 / MMIO 6 | S->M | 响应 ID |
| `rdata` | DDR 256 / MMIO 32 | S->M | 读数据 |
| `rresp` | 2 | S->M | 响应码 |
| `rlast` | 1 | S->M | 最后一个 beat |

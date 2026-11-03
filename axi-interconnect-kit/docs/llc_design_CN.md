# AXI_LLC 详细设计说明（中文）

本文档描述 `axi-interconnect-kit` 当前 AXI4-only 版本中的 `AXI_LLC` 设计与语义边界。

## 1. 术语约定

- `upstream`：连接到 `AXI_Interconnect::read_ports[]` / `write_ports[]` 的请求来源，以及返回给这些 master 的响应路径。
- `downstream`：位于 interconnect 之下、面向 DDR 或 MMIO 的路径，包括 `AXI_Router_AXI4`、`SimDDR` 与 MMIO 设备。
- `cacheable`：由 LLC 接管一致性与驻留所有权的请求。
- `bypass`：不在 LLC 中分配所有权的请求。读请求允许命中 resident line；写请求采用 write-through maintenance 语义。

## 2. 拓扑位置与职责

当前子模块的主拓扑是：

```text
upstream read/write masters
          |
          v
 +----------------------+
 | AXI_Interconnect     |
 +----------------------+
          |
          v
 +----------------------+
 | AXI_LLC (optional)   |
 +----------------------+
          |
          v
 +----------------------+
 | AXI_Router_AXI4      |
 +----------------------+
      |             |
      | DDR range   | MMIO range
      v             v
 +----------+   +---------------------+
 | SimDDR   |   | MMIO_Bus + UART16550|
 +----------+   +---------------------+
```

`AXI_Interconnect` 负责：

- upstream 读写端口仲裁
- read outstanding / write pending 管理
- LLC 维护接口的接入
- AXI4 channel 组织与响应回送

`AXI_LLC` 负责：

- shared unified cache 的 hit/miss 判定
- cacheable read / write 的驻留与替换
- bypass read / write 的语义分流
- dirty victim writeback
- `invalidate_line` / `invalidate_all` maintenance

## 3. 组织方式

### 3.1 地址与容量

当前默认配置：

- capacity: `8MB`
- line size: `64B`
- associativity: `16-way`
- lookup latency: `8`
- MSHR: `4`

容量、line size、way 数、MSHR 数都由 `AXI_LLCConfig` 提供，属于运行时配置而不是模板常量。

### 3.2 表结构

当前 LLC 采用外置 SRAM 风格查表接口，逻辑上分为四类表：

- `data` 表：每个 way 的整条 cache line 数据
- `meta` 表：压缩后的 `tag + shadow flags`
- `valid` 表：每个 way 的有效位
- `repl` 表：每个 set 的替换状态

`meta.flags` 当前保留：

- `VALID`
- `DIRTY`
- `PREFETCH`

其中：

- 命中/失效语义上的“真实有效性”只以独立 `valid` 表为准
- `meta.flags.VALID` 当前只作为兼容位保留，不再参与 hit / victim valid 判定
- lookup 完成必须收到显式的 `lookup_in.valid_valid` 和完整 valid row；模型不能从
  `meta.flags.VALID` 推导有效性
- 外部 `valid` 宿主当前采用每个 way 一字节的 regfile，按 byte write enable 更新单个 way
- 当前 `meta` 已从原型早期的较宽表示压缩到 `4B/line`

### 3.3 寻址方式

当前设计采用 `PIPT`：

- lookup / resident ownership / replacement 都基于物理地址 line 粒度
- 不依赖虚地址别名语义

## 4. 替换策略

当前 `repl` 表维护的是每个 set 的“下一个 victim way”，实现上是简单 round-robin：

- hit 后：`repl_next_way = (hit_way + 1) % ways`
- miss/install 后：`repl_next_way = (installed_way + 1) % ways`

因此这不是 LRU / PLRU，而是低复杂度、确定性的 round-robin 近似策略。

优点：

- 状态极小
- 单元测试容易做 reviewer-visible 的确定性断言

局限：

- 性能上不如更复杂的替换策略
- 未来若要继续优化，可把 `repl` 表升级为 tree-PLRU 或更细的 replacement state，而不需要改 upstream/downstream 接口

## 5. 读请求语义

### 5.1 cacheable read

流程：

1. upstream read 被 interconnect 捕获
2. LLC 发起 `data/meta/repl` lookup
3. hit：直接从 resident line 返回
4. miss：分配 read MSHR，向 downstream 发起整 line refill
5. refill 返回后安装 line，再返回 upstream read response

当前并发边界：

- AXI4 interconnect 读侧支持 multiple outstanding
- 但 LLC 对 cacheable demand miss 更保守：
  - 同一 read master 同时最多一个被 LLC actively owned 的 cacheable demand miss
  - 不同 master 之间仍可共享全局读资源

### 5.2 bypass read

语义不是“完全绕过 LLC 可见性”，而是：

- hit resident line：直接返回 LLC 中最新值
- miss：下发 downstream，不分配 LLC line

这样可以避免 bypass read 在同地址上读到 stale memory，而 resident line 中已经有更新值。

## 6. 写请求语义

### 6.1 cacheable write

cacheable write 始终由 LLC 接管，不会绕开 LLC 直接下发。

#### full-line write miss

如果请求覆盖整条 line：

- 不需要先 refill 旧 line
- 可直接构造新 line
- 若 victim dirty，则先写回 victim，再安装新 line

#### sub-line write miss

如果请求小于 `cache line`：

- 先对旧 line 发起 refill
- refill 返回后按 `wstrb` 和 `addr % line_bytes` 做 byte merge
- 以 `DIRTY` line 安装到 LLC
- 不能直接在全零 line 上 merge

这条语义是当前 round4 review 后显式补强和回归覆盖的重点。

### 6.2 bypass write

当前 bypass write 采用 write-through maintenance 语义：

- hit resident line：
  - 更新 resident line 数据
  - 不设置 `DIRTY`
- miss：
  - 直接下发 downstream
  - 不分配 LLC line

这种语义适合 uncacheable / maintenance 风格写入，避免无意义占用 LLC 容量。

## 7. dirty line 与 victim writeback

当前 LLC 支持 dirty resident line。

当 install 新 line 需要覆盖 dirty victim 时：

1. 先把 victim line 整条写回 downstream memory
2. 收到 downstream write response 后
3. 再安装新 line / 完成 pending write 或 refill commit

这条顺序在当前组件测试和 AXI4+LLC+SimDDR 集成测试里都有覆盖，包括：

- dirty victim writeback 地址正确
- dirty victim writeback 数据正确
- partial write miss + dirty victim 的交错

### 7.1 dirty victim snapshot 的三个阶段

当前实现里，dirty victim line 在替换路径上会经历三个不同的保存阶段：

1. `mshr[].victim_addr / victim_data`
   - 这是在 read miss / partial-write miss 选择 victim way 后，从 LLC data/meta 表中抓出的快照
   - 它仍然附着在对应的 MSHR / miss entry 上
   - 此时只能说明 “victim snapshot 已生成”，不能说明它已经脱离 refill slot 独立托管
2. `read_victim_wb_q`
   - 这是只服务于 read refill victim 的等待队列
   - 当 active victim writeback 槽正在被其它 victim 占用时，read refill 可以先把自己的 dirty victim snapshot 排进这里
   - 一旦进入这里，就说明 snapshot 已经脱离 refill slot 独立保存
3. `victim_wb_*`
   - 这是 LLC 内部全局唯一的 active victim writeback 槽
   - 它保存当前真正准备向 downstream memory 发出的 victim writeback 请求
   - read victim 和 write victim 最终都会经过这里完成 DDR writeback

因此：

- `mshr.victim_data` 表示 “snapshot 已抓出”
- `read_victim_wb_q` / `victim_wb_*` 表示 “snapshot 已被独立托管”

### 7.2 `victim_wb` 与 `read_victim_wb_q` 的职责分工

当前实现显式区分：

- `victim_wb_*`
  - 负责 active writeback 的协议推进
  - 负责 `issued` / `write_resp` / owner bookkeeping
  - 是真正与 downstream 写口对接的执行槽
- `read_victim_wb_q`
  - 只负责 read refill victim 的排队缓冲
  - 不直接与 downstream 写口握手
  - 当 `victim_wb_*` 空闲后，再由队头搬入 active 槽继续推进

write miss 的 dirty victim 仍然直接占用 `victim_wb_*`，因为 write path 的完成条件与
`victim_mem_done` 直接绑定；read refill 的 dirty victim 则允许先进入 `read_victim_wb_q`
做解耦。

### 7.3 当前 read refill victim 的前进条件

当前版本保留了 “`pick_refill_commit_slot()` 固定选择第一个 `refill_valid` slot” 的仲裁方式；
forward progress 不再依赖跳过 blocked slot，而依赖更明确的 victim snapshot 外部化语义：

1. read refill 遇到 dirty victim 时，先检查是否存在已经真正占用 cache state 的 same-line
   write
   - 例如已发出的 write lookup
   - 或已经进入 `cache_pending` 的 write context
2. 若不存在上述 blocker，则把 `mshr.victim_data` 外部化到：
   - active `victim_wb_*`
   - 或 `read_victim_wb_q`
3. 一旦 snapshot 已进入这两个独立存储之一，refill commit 就可以继续

这意味着 read refill commit 的依赖从：

- “write path 是否已经推进到 `cache_done` / victim writeback issued”

收敛为：

- “dirty victim snapshot 是否已经被独立托管”

因此即使 refill commit 一直只看第一个 slot，也不会再因为同 slot 的 victim snapshot 仍挂在
本地内部状态上而形成自锁。

## 8. 多 master 读写竞争

### 8.1 读侧

interconnect 读侧支持多个 outstanding，并带：

- 全局 read outstanding 上限
- per-read-master 上限

LLC 内部则进一步限制 cacheable demand miss 的同 master 并行度，以降低状态机复杂度。

### 8.2 写侧

当前实现已经不是“单一写入口串行捕获”：

- interconnect upstream 侧可接收多个 pending write
- LLC 内部维护：
  - per-write-master pending queue
  - 每个写 master 一个 active write context
  - 共享 lookup engine
  - 共享 victim writeback 资源
  - 共享 downstream memory write port

因此当前是：

- 多 active write context
- 但共享执行资源仍会限制真正的并发推进深度

### 8.3 same-master 顺序

同一 write master 的后继写不能越过前一个尚未消费的 write response slot。

目的：

- 保证 same-master 可见顺序更保守
- 避免 response ID / resp slot 交错导致的歧义

## 9. maintenance 语义

### 9.1 `invalidate_line`

`invalidate_line` 是 line 精确维护，不是 set 级或全局维护。

接受条件较严格；若目标 line 与以下状态冲突，则拒绝：

- inflight miss
- active write context
- queued write
- write lookup
- victim writeback
- active / pending bypass write
- same-cycle upstream write accept/capture 冲突

这里的“写侧冲突”不是只看一拍上游请求，而是看同一条 line 在三个层面是否已经完全排空：

- upstream 请求路径中的 visible request / ready-first capture hazard
- LLC 内部的 write queue / active write context / write response slot
- downstream 写路径中的 active write transaction / pending write response

调用方必须保持请求，直到看到 `invalidate_line_accepted` 脉冲。
对外推荐通过 `AXI_Interconnect` 的 maintenance API 使用：

- `set_llc_invalidate_line(bool, uint32_t)`
- `llc_invalidate_line_accepted()`

### 9.2 `invalidate_all`

当前 `invalidate_all` 采用保守 barrier 语义：

- 调用方必须保持请求，直到看到 `invalidate_all_accepted`
- pending 期间：
  - 阻止新的 upstream LLC 请求进入
  - 允许已捕获的 clean LLC-path work 排空
- 只有在不存在以下 hazard 时才接受：
  - dirty resident line
  - dirty victim writeback
  - write-side hazard

当前实现不是逐行扫描失效，而是：

- LLC 内部在 accept 当拍产生 `table_out.invalidate_all`
- 外部表实现收到该脉冲后，当前只对独立 `valid` 表执行整体 `reset()`
- 同时 LLC 递增一个 `invalidate_epoch`

因此：

- `data/repl` 内容允许保留为 stale 值
- `meta` 中的 tag / dirty / prefetch / valid-shadow 也允许保留为 stale 值
- 后续 hit 判定只会看独立 `valid` 表，因此这些 stale 内容都不可达

因此当前原型的失效粒度仍然是“整块 LLC 全清”，只是实现上采用了
“valid-only reset”，而不是清空 `data/meta/repl`。

### 9.3 stale refill 保护

`invalidate_all` 被接受后会推进一个 epoch。

旧 epoch 的 clean refill install 在返回时会被丢弃，不允许 stale line 因为延迟返回而重新复活到 LLC 中。

对外推荐通过 `AXI_Interconnect` 的 maintenance API 使用：

- `set_llc_invalidate_all(bool)`
- `llc_invalidate_all_accepted()`

mode 切换现在首先依赖 `drain -> invalidate_all -> activate` 合同来保证边界干净；
这里的 epoch 仍然作为 stale clean refill 的附加保护网保留。

### 9.4 submodule mode / offset 运行时控制

当前 `AXI_Interconnect` 增加了一组 submodule 运行时控制输入：

- `mode[1:0]`
- `llc_mapped_offset[31:0]`

其语义如下：

- `mode=1`
  - 正常 LLC_ON
  - 请求仍按原有 cacheable / bypass 语义进入共享 LLC datapath
- `mode=2`
  - 地址映射模式
  - `[llc_mapped_offset, llc_mapped_offset + 4MB)` 内请求被翻译到
    direct-mapped 的本地 LLC 存储窗口
  - 窗口内访问不做 tag compare / replacement，不分配 MSHR，也不会从 DDR refill
  - 窗口外请求被强制转换为 `bypass`，再按 dual-port route helper 分类
  - 窗口外地址分类：
    - `addr >= 0x4000_0000` 走 DDR/SDRAM 口
    - 其它地址走 MMIO 口
    - DDR/SDRAM 口使用 `256-bit` beat；小于等于 `32B` 的读写对齐到单
      beat，`64B` cacheline 读写拆成同一事务的 2 个 beat
    - MMIO 口只支持 `4B / 1 beat` 读写
- `mode=0`
  - LLC_OFF
  - 不再向 LLC 新分配 line
- 编码值 `mode=3` 会先规范化为 `mode=0`，不是独立运行模式

当前 mode 切换采用 `drain -> invalidate_all -> activate` 合同：

- 当 `mode` 改变，或 `mode=2` 下 `llc_mapped_offset` 改变时，interconnect 先
  停止接收新的上游请求
- 旧模式下已捕获的 LLC/AXI 工作先排空到 quiescent point
- 只有到达 quiescent point 后，interconnect 才会请求一次 `invalidate_all`
- 只有 flush 被接受后，新的运行时配置才真正生效

这个做法的目的不是追求最省硬件，而是先保证：

- 旧模式下 resident 的 line 不会带入新模式
- 旧模式下的 inflight LLC/AXI 工作不会穿越模式边界
- 旧 epoch 返回的 refill 即使晚到，也不会在 flush 之后把 stale line 重新装回 LLC
- mode `0/3`、`1`、`2` 都能复用同一条已验证过的共享 datapath

## 10. corner cases 与当前已覆盖点

当前测试面已显式覆盖这些 corner cases：

- cacheable partial write miss 保留未触及字节
- partial write miss + dirty victim writeback
- bypass read 命中 resident line 时返回最新值
- bypass write hit 更新 resident line 但不置 dirty
- bypass write miss 不分配 LLC
- `invalidate_line` 不误伤同 set 其它 way
- `invalidate_line` 与 same-cycle upstream write accept/capture 互锁
- `invalidate_all` busy pulse 不是 silent accept
- `invalidate_all` hold-until-accept 合同
- `invalidate_all` 接受后 stale refill 不会重新安装
- victim writeback + maintenance + demand miss 三方交错
- refill-ready read victim 只等待真正 cache-resolved 的 same-line write
- 尚未 issued 的 same-line write lookup 不会阻塞 refill commit
- active `victim_wb` 忙时，read victim snapshot 可进入 `read_victim_wb_q` 而不阻塞 refill commit

## 11. 当前阶段边界

当前 AXI4-only 版本的目标是 correctness-first：

- 机制已经覆盖 shared LLC 的主要正确性语义
- 维护合同可以被外部观察与回归验证
- 测试面已覆盖 P0 / P1 / P2 当前阶段需要的 deterministic 与 fixed-seed 场景

仍属于未来性能阶段、而非当前 correctness blocker 的事项包括：

- 更激进的 replacement policy
- 更深的写执行资源并行
- 更复杂的 prefetch 策略

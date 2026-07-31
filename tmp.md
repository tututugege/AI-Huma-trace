结论：这是一个确定性的 L1 DCache / WriteBuffer 同地址读写顺序错误，不是 checkpoint、RefCPU 或 difftest 误报。

直接根因是 WB lookup 响应比对应的 DCache S2 请求晚了一个 fast-domain tick：代码根据当前请求计算 `nxt.bypass*`，却输出上一拍的 `cur.bypass*`。结果是脏行刚被逐出到 WB 时，对同一 cache line 的立即重读没有命中 WB，错误地分配 MSHR 并从下层读回旧数据。

## 1. Difftest 失败现场

原始命令稳定失败于：

- cycle：`507641`
- committed：`406525`
- commit PC：`0x000192a4`
- 指令：`0x000e5e83`
- Ref/DUT next PC：均为 `0x000192a8`
- privilege、CSR、其他 GPR、store side effect：全部一致
- 唯一错误：
  - REF `t4/x29 = 0x00000aff`
  - DUT `t4/x29 = 0x00000afc`

`0x000e5e83` 解码为：

```asm
lhu t4, 0(t3)
```

当时：

- `t3 = VA 0x9597c734`
- LSU 已翻译为 `PA 0x938aa734`
- cache line：`0x938aa700`
- set：156
- word offset：13
- tag：`0x24e2a`

故障时实际内存字为：

- REF：`0x013d0aff`
- DUT：`0x00a60afc`

`lhu` 读取低 16 位，正好得到：

- REF：`0x0aff`
- DUT：`0x0afc`

因此 difftest 报错完全准确。原始现场见[原始运行日志](/tmp/qm260730-astar-sp2.lLA1ek.log:42)和[GDB 状态](/tmp/qm260730-astar-gdb.rGLeYu.log:95)。

## 2. 数据如何被破坏

目标 line 的完整演化如下：

| Cycle | 事件 | word13 |
|---:|---|---:|
| 327619 | 从下层首次填入 L1D way3 | `0x00a60afc` |
| 327649 | store `data=0x00000aff, strb=0x03` | `0x00a60aff` |
| 327650 | store `data=0x013d0000, strb=0x0c` | `0x013d0aff` |
| 414870 | line 被 `0x828d6700` 冲突逐出；送入 WB 的数据仍正确 | `0x013d0aff` |
| 414870 | 同 line 的隐藏重试 load 到达，但看到 `wb_valid=0, mshr_hit=0` | 错误分配 MSHR |
| 414870 | 已发出的下层读返回旧副本，再次填入 L1D way1 | `0x00a60afc` |
| 454419 | 该 stale line 此后变脏并被逐出，旧 word13 被写回下层 | `0x00a60afc` |
| 505039 | 再次从下层填入 L1D way2 | `0x00a60afc` |
| 507617/507632 | 两次 load 均命中 stale L1D | `0x00a60afc` |
| 507641 | `lhu` 提交，difftest 首次观察到架构寄存器错误 | `t4=0x0afc` |

关键证据都在[目标 line 跟踪日志](/tmp/qm260730-astar-wb-trace.fDolHe.log:68)。

这里不是 WB 满导致的丢数据：逐出时 `wb_free=7`，WB 确实接收了正确的 `0x013d0aff`。真正的问题是下层读已经在未观察到 WB 的情况下被放行；随后虽然正确 WB 也完成了，但在途旧读没有被取消、覆盖或与 WB 合并。

## 3. 代码级根因

DCache S1 同时产生：

- 下一拍的 S1→S2 load slot；
- WB `bypass_req`；
- MSHR `find_req`。

见[RealDcache.cpp:239](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/MemSubSystem/RealDcache.cpp:239)和[RealDcache.cpp:303](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/MemSubSystem/RealDcache.cpp:303)。

WB 随后使用当前请求和最新 `nxt.write_buffer` 计算：

```cpp
nxt.bypassvalid[i]
nxt.bypassdata[i]
```

但实际发布的是：

```cpp
out.wb2dcache->bypass_resp[i].valid = cur.bypassvalid[i];
out.wb2dcache->bypass_resp[i].data  = cur.bypassdata[i];
```

见[WriteBuffer.cpp:49](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/MemSubSystem/WriteBuffer.cpp:49)。

这只延迟了 response，却没有同步延迟 DCache S2 slot，也没有在 response 中携带地址或 request ID。于是 `cur.bypass*` 实际对应前一个端口请求，和当前 `s1s2_cur` 错拍。

DCache S2 的优先级是：

1. WB bypass；
2. L1D hit；
3. MSHR hit；
4. 新分配 MSHR。

见[RealDcache.cpp:531](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/MemSubSystem/RealDcache.cpp:531)和[RealDcache.cpp:548](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/MemSubSystem/RealDcache.cpp:548)。因此错拍的 `wb_valid=0` 会直接把本应从 WB 转发的请求送入 MSHR。

当前配置每个 CPU cycle 还会执行多次 DCache/AXI fast tick，因此日志中 cycle 414870 出现多次 S1/WB 事件是正常的 fast-domain 展开，不是同一条指令重复执行。调用顺序见[MemSubsystem.cpp:826](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/MemSubSystem/MemSubsystem.cpp:826)、[MemSubsystem.cpp:969](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/MemSubSystem/MemSubsystem.cpp:969)和[rv_simu_mmu_v2.cpp:711](/nfs_global/S/daiyihao/project/qm-rocky/debug/260730/simulator/rv_simu_mmu_v2.cpp:711)。

## 4. 回归来源

`git blame` 将 WB 的两行 `cur` 输出定位到：

```text
a1ed19c8 dcache-bank-conflict-fix-2
2026-07-27
```

该提交把：

```cpp
nxt.bypassvalid/data
```

改成了：

```cpp
cur.bypassvalid/data
```

同一提交还把以下响应改成了 `cur`：

- WB store merge `valid/busy`
- MSHR `find_resp.valid/hit`

所以本 checkpoint 的直接触发点是 WB load bypass，但另外两组接口也存在同类“只延迟响应、不延迟消费者 slot”的潜在错拍风险，需要一起审计。

当前 HEAD `a327180 fix exu global flush mask comparison` 只修改 EXU flush mask 写法，不涉及 LSU/DCache/WB，不是这次错误的直接来源。

## 5. 两行消融验证

我在独立 `build-difftest-diag` 中只临时把 WB load bypass 两行从 `cur` 改回 `nxt`，其他 MSHR/merge 行为不变。

结果：

- cycle 414870：目标 load 正确选择 `source=wb`
- 后续目标 line 在 cycle 414896 回填时，word13 已为正确的 `0x013d0aff`
- 运行到 cycle `534533`
- committed `429400`
- 已超过原失败点：
  - 多运行 26892 cycles
  - 多提交 22875 条指令
- 未出现 difftest

见[消融验证日志](/tmp/qm260730-astar-wb-nxt-ablation.nP12UY.log:78)及[最终统计](/tmp/qm260730-astar-wb-nxt-ablation.nP12UY.log:281)。

这足以确认当前失败的直接因果关系，但它只是越过原必现点的窄验证，并没有完成整个 1000 万 warmup，因此不能称为完整回归关闭。

## 6. 建议的正式修复边界

最低风险候选是恢复 WB bypass 对当前请求的当拍输出，即使用 `nxt.bypassvalid/data`。不过正式修改时建议同时完成：

- 审计 WB bypass、WB merge、MSHR find 三组请求/响应是否与 S1/S2 slot 对齐。
- 如果必须注册 response，就必须把对应地址、端口请求和 S2 slot 一起注册，不能只注册 `valid/data`。
- 对“当前 dirty victim line == 当前 load/store line”增加直接转发或 replay，禁止分配下层读。
- MSHR fill 安装前检查同 line 是否存在更新的 WB/victim 副本，必要时合并或丢弃旧 fill。
- 增加断言：WB 或本拍 dirty victim 命中时，不允许同 line `mshr_req.valid`。
- 增加一个小型定向测试：脏行逐出、同 set 冲突填充、bank-conflict hidden retry、同行立即 load，并在 AXI div16 下运行。
- 最后再跑本 checkpoint 至完整 `-w 10000000 -c 1`。

另外，这条命令中 `-c 1` 表示 warmup 完成后只测量 1 条提交，不是总共只运行 1 条；checkpoint interval 是 `10000004`，所以 RefCPU 先走 4 步，O3 随后计划 warmup 1000 万条。Difftest 在 warmup 阶段始终开启，因此会提前在 406525 committed 处退出。

本次没有提交正式修复。临时插桩、两行消融和独立诊断 build 均已撤销/删除；当前工作树为干净的 `main...origin/main`，原始 `build/simulator` 未改动。分析方法采用隔离构建和针对性正确性跟踪，并按显式 comb/seq 契约判断接口错拍。

# P1:trace bridge(功能 core 跟随模型的 dispatch 流)

> 历史记录:P3 迁移之前的状态(2026-11-04),可能已过时,现状见 [README.md](README.md)。

## 状态
- P1a 完成:src/TraceBridge.{h,cpp} + tests/bridge_selftest.cpp。只使用现有的 librefcpu API(不改 function-model)。
- P1b 完成:模型中的 tap + check 模式(TB_CHECK=1),patches/0003-p1b-bridge-check-mode.patch。

## 设计
- 标识:`uop.dbg.inst_idx`(在 Idu decode 中分配,随 uop 一直带到 ROB)即 uid。
  tap 时需要验证的注意点:该计数器是 static 变量,每次 decode 求值时递增;对于已 dispatch 的指令,它不能跳号/重号。
- dispatch(uid, pc, inst) 按 dispatch 顺序,即进入 ROB 的指令的 fetch 顺序。如果 pc != core 的下一个 pc,
  core 被强制跳转(错误路径以及嵌套的错误路径都跟随模型的流)。
- 每条指令一份 undo log(不是约 270 B 的 RefCpuState):pc、privilege、reservation、rd 旧值、trap 会写的 8 个 CSR
  (mstatus sstatus mepc mcause mtval sepc scause stval)、仅对 SYSTEM 指令保存整个 CSR 文件,
  以及 store/AMO 的旧 memory word。Rewind 时在一份状态拷贝上从最年轻的开始依次应用 log。
- 被 block 的情况(不做推测执行,保持状态,pc+4):pc 在 RAM 之外、ebreak(refcpu 会结束运行)、
  非对齐 load/store(refcpu exit(-1))、RAM 之外的 load/store(MMIO 或非法:不做推测性的设备访问)。
- 推测性 trap 立即撤销(状态 + store),流继续从 pc+4 走;committed 路径上的 trap
  在 commit 时执行(P1b)。
- 每条指令的 fact:next pc、rd/value、branch taken、load/store vaddr/size/store data、blocked/trapped 标志。

## 检查(P1a)
- 自测:普通运行(无错误路径)对比带随机错误路径的运行(从随机 text pc、垃圾 pc、栈区域取 1-24 条指令),
  回滚到最后一条正确路径的 uid,commit 滞后 16。
  每条指令必须一致:next pc、rd value、br_taken、vaddr、store data、blocked;以及最终的寄存器、CSR、pc、
  memory hash。结果:56/56 个已构建的测试在 50% 错误路径注入下通过(最多 300k 条指令)。
- 观察到的错误路径流量,例如 prime 在 100% 下:dispatch 86,647 条,强制跳转 6,371 次,推测性 trap 23,972 次,
  pc 在 RAM 之外 7,505 次,ebreak 664 次,非对齐 130 次,RAM 之外访问 1,334 次。
- 负对照(编译开关):不做 rd undo -> 不一致;不做 store undo -> 不一致。
- 限制:尚未处理地址翻译(satp)(load/store 把 vaddr 当 paddr 用),没有 MMIO/timer/interrupt 的 fact,
  推测执行的 CSR 指令只恢复 CSR 文件(设备副作用没有建模),
  AMO/LR/SC 的 reservation 语义仅通过保存的 reservation 实现。

## 下一步(P1b)
1. tap Dispatch(dis_fire -> bridge.dispatch,带 dbg.inst_idx、dbg.pc、dbg.instruction)。
2. tap Exu mispredict(mispredict uop 的 uid)和 ROB flush -> rewind_after;ROB commit -> commit()。
3. 在执行处比较:ExuRobUop.result 对比 fact.rd_value、br_taken,AGU 地址对比 fact.vaddr。
4. commit 时的异常/中断/CSR 副作用:在 commit 时真正执行。

## P1b 结果(check 模式,不注入)
tap(除非设置了 TB_CHECK 否则全是 no-op;不设置时 56 个测试的 gate 不变:cycle 数相等,log 相同):
- dispatch:Rob::comb_fire(uid = dbg.inst_idx;同时记录 rob_idx -> uid,因为 LSU 路径会丢失 dbg.inst_idx)。
- mispredict:Rob::comb_branch -> rewind_after(branch uid)。commit:BackTop commit 循环 -> commit(uid)。
- flush:BackTop(rob_bcast->flush)-> 在下一次 dispatch 时 bridge core 取 difftest reference 的状态
  (reference 已经执行了引起 flush 的指令,包括 trap 或 CSR 副作用)。
- exec:Exu::comb_fu2exu_collect 末尾,比较送往 ROB 的内容:rd value(add/mul/div/csr/fp/load)、分支
  方向、jal/jalr/branch 的 next pc(diag_val)、load/store 地址。
Patch(core 无法得知的值):MMIO load 结果(被 block 的 MemOutOfRam)和值不同的 CSR 读(mip)。bridge 回滚更年轻的条目,
写入 rd,再重放它们。56 个测试:35 个 mmio_load + 2 个 csr_read patch。

56 个 gate 测试上的结果(scripts/check_run.sh,results/check2):
- 比较的 rd 169,370;branch dir 61,200;addr 103,293;target 103,785。
- 不一致 0 / 0 / 0 / 0;nofact 0;uid 冲突 0;commit_front_miss 0。
- check 模式下 cycle 与基线完全相同(它只做观察)。
- patch 之前:200 个 rd + 19 个 br + 34 个 addr + 19 个 target 不一致,全部在 4 个 interrupt/timer 测试中
  (timer MMIO 读和 mip 读会影响后续的值)。
- 负对照(TB_NEG_NO_REWIND=1:忽略 mispredict 回滚):prime 3,866 个 rd + 112 个 br 不一致,
  quick-sort 1,889 个 rd + 512 个 br + 527 个 addr 不一致。
限制:没有地址翻译(satp)处理;store data 以及非对齐/AMO/LR-SC 的 fact 尚未比较;
commit 时的异常依赖 difftest reference 的状态;错误路径上看起来像 MMIO 的 load 用
模型的值做 patch(为了保持错误路径地址相同而必需),非对齐 load 被跳过。

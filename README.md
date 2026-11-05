# trace-model

The timing model and architecture-exploration platform of the AI-Huma project (function-model + trace-model + rtl).
It started as a trace-driven, cycle-identical distillation of tututugege/simulator (an RTL-style, execution-driven, cycle-accurate
out-of-order RISC-V simulator); that base was finished on 2026-11-04. Since then the work is our own optimization: ideas are
measured here first (cycles, IPC, queue/port usage), then carried to the RTL; the original model's cycle counts are no longer the target
(first merged result: the batch-3 area configuration, 2026-11-05).

Branch direction/target, addresses, store data and CSR write data come from facts supplied by the trace bridge (`diff/TraceBridge.*`),
a functional core (the sibling `function-model` repository, built as `librefcpu`; `function-model/` here is a symlink to `../function-model`) that follows the model's dispatch stream including wrong paths. Every change
must keep the gate's cycle counts (`results/baseline/baseline.csv`) unless it is an intentional configuration change merged to main.

See [docs/PROJECT.md](docs/PROJECT.md) for the architecture, the gate, the verification harnesses and the status.

## Build and run

```bash
gmake -C function-model librefcpu.a LDFLAGS="-lz"     # builds ../function-model (symlink); needs the function-model repository next to this one
gmake LDFLAGS="-lz" -j8
./build/simulator path/to/program.bin
bash scripts/run_gate.sh build/simulator results/gate      # 62 tests, pass = cycles equal to results/baseline/baseline.csv
```

## Origin

Forked from tututugege/simulator at 2f31e8d (`baseline.lock`). The upstream README, the design documents of the original
(execution-driven) modules, the upstream CI and review workflows, the SPEC scoring scripts and the vendored riscv-tests copy were removed
on 2026-11-04; they remain in the git history of this repository.

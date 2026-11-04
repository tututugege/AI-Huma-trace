# trace-model

Trace-driven, cycle-identical timing model distilled from tututugege/simulator (an RTL-style, execution-driven, cycle-accurate
out-of-order RISC-V simulator). Part of the AI-Huma project (function-model + trace-model + rtl).

Branch direction/target, addresses, store data and CSR write data come from facts supplied by the trace bridge (`diff/TraceBridge.*`),
a functional core (`GenSimpoint`, built as `librefcpu`) that follows the model's dispatch stream including wrong paths. Every change
must keep identical cycle counts: cycle equality is the correctness gate.

See [docs/PROJECT.md](docs/PROJECT.md) for the architecture, the gate, the verification harnesses and the status.

## Build and run

```bash
git submodule update --init
gmake -C GenSimpoint librefcpu.a LDFLAGS="-lz"     # first time on a fresh checkout
gmake LDFLAGS="-lz" -j8
./build/simulator path/to/program.bin
bash scripts/run_gate.sh build/simulator results/gate      # 62 tests, pass = cycles equal to results/baseline/baseline.csv
```

## Origin

Forked from tututugege/simulator at 2f31e8d (`baseline.lock`). The upstream README, the design documents of the original
(execution-driven) modules, the upstream CI and review workflows, the SPEC scoring scripts and the vendored riscv-tests copy were removed
on 2026-11-04; they remain in the git history of this repository.

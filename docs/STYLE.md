# trace-model code style

The rules the model follows (exp/style, 2026-11-16). Module structure, contracts and generated schedules:
[MODULE_CONTRACT.md](MODULE_CONTRACT.md).

## Names

* Types: CamelCase (`FrontTopIn`, `InstFifoEntry`, `ICacheRegs`); acronyms stay upper case (`MSHRState`, `FTQEntry`).
  Scalar aliases keep the `_t` suffix (`pc_t`, `tage_idx_t`). Not renamed: `CPU_state` / `Ckpt_CPU_state` (the
  function-model's checkpoint layout) and the `enum_*` opcode / CSR enums of the original simulator.
* No `struct` keyword in front of a type name outside its definition.
* Functions, members, locals: snake_case.
* A module's comb phase methods: `comb_<what>` (`comb_outputs`, `comb_stage1`); its clock edge: `seq`; reading its
  registers' q (a snapshot for the cycle): `read_q`. Helper functions are named after what they compute (`predecode`,
  `predecode_check`, `front_output`), without a `_comb` suffix.
* Registers: `Regs q_` / `Regs d` (MODULE_CONTRACT.md), SimCpu's own registers too (the fetch-group and MMIO-response
  holds). Exception: the I-cache port registers `icache_axi_q` / `icache_resp_q`, loaded at the start of the cycle from
  the interconnect's port (REG_WIRE.md). No `_1` suffix for next-state values.

## Types

* Hardware widths: `wire<N>` for a signal, `reg<N>` for a register (base_types.h: the narrowest integer type of N bits).
  The front end's old `wireN_t` / `wire_for_bits_t<N>` are gone.

## Errors and output

* A broken model invariant: `Assert(cond)` (compiled out with NDEBUG) or `fatal(fmt, ...)` (include/fatal.h; prints and
  exits with 1, always). Not `printf` + `exit(1)`. The driver's checkpoint-file errors (bad input, std::cerr) are
  separate.
* Output text that logs are compared against (console, PMU, FPR) is not changed by style work.

## Ownership

* A module owns its submodules by value or `std::unique_ptr` (BackTop's modules, the ITLB, FastWarm's state); raw
  pointers are non-owning bindings (ports, `ctx`). Exception: TraceBridge's entry chain, whose entries are
  referenced by in-flight uops.
* No file-scope mutable state in the model: state lives in the module that owns it (the front end: FrontCore; Isu's
  write-back busy tables; the LSU's memory-dependence predictor; the D-cache arrays: DcacheArrays in RealDcache).
  Exceptions: the uop pool (UopPool, shared storage of the in-flight uops), the functional memory binding (PhysMemory),
  the cycle counter `sim_time`, PMU event buses (`g_pmu_*`).
* Environment variables are read in one place: `sim_env()` (include/env.h); docs/ENV_VARS.md lists them.
* No `memset` on struct / class objects: `x = {}`, `{}` member initializers, or a loop over an array of structs.
  Plain integer arrays may be cleared with memset; the LSU's 0xA5 poison fill (a debug aid) and memory pages keep it.
* A per-cycle default (the start of a comb phase, a port nobody drives this cycle) clears the valid / enable bits, not
  the whole struct: the other fields are read only under them (the RTL's always_comb default). Done where the struct
  is large: the front modules' first phases, BackTop::comb_in_regs (back.in's fetch group),
  axi.comb_begin (the masters' req.valid); the small output-port clears in the memory subsystem and the LSU stay.
  A writer that sets valid drives the whole entry (no field left over from an earlier cycle); stand-alone control
  signals (retry, miss, busy) are cleared every cycle.
* Module ports: `struct In` (const pointers to what the module reads) and `struct Out` (what it drives), bound by the
  parent (BackTop::init, MemSubsystem::init, the FrontCore constructor); no module reaches a shared struct of all signals.

## Comments and docs

* Code comments are in English (project documents outside the code are Chinese). A comment says what the code does
  or why, in the current terms; stale comments are rewritten or deleted, not left beside the code.
* No commented-out code: the history is in git.
* A document that describes a superseded structure is either updated or marked historical at its top.

# AXI Interconnect Kit

Standalone AXI4 memory subsystem extracted from the simulator.

## Current Mainline

The current mainline is an integrated dual external AXI-port design:

- The CPU side still uses the simplified `read_ports[4]` / `write_ports[2]`
  upstream interface.
- The downstream side is split into a DDR/SDRAM AXI port and an MMIO AXI port.
- The DDR/SDRAM port uses 256-bit beats and supports 64B cache-line transfers
  as two beats.
- The MMIO port uses 32-bit beats and only accepts 4B single-beat transfers.
- Read and write outstanding budgets are shared across the two external ports
  but remain independent from each other. The dual-port profile target is 32
  reads and 32 writes.

The legacy single-AXI-port plus `AXI_Router_AXI4` path remains for historical
tests, demos, and bring-up. The current C++ interface is documented in
[docs/interfaces.md](docs/interfaces.md).

## Scope

- One AXI4 interconnect with simplified upstream ports:
  - `read_ports[4]`: `icache`, `dcache_r`, `uncore_lsu_r`, `extra_r`
  - `write_ports[2]`: `dcache_w`, `uncore_lsu_w`
- dual external AXI ports, legacy/demo AXI4 router, SimDDR downstream memory
  model, MMIO bus, UART16550 device
- Optional shared unified LLC on the AXI4 path

AXI3 support has been removed from this repository. The kit is now AXI4-only.

## Topology

```text
Read/Write masters
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
        +----------------------+
        |                      |
        v                      v
+---------------+       +---------------------+
| DDR/SDRAM AXI |       | MMIO AXI            |
| 256-bit beat  |       | 32-bit beat         |
+---------------+       +---------------------+
        |                      |
        v                      v
+----------+            +---------------------+
| SimDDR   |            | MMIO_Bus + UART16550|
+----------+            +---------------------+
```

The integrated dual-port mainline performs DDR/MMIO routing inside the
interconnect/bridge. `AXI_Router_AXI4` is still available for the legacy
single-port/demo path.

Terminology used in this repository:

- `upstream`: request sources connected to `read_ports[]` / `write_ports[]`
  and the response paths that return to those masters
- `downstream`: the DDR-side or MMIO-side interfaces below the interconnect,
  including `axi_ddr_io`, `axi_mmio_io`, legacy `AXI_Router_AXI4`, `SimDDR`,
  and MMIO devices

## LLC Summary

`AXI_LLC` sits behind the AXI4 interconnect and models a shared unified cache.

Default configuration:

- `8MB`
- `64B` line
- `16-way`
- default `8` MSHRs
- lookup latency `3`
- `PIPT`, `unified`, `NINE`
- prefetch disabled by default

Current behavior:

- Cacheable reads allocate and refill through external SRAM-style `data/meta/repl`
  tables supplied by the parent simulator.
- `CONFIG_AXI_LLC_DCACHE_READ_MISS_NOALLOC` controls DCache demand read-miss
  allocation:
  - `0` (default): install the refill into LLC
  - `1`: keep the miss on a no-allocate path while still returning refill data
    upstream
- AXI4 interconnect read upstream side supports multiple outstanding contexts:
  - dual-port profile global limit `32`
  - dual-port profile per-read-master limit `32`
  - standalone CMake defaults may be smaller unless explicitly configured
- LLC cacheable demand-miss execution is still more restrictive:
  - one read master can have at most one cacheable demand miss actively owned by
    the LLC at a time
  - other masters can still consume remaining global read resources
  - bypass reads are not subject to that same-master cacheable-miss restriction
- Cacheable writes are owned by the LLC path.
- Cacheable partial write miss performs `refill old line -> merge bytes ->
  install dirty line`; it does not merge onto an all-zero line.
- Bypass reads probe LLC first:
  - hit returns latest resident line
  - miss goes downstream without allocation
- Bypass writes are write-through maintenance operations:
  - hit updates resident line without setting `DIRTY`
  - miss goes downstream without allocation
- Sub-line writes merge by `addr % line_bytes`.
- `invalidate_all` is conservative:
  - it is accepted only when there is no dirty resident line, dirty victim
    writeback, or write-side hazard pending
  - callers should hold the request until `invalidate_all_accepted` is observed
  - already captured clean LLC-path work may drain while `invalidate_all` is
    pending
  - it drops stale clean refill installs by epoch once accepted
  - the external table runtime now hosts a dedicated byte-per-way `valid` table;
    `invalidate_all` resets that table only, so stale
    data/meta/repl contents may remain physically present but unreachable
  - it does not silently discard dirty resident data
- The interconnect also carries prototype runtime controls for the submodule:
  - `mode=1`: LLC_ON
  - `mode=2`: treat `[offset, offset + 4MB)` as a direct-mapped local LLC
    storage window; accesses inside the window stay local and do not allocate
    MSHRs or refill from DDR, while accesses outside the window are forced to
    `bypass`
  - `mode=0`: LLC_OFF
  - encoded `mode=3` is canonicalized to `mode=0`; it is not a separate
    runtime mode
  - mode/offset changes follow a `drain -> invalidate_all -> activate`
    contract: no new upstream requests are accepted during reconfiguration, the
    old in-flight LLC/AXI work drains first, and the new configuration becomes
    active only after `invalidate_all_accepted`

### AXI4 Write Concurrency

The current AXI4 write design is:

- Interconnect upstream side accepts up to `MAX_WRITE_OUTSTANDING` pending writes.
- Non-LLC path drains pending writes downstream and routes B responses by AXI ID.
- LLC path keeps:
  - per-master pending write queues
  - one active write context per write master
  - shared lookup engine
  - shared victim-writeback resource
  - shared downstream memory write port
- Same-master promotion waits until the previous write response slot is consumed.

`MAX_WRITE_OUTSTANDING` is therefore an upstream queueing bound, not a total
global bound on every LLC-internal write state bit combined with interconnect
state.

This is the current write-side correctness target for this stage. Future
performance work, if needed, would deepen internal write-resource parallelism
rather than reopen basic ordering/coherence rules.

## SimDDR Modeling Notes

- `AXI_KIT_SIM_DDR_BEAT_BYTES` supports `4/8/16/32B`; `32B` is the required
  setting for AXI4 256-bit single-beat traffic.
- `AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY` means: after the final `W` beat
  handshakes, wait this many additional full cycles before `B` can first become
  visible.
- In the parent simulator, `AXI_KIT_*` integration parameters come from
  `config.h`; missing required definitions are treated as build errors rather
  than silently falling back to submodule defaults.
- `SimDDR` remains a fixed-latency functional model. It is not intended to be a
  cycle-accurate DDR controller model, and it does not yet model richer `AW/W`
  backpressure or write-channel scheduling effects.

## Test Tiers

- `P0`: deterministic component tests
  - LLC read/write hit/miss
  - bypass read/write semantics
  - victim writeback
  - maintenance
  - stale refill protection
  - write-queue edge cases
- `P1`: deterministic AXI4 + LLC + SimDDR integration tests
  - cacheable+bypass coexistence
  - queued writes
  - maintenance interlocks
  - invalidate-all epoch behavior
- `P2`: fixed-seed mixed stress
  - mixed coherence stress
  - refill/maintenance/writeback races

Validated toolchains:

- `qm` environment default toolchain
- system `g++`
- Conda-hosted `x86_64-conda-linux-gnu-c++` in the `qm` environment

## Interface Documentation

- [docs/submodule_architecture_CN.md](docs/submodule_architecture_CN.md)
- [docs/interfaces.md](docs/interfaces.md)
- [docs/interfaces_CN.md](docs/interfaces_CN.md)
- [docs/llc_design_CN.md](docs/llc_design_CN.md)

## Main Files

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

## Current Closure

This repository is currently stabilized around the AXI4-only LLC/interconnect
correctness scope implemented here, with conservative maintenance semantics.
Final closure wording is intentionally deferred pending external review of the
maintenance barrier and upstream-side race regressions. Parent-simulator
integration bugs should be debugged outside this submodule unless the root
cause is clearly inside the kit itself.

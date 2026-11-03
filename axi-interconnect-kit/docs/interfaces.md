# AXI Interconnect Kit Interface Specification

This document lists:

1. Interconnect upstream interfaces
2. Dual external AXI4 channel signals

For the current architecture diagram and constraints, see
[submodule_architecture_CN.md](submodule_architecture_CN.md).

## 1) Interconnect Upstream Interfaces

Defined in `axi_interconnect/include/AXI_Interconnect_IO.h`.

### 1.1 Read Master Request (`ReadMasterReq_t`)

| Signal | Width | Direction (vs interconnect) | Meaning |
|---|---:|---|---|
| `valid` | 1 | input | Request is valid |
| `ready` | 1 | output | Interconnect accepts request |
| `addr` | 32 | input | Byte address |
| `total_size` | 8 | input | Transfer bytes minus 1 (`0=1B`, `255=256B`) |
| `id` | 4 | input | Upstream transaction ID |
| `bypass` | 1 | input | Bypass LLC allocation/ownership rules |

### 1.2 Read Master Response (`ReadMasterResp_t`)

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `valid` | 1 | output | Response valid |
| `ready` | 1 | input | Master accepts response |
| `data` | 2048 (`64x32`) | output | Wide read response payload (up to 256B) |
| `id` | 4 | output | Upstream ID echoed back |

### 1.3 Write Master Request (`WriteMasterReq_t`)

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `valid` | 1 | input | Request valid |
| `ready` | 1 | output | Interconnect accepts request |
| `addr` | 32 | input | Byte address |
| `wdata` | default 512 (`16x32`) | input | Wide write payload (`AXI_KIT_MAX_WRITE_TRANSACTION_BYTES`) |
| `wstrb` | 64 | input | Byte strobes (1 bit per byte) |
| `total_size` | 8 | input | Transfer bytes minus 1 (`0=1B`, default max `63=64B`) |
| `id` | 4 | input | Upstream transaction ID |
| `bypass` | 1 | input | Write-through maintenance / bypass semantics |

### 1.4 Write Master Response (`WriteMasterResp_t`)

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `valid` | 1 | output | Response valid |
| `ready` | 1 | input | Master accepts response |
| `id` | 4 | output | Upstream ID echoed back |
| `resp` | 2 | output | AXI response code |

### 1.5 LLC Maintenance Control Surface

The AXI4 interconnect exposes an LLC maintenance control surface through
setter/getter methods on `AXI_Interconnect`:

- `set_llc_invalidate_all(bool)`
- `set_llc_invalidate_line(bool, uint32_t line_addr)`
- `llc_invalidate_all_accepted()`
- `llc_invalidate_line_accepted()`

Contract:

1. Callers must hold `invalidate_all` / `invalidate_line` requests until the
   corresponding `*_accepted()` pulse is observed.
2. `invalidate_all` is conservative:
   - new upstream LLC requests are blocked while it is pending
   - already captured clean LLC-path work may drain
   - it is accepted only when there is no dirty resident line, dirty victim
     writeback, or write-side hazard pending
3. `invalidate_line` is targeted maintenance:
   - it is rejected when the same line still has any write-side hazard in the
     upstream request path, the LLC internal write pipeline/queue/response
     state, or the downstream write path
   - concrete examples include inflight miss, active write context, queued
     write, write lookup, victim writeback, active/pending bypass write, and
     same-cycle upstream write accept/capture hazards
4. After `invalidate_all` is accepted, stale clean refill installs from an
   older epoch are dropped instead of being re-installed into the LLC.
5. In the current prototype, effective line validity comes from a dedicated
   `valid` table; `meta.flags.VALID` remains only as a shadow/debug-compat bit.
   Lookup completion therefore requires an explicit `lookup_in.valid_valid`
   response with a complete valid row; the model must not infer validity from
   `meta.flags.VALID`.
   The valid row is byte-per-way so one-way updates can use byte write enables
   without clearing neighboring ways.

### 1.6 Submodule Runtime Mode Controls

`AXI_Interconnect` currently also exposes a small runtime control surface for
the submodule prototype:

- `mode[1:0]`
- `llc_mapped_offset[31:0]`

Current behavior:

1. `mode=1`
   - LLC_ON mode
   - requests keep using the shared LLC datapath
   - the original upstream `bypass` semantics are preserved
2. `mode=2`
   - mapped-address mode
   - requests inside `[llc_mapped_offset, llc_mapped_offset + 4MB)` are
     translated onto a direct-mapped local LLC storage window
   - those window accesses stay local: no tag compare, no replacement, no MSHR
     allocation, and no DDR refill
   - requests outside that window are forced to `bypass` and continue through
     the DDR/MMIO downstream path
3. `mode=0`
   - LLC_OFF mode
   - no new LLC line allocation happens in this mode

Encoded `mode=3` is canonicalized to `mode=0`; it is not a separate runtime
mode.

Current transition contract:

1. A change in `mode` or `llc_mapped_offset` enters a reconfiguration phase
2. During reconfiguration:
   - no new upstream requests are accepted
   - the old LLC/AXI in-flight work drains to a quiescent point first
3. Only after that quiescent point does interconnect request `invalidate_all`
4. The new runtime `mode/offset` becomes active only after
   `invalidate_all_accepted`

Implementation note:

- The local simulator harness may seed these inputs at `init()` time through
  `AXI_SUBMODULE_MODE` / `AXI_SUBMODULE_OFFSET`. That is only a simulation
  bring-up path; the datapath semantics still follow the live inputs.

## 2) Dual External AXI4 Channel Signals

Defined in `sim_ddr/include/SimDDR_IO.h`.

The current mainline has two external AXI master ports:

- DDR/SDRAM: `axi_ddr_io`
  - 6-bit ID profile
  - 256-bit data, 32B beat
  - requests <=32B are aligned to one 32B beat; 64B cache-line traffic uses two beats
- MMIO: `axi_mmio_io`
  - 6-bit ID profile
  - 32-bit data, 4B beat
  - only 4B single-beat reads/writes are supported

The old C++ `axi_io` field is a legacy alias to the DDR port.

### 2.1 Write Address Channel (`AW`)

| Signal | Width | Direction (master->slave) | Meaning |
|---|---:|---|---|
| `awvalid` | 1 | M->S | Address valid |
| `awready` | 1 | S->M | Address ready |
| `awid` | DDR 6 / MMIO 6 | M->S | Write transaction ID |
| `awaddr` | 32 | M->S | Byte address |
| `awlen` | 8 | M->S | Burst length minus 1 |
| `awsize` | 3 | M->S | `log2(bytes_per_beat)` |
| `awburst` | 2 | M->S | Burst type |

### 2.2 Write Data Channel (`W`)

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `wvalid` | 1 | M->S | Data valid |
| `wready` | 1 | S->M | Data ready |
| `wdata` | DDR 256 / MMIO 32 | M->S | Write payload |
| `wstrb` | DDR 32 / MMIO 4 | M->S | Byte enable bits |
| `wlast` | 1 | M->S | Last beat |

### 2.3 Write Response Channel (`B`)

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `bvalid` | 1 | S->M | Response valid |
| `bready` | 1 | M->S | Response ready |
| `bid` | DDR 6 / MMIO 6 | S->M | Response ID |
| `bresp` | 2 | S->M | Response code |

### 2.4 Read Address Channel (`AR`)

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `arvalid` | 1 | M->S | Address valid |
| `arready` | 1 | S->M | Address ready |
| `arid` | DDR 6 / MMIO 6 | M->S | Read transaction ID |
| `araddr` | 32 | M->S | Byte address |
| `arlen` | 8 | M->S | Burst length minus 1 |
| `arsize` | 3 | M->S | `log2(bytes_per_beat)` |
| `arburst` | 2 | M->S | Burst type |

### 2.5 Read Data Channel (`R`)

| Signal | Width | Direction | Meaning |
|---|---:|---|---|
| `rvalid` | 1 | S->M | Data valid |
| `rready` | 1 | M->S | Data ready |
| `rid` | DDR 6 / MMIO 6 | S->M | Response ID |
| `rdata` | DDR 256 / MMIO 32 | S->M | Read payload |
| `rresp` | 2 | S->M | Response code |
| `rlast` | 1 | S->M | Last beat |

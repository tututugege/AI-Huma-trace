---
name: axi-kit-dev
description: "Develop or refactor AXI Interconnect Kit modules (AXI3/AXI4 interconnect, router, SimDDR, MMIO) with two-phase comb/seq timing discipline."
---

# AXI Kit Development

## Scope
- `axi_interconnect/*`
- `sim_ddr/*`
- `mmio/*`

## Rules
- Keep two-phase execution order explicit: `comb_outputs -> comb_inputs -> seq`.
- Keep AXI handshake correctness:
  - `AR/AW valid` must hold until handshake.
  - `R/B valid` must hold while upstream `ready=0`.
- Keep upstream ready-first behavior:
  - `req.ready` is registered and consumed next cycle.
- For multi-master routing:
  - maintain strict ID encode/decode symmetry.
  - keep one source of truth for master-id packing.

## Build/Test
```bash
cmake -S projects/axi-interconnect-kit -B projects/axi-interconnect-kit/build
cmake --build projects/axi-interconnect-kit/build -j
cd projects/axi-interconnect-kit/build && ctest --output-on-failure
```

## Integration guardrails
- Do not include parent simulator headers from this project.
- Shared defaults must come from `include/axi_interconnect_compat.h`.

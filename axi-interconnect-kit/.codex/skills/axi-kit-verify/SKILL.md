---
name: axi-kit-verify
description: "Run focused AXI-kit verification: unit tests, smoke demos, and integration checks from the parent simulator."
---

# AXI Kit Verification

## Standalone tests
```bash
cmake -S projects/axi-interconnect-kit -B projects/axi-interconnect-kit/build
cmake --build projects/axi-interconnect-kit/build -j
cd projects/axi-interconnect-kit/build && ctest --output-on-failure
```

## Standalone demos
```bash
projects/axi-interconnect-kit/build/axi4_smoke_demo
projects/axi-interconnect-kit/build/axi3_smoke_demo
```

## Parent simulator checks
AXI4:
```bash
cmake -S . -B build -DUSE_SIM_DDR=ON -DUSE_SIM_DDR_AXI4=ON
cmake --build build -j
timeout 20s ./build/a.out baremetal/new_dhrystone/dhrystone.bin
timeout 20s ./build/a.out baremetal/new_coremark/coremark.bin
timeout 60s ./build/a.out baremetal/linux.bin
```

AXI3:
```bash
cmake -S . -B build_axi3 -DUSE_SIM_DDR=ON -DUSE_SIM_DDR_AXI3=ON
cmake --build build_axi3 -j
timeout 20s ./build_axi3/a.out baremetal/new_dhrystone/dhrystone.bin
timeout 20s ./build_axi3/a.out baremetal/new_coremark/coremark.bin
timeout 60s ./build_axi3/a.out baremetal/linux.bin
```

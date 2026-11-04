# LLVM is the default; override tool paths for installations outside PATH.
CLANG ?= clang
LD_LLD ?= ld.lld
OBJCOPY ?= llvm-objcopy
PYTHON ?= python3
SIMULATOR ?= ../../build/simulator
BUILD_DIR ?= build
CASES := dependent-x0 dependent-x31 dependent-zero sc-x0 lr-x0
BINS := $(addprefix $(BUILD_DIR)/,$(addsuffix .bin,$(CASES)))

.PHONY: all run
.SECONDARY:
all: $(BINS)

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/dependent-x0.o: dependent.S | $(BUILD_DIR)
	$(CLANG) --target=riscv32-unknown-elf -march=rv32i -mabi=ilp32 -c $< -o $@

$(BUILD_DIR)/dependent-x31.o: dependent.S | $(BUILD_DIR)
	$(CLANG) --target=riscv32-unknown-elf -march=rv32i -mabi=ilp32 -DLOAD_DEST=x31 -c $< -o $@

$(BUILD_DIR)/dependent-zero.o: dependent.S | $(BUILD_DIR)
	$(CLANG) --target=riscv32-unknown-elf -march=rv32i -mabi=ilp32 -DLOAD_OFFSET=1 -c $< -o $@

$(BUILD_DIR)/sc-x0.o: sc-x0.S | $(BUILD_DIR)
	$(CLANG) --target=riscv32-unknown-elf -march=rv32ia -mabi=ilp32 -c $< -o $@

$(BUILD_DIR)/lr-x0.o: lr-x0.S | $(BUILD_DIR)
	$(CLANG) --target=riscv32-unknown-elf -march=rv32ia -mabi=ilp32 -c $< -o $@

$(BUILD_DIR)/%.elf: $(BUILD_DIR)/%.o link.ld
	$(LD_LLD) -m elf32lriscv --no-relax -T link.ld $< -o $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

run: all
	$(PYTHON) run.py --binary $(SIMULATOR) --build-dir $(BUILD_DIR)

# Direct host-side tests of the real LSU implementation; no RISC-V tools needed.
ROOT := ../..
HOST_INCLUDES := $(addprefix -I$(ROOT)/,include libs/include back-end/include \
  back-end/Exu/include back-end/Lsu/include back-end/tools/include \
  MemSubSystem/include diff/include function-model/include/api front-end \
  axi-interconnect-kit/include axi-interconnect-kit/axi_interconnect/include \
  axi-interconnect-kit/sim_ddr/include axi-interconnect-kit/mmio/include)
ifeq ($(shell uname -s),Darwin)
HOST_GC := -Wl,-dead_strip
else
HOST_GC := -Wl,--gc-sections
endif
.PHONY: host-test
host-test: | $(BUILD_DIR)
	$(CXX) -std=c++20 -O2 -ffunction-sections -fdata-sections $(HOST_INCLUDES) \
	  stlf-unit.cpp $(ROOT)/back-end/Lsu/RealLsu.cpp \
	  $(ROOT)/back-end/Lsu/RealLsuPerf.cpp $(HOST_GC) -o $(BUILD_DIR)/stlf-unit
	./$(BUILD_DIR)/stlf-unit

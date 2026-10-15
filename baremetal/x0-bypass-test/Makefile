# LLVM is the default; override tool paths for installations outside PATH.
CLANG ?= clang
LD_LLD ?= ld.lld
OBJCOPY ?= llvm-objcopy
PYTHON ?= python3
SIMULATOR ?= ../../build/simulator
BUILD_DIR ?= build
CASES := dependent-x0 dependent-x31 dependent-zero lr-x0
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

$(BUILD_DIR)/lr-x0.o: lr-x0.S | $(BUILD_DIR)
	$(CLANG) --target=riscv32-unknown-elf -march=rv32ia -mabi=ilp32 -c $< -o $@

$(BUILD_DIR)/%.elf: $(BUILD_DIR)/%.o link.ld
	$(LD_LLD) -m elf32lriscv --no-relax -T link.ld $< -o $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

run: all
	$(PYTHON) run.py --binary $(SIMULATOR) --build-dir $(BUILD_DIR)

# ==========================================
# Optimized Simulator Makefile
# ==========================================

BUILD_DIR ?= build
.DEFAULT_GOAL := all
SIM_EXE   := $(BUILD_DIR)/simulator
IMG     := ./baremetal/memory
AXI_KIT_DIR := ./axi-interconnect-kit

# Compiler & Flags
CXX      ?= g++
CXXFLAGS := -O3 -march=native -funroll-loops -mtune=native
CXXFLAGS += -MMD -MP 
CXXFLAGS += -Wall -Wextra -Wno-unused-parameter
CXXFLAGS += --std=c++2a
CXXFLAGS += $(EXTRA_CXXFLAGS)
# register width of the function model (rv32 default; XLEN=64 needs an rv64 librefcpu: REFCPU_LIB=<its path>, and BUILD_DIR=build64)
XLEN ?= 32
CXXFLAGS += -DXLEN=$(XLEN)
# F/D (EXP-030): FP=1 (default; rv32 and rv64) needs a librefcpu built with the same FP (RefCpuState carries the FP registers); FP=0 = no floating point
FP ?= 1
CXXFLAGS += -DFP=$(FP)
# C extension (EXP-038): RVC=1 needs a librefcpu built with RVC=1 (inst_len, halfword fetch); default 0
RVC ?= 0
CXXFLAGS += -DRVC=$(RVC)
ZLIB_CFLAGS := $(shell pkg-config --silence-errors --cflags zlib)
ZLIB_LIBDIR := $(shell pkg-config --silence-errors --variable=libdir zlib)
CXXFLAGS += $(ZLIB_CFLAGS)

# Libraries
REFCPU_LIB ?= ./function-model/librefcpu.a
LIBS := $(REFCPU_LIB) ./libs/softfloat.a
LDFLAGS := -lz -lstdc++fs
ifneq ($(ZLIB_LIBDIR),)
LDFLAGS := -L$(ZLIB_LIBDIR) $(LDFLAGS)
endif

# Debug Flags (Use 'make DEBUG=1' to enable)
DEBUG_ENABLED := 0
ifneq ($(filter 1 true TRUE yes YES on ON,$(DEBUG)),)
DEBUG_ENABLED := 1
endif

ifeq ($(DEBUG_ENABLED),1)
    CXXFLAGS += -g -O0
else
    CXXFLAGS += -DNDEBUG
endif

# Front-end source root
FRONT_DIR := ./front-end

INCLUDES := -I./include/ \
            -I./libs/include/ \
            -I./back-end/include/ \
            -I./back-end/Exu/include/ \
            -I./back-end/Lsu/include/ \
            -I./back-end/tools/include/ \
            -I./MemSubSystem/include/ \
            -I./diff/include/ \
            -I./function-model/include/api/ \
            -I./legacy/mmu/include/ \
            -I$(FRONT_DIR)/

AXI_KIT_IO_HDR := $(AXI_KIT_DIR)/axi_interconnect/include/AXI_Interconnect_IO.h
ifneq ($(wildcard $(AXI_KIT_IO_HDR)),)
INCLUDES += -I$(AXI_KIT_DIR)/include/ \
            -I$(AXI_KIT_DIR)/axi_interconnect/include/ \
            -I$(AXI_KIT_DIR)/sim_ddr/include/ \
            -I$(AXI_KIT_DIR)/mmio/include/
AXI_KIT_SRC := $(shell find $(AXI_KIT_DIR)/axi_interconnect -name "*.cpp" ! -name "*_test.cpp") \
               $(shell find $(AXI_KIT_DIR)/sim_ddr -name "*.cpp" ! -name "*_test.cpp") \
               $(shell find $(AXI_KIT_DIR)/mmio -name "*.cpp" ! -name "*_test.cpp")
endif

# Source Files
# (Using find to locate all cpp files)
CXXSRC := $(shell find ./back-end -name "*.cpp") \
          ./MemSubSystem/MemSubsystem.cpp \
          ./MemSubSystem/PtwWalker.cpp \
          ./MemSubSystem/PeripheralAxi.cpp \
          ./MemSubSystem/RealDcache.cpp \
          ./MemSubSystem/MSHR.cpp \
          ./MemSubSystem/WriteBuffer.cpp \
          ./MemSubSystem/DcacheConfig.cpp \
          ./MemSubSystem/MemRouteBlock.cpp\
          $(shell find $(FRONT_DIR) -name "*.cpp") \
          $(shell find ./diff -name "*.cpp" ! -name "ref.cpp") \
          $(AXI_KIT_SRC) \
          ./main.cpp \
          ./rv_simu_mmu_v2.cpp

ifdef LSU_SHADOW
CXXSRC += ./lsu_ref/RealLsuRef.cpp ./lsu_ref/LsuShadow.cpp
INCLUDES += -I./lsu_ref
CXXFLAGS += -DLSU_SHADOW
endif

ifdef DDR_SHADOW
CXXSRC += ./ddr_ref/SimDDRRef.cpp ./ddr_ref/DdrShadow.cpp
INCLUDES += -I./ddr_ref
CXXFLAGS += -DDR_SHADOW
endif

# DRAMsim3 DRAM timing backend for SimDDR (EXP-006), on by default; `gmake DS3=0` builds the fixed-latency SimDDR.
# DRAMsim3 is the pinned git submodule third_party/DRAMsim3 (`git submodule update --init third_party/DRAMsim3`); its
# library is compiled here into build_dramsim3/libdramsim3.a (also used by rtl/tb/soc/build.sh, `gmake dramsim3`).
# Core frequency: `gmake CPU_FREQ_MHZ=...` or the CPU_FREQ_MHZ environment variable at run time.
DS3 ?= 1
DS3_DIR ?= third_party/DRAMsim3
DS3_LIB ?= build_dramsim3/libdramsim3.a
DS3_SRC := bankstate channel_state command_queue common configuration controller dram_system hmc refresh simple_stats timing memory_system
DS3_OBJS := $(DS3_SRC:%=$(dir $(DS3_LIB))%.o)
ifdef CPU_FREQ_MHZ
CXXFLAGS += -DCONFIG_CPU_FREQ_MHZ=$(CPU_FREQ_MHZ)u
endif
ifneq ($(DS3),0)
ifeq ($(wildcard $(DS3_DIR)/src/dramsim3.h),)
$(error DRAMsim3 not found in $(DS3_DIR): run 'git submodule update --init third_party/DRAMsim3' (or build with DS3=0))
endif
INCLUDES += -I$(DS3_DIR)/src
CXXFLAGS += -DSIM_DDR_DS3 -DDS3_DEFAULT_CONFIG='"$(abspath $(DS3_DIR))/configs/DDR4_8Gb_x16_3200.ini"'
LIBS += $(DS3_LIB)
endif
$(dir $(DS3_LIB))%.o: $(DS3_DIR)/src/%.cc
	@mkdir -p $(dir $(DS3_LIB))
	@echo "Compiling DRAMsim3 $*.cc"
	@$(CXX) -std=c++14 -O2 -w -DFMT_HEADER_ONLY=1 -I$(DS3_DIR)/src -I$(DS3_DIR)/ext/headers -I$(DS3_DIR)/ext/fmt/include -c $< -o $@
$(DS3_LIB): $(DS3_OBJS)
	@ar rcs $@ $^
.PHONY: dramsim3
dramsim3: $(DS3_LIB)

ifdef ROB_SHADOW
CXXSRC += ./rob_ref/RobRef.cpp ./rob_ref/RobShadow.cpp
INCLUDES += -I./rob_ref
CXXFLAGS += -DROB_SHADOW
endif

ifdef DIS_SHADOW
CXXSRC += ./dis_ref/DispatchRef.cpp ./dis_ref/DisShadow.cpp
INCLUDES += -I./dis_ref
CXXFLAGS += -DDIS_SHADOW
endif

ifdef EXU_SHADOW
CXXSRC += ./tests/exu_ref/ExuRef.cpp ./tests/exu_ref/ExuShadow.cpp
INCLUDES += -I./tests/exu_ref
CXXFLAGS += -DEXU_SHADOW
endif

ifdef ISU_SHADOW
CXXSRC += ./isu_ref/IsuRef.cpp ./isu_ref/IsuShadow.cpp
INCLUDES += -I./isu_ref
CXXFLAGS += -DISU_SHADOW
endif

ifdef PRE_SHADOW
CXXSRC += ./pre_ref/PreIduQueueRef.cpp ./pre_ref/PreShadow.cpp
INCLUDES += -I./pre_ref
CXXFLAGS += -DPRE_SHADOW
endif

ifdef CSR_SHADOW
CXXSRC += ./csr_ref/CsrRef.cpp ./csr_ref/CsrShadow.cpp
INCLUDES += -I./csr_ref
CXXFLAGS += -DCSR_SHADOW
endif

ifdef REN_SHADOW
CXXSRC += ./ren_ref/RenRef.cpp ./ren_ref/RenShadow.cpp
INCLUDES += -I./ren_ref
CXXFLAGS += -DREN_SHADOW
endif

ifdef IDU_SHADOW
CXXSRC += ./idu_ref/IduRef.cpp ./idu_ref/IduShadow.cpp
INCLUDES += -I./idu_ref
CXXFLAGS += -DIDU_SHADOW
endif

# Object Files
OBJS := $(CXXSRC:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

# ==========================================
# Rules
# ==========================================

.PHONY: all clean run gdb coverage help gdb_linux linux

all: $(SIM_EXE)

# Link
$(SIM_EXE): $(OBJS) $(LIBS)
	@echo "Linking $@"
	@$(CXX) $(OBJS) $(LIBS) $(LDFLAGS) -o $@ $(CXXFLAGS)

$(REFCPU_LIB):
	$(error Missing $(REFCPU_LIB). Build the function-model first with 'make -C function-model librefcpu.a LDFLAGS=-lz' (function-model is a symlink to ../function-model)')

# Compile
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Run
run: $(SIM_EXE)
	./$(SIM_EXE) $(IMG)

linux: $(SIM_EXE)
	./$(SIM_EXE) ./baremetal/linux.bin  

# Debug with GDB
gdb: CXXFLAGS += -g -O0
gdb: $(SIM_EXE)
	gdb --args ./$(SIM_EXE) $(IMG)

gdb_linux: CXXFLAGS += -g
gdb_linux: $(SIM_EXE)
	gdb --args ./$(SIM_EXE) ./baremetal/linux.bin

# Coverage
coverage: CXXFLAGS += --coverage -O0
coverage: $(SIM_EXE)

# Clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)
	@rm -rf ./baremetal/memory ./baremetal/test.code
	@rm -f a.out

help:
	@echo "Usage:"
	@echo "  make          - Build the simulator (single configuration: include/config.h)"
	@echo "  make run      - Build and run the simulator"
	@echo "  make DEBUG=1  - Build with debug symbols"
	@echo "  make clean    - Clean build files"
	@echo "  make gdb_linux - Debug linux"

# Include dependencies
-include $(DEPS)

# ==========================================
# Optimized Simulator Makefile
# ==========================================

BUILD_DIR ?= build
.DEFAULT_GOAL := all
SIM_EXE   := $(BUILD_DIR)/simulator
AXI_KIT_DIR := ./axi-interconnect-kit

# Compiler & Flags
CXX      ?= g++
CXXFLAGS := -O3 -march=native -funroll-loops -mtune=native
CXXFLAGS += -MMD -MP 
CXXFLAGS += -Wall -Wextra -Wno-unused-parameter
CXXFLAGS += --std=c++2a
CXXFLAGS += $(EXTRA_CXXFLAGS)
# register width of the core and of the function model (default 64; rv32 only exists on branch rv32-legacy). Needs a librefcpu built with the same XLEN, RVC (and FP=1).
XLEN ?= 64
CXXFLAGS += -DXLEN=$(XLEN)
# F/D (EXP-030) is always built in: librefcpu must be built with FP=1 (its default; RefCpuState carries the FP registers).
# C extension (EXP-038): RVC=1 (default since 2026-11-19, = the RTL) needs a librefcpu built with RVC=1 (inst_len, halfword fetch);
# RVC=0 builds the old 4-byte front end (its cycles differ even on programs without C instructions)
RVC ?= 1
CXXFLAGS += -DRVC=$(RVC)
ZLIB_CFLAGS := $(shell pkg-config --silence-errors --cflags zlib)
ZLIB_LIBDIR := $(shell pkg-config --silence-errors --variable=libdir zlib)
CXXFLAGS += $(ZLIB_CFLAGS)

# Function model (librefcpu): built here, from the pinned revision FM_REV of the function-model repository FM_GIT, with this build's
# XLEN / RVC (FP=1), into a directory named by revision and configuration -- so the library always matches the simulator and no
# shared, possibly stale copy is used (2026-11-19). The checkout's branch does not matter (git archive of FM_REV). Bump FM_REV
# deliberately when the model needs a newer function model. REFCPU_LIB / REFCPU_INC can still be given explicitly (e.g. the static
# server builds), then they must match XLEN / RVC.
FM_GIT ?= ./function-model
FM_REV ?= 55affa0
FM_DIR := $(BUILD_DIR)/fm-$(FM_REV)-x$(XLEN)c$(RVC)
REFCPU_LIB ?= $(FM_DIR)/librefcpu.a
REFCPU_INC ?= $(FM_DIR)/include/api/
SOFTFLOAT_LIB ?= $(FM_DIR)/lib/softfloat/softfloat.a
# the function model's RVC expansion table (front-end/front_top.cpp), from the same tree as the API header
CXXFLAGS += -DREFCPU_RVC_H='"$(abspath $(REFCPU_INC))/../rvc.h"'
LIBS := $(REFCPU_LIB) $(SOFTFLOAT_LIB)
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
            -I./back-end/include/ \
            -I./back-end/Exu/include/ \
            -I./back-end/Lsu/include/ \
            -I./MemSubSystem/include/ \
            -I./bridge/include/ \
            -I./driver/include/ \
            -I$(REFCPU_INC) \
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
          ./MemSubSystem/PeripheralAxi.cpp \
          ./MemSubSystem/RealDcache.cpp \
          ./MemSubSystem/MSHR.cpp \
          ./MemSubSystem/WriteBuffer.cpp \
          ./MemSubSystem/DcacheConfig.cpp \
          ./MemSubSystem/MemRouteBlock.cpp \
          ./MemSubSystem/IoBacking.cpp \
          $(shell find $(FRONT_DIR) -name "*.cpp") \
          $(shell find ./bridge -name "*.cpp") \
          $(shell find ./driver -name "*.cpp") \
          $(AXI_KIT_SRC) \
          ./SimCpu.cpp \
          ./Csr.cpp \
          ./AxiSubsystem.cpp \
          ./Front2Back.cpp \
          ./Back2Front.cpp

# DRAMsim3 DRAM timing backend for SimDDR (EXP-006). DRAMsim3 is the pinned git submodule third_party/DRAMsim3
# (`git submodule update --init third_party/DRAMsim3`); its library is compiled here into build_dramsim3/libdramsim3.a (also used by
# rtl/tb/soc/build.sh, `gmake dramsim3`).  Core frequency: `gmake CPU_FREQ_MHZ=...`.
DS3_DIR ?= third_party/DRAMsim3
DS3_LIB ?= build_dramsim3/libdramsim3.a
DS3_SRC := bankstate channel_state command_queue common configuration controller dram_system hmc refresh simple_stats timing memory_system
DS3_OBJS := $(DS3_SRC:%=$(dir $(DS3_LIB))%.o)
ifdef CPU_FREQ_MHZ
CXXFLAGS += -DCONFIG_CPU_FREQ_MHZ=$(CPU_FREQ_MHZ)u
endif
ifeq ($(wildcard $(DS3_DIR)/src/dramsim3.h),)
$(error DRAMsim3 not found in $(DS3_DIR): run 'git submodule update --init third_party/DRAMsim3')
endif
INCLUDES += -I$(DS3_DIR)/src
CXXFLAGS += -DDS3_DEFAULT_CONFIG='"$(abspath $(DS3_DIR))/configs/DDR4_8Gb_x8_3200.ini"'
LIBS += $(DS3_LIB)
$(dir $(DS3_LIB))%.o: $(DS3_DIR)/src/%.cc
	@mkdir -p $(dir $(DS3_LIB))
	@echo "Compiling DRAMsim3 $*.cc"
	@$(CXX) -std=c++14 -O2 -w -DFMT_HEADER_ONLY=1 -I$(DS3_DIR)/src -I$(DS3_DIR)/ext/headers -I$(DS3_DIR)/ext/fmt/include -c $< -o $@
$(DS3_LIB): $(DS3_OBJS)
	@ar rcs $@ $^
.PHONY: dramsim3
dramsim3: $(DS3_LIB)

# Object Files
OBJS := $(CXXSRC:%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

# ==========================================
# Rules
# ==========================================

.PHONY: all clean help

all: $(SIM_EXE)

# Link
$(SIM_EXE): $(OBJS) $(LIBS)
	@echo "Linking $@"
	@$(CXX) $(OBJS) $(LIBS) $(LDFLAGS) -o $@ $(CXXFLAGS)

$(FM_DIR)/lib/softfloat/softfloat.a: $(FM_DIR)/librefcpu.a

$(FM_DIR)/librefcpu.a:
	@echo "Building the function model $(FM_REV) (XLEN=$(XLEN) RVC=$(RVC) FP=1) in $(FM_DIR)"
	@mkdir -p $(FM_DIR)
	@git -C $(FM_GIT) archive $(FM_REV) | tar -x -C $(FM_DIR)
	@$(MAKE) -s -C $(FM_DIR) librefcpu.a XLEN=$(XLEN) RVC=$(RVC) FP=1 LDFLAGS=-lz > $(FM_DIR)/build.log 2>&1 || { tail -20 $(FM_DIR)/build.log; exit 1; }

# Compile
$(BUILD_DIR)/%.o: %.cpp | $(REFCPU_LIB)
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)

help:
	@echo "Usage:"
	@echo "  make          - Build the simulator (single configuration: include/config.h)"
	@echo "  make DEBUG=1  - Build with debug symbols"
	@echo "  make clean    - Clean build files"
	@echo "  The function model is built automatically (FM_REV=$(FM_REV), same XLEN / RVC) into $(FM_DIR)"

# Include dependencies
-include $(DEPS)

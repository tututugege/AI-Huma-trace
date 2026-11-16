#ifndef PREDECODE_H
#define PREDECODE_H

#include "front_IO.h"
#include <cstdint>
#include <RISCV.h>

static constexpr predecode_type_t PREDECODE_NON_BRANCH = 0;
static constexpr predecode_type_t PREDECODE_DIRECT_JUMP_NO_JAL = 1;
static constexpr predecode_type_t PREDECODE_JALR = 2;
static constexpr predecode_type_t PREDECODE_JAL = 3;

struct PredecodeResult {
  predecode_type_t type;
  target_addr_t target_address;
};

// The predecode of one instruction word at pc (combinational).
PredecodeResult predecode(inst_word_t inst, pc_t pc);

#endif

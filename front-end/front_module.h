#ifndef FRONT_MODULE_H
#define FRONT_MODULE_H

#include "front_IO.h"
#include <cstdint>

struct fetch_address_FIFO_read_data {
  fetch_addr_fifo_size_t size;
  wire1_t head_valid;
  fetch_addr_t head_entry;
  FetchPair head_pair;
  uint8_t head_age;  // BPU_PIPE: cycles since the head was pushed (saturates at 3)
};

struct instruction_FIFO_entry {
  inst_word_t instructions[FETCH_WIDTH];
  pc_t pc[FETCH_WIDTH];
  wire1_t page_fault_inst[FETCH_WIDTH];
  wire1_t inst_valid[FETCH_WIDTH];
  predecode_type_t predecode_type[FETCH_WIDTH];
  target_addr_t predecode_target_address[FETCH_WIDTH];
  pc_t seq_next_pc;
  wire1_t is_rvc[FETCH_WIDTH];
  wire1_t start_a[FETCH_WIDTH];
  wire1_t start_b[FETCH_WIDTH];
  uint16_t raw16[FETCH_WIDTH] = {};   // RVC=1: the first halfword of the window at each slot, as fetched (before expansion)
  uint16_t up16 = 0;                  // RVC=1: the upper halfword of the window of the group's last slot (before the carry mask) ...
  wire1_t up_ok = 0;                  // ... and whether it is in the same cache line as the slot (the fetch can deliver it with the group)
};

struct instruction_FIFO_read_data {
  instruction_fifo_size_t size;
  wire1_t head_valid;
  instruction_FIFO_entry head_entry;
};

struct PTAB_entry {
  wire1_t predict_dir[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  pc_t predict_base_pc[FETCH_WIDTH];
  PredMeta meta;
};

struct PTAB_read_data {
  ptab_size_t size;
  wire1_t head_valid;
  PTAB_entry head_entry;
};

struct front2back_FIFO_entry {
  inst_word_t fetch_group[FETCH_WIDTH];
  wire1_t page_fault_inst[FETCH_WIDTH];
  wire1_t inst_valid[FETCH_WIDTH];
  wire1_t is_rvc[FETCH_WIDTH];
  wire1_t predict_dir_corrected[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address_corrected;
  pc_t predict_base_pc[FETCH_WIDTH];
  PredMeta meta;
};

struct front2back_FIFO_read_data {
  front2back_fifo_size_t size;
  wire1_t head_valid;
  front2back_FIFO_entry head_entry;
};

struct FetchAddrCombOut;
struct InstructionCombOut;
struct PtabCombOut;
struct Front2BackCombOut;
struct FrontReadData;
struct FrontUpdateRequest;

#endif // FRONT_MODULE_H

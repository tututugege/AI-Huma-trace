#ifndef FRONT_MODULE_H
#define FRONT_MODULE_H

#include "front_IO.h"
#include <cstdint>

struct FetchAddrFifoReadData {
  fetch_addr_fifo_size_t size;
  wire<1> head_valid;
  fetch_addr_t head_entry;
  FetchPair head_pair;
  uint8_t head_age;  // BPU_PIPE: cycles since the head was pushed (saturates at 3)
};


struct InstFifoReadData {
  instruction_fifo_size_t size;
  wire<1> head_valid;
  InstFifoEntry head_entry;
};


struct PtabReadData {
  ptab_size_t size;
  wire<1> head_valid;
  PtabEntry head_entry;
};


struct F2bFifoReadData {
  front2back_fifo_size_t size;
  wire<1> head_valid;
  F2bFifoEntry head_entry;
};

struct FetchAddrCombOut;
struct InstructionCombOut;
struct PtabCombOut;
struct Front2BackCombOut;
struct FrontReadData;
struct FrontUpdateRequest;

#endif // FRONT_MODULE_H

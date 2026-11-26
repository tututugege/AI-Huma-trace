#ifndef FRONT_MODULE_H
#define FRONT_MODULE_H

#include "front_IO.h"
#include <cstdint>

struct InstFifoReadData {
  instruction_fifo_size_t size;
  wire<1> head_valid;
  const InstFifoEntry *head;   // in the FIFO's storage, or the entry written this cycle
};

struct FrontReadData;

#endif // FRONT_MODULE_H

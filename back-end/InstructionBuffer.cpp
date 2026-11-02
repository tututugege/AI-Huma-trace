#include "InstructionBuffer.h"
#include "util.h"
#include <cstring>

void InstructionBuffer::init() {
  clear();
  all_touched = false; // committed and working copies start identical
}

const InstructionBufferEntry &InstructionBuffer::peek(int offset) const {
  Assert(offset >= 0 && offset < count_r &&
         "InstructionBuffer::peek offset out of range");
  int idx = (head_r + offset) % IDU_INST_BUFFER_SIZE;
  return entries[idx];
}

void InstructionBuffer::pop_front(int n) {
  if (n <= 0) {
    return;
  }
  int actual = (n > count_r) ? count_r : n;
  for (int i = 0; i < actual; i++) {
    entries[head_r] = {};
    touched[head_r >> 6] |= 1ull << (head_r & 63);
    head_r = (head_r + 1) % IDU_INST_BUFFER_SIZE;
  }
  count_r -= actual;
}

void InstructionBuffer::push_back(const InstructionBufferEntry &entry) {
  Assert(count_r < IDU_INST_BUFFER_SIZE && "InstructionBuffer overflow");
  entries[tail_r] = entry;
  touched[tail_r >> 6] |= 1ull << (tail_r & 63);
  tail_r = (tail_r + 1) % IDU_INST_BUFFER_SIZE;
  count_r++;
}

void InstructionBuffer::clear() {
  head_r = 0;
  tail_r = 0;
  count_r = 0;
  for (int i = 0; i < IDU_INST_BUFFER_SIZE; i++) {
    entries[i] = {};
  }
  all_touched = true;
}

void InstructionBuffer::sync_from(InstructionBuffer &w) {
  head_r = w.head_r;
  tail_r = w.tail_r;
  count_r = w.count_r;
  if (w.all_touched) {
    std::memcpy(entries, w.entries, sizeof(entries));
  } else {
    for (size_t k = 0; k < sizeof(w.touched) / sizeof(w.touched[0]); ++k) {
      uint64_t m = w.touched[k];
      while (m) {
        const int i = static_cast<int>(k) * 64 + __builtin_ctzll(m);
        entries[i] = w.entries[i];
        m &= m - 1;
      }
    }
  }
  std::memset(w.touched, 0, sizeof(w.touched));
  w.all_touched = false;
}

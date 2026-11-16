// MemUtils.h
#pragma once

#include <cstdint>

#include "../../../include/config.h"
#include "mmio_address_map.h"

struct StoreTag {
  reg<STQ_IDX_WIDTH> idx = 0;
  reg<1> flag = false;
};

namespace {

[[maybe_unused]] static int get_mem_width(int func3) {
  switch (func3 & 0b11) {
  case 0b00:
    return 1;
  case 0b01:
    return 2;
  case 0b11:
    return 8;   // ld / sd (rv64) and fld / fsd (F/D, rv32 too; funct3 3 is no integer load on rv32)
  case 0b10:
  default:
    return 4;
  }
}


[[maybe_unused]] static inline uint8_t get_store_strb(uint32_t addr, uint8_t func3) {
  uint32_t off = addr & 0x3;
  switch (func3 & 0x3) {
  case 0:
    return 0x1u << off; // SB
  case 1:
    return 0x3u << off; // SH: off is 0 or 2
  case 2:
    return 0xFu; // SW: off is 0
  case 3:
    return 0xFFu;   // SD (rv64) / FSD: 8 bytes, 8-byte aligned (CheckAddr compares up to 8 bytes)
  default:
    return 0;
  }
}


constexpr uint32_t kLsuReqIdGenBits = 31 - LDQ_IDX_WIDTH;
constexpr uint32_t kLsuReqIdIdxMask = (1u << LDQ_IDX_WIDTH) - 1;
constexpr uint32_t kLsuReqIdGenMask = (1u << kLsuReqIdGenBits) - 1;

[[maybe_unused]] static uint32_t normalize_lsu_req_gen(uint32_t gen) {
  return gen & kLsuReqIdGenMask;
}

template <typename PtrT, typename FlagT>
[[maybe_unused]] static void advance_ring_ptr(PtrT &ptr, FlagT &flag, uint32_t size) {
  const uint32_t next = static_cast<uint32_t>(ptr) + 1;
  if (next >= size) {
    ptr = 0;
    flag = !flag;
  } else {
    ptr = next;
  }
}

template <typename PtrT>
[[maybe_unused]] static void advance_ring_ptr(PtrT &ptr, uint32_t size) {
  const uint32_t next = static_cast<uint32_t>(ptr) + 1;
  ptr = next >= size ? 0 : next;
}


[[maybe_unused]] static bool stq_idx_alive_after_flush(uint32_t idx, uint32_t head,
                                      uint32_t new_count) {
  return ((idx + STQ_SIZE - head) % STQ_SIZE) < new_count;
}

[[maybe_unused]] static bool stq_tail_flag(uint32_t head, uint32_t count, bool head_flag) {
  return (head + count) >= STQ_SIZE ? !head_flag : head_flag;
}

[[maybe_unused]] static uint32_t stq_idx_after(uint32_t head, uint32_t count) {
  return (head + count) % STQ_SIZE;
}

[[maybe_unused]] static bool lsu_is_mmio_addr(uint32_t paddr) {
  return simulator_is_mmio_addr(paddr);
}

[[maybe_unused]] static bool lsu_mmio_is_oldest_unfinished(const RobBroadcastIO *rob_bcast,
                                          uint32_t rob_idx) {
  if (rob_bcast == nullptr) {
    return false;
  }
  if (rob_bcast->head_incomplete_valid) {
    return rob_bcast->head_incomplete_rob_idx == rob_idx;
  }
  return rob_bcast->head_valid && rob_bcast->head_rob_idx == rob_idx;
}

[[maybe_unused]] static uint32_t stq_tag_value(uint32_t idx, bool flag) {
  return (flag ? STQ_SIZE : 0) + idx;
}

} // namespace

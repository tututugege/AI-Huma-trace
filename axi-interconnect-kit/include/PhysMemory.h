#pragma once

#if !defined(AXI_KIT_STANDALONE_PHYSMEM) && defined(__has_include_next)
#if __has_include_next("PhysMemory.h")
#include_next "PhysMemory.h"
#define AXI_KIT_PHYSMEM_USING_PARENT 1
#endif
#endif

#ifndef AXI_KIT_PHYSMEM_USING_PARENT

#include <cstddef>
#include <cstdint>
#include <cstring>

constexpr uint32_t PMEM_RAM_BASE = 0x80000000u;

extern uint32_t *p_memory;

inline uint32_t axi_kit_pmem_offset(uint32_t paddr) {
  return paddr >= PMEM_RAM_BASE ? paddr - PMEM_RAM_BASE : paddr;
}

inline bool pmem_init() { return p_memory != nullptr; }
inline void pmem_release() {}
inline void pmem_clear_all() {}

inline bool pmem_is_ram_addr(uint32_t, uint32_t = 4u) {
  return p_memory != nullptr;
}

inline uint32_t pmem_read(uint32_t paddr) {
  if (p_memory == nullptr) {
    return 0;
  }
  return p_memory[(axi_kit_pmem_offset(paddr) & ~0x3u) >> 2];
}

inline void pmem_write(uint32_t paddr, uint32_t data) {
  if (p_memory != nullptr) {
    p_memory[(axi_kit_pmem_offset(paddr) & ~0x3u) >> 2] = data;
  }
}

inline void pmem_memcpy_to_ram(uint32_t ram_paddr, const void *src,
                               size_t len) {
  if (p_memory != nullptr && len != 0) {
    std::memcpy(reinterpret_cast<uint8_t *>(p_memory) +
                    axi_kit_pmem_offset(ram_paddr),
                src, len);
  }
}

inline void pmem_memcpy_from_ram(void *dst, uint32_t ram_paddr, size_t len) {
  if (p_memory != nullptr && len != 0) {
    std::memcpy(dst,
                reinterpret_cast<const uint8_t *>(p_memory) +
                    axi_kit_pmem_offset(ram_paddr),
                len);
  }
}

inline uint32_t *pmem_ram_ptr() { return p_memory; }

#endif

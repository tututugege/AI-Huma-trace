#include "trap.h"
#include <stdint.h>

static volatile uint8_t region[8192] __attribute__((aligned(4096)));

static inline void store_u32(void *addr, uint32_t value) {
  asm volatile("sw %1, 0(%0)" : : "r"(addr), "r"(value) : "memory");
}

static inline uint32_t load_u32(const void *addr) {
  uint32_t value;
  asm volatile("lw %0, 0(%1)" : "=r"(value) : "r"(addr) : "memory");
  return value;
}

static inline void store_u16(void *addr, uint32_t value) {
  asm volatile("sh %1, 0(%0)" : : "r"(addr), "r"(value) : "memory");
}

static inline uint32_t load_u16(const void *addr) {
  uint32_t value;
  asm volatile("lhu %0, 0(%1)" : "=r"(value) : "r"(addr) : "memory");
  return value;
}

static void check_bytes(uint32_t offset, uint32_t value) {
  check(region[offset + 0] == (uint8_t)(value >> 0));
  check(region[offset + 1] == (uint8_t)(value >> 8));
  check(region[offset + 2] == (uint8_t)(value >> 16));
  check(region[offset + 3] == (uint8_t)(value >> 24));
}

int main(void) {
  /* Last and first aligned words on adjacent 64-byte D-cache lines. */
  store_u32((void *)&region[60], 0x11223344u);
  store_u32((void *)&region[64], 0x55667788u);
  check(load_u32((const void *)&region[60]) == 0x11223344u);
  check(load_u32((const void *)&region[64]) == 0x55667788u);
  check_bytes(60, 0x11223344u);
  check_bytes(64, 0x55667788u);

  /* Last and first aligned words on adjacent 4-KiB physical pages. */
  store_u32((void *)&region[4092], 0x89abcdefu);
  store_u32((void *)&region[4096], 0x01234567u);
  check(load_u32((const void *)&region[4092]) == 0x89abcdefu);
  check(load_u32((const void *)&region[4096]) == 0x01234567u);

  /* Partial write at the end of the page must not corrupt the next page. */
  store_u16((void *)&region[4094], 0x55aau);
  check(load_u16((const void *)&region[4094]) == 0x55aau);
  check(load_u32((const void *)&region[4092]) == 0x55aacdefu);
  check(load_u32((const void *)&region[4096]) == 0x01234567u);

  return 0;
}

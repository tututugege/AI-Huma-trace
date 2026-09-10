#include "trap.h"
#include <stdint.h>

static volatile uint32_t word __attribute__((aligned(4)));

static inline uint32_t load_word(void) {
  uint32_t value;
  asm volatile("lw %0, 0(%1)" : "=r"(value) : "r"(&word) : "memory");
  return value;
}

int main(void) {
  uint32_t loaded;

  word = 0x11223344u;
  asm volatile("sb %1, 1(%2)\n\tlw %0, 0(%2)"
               : "=r"(loaded)
               : "r"(0xaau), "r"(&word)
               : "memory");
  check(loaded == 0x1122aa44u);

  asm volatile("sh %1, 2(%2)\n\tlw %0, 0(%2)"
               : "=r"(loaded)
               : "r"(0xbbccu), "r"(&word)
               : "memory");
  check(loaded == 0xbbccaa44u);

  /* The youngest of two overlapping stores must supply the loaded byte. */
  asm volatile("sb %1, 0(%3)\n\tsb %2, 0(%3)\n\tlw %0, 0(%3)"
               : "=r"(loaded)
               : "r"(0x11u), "r"(0x22u), "r"(&word)
               : "memory");
  check(loaded == 0xbbccaa22u);

  /* Store address is ready while DIVU keeps its data unavailable. */
  uint32_t delayed;
  asm volatile(".insn r 0x33, 5, 1, %1, %3, %4\n\t"
               "sw %1, 0(%2)\n\t"
               "lw %0, 0(%2)"
               : "=&r"(loaded), "=&r"(delayed)
               : "r"(&word), "r"(300u), "r"(7u)
               : "memory");
  check(delayed == 42u);
  check(loaded == 42u);
  check(load_word() == 42u);

  return 0;
}

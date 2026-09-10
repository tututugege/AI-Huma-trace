#include "trap.h"
#include <stdint.h>

static inline uint32_t fsqrt_s_rne(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 0, 0x2c, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fsqrt_s_rtz(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 1, 0x2c, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fsqrt_s_rup(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 3, 0x2c, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

int main(void) {
  /* Exact roots and ordinary inputs. */
  check(fsqrt_s_rne(0x40800000u) == 0x40000000u); /* sqrt(4) = 2 */
  check(fsqrt_s_rne(0x3e800000u) == 0x3f000000u); /* sqrt(0.25) = 0.5 */
  check(fsqrt_s_rne(0x41100000u) == 0x40400000u); /* sqrt(9) = 3 */

  /* Irrational result and rounding-mode selection. */
  check(fsqrt_s_rne(0x40000000u) == 0x3fb504f3u); /* sqrt(2), nearest */
  check(fsqrt_s_rtz(0x40000000u) == 0x3fb504f3u);
  check(fsqrt_s_rup(0x40000000u) == 0x3fb504f4u);

  /* IEEE-754 special values. */
  check(fsqrt_s_rne(0x00000000u) == 0x00000000u); /* +0 */
  check(fsqrt_s_rne(0x80000000u) == 0x80000000u); /* -0 */
  check(fsqrt_s_rne(0x7f800000u) == 0x7f800000u); /* +inf */
  check(fsqrt_s_rne(0xbf800000u) == 0x7fc00000u); /* sqrt(-1) */

  /* A dependent chain covers result wakeup and repeated long-latency issue. */
  uint32_t value = 0x43800000u; /* 256 */
  value = fsqrt_s_rne(value);   /* 16 */
  value = fsqrt_s_rne(value);   /* 4 */
  value = fsqrt_s_rne(value);   /* 2 */
  check(value == 0x40000000u);

  return 0;
}

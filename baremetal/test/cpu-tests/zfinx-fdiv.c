#include "trap.h"
#include <stdint.h>

/* Zfinx keeps single-precision operands and results in integer registers. */
static inline uint32_t fdiv_s_rne(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile(".insn r 0x53, 0, 0x0c, %0, %1, %2"
               : "=r"(result)
               : "r"(a), "r"(b));
  return result;
}

static inline uint32_t fdiv_s_rtz(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile(".insn r 0x53, 1, 0x0c, %0, %1, %2"
               : "=r"(result)
               : "r"(a), "r"(b));
  return result;
}

static inline uint32_t fdiv_s_rdn(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile(".insn r 0x53, 2, 0x0c, %0, %1, %2"
               : "=r"(result)
               : "r"(a), "r"(b));
  return result;
}

static inline uint32_t fdiv_s_rup(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile(".insn r 0x53, 3, 0x0c, %0, %1, %2"
               : "=r"(result)
               : "r"(a), "r"(b));
  return result;
}

int main(void) {
  /* Exact finite results and signs. */
  check(fdiv_s_rne(0x40c00000u, 0x40000000u) == 0x40400000u); /* 6 / 2 = 3 */
  check(fdiv_s_rne(0xc0e00000u, 0x40000000u) == 0xc0600000u); /* -7 / 2 = -3.5 */
  check(fdiv_s_rne(0x3f800000u, 0x40800000u) == 0x3e800000u); /* 1 / 4 = 0.25 */

  /* Normal-to-subnormal transition. */
  check(fdiv_s_rne(0x00800000u, 0x40000000u) == 0x00400000u);

  /* IEEE-754 special values. */
  check(fdiv_s_rne(0x3f800000u, 0x00000000u) == 0x7f800000u);
  check(fdiv_s_rne(0xbf800000u, 0x00000000u) == 0xff800000u);
  check(fdiv_s_rne(0x00000000u, 0x00000000u) == 0x7fc00000u);
  check(fdiv_s_rne(0x7f800000u, 0x7f800000u) == 0x7fc00000u);

  /* 1 / 10 distinguishes directed rounding modes. */
  check(fdiv_s_rne(0x3f800000u, 0x41200000u) == 0x3dcccccdu);
  check(fdiv_s_rtz(0x3f800000u, 0x41200000u) == 0x3dccccccu);
  check(fdiv_s_rdn(0x3f800000u, 0x41200000u) == 0x3dccccccu);
  check(fdiv_s_rup(0x3f800000u, 0x41200000u) == 0x3dcccccdu);

  /* A dependent chain exercises back-to-back use of the iterative FPU. */
  uint32_t value = 0x43800000u; /* 256 */
  value = fdiv_s_rne(value, 0x40000000u);
  value = fdiv_s_rne(value, 0x40000000u);
  value = fdiv_s_rne(value, 0x40000000u);
  check(value == 0x42000000u); /* 32 */

  return 0;
}

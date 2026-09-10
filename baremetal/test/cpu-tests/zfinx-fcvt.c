#include "trap.h"
#include <stdint.h>

static inline uint32_t fcvt_w_s_rne(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 0, 0x60, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_w_s_rtz(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 1, 0x60, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_w_s_rdn(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 2, 0x60, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_w_s_rup(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 3, 0x60, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_w_s_rmm(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 4, 0x60, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_wu_s_rtz(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 1, 0x60, %0, %1, x1"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_s_w_rne(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 0, 0x68, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_s_w_rup(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 3, 0x68, %0, %1, x0"
               : "=r"(result)
               : "r"(a));
  return result;
}

static inline uint32_t fcvt_s_wu_rne(uint32_t a) {
  uint32_t result;
  asm volatile(".insn r 0x53, 0, 0x68, %0, %1, x1"
               : "=r"(result)
               : "r"(a));
  return result;
}

int main(void) {
  /* Float-to-signed-int: all implemented static rounding modes. */
  check(fcvt_w_s_rne(0x40700000u) == 4u);  /* 3.75 -> 4 */
  check(fcvt_w_s_rtz(0x40700000u) == 3u);  /* 3.75 -> 3 */
  check(fcvt_w_s_rdn(0x40700000u) == 3u);
  check(fcvt_w_s_rup(0x40700000u) == 4u);
  check(fcvt_w_s_rmm(0x40200000u) == 3u);  /* 2.5, ties away */
  check(fcvt_w_s_rne(0x40200000u) == 2u);  /* 2.5, ties to even */

  check(fcvt_w_s_rne(0xc0700000u) == (uint32_t)-4);
  check(fcvt_w_s_rtz(0xc0700000u) == (uint32_t)-3);
  check(fcvt_w_s_rdn(0xc0700000u) == (uint32_t)-4);
  check(fcvt_w_s_rup(0xc0700000u) == (uint32_t)-3);

  /* Float-to-unsigned-int selector (rs2=1), including negative saturation. */
  check(fcvt_wu_s_rtz(0x42f78000u) == 123u); /* 123.75 -> 123 */
  check(fcvt_wu_s_rtz(0xbf800000u) == 0u);   /* -1 -> 0 */

  /* Signed and unsigned integer-to-float selectors. */
  check(fcvt_s_w_rne(42u) == 0x42280000u);
  check(fcvt_s_w_rne((uint32_t)-42) == 0xc2280000u);
  check(fcvt_s_w_rne(0x80000000u) == 0xcf000000u);
  check(fcvt_s_wu_rne(0xffffffffu) == 0x4f800000u);

  /* 2^24+1 is the first positive int32 value not exactly representable. */
  check(fcvt_s_w_rne(0x01000001u) == 0x4b800000u);
  check(fcvt_s_w_rup(0x01000001u) == 0x4b800001u);

  /* Dependent round trips exercise FCVT result forwarding in both directions. */
  uint32_t value = fcvt_s_w_rne((uint32_t)-12345);
  value = fcvt_w_s_rtz(value);
  check(value == (uint32_t)-12345);

  return 0;
}

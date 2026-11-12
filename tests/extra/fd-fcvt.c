// F/D (rv32imafd, ilp32d) version of the former Zfinx test zfinx-fcvt.c: the operands stay uint32_t bit patterns, each operation moves them through the FP registers.
// The entry wrapper (fd-fcvt-helper.S) turns mstatus.FS on and calls fd_main.
#include "trap.h"
#include <stdint.h>

static inline uint32_t fcvt_w_s_rne(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfcvt.w.s %0, ft0, rne"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_w_s_rtz(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfcvt.w.s %0, ft0, rtz"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_w_s_rdn(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfcvt.w.s %0, ft0, rdn"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_w_s_rup(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfcvt.w.s %0, ft0, rup"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_w_s_rmm(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfcvt.w.s %0, ft0, rmm"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_wu_s_rtz(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfcvt.wu.s %0, ft0, rtz"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_s_w_rne(uint32_t a) {
  uint32_t result;
  asm volatile("fcvt.s.w ft0, %1, rne\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_s_w_rup(uint32_t a) {
  uint32_t result;
  asm volatile("fcvt.s.w ft0, %1, rup\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fcvt_s_wu_rne(uint32_t a) {
  uint32_t result;
  asm volatile("fcvt.s.wu ft0, %1, rne\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

int fd_main(void) {
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

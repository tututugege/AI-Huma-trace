// F/D (rv32imafd, ilp32d) version of the former Zfinx test zfinx-fsqrt.c: the operands stay uint32_t bit patterns, each operation moves them through the FP registers.
// The entry wrapper (fd-fsqrt-helper.S) turns mstatus.FS on and calls fd_main.
#include "trap.h"
#include <stdint.h>

static inline uint32_t fsqrt_s_rne(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfsqrt.s ft0, ft0, rne\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fsqrt_s_rtz(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfsqrt.s ft0, ft0, rtz\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

static inline uint32_t fsqrt_s_rup(uint32_t a) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfsqrt.s ft0, ft0, rup\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a)
               : "ft0");
  return result;
}

int fd_main(void) {
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

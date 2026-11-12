// F/D (rv32imafd, ilp32d) version of the former Zfinx test zfinx-fdiv.c: the operands stay uint32_t bit patterns, each operation moves them through the FP registers.
// The entry wrapper (fd-fdiv-helper.S) turns mstatus.FS on and calls fd_main.
#include "trap.h"
#include <stdint.h>

/* Zfinx keeps single-precision operands and results in integer registers. */
static inline uint32_t fdiv_s_rne(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfmv.w.x ft1, %2\n\tfdiv.s ft0, ft0, ft1, rne\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a), "r"(b)
               : "ft0", "ft1");
  return result;
}

static inline uint32_t fdiv_s_rtz(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfmv.w.x ft1, %2\n\tfdiv.s ft0, ft0, ft1, rtz\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a), "r"(b)
               : "ft0", "ft1");
  return result;
}

static inline uint32_t fdiv_s_rdn(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfmv.w.x ft1, %2\n\tfdiv.s ft0, ft0, ft1, rdn\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a), "r"(b)
               : "ft0", "ft1");
  return result;
}

static inline uint32_t fdiv_s_rup(uint32_t a, uint32_t b) {
  uint32_t result;
  asm volatile("fmv.w.x ft0, %1\n\tfmv.w.x ft1, %2\n\tfdiv.s ft0, ft0, ft1, rup\n\tfmv.x.w %0, ft0"
               : "=r"(result)
               : "r"(a), "r"(b)
               : "ft0", "ft1");
  return result;
}

int fd_main(void) {
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

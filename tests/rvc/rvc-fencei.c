#include <stdint.h>

static int failures;
#define check(c) do { if (!(c)) failures++; } while (0)
static void report(void) {   /* the simulator does not look at the result of main: print it */
  volatile uint8_t *uart = (volatile uint8_t *)0x10000000u;
  const char *m = failures ? "RVC_TEST_FAIL\n" : "RVC_TEST_PASS\n";
  while (*m) *uart = (uint8_t)*m++;
}

/* fence.i with compressed code: code is rewritten halfword by halfword at every 2-byte offset across a cache-line boundary (32 halfwords per
 * 64-byte line), as 32-bit and as 16-bit instructions; each version runs once (it is in the I-cache) before the next rewrite + fence.i. */
extern uint32_t execute_generated_code(const volatile uint16_t *code);
extern void fence_i_sync(void);

static volatile uint16_t buf[160] __attribute__((aligned(64)));

static void put32(volatile uint16_t *p, uint32_t w) {
  p[0] = (uint16_t)(w & 0xffffu);
  p[1] = (uint16_t)(w >> 16);
}

int main(void) {
  for (int pass = 0; pass < 3; pass++) {
    for (int idx = 0; idx < 100; idx++) {
      volatile uint16_t *p = buf + idx;
      const uint32_t k = (uint32_t)(idx * 7 + pass * 3 + 11) & 0x7ffu;
      /* A: addi a0, zero, k (32-bit); c.jr ra */
      put32(p, 0x00000513u | (k << 20));
      p[2] = 0x8082u;
      fence_i_sync();
      check(execute_generated_code(p) == k);
      /* B: c.li a0, 5; c.jr ra (16-bit over the 32-bit one) */
      p[0] = 0x4515u;
      p[1] = 0x8082u;
      fence_i_sync();
      check(execute_generated_code(p) == 5u);
      /* C: addi a0, zero, k+1; ret (jalr, 32-bit) */
      put32(p, 0x00000513u | ((k + 1u) << 20));
      put32(p + 2, 0x00008067u);
      fence_i_sync();
      check(execute_generated_code(p) == k + 1u);
    }
  }
  report();
  return failures;
}

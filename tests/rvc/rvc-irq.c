#include <stdint.h>

static int failures;
#define check(c) do { if (!(c)) failures++; } while (0)
static void report(void) {   /* the simulator does not look at the result of main: print it */
  volatile uint8_t *uart = (volatile uint8_t *)0x10000000u;
  const char *m = failures ? "RVC_TEST_FAIL\n" : "RVC_TEST_PASS\n";
  while (*m) *uart = (uint8_t)*m++;
}

/* A machine software interrupt (mip.MSIP written by the program) taken at different points of a stream of 32-bit instructions that all start at
 * 2 mod 4 (every one straddles a 4-byte boundary).  Each of the 16 segments raises the interrupt after a different number of instructions; the
 * handler clears it and returns to mepc.  No instruction may be lost or executed twice: the sum must come out exactly. */
extern uint32_t run_irq_segments(void);
volatile uint32_t irq_count;
volatile uint32_t irq_mepc[64];

int main(void) {
  irq_count = 0;
  const uint32_t sum = run_irq_segments();
  check(irq_count == 16u);
  check(sum == 760u);   /* sum over pre = 0..15 of (pre + 40) */
  for (int i = 0; i < 16; i++) check(irq_mepc[i] != 0u && (irq_mepc[i] & 1u) == 0u);
  report();
  return failures;
}

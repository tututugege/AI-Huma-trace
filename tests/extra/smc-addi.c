#include "trap.h"
#include <stdint.h>

/* Self-modifying code whose timing depends on the instruction content:
   the generated word alternates between "addi a0,zero,1" (1-cycle ALU) and "div a0,a0,a0" (divider),
   so a stale instruction fetch changes the cycle count. */
extern uint32_t execute_generated_code(const uint32_t *code);
extern void fence_i_sync(void);

static volatile uint32_t generated_code[2] __attribute__((aligned(64))) = {
    0x00100513u,
    0x00008067u,
};

int main(void) {
  for (int i = 0; i < 48; i++) {
    generated_code[0] = 0x00100513u; /* addi a0, zero, 1 */
    fence_i_sync();
    execute_generated_code((const uint32_t *)generated_code);
    generated_code[0] = 0x00100513u; /* control: same as addi */
    fence_i_sync();
    execute_generated_code((const uint32_t *)generated_code);
  }
  return 0;
}

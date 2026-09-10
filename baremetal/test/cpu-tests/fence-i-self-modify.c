#include "trap.h"
#include <stdint.h>

extern uint32_t execute_generated_code(const uint32_t *code);
extern void fence_i_sync(void);

/* Physical baremetal memory is executable; each pair is "li a0, imm; ret". */
static volatile uint32_t generated_code[2] __attribute__((aligned(64))) = {
    0x00100513u,
    0x00008067u,
};

static void replace_return_value(uint32_t value) {
  /* addi a0, zero, value; values used here fit the signed 12-bit immediate. */
  generated_code[0] = ((value & 0xfffu) << 20) | 0x00000513u;
  asm volatile("" ::: "memory");
}

int main(void) {
  check(execute_generated_code((const uint32_t *)generated_code) == 1u);

  replace_return_value(2u);
  fence_i_sync();
  check(execute_generated_code((const uint32_t *)generated_code) == 2u);

  replace_return_value(0x7ffu);
  fence_i_sync();
  check(execute_generated_code((const uint32_t *)generated_code) == 0x7ffu);
  return 0;
}

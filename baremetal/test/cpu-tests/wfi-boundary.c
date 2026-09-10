#include "trap.h"
#include <stdint.h>

static inline void execute_wfi(void) {
  asm volatile(".word 0x10500073" ::: "memory");
}

int main(void) {
  volatile uint32_t signature = 0x13579bdfu;

  /* The current bring-up model permits WFI to resume spuriously. */
  for (uint32_t i = 0; i < 4; i++) {
    signature ^= (0x11111111u + i);
    execute_wfi();
    signature += 0x01020304u;
  }

  check(signature == 0x135f83d3u);
  return 0;
}

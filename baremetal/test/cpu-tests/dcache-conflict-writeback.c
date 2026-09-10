#include "trap.h"
#include <stdint.h>

#define LINE_SIZE 64u
#define SET_STRIDE (256u * LINE_SIZE)
#define CONFLICT_LINES 8u
#define TEST_SETS 32u

static volatile uint8_t data[CONFLICT_LINES * SET_STRIDE]
    __attribute__((aligned(LINE_SIZE)));

static inline volatile uint32_t *test_word(uint32_t set, uint32_t way) {
  return (volatile uint32_t *)&data[way * SET_STRIDE + set * LINE_SIZE];
}

int main(void) {
  /* Eight dirty lines per set exceed the four-way L1D associativity. */
  for (uint32_t set = 0; set < TEST_SETS; set++) {
    for (uint32_t way = 0; way < CONFLICT_LINES; way++) {
      *test_word(set, way) = 0x60000000u ^ (set << 8) ^ way;
    }
  }

  asm volatile("fence rw, rw" ::: "memory");

  /* Reverse order forces evicted dirty lines to be fetched and checked. */
  for (uint32_t set = 0; set < TEST_SETS; set++) {
    for (uint32_t way = CONFLICT_LINES; way-- > 0;) {
      check(*test_word(set, way) ==
            (0x60000000u ^ (set << 8) ^ way));
    }
  }

  return 0;
}

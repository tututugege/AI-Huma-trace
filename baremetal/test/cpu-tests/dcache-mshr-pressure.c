#include "trap.h"
#include <stdint.h>

#define LINE_WORDS 16
#define MISS_LINES 20

static const uint32_t cold_lines[MISS_LINES * LINE_WORDS]
    __attribute__((aligned(64))) = {
        [0 * LINE_WORDS] = 1,   [1 * LINE_WORDS] = 2,
        [2 * LINE_WORDS] = 3,   [3 * LINE_WORDS] = 4,
        [4 * LINE_WORDS] = 5,   [5 * LINE_WORDS] = 6,
        [6 * LINE_WORDS] = 7,   [7 * LINE_WORDS] = 8,
        [8 * LINE_WORDS] = 9,   [9 * LINE_WORDS] = 10,
        [10 * LINE_WORDS] = 11, [11 * LINE_WORDS] = 12,
        [12 * LINE_WORDS] = 13, [13 * LINE_WORDS] = 14,
        [14 * LINE_WORDS] = 15, [15 * LINE_WORDS] = 16,
        [16 * LINE_WORDS] = 17, [17 * LINE_WORDS] = 18,
        [18 * LINE_WORDS] = 19, [19 * LINE_WORDS] = 20,
};

static const uint32_t merge_line[LINE_WORDS] __attribute__((aligned(64))) = {
    0x10203040u, 0x11223344u, 0x55667788u, 0xa5a55a5au,
};

extern uint32_t issue_independent_line_loads(const uint32_t *base);
extern uint32_t issue_same_line_loads(const uint32_t *base);

int main(void) {
  check(issue_independent_line_loads(cold_lines) == 210u);
  check(issue_same_line_loads(merge_line) ==
        (0x10203040u + 0x11223344u + 0x55667788u + 0xa5a55a5au));
  return 0;
}

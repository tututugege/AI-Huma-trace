// F/D: double-precision loads from memory that has never been touched (cold D-cache lines): a 64-bit FP load on rv32 reads two words.
// The values are checked by the per-commit function-model comparison; the sum is also checked here.
#include "trap.h"
#include <stdint.h>

static volatile double tab[32] = {
    1.5, 2.25, -3.125, 4.0, 5.5, 6.75, -7.875, 8.0, 9.5, 10.25, -11.125, 12.0, 13.5, 14.75, -15.875, 16.0,
    17.5, 18.25, -19.125, 20.0, 21.5, 22.75, -23.875, 24.0, 25.5, 26.25, -27.125, 28.0, 29.5, 30.75, -31.875, 32.0};

int fd_main() {
    double s = 0.0;
    for (int i = 0; i < 32; i++) s += tab[i];   // fld of every element, 4 cache lines, first touch
    check(s == 260.0);
    return 0;
}

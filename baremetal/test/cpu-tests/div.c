#include "trap.h"
#include <stdint.h>

extern int32_t hw_div(int32_t dividend, int32_t divisor);
extern uint32_t hw_divu(uint32_t dividend, uint32_t divisor);
extern int32_t hw_rem(int32_t dividend, int32_t divisor);
extern uint32_t hw_remu(uint32_t dividend, uint32_t divisor);

#define N 10
int a[N];

int main() {
  int i, j;
  for(i = 0; i < N; i ++)
    a[i] = i;
  for(i = 0; i < N; i ++)
    for(j = 1; j < N + 1; j ++)
      a[i] *= j;
  for(i = 0; i < N; i ++)
    for(j = 1; j < N + 1; j ++)
      a[i] /= j;

  for(i = 0; i < N; i ++)
    check(a[i] == i);

  /* RV32M signed combinations and the architecturally special cases. */
  check(hw_div(10, 3) == 3);
  check(hw_div(-10, 3) == -3);
  check(hw_div(10, -3) == -3);
  check(hw_div(-10, -3) == 3);
  check(hw_rem(10, 3) == 1);
  check(hw_rem(-10, 3) == -1);
  check(hw_rem(10, -3) == 1);
  check(hw_rem(-10, -3) == -1);

  check((uint32_t)hw_div(123, 0) == UINT32_MAX);
  check(hw_rem(-123, 0) == -123);
  check(hw_div(INT32_MIN, -1) == INT32_MIN);
  check(hw_rem(INT32_MIN, -1) == 0);

  check(hw_divu(UINT32_MAX, 2) == 0x7fffffffu);
  check(hw_remu(UINT32_MAX, 16) == 15u);
  check(hw_divu(123u, 0u) == UINT32_MAX);
  check(hw_remu(0x87654321u, 0u) == 0x87654321u);

  return 0;
}

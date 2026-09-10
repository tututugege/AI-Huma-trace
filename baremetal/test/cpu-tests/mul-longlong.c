#include "trap.h"
#include <stdint.h>

extern uint32_t hw_mul(uint32_t lhs, uint32_t rhs);
extern int32_t hw_mulh(int32_t lhs, int32_t rhs);
extern int32_t hw_mulhsu(int32_t lhs, uint32_t rhs);
extern uint32_t hw_mulhu(uint32_t lhs, uint32_t rhs);

long long mul(long long a,long long b) {
	long long ans = a*b;
	return ans;
}

int test_data[] = { 0xaeb1c2aa, 0x4500ff2b, 0x877190af, 0x11f42438};
long long ans[] = { 0x19d29ab9db1a18e4LL, 0xea15986d3ac3088eLL, 0x2649e980fc0db236LL, 0xfa4c43da0a4a7d30LL, 0x1299898e2c56b139LL, 0xdf8123d50a319e65LL, 0x4d6dfa84c15dd68LL, 0x38c5d79b9e4357a1LL, 0xf78b91cb1efc4248LL, 0x14255a47fdfcc40LL};

#define NR_DATA LENGTH(test_data)

int main() {
	int i,j,ans_idx = 0;
	for (i = 0;i < NR_DATA;i++) {
		for (j = i;j < NR_DATA;j++) {
			check(ans[ans_idx++] == mul(test_data[i],test_data[j]));
		}
		check(j == NR_DATA);
	}

	check(i == NR_DATA);

	/* Exercise every RV32M multiply result variant, especially high halves. */
	check(hw_mul(0xffffffffu, 3u) == 0xfffffffdu);
	check((uint32_t)hw_mulh(-2, 3) == 0xffffffffu);
	check((uint32_t)hw_mulh(INT32_MIN, 2) == 0xffffffffu);
	check((uint32_t)hw_mulh(INT32_MAX, INT32_MAX) == 0x3fffffffu);
	check((uint32_t)hw_mulhsu(-2, 3u) == 0xffffffffu);
	check((uint32_t)hw_mulhsu(INT32_MIN, UINT32_MAX) == 0x80000000u);
	check((uint32_t)hw_mulhsu(INT32_MAX, UINT32_MAX) == 0x7ffffffeu);
	check(hw_mulhu(UINT32_MAX, UINT32_MAX) == 0xfffffffeu);
	check(hw_mulhu(0x80000000u, 2u) == 1u);

	return 0;
}

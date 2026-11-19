#include "trap.h"

// Front-end taken-branch throughput (2-fetch / 2-taken study): a loop of 8 basic blocks, each 3 independent addi + an unconditional
// jump to the next block (4 x 4 B, no C encodings), all perfectly predictable. One taken branch per 4 instructions, so a front end
// that delivers one fetch block per cycle gives 4 instructions per cycle; two blocks (2-taken) give 8.
#define BLK(n) "1" #n ": addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 1" #n "f\n .balign 16\n"
int main() {
	long a1 = 0, a2 = 0, a3 = 0, n = 20000;
	asm volatile(
		".option push\n .option norvc\n .balign 64\n"
		"0: addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 10f\n .balign 16\n"
		"10: addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 11f\n .balign 16\n"
		"11: addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 12f\n .balign 16\n"
		"12: addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 13f\n .balign 16\n"
		"13: addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 14f\n .balign 16\n"
		"14: addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 15f\n .balign 16\n"
		"15: addi %0, %0, 1\n addi %1, %1, 1\n addi %2, %2, 1\n j 16f\n .balign 16\n"
		"16: addi %0, %0, 1\n addi %1, %1, 1\n addi %3, %3, -1\n bnez %3, 0b\n"
		".option pop\n"
		: "+r"(a1), "+r"(a2), "+r"(a3), "+r"(n) :: );
	check(a1 == 160000 && a2 == 160000 && a3 == 140000);
	return 0;
}

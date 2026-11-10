#include "trap.h"

// Indirect-jump predictor test (EXP-025): a bytecode loop whose dispatch targets repeat with a period of 64 and depend on the recent path,
// so a last-target BTB mispredicts most of them while history-indexed target tables (ITT) learn them. Two dispatch styles:
// a jalr call through a function-pointer table (rd = ra) and a computed goto (plain jalr). Both are checked against a switch version.

typedef unsigned (*handler_t)(unsigned);

static unsigned __attribute__((noinline)) h0(unsigned a) { return a + 3; }
static unsigned __attribute__((noinline)) h1(unsigned a) { return a ^ 0x5a5a; }
static unsigned __attribute__((noinline)) h2(unsigned a) { return (a << 1) | (a >> 31); }
static unsigned __attribute__((noinline)) h3(unsigned a) { return a - 7; }
static unsigned __attribute__((noinline)) h4(unsigned a) { return a * 5; }
static unsigned __attribute__((noinline)) h5(unsigned a) { return a ^ (a >> 3); }
static unsigned __attribute__((noinline)) h6(unsigned a) { return a + (a & 0xff); }
static unsigned __attribute__((noinline)) h7(unsigned a) { return ~a; }

static handler_t const table[8] = { h0, h1, h2, h3, h4, h5, h6, h7 };
static unsigned char ops[64];

static unsigned ref_step(unsigned op, unsigned a) {
	switch (op) {
	case 0: return a + 3;
	case 1: return a ^ 0x5a5a;
	case 2: return (a << 1) | (a >> 31);
	case 3: return a - 7;
	case 4: return a * 5;
	case 5: return a ^ (a >> 3);
	case 6: return a + (a & 0xff);
	default: return ~a;
	}
}

int main() {
	unsigned x = 0xace1u, i, r;
	for (i = 0; i < 64; i++) {            // pseudo-random but fixed targets
		x = (x >> 1) ^ (-(x & 1u) & 0xb400u);
		ops[i] = (unsigned char)(x & 7u);
	}
	unsigned a = 1, b = 1;
	for (r = 0; r < 300; r++) {
		for (i = 0; i < 64; i++) {
			const unsigned op = ops[i];
			if (op & 1) { a ^= i; b ^= i; } else { a += r; b += r; }   // conditional branches in front of the dispatch (path history)
			a = table[op](a);                                         // jalr, rd = ra
			b = ref_step(op, b);
		}
	}
	check(a == b);

	static void *const labels[4] = { &&l0, &&l1, &&l2, &&l3 };   // computed goto: plain jalr
	unsigned g = 3, e = 3;
	for (r = 0; r < 120; r++) {
		for (i = 0; i < 64; i++) {
			const unsigned k = ops[i] & 3u;
			goto *labels[k];
l0:			g += 1; goto done;
l1:			g ^= 0x33; goto done;
l2:			g = (g << 2) + 1; goto done;
l3:			g -= 5;
done:
			e = k == 0 ? e + 1 : k == 1 ? e ^ 0x33 : k == 2 ? (e << 2) + 1 : e - 5;
		}
	}
	check(g == e);
	return 0;
}

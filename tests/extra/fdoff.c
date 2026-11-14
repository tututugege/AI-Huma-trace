// F/D instructions with mstatus.FS = Off must raise illegal-instruction (cause 2): flw / fld / fsd and the compressed c.fld / c.fsd (raw encodings, the test
// itself is built without F/D; the compressed ones need an RVC=1 simulator).
#include "trap.h"
#include <stdint.h>

static volatile uint32_t traps, last_cause;
static uint32_t buf[4] __attribute__((aligned(8)));

static void handler(void) __attribute__((interrupt("machine")));
static void handler(void) {
    uint32_t c, e;
    asm volatile("csrr %0, mcause" : "=r"(c));
    asm volatile("csrr %0, mepc" : "=r"(e));
    last_cause = c;
    traps++;
    asm volatile("csrw mepc, %0" : : "r"(e + 4));
}

int main() {
    asm volatile("csrw mtvec, %0" : : "r"(handler));
    asm volatile("li t0, 0x6000\n\tcsrs mstatus, t0\n\tcsrc mstatus, t0" : : : "t0");   // FS = Dirty, then Off again
    volatile uint32_t x = 12345, ones = 0;   // unpredictable branches: mispredicts rewind the trace bridge between the CSR write and the FP accesses
    for (int i = 0; i < 400; i++) { x = x * 1103515245u + 12345u; if ((x >> 16) & 1) ones++; }
    uint32_t *p = buf;
    asm volatile(".insn i 0x07, 3, x0, 0(%0)" : : "r"(p) : "memory");   // fld f0, 0(p)
    check(traps == 1 && last_cause == 2);
    asm volatile(".insn i 0x07, 2, x0, 0(%0)" : : "r"(p) : "memory");   // flw f0, 0(p)
    check(traps == 2 && last_cause == 2);
    asm volatile(".insn s 0x27, 3, x0, 0(%0)" : : "r"(p) : "memory");   // fsd f0, 0(p)
    check(traps == 3 && last_cause == 2);
    asm volatile("mv s0, %0\n\t.hword 0x2000\n\t.hword 0x0001" : : "r"(p) : "s0", "memory");   // c.fld fs0, 0(s0) -- the handler adds 4: it skips the c.nop as well
    check(traps == 4 && last_cause == 2);
    return 0;
}

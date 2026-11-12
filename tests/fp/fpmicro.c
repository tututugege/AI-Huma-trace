// F/D micro-benchmarks for the timing model: KIND 0 = dependent fadd.d chain, 1 = 8 independent fadd.d streams, 2 = dependent fdiv.d chain,
// 3 = dependent fmadd.d chain, 4 = fld + fmadd.d + fsd stream (memory), 5 = int->fp->int round trip chain (fcvt.d.l / fcvt.l.d; rv32: fcvt.d.w / fcvt.w.d)
__asm__(".globl main\nmain:\n li t0, 0x2000\n csrs mstatus, t0\n j fp_main\n");
static double buf[512];
int fp_main(void) {
  double a = 1.0, b = 1.0000001, c0 = 1, c1 = 1, c2 = 1, c3 = 1, c4 = 1, c5 = 1, c6 = 1, c7 = 1;
  for (int i = 0; i < 512; i++) buf[i] = i;
  for (long i = 0; i < N; i++) {
#if KIND == 0
    __asm__ volatile("fadd.d %0, %0, %1\n fadd.d %0, %0, %1\n fadd.d %0, %0, %1\n fadd.d %0, %0, %1" : "+f"(a) : "f"(b));
#elif KIND == 1
    __asm__ volatile("fadd.d %0,%0,%8\n fadd.d %1,%1,%8\n fadd.d %2,%2,%8\n fadd.d %3,%3,%8\n fadd.d %4,%4,%8\n fadd.d %5,%5,%8\n fadd.d %6,%6,%8\n fadd.d %7,%7,%8"
                     : "+f"(c0), "+f"(c1), "+f"(c2), "+f"(c3), "+f"(c4), "+f"(c5), "+f"(c6), "+f"(c7) : "f"(b));
#elif KIND == 2
    __asm__ volatile("fdiv.d %0, %0, %1" : "+f"(a) : "f"(b));
#elif KIND == 3
    __asm__ volatile("fmadd.d %0, %0, %1, %1\n fmadd.d %0, %0, %1, %1\n fmadd.d %0, %0, %1, %1\n fmadd.d %0, %0, %1, %1" : "+f"(a) : "f"(b));
#elif KIND == 4
    double x;
    __asm__ volatile("fld %0, 0(%1)\n fmadd.d %0, %0, %2, %2\n fsd %0, 8(%1)" : "=&f"(x) : "r"(&buf[(i & 63) * 2]), "f"(b) : "memory");
    a += x;
#else
    long k;
    #if __riscv_xlen == 64
    __asm__ volatile("fcvt.l.d %0, %2\n addi %0, %0, 1\n fcvt.d.l %1, %0" : "=&r"(k), "=f"(a) : "f"(a));
#else
    __asm__ volatile("fcvt.w.d %0, %2\n addi %0, %0, 1\n fcvt.d.w %1, %0" : "=&r"(k), "=f"(a) : "f"(a));
#endif
#endif
  }
  return (a + c0 + c1 + c2 + c3 + c4 + c5 + c6 + c7) > 0 ? 0 : 1;
}

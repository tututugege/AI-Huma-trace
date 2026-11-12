// F/D (rv32imafd, ilp32d) version of the former Zfinx test zfinx.c: the operands stay uint32_t bit patterns, each operation moves them through the FP registers.
// The entry wrapper (fd-basic-helper.S) turns mstatus.FS on and calls fd_main.
#include "trap.h"
#include <stdint.h>

uint32_t fadd_zfinx(uint32_t a, uint32_t b) {
    uint32_t res;
    asm volatile (
        "fmv.w.x ft0, %1\n\tfmv.w.x ft1, %2\n\tfadd.s ft0, ft0, ft1, rne\n\tfmv.x.w %0, ft0"
        : "=r"(res)
        : "r"(a), "r"(b) : "ft0", "ft1");
    return res;
}

uint32_t fmul_zfinx(uint32_t a, uint32_t b) {
    uint32_t res;
    asm volatile (
        "fmv.w.x ft0, %1\n\tfmv.w.x ft1, %2\n\tfmul.s ft0, ft0, ft1, rne\n\tfmv.x.w %0, ft0"
        : "=r"(res)
        : "r"(a), "r"(b) : "ft0", "ft1");
    return res;
}

uint32_t fsub_zfinx(uint32_t a, uint32_t b) {
    uint32_t res;
    asm volatile (
        "fmv.w.x ft0, %1\n\tfmv.w.x ft1, %2\n\tfsub.s ft0, ft0, ft1, rne\n\tfmv.x.w %0, ft0"
        : "=r"(res)
        : "r"(a), "r"(b) : "ft0", "ft1");
    return res;
}

int fd_main() {
    uint32_t a = 0x3f800000; // 1.0
    uint32_t b = 0x40000000; // 2.0
    
    uint32_t sum = fadd_zfinx(a, b);
    uint32_t prod = fmul_zfinx(a, b);
    uint32_t diff = fsub_zfinx(sum, a);
    
    // 1.0 + 2.0 = 3.0 (0x40400000)
    // 1.0 * 2.0 = 2.0 (0x40000000)
    // 3.0 - 1.0 = 2.0 (0x40000000)
    
    check(sum == 0x40400000);
    check(prod == 0x40000000);
    check(diff == 0x40000000);
    
    return 0;
}

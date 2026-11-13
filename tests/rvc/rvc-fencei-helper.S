#if __riscv_xlen == 64
#define REG_S sd
#define REG_L ld
#else
#define REG_S sw
#define REG_L lw
#endif
    .section .text
    .align 2

    .globl execute_generated_code
execute_generated_code:
    addi sp, sp, -16
    REG_S ra, 8(sp)
    jalr ra, 0(a0)
    REG_L ra, 8(sp)
    addi sp, sp, 16
    ret

    .globl fence_i_sync
fence_i_sync:
    fence.i
    ret

    .section .text
    .align 2

    .globl execute_generated_code
execute_generated_code:
    addi sp, sp, -16
#if __riscv_xlen == 64
    sd ra, 8(sp)
#else
    sw ra, 12(sp)
#endif
    jalr ra, 0(a0)
#if __riscv_xlen == 64
    ld ra, 8(sp)
#else
    lw ra, 12(sp)
#endif
    addi sp, sp, 16
    ret

    .globl fence_i_sync
fence_i_sync:
    .word 0x0000100f       # fence.i
    ret

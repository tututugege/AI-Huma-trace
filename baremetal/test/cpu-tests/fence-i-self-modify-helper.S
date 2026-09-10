    .section .text
    .align 2

    .globl execute_generated_code
execute_generated_code:
    addi sp, sp, -16
    sw ra, 12(sp)
    jalr ra, 0(a0)
    lw ra, 12(sp)
    addi sp, sp, 16
    ret

    .globl fence_i_sync
fence_i_sync:
    .word 0x0000100f       # fence.i
    ret

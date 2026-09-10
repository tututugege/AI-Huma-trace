    .section .text
    .align 2

    .globl issue_independent_line_loads
issue_independent_line_loads:
    addi sp, sp, -32
    sw s0, 0(sp)
    sw s1, 4(sp)
    sw s2, 8(sp)
    sw s3, 12(sp)
    sw s4, 16(sp)
    sw s5, 20(sp)
    sw s6, 24(sp)

    # All addresses are on distinct 64-byte cache lines.  Keep every load
    # independent until they have all entered the LSU.
    lw t0, 0(a0)
    lw t1, 64(a0)
    lw t2, 128(a0)
    lw t3, 192(a0)
    lw t4, 256(a0)
    lw t5, 320(a0)
    lw t6, 384(a0)
    lw a1, 448(a0)
    lw a2, 512(a0)
    lw a3, 576(a0)
    lw a4, 640(a0)
    lw a5, 704(a0)
    lw a6, 768(a0)
    lw a7, 832(a0)
    lw s0, 896(a0)
    lw s1, 960(a0)
    lw s2, 1024(a0)
    lw s3, 1088(a0)
    lw s4, 1152(a0)
    lw s5, 1216(a0)

    add t0, t0, t1
    add t0, t0, t2
    add t0, t0, t3
    add t0, t0, t4
    add t0, t0, t5
    add t0, t0, t6
    add t0, t0, a1
    add t0, t0, a2
    add t0, t0, a3
    add t0, t0, a4
    add t0, t0, a5
    add t0, t0, a6
    add t0, t0, a7
    add t0, t0, s0
    add t0, t0, s1
    add t0, t0, s2
    add t0, t0, s3
    add t0, t0, s4
    add a0, t0, s5

    lw s0, 0(sp)
    lw s1, 4(sp)
    lw s2, 8(sp)
    lw s3, 12(sp)
    lw s4, 16(sp)
    lw s5, 20(sp)
    lw s6, 24(sp)
    addi sp, sp, 32
    ret

    .globl issue_same_line_loads
issue_same_line_loads:
    lw t0, 0(a0)
    lw t1, 4(a0)
    lw t2, 8(a0)
    lw t3, 12(a0)
    add t0, t0, t1
    add t0, t0, t2
    add a0, t0, t3
    ret

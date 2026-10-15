/* Discarding LR's loaded value must still establish a reservation. */
    .option norvc
    .option norelax
    .section .text.start,"ax",@progbits
    .globl _start
_start:
    lui t0, 0x80001
    li t1, 0x77
    lr.w x0, (t0)
    sc.w t2, t1, (t0)
    bnez t2, fail
    lw t2, 0(t0)
    bne t2, t1, fail
    li a0, 0
    ebreak
fail:
    li a0, 1
    ebreak
    .org 0x1000
    .word 0xac

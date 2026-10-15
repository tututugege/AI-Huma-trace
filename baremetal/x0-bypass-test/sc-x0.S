/* Discard SC's status without discarding its store/reservation effects.
 * A failed SC produces nonzero status, exercising another P0 producer.
 */
    .option norvc
    .option norelax
    .section .text.start,"ax",@progbits
    .globl _start
_start:
    lui t0, 0x80001
    li t1, 0x55
    li t2, 0x77
    lr.w t3, (t0)
    li t4, 0xac
    bne t3, t4, fail
    sc.w x0, t1, (t0)     # Must still store 0x55 and consume reservation.
    lw t3, 0(t0)
    bne t3, t1, fail
    sc.w x0, t2, (t0)     # Must fail without changing memory.
    add t4, x0, t1
    add t5, t1, x0
    sub t6, t1, x0
    bne t4, t1, fail
    bne t5, t1, fail
    bne t6, t1, fail
    lw t3, 0(t0)
    bne t3, t1, fail
    li a0, 0
    ebreak
fail:
    li a0, 1
    ebreak
    .org 0x1000
    .word 0xac

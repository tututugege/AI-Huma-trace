#if __riscv_xlen == 64
#define REG_S sd
#define REG_L ld
#define SHIFT 3
#else
#define REG_S sw
#define REG_L lw
#define SHIFT 2
#endif
    .section .text
    .align 2

irq_handler:
    csrr t5, mepc
    la t6, irq_count
    lw t4, 0(t6)             # t4 = index
    la t3, irq_mepc
    slli t2, t4, 2
    add t3, t3, t2
    sw t5, 0(t3)
    addi t4, t4, 1
    sw t4, 0(t6)
    li t5, 8
    csrc mip, t5
    mret

    .macro SEG pre
    .balign 4
    c.nop                    # the stream below starts at 2 mod 4
    addi a2, a2, 1           # (16-bit: c.addi) segment number
    .option push
    .option norvc
    .rept \pre
    addi a0, a0, 1
    .endr
    csrs mip, a3             # raise MSIP
    .rept 40
    addi a0, a0, 1
    .endr
    .option pop
    la t0, irq_count
1:  lw t1, 0(t0)             # wait for the interrupt to have been taken
    bne t1, a2, 1b
    .endm

    .globl run_irq_segments
run_irq_segments:
    addi sp, sp, -16
    REG_S ra, 8(sp)
    la t0, irq_handler
    csrw mtvec, t0
    li t0, 8
    csrs mie, t0             # MSIE
    csrs mstatus, t0         # MIE
    li a0, 0
    li a2, 0
    li a3, 8
    .irp pre, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
    SEG \pre
    .endr
    li t0, 8
    csrc mstatus, t0
    REG_L ra, 8(sp)
    addi sp, sp, 16
    ret

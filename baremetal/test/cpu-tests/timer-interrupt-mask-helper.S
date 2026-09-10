    .section .text
    .align 2

    .globl install_timer_trap_vector
install_timer_trap_vector:
    la t0, timer_trap_vector
    .word 0x30529073       # csrw mtvec, t0
    ret

    .align 2
timer_trap_vector:
    addi sp, sp, -32
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)
    sw t4, 16(sp)
    sw t5, 20(sp)

    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    .word 0x344023f3       # csrr t2, mip
    .word 0x30002e73       # csrr t3, mstatus

    la t4, last_mcause
    sw t0, 0(t4)
    la t4, last_mepc
    sw t1, 0(t4)
    la t4, last_mip
    sw t2, 0(t4)
    la t4, trap_mstatus
    sw t3, 0(t4)
    la t4, trap_count
    lw t5, 0(t4)
    addi t5, t5, 1
    sw t5, 0(t4)

    # Disable MTIE before returning so delayed MTIP deassertion cannot retrap.
    .word 0x304022f3       # csrr t0, mie
    li t2, -129
    and t0, t0, t2
    .word 0x30429073       # csrw mie, t0

    # Move mtimecmp to UINT64_MAX and deassert the timer source.
    li t0, 0x1fd0e008
    li t2, -1
    sw t2, 0(t0)
    sw t2, 4(t0)

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    lw t4, 16(sp)
    lw t5, 20(sp)
    addi sp, sp, 32
    .word 0x30200073       # mret; interrupt mepc is not advanced

    .align 2
    .globl read_mstatus_asm
read_mstatus_asm:
    .word 0x30002573       # csrr a0, mstatus
    ret

    .globl read_mie_asm
read_mie_asm:
    .word 0x30402573       # csrr a0, mie
    ret

    .globl read_mip_asm
read_mip_asm:
    .word 0x34402573       # csrr a0, mip
    ret

    .globl write_mie_asm
write_mie_asm:
    .word 0x30451073       # csrw mie, a0
    ret

    .globl set_mstatus_bits_asm
set_mstatus_bits_asm:
    .word 0x30052073       # csrs mstatus, a0
    ret

    .globl clear_mstatus_bits_asm
clear_mstatus_bits_asm:
    .word 0x30053073       # csrc mstatus, a0
    ret

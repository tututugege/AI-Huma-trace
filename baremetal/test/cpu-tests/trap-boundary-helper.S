    .section .text
    .align 2

    .globl install_trap_vector
install_trap_vector:
    la t0, trap_vector
    .word 0x30529073       # csrw mtvec, t0
    ret

    .globl trigger_ecall
trigger_ecall:
    .globl ecall_site
ecall_site:
    ecall
    ret

    .align 2
trap_vector:
    addi sp, sp, -32
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)
    sw t4, 16(sp)

    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    .word 0x343023f3       # csrr t2, mtval
    .word 0x30002e73       # csrr t3, mstatus

    la t4, last_mcause
    sw t0, 0(t4)
    la t4, last_mepc
    sw t1, 0(t4)
    la t4, last_mtval
    sw t2, 0(t4)
    la t4, trap_mstatus
    sw t3, 0(t4)
    la t4, trap_count
    lw t0, 0(t4)
    addi t0, t0, 1
    sw t0, 0(t4)

    addi t1, t1, 4
    .word 0x34131073       # csrw mepc, t1

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    lw t4, 16(sp)
    addi sp, sp, 32
    .word 0x30200073       # mret

    .align 2
    .globl read_mstatus_asm
read_mstatus_asm:
    .word 0x30002573       # csrr a0, mstatus
    ret

    .align 2
    .globl set_mstatus_bits_asm
set_mstatus_bits_asm:
    .word 0x30052073       # csrs mstatus, a0
    ret

    .align 2
    .globl clear_mstatus_bits_asm
clear_mstatus_bits_asm:
    .word 0x30053073       # csrc mstatus, a0
    ret

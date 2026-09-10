    .section .text
    .align 2

    .globl install_illegal_trap_vector
install_illegal_trap_vector:
    la t0, illegal_trap_vector
    .word 0x30529073       # csrw mtvec, t0
    ret

    .globl trigger_illegal_instruction
trigger_illegal_instruction:
    .globl illegal_instruction_site
illegal_instruction_site:
    .word 0xffffffff       # deliberately unsupported instruction encoding
    ret

    .align 2
illegal_trap_vector:
    addi sp, sp, -16
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)

    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    .word 0x343023f3       # csrr t2, mtval
    la t3, illegal_mcause
    sw t0, 0(t3)
    la t3, illegal_mepc
    sw t1, 0(t3)
    la t3, illegal_mtval
    sw t2, 0(t3)
    la t3, illegal_trap_count
    lw t0, 0(t3)
    addi t0, t0, 1
    sw t0, 0(t3)

    addi t1, t1, 4
    .word 0x34131073       # csrw mepc, t1

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    addi sp, sp, 16
    .word 0x30200073       # mret

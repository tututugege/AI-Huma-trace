    .section .text
    .align 2

    .globl install_precise_trap_vector
install_precise_trap_vector:
    la t0, precise_trap_vector
    .word 0x30529073       # csrw mtvec, t0
    ret

    .globl store_ecall_store
store_ecall_store:
    sw a1, 0(a0)
    .globl precise_ecall_site
precise_ecall_site:
    .option push
    .option norvc          # the handler skips 8 bytes: ecall + this 4-byte store
    ecall
    sw a3, 0(a2)           # must be skipped/squashed by the handler
    .option pop
    ret

    .align 2
precise_trap_vector:
    addi sp, sp, -16
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)

    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    la t2, last_mcause
    sw t0, 0(t2)
    la t2, last_mepc
    sw t1, 0(t2)
    la t2, trap_count
    lw t0, 0(t2)
    addi t0, t0, 1
    sw t0, 0(t2)

    addi t1, t1, 8         # skip the younger store and resume at ret
    .word 0x34131073       # csrw mepc, t1

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    addi sp, sp, 16
    .word 0x30200073       # mret

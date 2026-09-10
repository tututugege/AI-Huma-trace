    .section .text
    .align 2

    .globl run_software_interrupt_test
run_software_interrupt_test:
    la t0, software_saved_ra
    sw ra, 0(t0)

    # Phase 1: M-mode software interrupt, vectored slot 3.
    la t0, machine_software_vectors
    ori t0, t0, 1
    .word 0x30529073       # csrw mtvec, t0
    li t0, 1 << 3
    .word 0x3042a073       # csrs mie, t0 (MSIE)
    .word 0x3002a073       # csrs mstatus, t0 (MIE)
    .word 0x3442a073       # csrs mip, t0 (MSIP)

1:
    la t0, machine_software_count
    lw t1, 0(t0)
    beqz t1, 1b

    # Phase 2: enter S-mode with SSIP pending and take vectored slot 1.
    la t0, software_machine_exit
    .word 0x30529073       # csrw mtvec, t0
    la t0, supervisor_software_vectors
    ori t0, t0, 1
    .word 0x10529073       # csrw stvec, t0
    li t0, 1 << 1
    .word 0x3032a073       # csrs mideleg, t0 (SSIP)
    .word 0x1042a073       # csrs sie, t0 (SSIE)
    .word 0x1002a073       # csrs sstatus, t0 (SIE)
    .word 0x3442a073       # csrs mip, t0 (SSIP)

    la t0, software_supervisor_entry
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 1 << 11
    or t0, t0, t1
    .word 0x30029073       # csrw mstatus, t0 (MPP = S)
    .word 0x30200073       # mret

software_supervisor_entry:
    la t0, supervisor_software_count
2:
    lw t1, 0(t0)
    beqz t1, 2b
    ecall                  # return to M-mode test harness
3:
    j 3b

    .balign 64
machine_software_vectors:
    j software_unexpected_vector
    j software_unexpected_vector
    j software_unexpected_vector
    j machine_software_vector

machine_software_vector:
    addi sp, sp, -24
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)

    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    la t2, machine_software_mcause
    sw t0, 0(t2)
    la t2, machine_software_mepc
    sw t1, 0(t2)
    la t2, machine_software_slot
    li t3, 3
    sw t3, 0(t2)
    la t2, machine_software_count
    lw t3, 0(t2)
    addi t3, t3, 1
    sw t3, 0(t2)

    li t0, 1 << 3
    .word 0x3442b073       # csrc mip, t0 (clear MSIP)
    .word 0x3042b073       # csrc mie, t0 (disable MSIE)

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    addi sp, sp, 24
    .word 0x30200073       # mret

    .balign 64
supervisor_software_vectors:
    j software_unexpected_vector
    j supervisor_software_vector

supervisor_software_vector:
    addi sp, sp, -24
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)

    .word 0x142022f3       # csrr t0, scause
    .word 0x14102373       # csrr t1, sepc
    la t2, supervisor_software_scause
    sw t0, 0(t2)
    la t2, supervisor_software_sepc
    sw t1, 0(t2)
    la t2, supervisor_software_slot
    li t3, 1
    sw t3, 0(t2)
    la t2, supervisor_software_count
    lw t3, 0(t2)
    addi t3, t3, 1
    sw t3, 0(t2)

    li t0, 1 << 1
    .word 0x1442b073       # csrc sip, t0 (clear SSIP)
    .word 0x1042b073       # csrc sie, t0 (disable SSIE)

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    addi sp, sp, 24
    .word 0x10200073       # sret

software_unexpected_vector:
    li t0, 0x1c
    li t1, 1
    sw t1, 0(t0)
1:
    j 1b

software_machine_exit:
    .word 0x342022f3       # csrr t0, mcause
    la t1, software_exit_mcause
    sw t0, 0(t1)
    la t0, software_machine_resume
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 3 << 11
    or t0, t0, t1
    .word 0x30029073       # csrw mstatus, t0 (MPP = M)
    .word 0x30200073       # mret

software_machine_resume:
    la t0, software_saved_ra
    lw ra, 0(t0)
    ret

    .section .bss
    .align 2
software_saved_ra:
    .word 0

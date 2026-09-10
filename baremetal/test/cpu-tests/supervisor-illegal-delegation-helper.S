    .section .text
    .align 2

    .globl run_supervisor_illegal_delegation
run_supervisor_illegal_delegation:
    la t0, delegated_saved_ra
    sw ra, 0(t0)
    la t0, delegated_machine_vector
    .word 0x30529073       # csrw mtvec, t0
    la t0, delegated_supervisor_vector
    .word 0x10529073       # csrw stvec, t0
    li t0, 1 << 2
    .word 0x30229073       # csrw medeleg, t0
    la t0, delegated_supervisor_entry
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 1 << 11
    or t0, t0, t1
    .word 0x30029073       # csrw mstatus, t0 (MPP = S)
    .word 0x30200073       # mret

delegated_supervisor_entry:
    .globl supervisor_illegal_site
supervisor_illegal_site:
    .word 0xffffffff
    la t0, delegated_resume_marker
    li t1, 1
    sw t1, 0(t0)
    .globl supervisor_exit_ecall_site
supervisor_exit_ecall_site:
    ecall
1:
    j 1b

    .align 2
delegated_supervisor_vector:
    addi sp, sp, -20
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)
    .word 0x142022f3       # csrr t0, scause
    .word 0x14102373       # csrr t1, sepc
    .word 0x143023f3       # csrr t2, stval
    la t3, delegated_scause
    sw t0, 0(t3)
    la t3, delegated_sepc
    sw t1, 0(t3)
    la t3, delegated_stval
    sw t2, 0(t3)
    la t3, delegated_illegal_count
    lw t0, 0(t3)
    addi t0, t0, 1
    sw t0, 0(t3)
    addi t1, t1, 4
    .word 0x14131073       # csrw sepc, t1
    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    addi sp, sp, 20
    .word 0x10200073       # sret

    .align 2
delegated_machine_vector:
    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    .word 0x300023f3       # csrr t2, mstatus
    la t3, delegated_exit_mcause
    sw t0, 0(t3)
    la t3, delegated_exit_mepc
    sw t1, 0(t3)
    la t3, delegated_exit_mstatus
    sw t2, 0(t3)
    la t0, delegated_machine_resume
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 3 << 11
    or t0, t0, t1
    .word 0x30029073       # csrw mstatus, t0 (MPP = M)
    .word 0x30200073       # mret

delegated_machine_resume:
    la t0, delegated_saved_ra
    lw ra, 0(t0)
    ret

    .section .bss
    .align 2
delegated_saved_ra:
    .word 0

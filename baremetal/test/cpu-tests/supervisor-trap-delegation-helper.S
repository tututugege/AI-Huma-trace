#if __riscv_xlen == 64
#define REG_S sd
#define REG_L ld
#define REG_W .dword
#define LWU lwu
#else
#define REG_S sw
#define REG_L lw
#define REG_W .word
#define LWU lw
#endif
    .section .text
    .align 2

    .globl run_supervisor_trap_delegation
run_supervisor_trap_delegation:
    la t0, saved_machine_ra
    REG_S ra, 0(t0)

    la t0, machine_trap_vector
    .word 0x30529073       # csrw mtvec, t0
    la t0, supervisor_trap_vector
    .word 0x10529073       # csrw stvec, t0
    li t0, 1 << 9
    .word 0x30229073       # csrw medeleg, t0 (delegate S-mode ecall)

    # Make the SIE -> SPIE transition visible at the delegated trap.
    li t0, 1 << 1
    .word 0x3002a073       # csrs mstatus, t0

    la t0, supervisor_entry
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 1 << 11
    or t0, t0, t1         # MPP = supervisor
    .word 0x30029073       # csrw mstatus, t0
    .word 0x30200073       # mret

supervisor_entry:
    la t0, supervisor_marker
    li t1, 1
    sw t1, 0(t0)
    .globl supervisor_ecall_site
supervisor_ecall_site:
    ecall
    li t1, 2
    sw t1, 0(t0)

    # Return from supervisor mode into user mode.
    la t0, user_entry
    .word 0x14129073       # csrw sepc, t0
    li t0, 1 << 8
    .word 0x1002b073       # csrc sstatus, t0 (SPP = user)
    .word 0x10200073       # sret

user_entry:
    la t0, user_marker
    li t1, 1
    sw t1, 0(t0)
    .globl user_ecall_site
user_ecall_site:
    ecall
1:
    j 1b

    .align 2
supervisor_trap_vector:
    addi sp, sp, -32
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)
    sw t4, 16(sp)
    .word 0x142022f3       # csrr t0, scause
    .word 0x14102373       # csrr t1, sepc
    .word 0x100023f3       # csrr t2, sstatus
    la t3, supervisor_scause
    sw t0, 0(t3)
    la t3, supervisor_sepc
    sw t1, 0(t3)
    la t3, supervisor_sstatus
    sw t2, 0(t3)
    la t3, supervisor_trap_count
    lw t4, 0(t3)
    addi t4, t4, 1
    sw t4, 0(t3)
    addi t1, t1, 4
    .word 0x14131073       # csrw sepc, t1
    LWU t0, 0(sp)
    LWU t1, 4(sp)
    LWU t2, 8(sp)
    LWU t3, 12(sp)
    LWU t4, 16(sp)
    addi sp, sp, 32
    .word 0x10200073       # sret

    .align 2
machine_trap_vector:
    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    .word 0x300023f3       # csrr t2, mstatus
    la t3, machine_mcause
    sw t0, 0(t3)
    la t3, machine_mepc
    sw t1, 0(t3)
    la t3, machine_mstatus
    sw t2, 0(t3)
    la t3, machine_trap_count
    lw t4, 0(t3)
    addi t4, t4, 1
    sw t4, 0(t3)

    la t0, machine_resume
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 3 << 11
    or t0, t0, t1         # MPP = machine
    .word 0x30029073       # csrw mstatus, t0
    .word 0x30200073       # mret

machine_resume:
    la t0, machine_resume_marker
    li t1, 1
    sw t1, 0(t0)
    la t0, saved_machine_ra
    REG_L ra, 0(t0)
    ret

    .section .bss
    .align 2
    .balign 8
saved_machine_ra:
    REG_W 0

    .section .text
    .align 2

    .globl install_vectored_trap
install_vectored_trap:
    la t0, vector_base
    ori t0, t0, 1
    .word 0x30529073       # csrw mtvec, t0
    ret

    .globl enable_timer_interrupt
enable_timer_interrupt:
    .word 0x304022f3       # csrr t0, mie
    ori t0, t0, 128
    .word 0x30429073       # csrw mie, t0
    li t0, 8
    .word 0x3002a073       # csrs mstatus, t0
    ret

    .globl clear_mstatus_mie
clear_mstatus_mie:
    li t0, 8
    .word 0x3002b073       # csrc mstatus, t0
    ret

    .balign 64
vector_base:
    j unexpected_vector   # direct exceptions
    j unexpected_vector
    j unexpected_vector
    j unexpected_vector
    j unexpected_vector
    j unexpected_vector
    j unexpected_vector
    j machine_timer_vector

unexpected_vector:
    li t0, 0x1c
    li t1, 1
    sw t1, 0(t0)
1:
    j 1b

machine_timer_vector:
    addi sp, sp, -24
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)

    .word 0x342022f3       # csrr t0, mcause
    .word 0x34102373       # csrr t1, mepc
    la t2, last_mcause
    sw t0, 0(t2)
    la t2, last_mepc
    sw t1, 0(t2)
    la t2, vector_slot
    li t3, 7
    sw t3, 0(t2)
    la t2, trap_count
    lw t3, 0(t2)
    addi t3, t3, 1
    sw t3, 0(t2)

    # Disable MTIE and move mtimecmp to UINT64_MAX before returning.
    .word 0x304022f3       # csrr t0, mie
    li t1, -129
    and t0, t0, t1
    .word 0x30429073       # csrw mie, t0
    li t0, 0x1fd0e008
    li t1, -1
    sw t1, 0(t0)
    sw t1, 4(t0)

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    addi sp, sp, 24
    .word 0x30200073       # mret

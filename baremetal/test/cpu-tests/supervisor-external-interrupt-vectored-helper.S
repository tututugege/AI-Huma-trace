#if __riscv_xlen == 64
#define REG_S sd
#define REG_L ld
#define REG_W .dword
#else
#define REG_S sw
#define REG_L lw
#define REG_W .word
#endif
    .equ UART_THR,       0x10000000
    .equ UART_IER,       0x10000001
    .equ UART_IIR,       0x10000002
    .equ XPS_IER,        0x1fb00008
    .equ XPS_IAR,        0x1fb0000c
    .equ XPS_MER,        0x1fb0001c
    .equ UART_IRQ_MASK,  (1 << 2)

    .section .text
    .align 2

    .globl run_supervisor_external_interrupt
run_supervisor_external_interrupt:
    la t0, external_saved_ra
    REG_S ra, 0(t0)

    la t0, external_machine_vector
    .word 0x30529073       # csrw mtvec, t0
    la t0, external_supervisor_vectors
    ori t0, t0, 1
    .word 0x10529073       # csrw stvec, t0 (vectored)

    li t0, 1 << 9
    .word 0x30329073       # csrw mideleg, t0 (SEIP)
    .word 0x1042a073       # csrs sie, t0 (SEIE)
    li t0, 1 << 1
    .word 0x1002a073       # csrs sstatus, t0 (SIE)

    # Reset and enable UART IRQ 2 in the XPS interrupt controller.
    li t0, UART_IER
    sb zero, 0(t0)
    li t0, XPS_IAR
    li t1, -1
    sw t1, 0(t0)
    li t0, XPS_IER
    li t1, UART_IRQ_MASK
    sw t1, 0(t0)
    li t0, XPS_MER
    li t1, 3
    sw t1, 0(t0)

    # A THR write makes THRE/TEMT visible, then enabling THRI asserts IRQ 2.
    li t0, UART_THR
    li t1, 0x45            # 'E'
    sb t1, 0(t0)
    li t0, UART_IER
    li t1, 2
    sb t1, 0(t0)

    la t0, external_supervisor_entry
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 1 << 11
    or t0, t0, t1
    .word 0x30029073       # csrw mstatus, t0 (MPP = S)
    .word 0x30200073       # mret

external_supervisor_entry:
    la t0, external_trap_count
1:
    lw t1, 0(t0)
    beqz t1, 1b
    ecall                  # deliberately not delegated; return to M
2:
    j 2b

    .balign 64
external_supervisor_vectors:
    .option push
    .option norvc   # a vector table of 4-byte slots
    j external_unexpected_vector   # synchronous exceptions
    j external_unexpected_vector   # SSIP
    j external_unexpected_vector
    j external_unexpected_vector
    j external_unexpected_vector
    j external_unexpected_vector   # STIP
    j external_unexpected_vector
    j external_unexpected_vector
    j external_unexpected_vector
    j external_supervisor_vector   # SEIP
    .option pop

external_unexpected_vector:
    li t0, 0x1c
    li t1, 1
    sw t1, 0(t0)
1:
    j 1b

external_supervisor_vector:
    addi sp, sp, -32
    sw t0, 0(sp)
    sw t1, 4(sp)
    sw t2, 8(sp)
    sw t3, 12(sp)
    sw t4, 16(sp)
    sw t5, 20(sp)

    .word 0x142022f3       # csrr t0, scause
    .word 0x14102373       # csrr t1, sepc
    .word 0x100023f3       # csrr t2, sstatus
    la t3, external_scause
    sw t0, 0(t3)
    la t3, external_sepc
    sw t1, 0(t3)
    la t3, external_sstatus
    sw t2, 0(t3)
    la t3, external_vector_slot
    li t4, 9
    sw t4, 0(t3)

    li t0, 0x1fb00004      # XPS IPR before acknowledge
    lw t1, 0(t0)
    la t3, external_xps_ipr
    sw t1, 0(t3)
    li t0, 0x1fb00018      # XPS IVR
    lw t1, 0(t0)
    la t3, external_xps_ivr
    sw t1, 0(t3)
    li t0, UART_IIR
    lbu t1, 0(t0)
    la t3, external_uart_iir
    sw t1, 0(t3)

    # Clear the source before reenabling supervisor interrupts.
    li t0, UART_IER
    sb zero, 0(t0)
    li t0, XPS_IAR
    li t1, UART_IRQ_MASK
    sw t1, 0(t0)
    li t0, 1 << 9
    .word 0x1042b073       # csrc sie, t0
    li t0, 1 << 1
    .word 0x1002b073       # csrc sstatus, t0

    la t3, external_trap_count
    lw t4, 0(t3)
    addi t4, t4, 1
    sw t4, 0(t3)

    lw t0, 0(sp)
    lw t1, 4(sp)
    lw t2, 8(sp)
    lw t3, 12(sp)
    lw t4, 16(sp)
    lw t5, 20(sp)
    addi sp, sp, 32
    .word 0x10200073       # sret

    .align 2
external_machine_vector:
    .word 0x342022f3       # csrr t0, mcause
    la t1, external_exit_mcause
    sw t0, 0(t1)
    la t0, external_machine_resume
    .word 0x34129073       # csrw mepc, t0
    .word 0x300022f3       # csrr t0, mstatus
    li t1, ~(3 << 11)
    and t0, t0, t1
    li t1, 3 << 11
    or t0, t0, t1
    .word 0x30029073       # csrw mstatus, t0 (MPP = M)
    .word 0x30200073       # mret

external_machine_resume:
    la t0, external_saved_ra
    REG_L ra, 0(t0)
    ret

    .section .bss
    .align 2
    .balign 8
external_saved_ra:
    REG_W 0

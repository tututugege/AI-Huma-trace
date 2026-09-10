    .section .text
    .align 2

    .globl read_mscratch_asm
read_mscratch_asm:
    .word 0x34002573       # csrr a0, mscratch
    ret

    .globl write_mscratch_asm
write_mscratch_asm:
    .word 0x34051073       # csrw mscratch, a0
    ret

    .globl swap_mscratch_asm
swap_mscratch_asm:
    mv a1, a0
    .word 0x34059573       # csrrw a0, mscratch, a1
    ret

    .globl set_mscratch_asm
set_mscratch_asm:
    mv a1, a0
    .word 0x3405a573       # csrrs a0, mscratch, a1
    ret

    .globl clear_mscratch_asm
clear_mscratch_asm:
    mv a1, a0
    .word 0x3405b573       # csrrc a0, mscratch, a1
    ret

    .globl read_mstatus_asm
read_mstatus_asm:
    .word 0x30002573       # csrr a0, mstatus
    ret

    .globl read_sstatus_asm
read_sstatus_asm:
    .word 0x10002573       # csrr a0, sstatus
    ret

    .globl write_sstatus_asm
write_sstatus_asm:
    .word 0x10051073       # csrw sstatus, a0
    ret

    .globl read_mie_asm
read_mie_asm:
    .word 0x30402573       # csrr a0, mie
    ret

    .globl read_sie_asm
read_sie_asm:
    .word 0x10402573       # csrr a0, sie
    ret

    .globl write_mie_asm
write_mie_asm:
    .word 0x30451073       # csrw mie, a0
    ret

    .globl write_sie_asm
write_sie_asm:
    .word 0x10451073       # csrw sie, a0
    ret

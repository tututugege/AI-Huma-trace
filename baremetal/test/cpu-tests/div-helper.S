    .section .text
    .align 2

    # Encode RV32M explicitly: the common test harness intentionally compiles
    # C as rv32i, which would otherwise call libgcc instead of the hardware FU.
    .globl hw_div
hw_div:
    .word 0x02b54533              # div  a0, a0, a1
    ret

    .globl hw_divu
hw_divu:
    .word 0x02b55533              # divu a0, a0, a1
    ret

    .globl hw_rem
hw_rem:
    .word 0x02b56533              # rem  a0, a0, a1
    ret

    .globl hw_remu
hw_remu:
    .word 0x02b57533              # remu a0, a0, a1
    ret

    .section .text
    .align 2

    # Explicit RV32M encodings keep the common C harness at -march=rv32i.
    .globl hw_mul
hw_mul:
    .word 0x02b50533              # mul    a0, a0, a1
    ret

    .globl hw_mulh
hw_mulh:
    .word 0x02b51533              # mulh   a0, a0, a1
    ret

    .globl hw_mulhsu
hw_mulhsu:
    .word 0x02b52533              # mulhsu a0, a0, a1
    ret

    .globl hw_mulhu
hw_mulhu:
    .word 0x02b53533              # mulhu  a0, a0, a1
    ret

    .section .text
    .globl main
main:
    li t0, 0x2000
    csrs mstatus, t0
    j fd_main

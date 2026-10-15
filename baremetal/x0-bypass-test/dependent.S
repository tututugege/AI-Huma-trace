/* Preserve the handoff reproducer's instruction layout: third ADD is at
 * 0x8000004c. Difftest checks every commit, including overwritten results.
 * Do not insert checks/fences in the stream: they change the overlap timing.
 */
#ifndef LOAD_DEST
#define LOAD_DEST x0
#endif
#ifndef LOAD_OFFSET
#define LOAD_OFFSET 0
#endif
    .option norvc
    .option norelax
    .section .text.start,"ax",@progbits
    .globl _start
_start:
    lui x5, 0x80001
    .rept 4
    lbu x6, LOAD_OFFSET(x5)
    lbu LOAD_DEST, LOAD_OFFSET(x5)
    add x23, x0, x6
    add x24, x6, x0
    xor x21, x6, x0
    sub x20, x6, x0
    addi x22, x6, 1
    lui x19, 0xaa9b0
    .endr
    addi x10, x0, 0
    ebreak
    .org 0x1000
    .byte 0xac, 0

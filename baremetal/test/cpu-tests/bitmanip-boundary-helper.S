    .section .text
    .align 2

    .macro unary name, encoding
    .globl \name
\name:
    .word \encoding
    ret
    .endm

    .macro binary name, encoding
    .globl \name
\name:
    .word \encoding
    ret
    .endm

    unary hw_clz,    0x60051513
    unary hw_ctz,    0x60151513
    unary hw_cpop,   0x60251513
    unary hw_sext_b, 0x60451513
    unary hw_sext_h, 0x60551513
    unary hw_rev8,   0x69855513
    unary hw_orc_b,  0x28755513

    binary hw_sh1add, 0x20b52533
    binary hw_sh2add, 0x20b54533
    binary hw_sh3add, 0x20b56533
    binary hw_clmul,  0x0ab51533
    binary hw_clmulr, 0x0ab52533
    binary hw_clmulh, 0x0ab53533
    binary hw_xnor,   0x40b54533
    binary hw_min,    0x0ab54533
    binary hw_pack,   0x08b54533
    binary hw_ror,    0x60b55533
    binary hw_bext,   0x48b55533
    binary hw_minu,   0x0ab55533
    binary hw_orn,    0x40b56533
    binary hw_max,    0x0ab56533
    binary hw_andn,   0x40b57533
    binary hw_maxu,   0x0ab57533
    binary hw_packh,  0x08b57533

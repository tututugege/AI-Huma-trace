#include "trap.h"
#include <stdint.h>

#define UNARY(name) extern uint32_t name(uint32_t value)
#define BINARY(name) extern uint32_t name(uint32_t lhs, uint32_t rhs)

UNARY(hw_clz);
UNARY(hw_ctz);
UNARY(hw_cpop);
UNARY(hw_sext_b);
UNARY(hw_sext_h);
UNARY(hw_rev8);
UNARY(hw_orc_b);
BINARY(hw_sh1add);
BINARY(hw_sh2add);
BINARY(hw_sh3add);
BINARY(hw_clmul);
BINARY(hw_clmulr);
BINARY(hw_clmulh);
BINARY(hw_xnor);
BINARY(hw_min);
BINARY(hw_pack);
BINARY(hw_ror);
BINARY(hw_bext);
BINARY(hw_minu);
BINARY(hw_orn);
BINARY(hw_max);
BINARY(hw_andn);
BINARY(hw_maxu);
BINARY(hw_packh);

static uint64_t carryless_product(uint32_t lhs, uint32_t rhs) {
  uint64_t product = 0;
  for (uint32_t i = 0; i < 32; i++) {
    if ((rhs >> i) & 1u) {
      product ^= (uint64_t)lhs << i;
    }
  }
  return product;
}

static uint32_t rotate_right(uint32_t value, uint32_t shift) {
  shift &= 31u;
  return shift == 0 ? value : (value >> shift) | (value << (32u - shift));
}

int main(void) {
  check(hw_clz(0) == 32u);
  check(hw_clz(0x00100000u) == 11u);
  check(hw_ctz(0) == 32u);
  check(hw_ctz(0x80000000u) == 31u);
  check(hw_cpop(0xf0f00001u) == 9u);
  check(hw_sext_b(0x00000080u) == 0xffffff80u);
  check(hw_sext_h(0x00008001u) == 0xffff8001u);
  check(hw_rev8(0x11223344u) == 0x44332211u);
  check(hw_orc_b(0x00120080u) == 0x00ff00ffu);

  check(hw_sh1add(3u, 7u) == 13u);
  check(hw_sh2add(3u, 7u) == 19u);
  check(hw_sh3add(3u, 7u) == 31u);

  const uint32_t lhs = 0x81234567u;
  const uint32_t rhs = 0xa5a55a5au;
  const uint64_t cl_product = carryless_product(lhs, rhs);
  check(hw_clmul(lhs, rhs) == (uint32_t)cl_product);
  check(hw_clmulh(lhs, rhs) == (uint32_t)(cl_product >> 32));
  check(hw_clmulr(lhs, rhs) == (uint32_t)(cl_product >> 31));

  check(hw_xnor(0x0f0f55aau, 0x3333ffffu) ==
        ~(0x0f0f55aau ^ 0x3333ffffu));
  check(hw_orn(0x0000ffffu, 0x00ff00ffu) == 0xff00ffffu);
  check(hw_andn(0xffff0000u, 0xff00ff00u) == 0x00ff0000u);
  check(hw_min(0x80000000u, 1u) == 0x80000000u);
  check(hw_max(0x80000000u, 1u) == 1u);
  check(hw_minu(0x80000000u, 1u) == 1u);
  check(hw_maxu(0x80000000u, 1u) == 0x80000000u);
  check(hw_pack(0xaaaabeefu, 0x12345678u) == 0x5678beefu);
  check(hw_packh(0xaaaabeefu, 0x12345678u) == 0x000078efu);
  check(hw_ror(0x12345678u, 8u) == rotate_right(0x12345678u, 8u));
  check(hw_bext(0x80000001u, 31u) == 1u);
  check(hw_bext(0x80000001u, 1u) == 0u);
  return 0;
}

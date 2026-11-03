// Decoder differential: Idu::decode vs the frozen reference over targeted + random instruction words.
#include "Idu.h"
#include "IduRef.h"
#include "IO.h"
#include <cstdio>
#include <cstring>
#include <random>
UopS uop_pool[UOP_POOL_SIZE];

int main(int argc, char **argv) {
  long n = argc > 1 ? atol(argv[1]) : 20000000;
  Idu a(nullptr);
  IduRef b(nullptr);
  std::mt19937_64 rng(argc > 2 ? atol(argv[2]) : 1);
  long bad = 0;
  auto check = [&](uint32_t inst) {
    Uop ua = {}, ub = {};
    ua.id = 0; ub.id = 1;
    memset(&uop_pool[0], 0, sizeof(UopS)); memset(&uop_pool[1], 0, sizeof(UopS));
    a.decode(ua, inst);
    b.decode(ub, inst);
    UopS sa = uop_pool[0], sb = uop_pool[1];
    sa.dbg.inst_idx = sb.dbg.inst_idx = 0;  // independent counters
    ua.id = ub.id = 0;
    if (memcmp(&sa, &sb, sizeof(UopS)) || memcmp(&ua, &ub, sizeof(Uop))) {
      if (bad++ < 10) printf("MISMATCH inst=%08x\n", inst);
    }
  };
  // every opcode x funct3 x funct7 with a few register/immediate patterns, plus all 12-bit csr/system fields
  for (uint32_t op = 0; op < 128; op++)
    for (uint32_t f3 = 0; f3 < 8; f3++)
      for (uint32_t f7 = 0; f7 < 128; f7++)
        for (uint32_t r = 0; r < 4; r++) {
          uint32_t rd = r & 1 ? 0 : 5, ra = r & 2 ? 1 : 7, rb = 9;
          check(op | rd << 7 | f3 << 12 | ra << 15 | rb << 20 | f7 << 25);
        }
  for (uint32_t csr = 0; csr < 4096; csr++)
    for (uint32_t f3 = 0; f3 < 8; f3++)
      for (uint32_t rd : {0, 3}) check(0x73 | rd << 7 | f3 << 12 | 1 << 15 | csr << 20);
  for (long i = 0; i < n; i++) check((uint32_t)rng());
  printf("idu_diff: %ld mismatches\n", bad);
  return bad != 0;
}

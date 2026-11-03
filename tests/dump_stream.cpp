// Prints the architectural instruction stream (pc, inst) of an image executed by the bridge core, no wrong path.
#include "../src/TraceBridge.h"
#include "refcpu_api.h"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
int main(int argc, char **argv) {
  std::ifstream f(argv[1], std::ios::binary);
  std::vector<uint8_t> img((std::istreambuf_iterator<char>(f)), {});
  RefCpuContext *c = refcpu_init(0x80000000u, 0x80000000u);
  std::memcpy(refcpu_get_ram_ptr(c), img.data(), img.size());
  tb::Bridge b(c, 0x80000000u, 0x80000000u);
  const unsigned long long n = argc > 2 ? strtoull(argv[2], 0, 10) : 100000;
  for (unsigned long long i = 0; i < n; i++) {
    RefCpuState s; refcpu_get_state(c, &s);
    uint32_t inst = refcpu_load_word(c, s.pc);
    if (inst == 0x00100073) { printf("[END] ebreak at 0x%08x after %llu\n", s.pc, i); break; }
    b.dispatch(i, s.pc, inst); b.commit(i);
    printf("[COMMIT] pc=0x%08x inst=0x%08x\n", s.pc, inst);
  }
}

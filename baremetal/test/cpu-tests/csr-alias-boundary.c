#include "trap.h"
#include <stdint.h>

#define SSTATUS_SIE (1u << 1)
#define SSTATUS_SPIE (1u << 5)
#define SSTATUS_MASK 0x800de133u
#define SIE_MASK 0x00000222u
#define MIE_MASK 0x00000aaau

extern uint32_t read_mscratch_asm(void);
extern void write_mscratch_asm(uint32_t value);
extern uint32_t swap_mscratch_asm(uint32_t value);
extern uint32_t set_mscratch_asm(uint32_t bits);
extern uint32_t clear_mscratch_asm(uint32_t bits);
extern uint32_t read_mstatus_asm(void);
extern uint32_t read_sstatus_asm(void);
extern void write_sstatus_asm(uint32_t value);
extern uint32_t read_mie_asm(void);
extern uint32_t read_sie_asm(void);
extern void write_mie_asm(uint32_t value);
extern void write_sie_asm(uint32_t value);

int main(void) {
  uint32_t old_mscratch = read_mscratch_asm();

  write_mscratch_asm(0x12340000u);
  check(read_mscratch_asm() == 0x12340000u);
  check(set_mscratch_asm(0x00005678u) == 0x12340000u);
  check(read_mscratch_asm() == 0x12345678u);
  check(clear_mscratch_asm(0x00300070u) == 0x12345678u);
  check(read_mscratch_asm() == 0x12045608u);
  check(swap_mscratch_asm(0xa5a55a5au) == 0x12045608u);
  check(read_mscratch_asm() == 0xa5a55a5au);
  write_mscratch_asm(old_mscratch);

  /* sstatus is a masked alias of mstatus. */
  uint32_t old_sstatus = read_sstatus_asm();
  uint32_t toggled_sstatus = old_sstatus ^ (SSTATUS_SIE | SSTATUS_SPIE);
  write_sstatus_asm(toggled_sstatus);
  check((read_sstatus_asm() & SSTATUS_MASK) ==
        (toggled_sstatus & SSTATUS_MASK));
  check((read_mstatus_asm() & SSTATUS_MASK) ==
        (toggled_sstatus & SSTATUS_MASK));
  write_sstatus_asm(old_sstatus);

  /* sie writes only supervisor-visible bits and preserves machine-only mie. */
  uint32_t old_mie = read_mie_asm();
  write_mie_asm(MIE_MASK);
  check(read_mie_asm() == MIE_MASK);
  check(read_sie_asm() == SIE_MASK);
  write_sie_asm(0x022u);
  check(read_sie_asm() == 0x022u);
  check(read_mie_asm() == ((MIE_MASK & ~SIE_MASK) | 0x022u));
  write_mie_asm(old_mie);

  return 0;
}

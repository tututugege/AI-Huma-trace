#include "trap.h"
#include <stdint.h>

extern void install_precise_trap_vector(void);
extern void store_ecall_store(volatile uint32_t *older_addr,
                              uint32_t older_value,
                              volatile uint32_t *younger_addr,
                              uint32_t younger_value);
extern char precise_ecall_site[];

volatile uint32_t trap_count;
volatile uint32_t last_mcause;
volatile uint32_t last_mepc;
static volatile uint32_t older_word;
static volatile uint32_t younger_word;

int main(void) {
  older_word = 0;
  younger_word = 0x11223344u;
  trap_count = 0;
  install_precise_trap_vector();

  store_ecall_store(&older_word, 0xa5a55a5au, &younger_word, 0xdeadbeefu);

  check(trap_count == 1);
  check(last_mcause == 11u);
  check(last_mepc == (uint32_t)precise_ecall_site);
  check(older_word == 0xa5a55a5au);
  check(younger_word == 0x11223344u);
  return 0;
}

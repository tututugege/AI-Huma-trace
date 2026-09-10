#include "trap.h"
#include <stdint.h>

extern void install_illegal_trap_vector(void);
extern void trigger_illegal_instruction(void);
extern char illegal_instruction_site[];

volatile uint32_t illegal_trap_count;
volatile uint32_t illegal_mcause;
volatile uint32_t illegal_mepc;
volatile uint32_t illegal_mtval;

int main(void) {
  illegal_trap_count = 0;
  illegal_mcause = 0xffffffffu;
  illegal_mepc = 0;
  illegal_mtval = 0;

  install_illegal_trap_vector();
  trigger_illegal_instruction();

  check(illegal_trap_count == 1u);
  check(illegal_mcause == 2u);
  check(illegal_mepc == (uint32_t)illegal_instruction_site);
  check(illegal_mtval == 0xffffffffu);
  return 0;
}

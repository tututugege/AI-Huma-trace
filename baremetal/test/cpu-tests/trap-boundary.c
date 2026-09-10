#include "trap.h"
#include <stdint.h>

#define MSTATUS_MIE (1u << 3)
#define MSTATUS_MPIE (1u << 7)
#define MSTATUS_MPP (3u << 11)

extern void install_trap_vector(void);
extern void trigger_ecall(void);
extern uint32_t read_mstatus_asm(void);
extern void set_mstatus_bits_asm(uint32_t mask);
extern void clear_mstatus_bits_asm(uint32_t mask);
extern char ecall_site[];

volatile uint32_t trap_count;
volatile uint32_t last_mcause;
volatile uint32_t last_mepc;
volatile uint32_t last_mtval;
volatile uint32_t trap_mstatus;

static void reset_trap_record(void) {
  last_mcause = 0xffffffffu;
  last_mepc = 0;
  last_mtval = 0xffffffffu;
  trap_mstatus = 0;
}

static void check_trap_entry_state(uint32_t cause, uint32_t epc,
                                   uint32_t tval) {
  check(last_mcause == cause);
  check(last_mepc == epc);
  check(last_mtval == tval);
  check((trap_mstatus & MSTATUS_MIE) == 0);
  check((trap_mstatus & MSTATUS_MPIE) != 0);
  check((trap_mstatus & MSTATUS_MPP) == MSTATUS_MPP);
}

static void check_mret_state(void) {
  uint32_t mstatus = read_mstatus_asm();
  check((mstatus & MSTATUS_MIE) != 0);
  check((mstatus & MSTATUS_MPIE) != 0);
  check((mstatus & MSTATUS_MPP) == 0);
}

int main(void) {
  install_trap_vector();
  trap_count = 0;

  /* Force a visible MIE -> MPIE transition on synchronous trap entry. */
  set_mstatus_bits_asm(MSTATUS_MIE);

  reset_trap_record();
  trigger_ecall();
  check(trap_count == 1);
  check_trap_entry_state(11u, (uint32_t)ecall_site, 0u);
  check_mret_state();

  /* Re-entering the same trap verifies that mret fully restored trap state. */
  reset_trap_record();
  trigger_ecall();
  check(trap_count == 2);
  check_trap_entry_state(11u, (uint32_t)ecall_site, 0u);
  check_mret_state();

  clear_mstatus_bits_asm(MSTATUS_MIE);
  return 0;
}

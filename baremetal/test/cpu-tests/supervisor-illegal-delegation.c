#include "trap.h"
#include <stdint.h>

#define MSTATUS_MPP (3u << 11)

extern void run_supervisor_illegal_delegation(void);
extern char supervisor_illegal_site[];
extern char supervisor_exit_ecall_site[];

volatile uint32_t delegated_illegal_count;
volatile uint32_t delegated_scause;
volatile uint32_t delegated_sepc;
volatile uint32_t delegated_stval;
volatile uint32_t delegated_resume_marker;
volatile uint32_t delegated_exit_mcause;
volatile uint32_t delegated_exit_mepc;
volatile uint32_t delegated_exit_mstatus;

int main(void) {
  delegated_illegal_count = 0;
  delegated_resume_marker = 0;
  run_supervisor_illegal_delegation();

  check(delegated_illegal_count == 1u);
  check(delegated_scause == 2u);
  check(delegated_sepc == (uint32_t)supervisor_illegal_site);
  check(delegated_stval == 0xffffffffu);
  check(delegated_resume_marker == 1u);

  /* The following S-mode ECALL is deliberately not delegated. */
  check(delegated_exit_mcause == 9u);
  check(delegated_exit_mepc == (uint32_t)supervisor_exit_ecall_site);
  check((delegated_exit_mstatus & MSTATUS_MPP) == (1u << 11));
  return 0;
}

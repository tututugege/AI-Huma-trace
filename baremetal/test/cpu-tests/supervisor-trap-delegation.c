#include "trap.h"
#include <stdint.h>

#define SSTATUS_SIE  (1u << 1)
#define SSTATUS_SPIE (1u << 5)
#define SSTATUS_SPP  (1u << 8)
#define MSTATUS_MPP  (3u << 11)

extern void run_supervisor_trap_delegation(void);
extern char supervisor_ecall_site[];
extern char user_ecall_site[];

volatile uint32_t supervisor_marker;
volatile uint32_t supervisor_trap_count;
volatile uint32_t supervisor_scause;
volatile uint32_t supervisor_sepc;
volatile uint32_t supervisor_sstatus;
volatile uint32_t machine_trap_count;
volatile uint32_t machine_mcause;
volatile uint32_t machine_mepc;
volatile uint32_t machine_mstatus;
volatile uint32_t user_marker;
volatile uint32_t machine_resume_marker;

int main(void) {
  supervisor_marker = 0;
  supervisor_trap_count = 0;
  machine_trap_count = 0;
  user_marker = 0;
  machine_resume_marker = 0;

  run_supervisor_trap_delegation();

  check(supervisor_marker == 2u);
  check(supervisor_trap_count == 1u);
  check(supervisor_scause == 9u);
  check(supervisor_sepc == (uint32_t)supervisor_ecall_site);
  check((supervisor_sstatus & SSTATUS_SIE) == 0u);
  check((supervisor_sstatus & SSTATUS_SPIE) != 0u);
  check((supervisor_sstatus & SSTATUS_SPP) != 0u);

  check(user_marker == 1u);
  check(machine_trap_count == 1u);
  check(machine_mcause == 8u);
  check(machine_mepc == (uint32_t)user_ecall_site);
  check((machine_mstatus & MSTATUS_MPP) == 0u);
  check(machine_resume_marker == 1u);
  return 0;
}

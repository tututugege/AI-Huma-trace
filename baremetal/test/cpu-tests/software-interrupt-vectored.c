#include "trap.h"
#include <stdint.h>

#define M_SOFTWARE_CAUSE 0x80000003u
#define S_SOFTWARE_CAUSE 0x80000001u

extern void run_software_interrupt_test(void);

volatile uint32_t machine_software_count;
volatile uint32_t machine_software_mcause;
volatile uint32_t machine_software_mepc;
volatile uint32_t machine_software_slot;
volatile uint32_t supervisor_software_count;
volatile uint32_t supervisor_software_scause;
volatile uint32_t supervisor_software_sepc;
volatile uint32_t supervisor_software_slot;
volatile uint32_t software_exit_mcause;

int main(void) {
  machine_software_slot = UINT32_MAX;
  supervisor_software_slot = UINT32_MAX;
  software_exit_mcause = UINT32_MAX;

  run_software_interrupt_test();

  check(machine_software_count == 1u);
  check(machine_software_mcause == M_SOFTWARE_CAUSE);
  check(machine_software_mepc != 0u);
  check(machine_software_slot == 3u);
  check(supervisor_software_count == 1u);
  check(supervisor_software_scause == S_SOFTWARE_CAUSE);
  check(supervisor_software_sepc != 0u);
  check(supervisor_software_slot == 1u);
  check(software_exit_mcause == 9u);
  return 0;
}

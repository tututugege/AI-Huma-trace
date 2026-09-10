#include "trap.h"
#include <stdint.h>

#define UART_BASE 0x10000000u
#define UART_IIR (UART_BASE + 2u)
#define XPS_INTC_BASE 0x1fb00000u
#define XPS_IPR (XPS_INTC_BASE + 0x04u)
#define XPS_IVR (XPS_INTC_BASE + 0x18u)
#define UART_IRQ_MASK (1u << 2)
#define SEXT_INTERRUPT_CAUSE 0x80000009u

extern void run_supervisor_external_interrupt(void);

volatile uint32_t external_trap_count;
volatile uint32_t external_scause;
volatile uint32_t external_sepc;
volatile uint32_t external_sstatus;
volatile uint32_t external_vector_slot;
volatile uint32_t external_xps_ipr;
volatile uint32_t external_xps_ivr;
volatile uint32_t external_uart_iir;
volatile uint32_t external_exit_mcause;

static inline uint32_t mmio_read32(uint32_t addr) {
  uint32_t value;
  asm volatile("lw %0, 0(%1)" : "=r"(value) : "r"(addr));
  return value;
}

int main(void) {
  external_trap_count = 0;
  external_vector_slot = 0xffffffffu;
  external_exit_mcause = 0xffffffffu;

  run_supervisor_external_interrupt();

  check(external_trap_count == 1u);
  check(external_scause == SEXT_INTERRUPT_CAUSE);
  check(external_sepc != 0u);
  check(external_vector_slot == 9u);
  check((external_xps_ipr & UART_IRQ_MASK) != 0u);
  check(external_xps_ivr == 2u);
  check((external_uart_iir & 0x0fu) == 2u);
  check(external_exit_mcause == 9u);
  check(mmio_read32(XPS_IPR) == 0u);
  (void)mmio_read32(UART_IIR & ~3u);
  return 0;
}

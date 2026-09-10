#include "trap.h"
#include <stdint.h>

#define MSTATUS_MIE (1u << 3)
#define TIMER_LOW_ADDR 0x1fd0e000u
#define TIMER_HIGH_ADDR 0x1fd0e004u
#define TIMERCMP_LOW_ADDR 0x1fd0e008u
#define TIMERCMP_HIGH_ADDR 0x1fd0e00cu
#define MTIMER_INTERRUPT_CAUSE 0x80000007u

extern void install_vectored_trap(void);
extern void enable_timer_interrupt(void);
extern void clear_mstatus_mie(void);

volatile uint32_t trap_count;
volatile uint32_t last_mcause;
volatile uint32_t last_mepc;
volatile uint32_t vector_slot;

static inline uint32_t mmio_read32(uint32_t addr) {
  uint32_t value;
  asm volatile("lw %0, 0(%1)" : "=r"(value) : "r"(addr));
  return value;
}

static inline void mmio_write32(uint32_t addr, uint32_t value) {
  asm volatile("sw %0, 0(%1)" : : "r"(value), "r"(addr) : "memory");
}

static uint32_t read_timer_low_stable(void) {
  uint32_t hi1;
  uint32_t hi2;
  uint32_t lo;
  do {
    hi1 = mmio_read32(TIMER_HIGH_ADDR);
    lo = mmio_read32(TIMER_LOW_ADDR);
    hi2 = mmio_read32(TIMER_HIGH_ADDR);
  } while (hi1 != hi2);
  return lo;
}

static void write_timercmp(uint64_t value) {
  mmio_write32(TIMERCMP_LOW_ADDR, (uint32_t)value);
  mmio_write32(TIMERCMP_HIGH_ADDR, (uint32_t)(value >> 32));
}

int main(void) {
  trap_count = 0;
  last_mcause = 0;
  last_mepc = 0;
  vector_slot = 0;

  install_vectored_trap();
  write_timercmp(UINT64_MAX);
  write_timercmp((uint64_t)(read_timer_low_stable() + 2000u));
  enable_timer_interrupt();

  uint32_t timeout = 200000u;
  while (trap_count == 0 && timeout-- != 0) {
    asm volatile("" ::: "memory");
  }

  check(timeout != 0);
  check(trap_count == 1);
  check(last_mcause == MTIMER_INTERRUPT_CAUSE);
  check(last_mepc != 0);
  check(vector_slot == 7u);

  clear_mstatus_mie();
  write_timercmp(UINT64_MAX);
  return 0;
}

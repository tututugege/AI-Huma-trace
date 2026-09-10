#include "trap.h"
#include <stdint.h>

#define TIMER_LOW_ADDR 0x1fd0e000u
#define TIMER_HIGH_ADDR 0x1fd0e004u
#define TIMERCMP_LOW_ADDR 0x1fd0e008u
#define TIMERCMP_HIGH_ADDR 0x1fd0e00cu

#define MSTATUS_MIE (1u << 3)
#define MIP_MTIP (1u << 7)
#define MTIMER_INTERRUPT_CAUSE 0x80000007u

extern void install_timer_trap_vector(void);
extern uint32_t read_mstatus_asm(void);
extern uint32_t read_mie_asm(void);
extern uint32_t read_mip_asm(void);
extern void write_mie_asm(uint32_t value);
extern void set_mstatus_bits_asm(uint32_t mask);
extern void clear_mstatus_bits_asm(uint32_t mask);

volatile uint32_t trap_count;
volatile uint32_t last_mcause;
volatile uint32_t last_mepc;
volatile uint32_t last_mip;
volatile uint32_t trap_mstatus;

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

static void spin_cycles(uint32_t count) {
  while (count-- != 0) {
    asm volatile("nop");
  }
}

int main(void) {
  uint32_t timeout;

  install_timer_trap_vector();
  trap_count = 0;
  write_timercmp(UINT64_MAX);

  /* Gate 1: MTIP may become pending, but global MIE keeps it from trapping. */
  write_mie_asm(read_mie_asm() | MIP_MTIP);
  clear_mstatus_bits_asm(MSTATUS_MIE);
  write_timercmp((uint64_t)(read_timer_low_stable() + 2000u));

  timeout = 200000u;
  while ((read_mip_asm() & MIP_MTIP) == 0 && timeout-- != 0) {
    asm volatile("" ::: "memory");
  }
  check(timeout != 0);
  check(trap_count == 0);

  /* Gate 2: global MIE alone is insufficient while mie.MTIE is clear. */
  write_mie_asm(read_mie_asm() & ~MIP_MTIP);
  set_mstatus_bits_asm(MSTATUS_MIE);
  spin_cycles(512u);
  check((read_mip_asm() & MIP_MTIP) != 0);
  check(trap_count == 0);

  /* Opening both gates must deliver exactly one precise timer interrupt. */
  write_mie_asm(read_mie_asm() | MIP_MTIP);
  timeout = 200000u;
  while (trap_count == 0 && timeout-- != 0) {
    asm volatile("" ::: "memory");
  }
  check(timeout != 0);
  check(trap_count == 1);
  check(last_mcause == MTIMER_INTERRUPT_CAUSE);
  check((last_mip & MIP_MTIP) != 0);
  check((trap_mstatus & MSTATUS_MIE) == 0);
  check(last_mepc != 0);

  clear_mstatus_bits_asm(MSTATUS_MIE);
  write_timercmp(UINT64_MAX);
  return 0;
}

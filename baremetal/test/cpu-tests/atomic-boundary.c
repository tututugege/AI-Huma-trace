#include "trap.h"
#include <stdint.h>

static volatile uint32_t word_a __attribute__((aligned(4)));
static volatile uint32_t word_b __attribute__((aligned(4)));

static inline uint32_t lr_w(volatile uint32_t *addr) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x08, %0, %1, x0"
               : "=r"(old)
               : "r"(addr)
               : "memory");
  return old;
}

static inline uint32_t sc_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t status;
  asm volatile(".insn r 0x2f, 2, 0x0c, %0, %1, %2"
               : "=r"(status)
               : "r"(addr), "r"(value)
               : "memory");
  return status;
}

static inline uint32_t amoswap_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x04, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amoadd_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x00, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amoxor_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x10, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amoand_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x30, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amoor_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x20, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amomin_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x40, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amomax_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x50, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amominu_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x60, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

static inline uint32_t amomaxu_w(volatile uint32_t *addr, uint32_t value) {
  uint32_t old;
  asm volatile(".insn r 0x2f, 2, 0x70, %0, %1, %2"
               : "=r"(old)
               : "r"(addr), "r"(value)
               : "memory");
  return old;
}

int main(void) {
  word_a = 0x11223344u;
  word_b = 0x55667788u;

  /* Matching LR/SC succeeds and returns the pre-store value from LR. */
  check(lr_w(&word_a) == 0x11223344u);
  check(sc_w(&word_a, 0xa5a5a5a5u) == 0u);
  check(word_a == 0xa5a5a5a5u);

  /* SC consumes the reservation: a second SC must fail without writing. */
  check(sc_w(&word_a, 0xdeadbeefu) != 0u);
  check(word_a == 0xa5a5a5a5u);

  /* A reservation for A cannot authorize an SC to a different address B. */
  check(lr_w(&word_a) == 0xa5a5a5a5u);
  check(sc_w(&word_b, 0xcafebabeu) != 0u);
  check(word_b == 0x55667788u);

  /* AMOs return the old value and update memory atomically. */
  check(amoswap_w(&word_a, 0xffffffffu) == 0xa5a5a5a5u);
  check(word_a == 0xffffffffu);
  check(amoadd_w(&word_a, 1u) == 0xffffffffu);
  check(word_a == 0u); /* unsigned wraparound */

  word_a = 0x0f0f55aau;
  check(amoxor_w(&word_a, 0x3333ffffu) == 0x0f0f55aau);
  check(word_a == (0x0f0f55aau ^ 0x3333ffffu));
  check(amoand_w(&word_a, 0x00ffff00u) ==
        (0x0f0f55aau ^ 0x3333ffffu));
  check(word_a == ((0x0f0f55aau ^ 0x3333ffffu) & 0x00ffff00u));
  check(amoor_w(&word_a, 0x80000001u) ==
        ((0x0f0f55aau ^ 0x3333ffffu) & 0x00ffff00u));
  check(word_a ==
        (((0x0f0f55aau ^ 0x3333ffffu) & 0x00ffff00u) | 0x80000001u));

  word_a = 0x80000000u;
  check(amomin_w(&word_a, 1u) == 0x80000000u);
  check(word_a == 0x80000000u); /* signed INT_MIN remains minimum */
  word_a = 7u;
  check(amomin_w(&word_a, 0xffffffffu) == 7u);
  check(word_a == 0xffffffffu);
  word_a = 0x80000000u;
  check(amomax_w(&word_a, 1u) == 0x80000000u);
  check(word_a == 1u);
  word_a = 7u;
  check(amomax_w(&word_a, 0xffffffffu) == 7u);
  check(word_a == 7u);
  word_a = 0xffffffffu;
  check(amominu_w(&word_a, 1u) == 0xffffffffu);
  check(word_a == 1u);
  word_a = 0u;
  check(amominu_w(&word_a, 0xffffffffu) == 0u);
  check(word_a == 0u);
  word_a = 0x80000000u;
  check(amomaxu_w(&word_a, 0xffffffffu) == 0x80000000u);
  check(word_a == 0xffffffffu);
  check(amomaxu_w(&word_a, 1u) == 0xffffffffu);
  check(word_a == 0xffffffffu);

  return 0;
}

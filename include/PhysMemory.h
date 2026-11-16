#pragma once

#include <cstddef>
#include <cstdint>

// ============================================================
// physical memory
//
// address map:
// - SDRAM: [0x40000000, 0x50000000), contiguous
// - DDR: [0x80000000, 0x100000000), contiguous
// - IO: other 32-bit physical addresses, sparse word storage (word aligned)
//
// p_memory points at the RAM window base (physical 0x8000_0000);
// use pmem_read / pmem_write rather than indexing p_memory[].
// ============================================================

constexpr uint32_t PMEM_SDRAM_BASE = 0x40000000u;
constexpr uint32_t PMEM_SDRAM_SIZE = 0x10000000u;
constexpr uint32_t PMEM_RAM_BASE = 0x80000000u;

extern uint32_t *p_memory;

struct RefCpuContext;
bool pmem_init();
RefCpuContext *pmem_core_ctx();  // the core that owns the RAM
void pmem_release();
void pmem_clear_all();

bool pmem_is_ram_addr(uint32_t paddr, uint32_t size = 4u);
bool pmem_is_sdram_addr(uint32_t paddr, uint32_t size = 4u);
uint32_t pmem_read(uint32_t paddr);
void pmem_write(uint32_t paddr, uint32_t data);

// bulk RAM copy; paddr must be inside the RAM window

// the RAM base pointer (physical PMEM_RAM_BASE)
uint32_t *pmem_ram_ptr();
uint32_t *pmem_sdram_ptr();

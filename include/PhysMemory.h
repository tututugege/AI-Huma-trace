#pragma once

#include <cstddef>
#include <cstdint>

// ============================================================
// 物理内存统一接口
//
// 约定地址空间：
// - SDRAM: [0x40000000, 0x50000000) 独立连续存储
// - DDR: [0x80000000, 0x100000000) 独立连续存储
// - IO : 其余 32-bit 物理地址，离散字存储（按 word 对齐）
//
// p_memory 指向 RAM 窗口基址（对应物理地址 0x8000_0000）。
// 代码应优先使用 pmem_read / pmem_write，而非直接 p_memory[] 下标。
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

// 批量 RAM 拷贝接口，paddr 必须落在 RAM 窗口内。
void pmem_memcpy_to_ram(uint32_t ram_paddr, const void *src, size_t len);
void pmem_memcpy_from_ram(void *dst, uint32_t ram_paddr, size_t len);

// 返回 RAM 基址指针（对应物理地址 PMEM_RAM_BASE）。
uint32_t *pmem_ram_ptr();
uint32_t *pmem_sdram_ptr();

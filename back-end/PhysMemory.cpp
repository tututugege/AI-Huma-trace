#include "PhysMemory.h"
#include "config.h"
#include "refcpu_api.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_map>

uint32_t *p_memory = nullptr;

// The functional memory is the core's own RAM: one copy, shared by the loader, the model's
// instruction/page-table reads and the trace bridge (only the IO window is kept here).
namespace {
RefCpuContext *g_core = nullptr;
uint32_t *g_sdram_memory = nullptr;
std::unordered_map<uint32_t, uint32_t> g_io_words;

[[noreturn]] void pmem_fatal(const char *op, uint32_t addr, uint64_t size) {
  std::fprintf(stderr,
               "[PhysMemory] %s failed: addr=0x%08x size=%llu\n", op, addr,
               static_cast<unsigned long long>(size));
  std::abort();
}

inline void pmem_require_ready(const char *op) {
  if (p_memory != nullptr && g_sdram_memory != nullptr) {
    return;
  }
  std::fprintf(stderr,
               "[PhysMemory] %s failed: RAM backend not initialized\n", op);
  std::abort();
}
} // namespace

bool pmem_init() {
  if (p_memory != nullptr && g_sdram_memory != nullptr) {
    pmem_clear_all();
    return true;
  }
  g_core = refcpu_init(0, static_cast<uint32_t>(RAM_SIZE));
  if (g_core == nullptr) {
    return false;
  }
  refcpu_set_uart_print(g_core, false);
  p_memory = refcpu_get_ram_ptr(g_core);
  g_sdram_memory = refcpu_get_sdram_ptr(g_core);
  g_io_words.clear();
  return p_memory != nullptr && g_sdram_memory != nullptr;
}

void pmem_release() {
  if (g_core != nullptr) {
    refcpu_destroy(g_core);
    g_core = nullptr;
  }
  p_memory = nullptr;
  g_sdram_memory = nullptr;
  g_io_words.clear();
}

RefCpuContext *pmem_core_ctx() { return g_core; }

namespace {
// Zero a buffer page by page, touching only pages that are not zero already: a freshly calloc'ed 2 GiB stays unmapped
// (the result is the same as memset, the resident size is not).
void clear_sparse(void *base, size_t bytes) {
  constexpr size_t kPage = 4096;
  auto *p = static_cast<uint8_t *>(base);
  static const uint8_t zeros[kPage] = {};
  for (size_t off = 0; off < bytes; off += kPage) {
    const size_t n = bytes - off < kPage ? bytes - off : kPage;
    if (std::memcmp(p + off, zeros, n) != 0) std::memset(p + off, 0, n);
  }
}
}  // namespace

void pmem_clear_all() {
  if (p_memory != nullptr) {
    clear_sparse(p_memory, RAM_SIZE);
  }
  if (g_sdram_memory != nullptr) {
    clear_sparse(g_sdram_memory, PMEM_SDRAM_SIZE);
  }
  g_io_words.clear();
}

namespace {
bool pmem_range_within(uint32_t paddr, uint32_t size, uint64_t base,
                       uint64_t span) {
  if (size == 0 || static_cast<uint64_t>(paddr) < base) {
    return false;
  }
  const uint64_t end =
      static_cast<uint64_t>(paddr) + static_cast<uint64_t>(size) - 1u;
  return end < base + span;
}

bool pmem_is_ddr_addr(uint32_t paddr, uint32_t size) {
  return pmem_range_within(paddr, size, PMEM_RAM_BASE, RAM_SIZE);
}
} // namespace

bool pmem_is_sdram_addr(uint32_t paddr, uint32_t size) {
  return pmem_range_within(paddr, size, PMEM_SDRAM_BASE, PMEM_SDRAM_SIZE);
}

bool pmem_is_ram_addr(uint32_t paddr, uint32_t size) {
  return pmem_is_sdram_addr(paddr, size) || pmem_is_ddr_addr(paddr, size);
}

uint32_t pmem_read(uint32_t paddr) {
  pmem_require_ready("read");
  const uint32_t word_addr = paddr & ~0x3u;
  if (pmem_is_sdram_addr(word_addr, 4u)) {
    return g_sdram_memory[(word_addr - PMEM_SDRAM_BASE) >> 2];
  }
  if (pmem_is_ddr_addr(word_addr, 4u)) {
    return p_memory[(word_addr - PMEM_RAM_BASE) >> 2];
  }
  auto it = g_io_words.find(word_addr);
  return (it == g_io_words.end()) ? 0u : it->second;
}

void pmem_write(uint32_t paddr, uint32_t data) {
  pmem_require_ready("write");
  const uint32_t word_addr = paddr & ~0x3u;
  if (pmem_is_sdram_addr(word_addr, 4u)) {
    g_sdram_memory[(word_addr - PMEM_SDRAM_BASE) >> 2] = data;
    return;
  }
  if (pmem_is_ddr_addr(word_addr, 4u)) {
    p_memory[(word_addr - PMEM_RAM_BASE) >> 2] = data;
    return;
  }
  if (data == 0u) {
    g_io_words.erase(word_addr);
  } else {
    g_io_words[word_addr] = data;
  }
}

void pmem_memcpy_to_ram(uint32_t ram_paddr, const void *src, size_t len) {
  pmem_require_ready("memcpy_to_ram");
  if (len == 0) {
    return;
  }
  if (len > UINT32_MAX) {
    pmem_fatal("memcpy_to_ram", ram_paddr, len);
  }
  if (!pmem_is_ram_addr(ram_paddr, static_cast<uint32_t>(len))) {
    pmem_fatal("memcpy_to_ram", ram_paddr, len);
  }
  if (pmem_is_sdram_addr(ram_paddr, static_cast<uint32_t>(len))) {
    const size_t off = static_cast<size_t>(ram_paddr - PMEM_SDRAM_BASE);
    std::memcpy(reinterpret_cast<uint8_t *>(g_sdram_memory) + off, src, len);
    return;
  }
  const size_t off = static_cast<size_t>(ram_paddr - PMEM_RAM_BASE);
  std::memcpy(reinterpret_cast<uint8_t *>(p_memory) + off, src, len);
}

void pmem_memcpy_from_ram(void *dst, uint32_t ram_paddr, size_t len) {
  pmem_require_ready("memcpy_from_ram");
  if (len == 0) {
    return;
  }
  if (len > UINT32_MAX) {
    pmem_fatal("memcpy_from_ram", ram_paddr, len);
  }
  if (!pmem_is_ram_addr(ram_paddr, static_cast<uint32_t>(len))) {
    pmem_fatal("memcpy_from_ram", ram_paddr, len);
  }
  if (pmem_is_sdram_addr(ram_paddr, static_cast<uint32_t>(len))) {
    const size_t off = static_cast<size_t>(ram_paddr - PMEM_SDRAM_BASE);
    std::memcpy(dst, reinterpret_cast<const uint8_t *>(g_sdram_memory) + off,
                len);
    return;
  }
  const size_t off = static_cast<size_t>(ram_paddr - PMEM_RAM_BASE);
  std::memcpy(dst, reinterpret_cast<const uint8_t *>(p_memory) + off, len);
}

uint32_t *pmem_ram_ptr() {
  return p_memory;
}

uint32_t *pmem_sdram_ptr() {
  return g_sdram_memory;
}

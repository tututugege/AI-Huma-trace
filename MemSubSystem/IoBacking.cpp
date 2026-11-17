#include "IoBacking.h"
#include "mmio_address_map.h"
#include "util.h"

#include <unordered_map>

namespace {
std::unordered_map<uint32_t, uint32_t> g_words;   // word address -> value (absent: 0)
}

uint32_t io_read(uint32_t paddr) {
  const uint32_t a = paddr & ~0x3u;
  Assert(!is_ram_addr(a, 4u) && "io_read of a RAM address (the RAM is the function model's)");
  const auto it = g_words.find(a);
  return it == g_words.end() ? 0u : it->second;
}

void io_write(uint32_t paddr, uint32_t data) {
  const uint32_t a = paddr & ~0x3u;
  Assert(!is_ram_addr(a, 4u) && "io_write of a RAM address (the RAM is the function model's)");
  if (data == 0u) g_words.erase(a);
  else g_words[a] = data;
}

void io_clear() { g_words.clear(); }

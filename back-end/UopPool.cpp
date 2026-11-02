#include "types.h"
#include <cstdio>
#include <cstdlib>

UopS uop_pool[UOP_POOL_SIZE];

static uint16_t g_next = 1;

uint16_t uop_alloc() {
  const uint16_t id = g_next;
  g_next = static_cast<uint16_t>(g_next + UOP_BLOCK + UOP_BLOCK > UOP_POOL_SIZE ? 1 : g_next + UOP_BLOCK);
  std::memset(&uop_pool[id], 0, sizeof(UopS) * UOP_BLOCK);
  return id;
}

// Entry 0 is the shared null entry of default-constructed handles: nobody may write static fields through it.
static struct NullEntryCheck {
  ~NullEntryCheck() {
    static const UopS zero = {};
    if (std::memcmp(&uop_pool[0], &zero, sizeof(UopS)) != 0) {
      std::fprintf(stderr, "[UOP_POOL] the shared null entry was written (a static field was set on a uop without an id)\n");
      std::abort();
    }
  }
} g_null_entry_check;

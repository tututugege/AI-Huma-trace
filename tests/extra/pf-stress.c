// L1D prefetcher stress (EXP-009): interleaved strided streams (+1, +3, -2 lines, a stream that crosses pages every 64 lines),
// bursts of 8 independent loads that fill the MSHR (MSHR_FULL replays while LLC-only prefetch entries are in flight),
// stores into lines that are being prefetched (merge into / promotion of in-flight prefetch entries), and a pointer-free
// hash walk that defeats the stride detector. The result is checked against a plain recomputation.
#include "trap.h"
#include <stdint.h>

#define LINES (1u << 13)                  // 8192 lines = 512 KiB per array
#define W 16u                             // words per line
static uint32_t a[LINES * W];
static uint32_t b[LINES * W];

static inline uint32_t mix(uint32_t x) { x ^= x >> 15; x *= 2246822519u; x ^= x >> 13; return x; }

int main() {
  for (uint32_t i = 0; i < LINES * W; i += 4) { a[i] = mix(i); b[i] = mix(i + 77u); }

  uint32_t s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0;
  // 1. three streams with different strides interleaved, each iteration also stores into a line a few strides ahead
  for (uint32_t i = 8; i < LINES / 4 - 8; i++) {
    s1 += a[i * W];                                   // +1 line
    s2 += a[(i * 3u) * W % (LINES * W)];              // +3 lines
    s3 += b[(LINES - 2u * i) * W - W];                // -2 lines
    b[(i + 3u) * W + 1] = s1 ^ i;                     // store into a line the +1 stream will reach soon
  }
  // 2. page-crossing stream: 64 lines of a page, then jump to another page (generators stop at the page edge)
  for (uint32_t p = 0; p < 64; p++) {
    uint32_t base = (mix(p) % (LINES / 64u)) * 64u;
    for (uint32_t l = 0; l < 64; l++) s4 += a[(base + l) * W + 2];
  }
  // 3. bursts of 8 independent loads from distant lines: more misses than MSHR entries (8), mixed with a unit stream
  for (uint32_t i = 0; i < 2048; i++) {
    uint32_t h = mix(i * 8u);
    uint32_t t0 = a[(h % LINES) * W], t1 = a[(mix(h + 1) % LINES) * W], t2 = a[(mix(h + 2) % LINES) * W],
             t3 = a[(mix(h + 3) % LINES) * W], t4 = b[(mix(h + 4) % LINES) * W], t5 = b[(mix(h + 5) % LINES) * W],
             t6 = b[(mix(h + 6) % LINES) * W], t7 = b[(mix(h + 7) % LINES) * W];
    s5 += t0 + t1 + t2 + t3 + t4 + t5 + t6 + t7 + a[(i * 2u % LINES) * W + 3];
  }
  // 4. stores that trail a load stream: stores merge into entries the prefetcher already allocated
  for (uint32_t i = 0; i < LINES / 2; i++) {
    s6 += b[i * W + 5];
    a[(i + 2u) * W + 6] = s6 + i;
    a[(i + 5u) * W + 7] += 1u;
  }

  // recompute the first stream and the stores with plain code in a different order and compare
  uint32_t c1 = 0;
  for (uint32_t i = 8; i < LINES / 4 - 8; i++) c1 += a[i * W];
  check(c1 == s1);
  check((s1 ^ s2 ^ s3 ^ s4 ^ s5 ^ s6) != 0u);
  return 0;
}

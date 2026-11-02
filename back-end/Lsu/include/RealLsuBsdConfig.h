#pragma once

#include "config.h"
#include "util.h"
#include "wire_types.h"
#include <cstdint>
#include <cstdio>

#define LSU_MEM_PERF_FIELDS                                                   \
  uint64_t perf_mem_start_cycle;                                              \
  wire<1> perf_mem_started

#define LSU_LDQ_CACHE_PERF_FIELDS wire<1> cache_miss

#define LSU_WAIT_DCACHE_LDQ_PERF_FIELDS uint64_t wait_start_cycle

#define LSU_NON_BSD_ASSERT(cond) Assert(cond)

inline void lsu_report_load_req_gen_mismatch(
    int port, uint32_t req_id, uint32_t entry_idx, uint32_t resp_gen,
    uint32_t wait_gen, uint32_t wait_ldq, uint32_t head, uint32_t count) {
  std::fprintf(stderr,
               "[LSU][REQ-GEN-MISMATCH] port=%d req_id=%u entry_idx=%u "
               "resp_gen=%u wait_gen=%u wait_ldq=%u head=%u count=%u\n",
               port, req_id, entry_idx, resp_gen, wait_gen, wait_ldq, head,
               count);
}

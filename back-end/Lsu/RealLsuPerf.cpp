#include "RealLsuPerf.h"
#include "types.h"
#include <algorithm>
#include <array>
#include <cstdint>

namespace lsu_perf {
namespace {

#if !BSD_CONFIG












void record_mem_inst_latency(SimContext *ctx, bool started,
                             uint64_t start_cycle) {
  if (ctx == nullptr || !started) {
    return;
  }
  const uint64_t now = ctx->perf.cycle;
  if (now >= start_cycle) {


  }
}
#endif

} // namespace

uint64_t load_miss_mask(const LsuState &state) {
  uint64_t miss_mask = 0;
#if !BSD_CONFIG
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (state.ldq[i].cache_miss == true &&
        state.ldq[i].load_state != LoadState::Empty) {
      miss_mask |= (1ULL << state.ldq[i].rob_idx);
    }
  }
#else
  (void)state;
#endif
  return miss_mask;
}



void finish_mem_inst(SimContext *ctx, LdqEntry &entry) {
#if !BSD_CONFIG
  record_mem_inst_latency(ctx, entry.perf_mem_started,
                          entry.perf_mem_start_cycle);
  entry.perf_mem_started = false;
#else
  (void)ctx;
  (void)entry;
#endif
}

void finish_mem_inst(SimContext *ctx, StqEntry &entry) {
#if !BSD_CONFIG
  record_mem_inst_latency(ctx, entry.perf_mem_started,
                          entry.perf_mem_start_cycle);
  entry.perf_mem_started = false;
#else
  (void)ctx;
  (void)entry;
#endif
}

void start_mem_inst(SimContext *ctx, LdqEntry &entry) {
#if !BSD_CONFIG
  entry.perf_mem_started = true;
  entry.perf_mem_start_cycle = ctx == nullptr ? 0 : ctx->perf.cycle;
#else
  (void)ctx;
  (void)entry;
#endif
}

void start_mem_inst(SimContext *ctx, StqEntry &entry) {
#if !BSD_CONFIG
  entry.perf_mem_started = true;
  entry.perf_mem_start_cycle = ctx == nullptr ? 0 : ctx->perf.cycle;
#else
  (void)ctx;
  (void)entry;
#endif
}

void start_mem_inst_if_needed(SimContext *ctx, LdqEntry &entry) {
#if !BSD_CONFIG
  if (!entry.perf_mem_started) {
    start_mem_inst(ctx, entry);
  }
#else
  (void)ctx;
  (void)entry;
#endif
}

void start_mem_inst_if_needed(SimContext *ctx, const StqEntry &cur_entry,
                              StqEntry &nxt_entry) {
#if !BSD_CONFIG
  if (!cur_entry.perf_mem_started) {
    start_mem_inst(ctx, nxt_entry);
  }
#else
  (void)ctx;
  (void)cur_entry;
  (void)nxt_entry;
#endif
}

void reset_wait_start(WaitDcacheLDQEntry &entry) {
#if !BSD_CONFIG
  entry.wait_start_cycle = 0;
#else
  (void)entry;
#endif
}

void mark_wait_start(SimContext *ctx, WaitDcacheLDQEntry &entry) {
#if !BSD_CONFIG
  entry.wait_start_cycle = ctx == nullptr ? 0 : ctx->perf.cycle;
#else
  (void)ctx;
  (void)entry;
#endif
}

void retry_load_resp_timeouts(SimContext *ctx, const LsuState &cur,
                              LsuState &nxt) {
#if !BSD_CONFIG && CONFIG_LSU_DRESP_TIMEOUT_CYCLES > 0
  for (int i = 0; i < cur.wait_dcache_ldq_count; i++) {
    const uint32_t wait_idx = (cur.wait_dcache_ldq_head + i) % LDQ_SIZE;
    const auto &entry = cur.wait_dcache_ldq[wait_idx];
    if (!entry.valid) {
      continue;
    }
    const LdqEntry &ldq_entry = cur.ldq[entry.ldq_idx];
    if (ldq_entry.load_state != LoadState::WaitDcacheResp ||
        entry.wait_start_cycle == 0 || ctx == nullptr) {
      continue;
    }
    if (ctx->perf.cycle - entry.wait_start_cycle >=
        static_cast<uint64_t>(CONFIG_LSU_DRESP_TIMEOUT_CYCLES)) {
      nxt.ldq[entry.ldq_idx].load_state = LoadState::ReadyToIssue;
      nxt.ldq[entry.ldq_idx].replay_type = ReplayType::CONFLICT;
      nxt.wait_dcache_ldq[wait_idx].replay_wait_cycles = 0;
      nxt.wait_dcache_ldq[wait_idx].wait_start_cycle = 0;
      ctx->perf.ld_resp_timeout_retry_count++;
    }
  }
#else
  (void)ctx;
  (void)cur;
  (void)nxt;
#endif
}

void clear_replay(LoadReq &req) {
#if !BSD_CONFIG
  req.replay = false;
#else
  (void)req;
#endif
}

void clear_replay(StoreReq &req) {
#if !BSD_CONFIG
  req.replay = false;
#else
  (void)req;
#endif
}

void set_replay(LoadReq &req, bool is_replay) {
#if !BSD_CONFIG
  req.replay = is_replay;
#else
  (void)req;
  (void)is_replay;
#endif
}

void set_replay(StoreReq &req, bool is_replay) {
#if !BSD_CONFIG
  req.replay = is_replay;
#else
  (void)req;
  (void)is_replay;
#endif
}

bool is_load_replay(const LdqEntry &entry) {
#if !BSD_CONFIG
  return entry.replay_type != ReplayType::HIT;
#else
  (void)entry;
  return false;
#endif
}

bool is_store_replay(const StqEntry &entry) {
#if !BSD_CONFIG
  return entry.replay_type != ReplayType::HIT;
#else
  (void)entry;
  return false;
#endif
}

void mark_load_hit(LdqEntry &entry) {
#if !BSD_CONFIG
  entry.replay_type = ReplayType::HIT;
#else
  (void)entry;
#endif
}

void mark_load_replay(LdqEntry &entry, ReplayType replay) {
#if !BSD_CONFIG
  entry.replay_type = replay;
  if (entry.cache_miss == false) {
    entry.cache_miss = true;
  }
#else
  (void)entry;
  (void)replay;
#endif
}

void mark_store_hit(StqEntry &entry) {
#if !BSD_CONFIG
  entry.replay_type = ReplayType::HIT;
#else
  (void)entry;
#endif
}

void mark_store_replay(StqEntry &entry, ReplayType replay) {
#if !BSD_CONFIG
  entry.replay_type = replay;
#else
  (void)entry;
  (void)replay;
#endif
}

































} // namespace lsu_perf

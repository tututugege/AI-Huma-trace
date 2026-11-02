#include "RealLsuPerf.h"
#include "types.h"
#include <algorithm>
#include <array>
#include <cstdint>

namespace lsu_perf {
namespace {













void record_mem_inst_latency(SimContext *ctx, bool started,
                             uint64_t start_cycle) {
  if (ctx == nullptr || !started) {
    return;
  }
  const uint64_t now = ctx->perf.cycle;
  if (now >= start_cycle) {


  }
}

} // namespace

uint64_t load_miss_mask(const LsuState &state) {
  uint64_t miss_mask = 0;
  for (int i = 0; i < LDQ_SIZE; i++) {
    if (state.ldq[i].cache_miss == true &&
        state.ldq[i].load_state != LoadState::Empty) {
      miss_mask |= (1ULL << state.ldq[i].rob_idx);
    }
  }
  return miss_mask;
}



void finish_mem_inst(SimContext *ctx, LdqEntry &entry) {
  record_mem_inst_latency(ctx, entry.perf_mem_started,
                          entry.perf_mem_start_cycle);
  entry.perf_mem_started = false;
}

void finish_mem_inst(SimContext *ctx, StqEntry &entry) {
  record_mem_inst_latency(ctx, entry.perf_mem_started,
                          entry.perf_mem_start_cycle);
  entry.perf_mem_started = false;
}

void start_mem_inst(SimContext *ctx, LdqEntry &entry) {
  entry.perf_mem_started = true;
  entry.perf_mem_start_cycle = ctx == nullptr ? 0 : ctx->perf.cycle;
}

void start_mem_inst(SimContext *ctx, StqEntry &entry) {
  entry.perf_mem_started = true;
  entry.perf_mem_start_cycle = ctx == nullptr ? 0 : ctx->perf.cycle;
}

void start_mem_inst_if_needed(SimContext *ctx, LdqEntry &entry) {
  if (!entry.perf_mem_started) {
    start_mem_inst(ctx, entry);
  }
}

void start_mem_inst_if_needed(SimContext *ctx, const StqEntry &cur_entry,
                              StqEntry &nxt_entry) {
  if (!cur_entry.perf_mem_started) {
    start_mem_inst(ctx, nxt_entry);
  }
}

void reset_wait_start(WaitDcacheLDQEntry &entry) {
  entry.wait_start_cycle = 0;
}

void mark_wait_start(SimContext *ctx, WaitDcacheLDQEntry &entry) {
  entry.wait_start_cycle = ctx == nullptr ? 0 : ctx->perf.cycle;
}

void retry_load_resp_timeouts(SimContext *ctx, const LsuState &cur,
                              LsuState &nxt) {
  (void)ctx;
  (void)cur;
  (void)nxt;
}

void clear_replay(LoadReq &req) {
  req.replay = false;
}

void clear_replay(StoreReq &req) {
  req.replay = false;
}

void set_replay(LoadReq &req, bool is_replay) {
  req.replay = is_replay;
}

void set_replay(StoreReq &req, bool is_replay) {
  req.replay = is_replay;
}

bool is_load_replay(const LdqEntry &entry) {
  return entry.replay_type != ReplayType::HIT;
}

bool is_store_replay(const StqEntry &entry) {
  return entry.replay_type != ReplayType::HIT;
}

void mark_load_hit(LdqEntry &entry) {
  entry.replay_type = ReplayType::HIT;
}

void mark_load_replay(LdqEntry &entry, ReplayType replay) {
  entry.replay_type = replay;
  if (entry.cache_miss == false) {
    entry.cache_miss = true;
  }
}

void mark_store_hit(StqEntry &entry) {
  entry.replay_type = ReplayType::HIT;
}

void mark_store_replay(StqEntry &entry, ReplayType replay) {
  entry.replay_type = replay;
}

































} // namespace lsu_perf

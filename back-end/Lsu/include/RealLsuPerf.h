#pragma once

#include "RealLsu.h"
#include <cstdint>

namespace lsu_perf {

uint64_t load_miss_mask(const LsuState &state);

void finish_mem_inst(SimContext *ctx, LdqEntry &entry);
void finish_mem_inst(SimContext *ctx, StqEntry &entry);
void start_mem_inst(SimContext *ctx, LdqEntry &entry);
void start_mem_inst(SimContext *ctx, StqEntry &entry);
void start_mem_inst_if_needed(SimContext *ctx, LdqEntry &entry);
void start_mem_inst_if_needed(SimContext *ctx, const StqEntry &cur_entry,
                              StqEntry &nxt_entry);

void reset_wait_start(WaitDcacheLDQEntry &entry);
void mark_wait_start(SimContext *ctx, WaitDcacheLDQEntry &entry);
void retry_load_resp_timeouts(SimContext *ctx, const LsuState &cur,
                              LsuState &nxt);

void clear_replay(LoadReq &req);
void clear_replay(StoreReq &req);
void set_replay(LoadReq &req, bool is_replay);
void set_replay(StoreReq &req, bool is_replay);

bool is_load_replay(const LdqEntry &entry);
bool is_store_replay(const StqEntry &entry);
void mark_load_hit(LdqEntry &entry);
void mark_load_replay(LdqEntry &entry, ReplayType replay);
void mark_store_hit(StqEntry &entry);
void mark_store_replay(StqEntry &entry, ReplayType replay);




} // namespace lsu_perf

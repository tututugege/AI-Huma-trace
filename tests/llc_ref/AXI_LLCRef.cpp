#include "AXI_LLCRef.h"
#include "PhysMemory.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <type_traits>

extern long long sim_time;

namespace axi_interconnect {

namespace {
#ifndef CONFIG_AXI_LLC_FOCUS_LINE0
#define CONFIG_AXI_LLC_FOCUS_LINE0 0u
#endif

#ifndef CONFIG_AXI_LLC_FOCUS_LINE1
#define CONFIG_AXI_LLC_FOCUS_LINE1 0u
#endif


#ifndef CONFIG_AXI_LLC_FOCUS_RANGE_BEGIN
#define CONFIG_AXI_LLC_FOCUS_RANGE_BEGIN 0u
#endif

#ifndef CONFIG_AXI_LLC_FOCUS_RANGE_END
#define CONFIG_AXI_LLC_FOCUS_RANGE_END 0u
#endif

#ifndef CONFIG_AXI_LLC_RESP_TRACE_BEGIN
#define CONFIG_AXI_LLC_RESP_TRACE_BEGIN 0LL
#endif

#ifndef CONFIG_AXI_LLC_RESP_TRACE_END
#define CONFIG_AXI_LLC_RESP_TRACE_END 0LL
#endif

constexpr uint8_t kInvalidReadMaster = NUM_READ_MASTERS;

bool allow_parallel_demand_mshr(uint8_t master) {
  return master == MASTER_DCACHE_R;
}





bool llc_focus_line(uint32_t line_addr) {
  if (CONFIG_AXI_LLC_DEBUG_LOG == 0) {
    return false;
  }
  constexpr uint32_t kRangeBegin =
      static_cast<uint32_t>(CONFIG_AXI_LLC_FOCUS_RANGE_BEGIN);
  constexpr uint32_t kRangeEnd =
      static_cast<uint32_t>(CONFIG_AXI_LLC_FOCUS_RANGE_END);
  return (CONFIG_AXI_LLC_FOCUS_LINE0 != 0u &&
          line_addr == static_cast<uint32_t>(CONFIG_AXI_LLC_FOCUS_LINE0)) ||
         (CONFIG_AXI_LLC_FOCUS_LINE1 != 0u &&
          line_addr == static_cast<uint32_t>(CONFIG_AXI_LLC_FOCUS_LINE1)) ||
         (kRangeEnd > kRangeBegin && line_addr >= kRangeBegin &&
          line_addr < kRangeEnd);
}

bool llc_focus_set(const AXI_LLCConfig &config, uint32_t set) {
  if (CONFIG_AXI_LLC_DEBUG_LOG == 0) {
    return false;
  }
  (void)config;
  (void)set;
  return false;
}

bool llc_resp_trace_active() {
  return CONFIG_AXI_LLC_DEBUG_LOG != 0 &&
         CONFIG_AXI_LLC_RESP_TRACE_END >= CONFIG_AXI_LLC_RESP_TRACE_BEGIN &&
         sim_time >= static_cast<long long>(CONFIG_AXI_LLC_RESP_TRACE_BEGIN) &&
         sim_time <= static_cast<long long>(CONFIG_AXI_LLC_RESP_TRACE_END);
}







void perf_count_read_access(AXI_LLCPerfCounters_t &perf, uint8_t master) {
}

void perf_count_read_hit(AXI_LLCPerfCounters_t &perf, uint8_t master) {
}

void perf_count_read_miss(AXI_LLCPerfCounters_t &perf, uint8_t master) {
}

uint32_t read_u32_le(const uint8_t *ptr) {
  return static_cast<uint32_t>(ptr[0]) |
         (static_cast<uint32_t>(ptr[1]) << 8) |
         (static_cast<uint32_t>(ptr[2]) << 16) |
         (static_cast<uint32_t>(ptr[3]) << 24);
}

void write_u32_le(uint8_t *ptr, uint32_t value) {
  ptr[0] = static_cast<uint8_t>(value & 0xFFu);
  ptr[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  ptr[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  ptr[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

// Read responses carry no values in the trace-driven build: instruction words and page-table
// entries come from the core's memory (tbhook::mem_read), data from the trace facts.








WideWriteStrb_t full_line_strobe(const AXI_LLCConfig &config) {
  WideWriteStrb_t out;
  out.clear();
  for (uint32_t b = 0; b < config.line_bytes && b < MAX_WRITE_TRANSACTION_BYTES; ++b) {
    out.set(b, true);
  }
  return out;
}





uint32_t build_line_addr_from_tag_set(const AXI_LLCConfig &config, uint32_t tag,
                                      uint32_t set) {
  if (!config.valid()) {
    return 0;
  }
  return ((tag * config.set_count()) + set) * config.line_bytes;
}



bool is_full_line_write(const AXI_LLCConfig &config, uint32_t addr,
                        const WideWriteStrb_t &strobe, uint32_t total_size) {
  if (config.line_bytes == 0) {
    return false;
  }
  if ((addr % config.line_bytes) != 0) {
    return false;
  }
  if (total_size + 1u != config.line_bytes) {
    return false;
  }
  for (uint32_t byte = 0; byte < config.line_bytes; ++byte) {
    if (!strobe.test(byte)) {
      return false;
    }
  }
  return true;
}

uint32_t update_dirty_line_count(uint32_t current, bool old_dirty, bool new_dirty) {
  if (old_dirty == new_dirty) {
    return current;
  }
  if (new_dirty) {
    return current + 1u;
  }
  return current == 0 ? 0 : current - 1u;
}

AXI_LLC_Bytes_t build_repl_payload(uint32_t next_way) {
  AXI_LLC_Bytes_t bytes;
  bytes.resize(AXI_LLC_REPL_BYTES);
  write_u32_le(bytes.data(), next_way);
  return bytes;
}

uint32_t decode_repl_way(const AXI_LLC_Bytes_t &payload) {
  if (payload.size() < AXI_LLC_REPL_BYTES) {
    return 0;
  }
  return read_u32_le(payload.data());
}

bool llc_table_write_busy(const AXI_LLC_TableOut_t &table_out) {
  return table_out.data.enable || table_out.meta.enable ||
         table_out.valid.enable || table_out.repl.enable;
}

bool decode_valid_bit(const AXI_LLC_Bytes_t &payload, uint32_t way,
                      bool fallback_valid) {
  const size_t byte_idx = static_cast<size_t>(way);
  if (byte_idx >= payload.size()) {
    return fallback_valid;
  }
  return payload.data()[byte_idx] != 0;
}

void encode_valid_bit_payload(uint32_t ways, uint32_t way, bool valid,
                              AXI_LLC_Bytes_t &payload,
                              std::vector<uint8_t> &byte_enable) {
  const size_t row_bytes = static_cast<size_t>(ways);
  payload.resize(row_bytes);
  byte_enable.assign(row_bytes, 0);
  const size_t byte_idx = static_cast<size_t>(way);
  if (byte_idx >= row_bytes) {
    return;
  }
  payload.data()[byte_idx] = valid ? 1u : 0u;
  byte_enable[byte_idx] = 1;
}

} // namespace

AXI_LLCRef::AXI_LLCRef() { reset(); }

bool AXI_LLCRef::enqueue_read_response(uint8_t master, uint8_t id,
                                    const WideReadData_t &data) {
  if (master >= NUM_READ_MASTERS) {
    return false;
  }
  if (!io.reg_write.read_resp_valid_r[master]) {
    io.reg_write.read_resp_valid_r[master] = true;
    io.reg_write.read_resp_fresh_r[master] = true;
    io.reg_write.read_resp_id_r[master] = id;
    io.reg_write.read_resp_data_r[master] = data;
    return true;
  }
  if (io.reg_write.read_resp_q_count_r[master] >=
      AXI_LLC_READ_RESP_QUEUE_DEPTH) {
    return false;
  }
  const uint8_t tail =
      io.reg_write.read_resp_q_tail_r[master] % AXI_LLC_READ_RESP_QUEUE_DEPTH;
  io.reg_write.read_resp_q_id_r[master][tail] = id;
  io.reg_write.read_resp_q_data_r[master][tail] = data;
  io.reg_write.read_resp_q_tail_r[master] =
      static_cast<uint8_t>((tail + 1u) % AXI_LLC_READ_RESP_QUEUE_DEPTH);
  io.reg_write.read_resp_q_count_r[master]++;
  return true;
}

bool AXI_LLCRef::read_victim_wb_queue_full(const AXI_LLC_Regs_t &regs) const {
  return regs.read_victim_wb_q_count_r >= AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
}

bool AXI_LLCRef::read_victim_snapshot_queued(const AXI_LLC_Regs_t &regs,
                                          uint8_t owner_slot,
                                          uint32_t victim_addr) const {
  for (uint32_t i = 0; i < regs.read_victim_wb_q_count_r &&
                       i < AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
       ++i) {
    const uint8_t slot =
        static_cast<uint8_t>((regs.read_victim_wb_q_head_r + i) %
                             AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
    const auto &entry = regs.read_victim_wb_q[slot];
    if (!entry.valid) {
      continue;
    }
    if (entry.owner_slot == owner_slot && entry.victim_addr == victim_addr) {
      return true;
    }
  }
  return false;
}

bool AXI_LLCRef::enqueue_read_victim_snapshot(uint8_t owner_slot,
                                           uint32_t victim_addr,
                                           const WideWriteStrb_t &victim_strobe) {
  if (read_victim_snapshot_queued(io.reg_write, owner_slot, victim_addr)) {
    return true;
  }
  if (read_victim_wb_queue_full(io.reg_write)) {
    return false;
  }
  const uint8_t tail = io.reg_write.read_victim_wb_q_tail_r %
                       AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
  auto &entry = io.reg_write.read_victim_wb_q[tail];
  entry = {};
  entry.valid = true;
  entry.owner_slot = owner_slot;
  entry.victim_addr = victim_addr;
  entry.victim_strobe = victim_strobe;
  io.reg_write.read_victim_wb_q_tail_r =
      static_cast<uint8_t>((tail + 1u) % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
  io.reg_write.read_victim_wb_q_count_r++;
  return true;
}



bool AXI_LLCRef::has_pending_upstream_write_line(uint32_t line_addr_value) const {
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &req = io.ext_in.upstream.write_req[master];
    if (!req.valid || req.bypass) {
      continue;
    }
    if (line_addr(config_, req.addr) == line_addr_value) {
      return true;
    }
  }
  return false;
}

bool AXI_LLCRef::write_line_pending(const AXI_LLC_Regs_t &regs,
                                 uint32_t line_addr_value) const {
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = regs.write_ctx[master];
    if (!ctx.valid || ctx.line_addr != line_addr_value) {
      continue;
    }
    if (ctx.bypass) {
      if (!ctx.mem_done) {
        return true;
      }
    } else if (!ctx.cache_done) {
      return true;
    }
  }
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    for (uint32_t i = 0; i < regs.write_q_count_r[master] && i < MAX_WRITE_OUTSTANDING;
         ++i) {
      const uint32_t slot = (regs.write_q_head_r[master] + i) % MAX_WRITE_OUTSTANDING;
      const auto &entry = regs.write_q[master][slot];
      if (!entry.valid) {
        continue;
      }
      if (line_addr(config_, entry.addr) == line_addr_value) {
        return true;
      }
    }
  }
  return false;
}

bool AXI_LLCRef::read_victim_snapshot_present(const AXI_LLC_Regs_t &regs,
                                           uint32_t victim_addr) const {
  if (victim_addr == 0) {
    return false;
  }
  if (regs.victim_wb_valid_r && !regs.victim_wb_for_write_r &&
      regs.victim_wb_addr_r == victim_addr) {
    return true;
  }
  for (uint32_t i = 0; i < regs.read_victim_wb_q_count_r &&
                       i < AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
       ++i) {
    const uint8_t slot =
        static_cast<uint8_t>((regs.read_victim_wb_q_head_r + i) %
                             AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
    const auto &entry = regs.read_victim_wb_q[slot];
    if (entry.valid && entry.victim_addr == victim_addr) {
      return true;
    }
  }
  return false;
}

bool AXI_LLCRef::victim_snapshot_waits_for_write_resolution(
    const AXI_LLC_Regs_t &regs, uint32_t victim_addr) const {
  if (victim_addr == 0) {
    return false;
  }
  // Only writes that have already consumed cache state can block the victim
  // snapshot. Pre-lookup / queued writes must wait and retry after the refill
  // path has externalized the snapshot.
  if (regs.lookup_valid_r && regs.lookup_is_write_r && regs.lookup_issued_r &&
      !regs.lookup_is_bypass_r &&
      line_addr(config_, regs.lookup_addr_r) == victim_addr) {
    return true;
  }
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = regs.write_ctx[master];
    if (!ctx.valid || ctx.bypass || ctx.line_addr != victim_addr) {
      continue;
    }
    if (ctx.cache_pending) {
      return true;
    }
  }
  return false;
}

bool AXI_LLCRef::victim_line_pending(const AXI_LLC_Regs_t &regs,
                                  uint32_t line_addr_value) const {
  if (regs.victim_wb_valid_r && regs.victim_wb_addr_r == line_addr_value) {
    return true;
  }
  for (uint32_t i = 0; i < regs.read_victim_wb_q_count_r &&
                       i < AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
       ++i) {
    const uint8_t slot =
        static_cast<uint8_t>((regs.read_victim_wb_q_head_r + i) %
                             AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
    const auto &entry = regs.read_victim_wb_q[slot];
    if (entry.valid && entry.victim_addr == line_addr_value) {
      return true;
    }
  }
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = regs.write_ctx[master];
    if (!ctx.valid || ctx.bypass || !ctx.cache_pending || !ctx.victim_dirty) {
      continue;
    }
    if (ctx.victim_addr == line_addr_value) {
      return true;
    }
  }
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t slot = 0; slot < limit; ++slot) {
    const auto &entry = regs.mshr[slot];
    if (!entry.valid || entry.bypass || !entry.victim_dirty ||
        entry.refill_committed) {
      continue;
    }
    if (entry.victim_addr != line_addr_value) {
      continue;
    }
    const bool read_victim_writeback_armed =
        regs.victim_wb_valid_r && !regs.victim_wb_for_write_r &&
        regs.victim_wb_addr_r == line_addr_value &&
        regs.victim_wb_mshr_slot_r == static_cast<uint8_t>(slot);
    // Before refill data arrives, the victim line is still resident and write
    // hits must be able to refresh the pending victim snapshot. Once refill is
    // ready, the victim line may already be removed from the table before the
    // dedicated victim writeback request is armed, so block accesses through
    // the whole "refill_valid -> victim writeback done" window.
    if (entry.refill_valid || entry.victim_writeback_done ||
        read_victim_writeback_armed) {
      return true;
    }
  }
  return false;
}

bool AXI_LLCRef::way_reserved_by_pending_write(const AXI_LLC_Regs_t &regs,
                                            uint32_t set, uint8_t way,
                                            uint32_t line_addr_value) const {
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = regs.write_ctx[master];
    if (!ctx.valid || ctx.bypass || !ctx.cache_pending) {
      continue;
    }
    if (ctx.set != set || ctx.way != way) {
      continue;
    }
    if (ctx.line_addr == line_addr_value) {
      continue;
    }
    return true;
  }
  return false;
}

bool AXI_LLCRef::way_reserved_by_pending_refill(const AXI_LLC_Regs_t &regs,
                                             uint32_t set, uint8_t way,
                                             uint32_t line_addr_value) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t slot = 0; slot < limit; ++slot) {
    const auto &entry = regs.mshr[slot];
    if (!entry.valid || entry.bypass || entry.refill_committed) {
      continue;
    }
    if (entry.set != set || entry.way != way) {
      continue;
    }
    if (entry.line_addr == line_addr_value) {
      continue;
    }
    return true;
  }
  return false;
}

int AXI_LLCRef::pick_unreserved_way(const AXI_LLC_Regs_t &regs, uint32_t set,
                                 int first_invalid_way, uint32_t repl_way_raw,
                                 uint32_t line_addr_value) const {
  auto way_reserved = [&](uint8_t way) {
    return way_reserved_by_pending_write(regs, set, way, line_addr_value) ||
           way_reserved_by_pending_refill(regs, set, way, line_addr_value);
  };

  if (first_invalid_way >= 0) {
    const uint8_t way = static_cast<uint8_t>(first_invalid_way);
    if (!way_reserved(way)) {
      return first_invalid_way;
    }
  }

  for (uint32_t off = 0; off < config_.ways; ++off) {
    const uint8_t cand =
        static_cast<uint8_t>((repl_way_raw + off) % config_.ways);
    if (!way_reserved(cand)) {
      return static_cast<int>(cand);
    }
  }

  return -1;
}

bool AXI_LLCRef::can_accept_invalidate_line_now(uint32_t line_addr_value) const {
  for (uint8_t master = 0; master < NUM_READ_MASTERS; ++master) {
    if (io.regs.read_resp_valid_r[master] ||
        io.regs.read_resp_q_count_r[master] != 0) {
      return false;
    }
  }

  if (find_mshr_by_line_addr(io.regs, line_addr_value) >= 0) {
    return false;
  }

  const int bypass_write_resp_owner = find_bypass_write_mem_owner(io.regs);
  if (io.ext_in.mem.write_resp_valid && bypass_write_resp_owner >= 0) {
    const auto &ctx = io.regs.write_ctx[static_cast<uint8_t>(bypass_write_resp_owner)];
    if (ctx.valid && ctx.line_addr == line_addr_value) {
      return false;
    }
  }

  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (io.ext_in.upstream.write_req[master].valid &&
        line_addr(config_, io.ext_in.upstream.write_req[master].addr) ==
            line_addr_value) {
      return false;
    }
  }

  if (write_line_pending(io.regs, line_addr_value)) {
    return false;
  }
  if (victim_line_pending(io.regs, line_addr_value)) {
    return false;
  }

  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = io.regs.write_ctx[master];
    if (ctx.valid && ctx.line_addr == line_addr_value) {
      return false;
    }
    if (io.regs.write_resp_valid_r[master] &&
        io.regs.write_resp_line_addr_r[master] == line_addr_value) {
      return false;
    }
  }

  if (io.regs.lookup_valid_r &&
      line_addr(config_, io.regs.lookup_addr_r) == line_addr_value) {
    return false;
  }

  if (io.regs.victim_wb_valid_r) {
    if (io.regs.victim_wb_addr_r == line_addr_value) {
      return false;
    }
    if (io.regs.victim_wb_for_write_r &&
        io.regs.victim_wb_write_master_r < NUM_WRITE_MASTERS) {
      const auto &ctx = io.regs.write_ctx[io.regs.victim_wb_write_master_r];
      if (ctx.valid && ctx.line_addr == line_addr_value) {
        return false;
      }
    }
  }

  return true;
}

bool AXI_LLCRef::has_dirty_or_write_hazard(const AXI_LLC_Regs_t &regs) const {
  if (regs.dirty_line_count_r != 0) {
    return true;
  }
  if (regs.victim_wb_valid_r) {
    return true;
  }
  for (uint32_t slot = 0; slot < config_.mshr_num && slot < MAX_OUTSTANDING; ++slot) {
    const auto &entry = regs.mshr[slot];
    if (!entry.valid) {
      continue;
    }
    if (entry.is_write) {
      return true;
    }
    if (entry.victim_dirty && !entry.victim_writeback_done) {
      return true;
    }
  }
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (regs.write_ctx[master].valid || regs.write_q_count_r[master] != 0 ||
        regs.write_resp_valid_r[master]) {
      return true;
    }
  }
  if (regs.lookup_valid_r && regs.lookup_is_write_r) {
    return true;
  }
  return false;
}

bool AXI_LLCRef::can_accept_invalidate_all_now(const AXI_LLC_Regs_t &regs) const {
  return !has_dirty_or_write_hazard(regs);
}

bool AXI_LLCRef::direct_mapped_coords(uint32_t addr, uint32_t *set,
                                   uint8_t *way) const {
  if (!config_.valid() || config_.line_bytes == 0 || config_.set_count() == 0 ||
      set == nullptr || way == nullptr) {
    return false;
  }
  const uint32_t line_idx = addr / config_.line_bytes;
  const uint32_t set_count = config_.set_count();
  const uint32_t mapped_way = line_idx / set_count;
  if (mapped_way >= config_.ways) {
    return false;
  }
  *set = line_idx % set_count;
  *way = static_cast<uint8_t>(mapped_way);
  return true;
}


uint32_t AXI_LLCRef::line_addr(const AXI_LLCConfig &config, uint32_t addr) {
  return config.line_bytes == 0 ? addr : (addr / config.line_bytes) * config.line_bytes;
}

uint32_t AXI_LLCRef::set_index(const AXI_LLCConfig &config, uint32_t addr) {
  if (!config.valid() || config.line_bytes == 0) {
    return 0;
  }
  return (addr / config.line_bytes) % config.set_count();
}

uint32_t AXI_LLCRef::tag_of(const AXI_LLCConfig &config, uint32_t addr) {
  if (!config.valid() || config.line_bytes == 0 || config.set_count() == 0) {
    return 0;
  }
  return (addr / config.line_bytes) / config.set_count();
}

uint32_t AXI_LLCRef::valid_row_bytes(const AXI_LLCConfig &config) {
  return config.ways;
}

AXI_LLCMetaEntry_t AXI_LLCRef::decode_meta(const AXI_LLC_Bytes_t &payload,
                                        uint32_t way) {
  AXI_LLCMetaEntry_t entry{};
  const size_t offset = static_cast<size_t>(way) * AXI_LLC_META_ENTRY_BYTES;
  if (offset + AXI_LLC_META_ENTRY_BYTES > payload.size()) {
    return entry;
  }
  const uint32_t packed = read_u32_le(payload.data() + offset);
  entry.tag = packed & 0x1fffffffu;
  entry.flags = 0;
  if ((packed >> 29) & 0x1u) {
    entry.flags |= AXI_LLC_META_VALID;
  }
  if ((packed >> 30) & 0x1u) {
    entry.flags |= AXI_LLC_META_DIRTY;
  }
  if ((packed >> 31) & 0x1u) {
    entry.flags |= AXI_LLC_META_PREFETCH;
  }
  return entry;
}

void AXI_LLCRef::encode_meta(const AXI_LLCMetaEntry_t &entry,
                          AXI_LLC_Bytes_t &payload) {
  payload.resize(AXI_LLC_META_ENTRY_BYTES);
  if ((entry.tag & 0xe0000000u) != 0) {

    std::abort();
  }
  uint32_t packed = entry.tag & 0x1fffffffu;
  if ((entry.flags & AXI_LLC_META_VALID) != 0) {
    packed |= (1u << 29);
  }
  if ((entry.flags & AXI_LLC_META_DIRTY) != 0) {
    packed |= (1u << 30);
  }
  if ((entry.flags & AXI_LLC_META_PREFETCH) != 0) {
    packed |= (1u << 31);
  }
  write_u32_le(payload.data(), packed);
}

void AXI_LLCRef::set_config(const AXI_LLCConfig &config) {
  config_ = config;
  io.regs.enable_r = config_.enable;
  io.regs.state =
      config_.enable ? AXI_LLCState::kIdle : AXI_LLCState::kDisabled;
}

void AXI_LLCRef::reset() {
  io = {};
  io.regs.enable_r = config_.enable;
  io.regs.state =
      config_.enable ? AXI_LLCState::kIdle : AXI_LLCState::kDisabled;
}

int AXI_LLCRef::find_free_mshr(const AXI_LLC_Regs_t &regs) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (!regs.mshr[i].valid) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_LLCRef::find_mshr_by_line_addr(const AXI_LLC_Regs_t &regs,
                                    uint32_t line_addr_value) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && !regs.mshr[i].bypass &&
        regs.mshr[i].line_addr == line_addr_value) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_LLCRef::find_mshr_by_victim_addr(const AXI_LLC_Regs_t &regs,
                                      uint32_t victim_addr_value) const {
  if (victim_addr_value == 0) {
    return -1;
  }
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    const auto &entry = regs.mshr[i];
    if (!entry.valid || !entry.victim_dirty || entry.victim_writeback_done) {
      continue;
    }
    if (entry.victim_addr == victim_addr_value) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool AXI_LLCRef::has_mshr_for_master(const AXI_LLC_Regs_t &regs, uint8_t master) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && !regs.mshr[i].is_prefetch &&
        regs.mshr[i].master == master) {
      return true;
    }
  }
  return false;
}

uint32_t AXI_LLCRef::count_free_mshrs(const AXI_LLC_Regs_t &regs) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  uint32_t free_count = 0;
  for (uint32_t i = 0; i < limit; ++i) {
    if (!regs.mshr[i].valid) {
      ++free_count;
    }
  }
  return free_count;
}

bool AXI_LLCRef::has_demand_mshr(const AXI_LLC_Regs_t &regs) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && !regs.mshr[i].is_prefetch) {
      return true;
    }
  }
  return false;
}

int AXI_LLCRef::pick_mem_issue_slot(const AXI_LLC_Regs_t &regs) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && !regs.mshr[i].is_prefetch &&
        !regs.mshr[i].mem_req_issued) {
      return static_cast<int>(i);
    }
  }
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && regs.mshr[i].is_prefetch &&
        !regs.mshr[i].mem_req_issued) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_LLCRef::find_mshr_by_mem_id(const AXI_LLC_Regs_t &regs, uint8_t mem_id) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && static_cast<uint8_t>(i) == mem_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_LLCRef::pick_refill_commit_slot(const AXI_LLC_Regs_t &regs) const {
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && regs.mshr[i].refill_valid) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_LLCRef::pick_new_read_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_READ_MASTERS; ++off) {
    const uint32_t idx = (regs.rr_read_master_r + off) % NUM_READ_MASTERS;
    const auto &req = io.ext_in.upstream.read_req[idx];
    const bool resp_slot_busy_start_of_cycle = io.regs.read_resp_valid_r[idx];
    const bool resp_slot_busy_end_of_cycle = regs.read_resp_valid_r[idx];
    if (!req.valid || resp_slot_busy_start_of_cycle ||
        resp_slot_busy_end_of_cycle) {
      continue;
    }
    if (regs.lookup_valid_r) {
      continue;
    }
    if (has_pending_upstream_write_line(line_addr(config_, req.addr))) {
      continue;
    }
    if (write_line_pending(regs, line_addr(config_, req.addr))) {
      continue;
    }
    if (victim_line_pending(regs, line_addr(config_, req.addr))) {
      continue;
    }
    if (find_mshr_by_line_addr(regs, line_addr(config_, req.addr)) >= 0) {
      continue;
    }
    if (!req.bypass && !allow_parallel_demand_mshr(static_cast<uint8_t>(idx)) &&
        has_mshr_for_master(regs, static_cast<uint8_t>(idx))) {
      continue;
    }
    if (find_free_mshr(regs) < 0) {
      continue;
    }
    return static_cast<int>(idx);
  }
  return -1;
}

int AXI_LLCRef::pick_new_write_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t idx = (regs.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    const auto &req = io.ext_in.upstream.write_req[idx];
    if (!req.valid || write_queue_full(regs, static_cast<uint8_t>(idx))) {
      continue;
    }
    if (victim_line_pending(regs, line_addr(config_, req.addr))) {
      continue;
    }
    return static_cast<int>(idx);
  }
  return -1;
}

bool AXI_LLCRef::write_queue_full(const AXI_LLC_Regs_t &regs, uint8_t master) const {
  return master < NUM_WRITE_MASTERS &&
         regs.write_q_count_r[master] >= MAX_WRITE_OUTSTANDING;
}

bool AXI_LLCRef::write_queue_empty(const AXI_LLC_Regs_t &regs, uint8_t master) const {
  return master >= NUM_WRITE_MASTERS || regs.write_q_count_r[master] == 0;
}

const AXI_LLCWritePendingReq_t *AXI_LLCRef::write_queue_front(
    const AXI_LLC_Regs_t &regs, uint8_t master) const {
  if (write_queue_empty(regs, master)) {
    return nullptr;
  }
  const auto &entry =
      regs.write_q[master][regs.write_q_head_r[master] % MAX_WRITE_OUTSTANDING];
  return entry.valid ? &entry : nullptr;
}

int AXI_LLCRef::pick_write_lookup_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t master = (regs.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    const auto &ctx = regs.write_ctx[master];
    if (ctx.valid && ctx.lookup_pending) {
      return static_cast<int>(master);
    }
  }
  return -1;
}

int AXI_LLCRef::pick_bypass_write_issue_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t master = (regs.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    const auto &ctx = regs.write_ctx[master];
    if (ctx.valid && ctx.bypass && !ctx.mem_done && !ctx.mem_issued) {
      return static_cast<int>(master);
    }
  }
  return -1;
}

int AXI_LLCRef::find_bypass_write_mem_owner(const AXI_LLC_Regs_t &regs) const {
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = regs.write_ctx[master];
    if (ctx.valid && ctx.bypass && ctx.mem_issued && !ctx.mem_done) {
      return static_cast<int>(master);
    }
  }
  return -1;
}

int AXI_LLCRef::find_prefetch_queue_slot(const AXI_LLC_Regs_t &regs,
                                      uint32_t line_addr) const {
  for (uint32_t i = 0; i < AXI_LLC_MAX_PREFETCH_QUEUE; ++i) {
    if (regs.prefetch_q[i].valid && regs.prefetch_q[i].line_addr == line_addr) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_LLCRef::find_free_prefetch_queue_slot(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t i = 0; i < AXI_LLC_MAX_PREFETCH_QUEUE; ++i) {
    if (!regs.prefetch_q[i].valid) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_LLCRef::pick_prefetch_queue_slot(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t i = 0; i < AXI_LLC_MAX_PREFETCH_QUEUE; ++i) {
    if (regs.prefetch_q[i].valid) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool AXI_LLCRef::prefetch_candidate_exists(const AXI_LLC_Regs_t &regs,
                                        uint32_t line_addr) const {
  if (find_prefetch_queue_slot(regs, line_addr) >= 0) {
    return true;
  }
  if (regs.lookup_valid_r &&
      AXI_LLCRef::line_addr(config_, regs.lookup_addr_r) == line_addr) {
    return true;
  }
  return find_mshr_by_line_addr(regs, line_addr) >= 0;
}

bool AXI_LLCRef::can_allocate_prefetch_mshr(const AXI_LLC_Regs_t &regs) const {
  if (count_free_mshrs(regs) <= 1) {
    return false;
  }
  const uint32_t limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  for (uint32_t i = 0; i < limit; ++i) {
    if (regs.mshr[i].valid && regs.mshr[i].is_prefetch) {
      return false;
    }
  }
  return true;
}

bool AXI_LLCRef::line_has_valid_meta(const AXI_LLC_Bytes_t &valid_payload,
                                  const AXI_LLC_Bytes_t &meta_payload,
                                  uint32_t tag,
                                  int *hit_way, int *first_invalid_way,
                                  AXI_LLCMetaEntry_t *hit_meta) const {
  *hit_way = -1;
  *first_invalid_way = -1;
  if (hit_meta != nullptr) {
    *hit_meta = {};
  }
  for (uint32_t way = 0; way < config_.ways; ++way) {
    const auto meta = decode_meta(meta_payload, way);
    const bool valid = decode_valid_bit(valid_payload, way, false);
    if (!valid && *first_invalid_way < 0) {
      *first_invalid_way = static_cast<int>(way);
    }
    if (valid && meta.tag == tag) {
      *hit_way = static_cast<int>(way);
      if (hit_meta != nullptr) {
        *hit_meta = meta;
      }
      return true;
    }
  }
  return false;
}

void AXI_LLCRef::try_schedule_prefetch(const AXI_LLCMissEntry_t &entry) {
  if (!config_.prefetch_enable || entry.bypass || entry.is_prefetch ||
      !entry.prefetch_train || config_.line_bytes == 0) {
    if (!entry.bypass && !entry.is_prefetch) {
      io.reg_write.prefetch_stream_valid_r = true;
      io.reg_write.prefetch_last_miss_line_r = entry.line_addr;
    }
    return;
  }

  if (!io.regs.prefetch_stream_valid_r ||
      io.regs.prefetch_last_miss_line_r + config_.line_bytes != entry.line_addr) {
    io.reg_write.prefetch_stream_valid_r = true;
    io.reg_write.prefetch_last_miss_line_r = entry.line_addr;
    return;
  }

  const uint32_t degree = std::max<uint32_t>(1, config_.prefetch_degree);
  for (uint32_t d = 1; d <= degree; ++d) {
    const uint32_t next_line_addr = entry.line_addr + d * config_.line_bytes;
    if (prefetch_candidate_exists(io.reg_write, next_line_addr)) {
      continue;
    }
    const int slot = find_free_prefetch_queue_slot(io.reg_write);
    if (slot < 0) {
      break;
    }
    io.reg_write.prefetch_q[slot].valid = true;
    io.reg_write.prefetch_q[slot].line_addr = next_line_addr;
  }
  io.reg_write.prefetch_stream_valid_r = true;
  io.reg_write.prefetch_last_miss_line_r = entry.line_addr;
}

void AXI_LLCRef::try_launch_prefetch_lookup() {
  const auto &regs = io.reg_write;
  const uint32_t mshr_limit =
      std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  if (regs.lookup_valid_r || !io.ext_in.mem.prefetch_allow ||
      has_demand_mshr(regs) || count_free_mshrs(regs) != mshr_limit ||
      regs.prefetch_quiet_cycles_r == 0) {
    return;
  }
  const int slot = pick_prefetch_queue_slot(regs);
  if (slot < 0) {
    return;
  }
  io.reg_write.lookup_valid_r = true;
  io.reg_write.lookup_issued_r = false;
  io.reg_write.lookup_addr_r = regs.prefetch_q[slot].line_addr;
  io.reg_write.lookup_size_r = static_cast<uint8_t>(config_.line_bytes - 1);
  io.reg_write.lookup_master_r = kInvalidReadMaster;
  io.reg_write.lookup_id_r = static_cast<uint8_t>(slot);
  io.reg_write.lookup_is_prefetch_r = true;
  io.reg_write.lookup_is_invalidate_r = false;
  io.reg_write.lookup_is_write_r = false;
  io.reg_write.lookup_is_bypass_r = false;
  io.reg_write.lookup_is_direct_mapped_r = false;
  io.reg_write.lookup_is_mode2_ddr_aligned_r = false;
  io.reg_write.state = AXI_LLCState::kLookup;
}

void AXI_LLCRef::drive_read_responses() {
  for (uint8_t i = 0; i < NUM_READ_MASTERS; ++i) {
    io.ext_out.upstream.read_resp[i].valid = io.regs.read_resp_valid_r[i];
    io.ext_out.upstream.read_resp[i].data = io.regs.read_resp_data_r[i];
    io.ext_out.upstream.read_resp[i].id = io.regs.read_resp_id_r[i];
    if (llc_resp_trace_active() && i == MASTER_DCACHE_R &&
        (io.regs.read_resp_valid_r[i] ||
         io.regs.read_resp_q_count_r[i] != 0 ||
         io.ext_in.upstream.read_resp[i].ready)) {

    }
    // In the shared top-level fabric, upstream ready is sampled one phase
    // earlier than the consumer computes its current-cycle backpressure. Keep
    // a newly-produced response visible for one full cycle before letting the
    // previous-cycle ready retire it.
    if (io.regs.read_resp_valid_r[i] && io.regs.read_resp_fresh_r[i]) {
      io.reg_write.read_resp_fresh_r[i] = false;
    } else if (io.regs.read_resp_valid_r[i] &&
               io.ext_in.upstream.read_resp[i].ready) {
      if (io.regs.read_resp_q_count_r[i] != 0) {
        const uint8_t head =
            io.regs.read_resp_q_head_r[i] % AXI_LLC_READ_RESP_QUEUE_DEPTH;
        io.reg_write.read_resp_valid_r[i] = true;
        io.reg_write.read_resp_fresh_r[i] = true;
        io.reg_write.read_resp_id_r[i] = io.regs.read_resp_q_id_r[i][head];
        io.reg_write.read_resp_data_r[i] = io.regs.read_resp_q_data_r[i][head];
        io.reg_write.read_resp_q_id_r[i][head] = 0;
        io.reg_write.read_resp_q_data_r[i][head].clear();
        io.reg_write.read_resp_q_head_r[i] =
            static_cast<uint8_t>((head + 1u) % AXI_LLC_READ_RESP_QUEUE_DEPTH);
        io.reg_write.read_resp_q_count_r[i]--;
      } else {
        io.reg_write.read_resp_valid_r[i] = false;
        io.reg_write.read_resp_fresh_r[i] = false;
        io.reg_write.read_resp_id_r[i] = 0;
        io.reg_write.read_resp_data_r[i].clear();
      }
    }
    if (!io.reg_write.read_resp_valid_r[i] && io.regs.read_resp_q_count_r[i] != 0) {
      const uint8_t head =
          io.regs.read_resp_q_head_r[i] % AXI_LLC_READ_RESP_QUEUE_DEPTH;
      io.reg_write.read_resp_valid_r[i] = true;
      io.reg_write.read_resp_fresh_r[i] = true;
      io.reg_write.read_resp_id_r[i] = io.regs.read_resp_q_id_r[i][head];
      io.reg_write.read_resp_data_r[i] = io.regs.read_resp_q_data_r[i][head];
      io.reg_write.read_resp_q_id_r[i][head] = 0;
      io.reg_write.read_resp_q_data_r[i][head].clear();
      io.reg_write.read_resp_q_head_r[i] =
          static_cast<uint8_t>((head + 1u) % AXI_LLC_READ_RESP_QUEUE_DEPTH);
      io.reg_write.read_resp_q_count_r[i]--;
    }
  }
}

void AXI_LLCRef::drive_write_path() {
  auto try_commit_write_ctx_after_victim = [&](uint8_t master,
                                               bool focused_victim) -> bool {
    if (master >= NUM_WRITE_MASTERS) {
      return false;
    }
    auto &ctx = io.reg_write.write_ctx[master];
    if (!ctx.valid || ctx.bypass || !ctx.cache_pending || !ctx.victim_mem_done ||
        ctx.cache_done) {
      return false;
    }
    if (llc_table_write_busy(io.table_out)) {
      return false;
    }

    AXI_LLCMetaEntry_t meta{};
    meta.tag = ctx.tag;
    meta.flags = static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY);

    io.table_out.data.enable = true;
    io.table_out.data.write = true;
    io.table_out.data.index = ctx.set;
    io.table_out.data.way = ctx.way;

    io.table_out.meta.enable = true;
    io.table_out.meta.write = true;
    io.table_out.meta.index = ctx.set;
    io.table_out.meta.way = ctx.way;
    encode_meta(meta, io.table_out.meta.payload);
    io.table_out.meta.byte_enable.assign(AXI_LLC_META_ENTRY_BYTES, 1);

    io.table_out.valid.enable = true;
    io.table_out.valid.write = true;
    io.table_out.valid.index = ctx.set;
    io.table_out.valid.way = ctx.way;
    encode_valid_bit_payload(config_.ways, ctx.way, true, io.table_out.valid.payload,
                             io.table_out.valid.byte_enable);

    io.table_out.repl.enable = true;
    io.table_out.repl.write = true;
    io.table_out.repl.index = ctx.set;
    io.table_out.repl.payload = build_repl_payload(ctx.repl_next_way);
    io.table_out.repl.byte_enable.assign(AXI_LLC_REPL_BYTES, 1);

    ctx.cache_done = true;
    ctx.cache_pending = false;
    ctx.victim_mem_done = false;
    return true;
  };

  for (uint8_t i = 0; i < NUM_WRITE_MASTERS; ++i) {
    io.ext_out.upstream.write_resp[i].valid = io.regs.write_resp_valid_r[i];
    io.ext_out.upstream.write_resp[i].id = io.regs.write_resp_id_r[i];
    io.ext_out.upstream.write_resp[i].resp = io.regs.write_resp_code_r[i];
    if (io.regs.write_resp_valid_r[i] && io.ext_in.upstream.write_resp[i].ready) {
      io.reg_write.write_resp_valid_r[i] = false;
      io.reg_write.write_resp_id_r[i] = 0;
      io.reg_write.write_resp_code_r[i] = 0;
      io.reg_write.write_resp_line_addr_r[i] = 0;
    }
  }

  if (io.regs.victim_wb_valid_r && io.regs.victim_wb_issued_r &&
      !io.regs.victim_wb_for_write_r &&
      io.regs.victim_wb_mshr_slot_r != AXI_LLC_INVALID_VICTIM_MSHR_SLOT) {
    const uint8_t slot = io.regs.victim_wb_mshr_slot_r;
    const bool slot_in_range =
        slot < config_.mshr_num && slot < MAX_OUTSTANDING;
    const bool slot_valid =
        slot_in_range && io.regs.mshr[slot].valid;
    const bool slot_match =
        slot_valid && io.regs.mshr[slot].victim_dirty &&
        io.regs.mshr[slot].victim_addr == io.regs.victim_wb_addr_r;
    if (!slot_match) {
      uint32_t entry_line = 0;
      uint32_t entry_victim_addr = 0;
      int entry_set = -1;
      int entry_dirty = 0;
      if (slot_valid) {
        entry_line = io.regs.mshr[slot].line_addr;
        entry_victim_addr = io.regs.mshr[slot].victim_addr;
        entry_set = static_cast<int>(io.regs.mshr[slot].set);
        entry_dirty = static_cast<int>(io.regs.mshr[slot].victim_dirty);
      }
      if ((entry_set >= 0 && llc_focus_set(config_, static_cast<uint32_t>(entry_set))) ||
          llc_focus_line(io.regs.victim_wb_addr_r)) {

      }
    }
  }

  io.ext_out.mem.write_resp_ready = true;
  if (io.ext_in.mem.write_resp_valid) {
    if (io.regs.victim_wb_valid_r && io.regs.victim_wb_issued_r) {
      const bool focused_victim =
          llc_focus_line(io.regs.victim_wb_addr_r) ||
          (io.regs.victim_wb_for_write_r &&
           io.regs.victim_wb_write_master_r < NUM_WRITE_MASTERS &&
           llc_focus_line(
               io.regs.write_ctx[io.regs.victim_wb_write_master_r].line_addr));
      io.reg_write.victim_wb_valid_r = false;
      io.reg_write.victim_wb_issued_r = false;
      io.reg_write.victim_wb_for_write_r = false;
      io.reg_write.victim_wb_write_master_r = 0;
      io.reg_write.victim_wb_mshr_slot_r = 0;
      io.reg_write.victim_wb_addr_r = 0;
      io.reg_write.victim_wb_issue_cycle_r = 0;
      io.reg_write.victim_wb_strobe_r.clear();

      if (io.regs.victim_wb_for_write_r) {
        const uint8_t master = io.regs.victim_wb_write_master_r;
        auto &ctx = io.reg_write.write_ctx[master];
        ctx.victim_mem_done = true;
        (void)try_commit_write_ctx_after_victim(master, focused_victim);
      } else {
        const uint8_t slot = io.regs.victim_wb_mshr_slot_r;
        if (slot == AXI_LLC_INVALID_VICTIM_MSHR_SLOT) {
          if (io.regs.read_victim_wb_q_count_r != 0) {
            const uint8_t head =
                io.regs.read_victim_wb_q_head_r %
                AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
            const auto &queued = io.regs.read_victim_wb_q[head];
            if (queued.valid &&
                queued.victim_addr == io.regs.victim_wb_addr_r) {
              io.reg_write.read_victim_wb_q[head] = {};
              io.reg_write.read_victim_wb_q_head_r = static_cast<uint8_t>(
                  (head + 1u) % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
              io.reg_write.read_victim_wb_q_count_r--;
            } else if (llc_focus_line(io.regs.victim_wb_addr_r)) {

            }
          }
        } else {
          int owner_slot = -1;
          if (slot < config_.mshr_num && slot < MAX_OUTSTANDING &&
              io.regs.mshr[slot].valid) {
            const auto &entry = io.regs.mshr[slot];
            if (entry.victim_dirty &&
                entry.victim_addr == io.regs.victim_wb_addr_r) {
              owner_slot = static_cast<int>(slot);
            }
          }
          if (owner_slot < 0) {
            owner_slot =
                find_mshr_by_victim_addr(io.regs, io.regs.victim_wb_addr_r);
          }
          if (owner_slot >= 0) {
            io.reg_write.mshr[owner_slot].victim_writeback_done = true;
            if (static_cast<uint8_t>(owner_slot) != slot &&
                llc_focus_line(io.regs.victim_wb_addr_r)) {

            }
          } else if (slot < config_.mshr_num && slot < MAX_OUTSTANDING &&
                     io.regs.mshr[slot].valid) {
            const auto &entry = io.regs.mshr[slot];
            if (llc_focus_set(config_, entry.set) ||
                llc_focus_line(io.regs.victim_wb_addr_r)) {

            }
          }
        }
      }
    } else {
      const int owner = find_bypass_write_mem_owner(io.regs);
      if (owner >= 0) {
        auto &ctx = io.reg_write.write_ctx[owner];
        ctx.mem_done = true;
        ctx.mem_resp_code = io.ext_in.mem.write_resp;
      }
    }
  }

  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (try_commit_write_ctx_after_victim(master, false)) {
      break;
    }
  }

  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = io.reg_write.write_ctx[master];
    if (!ctx.valid || io.reg_write.write_resp_valid_r[master]) {
      continue;
    }
    const bool done = ctx.bypass ? ctx.mem_done : (ctx.cache_done && !ctx.cache_pending);
    if (!done) {
      continue;
    }
    io.reg_write.write_resp_valid_r[master] = true;
    io.reg_write.write_resp_id_r[master] = ctx.id;
    io.reg_write.write_resp_code_r[master] = ctx.bypass ? ctx.mem_resp_code : 0;
    io.reg_write.write_resp_line_addr_r[master] = ctx.line_addr;
    io.reg_write.write_ctx[master] = {};
  }

  const int bypass_owner = find_bypass_write_mem_owner(io.regs);
  if (!io.reg_write.victim_wb_valid_r &&
      io.regs.read_victim_wb_q_count_r != 0 && bypass_owner < 0) {
    const uint8_t head =
        io.regs.read_victim_wb_q_head_r % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
    const auto &queued = io.regs.read_victim_wb_q[head];
    if (queued.valid) {
      io.reg_write.victim_wb_valid_r = true;
      io.reg_write.victim_wb_issued_r = false;
      io.reg_write.victim_wb_for_write_r = false;
      io.reg_write.victim_wb_write_master_r = 0;
      io.reg_write.victim_wb_mshr_slot_r = AXI_LLC_INVALID_VICTIM_MSHR_SLOT;
      io.reg_write.victim_wb_addr_r = queued.victim_addr;
      io.reg_write.victim_wb_strobe_r = queued.victim_strobe;
    }
  }

  if (io.reg_write.victim_wb_valid_r) {
    if (!io.reg_write.victim_wb_issued_r) {
      if (bypass_owner >= 0) {
        return;
      }
      if (llc_focus_line(io.reg_write.victim_wb_addr_r) ||
          (io.reg_write.victim_wb_for_write_r &&
           io.reg_write.victim_wb_write_master_r < NUM_WRITE_MASTERS &&
           llc_focus_line(
               io.regs.write_ctx[io.reg_write.victim_wb_write_master_r].line_addr))) {

      }
      io.ext_out.mem.write_req_valid = true;
      io.ext_out.mem.write_req_addr = io.reg_write.victim_wb_addr_r;
      io.ext_out.mem.write_req_strobe = io.reg_write.victim_wb_strobe_r;
      io.ext_out.mem.write_req_size = static_cast<uint8_t>(config_.line_bytes - 1);
      io.ext_out.mem.write_req_mode2_ddr_aligned = false;
      io.ext_out.mem.write_req_id = 0;
      if (io.ext_in.mem.write_req_ready) {
        io.reg_write.victim_wb_issued_r = true;
        io.reg_write.victim_wb_issue_cycle_r = static_cast<uint64_t>(sim_time);
      }
    }
    return;
  }

  const int bypass_master = pick_bypass_write_issue_master(io.regs);
  if (bypass_master >= 0) {
    const auto &ctx = io.regs.write_ctx[bypass_master];
    io.ext_out.mem.write_req_valid = true;
    io.ext_out.mem.write_req_addr = ctx.addr;
    io.ext_out.mem.write_req_data = ctx.data;
    io.ext_out.mem.write_req_strobe = ctx.strobe;
    io.ext_out.mem.write_req_size = ctx.total_size;
    io.ext_out.mem.write_req_mode2_ddr_aligned = ctx.mode2_ddr_aligned;
    io.ext_out.mem.write_req_id = ctx.id;
    if (io.ext_in.mem.write_req_ready) {
      io.reg_write.write_ctx[bypass_master].mem_issued = true;
      io.reg_write.write_ctx[bypass_master].mem_issue_cycle =
          static_cast<uint64_t>(sim_time);
    }
  }

  const int write_master = pick_new_write_master(io.regs);
  if (write_master >= 0 &&
      !write_queue_full(io.regs, static_cast<uint8_t>(write_master))) {
    const auto &req = io.ext_in.upstream.write_req[write_master];
    io.ext_out.upstream.write_req[write_master].ready = true;
    if (req.valid) {
      const uint8_t slot =
          io.regs.write_q_tail_r[write_master] % MAX_WRITE_OUTSTANDING;
      auto &entry = io.reg_write.write_q[write_master][slot];
      entry = {};
      entry.valid = true;
      entry.bypass = req.bypass;
      entry.direct_mapped = req.direct_mapped;
      entry.mode2_ddr_aligned = req.mode2_ddr_aligned;
      entry.master = static_cast<uint8_t>(write_master);
      entry.id = req.id;
      entry.total_size = req.total_size;
      entry.addr = req.addr;
      entry.wdata = req.wdata;
      entry.wstrb = req.wstrb;
      io.reg_write.write_q_tail_r[write_master] =
          static_cast<uint8_t>((slot + 1) % MAX_WRITE_OUTSTANDING);
      io.reg_write.write_q_count_r[write_master] =
          static_cast<uint8_t>(io.regs.write_q_count_r[write_master] + 1);
      io.reg_write.rr_write_master_r =
          static_cast<uint8_t>((write_master + 1) % NUM_WRITE_MASTERS);
    }
  }

  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t master = (io.regs.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    if (io.reg_write.write_ctx[master].valid || io.reg_write.write_resp_valid_r[master]) {
      continue;
    }
    const auto *entry = write_queue_front(io.reg_write, static_cast<uint8_t>(master));
    if (entry == nullptr) {
      continue;
    }
    if (victim_line_pending(io.reg_write, line_addr(config_, entry->addr))) {
      continue;
    }
    auto &ctx = io.reg_write.write_ctx[master];
    ctx = {};
    ctx.valid = true;
    ctx.bypass = entry->bypass;
    ctx.direct_mapped = entry->direct_mapped;
    ctx.mode2_ddr_aligned = entry->mode2_ddr_aligned;
    ctx.lookup_pending = true;
    ctx.id = entry->id;
    ctx.total_size = entry->total_size;
    ctx.addr = entry->addr;
    ctx.line_addr = line_addr(config_, entry->addr);
    ctx.data = entry->wdata;
    ctx.strobe = entry->wstrb;
    if (llc_focus_line(ctx.line_addr)) {


    }
    io.reg_write.write_q[master][io.reg_write.write_q_head_r[master] %
                                 MAX_WRITE_OUTSTANDING] = {};
    io.reg_write.write_q_head_r[master] =
        static_cast<uint8_t>((io.reg_write.write_q_head_r[master] + 1) %
                             MAX_WRITE_OUTSTANDING);
    io.reg_write.write_q_count_r[master] =
        static_cast<uint8_t>(io.reg_write.write_q_count_r[master] - 1);
  }

  (void)try_launch_pending_write_lookup();
}

bool AXI_LLCRef::try_launch_pending_write_lookup() {
  if (io.reg_write.lookup_valid_r) {
    return false;
  }
  const int lookup_master = pick_write_lookup_master(io.reg_write);
  if (lookup_master < 0) {
    return false;
  }

  const auto &ctx = io.reg_write.write_ctx[lookup_master];
  io.reg_write.lookup_valid_r = true;
  io.reg_write.lookup_issued_r = false;
  io.reg_write.lookup_addr_r = ctx.addr;
  io.reg_write.lookup_size_r = ctx.total_size;
  io.reg_write.lookup_master_r = static_cast<uint8_t>(lookup_master);
  io.reg_write.lookup_id_r = ctx.id;
  io.reg_write.lookup_is_prefetch_r = false;
  io.reg_write.lookup_is_invalidate_r = false;
  io.reg_write.lookup_is_write_r = true;
  io.reg_write.lookup_is_bypass_r = ctx.bypass;
  io.reg_write.lookup_is_direct_mapped_r = ctx.direct_mapped;
  io.reg_write.lookup_is_mode2_ddr_aligned_r = ctx.mode2_ddr_aligned;
  io.reg_write.write_ctx[lookup_master].lookup_pending = false;
  io.reg_write.state = AXI_LLCState::kLookup;
  io.reg_write.rr_write_master_r =
      static_cast<uint8_t>((lookup_master + 1) % NUM_WRITE_MASTERS);
  return true;
}

void AXI_LLCRef::accept_maintenance_request() {
  io.ext_out.mem.invalidate_line_accepted = false;
  if (!io.ext_in.mem.invalidate_line_valid || io.ext_in.mem.invalidate_all) {
    return;
  }
  const uint32_t target_line =
      line_addr(config_, io.ext_in.mem.invalidate_line_addr);
  if (!can_accept_invalidate_line_now(target_line)) {
    return;
  }
  if (io.reg_write.lookup_valid_r) {
    return;
  }
  io.reg_write.lookup_valid_r = true;
  io.reg_write.lookup_issued_r = false;
  io.reg_write.lookup_addr_r = io.ext_in.mem.invalidate_line_addr;
  io.reg_write.lookup_size_r = 0;
  io.reg_write.lookup_master_r = 0;
  io.reg_write.lookup_id_r = 0;
  io.reg_write.lookup_is_prefetch_r = false;
  io.reg_write.lookup_is_invalidate_r = true;
  io.reg_write.lookup_is_write_r = false;
  io.reg_write.lookup_is_bypass_r = false;
  io.reg_write.lookup_is_direct_mapped_r = false;
  io.reg_write.lookup_is_mode2_ddr_aligned_r = false;
  io.reg_write.state = AXI_LLCState::kLookup;
  io.ext_out.mem.invalidate_line_accepted = true;
}

void AXI_LLCRef::drive_lookup_request() {
  if (!io.regs.lookup_valid_r || !io.reg_write.lookup_valid_r) {
    return;
  }
  if (io.regs.lookup_issued_r) {
    return;
  }
  const uint32_t lookup_line = line_addr(config_, io.reg_write.lookup_addr_r);
  if (victim_line_pending(io.reg_write, lookup_line)) {
    return;
  }
  if (io.table_out.data.enable || io.table_out.meta.enable ||
      io.table_out.valid.enable ||
      io.table_out.repl.enable) {
    return;
  }
  if (!io.reg_write.lookup_is_prefetch_r &&
      !io.reg_write.lookup_is_direct_mapped_r &&
      find_mshr_by_line_addr(io.reg_write, lookup_line) >= 0) {
    return;
  }
  const uint32_t index = set_index(config_, io.reg_write.lookup_addr_r);
  io.table_out.data.enable = true;
  io.table_out.meta.enable = true;
  io.table_out.valid.enable = true;
  io.table_out.repl.enable = true;
  io.table_out.data.index = index;
  io.table_out.meta.index = index;
  io.table_out.valid.index = index;
  io.table_out.repl.index = index;
  io.reg_write.lookup_issued_r = true;
  io.reg_write.state = AXI_LLCState::kLookup;
  if (llc_focus_line(lookup_line)) {


    std::fflush(stdout);
  }
}

bool AXI_LLCRef::try_complete_lookup() {
  const bool valid_table_ready =
      io.lookup_in.valid_valid &&
      io.lookup_in.valid.size() >= valid_row_bytes(config_);
  if (!io.regs.lookup_valid_r || !io.regs.lookup_issued_r ||
      !io.lookup_in.data_valid || !io.lookup_in.meta_valid || !valid_table_ready ||
      !io.lookup_in.repl_valid) {
    return false;
  }

  const bool is_prefetch_lookup = io.regs.lookup_is_prefetch_r;
  const bool is_invalidate_lookup = io.regs.lookup_is_invalidate_r;
  const bool is_write_lookup = io.regs.lookup_is_write_r;
  const bool is_bypass_lookup = io.regs.lookup_is_bypass_r;
  const bool is_direct_lookup = io.regs.lookup_is_direct_mapped_r;
  const uint8_t lookup_slot_id = io.regs.lookup_id_r;
  const uint32_t set = set_index(config_, io.regs.lookup_addr_r);
  const uint32_t tag = tag_of(config_, io.regs.lookup_addr_r);
  const uint32_t req_line_addr = line_addr(config_, io.regs.lookup_addr_r);
  const bool table_busy = llc_table_write_busy(io.table_out);
  auto stall_lookup_on_table_busy = [&]() -> bool {
    io.reg_write.lookup_issued_r = false;
    io.reg_write.state = AXI_LLCState::kLookup;
    return true;
  };
  auto stall_lookup_on_victim_wb_busy = [&]() -> bool {
    io.reg_write.lookup_issued_r = false;
    io.reg_write.state = AXI_LLCState::kLookup;
    return true;
  };
  if (is_direct_lookup) {
    uint32_t direct_set = 0;
    uint8_t direct_way = 0;
    const bool mapped_slot_valid =
        direct_mapped_coords(io.regs.lookup_addr_r, &direct_set, &direct_way);
    if (is_write_lookup) {
      auto &ctx = io.reg_write.write_ctx[io.regs.lookup_master_r];
      if (table_busy) {
        return stall_lookup_on_table_busy();
      }


      if (mapped_slot_valid) {
        io.table_out.data.enable = true;
        io.table_out.data.write = true;
        io.table_out.data.index = direct_set;
        io.table_out.data.way = direct_way;
        io.table_out.valid.enable = true;
        io.table_out.valid.write = true;
        io.table_out.valid.index = direct_set;
        io.table_out.valid.way = direct_way;
        encode_valid_bit_payload(config_.ways, direct_way, true,
                                 io.table_out.valid.payload,
                                 io.table_out.valid.byte_enable);
      }
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.lookup_is_direct_mapped_r = false;
      io.reg_write.lookup_is_mode2_ddr_aligned_r = false;
      ctx.cache_done = true;
      io.reg_write.state = AXI_LLCState::kIdle;
      return true;
    }

    WideReadData_t resp_data{};
    resp_data.clear();
    if (mapped_slot_valid &&
        decode_valid_bit(io.lookup_in.valid, direct_way, false)) {
      resp_data.clear();
    }
    const uint8_t master = io.regs.lookup_master_r;
    if (!enqueue_read_response(master, io.regs.lookup_id_r, resp_data)) {
      io.reg_write.lookup_issued_r = false;
      io.reg_write.state = AXI_LLCState::kLookup;
      return true;
    }
    io.reg_write.lookup_valid_r = false;
    io.reg_write.lookup_issued_r = false;
    io.reg_write.lookup_is_prefetch_r = false;
    io.reg_write.lookup_is_invalidate_r = false;
    io.reg_write.lookup_is_write_r = false;
    io.reg_write.lookup_is_bypass_r = false;
    io.reg_write.lookup_is_direct_mapped_r = false;
    io.reg_write.lookup_is_mode2_ddr_aligned_r = false;
    io.reg_write.state = AXI_LLCState::kIdle;
    return true;
  }

  int hit_way = -1;
  int first_invalid_way = -1;
  AXI_LLCMetaEntry_t hit_meta{};
  (void)line_has_valid_meta(io.lookup_in.valid, io.lookup_in.meta,
                            tag, &hit_way, &first_invalid_way,
                            &hit_meta);

  if (hit_way >= 0) {
    if (is_write_lookup) {
      if (table_busy) {
        return stall_lookup_on_table_busy();
      }
      auto &ctx = io.reg_write.write_ctx[io.regs.lookup_master_r];


      if (!is_bypass_lookup) {
        const uint32_t limit =
            std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
        for (uint32_t slot = 0; slot < limit; ++slot) {
          const auto &entry_prev = io.regs.mshr[slot];
          if (!entry_prev.valid || entry_prev.bypass || entry_prev.is_write ||
              entry_prev.refill_committed) {
            continue;
          }
          if (entry_prev.set != set ||
              entry_prev.way != static_cast<uint8_t>(hit_way)) {
            continue;
          }
          if (entry_prev.line_addr == req_line_addr) {
            continue;
          }
          auto &entry = io.reg_write.mshr[slot];
          entry.victim_dirty = true;
          entry.victim_writeback_done = false;
          entry.victim_addr = req_line_addr;
          if (llc_focus_line(req_line_addr) ||
              llc_focus_line(entry_prev.line_addr)) {

          }
        }
      }
      AXI_LLCMetaEntry_t meta{};
      meta.tag = tag;
      const uint8_t preserved_dirty =
          static_cast<uint8_t>(hit_meta.flags & AXI_LLC_META_DIRTY);
      meta.flags = static_cast<uint8_t>(AXI_LLC_META_VALID |
                                        (is_bypass_lookup ? preserved_dirty
                                                          : AXI_LLC_META_DIRTY));
      io.table_out.data.enable = true;
      io.table_out.data.write = true;
      io.table_out.data.index = set;
      io.table_out.data.way = static_cast<uint32_t>(hit_way);
      io.table_out.meta.enable = true;
      io.table_out.meta.write = true;
      io.table_out.meta.index = set;
      io.table_out.meta.way = static_cast<uint32_t>(hit_way);
      encode_meta(meta, io.table_out.meta.payload);
      io.table_out.meta.byte_enable.assign(AXI_LLC_META_ENTRY_BYTES, 1);
      io.table_out.valid.enable = true;
      io.table_out.valid.write = true;
      io.table_out.valid.index = set;
      io.table_out.valid.way = static_cast<uint32_t>(hit_way);
      encode_valid_bit_payload(config_.ways, static_cast<uint32_t>(hit_way), true,
                               io.table_out.valid.payload,
                               io.table_out.valid.byte_enable);
      io.table_out.repl.enable = true;
      io.table_out.repl.write = true;
      io.table_out.repl.index = set;
      io.table_out.repl.payload =
          build_repl_payload(static_cast<uint32_t>((hit_way + 1) % config_.ways));
      io.table_out.repl.byte_enable.assign(AXI_LLC_REPL_BYTES, 1);
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.dirty_line_count_r = update_dirty_line_count(
          io.regs.dirty_line_count_r,
          (hit_meta.flags & AXI_LLC_META_DIRTY) != 0,
          (meta.flags & AXI_LLC_META_DIRTY) != 0);
      ctx.cache_done = true;
      io.reg_write.state = AXI_LLCState::kIdle;
      return true;
    }
    if (is_invalidate_lookup) {
      if (table_busy) {
        return stall_lookup_on_table_busy();
      }
      io.table_out.valid.enable = true;
      io.table_out.valid.write = true;
      io.table_out.valid.index = set;
      io.table_out.valid.way = static_cast<uint32_t>(hit_way);
      encode_valid_bit_payload(config_.ways, static_cast<uint32_t>(hit_way), false,
                               io.table_out.valid.payload,
                               io.table_out.valid.byte_enable);
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.dirty_line_count_r = update_dirty_line_count(
          io.regs.dirty_line_count_r,
          (hit_meta.flags & AXI_LLC_META_DIRTY) != 0, false);
      io.reg_write.state = AXI_LLCState::kIdle;
      return true;
    }
    if (is_prefetch_lookup) {
      if (lookup_slot_id < AXI_LLC_MAX_PREFETCH_QUEUE) {
        io.reg_write.prefetch_q[lookup_slot_id] = {};
      }
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.state = AXI_LLCState::kIdle;
      return true;
    }
    if (is_bypass_lookup) {
      const uint8_t master = io.regs.lookup_master_r;
      if (!enqueue_read_response(master, io.regs.lookup_id_r,
                                 WideReadData_t{})) {
        io.reg_write.lookup_issued_r = false;
        io.reg_write.state = AXI_LLCState::kLookup;
        return true;
      }
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.state = AXI_LLCState::kIdle;
      return true;
    }
    if (table_busy) {
      return stall_lookup_on_table_busy();
    }
    const uint8_t master = io.regs.lookup_master_r;
    if (!enqueue_read_response(master, io.regs.lookup_id_r, WideReadData_t{})) {
      io.reg_write.lookup_issued_r = false;
      io.reg_write.state = AXI_LLCState::kLookup;
      return true;
    }
    io.table_out.repl.enable = true;
    io.table_out.repl.write = true;
    io.table_out.repl.index = set;
    io.table_out.repl.payload = build_repl_payload(
        static_cast<uint32_t>((hit_way + 1) % config_.ways));
    io.table_out.repl.byte_enable.assign(AXI_LLC_REPL_BYTES, 1);
    if ((hit_meta.flags & AXI_LLC_META_PREFETCH) != 0) {
      AXI_LLCMetaEntry_t meta = hit_meta;
      meta.flags = static_cast<uint8_t>(meta.flags & ~AXI_LLC_META_PREFETCH);
      io.table_out.meta.enable = true;
      io.table_out.meta.write = true;
      io.table_out.meta.index = set;
      io.table_out.meta.way = static_cast<uint32_t>(hit_way);
      encode_meta(meta, io.table_out.meta.payload);
      io.table_out.meta.byte_enable.assign(AXI_LLC_META_ENTRY_BYTES, 1);
    }
    io.reg_write.lookup_valid_r = false;
    io.reg_write.lookup_issued_r = false;
    io.reg_write.lookup_is_prefetch_r = false;
    io.reg_write.lookup_is_invalidate_r = false;
    io.reg_write.lookup_is_write_r = false;
    io.reg_write.lookup_is_bypass_r = false;
    io.reg_write.state = AXI_LLCState::kIdle;
    perf_count_read_hit(io.reg_write.perf, master);
    return true;
  }

  if (is_write_lookup) {
    auto &ctx = io.reg_write.write_ctx[io.regs.lookup_master_r];
    if (is_bypass_lookup) {
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      ctx.cache_done = true;
      io.reg_write.state = AXI_LLCState::kIdle;
      return true;
    }
    const uint32_t repl_way_raw = decode_repl_way(io.lookup_in.repl);
    const int victim_way_idx = pick_unreserved_way(io.regs, set,
                                                   first_invalid_way,
                                                   repl_way_raw, req_line_addr);
    if (victim_way_idx < 0) {
      io.reg_write.lookup_issued_r = false;
      io.reg_write.state = AXI_LLCState::kMiss;
      return true;
    }
    const uint8_t victim_way = static_cast<uint8_t>(victim_way_idx);
    const auto victim_meta =
        decode_meta(io.lookup_in.meta, static_cast<uint32_t>(victim_way));
    const bool victim_valid =
        decode_valid_bit(io.lookup_in.valid, victim_way, false);
    const bool victim_dirty = victim_valid &&
                              ((victim_meta.flags & AXI_LLC_META_DIRTY) != 0);
    const bool full_line_write =
        is_full_line_write(config_, ctx.addr, ctx.strobe, ctx.total_size);
    const uint32_t repl_next_way =
        static_cast<uint32_t>((victim_way + 1) % config_.ways);
    if (!full_line_write) {
      const int merge_slot = find_mshr_by_line_addr(io.regs, req_line_addr);
      if (merge_slot >= 0) {
        io.reg_write.lookup_issued_r = false;
        io.reg_write.state = AXI_LLCState::kMiss;
        return true;
      }
      const int free_slot = find_free_mshr(io.regs);
      if (free_slot < 0) {
        io.reg_write.lookup_issued_r = false;
        io.reg_write.state = AXI_LLCState::kMiss;
        return true;
      }
      auto &entry = io.reg_write.mshr[free_slot];
      entry = {};
      entry.valid = true;
      entry.bypass = false;
      entry.is_prefetch = false;
      entry.is_write = true;
      entry.prefetch_train = false;
      entry.addr = io.regs.lookup_addr_r;
      entry.line_addr = req_line_addr;
      entry.set = set;
      entry.tag = tag;
      entry.way = victim_way;
      entry.total_size = ctx.total_size;
      entry.master = io.regs.lookup_master_r;
      entry.id = io.regs.lookup_id_r;
      entry.epoch = io.regs.invalidate_epoch_r;
      entry.victim_dirty = victim_dirty;
      entry.victim_writeback_done = !victim_dirty;
      if (victim_dirty) {
        entry.victim_addr = build_line_addr_from_tag_set(config_, victim_meta.tag, set);
      } else {
        entry.victim_addr = 0;
      }
      ctx.cache_pending = true;
      ctx.set = set;
      ctx.way = victim_way;
      ctx.repl_next_way = repl_next_way;
      ctx.tag = tag;
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.state = AXI_LLCState::kMiss;
      return true;
    }
    AXI_LLCMetaEntry_t meta{};
    meta.tag = tag;
    meta.flags = static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY);
    if (victim_dirty) {
      if (io.reg_write.victim_wb_valid_r) {
        return stall_lookup_on_victim_wb_busy();
      }
      io.reg_write.victim_wb_valid_r = true;
      io.reg_write.victim_wb_issued_r = false;
      io.reg_write.victim_wb_for_write_r = true;
      io.reg_write.victim_wb_write_master_r = io.regs.lookup_master_r;
      io.reg_write.victim_wb_mshr_slot_r = 0;
      io.reg_write.victim_wb_addr_r =
          build_line_addr_from_tag_set(config_, victim_meta.tag, set);
      io.reg_write.victim_wb_issue_cycle_r = 0;
      io.reg_write.victim_wb_strobe_r = full_line_strobe(config_);
      ctx.cache_pending = true;
      ctx.victim_dirty = true;
      ctx.victim_addr = io.reg_write.victim_wb_addr_r;
      ctx.set = set;
      ctx.way = victim_way;
      ctx.repl_next_way = repl_next_way;
      ctx.tag = tag;
    } else {
      if (table_busy) {
        return stall_lookup_on_table_busy();
      }
      io.table_out.data.enable = true;
      io.table_out.data.write = true;
      io.table_out.data.index = set;
      io.table_out.data.way = victim_way;
      io.table_out.meta.enable = true;
      io.table_out.meta.write = true;
      io.table_out.meta.index = set;
      io.table_out.meta.way = victim_way;
      encode_meta(meta, io.table_out.meta.payload);
      io.table_out.meta.byte_enable.assign(AXI_LLC_META_ENTRY_BYTES, 1);
      io.table_out.valid.enable = true;
      io.table_out.valid.write = true;
      io.table_out.valid.index = set;
      io.table_out.valid.way = victim_way;
      encode_valid_bit_payload(config_.ways, victim_way, true,
                               io.table_out.valid.payload,
                               io.table_out.valid.byte_enable);
      io.table_out.repl.enable = true;
      io.table_out.repl.write = true;
      io.table_out.repl.index = set;
      io.table_out.repl.payload = build_repl_payload(repl_next_way);
      io.table_out.repl.byte_enable.assign(AXI_LLC_REPL_BYTES, 1);
      io.reg_write.dirty_line_count_r = update_dirty_line_count(
          io.regs.dirty_line_count_r, victim_dirty, true);
      ctx.cache_done = true;
    }
    io.reg_write.lookup_valid_r = false;
    io.reg_write.lookup_issued_r = false;
    io.reg_write.lookup_is_prefetch_r = false;
    io.reg_write.lookup_is_invalidate_r = false;
    io.reg_write.lookup_is_write_r = false;
    io.reg_write.lookup_is_bypass_r = false;
    io.reg_write.state = AXI_LLCState::kIdle;
    return true;
  }

  if (is_invalidate_lookup) {
    io.reg_write.lookup_valid_r = false;
    io.reg_write.lookup_issued_r = false;
    io.reg_write.lookup_is_prefetch_r = false;
    io.reg_write.lookup_is_invalidate_r = false;
    io.reg_write.lookup_is_write_r = false;
    io.reg_write.lookup_is_bypass_r = false;
    io.reg_write.state = AXI_LLCState::kIdle;
    return true;
  }

  const int merge_slot = find_mshr_by_line_addr(io.regs, req_line_addr);
  if (merge_slot >= 0) {
    if (is_prefetch_lookup) {
      if (lookup_slot_id < AXI_LLC_MAX_PREFETCH_QUEUE) {
        io.reg_write.prefetch_q[lookup_slot_id] = {};
      }
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.state = AXI_LLCState::kMiss;
      return true;
    }
    io.reg_write.lookup_issued_r = false;
    io.reg_write.state = AXI_LLCState::kMiss;
    return true;
  }

  const int free_slot = find_free_mshr(io.regs);
  if (free_slot < 0 || (is_prefetch_lookup && !can_allocate_prefetch_mshr(io.regs))) {
    if (is_prefetch_lookup) {
      if (lookup_slot_id < AXI_LLC_MAX_PREFETCH_QUEUE) {
        io.reg_write.prefetch_q[lookup_slot_id] = {};
      }
      io.reg_write.lookup_valid_r = false;
      io.reg_write.lookup_issued_r = false;
      io.reg_write.lookup_is_prefetch_r = false;
      io.reg_write.lookup_is_invalidate_r = false;
      io.reg_write.lookup_is_write_r = false;
      io.reg_write.lookup_is_bypass_r = false;
      io.reg_write.state = AXI_LLCState::kMiss;
      return true;
    }
    io.reg_write.lookup_issued_r = false;
    io.reg_write.state = AXI_LLCState::kMiss;
    return true;
  }

  const bool bypass_dcache_read_miss =
      AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC != 0 && !is_prefetch_lookup &&
      !is_bypass_lookup && io.regs.lookup_master_r == MASTER_DCACHE_R;
  if (bypass_dcache_read_miss) {
    auto &entry = io.reg_write.mshr[free_slot];
    entry = {};
    entry.valid = true;
    entry.bypass = true;
    entry.is_prefetch = false;
    entry.prefetch_train = false;
    entry.addr = line_addr(config_, io.regs.lookup_addr_r);
    entry.line_addr = line_addr(config_, io.regs.lookup_addr_r);
    entry.total_size = static_cast<uint8_t>(config_.line_bytes - 1);
    entry.master = io.regs.lookup_master_r;
    entry.id = io.regs.lookup_id_r;
    entry.epoch = io.regs.invalidate_epoch_r;
    io.reg_write.lookup_valid_r = false;
    io.reg_write.lookup_issued_r = false;
    io.reg_write.lookup_is_prefetch_r = false;
    io.reg_write.lookup_is_invalidate_r = false;
    io.reg_write.lookup_is_write_r = false;
    io.reg_write.lookup_is_bypass_r = false;
    io.reg_write.state = AXI_LLCState::kMiss;
    perf_count_read_miss(io.reg_write.perf, entry.master);
    return true;
  }

  const uint32_t repl_way_raw = decode_repl_way(io.lookup_in.repl);
  const int victim_way_idx = pick_unreserved_way(io.regs, set,
                                                 first_invalid_way,
                                                 repl_way_raw, req_line_addr);
  if (victim_way_idx < 0) {
    io.reg_write.lookup_issued_r = false;
    io.reg_write.state = AXI_LLCState::kMiss;
    return true;
  }
  const uint8_t victim_way = static_cast<uint8_t>(victim_way_idx);
  auto &entry = io.reg_write.mshr[free_slot];
  entry = {};
  entry.valid = true;
  entry.bypass = is_bypass_lookup;
  entry.is_prefetch = is_prefetch_lookup;
  entry.prefetch_train = !is_prefetch_lookup;
  entry.mode2_ddr_aligned = io.regs.lookup_is_mode2_ddr_aligned_r;
  entry.addr = io.regs.lookup_addr_r;
  entry.line_addr = is_bypass_lookup ? io.regs.lookup_addr_r
                                     : line_addr(config_, io.regs.lookup_addr_r);
  entry.set = set;
  entry.tag = tag;
  entry.way = victim_way;
  entry.total_size = io.regs.lookup_size_r;
  entry.master = io.regs.lookup_master_r;
  entry.id = is_prefetch_lookup ? 0 : io.regs.lookup_id_r;
  entry.epoch = io.regs.invalidate_epoch_r;
  const auto victim_meta =
      decode_meta(io.lookup_in.meta, static_cast<uint32_t>(victim_way));
  const bool victim_valid =
      decode_valid_bit(io.lookup_in.valid, victim_way, false);
  entry.victim_dirty =
      victim_valid && ((victim_meta.flags & AXI_LLC_META_DIRTY) != 0);
  entry.victim_writeback_done = !entry.victim_dirty;
  if (entry.victim_dirty) {
    entry.victim_addr = build_line_addr_from_tag_set(config_, victim_meta.tag, set);
  } else {
    entry.victim_addr = 0;
  }



  io.reg_write.lookup_valid_r = false;
  io.reg_write.lookup_issued_r = false;
  io.reg_write.lookup_is_prefetch_r = false;
  io.reg_write.lookup_is_invalidate_r = false;
  io.reg_write.lookup_is_write_r = false;
  io.reg_write.lookup_is_bypass_r = false;
  io.reg_write.state = AXI_LLCState::kMiss;
  if (is_prefetch_lookup) {
    if (lookup_slot_id < AXI_LLC_MAX_PREFETCH_QUEUE) {
      io.reg_write.prefetch_q[lookup_slot_id] = {};
    }
  } else {
    perf_count_read_miss(io.reg_write.perf, entry.master);
  }
  return true;
}

void AXI_LLCRef::drive_mem_read_path() {
  io.ext_out.mem.read_resp_ready = true;

  if (io.ext_in.mem.read_resp_valid) {
    const int slot = find_mshr_by_mem_id(io.regs, io.ext_in.mem.read_resp_id);
    if (slot >= 0) {
      const auto &entry_prev = io.regs.mshr[slot];

      if (true != 0 && entry_prev.bypass &&
          enqueue_read_response(entry_prev.master, entry_prev.id,
                                io.ext_in.mem.read_resp_data)) {
        io.reg_write.mshr[slot] = {};
        io.reg_write.state =
            io.regs.lookup_valid_r ? AXI_LLCState::kLookup : AXI_LLCState::kIdle;
      } else {
        auto &entry = io.reg_write.mshr[slot];
        entry.refill_valid = true;
        entry.refill_data = io.ext_in.mem.read_resp_data;
        entry.refill_committed = entry.bypass;
        io.reg_write.state = AXI_LLCState::kRefill;
      }
    }
  }

  int slot = pick_mem_issue_slot(io.regs);
  bool using_reg_write_slot = false;
  if (slot < 0 && true != 0) {
    const uint32_t mshr_limit =
        std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
    for (uint32_t cand = 0; cand < mshr_limit; ++cand) {
      const auto &entry = io.reg_write.mshr[cand];
      if (!entry.valid || !entry.bypass || entry.mem_req_issued ||
          entry.refill_valid) {
        continue;
      }
      if (cand < MAX_OUTSTANDING && io.regs.mshr[cand].valid) {
        continue;
      }
      slot = static_cast<int>(cand);
      using_reg_write_slot = true;
      break;
    }
  }
  if (slot >= 0) {
    const auto &entry =
        using_reg_write_slot ? io.reg_write.mshr[slot] : io.regs.mshr[slot];
    io.ext_out.mem.read_req_valid = true;
    io.ext_out.mem.read_req_addr = entry.line_addr;
    io.ext_out.mem.read_req_size = entry.bypass
                                       ? entry.total_size
                                       : static_cast<uint8_t>(config_.line_bytes - 1);
    io.ext_out.mem.read_req_mode2_ddr_aligned =
        entry.bypass && entry.mode2_ddr_aligned;
    io.ext_out.mem.read_req_id = static_cast<uint8_t>(slot);
    if (io.ext_in.mem.read_req_ready) {
      io.reg_write.mshr[slot].mem_req_issued = true;
      io.reg_write.mshr[slot].mem_issue_cycle = static_cast<uint64_t>(sim_time);
    }
  }

  const int commit_slot = pick_refill_commit_slot(io.regs);
  if (commit_slot < 0) {
    return;
  }

  auto &entry = io.reg_write.mshr[commit_slot];
  const bool stale_refill_epoch = entry.epoch != io.regs.invalidate_epoch_r;
  auto read_victim_snapshot_in_global_slot = [&]() -> bool {
    return io.reg_write.victim_wb_valid_r && !io.reg_write.victim_wb_for_write_r &&
           io.reg_write.victim_wb_addr_r == entry.victim_addr &&
           io.reg_write.victim_wb_mshr_slot_r == static_cast<uint8_t>(commit_slot);
  };
  auto ensure_read_victim_snapshot_externalized = [&]() -> bool {
    if (!entry.victim_dirty || entry.victim_writeback_done) {
      return true;
    }
    if (victim_snapshot_waits_for_write_resolution(io.regs, entry.victim_addr)) {
      return false;
    }
    if (read_victim_snapshot_in_global_slot() ||
        read_victim_snapshot_present(io.reg_write, entry.victim_addr)) {
      return true;
    }
    // Once the dirty victim snapshot is captured in the dedicated victim slot
    // or queue, refill commit no longer depends on write-path cache_done.
    if (!io.reg_write.victim_wb_valid_r) {
      io.reg_write.victim_wb_valid_r = true;
      io.reg_write.victim_wb_issued_r = false;
      io.reg_write.victim_wb_for_write_r = false;
      io.reg_write.victim_wb_write_master_r = 0;
      io.reg_write.victim_wb_mshr_slot_r = static_cast<uint8_t>(commit_slot);
      io.reg_write.victim_wb_addr_r = entry.victim_addr;
      io.reg_write.victim_wb_issue_cycle_r = 0;
      io.reg_write.victim_wb_strobe_r = full_line_strobe(config_);
      return true;
    }
    return enqueue_read_victim_snapshot(static_cast<uint8_t>(commit_slot),
                                        entry.victim_addr,
                                        full_line_strobe(config_));
  };
  auto log_active_victim_slot_clear = [&](const char *reason) {
    if (!(io.regs.victim_wb_valid_r && io.regs.victim_wb_issued_r &&
          !io.regs.victim_wb_for_write_r)) {
      return;
    }
    const uint8_t victim_slot = io.regs.victim_wb_mshr_slot_r;
    if (victim_slot != static_cast<uint8_t>(commit_slot)) {
      return;
    }
    if (!(llc_focus_line(entry.line_addr) || llc_focus_line(entry.victim_addr))) {
      return;
    }

  };


  if (entry.refill_valid && stale_refill_epoch && !entry.is_write) {
    if (!ensure_read_victim_snapshot_externalized()) {
      return;
    }
    if (entry.is_prefetch) {
      log_active_victim_slot_clear("stale_epoch_prefetch");
      io.reg_write.mshr[commit_slot] = {};
      io.reg_write.state = io.regs.lookup_valid_r ? AXI_LLCState::kLookup
                                                  : AXI_LLCState::kIdle;
      return;
    }

    WideReadData_t resp_data = entry.refill_data;
    if (!entry.bypass) {
      resp_data.clear();
    }
    if (!enqueue_read_response(entry.master, entry.id, resp_data)) {
      return;
    }

    log_active_victim_slot_clear("stale_epoch_resp");
    if (io.reg_write.victim_wb_valid_r && !io.reg_write.victim_wb_for_write_r &&
        io.reg_write.victim_wb_mshr_slot_r == static_cast<uint8_t>(commit_slot)) {
      io.reg_write.victim_wb_mshr_slot_r = AXI_LLC_INVALID_VICTIM_MSHR_SLOT;
    }
    io.reg_write.mshr[commit_slot] = {};
    io.reg_write.state = io.regs.lookup_valid_r ? AXI_LLCState::kLookup
                                                : AXI_LLCState::kIdle;
    return;
  }

  if (!entry.refill_committed && !entry.bypass) {
    if (!ensure_read_victim_snapshot_externalized()) {
      return;
    }
    if (llc_table_write_busy(io.table_out)) {
      io.reg_write.state = AXI_LLCState::kRefill;
      return;
    }
    AXI_LLCMetaEntry_t meta{};
    meta.tag = entry.tag;
    meta.flags = static_cast<uint8_t>(AXI_LLC_META_VALID |
                                      (entry.is_prefetch ? AXI_LLC_META_PREFETCH : 0));
    if (entry.is_write) {
      auto &ctx = io.reg_write.write_ctx[entry.master];


      meta.flags = static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY);
    }

    io.table_out.data.enable = true;
    io.table_out.data.write = true;
    io.table_out.data.index = entry.set;
    io.table_out.data.way = entry.way;

    io.table_out.meta.enable = true;
    io.table_out.meta.write = true;
    io.table_out.meta.index = entry.set;
    io.table_out.meta.way = entry.way;
    encode_meta(meta, io.table_out.meta.payload);
    io.table_out.meta.byte_enable.assign(AXI_LLC_META_ENTRY_BYTES, 1);

    io.table_out.valid.enable = true;
    io.table_out.valid.write = true;
    io.table_out.valid.index = entry.set;
    io.table_out.valid.way = entry.way;
    encode_valid_bit_payload(config_.ways, entry.way, true,
                             io.table_out.valid.payload,
                             io.table_out.valid.byte_enable);

    io.table_out.repl.enable = true;
    io.table_out.repl.write = true;
    io.table_out.repl.index = entry.set;
    io.table_out.repl.payload =
        build_repl_payload(static_cast<uint32_t>((entry.way + 1) % config_.ways));
    io.table_out.repl.byte_enable.assign(AXI_LLC_REPL_BYTES, 1);

    if (entry.is_write) {
      io.reg_write.write_ctx[entry.master].cache_done = true;
      io.reg_write.write_ctx[entry.master].cache_pending = false;
      io.reg_write.dirty_line_count_r = update_dirty_line_count(
          io.regs.dirty_line_count_r, entry.victim_dirty, true);
      log_active_victim_slot_clear("write_refill_commit");
      io.reg_write.mshr[commit_slot] = {};
      io.reg_write.state = io.regs.lookup_valid_r ? AXI_LLCState::kLookup
                                                  : AXI_LLCState::kIdle;
    } else {
      io.reg_write.dirty_line_count_r = update_dirty_line_count(
          io.regs.dirty_line_count_r, entry.victim_dirty, false);
      entry.refill_committed = true;
    }
    return;
  }

  if (entry.is_prefetch) {
    log_active_victim_slot_clear("prefetch_resp");
    if (io.reg_write.victim_wb_valid_r && !io.reg_write.victim_wb_for_write_r &&
        io.reg_write.victim_wb_mshr_slot_r == static_cast<uint8_t>(commit_slot)) {
      io.reg_write.victim_wb_mshr_slot_r = AXI_LLC_INVALID_VICTIM_MSHR_SLOT;
    }
    io.reg_write.mshr[commit_slot] = {};
    io.reg_write.state = io.regs.lookup_valid_r ? AXI_LLCState::kLookup
                                                : AXI_LLCState::kIdle;
    return;
  }

  WideReadData_t resp_data = entry.refill_data;
  if (!entry.bypass) {
    resp_data.clear();
  }
  if (!enqueue_read_response(entry.master, entry.id, resp_data)) {
    return;
  }

  try_schedule_prefetch(entry);
  log_active_victim_slot_clear("refill_resp");
  if (io.reg_write.victim_wb_valid_r && !io.reg_write.victim_wb_for_write_r &&
      io.reg_write.victim_wb_mshr_slot_r == static_cast<uint8_t>(commit_slot)) {
    io.reg_write.victim_wb_mshr_slot_r = AXI_LLC_INVALID_VICTIM_MSHR_SLOT;
  }
  io.reg_write.mshr[commit_slot] = {};
  io.reg_write.state = io.regs.lookup_valid_r ? AXI_LLCState::kLookup
                                              : AXI_LLCState::kIdle;
}

void AXI_LLCRef::accept_new_requests() {
  const auto &regs = io.reg_write;
  const int master = pick_new_read_master(regs);
  if (master < 0) {
    return;
  }

  const auto &req = io.ext_in.upstream.read_req[master];
  if (!req.valid) {
    return;
  }
  // Do not recycle the per-master response slot in the same cycle that an old
  // response is being drained. The shared-LLC path can otherwise accept a new
  // icache miss before the previous response has fully retired, which leaves
  // the new request without a durable lookup/MSHR owner.
  if (io.regs.read_resp_valid_r[master]) {
    return;
  }

  const uint32_t req_line_addr = line_addr(config_, req.addr);
  if (has_pending_upstream_write_line(req_line_addr)) {
    return;
  }
  if (write_line_pending(regs, req_line_addr)) {
    return;
  }
  if (victim_line_pending(regs, req_line_addr)) {
    return;
  }
  const int merge_slot = find_mshr_by_line_addr(regs, req_line_addr);
  if (!req.direct_mapped && merge_slot >= 0) {
    return;
  }

  io.ext_out.upstream.read_req[master].ready = true;
  io.reg_write.rr_read_master_r =
      static_cast<uint8_t>((master + 1) % NUM_READ_MASTERS);
  if (req.bypass) {
  } else if (!req.direct_mapped) {
    perf_count_read_access(io.reg_write.perf, static_cast<uint8_t>(master));
  }
  for (auto &prefetch_req : io.reg_write.prefetch_q) {
    prefetch_req = {};
  }
  io.reg_write.lookup_valid_r = true;
  io.reg_write.lookup_issued_r = false;
  io.reg_write.lookup_addr_r = req.addr;
  io.reg_write.lookup_size_r = req.total_size;
  io.reg_write.lookup_master_r = static_cast<uint8_t>(master);
  io.reg_write.lookup_id_r = req.id;
  io.reg_write.lookup_is_prefetch_r = false;
  io.reg_write.lookup_is_invalidate_r = false;
  io.reg_write.lookup_is_write_r = false;
  io.reg_write.lookup_is_bypass_r = req.bypass;
  io.reg_write.lookup_is_direct_mapped_r = req.direct_mapped;
  io.reg_write.lookup_is_mode2_ddr_aligned_r = req.mode2_ddr_aligned;
  io.reg_write.state = AXI_LLCState::kLookup;
}

void AXI_LLCRef::comb_disabled() {
  for (uint8_t i = 0; i < NUM_READ_MASTERS; ++i) {
    io.ext_out.upstream.read_req[i].ready = false;
    io.ext_out.upstream.read_resp[i].valid = false;
    io.ext_out.upstream.read_resp[i].id = 0;
    io.ext_out.upstream.read_resp[i].data.clear();
  }
  for (uint8_t i = 0; i < NUM_WRITE_MASTERS; ++i) {
    io.ext_out.upstream.write_req[i].ready = false;
    io.ext_out.upstream.write_resp[i].valid = false;
    io.ext_out.upstream.write_resp[i].id = 0;
    io.ext_out.upstream.write_resp[i].resp = 0;
  }
  io.ext_out.mem = {};
}

void AXI_LLCRef::comb() {
  io.ext_out = {};
  io.table_out.clear();
  copy_regs_live_ref(io.reg_write, io.regs);

  if (!config_.enable || !config_.valid()) {
    comb_disabled();
    io.reg_write.enable_r = false;
    io.reg_write.state = AXI_LLCState::kDisabled;
    return;
  }

  io.reg_write.enable_r = true;
  if (io.regs.state == AXI_LLCState::kDisabled) {
    io.reg_write.state = AXI_LLCState::kIdle;
  }
  const uint32_t mshr_limit = std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING);
  bool has_read_resp_pending = false;
  for (uint8_t i = 0; i < NUM_READ_MASTERS; ++i) {
    if (io.regs.read_resp_valid_r[i] ||
        io.regs.read_resp_q_count_r[i] != 0) {
      has_read_resp_pending = true;
      break;
    }
  }
  const bool quiet_cycle = io.ext_in.mem.prefetch_allow && !io.regs.lookup_valid_r &&
                           count_free_mshrs(io.regs) == mshr_limit &&
                           !has_demand_mshr(io.regs) && !has_read_resp_pending;
  io.reg_write.prefetch_quiet_cycles_r =
      quiet_cycle
          ? static_cast<uint8_t>(std::min<uint32_t>(
                static_cast<uint32_t>(io.regs.prefetch_quiet_cycles_r) + 1u, 0xFFu))
          : 0;

  for (uint8_t i = 0; i < NUM_READ_MASTERS; ++i) {
    if (io.regs.read_resp_valid_r[i] && !io.regs.read_resp_fresh_r[i] &&
        !io.ext_in.upstream.read_resp[i].ready) {
    }
  }
  for (uint32_t slot = 0; slot < mshr_limit; ++slot) {
    const auto &entry = io.regs.mshr[slot];
    if (!entry.valid) {
      continue;
    }
  }
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = io.regs.write_ctx[master];
    if (!ctx.valid) {
      continue;
    }
  }

  drive_read_responses();
  drive_write_path();
  (void)try_complete_lookup();
  drive_mem_read_path();
  (void)try_launch_pending_write_lookup();
  accept_new_requests();
  if (!io.ext_in.mem.invalidate_all) {
    accept_maintenance_request();
    try_launch_prefetch_lookup();
  }
  drive_lookup_request();

  if (io.ext_in.mem.invalidate_all && can_accept_invalidate_all_now(io.reg_write) &&
      !io.table_out.data.enable && !io.table_out.meta.enable &&
      !io.table_out.repl.enable && !io.ext_out.mem.read_req_valid &&
      !io.ext_out.mem.write_req_valid) {
    io.ext_out.mem.invalidate_all_accepted = true;
    io.table_out.invalidate_all = true;
    io.reg_write.invalidate_epoch_r =
        static_cast<uint8_t>(io.regs.invalidate_epoch_r + 1);
    io.reg_write.dirty_line_count_r = 0;
    if (io.regs.lookup_valid_r) {
      io.reg_write.lookup_issued_r = false;
      io.reg_write.state = AXI_LLCState::kLookup;
    }
  }

}


void copy_regs_live_ref(AXI_LLC_Regs_t &dst, const AXI_LLC_Regs_t &src) {
  static_assert(std::is_trivially_copyable<AXI_LLC_Regs_t>::value,
                "copy_regs_live_ref needs a trivially copyable Regs");
  constexpr size_t kBulk = __builtin_offsetof(AXI_LLC_Regs_t, read_resp_data_r);
  std::memcpy(&dst, &src, kBulk);
  // Dead slots still need their valid flag restored: comb() runs several times
  // per tick and an earlier pass may have allocated a slot in dst.
  for (int i = 0; i < AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH; ++i) {
    dst.read_victim_wb_q[i].valid = src.read_victim_wb_q[i].valid;
  }
  for (int m = 0; m < NUM_WRITE_MASTERS; ++m) {
    dst.write_ctx[m].valid = src.write_ctx[m].valid;
    for (int i = 0; i < MAX_WRITE_OUTSTANDING; ++i) {
      dst.write_q[m][i].valid = src.write_q[m][i].valid;
    }
  }
  for (uint32_t i = 0; i < MAX_OUTSTANDING; ++i) {
    dst.mshr[i].valid = src.mshr[i].valid;
  }
  for (int m = 0; m < NUM_READ_MASTERS; ++m) {
    if (src.read_resp_valid_r[m]) {
      dst.read_resp_data_r[m] = src.read_resp_data_r[m];
    }
    const uint32_t n = std::min<uint32_t>(src.read_resp_q_count_r[m],
                                          AXI_LLC_READ_RESP_QUEUE_DEPTH);
    for (uint32_t i = 0; i < n; ++i) {
      const uint32_t slot =
          (src.read_resp_q_head_r[m] + i) % AXI_LLC_READ_RESP_QUEUE_DEPTH;
      dst.read_resp_q_data_r[m][slot] = src.read_resp_q_data_r[m][slot];
    }
  }
  {
    const uint32_t n = std::min<uint32_t>(src.read_victim_wb_q_count_r,
                                          AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
    for (uint32_t i = 0; i < n; ++i) {
      const uint32_t slot = (src.read_victim_wb_q_head_r + i) %
                            AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
      dst.read_victim_wb_q[slot] = src.read_victim_wb_q[slot];
    }
  }
  for (int m = 0; m < NUM_WRITE_MASTERS; ++m) {
    if (src.write_ctx[m].valid) {
      dst.write_ctx[m] = src.write_ctx[m];
    }
    const uint32_t n =
        std::min<uint32_t>(src.write_q_count_r[m], MAX_WRITE_OUTSTANDING);
    for (uint32_t i = 0; i < n; ++i) {
      const uint32_t slot = (src.write_q_head_r[m] + i) % MAX_WRITE_OUTSTANDING;
      dst.write_q[m][slot] = src.write_q[m][slot];
    }
  }
  if (src.victim_wb_valid_r) {
    dst.victim_wb_strobe_r = src.victim_wb_strobe_r;
  }
  for (uint32_t i = 0; i < MAX_OUTSTANDING; ++i) {
    if (src.mshr[i].valid) {
      dst.mshr[i] = src.mshr[i];
    }
  }
}

void AXI_LLCRef::seq() { copy_regs_live_ref(io.regs, io.reg_write); }

} // namespace axi_interconnect

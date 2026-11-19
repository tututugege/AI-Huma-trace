#include "AXI_LLC.h"
#include "Bop.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>

extern long long sim_time;

namespace axi_interconnect {

namespace {
constexpr uint8_t kInvalidReadMaster = NUM_READ_MASTERS;

// Only the data cache may have several demand misses in flight.
bool allow_parallel_demand_mshr(uint8_t master) { return master == MASTER_DCACHE_R; }

// EXP-048: LLC_BOP=1 turns the Best-Offset prefetcher on in the LLC (parameters LLC_BOP_*; LLC_BOP_RES MSHRs kept for demand
// misses, LLC_BOP_INF prefetch MSHRs in flight at most)
struct LlcBopCfg {
  bool on = false;
  int res = 2, inf = 4;
  Bop *bop = nullptr;
  LlcBopCfg() {
    auto g = [](const char *n, int d) { const char *v = std::getenv(n); return v ? std::atoi(v) : d; };
    on = g("LLC_BOP", 0) != 0;
    res = g("LLC_BOP_RES", res);
    inf = g("LLC_BOP_INF", inf);
    if (on) { bop = new Bop("LLC_BOP"); g_bop_llc = bop; }
  }
};
LlcBopCfg &llc_bop() { static LlcBopCfg c; return c; }

uint32_t read_u32_le(const uint8_t *p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

void write_u32_le(uint8_t *p, uint32_t v) {
  p[0] = uint8_t(v);
  p[1] = uint8_t(v >> 8);
  p[2] = uint8_t(v >> 16);
  p[3] = uint8_t(v >> 24);
}

WideWriteStrb_t full_line_strobe(const AXI_LLCConfig &config) {
  WideWriteStrb_t out;
  out.clear();
  for (uint32_t b = 0; b < config.line_bytes && b < MAX_WRITE_TRANSACTION_BYTES; ++b) out.set(b, true);
  return out;
}

uint32_t build_line_addr_from_tag_set(const AXI_LLCConfig &config, uint32_t tag, uint32_t set) {
  return config.valid() ? ((tag * config.set_count()) + set) * config.line_bytes : 0;
}

bool is_full_line_write(const AXI_LLCConfig &config, uint32_t addr, const WideWriteStrb_t &strobe, uint32_t total_size) {
  if (config.line_bytes == 0 || (addr % config.line_bytes) != 0 || total_size + 1u != config.line_bytes) return false;
  for (uint32_t byte = 0; byte < config.line_bytes; ++byte)
    if (!strobe.test(byte)) return false;
  return true;
}

uint32_t update_dirty_line_count(uint32_t current, bool old_dirty, bool new_dirty) {
  if (old_dirty == new_dirty) return current;
  if (new_dirty) return current + 1u;
  return current == 0 ? 0 : current - 1u;
}

// ---------------------------------------------------------------------------------------------
// Replacement state (LLC_REPL, read once): one repl-table row per set, read by every lookup.
//   rr   (old main): the row is a next-victim way pointer; a hit or a line write sets it to way + 1.
//   plru: tree pseudo-LRU, ways-1 bits; node i has children 2i+1 / 2i+2, a bit set means "evict from the right half".
//   lru : true LRU, a log2(ways)-bit age per way (0 = most recent), stored XOR the way number so the all-zero reset row
//         is a valid order; a reference upper bound.
//   srrip (default, EXP-045) / brrip / drrip: a 2-bit re-reference prediction value (RRPV) per way. A hit sets it to 0; the victim is the
//         way with the largest RRPV (lowest way on a tie); an allocation first ages the row so that the largest RRPV
//         becomes 3, then inserts the new line at 2 (srrip), at 3 except every 32nd allocation at 2 (brrip), or as
//         chosen by set dueling (drrip: sets with set % 32 == 0 always use srrip, == 1 always brrip; their misses move a
//         10-bit counter, the other sets follow the winner).
// LLC_PF_LOW=1: a prefetch allocation goes in at the low-priority end (plru / lru: not touched; rrip: RRPV 3).
// plru / lru are updated read-modify-write when the lookup completes: a hit touches the hit way, an allocating miss
// touches its victim (allocation time); line fills do not write the row. Only one lookup is in flight, so the next
// lookup always reads the written row.
// ---------------------------------------------------------------------------------------------
enum class LlcRepl { kRR, kPLRU, kLRU, kSRRIP, kBRRIP, kDRRIP };
LlcRepl llc_repl() {
  static const LlcRepl m = [] {
    const char *v = std::getenv("LLC_REPL");
    if (v && std::strcmp(v, "plru") == 0) return LlcRepl::kPLRU;
    if (v && std::strcmp(v, "lru") == 0) return LlcRepl::kLRU;
    if (v && std::strcmp(v, "srrip") == 0) return LlcRepl::kSRRIP;
    if (v && std::strcmp(v, "brrip") == 0) return LlcRepl::kBRRIP;
    if (v && std::strcmp(v, "drrip") == 0) return LlcRepl::kDRRIP;
    if (v && std::strcmp(v, "rr") == 0) return LlcRepl::kRR;
    if (v) { std::fprintf(stderr, "LLC_REPL=%s: unknown\n", v); std::abort(); }
    return LlcRepl::kSRRIP;
  }();
  return m;
}

bool llc_rrip() { return llc_repl() == LlcRepl::kSRRIP || llc_repl() == LlcRepl::kBRRIP || llc_repl() == LlcRepl::kDRRIP; }
bool llc_pf_low() { static const bool on = std::getenv("LLC_PF_LOW") && std::getenv("LLC_PF_LOW")[0] == '1'; return on; }

uint32_t rrpv_of(uint64_t t, uint32_t way) { return uint32_t(t >> (2 * way)) & 3u; }
uint64_t rrpv_set(uint64_t t, uint32_t way, uint32_t v) { return (t & ~(3ull << (2 * way))) | (uint64_t(v & 3u) << (2 * way)); }
// the row aged so that its largest RRPV is 3 (the classic "increment all until one reaches 3")
uint64_t rrip_age(uint64_t t, uint32_t ways) {
  uint32_t mx = 0;
  for (uint32_t w = 0; w < ways; ++w) mx = std::max(mx, rrpv_of(t, w));
  const uint32_t add = 3u - mx;
  for (uint32_t w = 0; w < ways && add; ++w) t = rrpv_set(t, w, rrpv_of(t, w) + add);
  return t;
}

uint32_t log2_ways(uint32_t ways) {
  uint32_t b = 0;
  while ((1u << b) < ways) ++b;
  return b;
}

uint64_t plru_touch(uint64_t t, uint32_t ways, uint32_t way) {
  uint32_t node = 0, lo = 0, hi = ways;
  while (hi - lo > 1) {
    const uint32_t mid = (lo + hi) / 2;
    const bool right = way >= mid;
    if (right) t &= ~(1ull << node); else t |= 1ull << node;   // point away from the touched half
    node = 2 * node + (right ? 2 : 1);
    if (right) lo = mid; else hi = mid;
  }
  return t;
}

uint32_t plru_victim(uint64_t t, uint32_t ways) {
  uint32_t node = 0, lo = 0, hi = ways;
  while (hi - lo > 1) {
    const uint32_t mid = (lo + hi) / 2;
    const bool right = (t >> node) & 1;
    node = 2 * node + (right ? 2 : 1);
    if (right) lo = mid; else hi = mid;
  }
  return lo;
}

uint32_t lru_age(uint64_t t, uint32_t ways, uint32_t way) {
  const uint32_t b = log2_ways(ways), mask = (1u << b) - 1u;
  return (uint32_t(t >> (way * b)) & mask) ^ (way & mask);
}

uint64_t lru_touch(uint64_t t, uint32_t ways, uint32_t way) {
  const uint32_t b = log2_ways(ways), mask = (1u << b) - 1u, old = lru_age(t, ways, way);
  uint64_t out = 0;
  for (uint32_t w = 0; w < ways; ++w) {
    uint32_t a = lru_age(t, ways, w);
    if (w == way) a = 0; else if (a < old) ++a;
    out |= uint64_t((a ^ w) & mask) << (w * b);
  }
  return out;
}

// The repl row after `way` was used: a hit (alloc = false) or an allocation (rrip: inserted with RRPV `ins`; low: a
// low-priority insertion, plru / lru leave the row as it is).
uint64_t repl_after_use(uint64_t t, uint32_t ways, uint32_t way, bool alloc = false, uint32_t ins = 2, bool low = false) {
  const bool fits = llc_repl() == LlcRepl::kPLRU ? ways <= 64
                    : llc_repl() == LlcRepl::kLRU ? ways * log2_ways(ways) <= 64
                    : llc_rrip() ? ways <= 32 : true;
  if (!fits || (ways & (ways - 1)) != 0) {
    std::fprintf(stderr, "LLC_REPL: %u ways do not fit the 64-bit repl row (plru: power of two <= 64, lru: <= 16)\n", ways);
    std::abort();
  }
  if (llc_rrip()) return alloc ? rrpv_set(rrip_age(t, ways), way, low ? 3u : ins) : rrpv_set(t, way, 0);
  if (alloc && low && llc_repl() != LlcRepl::kRR) return t;
  switch (llc_repl()) {
    case LlcRepl::kPLRU: return plru_touch(t, ways, way);
    case LlcRepl::kLRU: return lru_touch(t, ways, way);
    default: return (way + 1) % ways;
  }
}

AXI_LLC_Bytes_t build_repl_payload(uint64_t state) {
  AXI_LLC_Bytes_t bytes;
  bytes.resize(AXI_LLC_REPL_BYTES);
  write_u32_le(bytes.data(), uint32_t(state));
  write_u32_le(bytes.data() + 4, uint32_t(state >> 32));
  return bytes;
}

uint64_t decode_repl(const AXI_LLC_Bytes_t &payload) {
  if (payload.size() < AXI_LLC_REPL_BYTES) return 0;
  return uint64_t(read_u32_le(payload.data())) | (uint64_t(read_u32_le(payload.data() + 4)) << 32);
}

bool llc_table_write_busy(const AXI_LLC_TableOut_t &t) {
  return t.data.enable || t.meta.enable || t.valid.enable || t.repl.enable;
}

bool decode_valid_bit(const AXI_LLC_Bytes_t &payload, uint32_t way, bool fallback_valid) {
  return way >= payload.size() ? fallback_valid : payload.data()[way] != 0;
}

void encode_valid_bit_payload(uint32_t ways, uint32_t way, bool valid, AXI_LLC_Bytes_t &payload,
                              std::vector<uint8_t> &byte_enable) {
  payload.resize(ways);
  byte_enable.assign(ways, 0);
  if (way >= ways) return;
  payload.data()[way] = valid ? 1u : 0u;
  byte_enable[way] = 1;
}

// Table command: write `way` of row `set` (the data table carries no values).
void begin_write(AXI_LLC_TableReq_t &r, uint32_t set, uint32_t way) {
  r.enable = true;
  r.write = true;
  r.index = set;
  r.way = way;
}

void put_meta(AXI_LLC_TableReq_t &r, uint32_t set, uint32_t way, const AXI_LLCMetaEntry_t &meta) {
  begin_write(r, set, way);
  AXI_LLC::encode_meta(meta, r.payload);
  r.byte_enable.assign(AXI_LLC_META_ENTRY_BYTES, 1);
}

void put_repl(AXI_LLC_TableReq_t &r, uint32_t set, uint64_t state) {
  r.enable = true;
  r.write = true;
  r.index = set;
  r.payload = build_repl_payload(state);
  r.byte_enable.assign(AXI_LLC_REPL_BYTES, 1);
}
}  // namespace

AXI_LLC::AXI_LLC() { reset(); }

uint32_t AXI_LLC::mshr_limit() const { return std::min<uint32_t>(config_.mshr_num, MAX_OUTSTANDING); }

// ---------------------------------------------------------------------------------------------
// Response and victim queues
// ---------------------------------------------------------------------------------------------

// A master's response goes to its output register if that is free, else into the per-master queue.
bool AXI_LLC::enqueue_read_response(uint8_t master, uint8_t id, const WideReadData_t &data) {
  if (master >= NUM_READ_MASTERS) return false;
  auto &w = io.reg_write;
  if (!w.read_resp_valid_r[master]) {
    w.read_resp_valid_r[master] = true;
    w.read_resp_fresh_r[master] = true;
    w.read_resp_id_r[master] = id;
    w.read_resp_data_r[master] = data;
    return true;
  }
  if (w.read_resp_q_count_r[master] >= AXI_LLC_READ_RESP_QUEUE_DEPTH) return false;
  const uint8_t tail = w.read_resp_q_tail_r[master] % AXI_LLC_READ_RESP_QUEUE_DEPTH;
  w.read_resp_q_id_r[master][tail] = id;
  w.read_resp_q_data_r[master][tail] = data;
  w.read_resp_q_tail_r[master] = uint8_t((tail + 1u) % AXI_LLC_READ_RESP_QUEUE_DEPTH);
  w.read_resp_q_count_r[master]++;
  return true;
}

// Move the oldest queued response into the (just freed) output register.
void AXI_LLC::promote_read_response(uint8_t i) {
  const uint8_t head = io.regs.read_resp_q_head_r[i] % AXI_LLC_READ_RESP_QUEUE_DEPTH;
  auto &w = io.reg_write;
  w.read_resp_valid_r[i] = true;
  w.read_resp_fresh_r[i] = true;
  w.read_resp_id_r[i] = io.regs.read_resp_q_id_r[i][head];
  w.read_resp_data_r[i] = io.regs.read_resp_q_data_r[i][head];
  w.read_resp_q_id_r[i][head] = 0;
  w.read_resp_q_data_r[i][head].clear();
  w.read_resp_q_head_r[i] = uint8_t((head + 1u) % AXI_LLC_READ_RESP_QUEUE_DEPTH);
  w.read_resp_q_count_r[i]--;
}

bool AXI_LLC::read_victim_wb_queue_full(const AXI_LLC_Regs_t &regs) const {
  return regs.read_victim_wb_q_count_r >= AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
}

namespace {
// Visit the live entries of the read-victim write-back queue, oldest first; stop when `f` returns true.
template <class F> bool any_read_victim(const AXI_LLC_Regs_t &regs, F f) {
  for (uint32_t i = 0; i < regs.read_victim_wb_q_count_r && i < AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH; ++i) {
    const auto &e = regs.read_victim_wb_q[(regs.read_victim_wb_q_head_r + i) % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH];
    if (e.valid && f(e)) return true;
  }
  return false;
}
}  // namespace

bool AXI_LLC::read_victim_snapshot_queued(const AXI_LLC_Regs_t &regs, uint8_t owner_slot, uint32_t victim_addr) const {
  return any_read_victim(regs, [&](const AXI_LLCReadVictimWbReq_t &e) {
    return e.owner_slot == owner_slot && e.victim_addr == victim_addr;
  });
}

bool AXI_LLC::enqueue_read_victim_snapshot(uint8_t owner_slot, uint32_t victim_addr, const WideWriteStrb_t &victim_strobe) {
  if (read_victim_snapshot_queued(io.reg_write, owner_slot, victim_addr)) return true;
  if (read_victim_wb_queue_full(io.reg_write)) return false;
  auto &w = io.reg_write;
  const uint8_t tail = w.read_victim_wb_q_tail_r % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
  auto &entry = w.read_victim_wb_q[tail];
  entry = {};
  entry.valid = true;
  entry.owner_slot = owner_slot;
  entry.victim_addr = victim_addr;
  entry.victim_strobe = victim_strobe;
  w.read_victim_wb_q_tail_r = uint8_t((tail + 1u) % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
  w.read_victim_wb_q_count_r++;
  return true;
}

// Occupy the single victim write-back slot (the caller checked it is free).
void AXI_LLC::arm_victim_writeback(bool for_write, uint8_t write_master, uint8_t mshr_slot, uint32_t addr) {
  auto &w = io.reg_write;
  w.victim_wb_valid_r = true;
  w.victim_wb_issued_r = false;
  w.victim_wb_for_write_r = for_write;
  w.victim_wb_write_master_r = write_master;
  w.victim_wb_mshr_slot_r = mshr_slot;
  w.victim_wb_addr_r = addr;
  w.victim_wb_issue_cycle_r = 0;
  w.victim_wb_strobe_r = full_line_strobe(config_);
}

// A full line goes in: data, meta (valid + flags), valid bit and the replacement pointer.
void AXI_LLC::write_line_tables(uint32_t set, uint32_t way, const AXI_LLCMetaEntry_t &meta, uint32_t repl_next_way) {
  auto &t = io.table_out;
  begin_write(t.data, set, way);
  put_meta(t.meta, set, way, meta);
  write_valid_table(set, way, true);
  if (llc_repl() == LlcRepl::kRR) put_repl(t.repl, set, repl_next_way);   // (plru / lru: written at the lookup)
}

void AXI_LLC::write_valid_table(uint32_t set, uint32_t way, bool valid) {
  auto &v = io.table_out.valid;
  begin_write(v, set, way);
  encode_valid_bit_payload(config_.ways, way, valid, v.payload, v.byte_enable);
}

// ---------------------------------------------------------------------------------------------
// Hazards between lines in flight
// ---------------------------------------------------------------------------------------------

bool AXI_LLC::has_pending_upstream_write_line(uint32_t line) const {
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    const auto &req = io.ext_in.upstream.write_req[m];
    if (req.valid && !req.bypass && line_addr(config_, req.addr) == line) return true;
  }
  return false;
}

// A write to the line is queued, or in flight and not yet written to the cache (bypass: to memory).
bool AXI_LLC::write_line_pending(const AXI_LLC_Regs_t &regs, uint32_t line) const {
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    const auto &ctx = regs.write_ctx[m];
    if (ctx.valid && ctx.line_addr == line && !(ctx.bypass ? ctx.mem_done : ctx.cache_done)) return true;
  }
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m)
    for (uint32_t i = 0; i < regs.write_q_count_r[m] && i < MAX_WRITE_OUTSTANDING; ++i) {
      const auto &e = regs.write_q[m][(regs.write_q_head_r[m] + i) % MAX_WRITE_OUTSTANDING];
      if (e.valid && line_addr(config_, e.addr) == line) return true;
    }
  return false;
}

bool AXI_LLC::read_victim_snapshot_present(const AXI_LLC_Regs_t &regs, uint32_t victim_addr) const {
  if (victim_addr == 0) return false;
  if (regs.victim_wb_valid_r && !regs.victim_wb_for_write_r && regs.victim_wb_addr_r == victim_addr) return true;
  return any_read_victim(regs, [&](const AXI_LLCReadVictimWbReq_t &e) { return e.victim_addr == victim_addr; });
}

// Only writes that already consumed cache state can block the victim snapshot; queued or pre-lookup writes must wait
// and retry after the refill path externalized the snapshot.
bool AXI_LLC::victim_snapshot_waits_for_write_resolution(const AXI_LLC_Regs_t &regs, uint32_t victim_addr) const {
  if (victim_addr == 0) return false;
  if (regs.lookup_valid_r && regs.lookup_is_write_r && regs.lookup_issued_r && !regs.lookup_is_bypass_r &&
      line_addr(config_, regs.lookup_addr_r) == victim_addr)
    return true;
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    const auto &ctx = regs.write_ctx[m];
    if (ctx.valid && !ctx.bypass && ctx.line_addr == victim_addr && ctx.cache_pending) return true;
  }
  return false;
}

// The line is (or is about to be) a dirty victim whose write-back has not completed: accesses must wait.
bool AXI_LLC::victim_line_pending(const AXI_LLC_Regs_t &regs, uint32_t line) const {
  if (regs.victim_wb_valid_r && regs.victim_wb_addr_r == line) return true;
  if (any_read_victim(regs, [&](const AXI_LLCReadVictimWbReq_t &e) { return e.victim_addr == line; })) return true;
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    const auto &ctx = regs.write_ctx[m];
    if (ctx.valid && !ctx.bypass && ctx.cache_pending && ctx.victim_dirty && ctx.victim_addr == line) return true;
  }
  for (uint32_t slot = 0; slot < mshr_limit(); ++slot) {
    const auto &e = regs.mshr[slot];
    if (!e.valid || e.bypass || !e.victim_dirty || e.refill_committed || e.victim_addr != line) continue;
    const bool armed = regs.victim_wb_valid_r && !regs.victim_wb_for_write_r && regs.victim_wb_addr_r == line &&
                       regs.victim_wb_mshr_slot_r == uint8_t(slot);
    // Before the refill data arrives the victim is still resident and write hits must be able to refresh the pending
    // snapshot. Once the refill is ready the victim may already be gone from the table before its write-back is armed,
    // so everything is blocked over the whole "refill_valid -> victim write-back done" window.
    if (e.refill_valid || e.victim_writeback_done || armed) return true;
  }
  return false;
}

// `way` of `set` is promised to another line: a write that already picked it, or a miss whose refill will land there.
bool AXI_LLC::way_reserved(const AXI_LLC_Regs_t &regs, uint32_t set, uint8_t way, uint32_t line) const {
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    const auto &ctx = regs.write_ctx[m];
    if (ctx.valid && !ctx.bypass && ctx.cache_pending && ctx.set == set && ctx.way == way && ctx.line_addr != line)
      return true;
  }
  for (uint32_t slot = 0; slot < mshr_limit(); ++slot) {
    const auto &e = regs.mshr[slot];
    if (e.valid && !e.bypass && !e.refill_committed && e.set == set && e.way == way && e.line_addr != line) return true;
  }
  return false;
}

// plru / lru / rrip: an allocating miss (alloc) or a write hit writes the repl row back from the value its lookup read.
// rrip: the insertion RRPV; drrip set dueling and the brrip 1-in-32 counter live in registers.
void AXI_LLC::touch_repl_at_lookup(uint32_t set, uint32_t way, bool alloc, bool prefetch) {
  if (llc_repl() == LlcRepl::kRR) return;
  // An allocation does not wait for the table write port: when the write-after-victim commit took it in this cycle
  // (RTL: one table write per tick, commit first), the replacement update of the allocation is dropped.
  if (alloc && lookup_tbl_busy_) return;
  uint32_t ins = 2;
  if (alloc && (llc_repl() == LlcRepl::kBRRIP || llc_repl() == LlcRepl::kDRRIP)) {
    const auto &r = io.regs;
    auto &w = io.reg_write;
    bool bimodal = llc_repl() == LlcRepl::kBRRIP;
    if (llc_repl() == LlcRepl::kDRRIP) {
      const uint32_t leader = set % 32u;
      if (leader == 0) {   // an srrip leader missed: brrip is better
        bimodal = false;
        w.repl_psel_r = uint16_t(std::min<uint32_t>(r.repl_psel_r + 1u, 1023u));
      } else if (leader == 1) {
        bimodal = true;
        w.repl_psel_r = uint16_t(r.repl_psel_r == 0 ? 0 : r.repl_psel_r - 1u);
      } else {
        bimodal = r.repl_psel_r >= 512u;
      }
    }
    if (bimodal) {
      ins = (r.repl_bip_ctr_r % 32u) == 0 ? 2u : 3u;
      w.repl_bip_ctr_r = uint8_t(r.repl_bip_ctr_r + 1u);
    }
  }
  put_repl(io.table_out.repl, set,
           repl_after_use(decode_repl(io.lookup_in.repl), config_.ways, way, alloc, ins, alloc && prefetch && llc_pf_low()));
}

int AXI_LLC::pick_unreserved_way(const AXI_LLC_Regs_t &regs, uint32_t set, int first_invalid_way, uint64_t repl_state,
                                 uint32_t line) const {
  if (first_invalid_way >= 0 && !way_reserved(regs, set, uint8_t(first_invalid_way), line)) return first_invalid_way;
  if (llc_repl() == LlcRepl::kLRU || llc_rrip()) {   // the oldest way (largest age / RRPV) that no in-flight miss reserved
    int best = -1;
    uint32_t best_age = 0;
    for (uint32_t w = 0; w < config_.ways; ++w) {
      if (way_reserved(regs, set, uint8_t(w), line)) continue;
      const uint32_t a = llc_rrip() ? rrpv_of(repl_state, w) : lru_age(repl_state, config_.ways, w);
      if (best < 0 || a > best_age) best = int(w), best_age = a;
    }
    return best;
  }
  // rr: the pointer; plru: the tree's victim. If an in-flight miss reserved it, the next unreserved way.
  const uint32_t repl_way_raw =
      llc_repl() == LlcRepl::kPLRU ? plru_victim(repl_state, config_.ways) : uint32_t(repl_state % config_.ways);
  for (uint32_t off = 0; off < config_.ways; ++off) {
    const uint8_t cand = uint8_t((repl_way_raw + off) % config_.ways);
    if (!way_reserved(regs, set, cand, line)) return int(cand);
  }
  return -1;
}

bool AXI_LLC::can_accept_invalidate_line_now(uint32_t line) const {
  const auto &r = io.regs;
  for (uint8_t m = 0; m < NUM_READ_MASTERS; ++m)
    if (r.read_resp_valid_r[m] || r.read_resp_q_count_r[m] != 0) return false;
  if (find_mshr_by_line_addr(r, line) >= 0) return false;

  const int bypass_owner = find_bypass_write_mem_owner(r);
  if (io.ext_in.mem.write_resp_valid && bypass_owner >= 0) {
    const auto &ctx = r.write_ctx[uint8_t(bypass_owner)];
    if (ctx.valid && ctx.line_addr == line) return false;
  }
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m)
    if (io.ext_in.upstream.write_req[m].valid && line_addr(config_, io.ext_in.upstream.write_req[m].addr) == line)
      return false;

  if (write_line_pending(r, line) || victim_line_pending(r, line)) return false;
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    if (r.write_ctx[m].valid && r.write_ctx[m].line_addr == line) return false;
    if (r.write_resp_valid_r[m] && r.write_resp_line_addr_r[m] == line) return false;
  }
  if (r.lookup_valid_r && line_addr(config_, r.lookup_addr_r) == line) return false;

  if (r.victim_wb_valid_r) {
    if (r.victim_wb_addr_r == line) return false;
    if (r.victim_wb_for_write_r && r.victim_wb_write_master_r < NUM_WRITE_MASTERS) {
      const auto &ctx = r.write_ctx[r.victim_wb_write_master_r];
      if (ctx.valid && ctx.line_addr == line) return false;
    }
  }
  return true;
}

bool AXI_LLC::has_dirty_or_write_hazard(const AXI_LLC_Regs_t &regs) const {
  if (regs.dirty_line_count_r != 0 || regs.victim_wb_valid_r) return true;
  for (uint32_t slot = 0; slot < mshr_limit(); ++slot) {
    const auto &e = regs.mshr[slot];
    if (e.valid && (e.is_write || (e.victim_dirty && !e.victim_writeback_done))) return true;
  }
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m)
    if (regs.write_ctx[m].valid || regs.write_q_count_r[m] != 0 || regs.write_resp_valid_r[m]) return true;
  return regs.lookup_valid_r && regs.lookup_is_write_r;
}

bool AXI_LLC::can_accept_invalidate_all_now(const AXI_LLC_Regs_t &regs) const { return !has_dirty_or_write_hazard(regs); }

uint32_t AXI_LLC::line_addr(const AXI_LLCConfig &config, uint32_t addr) {
  return config.line_bytes == 0 ? addr : (addr / config.line_bytes) * config.line_bytes;
}

uint32_t AXI_LLC::set_index(const AXI_LLCConfig &config, uint32_t addr) {
  if (!config.valid() || config.line_bytes == 0) return 0;
  return (addr / config.line_bytes) % config.set_count();
}

uint32_t AXI_LLC::tag_of(const AXI_LLCConfig &config, uint32_t addr) {
  if (!config.valid() || config.line_bytes == 0 || config.set_count() == 0) return 0;
  return (addr / config.line_bytes) / config.set_count();
}

uint32_t AXI_LLC::valid_row_bytes(const AXI_LLCConfig &config) { return config.ways; }

AXI_LLCMetaEntry_t AXI_LLC::decode_meta(const AXI_LLC_Bytes_t &payload, uint32_t way) {
  AXI_LLCMetaEntry_t entry{};
  const size_t offset = size_t(way) * AXI_LLC_META_ENTRY_BYTES;
  if (offset + AXI_LLC_META_ENTRY_BYTES > payload.size()) return entry;
  const uint32_t packed = read_u32_le(payload.data() + offset);
  entry.tag = packed & 0x1fffffffu;
  if ((packed >> 29) & 1u) entry.flags |= AXI_LLC_META_VALID;
  if ((packed >> 30) & 1u) entry.flags |= AXI_LLC_META_DIRTY;
  if ((packed >> 31) & 1u) entry.flags |= AXI_LLC_META_PREFETCH;
  return entry;
}

void AXI_LLC::encode_meta(const AXI_LLCMetaEntry_t &entry, AXI_LLC_Bytes_t &payload) {
  payload.resize(AXI_LLC_META_ENTRY_BYTES);
  if ((entry.tag & 0xe0000000u) != 0) std::abort();
  uint32_t packed = entry.tag & 0x1fffffffu;
  if (entry.flags & AXI_LLC_META_VALID) packed |= 1u << 29;
  if (entry.flags & AXI_LLC_META_DIRTY) packed |= 1u << 30;
  if (entry.flags & AXI_LLC_META_PREFETCH) packed |= 1u << 31;
  write_u32_le(payload.data(), packed);
}

void AXI_LLC::set_config(const AXI_LLCConfig &config) {
  config_ = config;
  memo_drop();
  io.regs.enable_r = config_.enable;
  io.regs.state = config_.enable ? AXI_LLCState::kIdle : AXI_LLCState::kDisabled;
}

void AXI_LLC::reset() {
  io = {};
  memo_drop();
  io.regs.enable_r = config_.enable;
  io.regs.state = config_.enable ? AXI_LLCState::kIdle : AXI_LLCState::kDisabled;
}

// ---------------------------------------------------------------------------------------------
// MSHR / queue queries
// ---------------------------------------------------------------------------------------------

int AXI_LLC::find_free_mshr(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t i = 0; i < mshr_limit(); ++i)
    if (!regs.mshr[i].valid) return int(i);
  return -1;
}

int AXI_LLC::find_mshr_by_line_addr(const AXI_LLC_Regs_t &regs, uint32_t line) const {
  for (uint32_t i = 0; i < mshr_limit(); ++i)
    if (regs.mshr[i].valid && !regs.mshr[i].bypass && regs.mshr[i].line_addr == line) return int(i);
  return -1;
}

// The miss whose dirty victim (address != 0) still has to be written back.
int AXI_LLC::find_mshr_by_victim_addr(const AXI_LLC_Regs_t &regs, uint32_t victim_addr) const {
  if (victim_addr == 0) return -1;
  for (uint32_t i = 0; i < mshr_limit(); ++i) {
    const auto &e = regs.mshr[i];
    if (e.valid && e.victim_dirty && !e.victim_writeback_done && e.victim_addr == victim_addr) return int(i);
  }
  return -1;
}

bool AXI_LLC::has_mshr_for_master(const AXI_LLC_Regs_t &regs, uint8_t master) const {
  for (uint32_t i = 0; i < mshr_limit(); ++i)
    if (regs.mshr[i].valid && !regs.mshr[i].is_prefetch && regs.mshr[i].master == master) return true;
  return false;
}

uint32_t AXI_LLC::count_free_mshrs(const AXI_LLC_Regs_t &regs) const {
  uint32_t n = 0;
  for (uint32_t i = 0; i < mshr_limit(); ++i) n += regs.mshr[i].valid ? 0 : 1;
  return n;
}

bool AXI_LLC::has_demand_mshr(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t i = 0; i < mshr_limit(); ++i)
    if (regs.mshr[i].valid && !regs.mshr[i].is_prefetch) return true;
  return false;
}

// Demand misses go to memory before prefetches.
int AXI_LLC::pick_mem_issue_slot(const AXI_LLC_Regs_t &regs) const {
  for (int want_prefetch = 0; want_prefetch < 2; ++want_prefetch)
    for (uint32_t i = 0; i < mshr_limit(); ++i)
      if (regs.mshr[i].valid && regs.mshr[i].is_prefetch == bool(want_prefetch) && !regs.mshr[i].mem_req_issued)
        return int(i);
  return -1;
}

int AXI_LLC::pick_refill_commit_slot(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t i = 0; i < mshr_limit(); ++i)
    if (regs.mshr[i].valid && regs.mshr[i].refill_valid) return int(i);
  return -1;
}

// Round-robin over the read masters; a request is taken only if it does not collide with anything in flight.
int AXI_LLC::pick_new_read_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_READ_MASTERS; ++off) {
    const uint32_t idx = (regs.rr_read_master_r + off) % NUM_READ_MASTERS;
    const auto &req = io.ext_in.upstream.read_req[idx];
    if (!req.valid || io.regs.read_resp_valid_r[idx] || regs.read_resp_valid_r[idx] || regs.lookup_valid_r) continue;
    const uint32_t line = line_addr(config_, req.addr);
    if (has_pending_upstream_write_line(line) || write_line_pending(regs, line) || victim_line_pending(regs, line) ||
        find_mshr_by_line_addr(regs, line) >= 0)
      continue;
    if (!req.bypass && !allow_parallel_demand_mshr(uint8_t(idx)) && has_mshr_for_master(regs, uint8_t(idx))) continue;
    if (find_free_mshr(regs) < 0) continue;
    return int(idx);
  }
  return -1;
}

int AXI_LLC::pick_new_write_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t idx = (regs.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    const auto &req = io.ext_in.upstream.write_req[idx];
    if (!req.valid || write_queue_full(regs, uint8_t(idx))) continue;
    if (victim_line_pending(regs, line_addr(config_, req.addr))) continue;
    return int(idx);
  }
  return -1;
}

bool AXI_LLC::write_queue_full(const AXI_LLC_Regs_t &regs, uint8_t master) const {
  return master < NUM_WRITE_MASTERS && regs.write_q_count_r[master] >= MAX_WRITE_OUTSTANDING;
}

const AXI_LLCWritePendingReq_t *AXI_LLC::write_queue_front(const AXI_LLC_Regs_t &regs, uint8_t master) const {
  if (master >= NUM_WRITE_MASTERS || regs.write_q_count_r[master] == 0) return nullptr;
  const auto &e = regs.write_q[master][regs.write_q_head_r[master] % MAX_WRITE_OUTSTANDING];
  return e.valid ? &e : nullptr;
}

int AXI_LLC::pick_write_lookup_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t m = (regs.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    if (regs.write_ctx[m].valid && regs.write_ctx[m].lookup_pending) return int(m);
  }
  return -1;
}

int AXI_LLC::pick_bypass_write_issue_master(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t m = (regs.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    const auto &ctx = regs.write_ctx[m];
    if (ctx.valid && ctx.bypass && !ctx.mem_done && !ctx.mem_issued) return int(m);
  }
  return -1;
}

// The bypass write whose memory response is outstanding.
int AXI_LLC::find_bypass_write_mem_owner(const AXI_LLC_Regs_t &regs) const {
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    const auto &ctx = regs.write_ctx[m];
    if (ctx.valid && ctx.bypass && ctx.mem_issued && !ctx.mem_done) return int(m);
  }
  return -1;
}

int AXI_LLC::find_prefetch_queue_slot(const AXI_LLC_Regs_t &regs, uint32_t line) const {
  for (uint32_t i = 0; i < AXI_LLC_MAX_PREFETCH_QUEUE; ++i)
    if (regs.prefetch_q[i].valid && regs.prefetch_q[i].line_addr == line) return int(i);
  return -1;
}

int AXI_LLC::find_free_prefetch_queue_slot(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t i = 0; i < AXI_LLC_MAX_PREFETCH_QUEUE; ++i)
    if (!regs.prefetch_q[i].valid) return int(i);
  return -1;
}

int AXI_LLC::pick_prefetch_queue_slot(const AXI_LLC_Regs_t &regs) const {
  for (uint32_t i = 0; i < AXI_LLC_MAX_PREFETCH_QUEUE; ++i)
    if (regs.prefetch_q[i].valid) return int(i);
  return -1;
}

bool AXI_LLC::prefetch_candidate_exists(const AXI_LLC_Regs_t &regs, uint32_t line) const {
  return find_prefetch_queue_slot(regs, line) >= 0 ||
         (regs.lookup_valid_r && line_addr(config_, regs.lookup_addr_r) == line) ||
         find_mshr_by_line_addr(regs, line) >= 0;
}

// A prefetch never takes the last free MSHR and only one is in flight at a time.
bool AXI_LLC::can_allocate_prefetch_mshr(const AXI_LLC_Regs_t &regs) const {
  if (llc_bop().on) {
    if (int(count_free_mshrs(regs)) <= llc_bop().res) return false;
    int n = 0;
    for (uint32_t i = 0; i < mshr_limit(); ++i) n += regs.mshr[i].valid && regs.mshr[i].is_prefetch;
    return n < llc_bop().inf;
  }
  if (count_free_mshrs(regs) <= 1) return false;
  for (uint32_t i = 0; i < mshr_limit(); ++i)
    if (regs.mshr[i].valid && regs.mshr[i].is_prefetch) return false;
  return true;
}

bool AXI_LLC::line_has_valid_meta(const AXI_LLC_Bytes_t &valid_payload, const AXI_LLC_Bytes_t &meta_payload, uint32_t tag,
                                  int *hit_way, int *first_invalid_way, AXI_LLCMetaEntry_t *hit_meta) const {
  *hit_way = -1;
  *first_invalid_way = -1;
  if (hit_meta != nullptr) *hit_meta = {};
  for (uint32_t way = 0; way < config_.ways; ++way) {
    const auto meta = decode_meta(meta_payload, way);
    const bool valid = decode_valid_bit(valid_payload, way, false);
    if (!valid && *first_invalid_way < 0) *first_invalid_way = int(way);
    if (valid && meta.tag == tag) {
      *hit_way = int(way);
      if (hit_meta != nullptr) *hit_meta = meta;
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------------------------
// Next-line prefetch: two consecutive demand misses start a stream; the next `degree` lines are queued and looked up
// when the cache is quiet.
// ---------------------------------------------------------------------------------------------

void AXI_LLC::try_schedule_prefetch(const AXI_LLCMissEntry_t &entry) {
  auto &w = io.reg_write;
  const bool trains = config_.prefetch_enable && !entry.bypass && !entry.is_prefetch && entry.prefetch_train &&
                      config_.line_bytes != 0;
  const bool in_stream = trains && io.regs.prefetch_stream_valid_r &&
                         io.regs.prefetch_last_miss_line_r + config_.line_bytes == entry.line_addr;
  if (in_stream) {
    const uint32_t degree = std::max<uint32_t>(1, config_.prefetch_degree);
    for (uint32_t d = 1; d <= degree; ++d) {
      const uint32_t next = entry.line_addr + d * config_.line_bytes;
      if (prefetch_candidate_exists(w, next)) continue;
      const int slot = find_free_prefetch_queue_slot(w);
      if (slot < 0) break;
      w.prefetch_q[slot].valid = true;
      w.prefetch_q[slot].line_addr = next;
    }
  }
  if (trains || (!entry.bypass && !entry.is_prefetch)) {
    w.prefetch_stream_valid_r = true;
    w.prefetch_last_miss_line_r = entry.line_addr;
  }
}

void AXI_LLC::try_launch_prefetch_lookup() {
  const auto &regs = io.reg_write;
  if (llc_bop().on) {
    if (bop_q_.empty() || regs.lookup_valid_r || !io.ext_in.mem.prefetch_allow || !can_allocate_prefetch_mshr(regs)) return;
    bop_ev_.launched = true;   // seq pops the head (launched or dropped as already in flight)
    const uint32_t a = bop_q_.front();
    if (find_mshr_by_line_addr(regs, a) >= 0) return;
    bop_ev_.sent = true;
    start_lookup(a, uint8_t(config_.line_bytes - 1), kInvalidReadMaster, uint8_t(AXI_LLC_MAX_PREFETCH_QUEUE), true, false, false,
                 false);
    return;
  }
  if (regs.lookup_valid_r || !io.ext_in.mem.prefetch_allow || has_demand_mshr(regs) ||
      count_free_mshrs(regs) != mshr_limit() || regs.prefetch_quiet_cycles_r == 0)
    return;
  const int slot = pick_prefetch_queue_slot(regs);
  if (slot < 0) return;
  start_lookup(regs.prefetch_q[slot].line_addr, uint8_t(config_.line_bytes - 1), kInvalidReadMaster, uint8_t(slot),
               true, false, false, false);
}

// ---------------------------------------------------------------------------------------------
// Lookup pipeline: one lookup at a time. It is started (start_lookup), issues the table read (drive_lookup_request),
// and completes when the tables answer (try_complete_lookup).
// ---------------------------------------------------------------------------------------------

void AXI_LLC::start_lookup(uint32_t addr, uint8_t size, uint8_t master, uint8_t id, bool prefetch, bool invalidate,
                           bool write, bool bypass) {
  auto &w = io.reg_write;
  w.lookup_valid_r = true;
  w.lookup_issued_r = false;
  w.lookup_addr_r = addr;
  w.lookup_size_r = size;
  w.lookup_master_r = master;
  w.lookup_id_r = id;
  w.lookup_is_prefetch_r = prefetch;
  w.lookup_is_invalidate_r = invalidate;
  w.lookup_is_write_r = write;
  w.lookup_is_bypass_r = bypass;
  w.state = AXI_LLCState::kLookup;
}

// The lookup is done (flags of a finished lookup are never read: every start rewrites them).
void AXI_LLC::end_lookup(AXI_LLCState next) {
  auto &w = io.reg_write;
  w.lookup_valid_r = false;
  w.lookup_issued_r = false;
  w.lookup_is_prefetch_r = w.lookup_is_invalidate_r = w.lookup_is_write_r = w.lookup_is_bypass_r = false;
  w.state = next;
}

// The lookup could not complete: ask the tables again next cycle.
void AXI_LLC::retry_lookup(AXI_LLCState next) {
  io.reg_write.lookup_issued_r = false;
  io.reg_write.state = next;
}

void AXI_LLC::drop_prefetch_slot(uint8_t slot) {
  if (slot < AXI_LLC_MAX_PREFETCH_QUEUE) io.reg_write.prefetch_q[slot] = {};
}

bool AXI_LLC::try_launch_pending_write_lookup() {
  if (io.reg_write.lookup_valid_r) return false;
  const int m = pick_write_lookup_master(io.reg_write);
  if (m < 0) return false;
  auto &ctx = io.reg_write.write_ctx[m];
  start_lookup(ctx.addr, ctx.total_size, uint8_t(m), ctx.id, false, false, true, ctx.bypass);
  ctx.lookup_pending = false;
  io.reg_write.rr_write_master_r = uint8_t((m + 1) % NUM_WRITE_MASTERS);
  return true;
}

void AXI_LLC::accept_maintenance_request() {
  io.ext_out.mem.invalidate_line_accepted = false;
  if (!io.ext_in.mem.invalidate_line_valid || io.ext_in.mem.invalidate_all) return;
  if (!can_accept_invalidate_line_now(line_addr(config_, io.ext_in.mem.invalidate_line_addr))) return;
  if (io.reg_write.lookup_valid_r) return;
  start_lookup(io.ext_in.mem.invalidate_line_addr, 0, 0, 0, false, true, false, false);
  io.ext_out.mem.invalidate_line_accepted = true;
}

void AXI_LLC::drive_lookup_request() {
  if (!io.regs.lookup_valid_r || !io.reg_write.lookup_valid_r || io.regs.lookup_issued_r) return;
  const uint32_t line = line_addr(config_, io.reg_write.lookup_addr_r);
  if (victim_line_pending(io.reg_write, line) || llc_table_write_busy(io.table_out)) return;
  if (!io.reg_write.lookup_is_prefetch_r &&
      find_mshr_by_line_addr(io.reg_write, line) >= 0)
    return;
  const uint32_t index = set_index(config_, io.reg_write.lookup_addr_r);
  for (auto *r : {&io.table_out.data, &io.table_out.meta, &io.table_out.valid, &io.table_out.repl}) {
    r->enable = true;
    r->index = index;
  }
  io.reg_write.lookup_issued_r = true;
  io.reg_write.state = AXI_LLCState::kLookup;
}

bool AXI_LLC::try_complete_lookup() {
  const auto &r = io.regs;
  const bool valid_table_ready = io.lookup_in.valid_valid && io.lookup_in.valid.size() >= valid_row_bytes(config_);
  if (!r.lookup_valid_r || !r.lookup_issued_r || !io.lookup_in.data_valid || !io.lookup_in.meta_valid ||
      !valid_table_ready || !io.lookup_in.repl_valid)
    return false;

  const bool table_busy = llc_table_write_busy(io.table_out);
  lookup_tbl_busy_ = table_busy;

  const uint32_t set = set_index(config_, r.lookup_addr_r), tag = tag_of(config_, r.lookup_addr_r),
                 line = line_addr(config_, r.lookup_addr_r);
  int hit_way = -1, first_invalid_way = -1;
  AXI_LLCMetaEntry_t hit_meta{};
  (void)line_has_valid_meta(io.lookup_in.valid, io.lookup_in.meta, tag, &hit_way, &first_invalid_way, &hit_meta);

  if (hit_way >= 0) return complete_hit_lookup(table_busy, set, tag, line, hit_way, hit_meta);
  if (r.lookup_is_write_r) return complete_write_miss(table_busy, set, tag, line, first_invalid_way);
  if (r.lookup_is_invalidate_r) {
    end_lookup(AXI_LLCState::kIdle);
    return true;
  }
  return complete_read_miss(set, tag, line, first_invalid_way);
}

bool AXI_LLC::complete_hit_lookup(bool table_busy, uint32_t set, uint32_t tag, uint32_t line, int hit_way,
                                  const AXI_LLCMetaEntry_t &hit_meta) {
  const auto &r = io.regs;
  auto &t = io.table_out;
  const bool hit_dirty = (hit_meta.flags & AXI_LLC_META_DIRTY) != 0;
  const uint32_t repl_next = uint32_t((hit_way + 1) % config_.ways);

  if (r.lookup_is_write_r) {
    if (table_busy) {
      retry_lookup(AXI_LLCState::kLookup);
      return true;
    }
    if (!r.lookup_is_bypass_r) {
      // A miss that is about to refill this very way gets the written line as its dirty victim.
      for (uint32_t slot = 0; slot < mshr_limit(); ++slot) {
        const auto &prev = r.mshr[slot];
        if (!prev.valid || prev.bypass || prev.is_write || prev.refill_committed) continue;
        if (prev.set != set || prev.way != uint8_t(hit_way) || prev.line_addr == line) continue;
        auto &e = io.reg_write.mshr[slot];
        e.victim_dirty = true;
        e.victim_writeback_done = false;
        e.victim_addr = line;
      }
    }
    AXI_LLCMetaEntry_t meta{};
    meta.tag = tag;
    // a bypass write only refreshes the line: it keeps the dirty state it had
    meta.flags = uint8_t(AXI_LLC_META_VALID | (r.lookup_is_bypass_r ? (hit_meta.flags & AXI_LLC_META_DIRTY) : AXI_LLC_META_DIRTY));
    write_line_tables(set, uint32_t(hit_way), meta, repl_next);
    if (llc_repl() != LlcRepl::kRR && !r.lookup_is_bypass_r) touch_repl_at_lookup(set, uint32_t(hit_way), false, false);
    end_lookup(AXI_LLCState::kIdle);
    io.reg_write.dirty_line_count_r =
        update_dirty_line_count(r.dirty_line_count_r, hit_dirty, (meta.flags & AXI_LLC_META_DIRTY) != 0);
    io.reg_write.write_ctx[r.lookup_master_r].cache_done = true;
    return true;
  }
  if (r.lookup_is_invalidate_r) {
    if (table_busy) {
      retry_lookup(AXI_LLCState::kLookup);
      return true;
    }
    write_valid_table(set, uint32_t(hit_way), false);
    end_lookup(AXI_LLCState::kIdle);
    io.reg_write.dirty_line_count_r = update_dirty_line_count(r.dirty_line_count_r, hit_dirty, false);
    return true;
  }
  if (r.lookup_is_prefetch_r) {  // already cached
    drop_prefetch_slot(r.lookup_id_r);
    end_lookup(AXI_LLCState::kIdle);
    return true;
  }
  if (!r.lookup_is_bypass_r && table_busy) {
    retry_lookup(AXI_LLCState::kLookup);
    return true;
  }
  if (!enqueue_read_response(r.lookup_master_r, r.lookup_id_r, WideReadData_t{})) {
    retry_lookup(AXI_LLCState::kLookup);
    return true;
  }
  if (!r.lookup_is_bypass_r) {  // a demand hit updates the replacement state and consumes the prefetch mark
    put_repl(t.repl, set, repl_after_use(decode_repl(io.lookup_in.repl), config_.ways, uint32_t(hit_way)));
    if (hit_meta.flags & AXI_LLC_META_PREFETCH) {
      if (llc_bop().on && r.lookup_master_r == MASTER_DCACHE_R) {
        bop_ev_.trig = true;   // EXP-048 trigger: the first demand use of a prefetched line
        bop_ev_.trig_pf = true;
        bop_ev_.trig_line = line_addr(config_, r.lookup_addr_r);
      }
      AXI_LLCMetaEntry_t meta = hit_meta;
      meta.flags = uint8_t(meta.flags & ~AXI_LLC_META_PREFETCH);
      put_meta(t.meta, set, uint32_t(hit_way), meta);
    }
  }
  end_lookup(AXI_LLCState::kIdle);
  return true;
}

bool AXI_LLC::complete_write_miss(bool table_busy, uint32_t set, uint32_t tag, uint32_t line, int first_invalid_way) {
  const auto &r = io.regs;
  auto &ctx = io.reg_write.write_ctx[r.lookup_master_r];
  if (r.lookup_is_bypass_r) {  // bypass writes do not allocate
    end_lookup(AXI_LLCState::kIdle);
    ctx.cache_done = true;
    return true;
  }
  const int way_idx = pick_unreserved_way(r, set, first_invalid_way, decode_repl(io.lookup_in.repl), line);
  if (way_idx < 0) {
    retry_lookup(AXI_LLCState::kMiss);
    return true;
  }
  const uint8_t way = uint8_t(way_idx);
  const auto victim_meta = decode_meta(io.lookup_in.meta, way);
  const bool victim_dirty = decode_valid_bit(io.lookup_in.valid, way, false) && (victim_meta.flags & AXI_LLC_META_DIRTY);
  const uint32_t victim_addr = build_line_addr_from_tag_set(config_, victim_meta.tag, set);
  const uint32_t repl_next = uint32_t((way + 1) % config_.ways);

  if (!is_full_line_write(config_, ctx.addr, ctx.strobe, ctx.total_size)) {
    // partial write: fetch the line first (the write is merged when the refill is committed)
    const int free_slot = find_free_mshr(r);
    if (find_mshr_by_line_addr(r, line) >= 0 || free_slot < 0) {
      retry_lookup(AXI_LLCState::kMiss);
      return true;
    }
    auto &e = io.reg_write.mshr[free_slot];
    e = {};
    e.valid = true;
    e.is_write = true;
    e.addr = r.lookup_addr_r;
    e.line_addr = line;
    e.set = set;
    e.tag = tag;
    e.way = way;
    e.total_size = ctx.total_size;
    e.master = r.lookup_master_r;
    e.id = r.lookup_id_r;
    e.epoch = r.invalidate_epoch_r;
    e.victim_dirty = victim_dirty;
    e.victim_writeback_done = !victim_dirty;
    e.victim_addr = victim_dirty ? victim_addr : 0;
    ctx.cache_pending = true;
    ctx.set = set;
    ctx.way = way;
    ctx.repl_next_way = repl_next;
    ctx.tag = tag;
    touch_repl_at_lookup(set, way, true, false);
    end_lookup(AXI_LLCState::kMiss);
    return true;
  }

  // full-line write: no fetch; a dirty victim goes to memory first, then the line is written
  if (victim_dirty) {
    if (io.reg_write.victim_wb_valid_r) {
      retry_lookup(AXI_LLCState::kLookup);
      return true;
    }
    arm_victim_writeback(true, r.lookup_master_r, 0, victim_addr);
    ctx.cache_pending = true;
    ctx.victim_dirty = true;
    ctx.victim_addr = victim_addr;
    ctx.set = set;
    ctx.way = way;
    ctx.repl_next_way = repl_next;
    ctx.tag = tag;
    touch_repl_at_lookup(set, way, true, false);
  } else {
    if (table_busy) {
      retry_lookup(AXI_LLCState::kLookup);
      return true;
    }
    write_line_tables(set, way, AXI_LLCMetaEntry_t{tag, uint8_t(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY)}, repl_next);
    touch_repl_at_lookup(set, way, true, false);
    io.reg_write.dirty_line_count_r = update_dirty_line_count(r.dirty_line_count_r, false, true);
    ctx.cache_done = true;
  }
  end_lookup(AXI_LLCState::kIdle);
  return true;
}

// A demand read or prefetch missed: allocate an MSHR (the refill goes to a way picked now).
bool AXI_LLC::complete_read_miss(uint32_t set, uint32_t tag, uint32_t line, int first_invalid_way) {
  const auto &r = io.regs;
  const bool prefetch = r.lookup_is_prefetch_r;
  // A prefetch that cannot proceed is dropped; a demand read retries.
  auto give_up = [&]() {
    if (prefetch) {
      drop_prefetch_slot(r.lookup_id_r);
      end_lookup(AXI_LLCState::kMiss);
    } else {
      retry_lookup(AXI_LLCState::kMiss);
    }
    return true;
  };
  if (llc_bop().on && !prefetch && !r.lookup_is_bypass_r && r.lookup_master_r == MASTER_DCACHE_R) {
    bop_ev_.trig = true;   // EXP-048 trigger: an L1D read missing in the LLC (recorded at every attempt; the same line)
    bop_ev_.trig_line = line;
  }
  if (find_mshr_by_line_addr(r, line) >= 0) return give_up();
  const int free_slot = find_free_mshr(r);
  if (free_slot < 0 || (prefetch && !can_allocate_prefetch_mshr(r))) return give_up();

  if (AXI_KIT_LLC_DCACHE_READ_MISS_NOALLOC != 0 && !prefetch && !r.lookup_is_bypass_r &&
      r.lookup_master_r == MASTER_DCACHE_R) {
    // data-cache read misses are fetched from memory without allocating a line
    auto &e = io.reg_write.mshr[free_slot];
    e = {};
    e.valid = true;
    e.bypass = true;
    e.addr = e.line_addr = line;
    e.total_size = uint8_t(config_.line_bytes - 1);
    e.master = r.lookup_master_r;
    e.id = r.lookup_id_r;
    e.epoch = r.invalidate_epoch_r;
    end_lookup(AXI_LLCState::kMiss);
    return true;
  }

  const int way_idx = pick_unreserved_way(r, set, first_invalid_way, decode_repl(io.lookup_in.repl), line);
  if (way_idx < 0) {
    retry_lookup(AXI_LLCState::kMiss);
    return true;
  }
  const uint8_t way = uint8_t(way_idx);
  const auto victim_meta = decode_meta(io.lookup_in.meta, way);
  auto &e = io.reg_write.mshr[free_slot];
  e = {};
  e.valid = true;
  e.bypass = r.lookup_is_bypass_r;
  e.is_prefetch = prefetch;
  e.prefetch_train = !prefetch;
  e.addr = r.lookup_addr_r;
  e.line_addr = r.lookup_is_bypass_r ? r.lookup_addr_r : line;
  e.set = set;
  e.tag = tag;
  e.way = way;
  e.total_size = r.lookup_size_r;
  e.master = r.lookup_master_r;
  e.id = prefetch ? 0 : r.lookup_id_r;
  e.epoch = r.invalidate_epoch_r;
  e.victim_dirty = decode_valid_bit(io.lookup_in.valid, way, false) && (victim_meta.flags & AXI_LLC_META_DIRTY);
  e.victim_writeback_done = !e.victim_dirty;
  e.victim_addr = e.victim_dirty ? build_line_addr_from_tag_set(config_, victim_meta.tag, set) : 0;
  if (!e.bypass) touch_repl_at_lookup(set, way, true, prefetch);
  end_lookup(AXI_LLCState::kMiss);
  if (prefetch) drop_prefetch_slot(r.lookup_id_r);
  return true;
}

// ---------------------------------------------------------------------------------------------
// Upstream read responses
// ---------------------------------------------------------------------------------------------

void AXI_LLC::drive_read_responses() {
  auto &w = io.reg_write;
  for (uint8_t i = 0; i < NUM_READ_MASTERS; ++i) {
    io.ext_out.upstream.read_resp[i].valid = io.regs.read_resp_valid_r[i];
    io.ext_out.upstream.read_resp[i].data = io.regs.read_resp_data_r[i];
    io.ext_out.upstream.read_resp[i].id = io.regs.read_resp_id_r[i];
    // Upstream ready is sampled one phase earlier than the consumer computes its backpressure: a freshly produced
    // response stays visible for one full cycle before the previous cycle's ready may retire it.
    if (io.regs.read_resp_valid_r[i] && io.regs.read_resp_fresh_r[i]) {
      w.read_resp_fresh_r[i] = false;
    } else if (io.regs.read_resp_valid_r[i] && io.ext_in.upstream.read_resp[i].ready) {
      if (io.regs.read_resp_q_count_r[i] != 0) {
        promote_read_response(i);
      } else {
        w.read_resp_valid_r[i] = false;
        w.read_resp_fresh_r[i] = false;
        w.read_resp_id_r[i] = 0;
        w.read_resp_data_r[i].clear();
      }
    }
    if (!w.read_resp_valid_r[i] && io.regs.read_resp_q_count_r[i] != 0) promote_read_response(i);
  }
}

// ---------------------------------------------------------------------------------------------
// Write path: requests queue per master, each runs as a write context (lookup -> cache write, or straight to memory for
// bypass); a dirty victim is written back through the single victim slot before its way is reused.
// ---------------------------------------------------------------------------------------------

// A write that waited for its dirty victim's write-back writes the new line into the table.
bool AXI_LLC::try_commit_write_after_victim(uint8_t master) {
  if (master >= NUM_WRITE_MASTERS) return false;
  auto &ctx = io.reg_write.write_ctx[master];
  if (!ctx.valid || ctx.bypass || !ctx.cache_pending || !ctx.victim_mem_done || ctx.cache_done) return false;
  if (llc_table_write_busy(io.table_out)) return false;
  write_line_tables(ctx.set, ctx.way, AXI_LLCMetaEntry_t{ctx.tag, uint8_t(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY)},
                    ctx.repl_next_way);
  ctx.cache_done = true;
  ctx.cache_pending = false;
  ctx.victim_mem_done = false;
  return true;
}

// Memory acknowledged a write: the victim write-back in flight, else the bypass write that owns the channel.
void AXI_LLC::on_mem_write_resp() {
  const auto &r = io.regs;
  auto &w = io.reg_write;
  if (!(r.victim_wb_valid_r && r.victim_wb_issued_r)) {
    const int owner = find_bypass_write_mem_owner(r);
    if (owner >= 0) {
      w.write_ctx[owner].mem_done = true;
      w.write_ctx[owner].mem_resp_code = io.ext_in.mem.write_resp;
    }
    return;
  }
  w.victim_wb_valid_r = false;
  w.victim_wb_issued_r = false;
  w.victim_wb_for_write_r = false;
  w.victim_wb_write_master_r = 0;
  w.victim_wb_mshr_slot_r = 0;
  w.victim_wb_addr_r = 0;
  w.victim_wb_issue_cycle_r = 0;
  w.victim_wb_strobe_r.clear();

  if (r.victim_wb_for_write_r) {
    const uint8_t master = r.victim_wb_write_master_r;
    w.write_ctx[master].victim_mem_done = true;
    (void)try_commit_write_after_victim(master);
    return;
  }
  const uint8_t slot = r.victim_wb_mshr_slot_r;
  if (slot == AXI_LLC_INVALID_VICTIM_MSHR_SLOT) {  // snapshot taken from the queue: pop it
    if (r.read_victim_wb_q_count_r != 0) {
      const uint8_t head = r.read_victim_wb_q_head_r % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH;
      const auto &queued = r.read_victim_wb_q[head];
      if (queued.valid && queued.victim_addr == r.victim_wb_addr_r) {
        w.read_victim_wb_q[head] = {};
        w.read_victim_wb_q_head_r = uint8_t((head + 1u) % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH);
        w.read_victim_wb_q_count_r--;
      }
    }
    return;
  }
  int owner = -1;
  if (slot < mshr_limit() && r.mshr[slot].valid && r.mshr[slot].victim_dirty &&
      r.mshr[slot].victim_addr == r.victim_wb_addr_r)
    owner = int(slot);
  if (owner < 0) owner = find_mshr_by_victim_addr(r, r.victim_wb_addr_r);
  if (owner >= 0) w.mshr[owner].victim_writeback_done = true;
}

void AXI_LLC::drive_write_path() {
  const auto &r = io.regs;
  auto &w = io.reg_write;
  auto &out = io.ext_out;

  for (uint8_t i = 0; i < NUM_WRITE_MASTERS; ++i) {
    out.upstream.write_resp[i].valid = r.write_resp_valid_r[i];
    out.upstream.write_resp[i].id = r.write_resp_id_r[i];
    out.upstream.write_resp[i].resp = r.write_resp_code_r[i];
    if (r.write_resp_valid_r[i] && io.ext_in.upstream.write_resp[i].ready) {
      w.write_resp_valid_r[i] = false;
      w.write_resp_id_r[i] = 0;
      w.write_resp_code_r[i] = 0;
      w.write_resp_line_addr_r[i] = 0;
    }
  }

  out.mem.write_resp_ready = true;
  if (io.ext_in.mem.write_resp_valid) on_mem_write_resp();

  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m)
    if (try_commit_write_after_victim(m)) break;

  // finished writes answer their master
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m) {
    const auto &ctx = w.write_ctx[m];
    if (!ctx.valid || w.write_resp_valid_r[m]) continue;
    if (!(ctx.bypass ? ctx.mem_done : (ctx.cache_done && !ctx.cache_pending))) continue;
    w.write_resp_valid_r[m] = true;
    w.write_resp_id_r[m] = ctx.id;
    w.write_resp_code_r[m] = ctx.bypass ? ctx.mem_resp_code : 0;
    w.write_resp_line_addr_r[m] = ctx.line_addr;
    w.write_ctx[m] = {};
  }

  // the victim slot takes the oldest queued read-miss snapshot
  const int bypass_owner = find_bypass_write_mem_owner(r);
  if (!w.victim_wb_valid_r && r.read_victim_wb_q_count_r != 0 && bypass_owner < 0) {
    const auto &queued = r.read_victim_wb_q[r.read_victim_wb_q_head_r % AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH];
    if (queued.valid) {
      w.victim_wb_valid_r = true;
      w.victim_wb_issued_r = false;
      w.victim_wb_for_write_r = false;
      w.victim_wb_write_master_r = 0;
      w.victim_wb_mshr_slot_r = AXI_LLC_INVALID_VICTIM_MSHR_SLOT;
      w.victim_wb_addr_r = queued.victim_addr;
      w.victim_wb_strobe_r = queued.victim_strobe;
    }
  }

  if (w.victim_wb_valid_r) {  // the write channel belongs to the victim write-back
    if (!w.victim_wb_issued_r) {
      if (bypass_owner >= 0) return;
      out.mem.write_req_valid = true;
      out.mem.write_req_addr = w.victim_wb_addr_r;
      out.mem.write_req_strobe = w.victim_wb_strobe_r;
      out.mem.write_req_size = uint8_t(config_.line_bytes - 1);
      out.mem.write_req_id = 0;
      if (io.ext_in.mem.write_req_ready) {
        w.victim_wb_issued_r = true;
        w.victim_wb_issue_cycle_r = uint64_t(sim_time);
      }
    }
    return;
  }

  const int bypass_master = pick_bypass_write_issue_master(r);
  if (bypass_master >= 0) {
    const auto &ctx = r.write_ctx[bypass_master];
    out.mem.write_req_valid = true;
    out.mem.write_req_addr = ctx.addr;
    out.mem.write_req_data = ctx.data;
    out.mem.write_req_strobe = ctx.strobe;
    out.mem.write_req_size = ctx.total_size;
    out.mem.write_req_id = ctx.id;
    if (io.ext_in.mem.write_req_ready) {
      w.write_ctx[bypass_master].mem_issued = true;
      w.write_ctx[bypass_master].mem_issue_cycle = uint64_t(sim_time);
    }
  }

  // accept one upstream write into its master's queue
  const int wm = pick_new_write_master(r);
  if (wm >= 0) {
    const auto &req = io.ext_in.upstream.write_req[wm];
    out.upstream.write_req[wm].ready = true;
    const uint8_t slot = r.write_q_tail_r[wm] % MAX_WRITE_OUTSTANDING;
    auto &e = w.write_q[wm][slot];
    e = {};
    e.valid = true;
    e.bypass = req.bypass;
    e.master = uint8_t(wm);
    e.id = req.id;
    e.total_size = req.total_size;
    e.addr = req.addr;
    e.wdata = req.wdata;
    e.wstrb = req.wstrb;
    w.write_q_tail_r[wm] = uint8_t((slot + 1) % MAX_WRITE_OUTSTANDING);
    w.write_q_count_r[wm] = uint8_t(r.write_q_count_r[wm] + 1);
    w.rr_write_master_r = uint8_t((wm + 1) % NUM_WRITE_MASTERS);
  }

  // a queued write becomes the master's write context when the previous one is fully retired
  for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
    const uint32_t m = (r.rr_write_master_r + off) % NUM_WRITE_MASTERS;
    if (w.write_ctx[m].valid || w.write_resp_valid_r[m]) continue;
    const auto *e = write_queue_front(w, uint8_t(m));
    if (e == nullptr || victim_line_pending(w, line_addr(config_, e->addr))) continue;
    auto &ctx = w.write_ctx[m];
    ctx = {};
    ctx.valid = true;
    ctx.bypass = e->bypass;
    ctx.lookup_pending = true;
    ctx.id = e->id;
    ctx.total_size = e->total_size;
    ctx.addr = e->addr;
    ctx.line_addr = line_addr(config_, e->addr);
    ctx.data = e->wdata;
    ctx.strobe = e->wstrb;
    w.write_q[m][w.write_q_head_r[m] % MAX_WRITE_OUTSTANDING] = {};
    w.write_q_head_r[m] = uint8_t((w.write_q_head_r[m] + 1) % MAX_WRITE_OUTSTANDING);
    w.write_q_count_r[m] = uint8_t(w.write_q_count_r[m] - 1);
  }

  (void)try_launch_pending_write_lookup();
}

// ---------------------------------------------------------------------------------------------
// Memory read path: issue MSHR requests, take refill data, commit refills into the cache
// ---------------------------------------------------------------------------------------------

// A dirty victim is written back from a snapshot; the refill may proceed once the snapshot is held by the victim slot
// or queue. Returns false while a write still owns the victim's cache state.
bool AXI_LLC::externalize_read_victim(uint8_t slot, const AXI_LLCMissEntry_t &entry) {
  if (!entry.victim_dirty || entry.victim_writeback_done) return true;
  if (victim_snapshot_waits_for_write_resolution(io.regs, entry.victim_addr)) return false;
  const auto &w = io.reg_write;
  const bool in_victim_slot = w.victim_wb_valid_r && !w.victim_wb_for_write_r && w.victim_wb_addr_r == entry.victim_addr &&
                              w.victim_wb_mshr_slot_r == slot;
  if (in_victim_slot || read_victim_snapshot_present(w, entry.victim_addr)) return true;
  if (!w.victim_wb_valid_r) {
    arm_victim_writeback(false, 0, slot, entry.victim_addr);
    return true;
  }
  return enqueue_read_victim_snapshot(slot, entry.victim_addr, full_line_strobe(config_));
}

void AXI_LLC::release_mshr(uint8_t slot, bool clear_victim_slot) {
  auto &w = io.reg_write;
  if (clear_victim_slot && w.victim_wb_valid_r && !w.victim_wb_for_write_r && w.victim_wb_mshr_slot_r == slot)
    w.victim_wb_mshr_slot_r = AXI_LLC_INVALID_VICTIM_MSHR_SLOT;
  w.mshr[slot] = {};
  w.state = io.regs.lookup_valid_r ? AXI_LLCState::kLookup : AXI_LLCState::kIdle;
}

void AXI_LLC::drive_mem_read_path() {
  const auto &r = io.regs;
  auto &w = io.reg_write;
  io.ext_out.mem.read_resp_ready = true;

  // refill data arrives (responses are tagged with the MSHR index)
  if (io.ext_in.mem.read_resp_valid) {
    const uint32_t id = io.ext_in.mem.read_resp_id;
    if (id < mshr_limit() && r.mshr[id].valid) {
      const auto &prev = r.mshr[id];
      if (prev.bypass && enqueue_read_response(prev.master, prev.id, io.ext_in.mem.read_resp_data)) {
        w.mshr[id] = {};
        w.state = r.lookup_valid_r ? AXI_LLCState::kLookup : AXI_LLCState::kIdle;
      } else {
        auto &e = w.mshr[id];
        e.refill_valid = true;
        e.refill_data = io.ext_in.mem.read_resp_data;
        e.refill_committed = e.bypass;
        w.state = AXI_LLCState::kRefill;
      }
    }
  }

  // issue the next memory read; a bypass read allocated this very cycle may go out right away
  int slot = pick_mem_issue_slot(r);
  bool from_write_regs = false;
  if (slot < 0) {
    for (uint32_t cand = 0; cand < mshr_limit(); ++cand) {
      const auto &e = w.mshr[cand];
      if (e.valid && e.bypass && !e.mem_req_issued && !e.refill_valid && !r.mshr[cand].valid) {
        slot = int(cand);
        from_write_regs = true;
        break;
      }
    }
  }
  if (slot >= 0) {
    const auto &e = from_write_regs ? w.mshr[slot] : r.mshr[slot];
    io.ext_out.mem.read_req_valid = true;
    io.ext_out.mem.read_req_addr = e.line_addr;
    io.ext_out.mem.read_req_size = e.bypass ? e.total_size : uint8_t(config_.line_bytes - 1);
    io.ext_out.mem.read_req_id = uint8_t(slot);
    if (io.ext_in.mem.read_req_ready) {
      w.mshr[slot].mem_req_issued = true;
      w.mshr[slot].mem_issue_cycle = uint64_t(sim_time);
    }
  }

  // commit one refill
  const int cs = pick_refill_commit_slot(r);
  if (cs < 0) return;
  const uint8_t commit_slot = uint8_t(cs);
  auto &e = w.mshr[commit_slot];
  const auto respond = [&]() {  // line fills carry no values; bypass reads return the memory data
    WideReadData_t data = e.refill_data;
    if (!e.bypass) data.clear();
    return enqueue_read_response(e.master, e.id, data);
  };

  if (e.refill_valid && e.epoch != r.invalidate_epoch_r && !e.is_write) {  // the cache was invalidated meanwhile
    if (!externalize_read_victim(commit_slot, e)) return;
    if (e.is_prefetch) {
      release_mshr(commit_slot, false);
      return;
    }
    if (!respond()) return;
    release_mshr(commit_slot, true);
    return;
  }

  if (!e.refill_committed && !e.bypass) {  // write the refilled line into the tables
    if (!externalize_read_victim(commit_slot, e)) return;
    if (llc_table_write_busy(io.table_out)) {
      w.state = AXI_LLCState::kRefill;
      return;
    }
    AXI_LLCMetaEntry_t meta{e.tag, uint8_t(AXI_LLC_META_VALID | (e.is_prefetch ? AXI_LLC_META_PREFETCH : 0))};
    if (e.is_write) meta.flags = uint8_t(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY);
    if (llc_bop().on && !e.is_write) {   // EXP-048: the recent-requests table is written at fill time
      bop_ev_.fill = true;
      bop_ev_.fill_line = e.line_addr;
      bop_ev_.fill_pf = e.is_prefetch;
    }
    write_line_tables(e.set, e.way, meta, uint32_t((e.way + 1) % config_.ways));
    if (e.is_write) {
      w.write_ctx[e.master].cache_done = true;
      w.write_ctx[e.master].cache_pending = false;
      w.dirty_line_count_r = update_dirty_line_count(r.dirty_line_count_r, e.victim_dirty, true);
      release_mshr(commit_slot, false);
    } else {
      w.dirty_line_count_r = update_dirty_line_count(r.dirty_line_count_r, e.victim_dirty, false);
      e.refill_committed = true;
    }
    return;
  }

  if (e.is_prefetch) {
    release_mshr(commit_slot, true);
    return;
  }
  if (!respond()) return;
  try_schedule_prefetch(e);
  release_mshr(commit_slot, true);
}

// ---------------------------------------------------------------------------------------------
// Request acceptance and the cycle function
// ---------------------------------------------------------------------------------------------

// pick_new_read_master already excluded every collision (pending writes, victims, same-line MSHRs, busy lookup).
void AXI_LLC::accept_new_requests() {
  const int master = pick_new_read_master(io.reg_write);
  if (master < 0) return;
  const auto &req = io.ext_in.upstream.read_req[master];
  io.ext_out.upstream.read_req[master].ready = true;
  io.reg_write.rr_read_master_r = uint8_t((master + 1) % NUM_READ_MASTERS);
  if (!llc_bop().on) for (auto &p : io.reg_write.prefetch_q) p = {};  // a demand request invalidates the pending stream
  start_lookup(req.addr, req.total_size, uint8_t(master), req.id, false, false, false, req.bypass);
}

void AXI_LLC::comb_full() {
  io.ext_out = {};
  io.table_out.clear();
  if (!rw_is_regs_) copy_regs_live(io.reg_write, io.regs);


  io.reg_write.enable_r = true;
  if (io.regs.state == AXI_LLCState::kDisabled) io.reg_write.state = AXI_LLCState::kIdle;

  // the prefetcher waits for a quiet cache: count consecutive idle cycles
  bool read_resp_pending = false;
  for (uint8_t i = 0; i < NUM_READ_MASTERS; ++i)
    read_resp_pending |= io.regs.read_resp_valid_r[i] || io.regs.read_resp_q_count_r[i] != 0;
  const bool quiet = io.ext_in.mem.prefetch_allow && !io.regs.lookup_valid_r &&
                     count_free_mshrs(io.regs) == mshr_limit() && !has_demand_mshr(io.regs) && !read_resp_pending;
  io.reg_write.prefetch_quiet_cycles_r =
      quiet ? uint8_t(std::min<uint32_t>(uint32_t(io.regs.prefetch_quiet_cycles_r) + 1u, 0xFFu)) : 0;

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

  // invalidate-all needs a quiet cache and no table / memory traffic this cycle
  if (io.ext_in.mem.invalidate_all && can_accept_invalidate_all_now(io.reg_write) && !io.table_out.data.enable &&
      !io.table_out.meta.enable && !io.table_out.repl.enable && !io.ext_out.mem.read_req_valid &&
      !io.ext_out.mem.write_req_valid) {
    io.ext_out.mem.invalidate_all_accepted = true;
    io.table_out.invalidate_all = true;
    io.reg_write.invalidate_epoch_r = uint8_t(io.regs.invalidate_epoch_r + 1);
    io.reg_write.dirty_line_count_r = 0;
    if (io.regs.lookup_valid_r) retry_lookup(AXI_LLCState::kLookup);
  }
}


void copy_regs_live(AXI_LLC_Regs_t &dst, const AXI_LLC_Regs_t &src) {
  static_assert(std::is_trivially_copyable<AXI_LLC_Regs_t>::value,
                "copy_regs_live needs a trivially copyable Regs");
  static_assert(std::is_trivially_copyable<AXI_LLC_ExtIn_t>::value, "the comb memo compares ExtIn bytewise");
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

namespace {

bool same_bytes(const AXI_LLC_Bytes_t &a, const AXI_LLC_Bytes_t &b) { return a.bytes == b.bytes; }

bool same_lookup(const AXI_LLC_LookupIn_t &a, const AXI_LLC_LookupIn_t &b) {
  return a.data_valid == b.data_valid && a.meta_valid == b.meta_valid && a.valid_valid == b.valid_valid &&
         a.repl_valid == b.repl_valid && same_bytes(a.data, b.data) && same_bytes(a.meta, b.meta) &&
         same_bytes(a.valid, b.valid) && same_bytes(a.repl, b.repl);
}

// No bulk payload slot is live (the scalar occupancy fields and the per-slot valid flags all say empty).
bool no_live_payload(const AXI_LLC_Regs_t &r) {
  for (int m = 0; m < NUM_READ_MASTERS; ++m)
    if (r.read_resp_valid_r[m] || r.read_resp_q_count_r[m] != 0) return false;
  if (r.read_victim_wb_q_count_r != 0 || r.victim_wb_valid_r) return false;
  for (int m = 0; m < NUM_WRITE_MASTERS; ++m) {
    if (r.write_q_count_r[m] != 0 || r.write_ctx[m].valid) return false;
    for (int i = 0; i < MAX_WRITE_OUTSTANDING; ++i)
      if (r.write_q[m][i].valid) return false;
  }
  for (int i = 0; i < AXI_LLC_READ_VICTIM_WB_QUEUE_DEPTH; ++i)
    if (r.read_victim_wb_q[i].valid) return false;
  for (uint32_t i = 0; i < MAX_OUTSTANDING; ++i)
    if (r.mshr[i].valid) return false;
  return true;
}

bool no_write_resp(const AXI_LLC_Regs_t &r) {
  for (int m = 0; m < NUM_WRITE_MASTERS; ++m)
    if (r.write_resp_valid_r[m]) return false;
  return true;
}

} // namespace

void AXI_LLC::memo_drop() {
  for (auto &m : memo_) m.valid = false;
  rw_is_regs_ = false;
  quiet_ = false;
}

bool AXI_LLC::at_idle_fixed_point() const {
  constexpr size_t kBulk = __builtin_offsetof(AXI_LLC_Regs_t, read_resp_data_r);
  return std::memcmp(&io.reg_write, &io.regs, kBulk) == 0 && no_live_payload(io.regs) && no_live_payload(io.reg_write);
}

bool AXI_LLC::memo_matches(const CombMemo &m) const {
  return std::memcmp(&m.ext_in, &io.ext_in, sizeof(io.ext_in)) == 0 && same_lookup(m.lookup_in, io.lookup_in);
}

void AXI_LLC::memo_record() {
  CombMemo &m = memo_[memo_next_];
  memo_next_ = (memo_next_ + 1) % kMemoSlots;
  std::memcpy(&m.ext_in, &io.ext_in, sizeof(io.ext_in));
  m.lookup_in = io.lookup_in;
  m.ext_out = io.ext_out;
  m.table_clear = !(io.table_out.data.enable || io.table_out.meta.enable || io.table_out.valid.enable ||
                    io.table_out.repl.enable || io.table_out.invalidate_all);
  if (!m.table_clear) m.table_out = io.table_out;
  m.valid = true;
}

void AXI_LLC::comb() {
  bop_ev_ = {};
  for (const auto &m : memo_) {
    if (m.valid && memo_matches(m)) {
      io.ext_out = m.ext_out;
      if (m.table_clear) io.table_out.clear();
      else io.table_out = m.table_out;
      if (!rw_is_regs_) {
        copy_regs_live(io.reg_write, io.regs);
        rw_is_regs_ = true;
      }
      return;
    }
  }
  comb_full();
  rw_is_regs_ = at_idle_fixed_point();
  quiet_ = rw_is_regs_ && !io.regs.lookup_valid_r && no_write_resp(io.regs);
  if (rw_is_regs_) memo_record();
}

void AXI_LLC::bop_seq() {
  Bop *b = llc_bop().bop;
  const uint32_t lb = config_.line_bytes;
  if (bop_ev_.launched && !bop_q_.empty()) bop_q_.erase(bop_q_.begin());
  b->stats_sent += bop_ev_.sent;
  b->stats_useful += bop_ev_.trig && bop_ev_.trig_pf;
  if (bop_ev_.fill) b->fill(bop_ev_.fill_line / lb, bop_ev_.fill_pf);
  if (bop_ev_.trig) {
    uint32_t out[8];
    const int n = b->trigger(bop_ev_.trig_line / lb, out, 8);
    for (int i = 0; i < n; i++) {
      const uint32_t a = out[i] * lb;
      if (std::find(bop_q_.begin(), bop_q_.end(), a) != bop_q_.end()) continue;
      if (bop_q_.size() >= AXI_LLC_MAX_PREFETCH_QUEUE) bop_q_.erase(bop_q_.begin());
      bop_q_.push_back(a);
    }
  }
  bop_ev_ = {};
  if (!bop_q_.empty()) memo_drop();   // the launch depends on the queue, which the memoised comb does not see
}

void AXI_LLC::seq() {
  if (llc_bop().on) bop_seq();
  if (rw_is_regs_) return;
  copy_regs_live(io.regs, io.reg_write);
  rw_is_regs_ = true;
  quiet_ = false;
  for (auto &m : memo_) m.valid = false;
}

} // namespace axi_interconnect

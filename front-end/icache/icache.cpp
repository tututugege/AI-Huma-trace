#include "trace_hook.h"
#include "../front_IO.h"
#include "PtwMemPort.h"
#include "include/ICacheTop.h"
#include "include/icache_module.h"
#include "GenericTable.h"
#include "PhysMemory.h"
#include <cassert>

#define CONFIG_ICACHE_FOCUS_VADDR_BEGIN 0u

#define CONFIG_ICACHE_FOCUS_VADDR_END 0u

// icache.cpp is the front-end wrapper/orchestrator:
// - own the external tables used by icache_module
// - bind the selected ICacheTop glue object into front_top
// - drive per-cycle seq/perf handoff after the combinational glue is evaluated

// Define global ICache instance
icache_module_n::ICache icache;
PtwMemPort *icache_ptw_mem_port = nullptr;
PtwWalkPort *icache_ptw_walk_port = nullptr;
ICacheMemPortReq *icache_mem_req_port = nullptr;
ICacheMemPortResp *icache_mem_resp_port = nullptr;
axi_interconnect::ReadMasterPort_t *icache_mem_read_port = nullptr;
static SimContext *icache_ctx = nullptr;

extern void (*g_icache_fill_hook)(uint32_t index, uint32_t tag);   // FE_IPF: demand fills are reported to the prefetcher (defined in ICacheTop.cpp)
namespace {
using LookupTimingPolicy =
    RegfileTablePolicy;

constexpr int kSetAddrBits = ICACHE_INDEX_BITS;
constexpr int kRows = ICACHE_SET_NUM;
constexpr int kTagChunks = ICACHE_WAY_NUM;
constexpr int kTagChunkBits = ICACHE_TAG_BITS;
constexpr int kValidChunks = ICACHE_WAY_NUM;
constexpr int kValidChunkBits = 1;

using TagTable = GenericTable<kRows, kTagChunks, kTagChunkBits, LookupTimingPolicy>;
using ValidTable = GenericTable<kRows, kValidChunks, kValidChunkBits, LookupTimingPolicy>;

GenericTableTimingConfig make_lookup_timing_config() {
  GenericTableTimingConfig cfg;
  cfg.fixed_latency = ICACHE_LOOKUP_LATENCY > 0 ? ICACHE_LOOKUP_LATENCY : 1;
  cfg.random_delay = false;
  cfg.random_min = 1;
  cfg.random_max = 1;
  return cfg;
}


TagTable &g_tag_table() { static TagTable t(make_lookup_timing_config()); return t; }
ValidTable &g_valid_table() { static ValidTable t(make_lookup_timing_config()); return t; }

void bind_icache_runtime(ICacheTop *instance) {
  static PtwMemPort *bound_mem_port = nullptr;
  static PtwWalkPort *bound_walk_port = nullptr;
  static ICacheMemPortReq *bound_mem_req_port = nullptr;
  static ICacheMemPortResp *bound_mem_resp_port = nullptr;
  static axi_interconnect::ReadMasterPort_t *bound_read_port = nullptr;
  static SimContext *bound_ctx = nullptr;

  if (bound_mem_port != icache_ptw_mem_port) {
    instance->set_ptw_mem_port(icache_ptw_mem_port);
    bound_mem_port = icache_ptw_mem_port;
  }
  if (bound_walk_port != icache_ptw_walk_port) {
    instance->set_ptw_walk_port(icache_ptw_walk_port);
    bound_walk_port = icache_ptw_walk_port;
  }
  if (bound_read_port != icache_mem_read_port) {
    instance->set_mem_read_port(icache_mem_read_port);
    bound_read_port = icache_mem_read_port;
  }
  if (bound_mem_req_port != icache_mem_req_port ||
      bound_mem_resp_port != icache_mem_resp_port) {
    instance->set_mem_probe_ports(icache_mem_req_port, icache_mem_resp_port);
    bound_mem_req_port = icache_mem_req_port;
    bound_mem_resp_port = icache_mem_resp_port;
  }
  if (bound_ctx != icache_ctx) {
    instance->setContext(icache_ctx);
    bound_ctx = icache_ctx;
  }
}


struct ICacheLookupTableResp {
  bool meta_resp_valid = false;
  bool data_snapshot_valid = false;
  uint32_t lookup_index = 0;
  uint32_t set_tag_snapshot[ICACHE_WAY_NUM] = {0};
  bool set_valid_snapshot[ICACHE_WAY_NUM] = {false};
};

ICacheLookupTableResp g_lookup_table_resp;

void cache_lookup_table_resp(const TagTable::ReadResp &tag_resp,
                             const ValidTable::ReadResp &valid_resp,
                             uint32_t lookup_index) {
  g_lookup_table_resp = {};
  g_lookup_table_resp.meta_resp_valid = tag_resp.valid && valid_resp.valid;
  g_lookup_table_resp.data_snapshot_valid = tag_resp.valid;
  g_lookup_table_resp.lookup_index = lookup_index;
  for (uint32_t way = 0; way < ICACHE_WAY_NUM; ++way) {
    g_lookup_table_resp.set_tag_snapshot[way] = tag_resp.payload.chunks[way][0];
    g_lookup_table_resp.set_valid_snapshot[way] = valid_resp.payload.chunks[way][0];
  }
}

void update_icache_perf_counters(const struct icache_in *in,
                                 const struct icache_out *out) {
  if (icache_ctx == nullptr || in == nullptr || out == nullptr) {
    return;
  }

  if (!in->refetch && out->perf_req_fire) {

  }
  if (icache.perf.miss_issue_valid) {

  }
  if (icache.perf.miss_penalty_valid) {



  }
  if (icache.perf.axi_read_valid) {



  }
}
} // namespace

void icache_fill_lookup_meta_input(icache_module_n::ICache_lookup_in_t &dst) {
  dst = {};
  dst.meta_resp_valid = g_lookup_table_resp.meta_resp_valid;
  if (!g_lookup_table_resp.meta_resp_valid) {
    return;
  }

  for (uint32_t way = 0; way < ICACHE_WAY_NUM; ++way) {
    dst.lookup_set_tag[way] = g_lookup_table_resp.set_tag_snapshot[way];
    dst.lookup_set_valid[way] = g_lookup_table_resp.set_valid_snapshot[way];
  }
}

void icache_fill_lookup_data_input(icache_module_n::ICache_lookup_in_t &dst,
                                   bool req_valid, uint32_t req_index,
                                   uint32_t req_way) {
  dst.data_resp_valid = false;
  dst.data_resp_way = 0;
  if (!req_valid || !g_lookup_table_resp.data_snapshot_valid ||
      g_lookup_table_resp.lookup_index != req_index ||
      req_way >= ICACHE_WAY_NUM) {
    return;
  }

  dst.data_resp_valid = true;
  dst.data_resp_way = static_cast<uint8_t>(req_way);
}

void icache_seq_read(struct icache_in *in, struct icache_out *out) {
  assert(in != nullptr);
  assert(out != nullptr);
  (void)in;
  (void)out;
}

void icache_peek_ready(struct icache_in *in, struct icache_out *out) {
  assert(in != nullptr);
  assert(out != nullptr);
  if (!in->reset) {
    assert(in->csr_status != nullptr &&
           "icache_peek_ready requires csr_status when not in reset");
  }
  ICacheTop *instance = get_icache_instance();
  bind_icache_runtime(instance);
  instance->setIO(in, out);
  instance->peek_ready();
}

void icache_comb_calc(struct icache_in *in, struct icache_out *out) {
  assert(in != nullptr);
  assert(out != nullptr);
  if (!in->reset) {
    assert(in->csr_status != nullptr &&
           "icache_comb_calc requires csr_status when not in reset");
  }
  TagTable &tag_table = g_tag_table();
  ValidTable &valid_table = g_valid_table();
  if (in->reset) {
    tag_table.reset();
    valid_table.reset();
  }
  ICacheTop *instance = get_icache_instance();
  bind_icache_runtime(instance);
  instance->setIO(in, out);

  pc_t lookup_pc = in->fetch_address;
  if (icache.io.regs.req_valid_r) {
    lookup_pc = icache.io.regs.req_pc_r;
  } else if (icache.io.regs.lookup_pending_r) {
    lookup_pc = icache.io.regs.lookup_pc_r;
  }
  uint32_t lookup_index =
      (lookup_pc >> ICACHE_OFFSET_BITS) &
      (ICACHE_SET_NUM - 1u);

  TagTable::ReadReq tag_read{};
  ValidTable::ReadReq valid_read{};
  tag_read.enable = !in->reset && !in->refetch &&
                    (ICACHE_LOOKUP_LATENCY == 0
                         ? (in->icache_read_valid || icache.io.regs.req_valid_r ||
                            icache.io.regs.lookup_pending_r)
                         : icache.io.regs.lookup_pending_r);
  valid_read.enable = tag_read.enable;
  tag_read.address = lookup_index;
  valid_read.address = lookup_index;

  TagTable::ReadResp tag_resp{};
  ValidTable::ReadResp valid_resp{};
  tag_table.comb(tag_read, tag_resp);
  valid_table.comb(valid_read, valid_resp);
  cache_lookup_table_resp(tag_resp, valid_resp, lookup_index);
  instance->comb();
  update_icache_perf_counters(in, out);
  instance->seq();
  tag_read.enable = icache.io.regs.lookup_pending_r;
  valid_read.enable = tag_read.enable;
  tag_read.address = icache.io.regs.lookup_index_r;
  valid_read.address = icache.io.regs.lookup_index_r;

  TagTable::WriteReq tag_write{};
  ValidTable::WriteReq valid_write{};
  if (icache.io.table_write.we) {
    if (g_icache_fill_hook && icache.io.table_write.valid) g_icache_fill_hook(icache.io.table_write.index, icache.io.table_write.tag);
    tag_write.enable = true;
    valid_write.enable = true;
    tag_write.address = icache.io.table_write.index;
    valid_write.address = icache.io.table_write.index;
    tag_write.payload.chunks[icache.io.table_write.way][0] =
        icache.io.table_write.tag;
    tag_write.chunk_enable[icache.io.table_write.way] = true;
    valid_write.payload.chunks[icache.io.table_write.way][0] =
        icache.io.table_write.valid;
    valid_write.chunk_enable[icache.io.table_write.way] = true;
  }
  if (icache.io.table_write.invalidate_all) {
    valid_table.reset();
  }
  tag_table.seq(tag_read, tag_write);
  valid_table.seq(valid_read, valid_write);
}

void icache_seq_write() {}
bool icache_peek_hit(uint32_t index, uint32_t ptag);

// FE_IPF: install a prefetched line (tag + valid only; the I-cache keeps no data array). An invalid way is used first, else a rotating victim.
bool icache_prefetch_install(uint32_t index, uint32_t ptag) {
  if (icache_peek_hit(index, ptag)) return false;
  static uint8_t rot[ICACHE_SET_NUM] = {};
  TagTable &tt = g_tag_table();
  ValidTable &vt = g_valid_table();
  const auto &vp = vt.peek_row(index);
  int way = -1;
  for (int w = 0; w < ICACHE_WAY_NUM; ++w) if (!vp.chunks[w][0]) { way = w; break; }
  if (way < 0) way = rot[index]++ % ICACHE_WAY_NUM;
  TagTable::WriteReq tw{};
  ValidTable::WriteReq vw{};
  tw.enable = vw.enable = true;
  tw.address = vw.address = index;
  tw.payload.chunks[way][0] = ptag;
  vw.payload.chunks[way][0] = 1;
  tw.chunk_enable[way] = vw.chunk_enable[way] = true;
  tt.seq(TagTable::ReadReq{}, tw);
  vt.seq(ValidTable::ReadReq{}, vw);
  return true;
}

// FE_PAIR: functional tag peek (no state change): is the line with this physical tag resident in set `index`?
bool icache_peek_hit(uint32_t index, uint32_t ptag) {
  const auto &tp = g_tag_table().peek_row(index);
  const auto &vp = g_valid_table().peek_row(index);
  for (int way = 0; way < ICACHE_WAY_NUM; ++way)
    if (vp.chunks[way][0] && tp.chunks[way][0] == ptag) return true;
  return false;
}

// Fast-warmup replay (EXP-011): load a ready-made L1I tag/valid state.
void icache_warm_install(const uint32_t tag[ICACHE_SET_NUM][ICACHE_WAY_NUM], const bool valid[ICACHE_SET_NUM][ICACHE_WAY_NUM]) {
  TagTable &tt = g_tag_table();
  ValidTable &vt = g_valid_table();
  for (int set = 0; set < ICACHE_SET_NUM; ++set) {
    TagTable::WriteReq tw{};
    ValidTable::WriteReq vw{};
    tw.enable = vw.enable = true;
    tw.address = vw.address = set;
    for (int way = 0; way < ICACHE_WAY_NUM; ++way) {
      tw.payload.chunks[way][0] = tag[set][way];
      vw.payload.chunks[way][0] = valid[set][way];
      tw.chunk_enable[way] = vw.chunk_enable[way] = true;
    }
    tt.seq(TagTable::ReadReq{}, tw);
    vt.seq(ValidTable::ReadReq{}, vw);
  }
}

void icache_top(struct icache_in *in, struct icache_out *out) {
  icache_seq_read(in, out);
  icache_comb_calc(in, out);
  icache_seq_write();
}

void icache_set_context(SimContext *ctx) {
  icache_ctx = ctx;
}

void icache_set_ptw_mem_port(PtwMemPort *port) {
  ICacheTop *instance = get_icache_instance();
  instance->set_ptw_mem_port(port);
}

void icache_set_ptw_walk_port(PtwWalkPort *port) {
  ICacheTop *instance = get_icache_instance();
  instance->set_ptw_walk_port(port);
}

void icache_set_mem_read_port(axi_interconnect::ReadMasterPort_t *port) {
  ICacheTop *instance = get_icache_instance();
  instance->set_mem_read_port(port);
}

void icache_set_mem_probe_ports(ICacheMemPortReq *req_port,
                                ICacheMemPortResp *resp_port) {
  ICacheTop *instance = get_icache_instance();
  instance->set_mem_probe_ports(req_port, resp_port);
}

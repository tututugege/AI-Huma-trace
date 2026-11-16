#include "MemSubsystem.h"
#include "PhysMemory.h"
#include "config.h"
#include "icache/GenericTable.h"
#include <assert.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <memory>

#include "AXI_Interconnect.h"


namespace {

struct AxiLlcTableRuntime {
  DynamicSramTable meta;
  DynamicSramTable valid;
  DynamicSramTable repl;
  axi_interconnect::AXI_LLC_LookupIn_t lookup_in{};
  axi_interconnect::AXI_LLCConfig config{};
  bool enabled = false;
  bool lookup_pending_valid = false;
  uint32_t lookup_pending_index = 0;
  uint32_t lookup_delay_left = 0;
  bool lookup_queued_valid = false;
  uint32_t lookup_queued_index = 0;

  static DynamicTableConfig make_table_config(uint32_t rows, uint32_t row_bytes,
                                              uint32_t latency) {
    DynamicTableConfig cfg;
    cfg.rows = rows;
    cfg.chunks = row_bytes;
    cfg.chunk_bits = 8;
    cfg.latency = latency == 0 ? 1 : latency;
    return cfg;
  }

  static DynamicTableWriteReq make_write_req(
      const axi_interconnect::AXI_LLC_TableReq_t &req, uint32_t row_bytes,
      uint32_t unit_bytes) {
    DynamicTableWriteReq write_req;
    write_req.enable = req.enable && req.write;
    write_req.address = req.index;
    if (!write_req.enable) {
      return write_req;  // write_chunks() ignores a disabled request: no payload needed
    }
    write_req.payload.reset(row_bytes);
    write_req.chunk_enable.assign(row_bytes, 0);
    const size_t base =
        unit_bytes == 0 ? 0 : static_cast<size_t>(req.way) * unit_bytes;
    const size_t copy_bytes =
        std::min(req.payload.size(), row_bytes > base ? row_bytes - base : 0u);
    if (copy_bytes == 0) {
      return write_req;
    }
    std::memcpy(write_req.payload.data() + base, req.payload.data(), copy_bytes);
    const size_t en_bytes = std::min(req.byte_enable.size(), copy_bytes);
    for (size_t i = 0; i < en_bytes; ++i) {
      write_req.chunk_enable[base + i] = req.byte_enable[i];
    }
    return write_req;
  }

  void configure(const axi_interconnect::AXI_LLCConfig &cfg) {
    config = cfg;
    enabled = cfg.enable && cfg.valid();
    lookup_in = {};
    lookup_pending_valid = false;
    lookup_pending_index = 0;
    lookup_delay_left = 0;
    lookup_queued_valid = false;
    lookup_queued_index = 0;
    if (!enabled) {
      return;
    }
    const uint32_t sets = cfg.set_count();
    // No line-data storage (P3): the data of a valid way is the functional memory (pmem) at the
    // way's address, which the commit-time store path keeps current.
    meta.configure(make_table_config(
        sets, cfg.ways * axi_interconnect::AXI_LLC_META_ENTRY_BYTES,
        cfg.lookup_latency));
    valid.configure(make_table_config(
        sets, axi_interconnect::AXI_LLC::valid_row_bytes(cfg),
        cfg.lookup_latency));
    repl.configure(make_table_config(sets, axi_interconnect::AXI_LLC_REPL_BYTES,
                                     cfg.lookup_latency));
    meta.reset();
    valid.reset();
    repl.reset();
  }

  void comb_outputs() {
    lookup_in = {};
    if (!enabled) {
      return;
    }
    if (!lookup_pending_valid || lookup_delay_left != 0) {
      return;
    }

    DynamicTablePayload meta_payload;
    DynamicTablePayload valid_payload;
    DynamicTablePayload repl_payload;
    lookup_in.meta_valid =
        meta.debug_read_row(lookup_pending_index, meta_payload);
    lookup_in.valid_valid =
        valid.debug_read_row(lookup_pending_index, valid_payload);
    lookup_in.repl_valid =
        repl.debug_read_row(lookup_pending_index, repl_payload);
    lookup_in.meta.bytes = meta_payload.bytes;
    lookup_in.valid.bytes = valid_payload.bytes;
    lookup_in.repl.bytes = repl_payload.bytes;
    // The data row carries no values (zero-filled).
    lookup_in.data_valid = lookup_in.meta_valid;
  }

  void seq(const axi_interconnect::AXI_LLC_TableOut_t &table_out) {
    if (!enabled) {
      return;
    }
    if (table_out.invalidate_all) {
      meta.reset();
      valid.reset();
      repl.reset();
      lookup_pending_valid = false;
      lookup_pending_index = 0;
      lookup_delay_left = 0;
      lookup_queued_valid = false;
      lookup_queued_index = 0;
      return;
    }
    const auto meta_write = make_write_req(
        table_out.meta,
        config.ways * axi_interconnect::AXI_LLC_META_ENTRY_BYTES,
        axi_interconnect::AXI_LLC_META_ENTRY_BYTES);
    const auto valid_write = make_write_req(
        table_out.valid, axi_interconnect::AXI_LLC::valid_row_bytes(config), 0);
    const auto repl_write =
        make_write_req(table_out.repl, axi_interconnect::AXI_LLC_REPL_BYTES, 0);

    meta.seq({}, meta_write);
    valid.seq({}, valid_write);
    repl.seq({}, repl_write);

    const bool data_read_en = table_out.data.enable && !table_out.data.write;
    const bool meta_read_en = table_out.meta.enable && !table_out.meta.write;
    const bool valid_read_en = table_out.valid.enable && !table_out.valid.write;
    const bool repl_read_en = table_out.repl.enable && !table_out.repl.write;
    const bool any_read_en =
        data_read_en || meta_read_en || valid_read_en || repl_read_en;
    if (any_read_en) {
      assert(data_read_en && meta_read_en && valid_read_en && repl_read_en);
      assert(table_out.data.index == table_out.meta.index);
      assert(table_out.data.index == table_out.valid.index);
      assert(table_out.data.index == table_out.repl.index);
    }

    if (lookup_pending_valid) {
      if (lookup_delay_left > 0) {
        lookup_delay_left--;
      } else {
        lookup_pending_valid = false;
        lookup_pending_index = 0;
      }
    }

    if (any_read_en) {
      const uint32_t req_index = table_out.data.index;
      if (!lookup_pending_valid && !lookup_queued_valid) {
        lookup_pending_valid = true;
        lookup_pending_index = req_index;
        lookup_delay_left = config.lookup_latency == 0 ? 0 : (config.lookup_latency - 1);
      } else if (!lookup_pending_valid) {
        lookup_pending_valid = true;
        lookup_pending_index = lookup_queued_index;
        lookup_delay_left = config.lookup_latency == 0 ? 0 : (config.lookup_latency - 1);
        lookup_queued_valid = false;
        lookup_queued_index = 0;
        lookup_queued_valid = true;
        lookup_queued_index = req_index;
      } else if (lookup_pending_index != req_index) {
        lookup_queued_valid = true;
        lookup_queued_index = req_index;
      }
    } else if (!lookup_pending_valid && lookup_queued_valid) {
      lookup_pending_valid = true;
      lookup_pending_index = lookup_queued_index;
      lookup_delay_left = config.lookup_latency == 0 ? 0 : (config.lookup_latency - 1);
      lookup_queued_valid = false;
      lookup_queued_index = 0;
    }
  }
};

} // namespace

struct AxiKitRuntime {
  AxiLlcTableRuntime llc_tables;
};

class MemSubsystemPtwWalkPortAdapter : public PtwWalkPort {
public:
  bool send_walk_req(const PtwWalkReq &req) override {
    if (!comb_out_.req_ready || comb_in_.req_valid) {
      return false;
    }
    comb_in_.req_valid = true;
    comb_in_.req = req;
    comb_out_.req_ready = false;
    comb_out_.resp_valid = false;
    comb_out_.resp = {};
    return true;
  }

  bool resp_valid() const override { return comb_out_.resp_valid; }

  PtwWalkResp resp() const override { return comb_out_.resp; }

  PtwWalkPortCombOut comb_output() const override { return comb_out_; }

  void consume_resp() override {
    comb_in_.resp_consumed = true;
    comb_out_.resp_valid = false;
    comb_out_.resp = {};
  }

  void flush_client() override {
    seq_in_.flush = true;
    comb_out_.req_ready = true;
    comb_out_.resp_valid = false;
    comb_out_.resp = {};
  }

  const PtwWalkPortCombIn &comb_input() const { return comb_in_; }

  const PtwWalkPortSeqIn &seq_input() const { return seq_in_; }

  void set_comb_output(const PtwWalkPortCombOut &out) { comb_out_ = out; }

  void reset_cycle_input() { comb_in_ = {}; }

  void reset_seq_input() { seq_in_ = {}; }

private:
  PtwWalkPortCombIn comb_in_{};
  PtwWalkPortSeqIn seq_in_{};
  PtwWalkPortCombOut comb_out_{};
};

void MemSubsystem::sync_ptw_port_outputs() {
  const auto &ptw_out = ptw_block.comb_outputs();
  if (dtlb_walk_port_inst != nullptr) {
    PtwWalkPortCombOut out{};
    const auto &src =
        ptw_out.walk_clients[static_cast<size_t>(PtwClient::DTLB)];
    out.req_ready = src.req_ready;
    out.resp_valid = src.resp_valid;
    out.resp = src.resp;
    dtlb_walk_port_inst->set_comb_output(out);
  }
  if (itlb_walk_port_inst != nullptr) {
    PtwWalkPortCombOut out{};
    const auto &src =
        ptw_out.walk_clients[static_cast<size_t>(PtwClient::ITLB)];
    out.req_ready = src.req_ready;
    out.resp_valid = src.resp_valid;
    out.resp = src.resp;
    itlb_walk_port_inst->set_comb_output(out);
  }
}




void MemSubsystem::set_llc_config(const axi_interconnect::AXI_LLCConfig &cfg) {
  axi_kit_runtime->llc_tables.configure(cfg);
}

void MemSubsystem::llc_warm_set(uint32_t set, const uint32_t *tag, const uint8_t *valid, const uint8_t *dirty,
                                uint32_t repl_next) {
  if (!axi_kit_runtime->llc_tables.enabled) return;
  auto &t = axi_kit_runtime->llc_tables;
  const uint32_t ways = t.config.ways;
  DynamicTableWriteReq m, v, r;
  m.enable = v.enable = r.enable = true;
  m.address = v.address = r.address = set;
  m.payload.reset(ways * axi_interconnect::AXI_LLC_META_ENTRY_BYTES);
  m.chunk_enable.assign(m.payload.size(), 1);
  v.payload.reset(axi_interconnect::AXI_LLC::valid_row_bytes(t.config));
  v.chunk_enable.assign(v.payload.size(), 1);
  r.payload.reset(axi_interconnect::AXI_LLC_REPL_BYTES);
  r.chunk_enable.assign(r.payload.size(), 1);
  for (uint32_t w = 0; w < ways; ++w) {
    axi_interconnect::AXI_LLCMetaEntry_t e{};
    e.tag = tag[w];
    e.flags = uint8_t((valid[w] ? axi_interconnect::AXI_LLC_META_VALID : 0) |
                      (dirty[w] ? axi_interconnect::AXI_LLC_META_DIRTY : 0));
    axi_interconnect::AXI_LLC_Bytes_t b;
    axi_interconnect::AXI_LLC::encode_meta(e, b);
    std::memcpy(m.payload.data() + w * axi_interconnect::AXI_LLC_META_ENTRY_BYTES, b.data(),
                axi_interconnect::AXI_LLC_META_ENTRY_BYTES);
    v.payload.data()[w] = valid[w] ? 1 : 0;
  }
  for (int i = 0; i < 4; ++i) r.payload.data()[i] = uint8_t(repl_next >> (8 * i));
  t.meta.seq({}, m);
  t.valid.seq({}, v);
  t.repl.seq({}, r);
}

void MemSubsystem::llc_comb_outputs() {
  axi_kit_runtime->llc_tables.comb_outputs();
}

const axi_interconnect::AXI_LLC_LookupIn_t &MemSubsystem::llc_lookup_in() const {
  return axi_kit_runtime->llc_tables.lookup_in;
}

void MemSubsystem::llc_seq(const axi_interconnect::AXI_LLC_TableOut_t &table_out) {
  axi_kit_runtime->llc_tables.seq(table_out);
}


MemSubsystem::MemSubsystem(SimContext *) {
  axi_kit_runtime = std::make_unique<AxiKitRuntime>();
  dtlb_walk_port_inst = std::make_unique<MemSubsystemPtwWalkPortAdapter>();
  itlb_walk_port_inst = std::make_unique<MemSubsystemPtwWalkPortAdapter>();
  dtlb_walk_port = dtlb_walk_port_inst.get();
  itlb_walk_port = itlb_walk_port_inst.get();
}

MemSubsystem::~MemSubsystem() = default;

void MemSubsystem::init() {
  Assert(peripheral_req != nullptr && "MemSubsystem: peripheral_req is not connected");
  Assert(peripheral_resp != nullptr && "MemSubsystem: peripheral_resp is not connected");
  Assert(csr_interrupt_inject != nullptr &&
         "MemSubsystem: csr interrupt inject IO is not connected");

  // Route LSU/PTW/ICache requests through MemRouteBlock and capture raw
  // DCache responses before routing them back to their owners.
  dcache_.in.lsu2dcache = &dcache_req_mux_;
  dcache_.out.dcache2lsu = &dcache_resp_raw_;

  // Internal MSHR ↔ DCache wires: RealDcache reads/writes MSHR IO structs
  // directly via pointers, keeping the connection zero-copy.
  dcache_.in.mshr2dcache = &mshr_dcache_io_;  // MSHR output → DCache input
  dcache_.out.dcache2mshr = &dcache_mshr_io_; // DCache output → MSHR input
  mshr_.in.dcache2mshr = &dcache_mshr_io_;    // DCache output → MSHR comb input
  mshr_.in.s1_q = &dcache_s1_q_;
  mshr_.out.mshr2dcache = &mshr_dcache_io_;   // MSHR output → DCache input

  Assert(dcache_axi_r && dcache_axi_w && extra_axi_r && extra_axi_w && "MemSubsystem: AXI master ports not bound");
  mshr_.in.axi = dcache_axi_r;       // MSHR <-> the DCACHE_R master port
  mshr_.out.axi = dcache_axi_r;
  wb_.in.axi = dcache_axi_w;         // write buffer <-> the DCACHE_W master port
  wb_.out.axi = dcache_axi_w;

  // Internal WriteBuffer ↔ DCache wires.
  dcache_.in.wb2dcache = &wb_dcache_io_;  // WB output → DCache input
  dcache_.out.dcache2wb = &dcache_wb_io_; // DCache output → WB input
  wb_.in.dcache2wb = &dcache_wb_io_;      // DCache output → WB comb input
  wb_.in.s1_q = &dcache_s1_q_;
  wb_.out.wb2dcache = &wb_dcache_io_;     // WB output → DCache inputs

  for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
    dcache_.in.dcachelinereadresp[i] = &dcache_line_read_resp_[i]; // DCache line read response → LSU
    dcache_.out.dcachereadreq[i] = &dcache_line_read_req_[i];      // DCache line read request ← LSU
    dcache_.out.lru_updates[i] = &lru_updates_[i];                 // DCache line write request ← LSU
    dcache_.out.pendingwrite[i] = &pending_writes_[i];             // DCache pending write flag ← LSU
  }

  dcache_.out.fill_write = &fill_writes_; // DCache fill write-back flag ← LSU
  dcache_.in.fill_store_dirty = &fill_store_dirty_;
  dcache_.in.fillin = &fill_in_;          // DCache fill input (from MSHR/WB)
  dcache_.out.fillout = &fill_out_;       // DCache fill output (to MSHR/WB)

  mem_route_block.out.dcache_req = &dcache_req_mux_;  // MemRouteBlock output → DCache request multiplexer
  mem_route_block.in.dcache_resp = &dcache_resp_route_; // DCache response → MemRouteBlock input

  mem_route_block.in.lsu_req = &lsu2dcache->req_ports;
  mem_route_block.out.lsu_resp = dcache2lsu;

  mem_route_block.in.icache_req = icache_req;
  mem_route_block.out.icache_resp = icache_resp;

  mem_route_block.in.ptw_walk_req = &ptw_walk_req; // PTW walk request → MemRouteBlock input

  mem_route_block.out.ptw_events = &ptw_events; // MemRouteBlock output → PTW event signals
  mem_route_block.out.ptw_grant = &ptw_grant;   // MemRouteBlock output → PTW grant signals
  mem_route_block.out.wakeup = &wakeup;         // MemRouteBlock output → LSU wakeup signals


  // ── Initialise sub-modules ─────────────────────────────────────────────────
  mshr_.init();
  wb_.init();
  dcache_.init();
  peripheral_model_.bind(csr_interrupt_inject);
  peripheral_axi_.in.req = peripheral_req;
  peripheral_axi_.out.resp = peripheral_resp;
  peripheral_axi_.peripheral_model = &peripheral_model_;
  peripheral_axi_.in.read = extra_axi_r;   // MMIO <-> the EXTRA_R / EXTRA_W master ports
  peripheral_axi_.out.read = extra_axi_r;
  peripheral_axi_.in.write = extra_axi_w;
  peripheral_axi_.out.write = extra_axi_w;
  peripheral_axi_.init();

  ptw_block.init();
  mem_route_block.init();
  sync_ptw_port_outputs();
  dtlb_walk_port_inst->reset_cycle_input();
  itlb_walk_port_inst->reset_cycle_input();
  dtlb_walk_port_inst->reset_seq_input();
  itlb_walk_port_inst->reset_seq_input();

  dcache_req_mux_ = {};
  dcache_resp_raw_ = {};
  dcache_resp_route_ = {};
  dcache_empty_req_ = LsuDcacheIO{};
  for (auto &meta : dcache_resp_meta_cur_) {
    meta = {};
  }
  for (auto &meta : dcache_resp_meta_nxt_) {
    meta = {};
  }

  mshr_dcache_io_ = {};
  dcache_mshr_io_ = {};
  wb_dcache_io_ = {};
  dcache_wb_io_ = {};

  memset(dcache_line_read_req_, 0, sizeof(dcache_line_read_req_));
  memset(dcache_line_read_resp_, 0, sizeof(dcache_line_read_resp_));
  for (auto &entry : lru_updates_) {
    entry = {};
  }
  for (auto &entry : pending_writes_) {
    entry = {};
  }
  fill_writes_ = {};
  fill_in_ = {};
  fill_out_ = {};
  dcache_s1_q_ = {};


  {
    axi_interconnect::AXI_LLCConfig llc_cfg;
    llc_cfg.enable = (CONFIG_AXI_LLC_ENABLE != 0);
    llc_cfg.size_bytes = CONFIG_AXI_LLC_SIZE_BYTES;
    llc_cfg.ways = CONFIG_AXI_LLC_WAYS;
    llc_cfg.mshr_num = CONFIG_AXI_LLC_MSHR_NUM;
    llc_cfg.lookup_latency = CONFIG_AXI_LLC_LOOKUP_LATENCY;
    set_llc_config(llc_cfg);
  }
}

void MemSubsystem::on_commit_store(uint32_t paddr, uint64_t data, uint8_t func3) {
  peripheral_model_.on_commit_store(paddr, data, func3);
}

void MemSubsystem::configure_interrupt_latency(uint64_t seed,
                                               uint32_t min_cycles,
                                               uint32_t max_cycles) {
  peripheral_model_.configure_interrupt_latency(seed, min_cycles, max_cycles);
}


void MemSubsystem::sync_mmio_devices_from_backing() {
  peripheral_model_.sync_from_backing();
}

static bool replay_waits_for_mshr_fill(ReplayType replay) {
  return replay == ReplayType::MSHR_HIT || replay == ReplayType::MSHR_FULL;
}


// DCache stage 2 of the request that stage 1 issued last cycle: the tag / fill array read, the stage-2 logic, the
// array writes, the response to the route block.
void MemSubsystem::dcache_stage2() {
  Dcache_Read(dcache_s1_q_.line_read_req,
              dcache_line_read_resp_,
              dcache_s1_q_.fill_out,
              fill_in_);
  dcache_.stage2_comb();
  Dcache_Write(pending_writes_,
               lru_updates_,
               fill_writes_);
  drive_dcache_response();
}

void MemSubsystem::record_dcache_stage1_metadata() {
  for (auto &meta : dcache_resp_meta_nxt_) {
    meta = {};
  }
  const S1S2Reg &stage1 = dcache_.stage1_next_state();
  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    const auto &load = stage1.loads[i];
    if (!load.valid) {
      continue;
    }
    dcache_resp_meta_nxt_[i].valid = true;
    dcache_resp_meta_nxt_[i].is_store = false;
    dcache_resp_meta_nxt_[i].addr = load.addr;
  }
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    const auto &store = stage1.stores[i];
    if (!store.valid) {
      continue;
    }
    const int idx = LSU_LDU_COUNT + i;
    dcache_resp_meta_nxt_[idx].valid = true;
    dcache_resp_meta_nxt_[idx].is_store = true;
    dcache_resp_meta_nxt_[idx].addr = store.addr;
  }
}

// This cycle's DCache response as the route block sees it (and through it the LSU, the I-cache probe, the PTW): the
// valid stage-2 responses, a response that waits for an MSHR fill arriving now turned into a hit (matched through the
// stage-1 metadata register), and the hidden bank-retry busy bits.
void MemSubsystem::drive_dcache_response() {
  dcache_resp_route_ = {};
  fill_store_dirty_ = false;
  for (int i = 0; i < LSU_LDU_COUNT; i++)
    if (dcache_resp_raw_.resp_ports.load_resps[i].valid) dcache_resp_route_.resp_ports.load_resps[i] = dcache_resp_raw_.resp_ports.load_resps[i];
  for (int i = 0; i < LSU_STA_COUNT; i++)
    if (dcache_resp_raw_.resp_ports.store_resps[i].valid) dcache_resp_route_.resp_ports.store_resps[i] = dcache_resp_raw_.resp_ports.store_resps[i];
  if (dcache_resp_raw_.mshr_fill) {
    dcache_resp_route_.mshr_fill = true;
    dcache_resp_route_.mshr_fill_addr = dcache_resp_raw_.mshr_fill_addr;
  }

  const MSHR_FILLReq &fill = mshr_dcache_io_.fill_req;
  if (fill.valid) {
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
      const LoadResp &resp = dcache_resp_route_.resp_ports.load_resps[i];
      const DcacheRespMeta &meta = dcache_resp_meta_cur_[i];
      if (!resp.valid || !meta.valid || meta.is_store || !replay_waits_for_mshr_fill(resp.replay) ||
          !cache_line_match(meta.addr, fill.addr))
        continue;
      dcache_resp_route_.resp_ports.load_resps[i].replay = ReplayType::HIT;
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
      const StoreResp &resp = dcache_resp_route_.resp_ports.store_resps[i];
      const DcacheRespMeta &meta = dcache_resp_meta_cur_[LSU_LDU_COUNT + i];
      if (!resp.valid || !meta.valid || !meta.is_store || !replay_waits_for_mshr_fill(resp.replay) ||
          !cache_line_match(meta.addr, fill.addr))
        continue;
      fill_store_dirty_ = true;
      dcache_resp_route_.resp_ports.store_resps[i].replay = ReplayType::HIT;
    }
  }

  for (int i = 0; i < LSU_LDU_COUNT; i++) dcache_resp_route_.load_retry_busy[i] = dcache_.load_retry_pending(i);
  for (int i = 0; i < LSU_STA_COUNT; i++) dcache_resp_route_.store_retry_busy[i] = dcache_.store_retry_pending(i);
}

// DCache stage 1 of this cycle's requests (a port whose response is still held is blocked), and its metadata for the
// response next cycle.
void MemSubsystem::dcache_stage1() {
  bool load_resp_blocked[LSU_LDU_COUNT] = {};
  bool store_resp_blocked[LSU_STA_COUNT] = {};
  for (int i = 0; i < LSU_LDU_COUNT; i++) {
    load_resp_blocked[i] =
        dcache_resp_route_.resp_ports.load_resps[i].valid;
  }
  for (int i = 0; i < LSU_STA_COUNT; i++) {
    store_resp_blocked[i] =
        dcache_resp_route_.resp_ports.store_resps[i].valid;
  }
  dcache_.set_hidden_bank_retry_blocked(load_resp_blocked,
                                        store_resp_blocked);

  dcache_.stage1_comb();
  record_dcache_stage1_metadata();
}

void MemSubsystem::dcache_domain_seq(bool sample_mshr_perf) {
  dcache_.seq();
  std::memcpy(dcache_s1_q_.find_req, dcache_mshr_io_.find_req, sizeof(dcache_s1_q_.find_req));
  std::memcpy(dcache_s1_q_.bypass_req, dcache_wb_io_.bypass_req, sizeof(dcache_s1_q_.bypass_req));
  std::memcpy(dcache_s1_q_.merge_req, dcache_wb_io_.merge_req, sizeof(dcache_s1_q_.merge_req));
  std::memcpy(dcache_s1_q_.line_read_req, dcache_line_read_req_, sizeof(dcache_s1_q_.line_read_req));
  dcache_s1_q_.fill_out = fill_out_;
  for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
    dcache_resp_meta_cur_[i] = dcache_resp_meta_nxt_[i];
  }
  mshr_.seq(sample_mshr_perf);
  wb_.seq();
}

// The responses (before the back end: the AXI responses first, so a line that returns or a write-back that retires now is
// visible to the DCache this cycle) and the requests (after it: newly allocated MSHR / write-buffer entries start their
// AXI transaction in the same cycle); both from the contracts in MemSubsystem.h.
void MemSubsystem::comb_outputs() {
  // BEGIN GENERATED by scripts/comb/gen_mem_order.py from the contracts in MemSubsystem.h: do not edit
  wb_.comb_axi_to_dcache();
  mshr_.comb_axi_to_dcache();
  dcache_stage2();
  mem_route_block.comb_response();
  // END GENERATED
}

void MemSubsystem::comb_inputs() {
  // BEGIN GENERATED by scripts/comb/gen_mem_order.py from the contracts in MemSubsystem.h: do not edit
  ptw_request();
  mem_route_block.comb_request();
  dcache_stage1();
  mshr_.comb_dcache_to_axi();
  wb_.comb_dcache_to_axi();
  ptw_feedback();
  peripheral_axi_.comb();
  // END GENERATED
}

// The shared page-table walker: the TLBs' walk requests in, its memory read request to the route block.
void MemSubsystem::ptw_request() {
  MemPtwBlock::PortIn ptw_port_in{};
  {
    const auto &dtlb_walk_in = dtlb_walk_port_inst->comb_input();
    ptw_port_in.walk_clients[static_cast<size_t>(PtwClient::DTLB)].req_valid =
        dtlb_walk_in.req_valid;
    ptw_port_in.walk_clients[static_cast<size_t>(PtwClient::DTLB)].req =
        dtlb_walk_in.req;
    ptw_port_in.walk_clients[static_cast<size_t>(PtwClient::DTLB)]
        .resp_consumed = dtlb_walk_in.resp_consumed;

    const auto &itlb_walk_in = itlb_walk_port_inst->comb_input();
    ptw_port_in.walk_clients[static_cast<size_t>(PtwClient::ITLB)].req_valid =
        itlb_walk_in.req_valid;
    ptw_port_in.walk_clients[static_cast<size_t>(PtwClient::ITLB)].req =
        itlb_walk_in.req;
    ptw_port_in.walk_clients[static_cast<size_t>(PtwClient::ITLB)]
        .resp_consumed = itlb_walk_in.resp_consumed;
  }
  ptw_block.comb_begin(ptw_port_in);
  const auto &ptw_out = ptw_block.comb_outputs();

  ptw_walk_req.valid = ptw_out.issue_walk_read;
  ptw_walk_req.addr = ptw_out.walk_read_addr;
}

// The shared page-table walker: the route block's grant / response back; the TLBs' request inputs of this cycle are
// consumed.
void MemSubsystem::ptw_feedback() {
  MemPtwBlock::FeedbackIn ptw_feedback{};

  ptw_feedback.wakeup_walk = wakeup.walk;

  // grant 转换
  if (ptw_grant.valid) {
    ptw_feedback.grant_valid = true;
    ptw_feedback.grant_req_id = ptw_grant.req_id;

    if (ptw_grant.owner != Owner::PTW_WALK) ptw_feedback.grant_valid = false;
  }

  // 单个 ptw_event 转换
  if (ptw_events.valid) {
    const auto &evt = ptw_events;

    MemPtwBlock::RoutedEvent mapped{};
    mapped.valid = true;
    mapped.data = evt.data;
    mapped.replay = evt.replay;
    mapped.req_addr = evt.req_addr;
    mapped.req_id = evt.req_id;

    if (evt.owner != Owner::PTW_WALK) mapped.valid = false;

    if (mapped.valid) {
      ptw_feedback.events[0] = mapped;
      ptw_feedback.event_count = 1;
    }
  }

  ptw_feedback.walk_client_flush[static_cast<size_t>(PtwClient::DTLB)] = dtlb_walk_port_inst->seq_input().flush;
  ptw_feedback.walk_client_flush[static_cast<size_t>(PtwClient::ITLB)] = itlb_walk_port_inst->seq_input().flush;
  ptw_block.comb_finish(ptw_feedback);
  dtlb_walk_port_inst->reset_cycle_input();
  itlb_walk_port_inst->reset_cycle_input();
}

// The walk-port outputs (req_ready / resp of the shared PTW) are registers: set here, at the clock edge, from the PTW
// block's state and read by the TLBs during the next cycle.
void MemSubsystem::seq() {
  ptw_block.seq();
  dcache_domain_seq(true);
  peripheral_model_.tick();
  peripheral_axi_.seq();
  mem_route_block.seq();
  dtlb_walk_port_inst->reset_seq_input();
  itlb_walk_port_inst->reset_seq_input();
  sync_ptw_port_outputs();
}

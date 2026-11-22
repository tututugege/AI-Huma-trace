#include "MemSubsystem.h"
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
  dcache_.out.dcache2route = &dcache_resp_route_;

  // Internal MSHR ↔ DCache wires: RealDcache reads/writes MSHR IO structs
  // directly via pointers, keeping the connection zero-copy.
  dcache_.in.mshr2dcache = &mshr_dcache_io_;  // MSHR output → DCache input
  dcache_.out.dcache2mshr = &dcache_mshr_io_; // DCache output → MSHR input
  mshr_.in.dcache2mshr = &dcache_mshr_io_;    // DCache output → MSHR comb input
  mshr_.in.s1_q = &dcache_.s1_q();
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
  wb_.in.s1_q = &dcache_.s1_q();
  wb_.out.wb2dcache = &wb_dcache_io_;     // WB output → DCache inputs


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

  // the shared page-table walker: the TLBs' walk ports, the route block's request / grant / response
  ptw_block.in.dtlb_walk = &dtlb_walk_req;
  ptw_block.in.itlb_walk = &itlb_walk_req;
  ptw_block.in.ptw_grant = &ptw_grant;
  ptw_block.in.ptw_events = &ptw_events;
  ptw_block.in.wakeup = &wakeup;
  ptw_block.out.ptw_walk_req = &ptw_walk_req;
  ptw_block.out.dtlb_walk = &dtlb_walk_req;
  ptw_block.out.itlb_walk = &itlb_walk_req;


  // ── Initialise sub-modules ─────────────────────────────────────────────────
  mshr_.init();
  wb_.init();
  dcache_.init();
  peripheral_axi_.in.commit_store = commit_store;
  peripheral_axi_.out.csr_interrupt_inject = csr_interrupt_inject;
  peripheral_model_.csr_interrupt_inject = peripheral_axi_.out.csr_interrupt_inject;
  peripheral_model_.reset_devices();
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
  dtlb_walk_req = {};
  itlb_walk_req = {};

  dcache_req_mux_ = {};
  dcache_resp_route_ = {};
  dcache_empty_req_ = LsuDcacheIO{};

  mshr_dcache_io_ = {};
  dcache_mshr_io_ = {};
  wb_dcache_io_ = {};
  dcache_wb_io_ = {};



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


void MemSubsystem::configure_interrupt_latency(uint64_t seed,
                                               uint32_t min_cycles,
                                               uint32_t max_cycles) {
  peripheral_model_.configure_interrupt_latency(seed, min_cycles, max_cycles);
}


void MemSubsystem::sync_mmio_devices_from_backing() {
  peripheral_model_.sync_from_backing();
}
// DCache stage 2 of the request that stage 1 issued last cycle: the tag / fill array read, the stage-2 logic, the
// array writes, the response to the route block.
// DCache stage 1 of this cycle's requests (a port whose response is still held is blocked), and its metadata for the
// response next cycle.


void MemSubsystem::dcache_domain_seq(bool sample_mshr_perf) {
  dcache_.seq();   // (also its stage-1 request / response metadata registers)
  mshr_.seq(sample_mshr_perf);
  wb_.seq();
}

// The responses (before the back end: the AXI responses first, so a line that returns or a write-back that retires now is
// visible to the DCache this cycle) and the requests (after it: newly allocated MSHR / write-buffer entries start their
// AXI transaction in the same cycle); both from the contracts in MemSubsystem.h.
// The walk-port outputs (req_ready / resp of the shared PTW) are registers: set here, at the clock edge, from the PTW
// block's state and read by the TLBs during the next cycle.
void MemSubsystem::seq() {
  ptw_block.seq();
  dcache_domain_seq(true);
  peripheral_axi_.seq();   // (its MMIO load reads the devices before their tick)
  peripheral_model_.tick();
  mem_route_block.seq();
}

// ---- the shared page-table walker's comb phases (MemPtwBlock.h) ----

// The TLBs' walk requests in, the walker's memory read request to the route block.
void MemPtwBlock::comb_request() {
  PortIn ptw_port_in{};
  {
    const PtwWalkPortCombIn &dtlb_walk_in = *in.dtlb_walk;
    ptw_port_in.walk_clients[static_cast<size_t>(Client::DTLB)].req_valid = dtlb_walk_in.req_valid;
    ptw_port_in.walk_clients[static_cast<size_t>(Client::DTLB)].req = dtlb_walk_in.req;
    ptw_port_in.walk_clients[static_cast<size_t>(Client::DTLB)].resp_consumed = dtlb_walk_in.resp_consumed;

    const PtwWalkPortCombIn &itlb_walk_in = *in.itlb_walk;
    ptw_port_in.walk_clients[static_cast<size_t>(Client::ITLB)].req_valid = itlb_walk_in.req_valid;
    ptw_port_in.walk_clients[static_cast<size_t>(Client::ITLB)].req = itlb_walk_in.req;
    ptw_port_in.walk_clients[static_cast<size_t>(Client::ITLB)].resp_consumed = itlb_walk_in.resp_consumed;
  }
  comb_begin(ptw_port_in);
  const auto &ptw_out = comb_outputs();

  out.ptw_walk_req->valid = ptw_out.issue_walk_read;
  out.ptw_walk_req->addr = ptw_out.walk_read_addr;
}

// The route block's grant / response back; the TLBs' request inputs of this cycle are consumed.
void MemPtwBlock::comb_feedback() {
  FeedbackIn ptw_feedback{};

  ptw_feedback.wakeup_walk = in.wakeup->walk;

  // the grant
  if (in.ptw_grant->valid) {
    ptw_feedback.grant_valid = true;
    ptw_feedback.grant_req_id = in.ptw_grant->req_id;

    if (in.ptw_grant->owner != Owner::PTW_WALK) ptw_feedback.grant_valid = false;
  }

  // the walker read response (one event)
  if (in.ptw_events->valid) {
    const PtwEvent &evt = *in.ptw_events;

    RoutedEvent mapped{};
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

  ptw_feedback.walk_client_flush[static_cast<size_t>(Client::DTLB)] = in.dtlb_walk->flush;
  ptw_feedback.walk_client_flush[static_cast<size_t>(Client::ITLB)] = in.itlb_walk->flush;
  ptw_feedback.sfence_flush = in.dtlb_walk->sfence_flush;
  comb_finish(ptw_feedback);
  *out.dtlb_walk = {};
  *out.itlb_walk = {};
}

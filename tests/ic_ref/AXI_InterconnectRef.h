#pragma once
// Frozen copy of the original AXI_Interconnect (differential reference, tests/ic_diff.cpp). Shares the helper structs with the live class.
#include "AXI_Interconnect.h"

namespace axi_interconnect {

class AXI_InterconnectRef {
public:
  // True when no transaction is in flight anywhere in the interconnect/LLC and
  // no master drives a request: further idle ticks do not change state.
  bool idle_quiescent() const;
  AXI_InterconnectRef()
      : write_port(write_ports[MASTER_DCACHE_W]), axi_io(axi_ddr_io) {}

  // Runtime control inputs for the shared AXI submodule.
  // For simulator-only bring-up, init() may seed these inputs from env vars,
  // but the active mode/offset seen by the datapath remains hardware-like:
  // the module observes these inputs at runtime and performs a flush when a
  // mode transition involving mode=2 is requested.
  wire<2> mode = 1;
  wire<32> llc_mapped_offset = 0;

  void init();
  void set_llc_config(const AXI_LLCConfig &config) {
    llc_config = config;
    llc.set_config(llc_config);
  }
  const AXI_LLCConfig &get_llc_config() const { return llc_config; }
  bool llc_enabled() const;
  void set_llc_lookup_in(const AXI_LLC_LookupIn_t &lookup_in) {
    llc.io.lookup_in = lookup_in;
  }
  void set_llc_invalidate_all(bool invalidate) { llc_invalidate_all_req_ = invalidate; }
  void set_llc_invalidate_line(bool valid, uint32_t addr) {
    llc_invalidate_line_valid_ = valid;
    llc_invalidate_line_addr_ = addr;
  }
  uint8_t active_mode() const { return runtime_mode_; }
  uint32_t active_llc_mapped_offset() const { return llc_mapped_offset_; }
  bool llc_invalidate_all_accepted() const {
    return llc.io.ext_out.mem.invalidate_all_accepted;
  }
  bool llc_invalidate_line_accepted() const {
    return llc.io.ext_out.mem.invalidate_line_accepted;
  }
  const AXI_LLC_TableOut_t &get_llc_table_out() const { return llc.io.table_out; }
  const AXI_LLCPerfCounters_t &get_llc_perf_counters() const {
    return llc.perf_counters();
  }

  // Two-phase combinational logic for proper signal timing
  void comb_outputs(); // Phase 1: Update resp signals for masters, req.ready
  void comb_inputs();  // Phase 2: Process req from masters, drive DDR AR/AW/W
  void comb() {
    comb_outputs();
    comb_inputs();
  } // Convenience wrapper

  void seq();

  void debug_print();

  // Upstream IO (Masters)
  ReadMasterPort_t read_ports[NUM_READ_MASTERS];
  WriteMasterPort_t write_ports[NUM_WRITE_MASTERS];
  // Backward-compatible alias to the primary write master.
  WriteMasterPort_t &write_port;

  // Downstream IO. axi_io is a legacy alias to axi_ddr_io.
  sim_ddr::SimDDR_IO_t axi_ddr_io;
  sim_ddr::SimDDR_IO_t axi_mmio_io;
  sim_ddr::SimDDR_IO_t &axi_io;
  bool read_req_accepted[NUM_READ_MASTERS] = {};
  uint8_t read_req_accepted_id[NUM_READ_MASTERS] = {};
  bool write_req_accepted[NUM_WRITE_MASTERS] = {};

private:
  uint8_t count_master_read_pending(uint8_t master_id) const;
  uint8_t count_total_read_inflight() const;
  bool can_accept_read_master(uint8_t master_id) const;
  bool has_read_id_conflict(uint8_t master_id, uint8_t orig_id) const;
  uint8_t alloc_write_axi_id() const;
  uint8_t alloc_write_axi_id(DownstreamPort port) const;
  bool can_accept_write_now() const;
  bool can_issue_external_read(uint32_t addr) const;
  bool can_issue_external_write(uint32_t addr) const;
  uint32_t count_llc_write_pending() const;
  int find_write_pending_by_axi_id(DownstreamPort port, uint8_t axi_id) const;
  int find_next_aw_pending() const;
  int find_next_w_pending() const;
  int find_next_aw_pending(DownstreamPort port) const;
  int find_next_w_pending(DownstreamPort port) const;
  void refresh_non_llc_w_active(DownstreamPort port);
  ARLatch_t &ar_latch(DownstreamPort port);
  const ARLatch_t &ar_latch(DownstreamPort port) const;
  AWLatch_t &aw_latch(DownstreamPort port);
  const AWLatch_t &aw_latch(DownstreamPort port) const;
  bool &w_active_ref(DownstreamPort port);
  bool w_active_ref(DownstreamPort port) const;
  WritePendingTxn &w_current_ref(DownstreamPort port);
  const WritePendingTxn &w_current_ref(DownstreamPort port) const;
  int &w_current_master_ref(DownstreamPort port);
  bool any_ar_latched() const;
  bool any_aw_latched() const;
  bool any_w_active() const;
  bool external_write_busy() const;
  uint8_t write_resp_buffer_count(uint8_t master) const;
  bool write_resp_buffer_has_space(uint8_t master) const;
  bool any_write_resp_buffered() const;
  void enqueue_write_resp(uint8_t master, uint8_t id, uint8_t resp,
                          bool line_valid = false, uint32_t line_addr = 0);
  void promote_write_resp_queue();

  // Read arbiter state
  uint8_t r_arb_rr_idx;
  int r_current_master;

  // Registered req.ready for each master (persists until handshake)
  bool req_ready_r[NUM_READ_MASTERS];
  bool req_drop_warned[NUM_READ_MASTERS];
  uint8_t w_arb_rr_idx;
  int w_current_master;
  int w_current_master_mmio;
  bool w_req_ready_r[NUM_WRITE_MASTERS];
  bool write_req_fire_c[NUM_WRITE_MASTERS];

  // AR latch for AXI compliance
  ARLatch_t ar_latched;
  ARLatch_t ar_latched_mmio;

  // Pending read transactions
  std::vector<ReadPendingTxn> r_pending;
  uint32_t direct_read_completion_seq_ = 0;

  // Write state
  bool w_active;
  bool w_active_mmio;
  WritePendingTxn w_current;
  WritePendingTxn w_current_mmio;
  std::deque<WritePendingTxn> w_pending;
  bool w_resp_valid[NUM_WRITE_MASTERS];
  uint8_t w_resp_id[NUM_WRITE_MASTERS];
  uint8_t w_resp_resp[NUM_WRITE_MASTERS];
  std::deque<WriteRespEntry> w_resp_queue[NUM_WRITE_MASTERS];
  std::deque<DelayedWriteRespEntry> llc_write_resp_delay[NUM_WRITE_MASTERS];

  // AW latch for AXI compliance
  AWLatch_t aw_latched;
  AWLatch_t aw_latched_mmio;

  void comb_read_arbiter();
  void comb_read_response();
  void comb_write_request();
  void comb_write_response();
  void prepare_llc_inputs(bool sample_upstream_ready);
  sim_ddr::SimDDR_IO_t &downstream_io(DownstreamPort port);
  const sim_ddr::SimDDR_IO_t &downstream_io(DownstreamPort port) const;
  bool can_issue_llc_read_req() const;
  bool has_same_line_write_hazard(uint32_t line_addr) const;
  bool has_same_line_read_hazard(uint32_t line_addr) const;
  bool has_external_pending_read_hazard(uint32_t addr) const;
  bool has_external_pending_write_hazard(uint32_t addr) const;
  bool has_direct_read_response(uint8_t master_id) const;
  uint32_t external_hazard_line_addr(uint32_t addr) const;
  DownstreamPort classify_downstream_port(uint32_t addr,
                                          uint8_t total_size) const;
  bool request_uses_ddr_port(uint32_t addr, uint8_t total_size) const;
  bool request_uses_mmio_port(uint32_t addr, uint8_t total_size) const;

  uint8_t calc_burst_len(uint8_t total_size);
  uint8_t alloc_read_axi_id() const;
  void sample_runtime_controls();
  uint8_t requested_mode() const;
  uint32_t requested_llc_mapped_offset() const;
  bool requested_config_differs_from_active() const;
  bool mode_transition_needs_flush() const;
  bool mode_transition_invalidate_requested() const;
  bool invalidate_all_requested() const;
  bool request_in_mapped_window(uint32_t addr, uint8_t total_size) const;
  bool request_uses_direct_mapped_llc(uint32_t addr, uint8_t total_size) const;
  uint32_t translate_llc_addr(uint32_t addr, uint8_t total_size) const;
  bool effective_llc_bypass(uint32_t addr, uint8_t total_size,
                            bool upstream_bypass) const;
  bool non_llc_path_quiescent() const;
  bool llc_path_quiescent() const;
  bool reconfig_path_quiescent() const;

  AXI_LLCConfig llc_config{};
  AXI_LLC llc{};
  bool llc_invalidate_all_req_ = false;
  bool llc_invalidate_line_valid_ = false;
  uint32_t llc_invalidate_line_addr_ = 0;
  ReadReqHoldLatch read_req_hold[NUM_READ_MASTERS] = {};
  LlcUpstreamReqLatch llc_upstream_req[NUM_READ_MASTERS] = {};
  LlcUpstreamReqLatch llc_upstream_capture_c[NUM_READ_MASTERS] = {};
  bool llc_upstream_accept_c[NUM_READ_MASTERS] = {};
  LlcUpstreamWriteReqLatch llc_upstream_write_req[NUM_WRITE_MASTERS] = {};
  LlcUpstreamWriteReqLatch llc_upstream_write_capture_c[NUM_WRITE_MASTERS] = {};
  bool llc_upstream_write_accept_c[NUM_WRITE_MASTERS] = {};
  std::deque<LlcUpstreamWriteReqLatch> llc_upstream_write_q[NUM_WRITE_MASTERS];
  LlcCoreReqStage llc_core_req_stage_ = {};
  uint8_t llc_core_dispatch_rr_ = 0;
  bool llc_read_resp_valid_[NUM_READ_MASTERS] = {};
  WideReadData_t llc_read_resp_data_[NUM_READ_MASTERS] = {};
  uint8_t llc_read_resp_id_[NUM_READ_MASTERS] = {};
  bool llc_mem_write_resp_valid_ = false;
  uint8_t llc_mem_write_resp_ = 0;
  uint32_t llc_mem_ignored_b_count_ = 0;
  bool llc_synth_read_resp_valid_ = false;
  uint8_t llc_synth_read_resp_id_ = 0;
  WideReadData_t llc_synth_read_resp_data_{};
  bool llc_mem_read_issue_seen_valid_ = false;
  uint8_t llc_mem_read_issue_seen_id_ = 0;
  uint32_t llc_mem_read_issue_seen_addr_ = 0;
  uint8_t llc_mem_read_issue_seen_size_ = 0;
  bool ar_from_llc_c = false;
  uint8_t ar_llc_mem_id_c = 0;
  DownstreamPort ar_port_c = DownstreamPort::DDR;
  ARIssueMeta_t ar_issue_c[2] = {};
  bool ar_llc_resp_extract_from_aligned_beat_c = false;
  uint32_t ar_llc_upstream_addr_c = 0;
  uint8_t ar_llc_upstream_total_size_c = 0;
  int ar_master_c = -1;
  uint8_t ar_orig_id_c = 0;
  DownstreamPort aw_port_c = DownstreamPort::DDR;
  uint8_t runtime_mode_ = 1;
  uint32_t llc_mapped_offset_ = 0;
  bool llc_mapped_offset_override_ = false;
  bool reconfig_pending_ = false;
  uint8_t reconfig_target_mode_ = 1;
  uint32_t reconfig_target_offset_ = 0;
  static constexpr uint32_t kMappedLlcWindowBytes = 4u << 20;
};

} // namespace axi_interconnect

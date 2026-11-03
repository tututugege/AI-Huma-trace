#pragma once
/**
 * @file AXI_Interconnect.h
 * @brief AXI-Interconnect Layer
 *
 * Converts simplified master interfaces (single-beat, wide data) to
 * AXI4 protocol (multi-beat bursts) for connection to SimDDR.
 *
 * AXI Protocol Compliance:
 * - AR/AW valid signals are latched until ready handshake
 * - Upstream req_valid can be deasserted without affecting AXI valid
 */

#include "AXI_Interconnect_IO.h"
#include "AXI_LLC.h"
#include "SimDDR_IO.h"
#include "axi_interconnect_compat.h"
#include <deque>
#include <queue>
#include <vector>

namespace axi_interconnect {

enum class DownstreamPort : uint8_t {
  DDR = 0,
  MMIO = 1,
};

struct DownstreamReadIssueProbe {
  DownstreamPort port = DownstreamPort::DDR;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  bool extract_from_aligned_beat = false;
};

struct DownstreamWriteIssueProbe {
  DownstreamPort port = DownstreamPort::DDR;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  WideWriteData_t wdata{};
  WideWriteStrb_t wstrb{};
};

// Verification/test hook for the same production issue-shaping logic used by
// AXI_Interconnect comb paths. Keep these as thin wrappers; they are not a
// separate reference model.
DownstreamReadIssueProbe probe_downstream_read_issue(DownstreamPort port,
                                                     uint32_t addr,
                                                     uint8_t total_size,
                                                     uint32_t line_bytes,
                                                     bool force_line_aligned);
DownstreamWriteIssueProbe
probe_downstream_write_issue(DownstreamPort port, uint32_t addr,
                             uint8_t total_size, const WideWriteData_t &wdata,
                             const WideWriteStrb_t &wstrb,
                             uint32_t line_bytes, bool force_line_aligned);

// ============================================================================
// Latched AR/AW Requests (for AXI compliance)
// ============================================================================

// Latched AR request - holds values until arready
struct ARLatch_t {
  bool valid;
  bool accepted_upstream;
  bool to_llc;
  bool resp_extract_from_aligned_beat;
  DownstreamPort port;
  uint32_t addr;
  uint32_t upstream_addr;
  uint8_t len;
  uint8_t size;
  uint8_t burst;
  uint8_t id;
  uint8_t master_id;
  uint8_t orig_id;
  uint8_t upstream_total_size;
};

// Latched AW request - holds values until awready
struct AWLatch_t {
  bool valid;
  DownstreamPort port;
  uint32_t addr;
  uint8_t len;
  uint8_t size;
  uint8_t burst;
  uint8_t id;
};

struct ARIssueMeta_t {
  bool valid = false;
  bool from_llc = false;
  bool resp_extract_from_aligned_beat = false;
  uint8_t llc_mem_id = 0;
  uint32_t upstream_addr = 0;
  uint8_t upstream_total_size = 0;
  int master_id = -1;
  uint8_t orig_id = 0;
};

// ============================================================================
// Pending Transaction Tracking
// ============================================================================

struct ReadPendingTxn {
  uint8_t axi_id;
  uint8_t master_id;
  uint8_t orig_id;
  uint8_t total_beats;
  uint8_t beats_done;
  DownstreamPort port;
  uint32_t addr;
  uint32_t upstream_addr;
  uint8_t upstream_total_size;
  bool resp_extract_from_aligned_beat;
  bool to_llc;
  WideReadData_t data;
  uint32_t stall_cycles;
  uint8_t last_beats_done;
  bool timeout_warned;
  bool completion_queued = false;
  uint32_t completion_seq = 0;
};

struct LlcUpstreamReqLatch {
  bool valid = false;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  uint8_t id = 0;
  bool bypass = false;
  bool direct_mapped = false;
  bool mode2_ddr_aligned = false;
};

struct ReadReqHoldLatch {
  bool valid = false;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  uint8_t id = 0;
  bool bypass = false;
};

struct LlcUpstreamWriteReqLatch {
  bool valid = false;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  uint8_t id = 0;
  WideWriteData_t wdata{};
  WideWriteStrb_t wstrb{};
  bool bypass = false;
  bool direct_mapped = false;
  bool mode2_ddr_aligned = false;
};

struct LlcCoreReqStage {
  bool valid = false;
  bool is_write = false;
  uint8_t master = 0;
  LlcUpstreamReqLatch read{};
  LlcUpstreamWriteReqLatch write{};
};

struct WritePendingTxn {
  uint8_t axi_id;
  uint8_t master_id;
  uint8_t orig_id;
  DownstreamPort port;
  uint32_t addr;
  WideWriteData_t wdata;
  WideWriteStrb_t wstrb{};
  uint8_t total_beats;
  uint8_t beats_sent;
  bool aw_done;
  bool w_done;
  bool to_llc_mem;
  bool llc_victim_write;
};

struct WriteRespEntry {
  uint8_t id = 0;
  uint8_t resp = 0;
  bool line_valid = false;
  uint32_t line_addr = 0;
};

struct DelayedWriteRespEntry {
  WriteRespEntry entry{};
  uint8_t age = 0;
};

// ============================================================================
// AXI_Interconnect Class
// ============================================================================

class AXI_Interconnect {
public:
  // True when no transaction is in flight anywhere in the interconnect/LLC and
  // no master drives a request: further idle ticks do not change state.
  bool idle_quiescent() const;
  AXI_Interconnect()
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

  // Registered req.ready for each master (persists until handshake)
  bool req_ready_r[NUM_READ_MASTERS];
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

  // sequential-phase sections (SeqPrev: inputs sampled before seq() changes anything)
  struct SeqPrev {
    LlcCoreReqStage stage;
    bool mem_write_resp_valid;
    bool synth_read_resp_valid;
  };
  void seq_read_ar();
  void seq_llc_dispatch(const SeqPrev &prev);
  void seq_read_data(const SeqPrev &prev);
  void seq_llc_write(const SeqPrev &prev);
  void seq_direct_write();
  void start_downstream_write(DownstreamPort port, uint32_t addr, uint8_t issue_size, const WideWriteData_t &wdata,
                              const WideWriteStrb_t &wstrb, uint8_t axi_id, uint8_t master_id, uint8_t orig_id,
                              bool to_llc_mem, bool victim_write, int current_master);
  void seq_finish(const SeqPrev &prev);
  void check_llc_consumed_reads(const SeqPrev &prev);

  void comb_read_arbiter();
  void read_comb_begin();
  void read_replay_latched_ar();
  void read_issue_llc_mem_read();
  void read_route_masters_llc();
  void read_complete_ready_handshakes();
  void read_arbitrate_new();
  uint8_t alloc_comb_read_axi_id(DownstreamPort port);
  bool can_accept_read_master_comb(uint8_t master_id) const;
  void drive_read_ar(DownstreamPort port, uint32_t addr, uint8_t total_size, bool mode2_ddr_aligned, uint8_t axi_id,
                     ARIssueMeta_t meta);
  void issue_master_read(int master, DownstreamPort port, uint32_t addr, uint8_t total_size, uint8_t id,
                         uint8_t axi_id);
  ReadReqHoldLatch read_req_snapshot(int master) const;
  void comb_read_response();
  void comb_write_request();
  void drive_aw(DownstreamPort port, uint32_t addr, uint8_t len, uint8_t size, uint8_t burst, uint8_t id);
  void drive_aw_latch(DownstreamPort port);
  void drive_w_current(DownstreamPort port);
  void write_ready_first(int idx, bool direct_fire);
  void write_issue_llc_mem_write();
  bool write_route_direct_mmio();
  void write_route_llc();
  void write_issue_pending_aw(bool any_w_resp_valid);
  void write_accept_direct();
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
  bool mode2_ddr_aligned(uint32_t addr, uint8_t total_size) const;
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
  bool write_comb_ready_curr_[NUM_WRITE_MASTERS] = {};  // w_req_ready_r as it was before this comb_write_request() pass
  // Scratch of one comb_read_arbiter() pass.
  struct ReadComb {
    bool port_busy[2];
    bool id_used[2][1u << sim_ddr::AXI_ID_WIDTH];
    uint8_t master_count[NUM_READ_MASTERS];
    bool ready_curr[NUM_READ_MASTERS];
  } read_comb_ = {};
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
  ARIssueMeta_t ar_issue_c[2] = {};
  uint8_t runtime_mode_ = 1;
  uint32_t llc_mapped_offset_ = 0;
  bool llc_mapped_offset_override_ = false;
  bool reconfig_pending_ = false;
  uint8_t reconfig_target_mode_ = 1;
  uint32_t reconfig_target_offset_ = 0;
  static constexpr uint32_t kMappedLlcWindowBytes = 4u << 20;
};

} // namespace axi_interconnect

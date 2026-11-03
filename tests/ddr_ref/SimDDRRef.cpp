/**
 * @file SimDDRRef.cpp
 * @brief SimDDRRef Implementation - DDR Memory Simulator with AXI4 Interface
 *
 * Implementation with outstanding transaction support and burst-drain read
 * service. Reads still use round-robin arbitration, but only between bursts,
 * so a selected burst drains contiguously instead of ping-ponging every beat.
 */

#include <cstdlib>
#include "SimDDRRef.h"
#include "PhysMemory.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

extern long long sim_time;

// Cleared by the trace-driven build: functional memory is then written at ROB commit instead.

namespace sim_ddr {

namespace {
#ifndef CONFIG_AXI_LLC_FOCUS_LINE0
#define CONFIG_AXI_LLC_FOCUS_LINE0 0u
#endif

#ifndef CONFIG_AXI_LLC_FOCUS_LINE1
#define CONFIG_AXI_LLC_FOCUS_LINE1 0u
#endif


constexpr uint32_t kFocusWriteLineBytes = 64u;

bool focus_write_line(uint32_t addr) {
  if (CONFIG_AXI_LLC_DEBUG_LOG == 0) {
    return false;
  }
  const uint32_t line_addr = addr & ~(kFocusWriteLineBytes - 1u);
  return (CONFIG_AXI_LLC_FOCUS_LINE0 != 0u &&
          line_addr == static_cast<uint32_t>(CONFIG_AXI_LLC_FOCUS_LINE0)) ||
         (CONFIG_AXI_LLC_FOCUS_LINE1 != 0u &&
          line_addr == static_cast<uint32_t>(CONFIG_AXI_LLC_FOCUS_LINE1));
}

} // namespace

// ============================================================================
// Initialization
// ============================================================================
void SimDDRRef::init() {
  // Clear write channel state
  w_active = false;
  w_current = {};
  w_pending.clear();
  w_data_fifo.clear();
  w_accept_cooldown = 0;
  w_drain_cooldown = 0;
  w_drain_mode = false;

  // Clear queues
  while (!w_resp_queue.empty())
    w_resp_queue.pop();

  // Clear read transactions
  r_transactions.clear();
  r_rr_index = 0;
  r_selected_idx = -1;
  r_active_idx = -1;
  backend_last_service_mode = BackendServiceMode::None;
  backend_turnaround_cooldown = 0;
  backend_read_grant = false;
  backend_write_grant = false;
  backend_switch_pending = false;
  backend_any_request_pending = false;
  backend_switch_target_mode = BackendServiceMode::None;

  // Initialize IO outputs
  io.aw.awready = false;
  io.w.wready = false;
  io.b.bvalid = false;
  io.b.bid = 0;
  io.b.bresp = AXI_RESP_OKAY;
  io.ar.arready = false;
  io.r.rvalid = false;
  io.r.rid = 0;
  io.r.rdata = 0;
  io.r.rresp = AXI_RESP_OKAY;
  io.r.rlast = false;
}

// ============================================================================
// Two-Phase Combinational Logic
// ============================================================================

// Phase 1: Output signals (run BEFORE cpu.cycle())
// Sets: arready, rvalid, rdata, bvalid, bresp
void SimDDRRef::comb_outputs() {
  select_read_transaction();
  update_backend_arbitration();
  // Read channel outputs (rvalid, rdata, arready)
  comb_read_channel();
  // Write channel outputs (awready, wready, bvalid, bresp)
  comb_write_channel();
}

// Phase 2: Input processing (run AFTER cpu.cycle())
// Note: For SimDDRRef, both channels compute based on current io inputs,
// which are already set. The wiring order in MemorySubsystem handles
// the dependency. This function is a no-op for now but kept for symmetry.
void SimDDRRef::comb_inputs() {
  // Input processing is already handled in comb_outputs for SimDDRRef
  // because the ready/valid signals are computed in the same pass.
  // The two-phase split is primarily for AXI_Interconnect's complex logic.
}

// ============================================================================
// Combinational Logic - Write Channel
// ============================================================================
void SimDDRRef::comb_write_channel() {
  // Default outputs
  io.aw.awready = false;
  io.w.wready = false;
  io.b.bvalid = false;
  io.b.bid = 0;
  io.b.bresp = AXI_RESP_OKAY;

  // --- AW Channel: Accept new write address if the queue has room ---
  if (w_pending.size() < REF_SIM_DDR_WRITE_QUEUE_DEPTH) {
    io.aw.awready = true;
  }

  // --- W Channel: Accept write data while there is FIFO credit for the next
  // transaction in AXI order. Backpressure appears when sustained writes fill
  // the FIFO faster than the backend drain loop can consume them.
  if (find_write_data_target() >= 0 &&
      w_data_fifo.size() < REF_SIM_DDR_WRITE_DATA_FIFO_DEPTH &&
      w_accept_cooldown == 0 && !w_drain_mode) {
    io.w.wready = true;
  }

  // --- B Channel: Send response if latency complete ---
  if (!w_resp_queue.empty()) {
    RefWriteRespPending &front =
        const_cast<RefWriteRespPending &>(w_resp_queue.front());
    uint32_t required_latency = REF_SIM_DDR_WRITE_RESP_LATENCY;
    if (required_latency == 0 || front.latency_cnt >= required_latency) {
      io.b.bvalid = true;
      io.b.bid = front.id;
      io.b.bresp = AXI_RESP_OKAY;
    }
  }
}

// ============================================================================
// Find next ready transaction using round-robin once the current burst finishes
// ============================================================================
int SimDDRRef::find_next_ready_transaction() {
  if (r_transactions.empty())
    return -1;

  size_t count = r_transactions.size();

  // Start from current rr_index and search for a ready transaction
  for (size_t i = 0; i < count; i++) {
    size_t idx = (r_rr_index + i) % count;
    if (r_transactions[idx].in_data_phase && !r_transactions[idx].complete) {
      return static_cast<int>(idx);
    }
  }

  return -1; // No ready transactions
}

uint32_t SimDDRRef::turnaround_cycles(BackendServiceMode from,
                                   BackendServiceMode to) const {
  if (from == BackendServiceMode::Read && to == BackendServiceMode::Write) {
    return REF_SIM_DDR_READ_TO_WRITE_TURNAROUND;
  }
  if (from == BackendServiceMode::Write && to == BackendServiceMode::Read) {
    return REF_SIM_DDR_WRITE_TO_READ_TURNAROUND;
  }
  return 0;
}

void SimDDRRef::select_read_transaction() {
  r_selected_idx = -1;

  if (r_active_idx >= 0 &&
      r_active_idx < static_cast<int>(r_transactions.size())) {
    const RefReadTransaction &active =
        r_transactions[static_cast<size_t>(r_active_idx)];
    if (active.in_data_phase && !active.complete) {
      r_selected_idx = r_active_idx;
    }
  }

  if (r_selected_idx < 0) {
    r_selected_idx = find_next_ready_transaction();
  }
}

void SimDDRRef::update_backend_arbitration() {
  backend_read_grant = false;
  backend_write_grant = false;
  backend_switch_pending = false;
  backend_any_request_pending = false;
  backend_switch_target_mode = BackendServiceMode::None;

  const bool read_ready = r_selected_idx >= 0;
  const bool read_burst_active =
      read_ready && r_active_idx >= 0 && r_selected_idx == r_active_idx;
  const bool write_ready = w_drain_mode && !w_data_fifo.empty() &&
                           w_drain_cooldown == 0 &&
                           find_write_drain_target() >= 0;
  backend_any_request_pending = read_ready || write_ready;

  if (!backend_any_request_pending || backend_turnaround_cooldown != 0) {
    return;
  }

  BackendServiceMode desired = BackendServiceMode::None;
  if (backend_last_service_mode == BackendServiceMode::Write && write_ready &&
      should_keep_write_drain_mode()) {
    desired = BackendServiceMode::Write;
  } else if (read_burst_active) {
    desired = BackendServiceMode::Read;
  } else if (backend_last_service_mode == BackendServiceMode::Read &&
             read_ready) {
    desired = BackendServiceMode::Read;
  } else if (read_ready) {
    desired = BackendServiceMode::Read;
  } else if (write_ready) {
    desired = BackendServiceMode::Write;
  }

  if (desired == BackendServiceMode::None) {
    return;
  }

  if (backend_last_service_mode != BackendServiceMode::None &&
      desired != backend_last_service_mode) {
    const uint32_t turnaround =
        turnaround_cycles(backend_last_service_mode, desired);
    if (turnaround != 0) {
      backend_switch_pending = true;
      backend_switch_target_mode = desired;
      return;
    }
  }

  backend_read_grant = desired == BackendServiceMode::Read;
  backend_write_grant = desired == BackendServiceMode::Write;
}

int SimDDRRef::find_write_data_target() const {
  for (size_t i = 0; i < w_pending.size(); ++i) {
    if (!w_pending[i].data_done) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int SimDDRRef::find_write_drain_target() const {
  for (size_t i = 0; i < w_pending.size(); ++i) {
    if (w_pending[i].beats_drained < w_pending[i].beats_accepted) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool SimDDRRef::head_write_needs_drain() const {
  if (w_pending.empty()) {
    return false;
  }
  const RefWriteTransaction &head = w_pending.front();
  return head.data_done && head.beats_drained < head.beats_accepted;
}

bool SimDDRRef::should_enter_write_drain_mode() const {
  if (w_data_fifo.empty()) {
    return false;
  }
  return head_write_needs_drain() ||
         w_data_fifo.size() >= REF_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK;
}

bool SimDDRRef::should_keep_write_drain_mode() const {
  if (w_data_fifo.empty()) {
    return false;
  }
  if (head_write_needs_drain()) {
    return true;
  }
  return w_data_fifo.size() > REF_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK;
}

void SimDDRRef::retire_completed_writes() {
  while (!w_pending.empty()) {
    const RefWriteTransaction &front = w_pending.front();
    const uint16_t total_beats = static_cast<uint16_t>(front.len) + 1u;
    if (!front.data_done || front.beats_drained != total_beats) {
      break;
    }

    RefWriteRespPending resp{};
    resp.id = front.id;
    resp.addr = front.addr;
    resp.latency_cnt = 0;
    w_resp_queue.push(resp);
    w_pending.pop_front();
  }
}

// ============================================================================
// Combinational Logic - Read Channel with Burst-Drain Service
// ============================================================================
void SimDDRRef::comb_read_channel() {
  // Default outputs
  io.ar.arready = false;
  io.r.rvalid = false;
  io.r.rid = 0;
  io.r.rdata = 0;
  io.r.rresp = AXI_RESP_OKAY;
  io.r.rlast = false;

  // --- AR Channel: Accept new read address if not full ---
  if (r_transactions.size() < REF_SIM_DDR_MAX_OUTSTANDING) {
    io.ar.arready = true;
  }

  if (backend_read_grant && r_selected_idx >= 0) {
    RefReadTransaction &txn = r_transactions[r_selected_idx];
    io.r.rvalid = true;
    io.r.rid = txn.id;
    io.r.rdata = axi_data_t{};  // read data carries no values (see tbhook::mem_read)
    io.r.rresp = AXI_RESP_OKAY;
    io.r.rlast = (txn.beat_cnt == txn.len);
  }
}

// ============================================================================
// Sequential Logic
// ============================================================================
void SimDDRRef::seq() {
  // ========== Write Channel Sequential Logic ==========

  if (backend_turnaround_cooldown > 0) {
    backend_turnaround_cooldown--;
  }
  if (w_accept_cooldown > 0) {
    w_accept_cooldown--;
  }
  if (w_drain_cooldown > 0) {
    w_drain_cooldown--;
  }

  // B handshake: Complete response
  if (io.b.bvalid && io.b.bready) {
    if (!w_resp_queue.empty()) {
      const auto &resp = w_resp_queue.front();
    }
    w_resp_queue.pop();
  }

  // Responses age once per full cycle after enqueue. Increment existing
  // responses before processing the current cycle's W handshake so newly
  // enqueued responses keep latency_cnt=0 until the next cycle.
  std::queue<RefWriteRespPending> temp_queue;
  while (!w_resp_queue.empty()) {
    RefWriteRespPending resp = w_resp_queue.front();
    w_resp_queue.pop();
    resp.latency_cnt++;
    temp_queue.push(resp);
  }
  w_resp_queue = temp_queue;

  // AW handshake: Start new write transaction
  if (io.aw.awvalid && io.aw.awready) {
    RefWriteTransaction txn{};
    txn.addr = io.aw.awaddr;
    txn.id = io.aw.awid;
    txn.len = io.aw.awlen;
    txn.size = io.aw.awsize;
    txn.burst = io.aw.awburst;
    txn.beats_accepted = 0;
    txn.beats_drained = 0;
    txn.data_done = false;
    w_pending.push_back(txn);
  }

  // W handshake: Process write data
  if (io.w.wvalid && io.w.wready) {
    const int target_idx = find_write_data_target();
    if (target_idx >= 0) {
      RefWriteTransaction &txn = w_pending[static_cast<size_t>(target_idx)];
      uint32_t current_addr =
          txn.addr + (static_cast<uint32_t>(txn.beats_accepted) << txn.size);
      RefWriteBeatPending beat{};
      beat.addr = current_addr;
      beat.data = io.w.wdata;
      beat.wstrb = io.w.wstrb;

      w_data_fifo.push_back(beat);
      txn.beats_accepted++;
      w_accept_cooldown = REF_SIM_DDR_WRITE_ACCEPT_GAP;

      if (focus_write_line(txn.addr)) {
        const std::string data_hex =
            axi_compat::hex_string(io.w.wdata, AXI_DATA_BYTES);
        const std::string wstrb_hex =
            axi_compat::hex_string(io.w.wstrb, AXI_STRB_STORAGE_BYTES);

      }

      if (io.w.wlast) {
        txn.data_done = true;
      }
    }
  }

  if (!w_drain_mode && should_enter_write_drain_mode()) {
    w_drain_mode = true;
  }

  // Drain at most one buffered W beat per cycle into backing memory while the
  // controller is in write-drain mode. This creates bursty W backpressure
  // instead of reopening WREADY immediately after a single beat drains.
  if (backend_write_grant && w_drain_mode && !w_data_fifo.empty() &&
      w_drain_cooldown == 0) {
    const int target_idx = find_write_drain_target();
    if (target_idx >= 0) {
      w_data_fifo.pop_front();
      RefWriteTransaction &txn = w_pending[static_cast<size_t>(target_idx)];
      txn.beats_drained++;
      w_drain_cooldown = REF_SIM_DDR_WRITE_DRAIN_GAP;
    }
  }

  retire_completed_writes();

  if (w_drain_mode && !should_keep_write_drain_mode()) {
    w_drain_mode = false;
  }

  w_active = !w_pending.empty() || !w_data_fifo.empty();
  w_current = !w_pending.empty() ? w_pending.front() : RefWriteTransaction{};

  // ========== Read Channel Sequential Logic ==========

  // AR handshake: Start new read transaction
  if (io.ar.arvalid && io.ar.arready) {
    RefReadTransaction txn;
    txn.addr = io.ar.araddr;
    txn.id = io.ar.arid;
    txn.len = io.ar.arlen;
    txn.size = io.ar.arsize;
    txn.burst = io.ar.arburst;
    txn.beat_cnt = 0;
    txn.latency_cnt = 0;
    txn.in_data_phase = false;
    txn.complete = false;
    r_transactions.push_back(txn);
  }

  // R handshake: Advance data beat on selected transaction
  if (backend_read_grant && io.r.rvalid && io.r.rready && r_selected_idx >= 0) {
    RefReadTransaction &txn = r_transactions[r_selected_idx];
    txn.beat_cnt++;

    if (io.r.rlast) {
      txn.complete = true;
      r_active_idx = -1;
      r_rr_index =
          (static_cast<size_t>(r_selected_idx) + 1) %
          std::max(static_cast<size_t>(1), r_transactions.size());
    } else {
      r_active_idx = r_selected_idx;
    }
  }

  // Update all read transactions: increment latency, mark data phase
  for (auto &txn : r_transactions) {
    if (!txn.in_data_phase && !txn.complete) {
      txn.latency_cnt++;
      if (txn.latency_cnt >= REF_SIM_DDR_LATENCY) {
        txn.in_data_phase = true;
      }
    }
  }

  // Remove completed transactions while keeping the active index stable.
  size_t dst = 0;
  for (size_t src = 0; src < r_transactions.size(); ++src) {
    if (r_transactions[src].complete) {
      if (r_active_idx >= 0 && static_cast<int>(src) < r_active_idx) {
        r_active_idx--;
      }
      continue;
    }
    if (dst != src) {
      r_transactions[dst] = r_transactions[src];
    }
    dst++;
  }
  r_transactions.resize(dst);

  // Adjust rr_index if vector size changed
  if (!r_transactions.empty() && r_rr_index >= r_transactions.size()) {
    r_rr_index = 0;
  }
  if (r_active_idx >= static_cast<int>(r_transactions.size())) {
    r_active_idx = -1;
  }

  if (!backend_any_request_pending && backend_turnaround_cooldown == 0) {
    backend_last_service_mode = BackendServiceMode::None;
  } else if (backend_switch_pending) {
    backend_turnaround_cooldown =
        turnaround_cycles(backend_last_service_mode, backend_switch_target_mode);
    backend_last_service_mode = BackendServiceMode::None;
  } else if (backend_read_grant) {
    backend_last_service_mode = BackendServiceMode::Read;
  } else if (backend_write_grant) {
    backend_last_service_mode = BackendServiceMode::Write;
  }
}

// ============================================================================
// Helper Functions
// ============================================================================




} // namespace sim_ddr

namespace sim_ddr {
uint64_t SimDDRRef::digest() const {
  uint64_t h = 1469598103934665603ull;
  auto m = [&](uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; };
  m(w_pending.size());
  for (auto &t : w_pending) { m(t.id); m(t.len); m(t.beats_accepted); m(t.beats_drained); m(t.data_done); }
  m(w_data_fifo.size()); m(w_accept_cooldown); m(w_drain_cooldown); m(w_drain_mode);
  m(w_resp_queue.size());
  { auto q = w_resp_queue; while (!q.empty()) { m(q.front().id); m(q.front().latency_cnt); q.pop(); } }
  m(r_transactions.size());
  for (auto &t : r_transactions) { m(t.id); m(t.len); m(t.beat_cnt); m(t.latency_cnt); m(t.in_data_phase); m(t.complete); }
  m(r_rr_index); m(uint64_t(int64_t(r_active_idx)));
  m(uint64_t(backend_last_service_mode)); m(backend_turnaround_cooldown);
  return h;
}
}

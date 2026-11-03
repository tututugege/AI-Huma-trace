#pragma once
/**
 * @file SimDDRRef.h
 * @brief SimDDRRef - DDR Memory Simulator with AXI4 Interface
 *
 * This module simulates DDR memory with standard AXI4 interface.
 * It follows the simulator's comb/seq pattern for cycle-accurate behavior.
 *
 * Features:
 * - 5 AXI4 channels (AW, W, B, AR, R)
 * - Configurable memory latency
 * - Outstanding transaction support (multiple in-flight transactions)
 * - Burst-drain read service with burst-to-burst round-robin fairness
 * - Shared read/write backend service with configurable turnaround
 * - INCR burst mode support
 * - Uses external p_memory for storage (shared with main simulator)
 */

#include "SimDDR_IO.h"
#include "axi_interconnect_compat.h"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <queue>
#include <vector>

namespace sim_ddr {

// ============================================================================
// SimDDRRef Configuration
// ============================================================================
constexpr uint32_t REF_SIM_DDR_LATENCY = AXI_KIT_SIM_DDR_READ_LATENCY;
// Write response latency is modeled separately from read latency. Older
// simulator glue reused a frontend-side latency knob for the whole DDR path,
// but the current shared-AXI design benefits from controlling write completion
// independently when analyzing dirty-victim critical paths.
//
// Semantics:
// - The final W handshake enqueues a write response with latency_cnt = 0.
// - Because comb runs before seq, the earliest observable B response is the
//   next cycle.
// - This knob counts the additional full cycles to wait after enqueue before
//   B can first become visible:
//   - 0 => visible on the very next cycle
//   - 1 => one bubble cycle, then visible
//   - N => N bubble cycles, then visible
#ifndef AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY
#ifdef CONFIG_AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY
#define AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY \
  CONFIG_AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY
#else
#define AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY 1
#endif
#endif
constexpr uint32_t REF_SIM_DDR_WRITE_RESP_LATENCY =
    AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY;
constexpr uint32_t REF_SIM_DDR_MAX_BURST = 256; // Max burst length (AXI4 limit)
#ifndef AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#ifdef CONFIG_AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#define AXI_KIT_SIM_DDR_MAX_OUTSTANDING CONFIG_AXI_KIT_SIM_DDR_MAX_OUTSTANDING
#else
#define AXI_KIT_SIM_DDR_MAX_OUTSTANDING 8
#endif
#endif
constexpr uint32_t REF_SIM_DDR_MAX_OUTSTANDING =
    AXI_KIT_SIM_DDR_MAX_OUTSTANDING; // Max outstanding transactions

constexpr uint32_t REF_SIM_DDR_WRITE_QUEUE_DEPTH =
    AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH;

constexpr uint32_t REF_SIM_DDR_WRITE_ACCEPT_GAP =
    AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP;

constexpr uint32_t REF_SIM_DDR_WRITE_DATA_FIFO_DEPTH =
    AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH;

constexpr uint32_t REF_SIM_DDR_WRITE_DRAIN_GAP =
    AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP;

// When buffered W beats reach the high watermark, the controller enters a
// drain window and can keep WREADY low until occupancy falls back below the
// low watermark. This yields bursty write backpressure instead of a uniform
// per-beat throttle.
constexpr uint32_t REF_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK =
    AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK;

constexpr uint32_t REF_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK =
    AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK;

// Direction-switch turnaround is modeled between read-burst service windows
// and write-drain windows. Reads and writes can still queue addresses while the
// opposite direction owns the backend, but data service itself switches only
// after these cooldowns expire.
constexpr uint32_t REF_SIM_DDR_READ_TO_WRITE_TURNAROUND =
    AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND;

constexpr uint32_t REF_SIM_DDR_WRITE_TO_READ_TURNAROUND =
    AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND;

// ============================================================================
// Transaction Structures for Outstanding Support
// ============================================================================

// Write transaction (after AW handshake, waiting for W data or in latency)
struct RefWriteTransaction {
  uint32_t addr;
  uint8_t id;
  uint8_t len; // Burst length - 1
  uint8_t size;
  uint8_t burst;
  uint16_t beats_accepted;
  uint16_t beats_drained;
  bool data_done; // All W beats received
};

// Write response pending (in latency phase after W complete)
struct RefWriteRespPending {
  uint8_t id;
  uint32_t addr;
  // Number of full cycles elapsed since the last W beat enqueued the response.
  uint32_t latency_cnt;
};

struct RefWriteBeatPending {
  uint32_t addr;
  axi_data_t data;
  axi_strb_t wstrb;
};

// Read transaction (after AR handshake, in latency or sending data)
struct RefReadTransaction {
  uint32_t addr;
  uint8_t id;
  uint8_t len; // Burst length - 1
  uint8_t size;
  uint8_t burst;
  uint8_t beat_cnt; // Current beat sent
  uint32_t latency_cnt;
  bool in_data_phase; // True if latency done, sending data
  bool complete;      // True when all beats sent and rlast accepted
};

// ============================================================================
// SimDDRRef Class with Outstanding + Burst-Drain Read Service
// ============================================================================
class SimDDRRef {
public:
  // ========== Simulator Interface ==========
  void init();

  // Two-phase combinational logic for proper signal timing
  void comb_outputs(); // Phase 1: arready, rvalid, rdata, bvalid, bresp
  void comb_inputs();  // Phase 2: Process arvalid, awvalid, wvalid
  void comb() {
    comb_outputs();
    comb_inputs();
  } // Convenience wrapper

  void seq();
  bool idle_quiescent() const {
    return !w_active && w_pending.empty() && w_data_fifo.empty() &&
           w_resp_queue.empty() && r_transactions.empty() &&
           w_accept_cooldown == 0 && w_drain_cooldown == 0 && !w_drain_mode &&
           backend_turnaround_cooldown == 0 && !backend_switch_pending;
  }

  // ========== IO Ports ==========
  SimDDR_IO_t io;

  // ========== Debug ==========
  void print_state();
  uint64_t digest() const;

private:
  enum class BackendServiceMode : uint8_t { None = 0, Read = 1, Write = 2 };

  // ========== Write Channel ==========
  // Write address commands queue independently from write data buffering.
  bool w_active;
  RefWriteTransaction w_current;
  std::deque<RefWriteTransaction> w_pending;
  std::deque<RefWriteBeatPending> w_data_fifo;
  uint32_t w_accept_cooldown;
  uint32_t w_drain_cooldown;
  bool w_drain_mode;

  // Pending write responses (in latency)
  std::queue<RefWriteRespPending> w_resp_queue;

  // ========== Read Channel with Burst-Drain Service ==========
  // Vector allows access to any transaction while preserving issue order.
  std::vector<RefReadTransaction> r_transactions;

  // Round-robin index for fair burst-to-burst scheduling
  size_t r_rr_index;

  // Currently selected transaction index for this cycle (-1 if none)
  int r_selected_idx;

  // Active transaction whose burst is being drained.
  int r_active_idx;

  // Shared read/write backend arbitration state.
  BackendServiceMode backend_last_service_mode = BackendServiceMode::None;
  uint32_t backend_turnaround_cooldown = 0;
  bool backend_read_grant = false;
  bool backend_write_grant = false;
  bool backend_switch_pending = false;
  bool backend_any_request_pending = false;
  BackendServiceMode backend_switch_target_mode = BackendServiceMode::None;

  // ========== Combinational Logic Functions ==========
  void select_read_transaction();
  void update_backend_arbitration();
  void comb_write_channel();
  void comb_read_channel();

  // ========== Helper Functions ==========

  uint32_t turnaround_cycles(BackendServiceMode from,
                             BackendServiceMode to) const;
  int find_write_data_target() const;
  int find_write_drain_target() const;
  void retire_completed_writes();
  bool head_write_needs_drain() const;
  bool should_enter_write_drain_mode() const;
  bool should_keep_write_drain_mode() const;

  // Find next ready transaction using round-robin once the current burst finishes
  int find_next_ready_transaction();
};

} // namespace sim_ddr

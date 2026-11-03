#pragma once
/**
 * @file SimDDR.h
 * @brief Timing model of a DDR controller behind an AXI4 slave port.
 *
 * Reads: up to MAX_OUTSTANDING requests, each ready after SIM_DDR_LATENCY cycles; bursts drain contiguously and the next
 * burst is chosen round-robin. Writes: AW queue, a W beat FIFO that is drained in bursts (high/low watermark), and a
 * delayed B response. Reads and writes share the backend; switching direction costs a turnaround. No data is stored
 * (functional memory lives in the trace bridge).
 */

#include "SimDDR_IO.h"
#include "axi_interconnect_compat.h"
#include <cstdint>
#include <deque>
#include <vector>

namespace sim_ddr {

constexpr uint32_t SIM_DDR_LATENCY = AXI_KIT_SIM_DDR_READ_LATENCY;
// Extra full cycles between the last W beat and B becoming visible (0 = the very next cycle).
constexpr uint32_t SIM_DDR_WRITE_RESP_LATENCY = AXI_KIT_SIM_DDR_WRITE_RESP_LATENCY;
constexpr uint32_t SIM_DDR_MAX_OUTSTANDING = AXI_KIT_SIM_DDR_MAX_OUTSTANDING;
constexpr uint32_t SIM_DDR_WRITE_QUEUE_DEPTH = AXI_KIT_SIM_DDR_WRITE_QUEUE_DEPTH;
constexpr uint32_t SIM_DDR_WRITE_ACCEPT_GAP = AXI_KIT_SIM_DDR_WRITE_ACCEPT_GAP;
constexpr uint32_t SIM_DDR_WRITE_DATA_FIFO_DEPTH = AXI_KIT_SIM_DDR_WRITE_DATA_FIFO_DEPTH;
constexpr uint32_t SIM_DDR_WRITE_DRAIN_GAP = AXI_KIT_SIM_DDR_WRITE_DRAIN_GAP;
// Buffered W beats at the high watermark start a drain window (WREADY low) that lasts until occupancy is below the low one.
constexpr uint32_t SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK = AXI_KIT_SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK;
constexpr uint32_t SIM_DDR_WRITE_DRAIN_LOW_WATERMARK = AXI_KIT_SIM_DDR_WRITE_DRAIN_LOW_WATERMARK;
constexpr uint32_t SIM_DDR_READ_TO_WRITE_TURNAROUND = AXI_KIT_SIM_DDR_READ_TO_WRITE_TURNAROUND;
constexpr uint32_t SIM_DDR_WRITE_TO_READ_TURNAROUND = AXI_KIT_SIM_DDR_WRITE_TO_READ_TURNAROUND;

class SimDDR {
  friend struct DdrShadowAccess;  // lockstep test hook (DDR_SHADOW builds)
public:
  void init();
  void comb_outputs();  // ready/valid/response outputs from the registered state (run before the masters evaluate)
  void comb_inputs() {} // kept for symmetry with AXI_Interconnect: everything is decided in comb_outputs
  void comb() { comb_outputs(); comb_inputs(); }
  void seq();
  bool idle_quiescent() const {
    return pend_.empty() && fifo_ == 0 && resp_.empty() && rd_.empty() && accept_cd_ == 0 && drain_cd_ == 0 &&
           !drain_mode_ && turn_cd_ == 0 && !switch_pending_;
  }

  SimDDR_IO_t io;

private:
  enum Mode : uint8_t { None, Read, Write };
  struct WrTxn { uint16_t beats, accepted, drained; uint8_t id; bool data_done; };
  struct RdTxn { uint32_t lat; uint8_t id, len, beat; bool in_data, complete; };
  struct Resp { uint8_t id; uint32_t lat; };

  // write side: the data itself is not kept, only the beat count
  std::deque<WrTxn> pend_;
  uint32_t fifo_, accept_cd_, drain_cd_;
  bool drain_mode_;
  std::deque<Resp> resp_;
  // read side
  std::vector<RdTxn> rd_;
  size_t rr_;
  int sel_, active_;
  // shared backend
  Mode last_ = None, switch_target_ = None;
  uint32_t turn_cd_ = 0;
  bool read_grant_ = false, write_grant_ = false, switch_pending_ = false, any_pending_ = false;

  int first_where(bool (*pred)(const WrTxn &)) const {
    for (size_t i = 0; i < pend_.size(); i++) if (pred(pend_[i])) return int(i);
    return -1;
  }
  int data_target() const { return first_where([](const WrTxn &t) { return !t.data_done; }); }
  int drain_target() const { return first_where([](const WrTxn &t) { return t.drained < t.accepted; }); }
  bool head_needs_drain() const { return !pend_.empty() && pend_.front().data_done && pend_.front().drained < pend_.front().accepted; }
  bool keep_drain_mode() const { return fifo_ != 0 && (head_needs_drain() || fifo_ > SIM_DDR_WRITE_DRAIN_LOW_WATERMARK); }
  bool enter_drain_mode() const { return fifo_ != 0 && (head_needs_drain() || fifo_ >= SIM_DDR_WRITE_DRAIN_HIGH_WATERMARK); }
  static uint32_t turnaround(Mode from, Mode to) {
    return from == Read && to == Write ? SIM_DDR_READ_TO_WRITE_TURNAROUND : from == Write && to == Read ? SIM_DDR_WRITE_TO_READ_TURNAROUND : 0;
  }
  void arbitrate();
  void retire_writes();
};

} // namespace sim_ddr

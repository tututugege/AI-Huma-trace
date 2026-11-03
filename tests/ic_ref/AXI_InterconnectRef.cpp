/**
 * @file AXI_Interconnect.cpp
 * @brief AXI-Interconnect Layer Implementation
 *
 * AXI Protocol Compliance:
 * - AR/AW valid signals are latched until ready handshake
 * - Upstream req_valid can be deasserted without affecting AXI valid
 */

#include "AXI_InterconnectRef.h"
#include "axi_dual_port_route_shape.h"
#include "axi_mmio_map.h"
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <string>

extern long long sim_time;

#if defined(__GNUC__)
extern uint32_t pmem_read(uint32_t paddr) __attribute__((weak));
extern void pmem_write(uint32_t paddr, uint32_t data) __attribute__((weak));
#else
extern uint32_t pmem_read(uint32_t paddr);
extern void pmem_write(uint32_t paddr, uint32_t data);
#endif

namespace axi_interconnect {

namespace {
constexpr uint8_t kInvalidAxiReadId = 0xFF;
constexpr uint8_t kAxiIdMask = static_cast<uint8_t>(
    AXI_DUAL_PORT_AXI_ID_MASK_FOR_WIDTH(sim_ddr::AXI_ID_WIDTH));
constexpr uint8_t kDownstreamBeatBytes = sim_ddr::AXI_DATA_BYTES;
constexpr uint8_t kDownstreamBeatWords =
    kDownstreamBeatBytes / static_cast<uint8_t>(sizeof(uint32_t));
constexpr uint8_t kDownstreamAxiSize = sim_ddr::AXI_SIZE_CODE;
constexpr uint8_t kMmioAxiSize = AXI_DUAL_PORT_AXI_SIZE_32B;
constexpr uint8_t kMmioTransactionSize = AXI_DUAL_PORT_MMIO_TOTAL_SIZE;
constexpr uint8_t kUpstreamStrbBytes =
    (MAX_WRITE_TRANSACTION_BYTES + 7u) / 8u;
#ifndef AXI_KIT_DDR_BASE
#ifdef CONFIG_AXI_KIT_DDR_BASE
#define AXI_KIT_DDR_BASE CONFIG_AXI_KIT_DDR_BASE
#else
#define AXI_KIT_DDR_BASE 0u
#endif
#endif
constexpr uint32_t kDdrAddressBase = AXI_KIT_DDR_BASE;
constexpr uint32_t kDefaultMappedLlcOffset = 0x30000000u;

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


#ifndef SIM_DEBUG_PRINT_CYCLE_BEGIN
#define SIM_DEBUG_PRINT_CYCLE_BEGIN 0LL
#endif

#ifndef SIM_DEBUG_PRINT_CYCLE_END
#define SIM_DEBUG_PRINT_CYCLE_END 0LL
#endif

bool interconnect_debug_active(long long cycle) {
  return SIM_DEBUG_PRINT &&
         cycle >= static_cast<long long>(SIM_DEBUG_PRINT_CYCLE_BEGIN) &&
         cycle <= static_cast<long long>(SIM_DEBUG_PRINT_CYCLE_END);
}

bool llc_resp_trace_active() {
  return CONFIG_AXI_LLC_DEBUG_LOG != 0 &&
         CONFIG_AXI_LLC_RESP_TRACE_END >= CONFIG_AXI_LLC_RESP_TRACE_BEGIN &&
         sim_time >= static_cast<long long>(CONFIG_AXI_LLC_RESP_TRACE_BEGIN) &&
         sim_time <= static_cast<long long>(CONFIG_AXI_LLC_RESP_TRACE_END);
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

bool focus_read_line(uint32_t line_addr) { return llc_focus_line(line_addr); }

bool focus_write_line(uint32_t addr) {
  const uint32_t line_addr =
      addr & ~static_cast<uint32_t>(MAX_WRITE_TRANSACTION_BYTES - 1u);
  return llc_focus_line(line_addr);
}

bool mmio_request_supported(uint8_t total_size) {
  return axi_dual_port_mmio_request_supported(total_size);
}

bool downstream_request_supported(DownstreamPort port, uint8_t total_size) {
  return port != DownstreamPort::MMIO || mmio_request_supported(total_size);
}

bool unsupported_mmio_request(DownstreamPort port, uint8_t total_size) {
  return port == DownstreamPort::MMIO && !mmio_request_supported(total_size);
}

int port_index(DownstreamPort port) {
  return port == DownstreamPort::MMIO ? 1 : 0;
}

uint32_t read_backing_word_or_zero(uint32_t paddr) {
#if defined(__GNUC__)
  if (::pmem_read == nullptr) {
    return 0;
  }
#endif
  return ::pmem_read(paddr);
}

void write_backing_word_if_available(uint32_t paddr, uint32_t data) {
#if defined(__GNUC__)
  if (::pmem_write == nullptr) {
    return;
  }
#endif
  ::pmem_write(paddr, data);
}

WideReadData_t load_legacy_mmio_backing_line(uint32_t addr) {
  WideReadData_t data{};
  data.clear();
  for (uint32_t word = 0; word < MAX_READ_TRANSACTION_WORDS; ++word) {
    data[static_cast<int>(word)] =
        read_backing_word_or_zero(addr + word * sizeof(uint32_t));
  }
  return data;
}

void store_legacy_mmio_backing_line(uint32_t addr, const WideWriteData_t &wdata,
                                    const WideWriteStrb_t &wstrb,
                                    uint8_t total_size) {
  const uint32_t byte_count = static_cast<uint32_t>(total_size) + 1u;
  const uint32_t bounded_bytes =
      std::min<uint32_t>(byte_count, MAX_WRITE_TRANSACTION_BYTES);
  for (uint32_t byte = 0; byte < bounded_bytes; ++byte) {
    if (!wstrb.test(byte)) {
      continue;
    }
    const uint32_t word_idx = byte / sizeof(uint32_t);
    const uint32_t payload_shift = (byte % sizeof(uint32_t)) * 8u;
    const uint32_t byte_in_word = (addr + byte) % sizeof(uint32_t);
    const uint32_t word_addr = (addr + byte) & ~0x3u;
    const uint32_t shift = byte_in_word * 8u;
    const uint32_t mask = 0xFFu << shift;
    const uint32_t new_byte =
        (wdata[static_cast<int>(word_idx)] >> payload_shift) & 0xFFu;
    const uint32_t old_word = read_backing_word_or_zero(word_addr);
    write_backing_word_if_available(word_addr,
                                    (old_word & ~mask) | (new_byte << shift));
  }
}

bool focus_read_txn(const ReadPendingTxn &txn) {
  return focus_read_line(txn.addr);
}

bool trace_icache_read_txn(const ReadPendingTxn &txn, long long cycle) {
  return txn.master_id == MASTER_ICACHE && interconnect_debug_active(cycle);
}

bool trace_icache_llc_master(int master, long long cycle) {
  return master == MASTER_ICACHE && interconnect_debug_active(cycle);
}

uint32_t txn_word_count(const ReadPendingTxn &txn) {
  return std::min<uint32_t>(MAX_READ_TRANSACTION_WORDS,
                            static_cast<uint32_t>(txn.total_beats) *
                                kDownstreamBeatWords);
}

sim_ddr::axi_data_t pack_downstream_write_beat(const WideWriteData_t &wdata,
                                               uint8_t beat_idx) {
  sim_ddr::axi_data_t value{};
  const uint32_t base = static_cast<uint32_t>(beat_idx) * kDownstreamBeatWords;
  for (uint8_t word = 0; word < kDownstreamBeatWords; ++word) {
    const uint32_t idx = base + word;
    if (idx >= MAX_WRITE_TRANSACTION_WORDS) {
      break;
    }
    axi_compat::set_u32(value, word, wdata[idx]);
  }
  return value;
}

sim_ddr::axi_strb_t pack_downstream_write_strobe(const WideWriteStrb_t &wstrb,
                                                 uint8_t beat_idx) {
  sim_ddr::axi_strb_t mask{};
  const uint32_t first_byte =
      static_cast<uint32_t>(beat_idx) * kDownstreamBeatBytes;
  for (uint8_t byte = 0; byte < kDownstreamBeatBytes; ++byte) {
    if (wstrb.test(first_byte + byte)) {
      axi_compat::set_bit(mask, byte, true);
    }
  }
  return mask;
}

void unpack_downstream_read_beat(ReadPendingTxn &txn, sim_ddr::axi_data_t beat) {
  const uint32_t base =
      static_cast<uint32_t>(txn.beats_done) * kDownstreamBeatWords;
  for (uint8_t word = 0; word < kDownstreamBeatWords; ++word) {
    const uint32_t idx = base + word;
    if (idx >= MAX_READ_TRANSACTION_WORDS) {
      break;
    }
    txn.data[idx] = axi_compat::get_u32(beat, word);
  }
}

void clear_downstream_master_outputs(sim_ddr::SimDDR_IO_t &io) {
  io.ar.arvalid = false;
  io.ar.arid = 0;
  io.ar.araddr = 0;
  io.ar.arlen = 0;
  io.ar.arsize = kDownstreamAxiSize;
  io.ar.arburst = sim_ddr::AXI_BURST_INCR;
  io.r.rready = true;

  io.aw.awvalid = false;
  io.aw.awid = 0;
  io.aw.awaddr = 0;
  io.aw.awlen = 0;
  io.aw.awsize = kDownstreamAxiSize;
  io.aw.awburst = sim_ddr::AXI_BURST_INCR;
  io.w.wvalid = false;
  io.w.wdata = 0;
  io.w.wstrb = 0;
  io.w.wlast = false;
  io.b.bready = true;
}

void clear_downstream_slave_inputs(sim_ddr::SimDDR_IO_t &io) {
  io.ar.arready = false;
  io.aw.awready = false;
  io.w.wready = false;

  io.r.rvalid = false;
  io.r.rid = 0;
  io.r.rdata = {};
  io.r.rresp = sim_ddr::AXI_RESP_OKAY;
  io.r.rlast = false;

  io.b.bvalid = false;
  io.b.bid = 0;
  io.b.bresp = sim_ddr::AXI_RESP_OKAY;
}

uint8_t get_wide_read_byte(const WideReadData_t &data, uint32_t byte_idx) {
  const uint32_t word_idx = byte_idx / sizeof(uint32_t);
  const uint32_t byte_off = byte_idx % sizeof(uint32_t);
  if (word_idx >= MAX_READ_TRANSACTION_WORDS) {
    return 0;
  }
  return static_cast<uint8_t>((data[word_idx] >> (byte_off * 8u)) & 0xFFu);
}

void set_wide_read_byte(WideReadData_t &data, uint32_t byte_idx, uint8_t value) {
  const uint32_t word_idx = byte_idx / sizeof(uint32_t);
  const uint32_t byte_off = byte_idx % sizeof(uint32_t);
  if (word_idx >= MAX_READ_TRANSACTION_WORDS) {
    return;
  }
  const uint32_t shift = byte_off * 8u;
  const uint32_t mask = 0xFFu << shift;
  data[word_idx] =
      (data[word_idx] & ~mask) | (static_cast<uint32_t>(value) << shift);
}

uint8_t get_wide_write_byte(const WideWriteData_t &data, uint32_t byte_idx) {
  const uint32_t word_idx = byte_idx / sizeof(uint32_t);
  const uint32_t byte_off = byte_idx % sizeof(uint32_t);
  if (word_idx >= MAX_WRITE_TRANSACTION_WORDS) {
    return 0;
  }
  return static_cast<uint8_t>((data[word_idx] >> (byte_off * 8u)) & 0xFFu);
}

void set_wide_write_byte(WideWriteData_t &data, uint32_t byte_idx, uint8_t value) {
  const uint32_t word_idx = byte_idx / sizeof(uint32_t);
  const uint32_t byte_off = byte_idx % sizeof(uint32_t);
  if (word_idx >= MAX_WRITE_TRANSACTION_WORDS) {
    return;
  }
  const uint32_t shift = byte_off * 8u;
  const uint32_t mask = 0xFFu << shift;
  data[word_idx] =
      (data[word_idx] & ~mask) | (static_cast<uint32_t>(value) << shift);
}



WideReadData_t extract_aligned_downstream_read(const ReadPendingTxn &txn) {
  WideReadData_t out;
  out.clear();
  const uint32_t byte_off = txn.upstream_addr - txn.addr;
  for (uint32_t dst = 0; dst + byte_off < MAX_READ_TRANSACTION_BYTES; ++dst) {
    set_wide_read_byte(out, dst, get_wide_read_byte(txn.data, byte_off + dst));
  }
  return out;
}

WideWriteData_t align_mode2_downstream_write_data(const WideWriteData_t &src,
                                                  uint32_t addr,
                                                  uint32_t issued_addr) {
  WideWriteData_t out;
  out.clear();
  const uint32_t byte_off = addr - issued_addr;
  for (uint32_t byte = 0;
       byte < MAX_WRITE_TRANSACTION_BYTES && (byte + byte_off) < MAX_WRITE_TRANSACTION_BYTES;
       ++byte) {
    set_wide_write_byte(out, byte + byte_off, get_wide_write_byte(src, byte));
  }
  return out;
}

WideWriteStrb_t align_mode2_downstream_write_strobe(const WideWriteStrb_t &src,
                                                    uint32_t addr,
                                                    uint32_t issued_addr) {
  WideWriteStrb_t out;
  out.clear();
  const uint32_t byte_off = addr - issued_addr;
  for (uint32_t byte = 0;
       byte < MAX_WRITE_TRANSACTION_BYTES && (byte + byte_off) < MAX_WRITE_TRANSACTION_BYTES;
       ++byte) {
    if (src.test(byte)) {
      out.set(byte + byte_off, true);
    }
  }
  return out;
}

struct DownstreamReadIssue {
  DownstreamPort port = DownstreamPort::DDR;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  bool extract_from_aligned_beat = false;
};

struct DownstreamWriteIssue {
  DownstreamPort port = DownstreamPort::DDR;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  WideWriteData_t wdata{};
  WideWriteStrb_t wstrb{};
};

DownstreamReadIssue make_downstream_read_issue(DownstreamPort port,
                                               uint32_t addr,
                                               uint8_t total_size,
                                               uint32_t line_bytes,
                                               bool force_line_aligned) {
  DownstreamReadIssue out{};
  out.port = port;
  const AxiBridgeDownstreamIssueShape shape =
      axi_bridge_downstream_read_issue_shape(
          port == DownstreamPort::MMIO, addr, total_size,
          static_cast<uint8_t>(line_bytes), kDownstreamBeatBytes,
          force_line_aligned);
  out.addr = shape.issue_addr;
  out.total_size = shape.issue_size;
  out.extract_from_aligned_beat = shape.extract_from_aligned_beat;
  return out;
}

DownstreamWriteIssue make_downstream_write_issue(DownstreamPort port,
                                                 uint32_t addr,
                                                 uint8_t total_size,
                                                 const WideWriteData_t &wdata,
                                                 const WideWriteStrb_t &wstrb,
                                                 uint32_t line_bytes,
                                                 bool force_line_aligned) {
  DownstreamWriteIssue out{};
  out.port = port;
  const AxiBridgeDownstreamIssueShape shape =
      axi_bridge_downstream_write_issue_shape(
          port == DownstreamPort::MMIO, addr, total_size,
          static_cast<uint8_t>(line_bytes), kDownstreamBeatBytes,
          force_line_aligned);
  out.addr = shape.issue_addr;
  out.total_size = shape.issue_size;
  out.wdata = wdata;
  out.wstrb = wstrb;
  const uint16_t bytes = static_cast<uint16_t>(total_size) + 1u;
  if (port != DownstreamPort::MMIO &&
      (force_line_aligned || bytes <= kDownstreamBeatBytes)) {
    out.wdata = align_mode2_downstream_write_data(wdata, addr, out.addr);
    out.wstrb = align_mode2_downstream_write_strobe(wstrb, addr, out.addr);
  }
  return out;
}

void dump_focus_read_txn(const char *tag, long long cyc,
                         const ReadPendingTxn &txn) {
  if (!focus_read_txn(txn) && !trace_icache_read_txn(txn, cyc)) {
    return;
  }

  for (uint32_t word = 0; word < txn_word_count(txn); ++word) {

  }

}

void dump_focus_read_words(const char *tag, long long cyc, const ReadPendingTxn &txn) {
  if (!txn.to_llc ||
      (!llc_focus_line(txn.addr) && !trace_icache_read_txn(txn, cyc))) {
    return;
  }

}

inline bool write_size_supported(uint8_t total_size) {
  return (static_cast<uint16_t>(total_size) + 1u) <=
         MAX_WRITE_TRANSACTION_BYTES;
}
constexpr uint8_t kInvalidAxiWriteId = 0xFF;

bool read_env_u64(const char *name, uint64_t &value_out) {
  const char *raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0') {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  const unsigned long long parsed = std::strtoull(raw, &end, 0);
  if (errno != 0 || end == raw || (end != nullptr && *end != '\0')) {

    return false;
  }
  value_out = static_cast<uint64_t>(parsed);
  return true;
}
} // namespace





// ============================================================================
// Initialization
// ============================================================================
bool AXI_InterconnectRef::llc_enabled() const {
  const bool mode_uses_llc = runtime_mode_ != 0u;
  return mode_uses_llc && llc_config.enable && llc_config.valid();
}

sim_ddr::SimDDR_IO_t &AXI_InterconnectRef::downstream_io(DownstreamPort port) {
  return port == DownstreamPort::DDR ? axi_ddr_io : axi_mmio_io;
}

const sim_ddr::SimDDR_IO_t &
AXI_InterconnectRef::downstream_io(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? axi_ddr_io : axi_mmio_io;
}

ARLatch_t &AXI_InterconnectRef::ar_latch(DownstreamPort port) {
  return port == DownstreamPort::DDR ? ar_latched : ar_latched_mmio;
}


AWLatch_t &AXI_InterconnectRef::aw_latch(DownstreamPort port) {
  return port == DownstreamPort::DDR ? aw_latched : aw_latched_mmio;
}

const AWLatch_t &AXI_InterconnectRef::aw_latch(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? aw_latched : aw_latched_mmio;
}

bool &AXI_InterconnectRef::w_active_ref(DownstreamPort port) {
  return port == DownstreamPort::DDR ? w_active : w_active_mmio;
}

bool AXI_InterconnectRef::w_active_ref(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? w_active : w_active_mmio;
}

WritePendingTxn &AXI_InterconnectRef::w_current_ref(DownstreamPort port) {
  return port == DownstreamPort::DDR ? w_current : w_current_mmio;
}

const WritePendingTxn &
AXI_InterconnectRef::w_current_ref(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? w_current : w_current_mmio;
}

int &AXI_InterconnectRef::w_current_master_ref(DownstreamPort port) {
  return port == DownstreamPort::DDR ? w_current_master : w_current_master_mmio;
}

bool AXI_InterconnectRef::any_ar_latched() const {
  return ar_latched.valid || ar_latched_mmio.valid;
}

bool AXI_InterconnectRef::any_aw_latched() const {
  return aw_latched.valid || aw_latched_mmio.valid;
}

bool AXI_InterconnectRef::any_w_active() const {
  return w_active || w_active_mmio;
}

bool AXI_InterconnectRef::external_write_busy() const {
  return any_w_active() || any_aw_latched();
}

uint8_t AXI_InterconnectRef::write_resp_buffer_count(uint8_t master) const {
  if (master >= NUM_WRITE_MASTERS) {
    return 0;
  }
  return static_cast<uint8_t>((w_resp_valid[master] ? 1u : 0u) +
                              w_resp_queue[master].size());
}

bool AXI_InterconnectRef::write_resp_buffer_has_space(uint8_t master) const {
  return master < NUM_WRITE_MASTERS &&
         write_resp_buffer_count(master) < MAX_WRITE_OUTSTANDING;
}

bool AXI_InterconnectRef::any_write_resp_buffered() const {
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (w_resp_valid[master] || !w_resp_queue[master].empty()) {
      return true;
    }
  }
  return false;
}

void AXI_InterconnectRef::enqueue_write_resp(uint8_t master, uint8_t id,
                                          uint8_t resp, bool line_valid,
                                          uint32_t line_addr) {
  if (master >= NUM_WRITE_MASTERS) {
    return;
  }
  if (!w_resp_valid[master]) {
    w_resp_valid[master] = true;
    w_resp_id[master] = id;
    w_resp_resp[master] = resp;
    return;
  }
  if (write_resp_buffer_has_space(master)) {
    w_resp_queue[master].push_back(
        WriteRespEntry{id, resp, line_valid, line_addr});
  }
}

void AXI_InterconnectRef::promote_write_resp_queue() {
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (!w_resp_valid[master] && !w_resp_queue[master].empty()) {
      const auto entry = w_resp_queue[master].front();
      w_resp_queue[master].pop_front();
      w_resp_valid[master] = true;
      w_resp_id[master] = entry.id;
      w_resp_resp[master] = entry.resp;
    }
  }
}

void AXI_InterconnectRef::sample_runtime_controls() {
  uint64_t env_value = 0;
  if (read_env_u64("AXI_SUBMODULE_MODE", env_value)) {
    mode = static_cast<uint8_t>(env_value & 0x3u);
  }
  llc_mapped_offset_override_ = false;
  if (read_env_u64("AXI_SUBMODULE_OFFSET", env_value)) {
    llc_mapped_offset = static_cast<uint32_t>(env_value);
    llc_mapped_offset_override_ = true;
  }

  runtime_mode_ = requested_mode();
  llc_mapped_offset_ = requested_llc_mapped_offset();
  reconfig_pending_ = false;
  reconfig_target_mode_ = runtime_mode_;
  reconfig_target_offset_ = llc_mapped_offset_;



}

uint8_t AXI_InterconnectRef::requested_mode() const {
  const uint8_t encoded = static_cast<uint8_t>(mode) & 0x3u;
  return encoded == 3u ? 0u : encoded;
}

uint32_t AXI_InterconnectRef::requested_llc_mapped_offset() const {
  const uint32_t requested = static_cast<uint32_t>(llc_mapped_offset);
  return (!llc_mapped_offset_override_ && requested == 0) ? kDefaultMappedLlcOffset
                                                         : requested;
}

bool AXI_InterconnectRef::requested_config_differs_from_active() const {
  const uint8_t req_mode = requested_mode();
  const uint32_t req_offset = requested_llc_mapped_offset();
  return req_mode != runtime_mode_ ||
         (req_mode == 2u && req_offset != llc_mapped_offset_);
}

bool AXI_InterconnectRef::mode_transition_needs_flush() const {
  return reconfig_pending_ || requested_config_differs_from_active();
}

bool AXI_InterconnectRef::mode_transition_invalidate_requested() const {
  return reconfig_pending_ && requested_config_differs_from_active() &&
         reconfig_path_quiescent();
}

bool AXI_InterconnectRef::invalidate_all_requested() const {
  return llc_invalidate_all_req_ || mode_transition_invalidate_requested();
}

bool AXI_InterconnectRef::request_in_mapped_window(uint32_t addr,
                                                uint8_t total_size) const {
  if (runtime_mode_ != 2u) {
    return false;
  }
  const uint64_t start = static_cast<uint64_t>(addr);
  const uint64_t end = start + static_cast<uint64_t>(total_size) + 1u;
  const uint64_t base = static_cast<uint64_t>(llc_mapped_offset_);
  const uint64_t limit = base + static_cast<uint64_t>(kMappedLlcWindowBytes);
  return start >= base && end <= limit;
}

bool AXI_InterconnectRef::request_uses_direct_mapped_llc(uint32_t addr,
                                                      uint8_t total_size) const {
  return runtime_mode_ == 2u && request_in_mapped_window(addr, total_size);
}

DownstreamPort AXI_InterconnectRef::classify_downstream_port(
    uint32_t addr, uint8_t total_size) const {
  const AxiDualPortRouteSupport route =
      axi_dual_port_route_support(addr, total_size, kDdrAddressBase);
  if (route.ddr_port) {
    return DownstreamPort::DDR;
  }
  return DownstreamPort::MMIO;
}

bool AXI_InterconnectRef::request_uses_ddr_port(uint32_t addr,
                                             uint8_t total_size) const {
  if (request_uses_direct_mapped_llc(addr, total_size)) {
    return false;
  }
  return classify_downstream_port(addr, total_size) == DownstreamPort::DDR;
}

bool AXI_InterconnectRef::request_uses_mmio_port(uint32_t addr,
                                              uint8_t total_size) const {
  if (request_uses_direct_mapped_llc(addr, total_size)) {
    return false;
  }
  return classify_downstream_port(addr, total_size) == DownstreamPort::MMIO;
}

uint32_t AXI_InterconnectRef::translate_llc_addr(uint32_t addr,
                                              uint8_t total_size) const {
  if (!request_uses_direct_mapped_llc(addr, total_size)) {
    return addr;
  }
  return static_cast<uint32_t>(
      static_cast<uint64_t>(addr) - static_cast<uint64_t>(llc_mapped_offset_));
}

bool AXI_InterconnectRef::effective_llc_bypass(uint32_t addr, uint8_t total_size,
                                            bool upstream_bypass) const {
  if (request_uses_mmio_port(addr, total_size)) {
    return true;
  }
  if (runtime_mode_ == 2u) {
    return !request_in_mapped_window(addr, total_size);
  }
  if (runtime_mode_ == 0u) {
    return true;
  }
  if (runtime_mode_ == 1u) {
    return upstream_bypass;
  }
  return true;
}

void AXI_InterconnectRef::init() {
  sample_runtime_controls();
  llc.set_config(llc_config);
  llc.reset();
  r_arb_rr_idx = 0;
  r_current_master = -1;
  r_pending.clear();
  direct_read_completion_seq_ = 0;
  for (int i = 0; i < NUM_READ_MASTERS; ++i) {
    read_req_hold[i] = {};
    llc_upstream_req[i] = {};
    llc_upstream_capture_c[i] = {};
    llc_upstream_accept_c[i] = false;
    llc_read_resp_valid_[i] = false;
    llc_read_resp_data_[i].clear();
    llc_read_resp_id_[i] = 0;
  }
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
    llc_upstream_write_req[i] = {};
    llc_upstream_write_capture_c[i] = {};
    llc_upstream_write_accept_c[i] = false;
    llc_upstream_write_q[i].clear();
  }
  llc_core_req_stage_ = {};
  llc_core_dispatch_rr_ = 0;
  llc_mem_write_resp_valid_ = false;
  llc_mem_write_resp_ = 0;
  llc_mem_ignored_b_count_ = 0;
  llc_synth_read_resp_valid_ = false;
  llc_synth_read_resp_id_ = 0;
  llc_synth_read_resp_data_.clear();
  llc_mem_read_issue_seen_valid_ = false;
  llc_mem_read_issue_seen_id_ = 0;
  llc_mem_read_issue_seen_addr_ = 0;
  llc_mem_read_issue_seen_size_ = 0;

  auto clear_ar_latch = [](ARLatch_t &latch, DownstreamPort port) {
    latch.valid = false;
    latch.accepted_upstream = false;
    latch.to_llc = false;
    latch.resp_extract_from_aligned_beat = false;
    latch.port = port;
    latch.addr = 0;
    latch.upstream_addr = 0;
    latch.len = 0;
    latch.size = port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize;
    latch.burst = sim_ddr::AXI_BURST_INCR;
    latch.id = 0;
    latch.master_id = 0;
    latch.orig_id = 0;
    latch.upstream_total_size = 0;
  };
  clear_ar_latch(ar_latched, DownstreamPort::DDR);
  clear_ar_latch(ar_latched_mmio, DownstreamPort::MMIO);

  w_active = false;
  w_active_mmio = false;
  w_current = {};
  w_current_mmio = {};
  w_pending.clear();
  w_arb_rr_idx = 0;
  w_current_master = -1;
  w_current_master_mmio = -1;

  auto clear_aw_latch = [](AWLatch_t &latch, DownstreamPort port) {
    latch.valid = false;
    latch.port = port;
    latch.addr = 0;
    latch.len = 0;
    latch.size = port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize;
    latch.burst = sim_ddr::AXI_BURST_INCR;
    latch.id = 0;
  };
  clear_aw_latch(aw_latched, DownstreamPort::DDR);
  clear_aw_latch(aw_latched_mmio, DownstreamPort::MMIO);

  // Clear registered req.ready signals
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    req_ready_r[i] = false;
    req_drop_warned[i] = false;
  }

  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    read_ports[i].req.ready = false;
    read_ports[i].req.accepted = false;
    read_ports[i].req.accepted_id = 0;
    read_ports[i].resp.valid = false;
    read_ports[i].resp.data.clear();
    read_ports[i].resp.id = 0;
    read_req_accepted[i] = false;
    read_req_accepted_id[i] = 0;
  }
  for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
    write_ports[i].req.ready = false;
    write_ports[i].req.accepted = false;
    write_ports[i].resp.valid = false;
    write_ports[i].resp.id = 0;
    write_ports[i].resp.resp = 0;
    w_req_ready_r[i] = false;
    write_req_fire_c[i] = false;
    w_resp_valid[i] = false;
    w_resp_id[i] = 0;
    w_resp_resp[i] = 0;
    w_resp_queue[i].clear();
    llc_write_resp_delay[i].clear();
    write_req_accepted[i] = false;
  }

  clear_downstream_master_outputs(axi_ddr_io);
  clear_downstream_master_outputs(axi_mmio_io);
  clear_downstream_slave_inputs(axi_ddr_io);
  clear_downstream_slave_inputs(axi_mmio_io);
  for (uint8_t byte = 0; byte < kDownstreamBeatBytes; ++byte) {
    axi_compat::set_bit(axi_ddr_io.w.wstrb, byte, true);
    axi_compat::set_bit(axi_mmio_io.w.wstrb, byte, true);
  }
  ar_port_c = DownstreamPort::DDR;
  ar_issue_c[0] = {};
  ar_issue_c[1] = {};
  aw_port_c = DownstreamPort::DDR;
}

uint8_t AXI_InterconnectRef::alloc_read_axi_id() const {
  bool used[1u << sim_ddr::AXI_ID_WIDTH] = {false};
  for (const auto &txn : r_pending) {
    used[txn.axi_id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  if (ar_latched.valid) {
    used[ar_latched.id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  if (ar_latched_mmio.valid) {
    used[ar_latched_mmio.id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  for (uint8_t id = 0; id < (1u << sim_ddr::AXI_ID_WIDTH); ++id) {
    if (!used[id]) {
      return id;
    }
  }
  return kInvalidAxiReadId;
}

uint8_t AXI_InterconnectRef::count_master_read_pending(uint8_t master_id) const {
  uint8_t count = 0;
  for (const auto &txn : r_pending) {
    if (txn.master_id == master_id) {
      ++count;
    }
  }
  if (ar_latched.valid && ar_latched.master_id == master_id) {
    ++count;
  }
  if (ar_latched_mmio.valid && ar_latched_mmio.master_id == master_id) {
    ++count;
  }
  return count;
}

uint8_t AXI_InterconnectRef::count_total_read_inflight() const {
  return static_cast<uint8_t>(r_pending.size() + (ar_latched.valid ? 1 : 0) +
                              (ar_latched_mmio.valid ? 1 : 0));
}

uint8_t AXI_InterconnectRef::alloc_write_axi_id() const {
  bool used[1u << sim_ddr::AXI_ID_WIDTH] = {false};
  for (const auto &txn : w_pending) {
    used[txn.axi_id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  if (w_active) {
    used[w_current.axi_id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  if (w_active_mmio) {
    used[w_current_mmio.axi_id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  if (aw_latched.valid) {
    used[aw_latched.id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  if (aw_latched_mmio.valid) {
    used[aw_latched_mmio.id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  for (uint8_t id = 0; id < (1u << sim_ddr::AXI_ID_WIDTH); ++id) {
    if (!used[id]) {
      return id;
    }
  }
  return kInvalidAxiWriteId;
}

uint8_t AXI_InterconnectRef::alloc_write_axi_id(DownstreamPort port) const {
  bool used[1u << sim_ddr::AXI_ID_WIDTH] = {false};
  for (const auto &txn : w_pending) {
    if (txn.port == port) {
      used[txn.axi_id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
    }
  }
  if (w_active_ref(port)) {
    used[w_current_ref(port).axi_id &
         ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  if (aw_latch(port).valid) {
    used[aw_latch(port).id & ((1u << sim_ddr::AXI_ID_WIDTH) - 1u)] = true;
  }
  for (uint8_t id = 0; id < (1u << sim_ddr::AXI_ID_WIDTH); ++id) {
    if (!used[id]) {
      return id;
    }
  }
  return kInvalidAxiWriteId;
}

bool AXI_InterconnectRef::can_accept_write_now() const {
  return !llc_enabled() &&
         w_pending.size() < MAX_WRITE_OUTSTANDING &&
         alloc_write_axi_id() != kInvalidAxiWriteId;
}

uint32_t AXI_InterconnectRef::count_llc_write_pending() const {
  uint32_t count = 0;
  count += (llc_core_req_stage_.valid && llc_core_req_stage_.is_write) ? 1u : 0u;
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
    count += llc_upstream_write_req[i].valid ? 1u : 0u;
    count += static_cast<uint32_t>(llc_upstream_write_q[i].size());
  }
  return count;
}

bool AXI_InterconnectRef::non_llc_path_quiescent() const {
  if (any_ar_latched() || !r_pending.empty() || any_aw_latched() ||
      any_w_active() ||
      !w_pending.empty()) {
    return false;
  }
  for (int i = 0; i < NUM_READ_MASTERS; ++i) {
    if (req_ready_r[i] || read_req_hold[i].valid) {
      return false;
    }
  }
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
    if (w_req_ready_r[i] || write_ports[i].req.ready) {
      return false;
    }
  }
  return true;
}

bool AXI_InterconnectRef::llc_path_quiescent() const {
  if (!llc_enabled()) {
    return true;
  }
  if (any_ar_latched() || !r_pending.empty()) {
    return false;
  }
  if (any_w_active() || any_aw_latched() || llc_mem_write_resp_valid_ ||
      llc_mem_ignored_b_count_ != 0 || !w_pending.empty()) {
    return false;
  }
  for (int master = 0; master < NUM_READ_MASTERS; ++master) {
    if (req_ready_r[master] || read_req_hold[master].valid) {
      return false;
    }
  }
  for (int master = 0; master < NUM_READ_MASTERS; ++master) {
    if (llc_read_resp_valid_[master]) {
      return false;
    }
  }
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (w_req_ready_r[master] || write_ports[master].req.ready ||
        w_resp_valid[master] || !w_resp_queue[master].empty()) {
      return false;
    }
  }
  if (llc_core_req_stage_.valid) {
    return false;
  }
  for (int master = 0; master < NUM_READ_MASTERS; ++master) {
    if (llc_upstream_req[master].valid || llc.io.regs.read_resp_valid_r[master] ||
        llc.io.regs.read_resp_q_count_r[master] != 0) {
      return false;
    }
  }
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (llc_upstream_write_req[master].valid ||
        !llc_upstream_write_q[master].empty()) {
      return false;
    }
    if (llc.io.regs.write_ctx[master].valid ||
        llc.io.regs.write_q_count_r[master] != 0 ||
        llc.io.regs.write_resp_valid_r[master]) {
      return false;
    }
  }
  if (llc.io.regs.lookup_valid_r || llc.io.regs.victim_wb_valid_r ||
      llc.io.regs.read_victim_wb_q_count_r != 0 ||
      llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.write_req_valid) {
    return false;
  }
  for (uint32_t slot = 0; slot < llc_config.mshr_num && slot < MAX_OUTSTANDING;
       ++slot) {
    if (llc.io.regs.mshr[slot].valid) {
      return false;
    }
  }
  return true;
}

bool AXI_InterconnectRef::idle_quiescent() const {
  if (!reconfig_path_quiescent() || reconfig_pending_) {
    return false;
  }
  for (int i = 0; i < NUM_READ_MASTERS; ++i) {
    if (read_ports[i].req.valid || read_ports[i].resp.valid) {
      return false;
    }
  }
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
    if (write_ports[i].req.valid || write_ports[i].resp.valid ||
        !llc_write_resp_delay[i].empty() || !w_resp_queue[i].empty()) {
      return false;
    }
  }
  return true;
}

bool AXI_InterconnectRef::reconfig_path_quiescent() const {
  return llc_enabled() ? llc_path_quiescent() : non_llc_path_quiescent();
}

bool AXI_InterconnectRef::has_same_line_write_hazard(uint32_t line_addr) const {
  if (!llc_enabled()) {
    return false;
  }
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (llc_upstream_write_req[master].valid &&
        AXI_LLC::line_addr(llc_config, llc_upstream_write_req[master].addr) ==
            line_addr) {
      return true;
    }
    for (const auto &entry : llc_upstream_write_q[master]) {
      if (entry.valid && AXI_LLC::line_addr(llc_config, entry.addr) == line_addr) {
        return true;
      }
    }
    for (const auto &entry : llc_write_resp_delay[master]) {
      if (entry.entry.line_valid && entry.entry.line_addr == line_addr) {
        return true;
      }
    }
    for (const auto &entry : w_resp_queue[master]) {
      if (entry.line_valid && entry.line_addr == line_addr) {
        return true;
      }
    }
    if (write_ports[master].req.valid &&
        AXI_LLC::line_addr(llc_config, write_ports[master].req.addr) == line_addr) {
      return true;
    }
  }
  if (llc_core_req_stage_.valid && llc_core_req_stage_.is_write &&
      AXI_LLC::line_addr(llc_config, llc_core_req_stage_.write.addr) == line_addr) {
    return true;
  }
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    const auto &ctx = llc.io.regs.write_ctx[master];
    if (ctx.valid && ctx.line_addr == line_addr) {
      return true;
    }
    if (llc.io.regs.write_resp_valid_r[master] &&
        llc.io.regs.write_resp_line_addr_r[master] == line_addr) {
      return true;
    }
  }
  for (uint8_t master = 0; master < NUM_WRITE_MASTERS; ++master) {
    for (uint32_t i = 0;
         i < llc.io.regs.write_q_count_r[master] && i < MAX_WRITE_OUTSTANDING; ++i) {
      const uint32_t slot =
          (llc.io.regs.write_q_head_r[master] + i) % MAX_WRITE_OUTSTANDING;
      const auto &entry = llc.io.regs.write_q[master][slot];
      if (!entry.valid) {
        continue;
      }
      if (AXI_LLC::line_addr(llc_config, entry.addr) == line_addr) {
        return true;
      }
    }
  }
  if (w_active && AXI_LLC::line_addr(llc_config, w_current.addr) == line_addr) {
    return true;
  }
  if (w_active_mmio &&
      AXI_LLC::line_addr(llc_config, w_current_mmio.addr) == line_addr) {
    return true;
  }
  if (aw_latched.valid &&
      AXI_LLC::line_addr(llc_config, aw_latched.addr) == line_addr) {
    return true;
  }
  if (aw_latched_mmio.valid &&
      AXI_LLC::line_addr(llc_config, aw_latched_mmio.addr) == line_addr) {
    return true;
  }
  return false;
}

bool AXI_InterconnectRef::has_same_line_read_hazard(uint32_t line_addr) const {
  if (!llc_enabled()) {
    return false;
  }

  auto request_hits_line = [&](uint32_t addr, uint8_t total_size) {
    return AXI_LLC::line_addr(llc_config, translate_llc_addr(addr, total_size)) ==
           line_addr;
  };

  for (int master = 0; master < NUM_READ_MASTERS; ++master) {
    if (llc_upstream_req[master].valid &&
        request_hits_line(llc_upstream_req[master].addr,
                          llc_upstream_req[master].total_size)) {
      return true;
    }
    if (read_req_hold[master].valid &&
        !request_uses_mmio_port(read_req_hold[master].addr,
                                read_req_hold[master].total_size) &&
        request_hits_line(read_req_hold[master].addr,
                          read_req_hold[master].total_size)) {
      return true;
    }
    if (read_ports[master].req.valid) {
      const uint32_t addr = static_cast<uint32_t>(read_ports[master].req.addr);
      const uint8_t total_size =
          static_cast<uint8_t>(read_ports[master].req.total_size);
      if (!request_uses_mmio_port(addr, total_size) &&
          request_hits_line(addr, total_size)) {
        return true;
      }
    }
  }

  if (llc_core_req_stage_.valid && !llc_core_req_stage_.is_write &&
      request_hits_line(llc_core_req_stage_.read.addr,
                        llc_core_req_stage_.read.total_size)) {
    return true;
  }

  if (llc.io.regs.lookup_valid_r && !llc.io.regs.lookup_is_write_r &&
      AXI_LLC::line_addr(llc_config, llc.io.regs.lookup_addr_r) == line_addr) {
    return true;
  }
  if (llc.io.ext_out.mem.read_req_valid &&
      AXI_LLC::line_addr(llc_config, llc.io.ext_out.mem.read_req_addr) ==
          line_addr) {
    return true;
  }

  for (uint32_t slot = 0; slot < llc_config.mshr_num && slot < MAX_OUTSTANDING;
       ++slot) {
    const auto &entry = llc.io.regs.mshr[slot];
    if (entry.valid && !entry.is_write && entry.line_addr == line_addr) {
      return true;
    }
  }

  if (ar_latched.valid && ar_latched.to_llc &&
      AXI_LLC::line_addr(llc_config, ar_latched.upstream_addr) == line_addr) {
    return true;
  }
  if (ar_latched_mmio.valid && ar_latched_mmio.to_llc &&
      AXI_LLC::line_addr(llc_config, ar_latched_mmio.upstream_addr) ==
          line_addr) {
    return true;
  }
  for (const auto &txn : r_pending) {
    if (txn.to_llc && txn.beats_done < txn.total_beats &&
        AXI_LLC::line_addr(llc_config, txn.upstream_addr) == line_addr) {
      return true;
    }
  }

  return false;
}

uint32_t AXI_InterconnectRef::external_hazard_line_addr(uint32_t addr) const {
  constexpr uint32_t kHazardBytes = MAX_WRITE_TRANSACTION_BYTES;
  static_assert((kHazardBytes & (kHazardBytes - 1u)) == 0,
                "external hazard granule must be a power of two");
  return addr & ~(kHazardBytes - 1u);
}

bool AXI_InterconnectRef::has_external_pending_read_hazard(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    const auto &io = downstream_io(port);
    const auto &meta = ar_issue_c[port_index(port)];
    if (io.ar.arvalid && meta.valid &&
        external_hazard_line_addr(meta.upstream_addr) == line) {
      return true;
    }
  }
  if (ar_latched.valid &&
      external_hazard_line_addr(ar_latched.upstream_addr) == line) {
    return true;
  }
  if (ar_latched_mmio.valid &&
      external_hazard_line_addr(ar_latched_mmio.upstream_addr) == line) {
    return true;
  }
  for (const auto &txn : r_pending) {
    if (txn.beats_done < txn.total_beats &&
        external_hazard_line_addr(txn.upstream_addr) == line) {
      return true;
    }
  }
  return false;
}

bool AXI_InterconnectRef::has_external_pending_write_hazard(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  if (aw_latched.valid && external_hazard_line_addr(aw_latched.addr) == line) {
    return true;
  }
  if (aw_latched_mmio.valid &&
      external_hazard_line_addr(aw_latched_mmio.addr) == line) {
    return true;
  }
  if (w_active && external_hazard_line_addr(w_current.addr) == line) {
    return true;
  }
  if (w_active_mmio && external_hazard_line_addr(w_current_mmio.addr) == line) {
    return true;
  }
  for (const auto &txn : w_pending) {
    if (external_hazard_line_addr(txn.addr) == line) {
      return true;
    }
  }
  return false;
}

bool AXI_InterconnectRef::has_direct_read_response(uint8_t master_id) const {
  for (const auto &txn : r_pending) {
    if (!txn.to_llc && txn.master_id == master_id &&
        txn.beats_done == txn.total_beats) {
      return true;
    }
  }
  return false;
}

bool AXI_InterconnectRef::can_issue_external_read(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  const AxiDualPortIssueGateResult gate = axi_dual_port_issue_gate(
      true, true, line, false, has_external_pending_write_hazard(addr), false,
      true, 0, false, false);
  return gate.axi_arvalid;
}

bool AXI_InterconnectRef::can_issue_external_write(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  const AxiDualPortIssueGateResult gate = axi_dual_port_issue_gate(
      false, true, 0, false, false, true, true, line, false,
      has_external_pending_read_hazard(addr));
  return gate.axi_awvalid;
}

int AXI_InterconnectRef::find_write_pending_by_axi_id(DownstreamPort port,
                                                   uint8_t axi_id) const {
  for (size_t i = 0; i < w_pending.size(); ++i) {
    if (w_pending[i].port == port && w_pending[i].axi_id == axi_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_InterconnectRef::find_next_aw_pending() const {
  for (size_t i = 0; i < w_pending.size(); ++i) {
    if (!w_pending[i].aw_done) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_InterconnectRef::find_next_aw_pending(DownstreamPort port) const {
  for (size_t i = 0; i < w_pending.size(); ++i) {
    if (!w_pending[i].aw_done && w_pending[i].port == port) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_InterconnectRef::find_next_w_pending() const {
  for (size_t i = 0; i < w_pending.size(); ++i) {
    if (w_pending[i].aw_done && !w_pending[i].w_done) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int AXI_InterconnectRef::find_next_w_pending(DownstreamPort port) const {
  for (size_t i = 0; i < w_pending.size(); ++i) {
    if (w_pending[i].port == port && w_pending[i].aw_done && !w_pending[i].w_done) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void AXI_InterconnectRef::refresh_non_llc_w_active(DownstreamPort port) {
  if (llc_enabled()) {
    return;
  }
  bool &active = w_active_ref(port);
  auto &current = w_current_ref(port);
  int &current_master = w_current_master_ref(port);
  if (active) {
    return;
  }
  const int idx = find_next_w_pending(port);
  if (idx < 0) {
    current = {};
    current_master = -1;
    return;
  }
  current = w_pending[static_cast<size_t>(idx)];
  active = true;
  current_master = current.master_id;
}

bool AXI_InterconnectRef::can_accept_read_master(uint8_t master_id) const {
  if (master_id >= NUM_READ_MASTERS) {
    return false;
  }
  if (count_total_read_inflight() >= MAX_OUTSTANDING) {
    return false;
  }
  if (count_master_read_pending(master_id) >= MAX_READ_OUTSTANDING_PER_MASTER) {
    return false;
  }
  return alloc_read_axi_id() != kInvalidAxiReadId;
}

bool AXI_InterconnectRef::has_read_id_conflict(uint8_t master_id,
                                            uint8_t orig_id) const {
  if (ar_latched.valid && !ar_latched.to_llc &&
      ar_latched.master_id == master_id && ar_latched.orig_id == orig_id) {
    return true;
  }
  if (ar_latched_mmio.valid && !ar_latched_mmio.to_llc &&
      ar_latched_mmio.master_id == master_id &&
      ar_latched_mmio.orig_id == orig_id) {
    return true;
  }
  for (const auto &txn : r_pending) {
    if (txn.master_id == master_id && txn.orig_id == orig_id) {
      return true;
    }
  }
  if (llc_enabled()) {
    if (llc_core_req_stage_.valid && !llc_core_req_stage_.is_write &&
        llc_core_req_stage_.master == master_id &&
        llc_core_req_stage_.read.id == orig_id) {
      return true;
    }
    if (llc_upstream_req[master_id].valid && llc_upstream_req[master_id].id == orig_id) {
      return true;
    }
    if (llc_read_resp_valid_[master_id] &&
        llc_read_resp_id_[master_id] == orig_id) {
      return true;
    }
    if (llc.io.regs.read_resp_valid_r[master_id] &&
        llc.io.regs.read_resp_id_r[master_id] == orig_id) {
      return true;
    }
    if (llc.io.regs.lookup_valid_r &&
        llc.io.regs.lookup_master_r == master_id &&
        llc.io.regs.lookup_id_r == orig_id) {
      return true;
    }
    for (uint32_t slot = 0; slot < llc_config.mshr_num && slot < MAX_OUTSTANDING;
         ++slot) {
      const auto &entry = llc.io.regs.mshr[slot];
      if (entry.valid && !entry.is_prefetch && entry.master == master_id &&
          entry.id == orig_id) {
        return true;
      }
    }
  }
  return false;
}

bool AXI_InterconnectRef::can_issue_llc_read_req() const {
  return !any_ar_latched() && count_total_read_inflight() < MAX_OUTSTANDING &&
         alloc_read_axi_id() != kInvalidAxiReadId;
}

void AXI_InterconnectRef::prepare_llc_inputs(bool sample_upstream_ready) {
  llc.io.ext_in = {};

  if (!llc_enabled() && !mode_transition_needs_flush()) {
    return;
  }

  const uint32_t invalidate_line_addr =
      AXI_LLC::line_addr(llc_config, llc_invalidate_line_addr_);
  auto llc_external_read_line_pending = [&](uint32_t line_addr_value) {
    if (ar_latched.valid && ar_latched.to_llc &&
        AXI_LLC::line_addr(llc_config, ar_latched.upstream_addr) ==
            line_addr_value) {
      return true;
    }
    if (ar_latched_mmio.valid && ar_latched_mmio.to_llc &&
        AXI_LLC::line_addr(llc_config, ar_latched_mmio.upstream_addr) ==
            line_addr_value) {
      return true;
    }
    for (const auto &txn : r_pending) {
      if (txn.to_llc &&
          AXI_LLC::line_addr(llc_config, txn.upstream_addr) ==
              line_addr_value) {
        return true;
      }
    }
    return false;
  };
  const bool llc_external_write_busy =
      external_write_busy() || llc_mem_write_resp_valid_ ||
      llc_mem_ignored_b_count_ != 0;
  // Mode-transition flushes must wait for the full shared path to quiesce before
  // changing address interpretation. A normal invalidate_all can enter the LLC
  // while clean read misses are still pending; the LLC epoch prevents stale
  // refills from being installed.
  const bool llc_maintenance_quiescent =
      reconfig_path_quiescent() && !llc_external_write_busy;
  const bool transition_invalidate_all =
      mode_transition_invalidate_requested() && llc_maintenance_quiescent;
  const bool explicit_invalidate_all =
      llc_invalidate_all_req_ && !llc_external_write_busy;
  llc.io.ext_in.mem.invalidate_all =
      explicit_invalidate_all || transition_invalidate_all;
  const bool line_hazard =
      llc_invalidate_line_valid_ &&
      (has_same_line_write_hazard(invalidate_line_addr) ||
       llc_external_read_line_pending(invalidate_line_addr) ||
       llc_external_write_busy);
  llc.io.ext_in.mem.invalidate_line_valid =
      llc_invalidate_line_valid_ && llc_maintenance_quiescent &&
      !line_hazard;
  llc.io.ext_in.mem.invalidate_line_addr = llc_invalidate_line_addr_;
  bool any_upstream_capture_pending = false;
  bool any_upstream_req_visible = llc_core_req_stage_.valid;
  for (int master = 0; master < NUM_READ_MASTERS; ++master) {
    const bool capture_pending =
        !llc_upstream_req[master].valid && req_ready_r[master] &&
        read_ports[master].req.valid;
    any_upstream_capture_pending =
        any_upstream_capture_pending || capture_pending;
    any_upstream_req_visible = any_upstream_req_visible ||
                               llc_upstream_req[master].valid ||
                               read_ports[master].req.valid;

    if (llc_core_req_stage_.valid && !llc_core_req_stage_.is_write &&
        llc_core_req_stage_.master == master) {
      llc.io.ext_in.upstream.read_req[master].valid = true;
      llc.io.ext_in.upstream.read_req[master].addr = llc_core_req_stage_.read.addr;
      llc.io.ext_in.upstream.read_req[master].total_size =
          llc_core_req_stage_.read.total_size;
      llc.io.ext_in.upstream.read_req[master].id = llc_core_req_stage_.read.id;
      llc.io.ext_in.upstream.read_req[master].bypass =
          llc_core_req_stage_.read.bypass;
      llc.io.ext_in.upstream.read_req[master].direct_mapped =
          llc_core_req_stage_.read.direct_mapped;
      llc.io.ext_in.upstream.read_req[master].mode2_ddr_aligned =
          llc_core_req_stage_.read.mode2_ddr_aligned;
    }
    // The shared-fabric model adds a registered response boundary after the LLC
    // core. The C++ core keeps fresh responses visible for one full cycle;
    // consume them into the boundary only after that fresh cycle and only when the
    // boundary slot is free.
    llc.io.ext_in.upstream.read_resp[master].ready =
        sample_upstream_ready && !llc_read_resp_valid_[master] &&
        !llc.io.regs.read_resp_fresh_r[master] &&
        !has_direct_read_response(static_cast<uint8_t>(master));
    if (llc_resp_trace_active() && master == MASTER_DCACHE_R &&
        (read_ports[master].resp.ready ||
         llc.io.ext_in.upstream.read_resp[master].ready)) {

    }
  }

  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    any_upstream_req_visible = any_upstream_req_visible ||
                               llc_upstream_write_req[master].valid ||
                               !llc_upstream_write_q[master].empty() ||
                               write_ports[master].req.valid;
    if (llc_core_req_stage_.valid && llc_core_req_stage_.is_write &&
        llc_core_req_stage_.master == master) {
      llc.io.ext_in.upstream.write_req[master].valid = true;
      llc.io.ext_in.upstream.write_req[master].addr =
          llc_core_req_stage_.write.addr;
      llc.io.ext_in.upstream.write_req[master].total_size =
          llc_core_req_stage_.write.total_size;
      llc.io.ext_in.upstream.write_req[master].id = llc_core_req_stage_.write.id;
      llc.io.ext_in.upstream.write_req[master].wdata =
          llc_core_req_stage_.write.wdata;
      llc.io.ext_in.upstream.write_req[master].wstrb =
          llc_core_req_stage_.write.wstrb;
      llc.io.ext_in.upstream.write_req[master].bypass =
          llc_core_req_stage_.write.bypass;
      llc.io.ext_in.upstream.write_req[master].direct_mapped =
          llc_core_req_stage_.write.direct_mapped;
      llc.io.ext_in.upstream.write_req[master].mode2_ddr_aligned =
          llc_core_req_stage_.write.mode2_ddr_aligned;
    }
    llc.io.ext_in.upstream.write_resp[master].ready =
        sample_upstream_ready && !w_resp_valid[master] &&
        llc_write_resp_delay[master].size() < 2;
  }

  llc.io.ext_in.mem.prefetch_allow =
      !any_upstream_capture_pending && !any_upstream_req_visible;
  const uint8_t llc_mem_read_req_size =
      static_cast<uint8_t>(llc.io.ext_out.mem.read_req_size);
  const DownstreamPort llc_mem_read_req_port = classify_downstream_port(
      llc.io.ext_out.mem.read_req_addr, llc_mem_read_req_size);
  const bool llc_mem_read_req_supported =
      downstream_request_supported(llc_mem_read_req_port, llc_mem_read_req_size);
  const bool llc_mem_read_req_synth =
      unsupported_mmio_request(llc_mem_read_req_port, llc_mem_read_req_size);
  llc.io.ext_in.mem.read_req_ready =
      (llc.io.ext_out.mem.read_req_valid && llc_mem_read_req_synth)
          ? !llc_synth_read_resp_valid_
          : (can_issue_llc_read_req() &&
             (!llc.io.ext_out.mem.read_req_valid ||
              (llc_mem_read_req_supported &&
               can_issue_external_read(llc.io.ext_out.mem.read_req_addr))));
  const uint8_t llc_mem_write_req_size =
      static_cast<uint8_t>(llc.io.ext_out.mem.write_req_size);
  const DownstreamPort llc_mem_write_req_port = classify_downstream_port(
      llc.io.ext_out.mem.write_req_addr, llc_mem_write_req_size);
  const bool llc_mem_write_req_supported = downstream_request_supported(
      llc_mem_write_req_port, llc_mem_write_req_size);
  const bool llc_mem_write_req_synth =
      unsupported_mmio_request(llc_mem_write_req_port, llc_mem_write_req_size);
  llc.io.ext_in.mem.write_req_ready =
      !any_w_active() && !any_aw_latched() && !llc_mem_write_resp_valid_ &&
      (!llc.io.ext_out.mem.write_req_valid ||
       llc_mem_write_req_synth ||
       (llc_mem_write_req_supported &&
        can_issue_external_write(llc.io.ext_out.mem.write_req_addr)));
  llc.io.ext_in.mem.write_resp_valid = llc_mem_write_resp_valid_;
  llc.io.ext_in.mem.write_resp = llc_mem_write_resp_;

  if (llc_synth_read_resp_valid_) {
    llc.io.ext_in.mem.read_resp_valid = true;
    llc.io.ext_in.mem.read_resp_data = llc_synth_read_resp_data_;
    llc.io.ext_in.mem.read_resp_id = llc_synth_read_resp_id_;
    return;
  }

  for (const auto &txn : r_pending) {
    if (!txn.to_llc || txn.beats_done != txn.total_beats) {
      continue;
    }
    dump_focus_read_words("MEM-RSP-FWD", sim_time, txn);
    llc.io.ext_in.mem.read_resp_valid = true;
    llc.io.ext_in.mem.read_resp_data = txn.resp_extract_from_aligned_beat
                                           ? extract_aligned_downstream_read(txn)
                                           : txn.data;
    llc.io.ext_in.mem.read_resp_id = txn.orig_id;
    break;
  }
}

// ============================================================================
// Two-Phase Combinational Logic
// ============================================================================

// Phase 1: Output signals for masters (run BEFORE cpu.cycle())
// Sets: port.resp.valid/data, port.req.ready (from register), DDR rready
void AXI_InterconnectRef::comb_outputs() {
  prepare_llc_inputs(false);
  llc.comb();

  // Response path: DDR → masters
  comb_read_response();
  comb_write_response();

  // Use registered req.ready values (set by previous cycle's comb_inputs)
  // This ensures ICache sees req.ready in the same cycle as it transitions
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    const bool llc_slot_busy =
        llc_enabled() &&
        (llc_upstream_req[i].valid ||
         (llc_core_req_stage_.valid && !llc_core_req_stage_.is_write &&
          llc_core_req_stage_.master == i)) &&
        !any_ar_latched();
    read_ports[i].req.ready =
        req_ready_r[i] && !llc_slot_busy &&
        !(llc_enabled() && invalidate_all_requested()) &&
        !mode_transition_needs_flush();
    read_ports[i].req.accepted = read_req_accepted[i];
    read_ports[i].req.accepted_id = read_req_accepted_id[i];
  }

  // If AR is latched (waiting for arready), also keep req.ready true.
  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    const auto &latch = ar_latch(port);
    if (latch.valid && !latch.to_llc && latch.master_id < NUM_READ_MASTERS) {
      read_ports[latch.master_id].req.ready = true;
    }
  }

  // Registered write req.ready (two-phase timing)
  for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
    write_ports[i].req.accepted = write_req_accepted[i];
    if (llc_enabled()) {
      const uint32_t req_addr =
          static_cast<uint32_t>(write_ports[i].req.addr);
      const uint8_t req_total_size =
          static_cast<uint8_t>(write_ports[i].req.total_size);
      const uint32_t req_line = AXI_LLC::line_addr(
          llc_config, translate_llc_addr(req_addr, req_total_size));
      const bool blocked_by_line_invalidate =
          llc_invalidate_line_valid_ && write_ports[i].req.valid &&
          req_line ==
              AXI_LLC::line_addr(llc_config, llc_invalidate_line_addr_);
      const bool request_goes_through_llc =
          write_ports[i].req.valid &&
          !request_uses_mmio_port(req_addr, req_total_size);
      const bool blocked_by_pending_read =
          write_ports[i].req.valid &&
          (has_external_pending_read_hazard(req_addr) ||
           (request_goes_through_llc && has_same_line_read_hazard(req_line)));
      write_ports[i].req.ready =
          !invalidate_all_requested() &&
          !blocked_by_line_invalidate &&
          !blocked_by_pending_read &&
          w_req_ready_r[i];
    } else {
      write_ports[i].req.ready = w_req_ready_r[i] && !mode_transition_needs_flush();
    }
  }
  if (llc_enabled()) {
    for (int i = 0; i < NUM_READ_MASTERS; ++i) {
      if (!read_ports[i].resp.valid) {
        read_ports[i].resp.valid = llc_read_resp_valid_[i];
        read_ports[i].resp.data = llc_read_resp_data_[i];
        read_ports[i].resp.id = llc_read_resp_id_[i];
      }
      if (llc_resp_trace_active() && i == MASTER_DCACHE_R &&
          (read_ports[i].resp.valid || read_ports[i].resp.ready)) {

      }
    }
    for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
      if (w_resp_valid[i]) {
        write_ports[i].resp.valid = true;
        write_ports[i].resp.id = w_resp_id[i];
        write_ports[i].resp.resp = w_resp_resp[i];
      } else {
        write_ports[i].resp.valid = false;
        write_ports[i].resp.id = 0;
        write_ports[i].resp.resp = 0;
      }
    }
  }
}

// Phase 2: Input signals from masters (run AFTER cpu.cycle())
// Processes: port.req.valid/addr, drives DDR AR/AW/W
void AXI_InterconnectRef::comb_inputs() {
  comb_read_arbiter();
  comb_write_request();
  if (llc_enabled() || mode_transition_needs_flush()) {
    prepare_llc_inputs(true);
    llc.comb();
  }
}

// ============================================================================
// Read Arbiter with Latched AR (AXI Compliant)
// ============================================================================
void AXI_InterconnectRef::comb_read_arbiter() {
  ar_from_llc_c = false;
  ar_llc_mem_id_c = 0;
  ar_llc_resp_extract_from_aligned_beat_c = false;
  ar_llc_upstream_addr_c = 0;
  ar_llc_upstream_total_size_c = 0;
  ar_port_c = DownstreamPort::DDR;
  ar_master_c = -1;
  ar_orig_id_c = 0;
  ar_issue_c[0] = {};
  ar_issue_c[1] = {};
  for (int i = 0; i < NUM_READ_MASTERS; ++i) {
    llc_upstream_accept_c[i] = false;
    llc_upstream_capture_c[i] = {};
  }
  bool req_ready_curr[NUM_READ_MASTERS];
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    req_ready_curr[i] = req_ready_r[i];
    req_ready_r[i] = false;
  }

  // Detect ready pulses without a corresponding valid (possible dropped req).
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    if (req_ready_curr[i] && !read_ports[i].req.valid && !req_drop_warned[i] &&
        DEBUG) {

      req_drop_warned[i] = true;
    }
    if (req_ready_curr[i] && !read_ports[i].req.valid) {
      read_req_hold[i] = {};
    }
  }

  // Default: don't accept new requests
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    read_ports[i].req.ready = false;
  }
  axi_ddr_io.ar.arvalid = false;
  axi_mmio_io.ar.arvalid = false;

  bool port_busy[2] = {false, false};
  bool comb_read_id_used[2][1u << sim_ddr::AXI_ID_WIDTH] = {};
  uint8_t comb_master_read_count[NUM_READ_MASTERS] = {};

  auto alloc_comb_read_axi_id = [&](DownstreamPort port) -> uint8_t {
    bool used[1u << sim_ddr::AXI_ID_WIDTH] = {false};
    for (const auto &txn : r_pending) {
      if (txn.port == port) {
        used[txn.axi_id & kAxiIdMask] = true;
      }
    }
    const auto &latch = ar_latch(port);
    if (latch.valid) {
      used[latch.id & kAxiIdMask] = true;
    }
    const uint8_t port_idx = port_index(port);
    for (uint8_t id = 0; id < (1u << sim_ddr::AXI_ID_WIDTH); ++id) {
      if (!used[id] && !comb_read_id_used[port_idx][id]) {
        comb_read_id_used[port_idx][id] = true;
        return id;
      }
    }
    return kInvalidAxiReadId;
  };

  auto can_accept_read_master_comb = [&](uint8_t master_id) {
    return master_id < NUM_READ_MASTERS &&
           count_total_read_inflight() +
                   comb_master_read_count[0] + comb_master_read_count[1] +
                   comb_master_read_count[2] + comb_master_read_count[3] <
               MAX_OUTSTANDING &&
           count_master_read_pending(master_id) +
                   comb_master_read_count[master_id] <
               MAX_READ_OUTSTANDING_PER_MASTER;
  };

  auto drive_ar = [&](const DownstreamReadIssue &issue, uint8_t axi_id,
                      const ARIssueMeta_t &meta) {
    const int pidx = port_index(issue.port);
    auto &ar_io = downstream_io(issue.port);
    ar_io.ar.arvalid = true;
    ar_io.ar.araddr = issue.addr;
    ar_io.ar.arlen = calc_burst_len(issue.total_size);
    ar_io.ar.arsize =
        issue.port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize;
    ar_io.ar.arburst = sim_ddr::AXI_BURST_INCR;
    ar_io.ar.arid = axi_id;
    ar_issue_c[pidx] = meta;
    ar_issue_c[pidx].valid = true;
    port_busy[pidx] = true;
    ar_port_c = issue.port;
  };

  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    auto &latch = ar_latch(port);
    if (!latch.valid) {
      continue;
    }
    auto &ar_io = downstream_io(port);
    ar_io.ar.arvalid = true;
    ar_io.ar.araddr = latch.addr;
    ar_io.ar.arlen = latch.len;
    ar_io.ar.arsize = latch.size;
    ar_io.ar.arburst = latch.burst;
    ar_io.ar.arid = latch.id;
    port_busy[port_index(port)] = true;
    if (!latch.to_llc && !latch.accepted_upstream &&
        latch.master_id < NUM_READ_MASTERS) {
      read_ports[latch.master_id].req.ready = true;
      req_ready_r[latch.master_id] = true;
    }
  }

  if (llc_enabled()) {
    const uint8_t llc_read_req_id = llc.io.ext_out.mem.read_req_id;
    const uint32_t llc_read_req_addr = llc.io.ext_out.mem.read_req_addr;
    const uint8_t llc_read_req_size =
        static_cast<uint8_t>(llc.io.ext_out.mem.read_req_size);
    const bool stale_llc_read_req_seen =
        llc_mem_read_issue_seen_valid_ &&
        llc_mem_read_issue_seen_id_ == llc_read_req_id &&
        llc_mem_read_issue_seen_addr_ == llc_read_req_addr &&
        llc_mem_read_issue_seen_size_ == llc_read_req_size;
    if (llc.io.ext_out.mem.read_req_valid && !stale_llc_read_req_seen &&
        can_issue_llc_read_req() &&
        can_issue_external_read(llc.io.ext_out.mem.read_req_addr)) {
      const uint8_t issue_size =
          static_cast<uint8_t>(llc.io.ext_out.mem.read_req_size);
      const auto issue_port = classify_downstream_port(
          llc.io.ext_out.mem.read_req_addr, issue_size);
      if (!port_busy[port_index(issue_port)] &&
          downstream_request_supported(issue_port, issue_size)) {
        const uint8_t axi_id = alloc_comb_read_axi_id(issue_port);
        if (axi_id != kInvalidAxiReadId) {
          const auto issue = make_downstream_read_issue(
              issue_port, llc.io.ext_out.mem.read_req_addr, issue_size,
              llc_config.line_bytes,
              llc.io.ext_out.mem.read_req_mode2_ddr_aligned);
          ar_from_llc_c = true;
          ar_llc_mem_id_c = llc.io.ext_out.mem.read_req_id;
          ar_llc_resp_extract_from_aligned_beat_c =
              issue.extract_from_aligned_beat;
          ar_llc_upstream_addr_c = llc.io.ext_out.mem.read_req_addr;
          ar_llc_upstream_total_size_c = llc.io.ext_out.mem.read_req_size;
          ARIssueMeta_t meta{};
          meta.from_llc = true;
          meta.llc_mem_id = llc.io.ext_out.mem.read_req_id;
          meta.resp_extract_from_aligned_beat = issue.extract_from_aligned_beat;
          meta.upstream_addr = llc.io.ext_out.mem.read_req_addr;
          meta.upstream_total_size = llc.io.ext_out.mem.read_req_size;
          meta.master_id = 0;
          meta.orig_id = llc.io.ext_out.mem.read_req_id;
          drive_ar(issue, axi_id, meta);
          llc_mem_read_issue_seen_valid_ = true;
          llc_mem_read_issue_seen_id_ = llc_read_req_id;
          llc_mem_read_issue_seen_addr_ = llc_read_req_addr;
          llc_mem_read_issue_seen_size_ = llc_read_req_size;
        }
      }
    }
    for (int master = 0; master < NUM_READ_MASTERS; ++master) {
      const bool stage_busy_for_master =
          llc_core_req_stage_.valid && !llc_core_req_stage_.is_write &&
          llc_core_req_stage_.master == master;
      if (llc_upstream_req[master].valid || stage_busy_for_master ||
          !read_ports[master].req.valid ||
          invalidate_all_requested() || mode_transition_needs_flush()) {
        continue;
      }
      if (has_read_id_conflict(static_cast<uint8_t>(master),
                               static_cast<uint8_t>(read_ports[master].req.id))) {
        continue;
      }
      const uint32_t req_addr =
          static_cast<uint32_t>(read_ports[master].req.addr);
      const uint8_t req_total_size =
          static_cast<uint8_t>(read_ports[master].req.total_size);
      const DownstreamPort req_port =
          classify_downstream_port(req_addr, req_total_size);
      const bool req_supported =
          downstream_request_supported(req_port, req_total_size);
      if (!request_uses_direct_mapped_llc(req_addr, req_total_size) &&
          !req_supported) {
        continue;
      }
      const bool direct_mmio_read =
          request_uses_mmio_port(req_addr, req_total_size) && req_supported;
      if (direct_mmio_read &&
          can_issue_external_read(req_addr) &&
          !port_busy[port_index(DownstreamPort::MMIO)] &&
          can_accept_read_master_comb(static_cast<uint8_t>(master))) {
        const bool allow_same_cycle_accept = (master == MASTER_DCACHE_R);
        if (!req_ready_curr[master] && !allow_same_cycle_accept) {
          req_ready_r[master] = true;
          read_ports[master].req.ready = true;
          read_req_hold[master].valid = true;
          read_req_hold[master].addr = req_addr;
          read_req_hold[master].total_size = req_total_size;
          read_req_hold[master].id =
              static_cast<uint8_t>(read_ports[master].req.id);
          read_req_hold[master].bypass = read_ports[master].req.bypass;
          continue;
        }

        const ReadReqHoldLatch cap =
            read_req_hold[master].valid
                ? read_req_hold[master]
                : ReadReqHoldLatch{true,
                                   req_addr,
                                   req_total_size,
                                   static_cast<uint8_t>(
                                       read_ports[master].req.id),
                                   static_cast<bool>(
                                       read_ports[master].req.bypass)};
        if (has_read_id_conflict(static_cast<uint8_t>(master), cap.id)) {
          continue;
        }
        const uint8_t axi_id = alloc_comb_read_axi_id(DownstreamPort::MMIO);
        if (axi_id == kInvalidAxiReadId) {
          continue;
        }
        const auto issue = make_downstream_read_issue(
            DownstreamPort::MMIO, cap.addr, cap.total_size,
            llc_config.line_bytes, false);
        ARIssueMeta_t meta{};
        meta.resp_extract_from_aligned_beat = issue.extract_from_aligned_beat;
        meta.upstream_addr = cap.addr;
        meta.upstream_total_size = cap.total_size;
        meta.master_id = master;
        meta.orig_id = cap.id;
        drive_ar(issue, axi_id, meta);
        read_ports[master].req.ready = true;
        read_req_hold[master] = {};
        comb_master_read_count[master]++;
        if (port_busy[0] && port_busy[1]) {
          break;
        }
        continue;
      }
      if (direct_mmio_read) {
        continue;
      }
      const bool allow_same_cycle_accept = (master == MASTER_DCACHE_R);
      if (req_ready_curr[master] || allow_same_cycle_accept) {
        read_ports[master].req.ready = true;
        llc_upstream_accept_c[master] = true;
        llc_upstream_capture_c[master].valid = true;
        llc_upstream_capture_c[master].addr =
            translate_llc_addr(req_addr, req_total_size);
        llc_upstream_capture_c[master].total_size = req_total_size;
        llc_upstream_capture_c[master].id = read_ports[master].req.id;
        llc_upstream_capture_c[master].bypass = effective_llc_bypass(
            req_addr, req_total_size,
            static_cast<bool>(read_ports[master].req.bypass));
        llc_upstream_capture_c[master].direct_mapped =
            request_uses_direct_mapped_llc(req_addr, req_total_size);
        llc_upstream_capture_c[master].mode2_ddr_aligned =
            runtime_mode_ == 2u &&
            request_uses_ddr_port(req_addr, req_total_size) &&
            !request_in_mapped_window(req_addr, req_total_size);
        continue;
      }
      req_ready_r[master] = true;
      read_ports[master].req.ready = true;
    }
    return;
  }

  if (mode_transition_needs_flush()) {
    return;
  }

  // If a master saw ready last cycle, complete that handshake first.
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    if (!req_ready_curr[i] || !read_ports[i].req.valid) {
      continue;
    }
    ReadReqHoldLatch cap =
        read_req_hold[i].valid
            ? read_req_hold[i]
            : ReadReqHoldLatch{true,
                               static_cast<uint32_t>(read_ports[i].req.addr),
                               static_cast<uint8_t>(read_ports[i].req.total_size),
                               static_cast<uint8_t>(read_ports[i].req.id),
                               static_cast<bool>(read_ports[i].req.bypass)};
    const bool hold_matches_current =
        read_req_hold[i].valid &&
        cap.addr == static_cast<uint32_t>(read_ports[i].req.addr) &&
        cap.total_size ==
            static_cast<uint8_t>(read_ports[i].req.total_size) &&
        cap.id == static_cast<uint8_t>(read_ports[i].req.id) &&
        cap.bypass == static_cast<bool>(read_ports[i].req.bypass);
    if (read_req_hold[i].valid && !hold_matches_current) {
      const uint32_t old_addr = cap.addr;
      cap = ReadReqHoldLatch{true,
                             static_cast<uint32_t>(read_ports[i].req.addr),
                             static_cast<uint8_t>(read_ports[i].req.total_size),
                             static_cast<uint8_t>(read_ports[i].req.id),
                             static_cast<bool>(read_ports[i].req.bypass)};
      read_req_hold[i] = cap;
    }
    if (has_read_id_conflict(static_cast<uint8_t>(i), cap.id)) {
      continue;
    }
    if (!can_issue_external_read(cap.addr)) {
      continue;
    }
    const auto cap_port = classify_downstream_port(cap.addr, cap.total_size);
    if (!downstream_request_supported(cap_port, cap.total_size)) {
      continue;
    }
    if (port_busy[port_index(cap_port)]) {
      continue;
    }
    if (!can_accept_read_master_comb(static_cast<uint8_t>(i))) {
      continue;
    }

    r_current_master = i;
    ar_master_c = i;
    ar_orig_id_c = cap.id;
    uint8_t axi_id = alloc_comb_read_axi_id(cap_port);
    if (axi_id == kInvalidAxiReadId) {
      continue;
    }
    const auto issue = make_downstream_read_issue(
        cap_port, cap.addr, cap.total_size, llc_config.line_bytes, false);
    ar_llc_resp_extract_from_aligned_beat_c = issue.extract_from_aligned_beat;
    ar_llc_upstream_addr_c = cap.addr;
    ar_llc_upstream_total_size_c = cap.total_size;
    ARIssueMeta_t meta{};
    meta.resp_extract_from_aligned_beat = issue.extract_from_aligned_beat;
    meta.upstream_addr = cap.addr;
    meta.upstream_total_size = cap.total_size;
    meta.master_id = i;
    meta.orig_id = cap.id;
    drive_ar(issue, axi_id, meta);
    read_ports[i].req.ready = true;
    comb_master_read_count[i]++;
  }

  // Round-robin search for valid request
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    int idx = (r_arb_rr_idx + i) % NUM_READ_MASTERS;

    if (read_ports[idx].req.valid) {
      if (!can_accept_read_master(static_cast<uint8_t>(idx))) {
        continue;
      }
      if (has_read_id_conflict(static_cast<uint8_t>(idx),
                               static_cast<uint8_t>(read_ports[idx].req.id))) {
        continue;
      }
      if (!can_issue_external_read(static_cast<uint32_t>(read_ports[idx].req.addr))) {
        continue;
      }
      const uint32_t req_addr = static_cast<uint32_t>(read_ports[idx].req.addr);
      const uint8_t req_total_size =
          static_cast<uint8_t>(read_ports[idx].req.total_size);
      const auto req_port = classify_downstream_port(req_addr, req_total_size);
      if (!downstream_request_supported(req_port, req_total_size)) {
        continue;
      }
      if (port_busy[port_index(req_port)]) {
        continue;
      }
      if (!can_accept_read_master_comb(static_cast<uint8_t>(idx))) {
        continue;
      }

      r_current_master = idx;
      uint8_t axi_id = alloc_comb_read_axi_id(req_port);
      if (axi_id == kInvalidAxiReadId) {
        continue;
      }

      const bool allow_same_cycle_accept = (idx == MASTER_DCACHE_R);
      const bool require_ready_first = !allow_same_cycle_accept;
      if (require_ready_first && !req_ready_curr[idx]) {
        req_ready_r[idx] = true;
        read_ports[idx].req.ready = true;
        read_req_hold[idx].valid = true;
        read_req_hold[idx].addr = read_ports[idx].req.addr;
        read_req_hold[idx].total_size =
            static_cast<uint8_t>(read_ports[idx].req.total_size);
        read_req_hold[idx].id = static_cast<uint8_t>(read_ports[idx].req.id);
        read_req_hold[idx].bypass = read_ports[idx].req.bypass;
        continue;
      }

      // Data-side masters can use same-cycle accept to keep the AR issue rate
      // from being artificially halved. ICache still relies on ready-first
      // semantics internally, so preserve the existing two-cycle contract
      // there until the front-end request state machine is updated.
      const auto issue = make_downstream_read_issue(
          req_port, req_addr, req_total_size, llc_config.line_bytes, false);
      ar_llc_resp_extract_from_aligned_beat_c = issue.extract_from_aligned_beat;
      ar_llc_upstream_addr_c = req_addr;
      ar_llc_upstream_total_size_c = req_total_size;
      ARIssueMeta_t meta{};
      meta.resp_extract_from_aligned_beat = issue.extract_from_aligned_beat;
      meta.upstream_addr = req_addr;
      meta.upstream_total_size = req_total_size;
      meta.master_id = idx;
      meta.orig_id = static_cast<uint8_t>(read_ports[idx].req.id);
      drive_ar(issue, axi_id, meta);
      read_ports[idx].req.ready = true;
      comb_master_read_count[idx]++;
      if (port_busy[0] && port_busy[1]) {
        break;
      }
    }
  }
}

void AXI_InterconnectRef::comb_read_response() {
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    read_ports[i].resp.valid = false;
  }
  axi_ddr_io.r.rready = true;
  axi_mmio_io.r.rready = true;

  // Present at most one completed direct response per master per cycle. Use
  // downstream completion order, not AR order, so a response held under
  // upstream backpressure remains stable when an older transaction completes.
  ReadPendingTxn *selected[NUM_READ_MASTERS] = {};
  for (auto &txn : r_pending) {
    if (txn.to_llc || txn.beats_done != txn.total_beats ||
        txn.master_id >= NUM_READ_MASTERS) {
      continue;
    }
    auto *&best = selected[txn.master_id];
    if (best == nullptr || txn.completion_seq < best->completion_seq) {
      best = &txn;
    }
  }
  for (int master = 0; master < NUM_READ_MASTERS; ++master) {
    auto *txn = selected[master];
    if (txn == nullptr) {
      continue;
    }
    read_ports[master].resp.valid = true;
    read_ports[master].resp.data =
        txn->resp_extract_from_aligned_beat
            ? extract_aligned_downstream_read(*txn)
            : txn->data;
    read_ports[master].resp.id = txn->orig_id;
    if (focus_read_txn(*txn) || trace_icache_read_txn(*txn, sim_time)) {
      dump_focus_read_txn("RESP-DRIVE", sim_time, *txn);
    }
  }
}

// ============================================================================
// Write Request with Latched AW (AXI Compliant)
// ============================================================================
void AXI_InterconnectRef::comb_write_request() {
  bool w_req_ready_curr[NUM_WRITE_MASTERS];
  const bool any_w_resp_valid = any_write_resp_buffered();
  for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
    w_req_ready_curr[i] = w_req_ready_r[i];
    w_req_ready_r[i] = false;
    write_req_fire_c[i] = false;
    llc_upstream_write_accept_c[i] = false;
    llc_upstream_write_capture_c[i] = {};
  }

  aw_port_c = DownstreamPort::DDR;
  axi_ddr_io.aw.awvalid = false;
  axi_mmio_io.aw.awvalid = false;
  axi_ddr_io.w.wvalid = false;
  axi_mmio_io.w.wvalid = false;

  auto drive_aw_latch = [&](DownstreamPort port) {
    const auto &latch = aw_latch(port);
    if (!latch.valid) {
      return;
    }
    auto &aw_io = downstream_io(port);
    aw_io.aw.awvalid = true;
    aw_io.aw.awaddr = latch.addr;
    aw_io.aw.awlen = latch.len;
    aw_io.aw.awsize = latch.size;
    aw_io.aw.awburst = latch.burst;
    aw_io.aw.awid = latch.id;
  };

  auto drive_aw_txn = [&](const WritePendingTxn &txn) {
    aw_port_c = txn.port;
    auto &aw_io = downstream_io(txn.port);
    aw_io.aw.awvalid = true;
    aw_io.aw.awaddr = txn.addr;
    aw_io.aw.awlen = txn.total_beats - 1;
    aw_io.aw.awsize =
        txn.port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize;
    aw_io.aw.awburst = sim_ddr::AXI_BURST_INCR;
    aw_io.aw.awid = txn.axi_id;
  };

  auto drive_w_current = [&](DownstreamPort port) {
    const bool active = w_active_ref(port);
    const auto &current = w_current_ref(port);
    if (!active || !current.aw_done || current.w_done) {
      return;
    }
    auto &w_io = downstream_io(port);
    w_io.w.wvalid = true;
    w_io.w.wdata =
        pack_downstream_write_beat(current.wdata, current.beats_sent);
    w_io.w.wstrb =
        pack_downstream_write_strobe(current.wstrb, current.beats_sent);
    w_io.w.wlast = (current.beats_sent == current.total_beats - 1);
  };

  if (llc_enabled()) {
    drive_aw_latch(DownstreamPort::DDR);
    drive_aw_latch(DownstreamPort::MMIO);
    if (!llc_mem_write_resp_valid_ && llc.io.ext_out.mem.write_req_valid &&
        can_issue_external_write(llc.io.ext_out.mem.write_req_addr)) {
      const uint8_t issue_size =
          static_cast<uint8_t>(llc.io.ext_out.mem.write_req_size);
      const auto issue_port = classify_downstream_port(
          llc.io.ext_out.mem.write_req_addr, issue_size);
      if (!w_active_ref(issue_port) && !aw_latch(issue_port).valid &&
          downstream_request_supported(issue_port, issue_size)) {
        const auto issue = make_downstream_write_issue(
            issue_port, llc.io.ext_out.mem.write_req_addr, issue_size,
            llc.io.ext_out.mem.write_req_data,
            llc.io.ext_out.mem.write_req_strobe, llc_config.line_bytes,
            llc.io.ext_out.mem.write_req_mode2_ddr_aligned);
        auto &aw_io = downstream_io(issue.port);
        aw_port_c = issue.port;
        aw_io.aw.awvalid = true;
        aw_io.aw.awaddr = issue.addr;
        aw_io.aw.awlen = calc_burst_len(issue.total_size);
        aw_io.aw.awsize = issue.port == DownstreamPort::MMIO
                              ? kMmioAxiSize
                              : kDownstreamAxiSize;
        aw_io.aw.awburst = sim_ddr::AXI_BURST_INCR;
        aw_io.aw.awid = llc.io.ext_out.mem.write_req_id & kAxiIdMask;
      }
    }

    bool direct_write_selected = false;
    if (!any_w_resp_valid && !llc_mem_write_resp_valid_ &&
        !invalidate_all_requested() && !mode_transition_needs_flush()) {
      for (int k = 0; k < NUM_WRITE_MASTERS; ++k) {
        const int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
        if (!write_ports[idx].req.valid) {
          continue;
        }
        const uint32_t req_addr =
            static_cast<uint32_t>(write_ports[idx].req.addr);
        if (has_external_pending_read_hazard(req_addr)) {
          continue;
        }
        const uint8_t req_total_size =
            static_cast<uint8_t>(write_ports[idx].req.total_size);
        const DownstreamPort req_port =
            classify_downstream_port(req_addr, req_total_size);
        const bool direct_mmio_write =
            request_uses_mmio_port(req_addr, req_total_size) &&
            downstream_request_supported(req_port, req_total_size);
        if (!direct_mmio_write) {
          continue;
        }
        if (llc.io.ext_out.upstream.write_resp[idx].valid ||
            w_active_ref(req_port) || aw_latch(req_port).valid ||
            !can_issue_external_write(req_addr)) {
          direct_write_selected = true;
          break;
        }
        direct_write_selected = true;
        if (w_req_ready_curr[idx]) {
          write_ports[idx].req.ready = true;
          write_req_fire_c[idx] = true;
        } else {
          w_req_ready_r[idx] = true;
          write_ports[idx].req.ready = true;
        }
        break;
      }
    }

    const bool llc_write_queue_has_space =
        count_llc_write_pending() < MAX_WRITE_OUTSTANDING;
    if (!direct_write_selected && llc_write_queue_has_space &&
        !invalidate_all_requested() &&
        !mode_transition_needs_flush()) {
      for (int k = 0; k < NUM_WRITE_MASTERS; ++k) {
        const int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
        if (!write_ports[idx].req.valid) {
          continue;
        }
        const uint32_t req_addr =
            static_cast<uint32_t>(write_ports[idx].req.addr);
        const uint8_t req_total_size =
            static_cast<uint8_t>(write_ports[idx].req.total_size);
        const uint32_t req_line = AXI_LLC::line_addr(
            llc_config, translate_llc_addr(req_addr, req_total_size));
        const bool blocked_by_line_invalidate =
            llc_invalidate_line_valid_ &&
            req_line == AXI_LLC::line_addr(llc_config, llc_invalidate_line_addr_);
        const bool blocked_by_pending_read =
            !request_uses_mmio_port(req_addr, req_total_size) &&
            has_same_line_read_hazard(req_line);
        if (blocked_by_line_invalidate || blocked_by_pending_read) {
          continue;
        }
        const DownstreamPort req_port =
            classify_downstream_port(req_addr, req_total_size);
        const bool req_supported =
            downstream_request_supported(req_port, req_total_size);
        if (!request_uses_direct_mapped_llc(req_addr, req_total_size) &&
            !req_supported) {
          continue;
        }
        if (w_req_ready_curr[idx]) {
          write_ports[idx].req.ready = true;
          llc_upstream_write_accept_c[idx] = true;
          llc_upstream_write_capture_c[idx].valid = true;
          llc_upstream_write_capture_c[idx].addr =
              translate_llc_addr(req_addr, req_total_size);
          llc_upstream_write_capture_c[idx].total_size = req_total_size;
          llc_upstream_write_capture_c[idx].id = write_ports[idx].req.id;
          llc_upstream_write_capture_c[idx].wdata = write_ports[idx].req.wdata;
          llc_upstream_write_capture_c[idx].wstrb = write_ports[idx].req.wstrb;
          llc_upstream_write_capture_c[idx].bypass = effective_llc_bypass(
              req_addr, req_total_size,
              static_cast<bool>(write_ports[idx].req.bypass));
          llc_upstream_write_capture_c[idx].direct_mapped =
              request_uses_direct_mapped_llc(req_addr, req_total_size);
          llc_upstream_write_capture_c[idx].mode2_ddr_aligned =
              runtime_mode_ == 2u &&
              request_uses_ddr_port(req_addr, req_total_size) &&
              !request_in_mapped_window(req_addr, req_total_size);
          break;
        }
        w_req_ready_r[idx] = true;
        write_ports[idx].req.ready = true;
        break;
      }
    }

    drive_w_current(DownstreamPort::DDR);
    drive_w_current(DownstreamPort::MMIO);
    return;
  }

  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    if (aw_latch(port).valid) {
      drive_aw_latch(port);
      continue;
    }
    if (!any_w_resp_valid) {
      const int aw_idx = find_next_aw_pending(port);
      if (aw_idx >= 0) {
        const auto &txn = w_pending[static_cast<size_t>(aw_idx)];
        if (can_issue_external_write(txn.addr)) {
          drive_aw_txn(txn);
        }
      }
    }
  }

  if (!mode_transition_needs_flush()) {
    // Ready-first handshake: raise req.ready one cycle before accepting.
    if (can_accept_write_now() && !any_w_resp_valid) {
      for (int k = 0; k < NUM_WRITE_MASTERS; k++) {
        int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
        if (!write_ports[idx].req.valid) {
          continue;
        }
        if (!write_size_supported(write_ports[idx].req.total_size)) {

          continue;
        }
        const uint32_t req_addr =
            static_cast<uint32_t>(write_ports[idx].req.addr);
        const uint8_t req_total_size =
            static_cast<uint8_t>(write_ports[idx].req.total_size);
        const DownstreamPort req_port =
            classify_downstream_port(req_addr, req_total_size);
        if (!downstream_request_supported(req_port, req_total_size)) {
          continue;
        }
        if (w_req_ready_curr[idx]) {
          write_ports[idx].req.ready = true;
          write_req_fire_c[idx] = true;
          break;
        } else {
          w_req_ready_r[idx] = true;
          write_ports[idx].req.ready = true;
        }
        break;
      }
    }
  }

  drive_w_current(DownstreamPort::DDR);
  drive_w_current(DownstreamPort::MMIO);
}

void AXI_InterconnectRef::comb_write_response() {
  if (llc_enabled()) {
    auto bready_for_port = [&](DownstreamPort port,
                               const sim_ddr::SimDDR_IO_t &io) {
      const auto &current = w_current_ref(port);
      const bool active = w_active_ref(port);
      const bool current_matches =
          active && current.w_done &&
          current.axi_id == static_cast<uint8_t>(io.b.bid & kAxiIdMask);
      if (current_matches) {
        if (current.to_llc_mem) {
          return !llc_mem_write_resp_valid_;
        }
        return current.master_id < NUM_WRITE_MASTERS &&
               write_resp_buffer_has_space(current.master_id);
      }
      const bool can_ignore_victim_b =
          !active && llc_mem_ignored_b_count_ > 0;
      return can_ignore_victim_b;
    };
    axi_ddr_io.b.bready =
        bready_for_port(DownstreamPort::DDR, axi_ddr_io);
    axi_mmio_io.b.bready =
        bready_for_port(DownstreamPort::MMIO, axi_mmio_io);
    return;
  }

  for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
    write_ports[i].req.accepted = write_req_accepted[i];
    write_ports[i].resp.valid = w_resp_valid[i];
    write_ports[i].resp.id = w_resp_id[i];
    write_ports[i].resp.resp = w_resp_resp[i];
  }

  auto b_target_master = [&](DownstreamPort port,
                             const sim_ddr::SimDDR_IO_t &io) -> int {
    if (!io.b.bvalid) {
      return -2;
    }
    const uint8_t bid = static_cast<uint8_t>(io.b.bid & kAxiIdMask);
    for (const auto &txn : w_pending) {
      if (txn.port == port && txn.axi_id == bid && txn.w_done &&
          txn.master_id < NUM_WRITE_MASTERS) {
        return txn.master_id;
      }
    }
    return -1;
  };
  auto can_accept_b_for_master = [&](int master) -> bool {
    if (master == -2) {
      return true;
    }
    if (master < 0 || master >= NUM_WRITE_MASTERS) {
      return false;
    }
    return write_resp_buffer_has_space(static_cast<uint8_t>(master));
  };

  const int ddr_b_master = b_target_master(DownstreamPort::DDR, axi_ddr_io);
  const int mmio_b_master = b_target_master(DownstreamPort::MMIO, axi_mmio_io);
  bool ddr_bready = can_accept_b_for_master(ddr_b_master);
  bool mmio_bready = can_accept_b_for_master(mmio_b_master);
  if (ddr_b_master >= 0 && ddr_b_master == mmio_b_master &&
      ddr_bready && mmio_bready) {
    mmio_bready = false;
  }
  axi_ddr_io.b.bready = ddr_bready;
  axi_mmio_io.b.bready = mmio_bready;
}

// ============================================================================
// Sequential Logic
// ============================================================================
void AXI_InterconnectRef::seq() {
  constexpr uint32_t kPendingTimeout = 100000;
  const LlcCoreReqStage llc_core_req_stage_prev = llc_core_req_stage_;
  const bool llc_mem_write_resp_valid_prev = llc_mem_write_resp_valid_;
  const bool llc_synth_read_resp_valid_prev = llc_synth_read_resp_valid_;
  const uint8_t req_mode = requested_mode();
  const uint32_t req_offset = requested_llc_mapped_offset();
  const bool requested_diff = requested_config_differs_from_active();
  auto assert_llc_consumed_reads = [&]() {
    if (!llc_enabled()) {
      return;
    }
    auto read_resp_queue_has_id = [&](int master, uint8_t id) {
      const uint8_t count = llc.io.regs.read_resp_q_count_r[master];
      const uint8_t head = llc.io.regs.read_resp_q_head_r[master];
      for (uint8_t off = 0; off < count; ++off) {
        const uint8_t slot =
            static_cast<uint8_t>((head + off) % AXI_LLC_READ_RESP_QUEUE_DEPTH);
        if (llc.io.regs.read_resp_q_id_r[master][slot] == id) {
          return true;
        }
      }
      return false;
    };
    if (!(llc_core_req_stage_prev.valid &&
          !llc_core_req_stage_prev.is_write)) {
      return;
    }
    const int master = llc_core_req_stage_prev.master;
    if (master < 0 || master >= NUM_READ_MASTERS ||
        !llc.io.ext_out.upstream.read_req[master].ready) {
      return;
    }
    const auto &consumed_req = llc_core_req_stage_prev.read;
    bool retained = llc.io.regs.lookup_valid_r &&
                    llc.io.regs.lookup_master_r ==
                        static_cast<uint8_t>(master) &&
                    llc.io.regs.lookup_id_r == consumed_req.id;
    for (uint32_t slot = 0; !retained && slot < llc_config.mshr_num &&
                            slot < MAX_OUTSTANDING;
         ++slot) {
      const auto &entry = llc.io.regs.mshr[slot];
      retained = entry.valid && !entry.is_prefetch &&
                 entry.master == static_cast<uint8_t>(master) &&
                 entry.id == consumed_req.id;
    }
    retained = retained || (llc.io.regs.read_resp_valid_r[master] &&
                            llc.io.regs.read_resp_id_r[master] ==
                                consumed_req.id);
    retained = retained || read_resp_queue_has_id(master, consumed_req.id);
    if (!retained) {

      std::exit(1);
    }
  };
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    read_req_accepted[i] = false;
    read_req_accepted_id[i] = 0;
  }
  for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
    write_req_accepted[i] = false;
  }

  // ========== AR Channel with Latch ==========
  auto finish_ar_txn = [&](ReadPendingTxn &txn) {
    txn.beats_done = 0;
    txn.data.clear();
    txn.stall_cycles = 0;
    txn.last_beats_done = 0;
    txn.timeout_warned = false;
    txn.completion_queued = false;
    txn.completion_seq = 0;
    r_pending.push_back(txn);
    if ((sim_time < 400 || focus_read_txn(txn) ||
         trace_icache_read_txn(txn, sim_time)) &&
        !txn.to_llc) {
      dump_focus_read_txn("AR-HS", sim_time, txn);
    }
    r_arb_rr_idx = (txn.master_id + 1) % NUM_READ_MASTERS;
    if (!txn.to_llc && txn.master_id < NUM_READ_MASTERS) {
      read_req_accepted[txn.master_id] = true;
      read_req_accepted_id[txn.master_id] = txn.orig_id;
      read_req_hold[txn.master_id] = {};
    }
  };

  auto process_ar_port = [&](DownstreamPort port) {
    auto &seq_ar_io = downstream_io(port);
    auto &latch = ar_latch(port);
    const auto &meta = ar_issue_c[port_index(port)];

    if (seq_ar_io.ar.arvalid && !latch.valid && !seq_ar_io.ar.arready &&
        meta.valid) {
      latch.valid = true;
      latch.accepted_upstream = false;
      latch.port = port;
      latch.addr = seq_ar_io.ar.araddr;
      latch.len = seq_ar_io.ar.arlen;
      latch.size = seq_ar_io.ar.arsize;
      latch.burst = seq_ar_io.ar.arburst;
      latch.id = seq_ar_io.ar.arid;
      latch.master_id = static_cast<uint8_t>(meta.master_id < 0 ? 0 : meta.master_id);
      latch.orig_id = meta.orig_id;
      latch.to_llc = meta.from_llc;
      latch.resp_extract_from_aligned_beat =
          meta.resp_extract_from_aligned_beat;
      latch.upstream_addr = meta.upstream_addr;
      latch.upstream_total_size = meta.upstream_total_size;
    }

    if (!seq_ar_io.ar.arvalid || !seq_ar_io.ar.arready) {
      return;
    }

    ReadPendingTxn txn;
    if (latch.valid) {
      txn.axi_id = latch.id;
      txn.master_id = latch.master_id;
      txn.orig_id = latch.orig_id;
      txn.total_beats = latch.len + 1;
      txn.port = latch.port;
      txn.addr = latch.addr;
      txn.upstream_addr = latch.upstream_addr;
      txn.upstream_total_size = latch.upstream_total_size;
      txn.resp_extract_from_aligned_beat =
          latch.resp_extract_from_aligned_beat;
      txn.to_llc = latch.to_llc;
      const bool upstream_accepted = latch.accepted_upstream;
      latch.valid = false;
      latch.accepted_upstream = false;
      latch.to_llc = false;
      latch.resp_extract_from_aligned_beat = false;
      latch.port = port;
      latch.upstream_addr = 0;
      latch.upstream_total_size = 0;
      if (!txn.to_llc && !upstream_accepted &&
          txn.master_id < NUM_READ_MASTERS) {
        read_req_accepted[txn.master_id] = true;
        read_req_accepted_id[txn.master_id] = txn.orig_id;
        read_req_hold[txn.master_id] = {};
      }
    } else {
      if (!meta.valid || meta.master_id < 0) {
        return;
      }
      txn.axi_id = seq_ar_io.ar.arid;
      txn.master_id = static_cast<uint8_t>(meta.master_id);
      txn.orig_id = meta.orig_id;
      txn.total_beats = seq_ar_io.ar.arlen + 1;
      txn.port = port;
      txn.addr = seq_ar_io.ar.araddr;
      txn.upstream_addr = meta.upstream_addr;
      txn.upstream_total_size = meta.upstream_total_size;
      txn.resp_extract_from_aligned_beat =
          meta.resp_extract_from_aligned_beat;
      txn.to_llc = meta.from_llc;
      if (!meta.from_llc) {
        read_req_hold[meta.master_id] = {};
      }
    }
    finish_ar_txn(txn);
  };
  process_ar_port(DownstreamPort::DDR);
  process_ar_port(DownstreamPort::MMIO);

  if (llc_enabled()) {
    auto promote_write_head = [&]() {
      for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
        if (llc_upstream_write_req[master].valid ||
            llc_upstream_write_q[master].empty()) {
          continue;
        }
        llc_upstream_write_req[master] = llc_upstream_write_q[master].front();
        llc_upstream_write_q[master].pop_front();
        if (focus_write_line(llc_upstream_write_req[master].addr) ||
            llc_focus_line(llc_upstream_write_req[master].addr)) {

        }
      }
    };

    auto dispatch_write_head = [&](uint32_t skip_line, bool use_skip_line) {
      for (uint32_t off = 0; off < NUM_WRITE_MASTERS; ++off) {
        const uint32_t master =
            (static_cast<uint32_t>(llc_core_dispatch_rr_) + off) %
            NUM_WRITE_MASTERS;
        if (!llc_upstream_write_req[master].valid) {
          continue;
        }
        if (use_skip_line &&
            AXI_LLC::line_addr(llc_config,
                               llc_upstream_write_req[master].addr) ==
                skip_line) {
          continue;
        }
        llc_core_req_stage_.valid = true;
        llc_core_req_stage_.is_write = true;
        llc_core_req_stage_.master = static_cast<uint8_t>(master);
        llc_core_req_stage_.read = {};
        llc_core_req_stage_.write = llc_upstream_write_req[master];
        llc_upstream_write_req[master] = {};
        llc_core_dispatch_rr_ =
            static_cast<uint8_t>((NUM_READ_MASTERS + master + 1) %
                                 (NUM_READ_MASTERS + NUM_WRITE_MASTERS));
        return true;
      }
      return false;
    };

    auto preempt_stalled_read_for_write = [&]() {
      if (!llc_core_req_stage_.valid || llc_core_req_stage_.is_write ||
          llc_core_req_stage_.master >= NUM_READ_MASTERS ||
          llc.io.ext_out.upstream
              .read_req[llc_core_req_stage_.master]
              .ready ||
          llc_upstream_req[llc_core_req_stage_.master].valid) {
        return false;
      }
      const uint32_t read_line =
          AXI_LLC::line_addr(llc_config, llc_core_req_stage_.read.addr);
      const uint8_t read_master = llc_core_req_stage_.master;
      const auto read_req = llc_core_req_stage_.read;
      if (!dispatch_write_head(read_line, true)) {
        return false;
      }
      llc_upstream_req[read_master] = read_req;
      return true;
    };

    const bool core_stage_read_pop =
        llc_core_req_stage_prev.valid && !llc_core_req_stage_prev.is_write &&
        llc_core_req_stage_prev.master < NUM_READ_MASTERS &&
        llc.io.ext_out.upstream.read_req[llc_core_req_stage_prev.master].ready;
    const bool core_stage_write_pop =
        llc_core_req_stage_prev.valid && llc_core_req_stage_prev.is_write &&
        llc_core_req_stage_prev.master < NUM_WRITE_MASTERS &&
        llc.io.ext_out.upstream.write_req[llc_core_req_stage_prev.master].ready;
    if (core_stage_read_pop || core_stage_write_pop) {
      if (core_stage_read_pop &&
          (llc_focus_line(llc_core_req_stage_prev.read.addr) ||
           trace_icache_llc_master(llc_core_req_stage_prev.master, sim_time))) {

      }
      if (core_stage_write_pop &&
          (focus_write_line(llc_core_req_stage_prev.write.addr) ||
           llc_focus_line(llc_core_req_stage_prev.write.addr))) {

      }
      llc_core_req_stage_ = {};
    }

    promote_write_head();
    (void)preempt_stalled_read_for_write();

    if (!llc_core_req_stage_.valid) {
      constexpr uint32_t total_ports = NUM_READ_MASTERS + NUM_WRITE_MASTERS;
      for (uint32_t off = 0; off < total_ports; ++off) {
        const uint32_t port = (llc_core_dispatch_rr_ + off) % total_ports;
        if (port < NUM_READ_MASTERS) {
          const uint32_t master = port;
          if (!llc_upstream_req[master].valid) {
            continue;
          }
          llc_core_req_stage_.valid = true;
          llc_core_req_stage_.is_write = false;
          llc_core_req_stage_.master = static_cast<uint8_t>(master);
          llc_core_req_stage_.read = llc_upstream_req[master];
          llc_core_req_stage_.write = {};
          llc_upstream_req[master] = {};
          llc_core_dispatch_rr_ = static_cast<uint8_t>((port + 1) % total_ports);
          break;
        }
        const uint32_t master = port - NUM_READ_MASTERS;
        if (!llc_upstream_write_req[master].valid) {
          continue;
        }
        llc_core_req_stage_.valid = true;
        llc_core_req_stage_.is_write = true;
        llc_core_req_stage_.master = static_cast<uint8_t>(master);
        llc_core_req_stage_.read = {};
        llc_core_req_stage_.write = llc_upstream_write_req[master];
        llc_upstream_write_req[master] = {};
        llc_core_dispatch_rr_ = static_cast<uint8_t>((port + 1) % total_ports);
        break;
      }
    }

    promote_write_head();

    for (int master = 0; master < NUM_READ_MASTERS; ++master) {
      if (!llc_upstream_req[master].valid && llc_upstream_accept_c[master]) {
        llc_upstream_req[master] = llc_upstream_capture_c[master];
        if (llc_focus_line(llc_upstream_capture_c[master].addr) ||
            trace_icache_llc_master(master, sim_time)) {

        }
        read_req_accepted[master] = true;
        read_req_accepted_id[master] =
            static_cast<uint8_t>(llc_upstream_capture_c[master].id);
      }
    }
    for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
      if (llc_upstream_write_accept_c[master]) {
        if (!llc_upstream_write_req[master].valid) {
          llc_upstream_write_req[master] = llc_upstream_write_capture_c[master];
        } else {
          llc_upstream_write_q[master].push_back(
              llc_upstream_write_capture_c[master]);
        }
        if (focus_write_line(llc_upstream_write_capture_c[master].addr) ||
            llc_focus_line(llc_upstream_write_capture_c[master].addr)) {

        }
        write_req_accepted[master] = true;
      }
    }
  }

  // R handshake
  auto handle_r_handshake = [&](DownstreamPort port, sim_ddr::SimDDR_IO_t &io) {
    if (!io.r.rvalid || !io.r.rready) {
      return;
    }
    for (auto &txn : r_pending) {
      if (txn.port == port &&
          txn.axi_id == static_cast<uint8_t>(io.r.rid & kAxiIdMask) &&
          txn.beats_done < txn.total_beats) {
        unpack_downstream_read_beat(txn, io.r.rdata);
        txn.beats_done++;
        if (!txn.to_llc && txn.beats_done == txn.total_beats &&
            !txn.completion_queued) {
          txn.completion_queued = true;
          txn.completion_seq = direct_read_completion_seq_++;
        }
        if (focus_read_txn(txn) || trace_icache_read_txn(txn, sim_time)) {
          const std::string beat_hex =
              axi_compat::hex_string(io.r.rdata, kDownstreamBeatBytes);

          if (txn.beats_done == txn.total_beats) {
            dump_focus_read_txn("R-COMPLETE", sim_time, txn);
          }
        }
        if (txn.to_llc && llc_focus_line(txn.addr)) {
          dump_focus_read_words("R-BEAT", sim_time, txn);
        }
        break;
      }
    }
  };
  handle_r_handshake(DownstreamPort::DDR, axi_ddr_io);
  handle_r_handshake(DownstreamPort::MMIO, axi_mmio_io);

  // Response handshake
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    if (read_ports[i].resp.valid && read_ports[i].resp.ready) {
      const uint8_t driven_orig_id =
          static_cast<uint8_t>(read_ports[i].resp.id & 0xFFu);
      auto it = std::find_if(r_pending.begin(), r_pending.end(),
                             [i, driven_orig_id](const ReadPendingTxn &t) {
                               return t.master_id == i &&
                                      !t.to_llc &&
                                      t.beats_done == t.total_beats &&
                                      t.orig_id == driven_orig_id;
                             });
      if (it != r_pending.end()) {
        if (focus_read_txn(*it) || trace_icache_read_txn(*it, sim_time)) {
          dump_focus_read_txn("RESP-RETIRE", sim_time, *it);
        }
        r_pending.erase(it);
      }
    }
  }
  if (llc_enabled() && !llc_synth_read_resp_valid_prev &&
      llc.io.ext_in.mem.read_resp_valid && llc.io.ext_out.mem.read_resp_ready) {
    const uint8_t mem_id = llc.io.ext_in.mem.read_resp_id;
    auto it = std::find_if(r_pending.begin(), r_pending.end(),
                           [mem_id](const ReadPendingTxn &t) {
                             return t.to_llc && t.beats_done == t.total_beats;
                           });
    auto exact_it = std::find_if(r_pending.begin(), r_pending.end(),
                                 [mem_id](const ReadPendingTxn &t) {
                                   return t.to_llc &&
                                          t.beats_done == t.total_beats &&
                                          t.orig_id == mem_id;
                                 });
    if (exact_it != r_pending.end()) {
      if (llc_focus_line(exact_it->addr) ||
          trace_icache_read_txn(*exact_it, sim_time)) {

      }
      r_pending.erase(exact_it);
    } else if (it != r_pending.end()) {
      r_pending.erase(it);
    }
  }

  // Monitor reads that have stopped making forward progress on the memory side.
  // Completed responses may legitimately wait for the upstream client, so only
  // track incomplete transactions and reset the timer whenever a new beat lands.
  for (auto &txn : r_pending) {
    if (txn.beats_done >= txn.total_beats) {
      txn.stall_cycles = 0;
      txn.last_beats_done = txn.beats_done;
      txn.timeout_warned = false;
      continue;
    }
    if (txn.beats_done != txn.last_beats_done) {
      txn.stall_cycles = 0;
      txn.last_beats_done = txn.beats_done;
      txn.timeout_warned = false;
      continue;
    }
    txn.stall_cycles++;
    if (txn.stall_cycles > kPendingTimeout && !txn.timeout_warned && DEBUG) {

      txn.timeout_warned = true;
    }
  }

  // ========== Write Channel ==========

  if (llc_enabled()) {
    bool llc_read_resp_accept[NUM_READ_MASTERS] = {};
    for (int i = 0; i < NUM_READ_MASTERS; ++i) {
      const bool direct_resp_displayed = has_direct_read_response(static_cast<uint8_t>(i));
      const bool compat_resp_displayed =
          llc_read_resp_valid_[i] && read_ports[i].resp.valid &&
          read_ports[i].resp.ready && !direct_resp_displayed &&
          static_cast<uint8_t>(read_ports[i].resp.id) == llc_read_resp_id_[i];
      if (compat_resp_displayed) {
        llc_read_resp_valid_[i] = false;
        llc_read_resp_data_[i].clear();
        llc_read_resp_id_[i] = 0;
      }
      llc_read_resp_accept[i] =
          llc.io.ext_out.upstream.read_resp[i].valid &&
          llc.io.ext_in.upstream.read_resp[i].ready &&
          !llc.io.regs.read_resp_fresh_r[i];
    }

    for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
      if (w_resp_valid[i] && write_ports[i].resp.ready) {
        w_resp_valid[i] = false;
        w_resp_id[i] = 0;
        w_resp_resp[i] = 0;
      }
      for (auto &entry : llc_write_resp_delay[i]) {
        if (entry.age < 1) {
          entry.age++;
        }
      }
      if (!llc_write_resp_delay[i].empty() &&
          llc_write_resp_delay[i].front().age >= 1 && !w_resp_valid[i]) {
        const auto entry = llc_write_resp_delay[i].front().entry;
        llc_write_resp_delay[i].pop_front();
        enqueue_write_resp(static_cast<uint8_t>(i), entry.id, entry.resp);
      }
      if (llc.io.ext_out.upstream.write_resp[i].valid &&
          llc.io.ext_in.upstream.write_resp[i].ready) {
        const uint32_t resp_line_addr = llc.io.regs.write_resp_line_addr_r[i];
        WriteRespEntry entry{
            static_cast<uint8_t>(llc.io.ext_out.upstream.write_resp[i].id),
            static_cast<uint8_t>(llc.io.ext_out.upstream.write_resp[i].resp),
            true,
            resp_line_addr};
        llc_write_resp_delay[i].push_back(DelayedWriteRespEntry{
            entry,
            0});
        llc.io.reg_write.write_resp_valid_r[i] = false;
        llc.io.reg_write.write_resp_id_r[i] = 0;
        llc.io.reg_write.write_resp_code_r[i] = 0;
        llc.io.reg_write.write_resp_line_addr_r[i] = 0;
      }
      if (!write_size_supported(write_ports[i].req.total_size)) {
        continue;
      }
    }
    promote_write_resp_queue();

    for (int i = 0; i < NUM_READ_MASTERS; ++i) {
      if (llc_read_resp_accept[i]) {
        llc_read_resp_valid_[i] = true;
        llc_read_resp_data_[i] = llc.io.ext_out.upstream.read_resp[i].data;
        llc_read_resp_id_[i] =
            static_cast<uint8_t>(llc.io.ext_out.upstream.read_resp[i].id);
      }
    }

    for (int k = 0; k < NUM_WRITE_MASTERS; ++k) {
      const int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
      if (!write_ports[idx].req.valid || !write_req_fire_c[idx]) {
        continue;
      }
      const uint32_t req_addr =
          static_cast<uint32_t>(write_ports[idx].req.addr);
      const uint8_t req_total_size =
          static_cast<uint8_t>(write_ports[idx].req.total_size);
      const DownstreamPort req_port =
          classify_downstream_port(req_addr, req_total_size);
      if (!request_uses_mmio_port(req_addr, req_total_size) ||
          !downstream_request_supported(req_port, req_total_size) ||
          w_active_ref(req_port) || aw_latch(req_port).valid ||
          !can_issue_external_write(req_addr)) {
        continue;
      }
      const uint8_t axi_id = alloc_write_axi_id(req_port);
      if (axi_id == kInvalidAxiWriteId) {
        break;
      }
      const auto issue = make_downstream_write_issue(
          req_port, req_addr, req_total_size, write_ports[idx].req.wdata,
          write_ports[idx].req.wstrb, llc_config.line_bytes, false);
      bool &active = w_active_ref(issue.port);
      auto &current = w_current_ref(issue.port);
      auto &latch = aw_latch(issue.port);
      active = true;
      current = {};
      current.axi_id = axi_id;
      current.master_id = static_cast<uint8_t>(idx);
      current.orig_id = write_ports[idx].req.id;
      current.port = issue.port;
      current.addr = issue.addr;
      current.wdata = issue.wdata;
      current.wstrb = issue.wstrb;
      current.total_beats = calc_burst_len(issue.total_size) + 1;
      current.beats_sent = 0;
      current.aw_done = false;
      current.w_done = false;
      current.to_llc_mem = false;
      current.llc_victim_write = false;
      w_current_master_ref(issue.port) = idx;

      latch.valid = true;
      latch.port = current.port;
      latch.addr = current.addr;
      latch.len = current.total_beats - 1;
      latch.size = kMmioAxiSize;
      latch.burst = sim_ddr::AXI_BURST_INCR;
      latch.id = current.axi_id;

      write_req_accepted[idx] = true;
      w_arb_rr_idx = (idx + 1) % NUM_WRITE_MASTERS;
      break;
    }

    if (llc.io.ext_out.mem.write_req_valid &&
        llc.io.ext_in.mem.write_req_ready &&
        can_issue_external_write(llc.io.ext_out.mem.write_req_addr)) {
      const uint8_t issue_size =
          static_cast<uint8_t>(llc.io.ext_out.mem.write_req_size);
      const DownstreamPort issue_port = classify_downstream_port(
          llc.io.ext_out.mem.write_req_addr, issue_size);
      if (!w_active_ref(issue_port) && !aw_latch(issue_port).valid &&
          downstream_request_supported(issue_port, issue_size)) {
        const auto issue = make_downstream_write_issue(
            issue_port, llc.io.ext_out.mem.write_req_addr, issue_size,
            llc.io.ext_out.mem.write_req_data,
            llc.io.ext_out.mem.write_req_strobe, llc_config.line_bytes,
            llc.io.ext_out.mem.write_req_mode2_ddr_aligned);
        bool &active = w_active_ref(issue.port);
        auto &current = w_current_ref(issue.port);
        auto &latch = aw_latch(issue.port);
        active = true;
        current = {};
        current.axi_id = llc.io.ext_out.mem.write_req_id & kAxiIdMask;
        current.master_id = 0;
        current.port = issue.port;
        current.addr = issue.addr;
        current.wdata = issue.wdata;
        current.wstrb = issue.wstrb;
        current.orig_id = llc.io.ext_out.mem.write_req_id;
        current.total_beats = calc_burst_len(issue.total_size) + 1;
        current.beats_sent = 0;
        current.aw_done = false;
        current.w_done = false;
        current.to_llc_mem = true;
        current.llc_victim_write =
            llc.io.regs.victim_wb_valid_r || llc.io.reg_write.victim_wb_valid_r;
        w_current_master_ref(issue.port) = -1;

        latch.valid = true;
        latch.port = current.port;
        latch.addr = current.addr;
        latch.len = current.total_beats - 1;
        latch.size = current.port == DownstreamPort::MMIO ? kMmioAxiSize
                                                          : kDownstreamAxiSize;
        latch.burst = sim_ddr::AXI_BURST_INCR;
        latch.id = current.axi_id;
      }
    }

    for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
      auto &llc_w_io = downstream_io(port);
      auto &latch = aw_latch(port);
      auto &current = w_current_ref(port);
      bool &active = w_active_ref(port);
      if (llc_w_io.aw.awvalid && llc_w_io.aw.awready) {
        latch.valid = false;
        latch.port = port;
        current.aw_done = true;
      }

      if (active && llc_w_io.w.wvalid && llc_w_io.w.wready) {
        current.beats_sent++;
        if (llc_w_io.w.wlast) {
          current.w_done = true;
        }
      }
    }

    auto handle_llc_b = [&](DownstreamPort port, sim_ddr::SimDDR_IO_t &io) {
      if (!io.b.bvalid || !io.b.bready) {
        return;
      }
      auto &current = w_current_ref(port);
      bool &active = w_active_ref(port);
      if (active && current.w_done &&
          current.axi_id == static_cast<uint8_t>(io.b.bid & kAxiIdMask)) {
        if (current.to_llc_mem) {
          llc_mem_write_resp_valid_ = true;
          llc_mem_write_resp_ = io.b.bresp;
        } else if (current.master_id < NUM_WRITE_MASTERS) {
          enqueue_write_resp(current.master_id, current.orig_id, io.b.bresp);
        }
        active = false;
        current = {};
        w_current_master_ref(port) = -1;
        return;
      }
      if (llc_mem_ignored_b_count_ > 0) {
        llc_mem_ignored_b_count_--;
      } else if (DEBUG) {

      }
    };
    handle_llc_b(DownstreamPort::DDR, axi_ddr_io);
    handle_llc_b(DownstreamPort::MMIO, axi_mmio_io);

    if (llc_mem_write_resp_valid_prev && llc.io.ext_out.mem.write_resp_ready) {
      llc_mem_write_resp_valid_ = false;
      llc_mem_write_resp_ = 0;
      for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
        if (w_active_ref(port)) {
          w_active_ref(port) = false;
          w_current_ref(port) = {};
          w_current_master_ref(port) = -1;
        }
      }
    }

    if (llc_synth_read_resp_valid_prev &&
        llc.io.ext_out.mem.read_resp_ready) {
      llc_synth_read_resp_valid_ = false;
      llc_synth_read_resp_id_ = 0;
      llc_synth_read_resp_data_.clear();
    }

    const uint8_t llc_read_req_size =
        static_cast<uint8_t>(llc.io.ext_out.mem.read_req_size);
    const DownstreamPort llc_read_req_port = classify_downstream_port(
        llc.io.ext_out.mem.read_req_addr, llc_read_req_size);
    if (llc.io.ext_out.mem.read_req_valid &&
        llc.io.ext_in.mem.read_req_ready &&
        unsupported_mmio_request(llc_read_req_port, llc_read_req_size)) {
      llc_synth_read_resp_valid_ = true;
      llc_synth_read_resp_id_ = llc.io.ext_out.mem.read_req_id;
      llc_synth_read_resp_data_ =
          load_legacy_mmio_backing_line(llc.io.ext_out.mem.read_req_addr);
    }

    const uint8_t llc_write_req_size =
        static_cast<uint8_t>(llc.io.ext_out.mem.write_req_size);
    const DownstreamPort llc_write_req_port = classify_downstream_port(
        llc.io.ext_out.mem.write_req_addr, llc_write_req_size);
    if (llc.io.ext_out.mem.write_req_valid &&
        llc.io.ext_in.mem.write_req_ready &&
        unsupported_mmio_request(llc_write_req_port, llc_write_req_size)) {
      store_legacy_mmio_backing_line(llc.io.ext_out.mem.write_req_addr,
                                     llc.io.ext_out.mem.write_req_data,
                                     llc.io.ext_out.mem.write_req_strobe,
                                     llc_write_req_size);
      llc_mem_write_resp_valid_ = true;
      llc_mem_write_resp_ = sim_ddr::AXI_RESP_OKAY;
    }

    llc.seq();
    if (!llc.io.ext_out.mem.read_req_valid) {
      llc_mem_read_issue_seen_valid_ = false;
      llc_mem_read_issue_seen_id_ = 0;
      llc_mem_read_issue_seen_addr_ = 0;
      llc_mem_read_issue_seen_size_ = 0;
    }
    if (requested_diff) {
      reconfig_pending_ = true;
      reconfig_target_mode_ = req_mode;
      reconfig_target_offset_ = req_offset;
    } else if (reconfig_pending_) {
      reconfig_pending_ = false;
      reconfig_target_mode_ = runtime_mode_;
      reconfig_target_offset_ = llc_mapped_offset_;
    }
    if (reconfig_pending_ && llc.io.ext_out.mem.invalidate_all_accepted) {
      runtime_mode_ = reconfig_target_mode_;
      llc_mapped_offset_ = reconfig_target_offset_;
      reconfig_pending_ = false;
      reconfig_target_mode_ = runtime_mode_;
      reconfig_target_offset_ = llc_mapped_offset_;
    }
    assert_llc_consumed_reads();
    return;
  }

  // Accept new write request into pending queue.
  for (int k = 0; k < NUM_WRITE_MASTERS; k++) {
    int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
    if (!write_ports[idx].req.valid || !write_req_fire_c[idx] ||
        !can_accept_write_now()) {
      continue;
    }
    WritePendingTxn txn{};
    txn.master_id = idx;
    txn.orig_id = write_ports[idx].req.id;
    const uint32_t req_addr = static_cast<uint32_t>(write_ports[idx].req.addr);
    const uint8_t req_total_size =
        static_cast<uint8_t>(write_ports[idx].req.total_size);
    const DownstreamPort req_port =
        classify_downstream_port(req_addr, req_total_size);
    if (!downstream_request_supported(req_port, req_total_size)) {
      continue;
    }
    txn.axi_id = alloc_write_axi_id(req_port);
    if (txn.axi_id == kInvalidAxiWriteId) {
      break;
    }
    const auto issue = make_downstream_write_issue(
        req_port, req_addr, req_total_size, write_ports[idx].req.wdata,
        write_ports[idx].req.wstrb,
        llc_config.line_bytes, false);
    txn.port = issue.port;
    txn.addr = issue.addr;
    txn.wdata = issue.wdata;
    txn.wstrb = issue.wstrb;
    txn.total_beats = calc_burst_len(issue.total_size) + 1;
    txn.beats_sent = 0;
    txn.aw_done = false;
    txn.w_done = false;
    w_pending.push_back(txn);
    if (focus_write_line(txn.addr)) {
      const std::string wstrb_hex =
          axi_compat::hex_string(txn.wstrb, kUpstreamStrbBytes);


    }
    write_req_accepted[idx] = true;
    w_arb_rr_idx = (idx + 1) % NUM_WRITE_MASTERS;
    break;
  }

  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    auto &seq_aw_io = downstream_io(port);
    auto &latch = aw_latch(port);
    if (seq_aw_io.aw.awvalid && seq_aw_io.aw.awready) {
      const uint8_t axi_id = latch.valid ? latch.id : seq_aw_io.aw.awid;
      const int pending_idx = find_write_pending_by_axi_id(port, axi_id);
      if (pending_idx >= 0) {
        w_pending[static_cast<size_t>(pending_idx)].aw_done = true;
        const auto &txn = w_pending[static_cast<size_t>(pending_idx)];
      }
      latch.valid = false;
      latch.port = port;
    }

    auto &seq_w_io = downstream_io(port);
    auto &current = w_current_ref(port);
    bool &active = w_active_ref(port);
    if (active && seq_w_io.w.wvalid && seq_w_io.w.wready) {
      const int pending_idx =
          find_write_pending_by_axi_id(port, current.axi_id);
      if (pending_idx >= 0) {
        auto &txn = w_pending[static_cast<size_t>(pending_idx)];
        if (focus_write_line(txn.addr)) {
          const uint32_t beat_addr =
              txn.addr +
              static_cast<uint32_t>(txn.beats_sent) * kDownstreamBeatBytes;
          const std::string beat_hex =
              axi_compat::hex_string(seq_w_io.w.wdata, kDownstreamBeatBytes);
          const std::string strobe_hex =
              axi_compat::hex_string(seq_w_io.w.wstrb,
                                     sim_ddr::AXI_STRB_STORAGE_BYTES);

        }
        txn.beats_sent++;
        current.beats_sent = txn.beats_sent;
      }
      if (seq_w_io.w.wlast) {
        if (const int idx =
                find_write_pending_by_axi_id(port, current.axi_id);
            idx >= 0) {
          w_pending[static_cast<size_t>(idx)].w_done = true;
          const auto &txn = w_pending[static_cast<size_t>(idx)];
        }
        current.w_done = true;
        active = false;
        current = {};
        w_current_master_ref(port) = -1;
      }
    }
  }

  // B handshake
  auto handle_b_handshake = [&](DownstreamPort port, sim_ddr::SimDDR_IO_t &io) {
    if (!io.b.bvalid || !io.b.bready) {
      return;
    }
    const int pending_idx = find_write_pending_by_axi_id(
        port, static_cast<uint8_t>(io.b.bid & kAxiIdMask));
    if (pending_idx >= 0) {
      const auto txn = w_pending[static_cast<size_t>(pending_idx)];
      if (txn.master_id < NUM_WRITE_MASTERS) {
        enqueue_write_resp(txn.master_id, txn.orig_id, io.b.bresp);
      }
      w_pending.erase(w_pending.begin() + pending_idx);
    }
  };
  handle_b_handshake(DownstreamPort::DDR, axi_ddr_io);
  handle_b_handshake(DownstreamPort::MMIO, axi_mmio_io);

  // Upstream response handshake.
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (write_ports[master].resp.valid && write_ports[master].resp.ready) {
      w_resp_valid[master] = false;
      w_resp_id[master] = 0;
      w_resp_resp[master] = 0;
    }
  }
  promote_write_resp_queue();

  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    auto &latch = aw_latch(port);
    if (!latch.valid) {
      const int next_aw_idx = find_next_aw_pending(port);
      if (next_aw_idx >= 0) {
        const auto &txn = w_pending[static_cast<size_t>(next_aw_idx)];
        if (can_issue_external_write(txn.addr)) {
          latch.valid = true;
          latch.port = port;
          latch.addr = txn.addr;
          latch.len = txn.total_beats - 1;
          latch.size =
              port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize;
          latch.burst = sim_ddr::AXI_BURST_INCR;
          latch.id = txn.axi_id;
        }
      }
    }
  }

  refresh_non_llc_w_active(DownstreamPort::DDR);
  refresh_non_llc_w_active(DownstreamPort::MMIO);

  llc.seq();
  if (!llc.io.ext_out.mem.read_req_valid) {
    llc_mem_read_issue_seen_valid_ = false;
    llc_mem_read_issue_seen_id_ = 0;
    llc_mem_read_issue_seen_addr_ = 0;
    llc_mem_read_issue_seen_size_ = 0;
  }
  if (requested_diff) {
    reconfig_pending_ = true;
    reconfig_target_mode_ = req_mode;
    reconfig_target_offset_ = req_offset;
  } else if (reconfig_pending_) {
    reconfig_pending_ = false;
    reconfig_target_mode_ = runtime_mode_;
    reconfig_target_offset_ = llc_mapped_offset_;
  }
  if (reconfig_pending_ && llc.io.ext_out.mem.invalidate_all_accepted) {
    runtime_mode_ = reconfig_target_mode_;
    llc_mapped_offset_ = reconfig_target_offset_;
    reconfig_pending_ = false;
    reconfig_target_mode_ = runtime_mode_;
    reconfig_target_offset_ = llc_mapped_offset_;
  }
  assert_llc_consumed_reads();
}


// ============================================================================
// Helpers
// ============================================================================
uint8_t AXI_InterconnectRef::calc_burst_len(uint8_t total_size) {
  return axi_dual_port_axi_len_for_beat_bytes(total_size, kDownstreamBeatBytes);
}

} // namespace axi_interconnect

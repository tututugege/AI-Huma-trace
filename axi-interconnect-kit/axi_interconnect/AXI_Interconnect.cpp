/**
 * @file AXI_Interconnect.cpp
 * @brief AXI-Interconnect Layer Implementation
 *
 * AXI Protocol Compliance:
 * - AR/AW valid signals are latched until ready handshake
 * - Upstream req_valid can be deasserted without affecting AXI valid
 */

#include "AXI_Interconnect.h"
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

AXI_LLC_ReadReqIn_t to_llc_read_req(const LlcUpstreamReqLatch &r) {
  AXI_LLC_ReadReqIn_t o;
  o.valid = true;
  o.addr = r.addr;
  o.total_size = r.total_size;
  o.id = r.id;
  o.bypass = r.bypass;
  o.direct_mapped = r.direct_mapped;
  o.mode2_ddr_aligned = r.mode2_ddr_aligned;
  return o;
}

AXI_LLC_WriteReqIn_t to_llc_write_req(const LlcUpstreamWriteReqLatch &w) {
  AXI_LLC_WriteReqIn_t o;
  o.valid = true;
  o.addr = w.addr;
  o.total_size = w.total_size;
  o.id = w.id;
  o.wdata = w.wdata;
  o.wstrb = w.wstrb;
  o.bypass = w.bypass;
  o.direct_mapped = w.direct_mapped;
  o.mode2_ddr_aligned = w.mode2_ddr_aligned;
  return o;
}

inline bool write_size_supported(uint8_t total_size) {
  return (static_cast<uint16_t>(total_size) + 1u) <= MAX_WRITE_TRANSACTION_BYTES;
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
bool AXI_Interconnect::llc_enabled() const {
  const bool mode_uses_llc = runtime_mode_ != 0u;
  return mode_uses_llc && llc_config.enable && llc_config.valid();
}

// Per-port state: DDR is index 0, MMIO index 1.
sim_ddr::SimDDR_IO_t &AXI_Interconnect::downstream_io(DownstreamPort port) {
  return port == DownstreamPort::DDR ? axi_ddr_io : axi_mmio_io;
}

const sim_ddr::SimDDR_IO_t &AXI_Interconnect::downstream_io(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? axi_ddr_io : axi_mmio_io;
}

ARLatch_t &AXI_Interconnect::ar_latch(DownstreamPort port) {
  return port == DownstreamPort::DDR ? ar_latched : ar_latched_mmio;
}

AWLatch_t &AXI_Interconnect::aw_latch(DownstreamPort port) {
  return port == DownstreamPort::DDR ? aw_latched : aw_latched_mmio;
}

const AWLatch_t &AXI_Interconnect::aw_latch(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? aw_latched : aw_latched_mmio;
}

bool &AXI_Interconnect::w_active_ref(DownstreamPort port) { return port == DownstreamPort::DDR ? w_active : w_active_mmio; }

bool AXI_Interconnect::w_active_ref(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? w_active : w_active_mmio;
}

WritePendingTxn &AXI_Interconnect::w_current_ref(DownstreamPort port) {
  return port == DownstreamPort::DDR ? w_current : w_current_mmio;
}

const WritePendingTxn &AXI_Interconnect::w_current_ref(DownstreamPort port) const {
  return port == DownstreamPort::DDR ? w_current : w_current_mmio;
}

int &AXI_Interconnect::w_current_master_ref(DownstreamPort port) {
  return port == DownstreamPort::DDR ? w_current_master : w_current_master_mmio;
}

bool AXI_Interconnect::any_ar_latched() const { return ar_latched.valid || ar_latched_mmio.valid; }
bool AXI_Interconnect::any_aw_latched() const { return aw_latched.valid || aw_latched_mmio.valid; }
bool AXI_Interconnect::any_w_active() const { return w_active || w_active_mmio; }
bool AXI_Interconnect::external_write_busy() const { return any_w_active() || any_aw_latched(); }

uint8_t AXI_Interconnect::write_resp_buffer_count(uint8_t master) const {
  if (master >= NUM_WRITE_MASTERS) {
    return 0;
  }
  return static_cast<uint8_t>((w_resp_valid[master] ? 1u : 0u) +
                              w_resp_queue[master].size());
}

bool AXI_Interconnect::write_resp_buffer_has_space(uint8_t master) const {
  return master < NUM_WRITE_MASTERS &&
         write_resp_buffer_count(master) < MAX_WRITE_OUTSTANDING;
}

bool AXI_Interconnect::any_write_resp_buffered() const {
  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    if (w_resp_valid[master] || !w_resp_queue[master].empty()) {
      return true;
    }
  }
  return false;
}

void AXI_Interconnect::enqueue_write_resp(uint8_t master, uint8_t id,
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

void AXI_Interconnect::promote_write_resp_queue() {
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

void AXI_Interconnect::sample_runtime_controls() {
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

uint8_t AXI_Interconnect::requested_mode() const {
  const uint8_t encoded = static_cast<uint8_t>(mode) & 0x3u;
  return encoded == 3u ? 0u : encoded;
}

uint32_t AXI_Interconnect::requested_llc_mapped_offset() const {
  const uint32_t requested = static_cast<uint32_t>(llc_mapped_offset);
  return (!llc_mapped_offset_override_ && requested == 0) ? kDefaultMappedLlcOffset
                                                         : requested;
}

bool AXI_Interconnect::requested_config_differs_from_active() const {
  const uint8_t req_mode = requested_mode();
  const uint32_t req_offset = requested_llc_mapped_offset();
  return req_mode != runtime_mode_ ||
         (req_mode == 2u && req_offset != llc_mapped_offset_);
}

bool AXI_Interconnect::mode_transition_needs_flush() const {
  return reconfig_pending_ || requested_config_differs_from_active();
}

bool AXI_Interconnect::mode_transition_invalidate_requested() const {
  return reconfig_pending_ && requested_config_differs_from_active() &&
         reconfig_path_quiescent();
}

bool AXI_Interconnect::invalidate_all_requested() const {
  return llc_invalidate_all_req_ || mode_transition_invalidate_requested();
}

bool AXI_Interconnect::request_in_mapped_window(uint32_t addr,
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

bool AXI_Interconnect::request_uses_direct_mapped_llc(uint32_t addr,
                                                      uint8_t total_size) const {
  return runtime_mode_ == 2u && request_in_mapped_window(addr, total_size);
}

DownstreamPort AXI_Interconnect::classify_downstream_port(
    uint32_t addr, uint8_t total_size) const {
  const AxiDualPortRouteSupport route =
      axi_dual_port_route_support(addr, total_size, kDdrAddressBase);
  if (route.ddr_port) {
    return DownstreamPort::DDR;
  }
  return DownstreamPort::MMIO;
}

bool AXI_Interconnect::request_uses_ddr_port(uint32_t addr,
                                             uint8_t total_size) const {
  if (request_uses_direct_mapped_llc(addr, total_size)) {
    return false;
  }
  return classify_downstream_port(addr, total_size) == DownstreamPort::DDR;
}

bool AXI_Interconnect::request_uses_mmio_port(uint32_t addr,
                                              uint8_t total_size) const {
  if (request_uses_direct_mapped_llc(addr, total_size)) {
    return false;
  }
  return classify_downstream_port(addr, total_size) == DownstreamPort::MMIO;
}

uint32_t AXI_Interconnect::translate_llc_addr(uint32_t addr,
                                              uint8_t total_size) const {
  if (!request_uses_direct_mapped_llc(addr, total_size)) {
    return addr;
  }
  return static_cast<uint32_t>(
      static_cast<uint64_t>(addr) - static_cast<uint64_t>(llc_mapped_offset_));
}

bool AXI_Interconnect::effective_llc_bypass(uint32_t addr, uint8_t total_size,
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

// Mapped mode: DDR requests outside the mapped window are tagged so the LLC issues them line-aligned.
bool AXI_Interconnect::mode2_ddr_aligned(uint32_t addr, uint8_t total_size) const {
  return runtime_mode_ == 2u && request_uses_ddr_port(addr, total_size);  // (the mapped window is never a DDR request)
}

void AXI_Interconnect::init() {
  sample_runtime_controls();
  llc.set_config(llc_config);
  llc.reset();
  r_arb_rr_idx = 0;
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
  ar_issue_c[0] = {};
  ar_issue_c[1] = {};
}

namespace {
// Lowest AXI id not marked in `used`, or `none`.
template <size_t N> uint8_t first_free_id(const bool (&used)[N], uint8_t none) {
  for (size_t id = 0; id < N; ++id)
    if (!used[id]) return uint8_t(id);
  return none;
}
constexpr uint32_t kIdSpace = 1u << sim_ddr::AXI_ID_WIDTH;
}  // namespace

uint8_t AXI_Interconnect::alloc_read_axi_id() const {
  bool used[kIdSpace] = {false};
  for (const auto &txn : r_pending) used[txn.axi_id & (kIdSpace - 1u)] = true;
  if (ar_latched.valid) used[ar_latched.id & (kIdSpace - 1u)] = true;
  if (ar_latched_mmio.valid) used[ar_latched_mmio.id & (kIdSpace - 1u)] = true;
  return first_free_id(used, kInvalidAxiReadId);
}

uint8_t AXI_Interconnect::count_master_read_pending(uint8_t master_id) const {
  uint8_t count = 0;
  for (const auto &txn : r_pending) count += txn.master_id == master_id;
  count += ar_latched.valid && ar_latched.master_id == master_id;
  count += ar_latched_mmio.valid && ar_latched_mmio.master_id == master_id;
  return count;
}

uint8_t AXI_Interconnect::count_total_read_inflight() const {
  return static_cast<uint8_t>(r_pending.size() + (ar_latched.valid ? 1 : 0) + (ar_latched_mmio.valid ? 1 : 0));
}

uint8_t AXI_Interconnect::alloc_write_axi_id() const {
  bool used[kIdSpace] = {false};
  for (const auto &txn : w_pending) used[txn.axi_id & (kIdSpace - 1u)] = true;
  if (w_active) used[w_current.axi_id & (kIdSpace - 1u)] = true;
  if (w_active_mmio) used[w_current_mmio.axi_id & (kIdSpace - 1u)] = true;
  if (aw_latched.valid) used[aw_latched.id & (kIdSpace - 1u)] = true;
  if (aw_latched_mmio.valid) used[aw_latched_mmio.id & (kIdSpace - 1u)] = true;
  return first_free_id(used, kInvalidAxiWriteId);
}

// Same, among the transactions of one downstream port.
uint8_t AXI_Interconnect::alloc_write_axi_id(DownstreamPort port) const {
  bool used[kIdSpace] = {false};
  for (const auto &txn : w_pending)
    if (txn.port == port) used[txn.axi_id & (kIdSpace - 1u)] = true;
  if (w_active_ref(port)) used[w_current_ref(port).axi_id & (kIdSpace - 1u)] = true;
  if (aw_latch(port).valid) used[aw_latch(port).id & (kIdSpace - 1u)] = true;
  return first_free_id(used, kInvalidAxiWriteId);
}

bool AXI_Interconnect::can_accept_write_now() const {
  return !llc_enabled() &&
         w_pending.size() < MAX_WRITE_OUTSTANDING &&
         alloc_write_axi_id() != kInvalidAxiWriteId;
}

uint32_t AXI_Interconnect::count_llc_write_pending() const {
  uint32_t count = 0;
  count += (llc_core_req_stage_.valid && llc_core_req_stage_.is_write) ? 1u : 0u;
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
    count += llc_upstream_write_req[i].valid ? 1u : 0u;
    count += static_cast<uint32_t>(llc_upstream_write_q[i].size());
  }
  return count;
}

bool AXI_Interconnect::non_llc_path_quiescent() const {
  if (any_ar_latched() || !r_pending.empty() || any_aw_latched() || any_w_active() || !w_pending.empty()) return false;
  for (int i = 0; i < NUM_READ_MASTERS; ++i)
    if (req_ready_r[i] || read_req_hold[i].valid) return false;
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i)
    if (w_req_ready_r[i] || write_ports[i].req.ready) return false;
  return true;
}

bool AXI_Interconnect::llc_path_quiescent() const {
  if (!llc_enabled()) return true;
  if (any_ar_latched() || !r_pending.empty() || any_w_active() || any_aw_latched() || llc_mem_write_resp_valid_ ||
      llc_mem_ignored_b_count_ != 0 || !w_pending.empty() || llc_core_req_stage_.valid)
    return false;
  const auto &regs = llc.io.regs;
  for (int m = 0; m < NUM_READ_MASTERS; ++m)
    if (req_ready_r[m] || read_req_hold[m].valid || llc_read_resp_valid_[m] || llc_upstream_req[m].valid ||
        regs.read_resp_valid_r[m] || regs.read_resp_q_count_r[m] != 0)
      return false;
  for (int m = 0; m < NUM_WRITE_MASTERS; ++m)
    if (w_req_ready_r[m] || write_ports[m].req.ready || w_resp_valid[m] || !w_resp_queue[m].empty() ||
        llc_upstream_write_req[m].valid || !llc_upstream_write_q[m].empty() || regs.write_ctx[m].valid ||
        regs.write_q_count_r[m] != 0 || regs.write_resp_valid_r[m])
      return false;
  if (regs.lookup_valid_r || regs.victim_wb_valid_r || regs.read_victim_wb_q_count_r != 0 ||
      llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.write_req_valid)
    return false;
  if (llc.regs_quiet()) return true;  // no live MSHR either
  for (uint32_t slot = 0; slot < llc_config.mshr_num && slot < MAX_OUTSTANDING; ++slot)
    if (regs.mshr[slot].valid) return false;
  return true;
}

bool AXI_Interconnect::idle_quiescent() const {
  if (!reconfig_path_quiescent() || reconfig_pending_) return false;
  for (int i = 0; i < NUM_READ_MASTERS; ++i)
    if (read_ports[i].req.valid || read_ports[i].resp.valid) return false;
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i)
    if (write_ports[i].req.valid || write_ports[i].resp.valid || !llc_write_resp_delay[i].empty() ||
        !w_resp_queue[i].empty())
      return false;
  return true;
}

bool AXI_Interconnect::reconfig_path_quiescent() const {
  return llc_enabled() ? llc_path_quiescent() : non_llc_path_quiescent();
}

bool AXI_Interconnect::has_same_line_write_hazard(uint32_t line_addr) const {
  if (!llc_enabled()) return false;
  const auto hits = [&](uint32_t a) { return AXI_LLC::line_addr(llc_config, a) == line_addr; };
  for (int m = 0; m < NUM_WRITE_MASTERS; ++m) {
    if (llc_upstream_write_req[m].valid && hits(llc_upstream_write_req[m].addr)) return true;
    for (const auto &e : llc_upstream_write_q[m])
      if (e.valid && hits(e.addr)) return true;
    for (const auto &e : llc_write_resp_delay[m])
      if (e.entry.line_valid && e.entry.line_addr == line_addr) return true;
    for (const auto &e : w_resp_queue[m])
      if (e.line_valid && e.line_addr == line_addr) return true;
    if (write_ports[m].req.valid && hits(write_ports[m].req.addr)) return true;
  }
  if (llc_core_req_stage_.valid && llc_core_req_stage_.is_write && hits(llc_core_req_stage_.write.addr)) return true;
  const auto &regs = llc.io.regs;
  for (int m = 0; m < NUM_WRITE_MASTERS; ++m) {
    if (regs.write_ctx[m].valid && regs.write_ctx[m].line_addr == line_addr) return true;
    if (regs.write_resp_valid_r[m] && regs.write_resp_line_addr_r[m] == line_addr) return true;
  }
  for (uint8_t m = 0; m < NUM_WRITE_MASTERS; ++m)
    for (uint32_t i = 0; i < regs.write_q_count_r[m] && i < MAX_WRITE_OUTSTANDING; ++i) {
      const auto &e = regs.write_q[m][(regs.write_q_head_r[m] + i) % MAX_WRITE_OUTSTANDING];
      if (e.valid && hits(e.addr)) return true;
    }
  return (w_active && hits(w_current.addr)) || (w_active_mmio && hits(w_current_mmio.addr)) ||
         (aw_latched.valid && hits(aw_latched.addr)) || (aw_latched_mmio.valid && hits(aw_latched_mmio.addr));
}

// A read of `line_addr` through the LLC is in flight (request stage, LLC lookup/MSHR, memory read).
bool AXI_Interconnect::has_same_line_read_hazard(uint32_t line_addr) const {
  if (!llc_enabled()) return false;
  const auto request_hits_line = [&](uint32_t addr, uint8_t total_size) {
    return AXI_LLC::line_addr(llc_config, translate_llc_addr(addr, total_size)) == line_addr;
  };
  const auto hits = [&](uint32_t a) { return AXI_LLC::line_addr(llc_config, a) == line_addr; };
  for (int m = 0; m < NUM_READ_MASTERS; ++m) {
    if (llc_upstream_req[m].valid && request_hits_line(llc_upstream_req[m].addr, llc_upstream_req[m].total_size))
      return true;
    const auto &hold = read_req_hold[m];
    if (hold.valid && !request_uses_mmio_port(hold.addr, hold.total_size) && request_hits_line(hold.addr, hold.total_size))
      return true;
    if (read_ports[m].req.valid) {
      const uint32_t addr = static_cast<uint32_t>(read_ports[m].req.addr);
      const uint8_t size = static_cast<uint8_t>(read_ports[m].req.total_size);
      if (!request_uses_mmio_port(addr, size) && request_hits_line(addr, size)) return true;
    }
  }
  if (llc_core_req_stage_.valid && !llc_core_req_stage_.is_write &&
      request_hits_line(llc_core_req_stage_.read.addr, llc_core_req_stage_.read.total_size))
    return true;
  const auto &regs = llc.io.regs;
  if (regs.lookup_valid_r && !regs.lookup_is_write_r && hits(regs.lookup_addr_r)) return true;
  if (llc.io.ext_out.mem.read_req_valid && hits(llc.io.ext_out.mem.read_req_addr)) return true;
  for (uint32_t slot = 0; slot < llc_config.mshr_num && slot < MAX_OUTSTANDING; ++slot) {
    const auto &e = regs.mshr[slot];
    if (e.valid && !e.is_write && e.line_addr == line_addr) return true;
  }
  if ((ar_latched.valid && ar_latched.to_llc && hits(ar_latched.upstream_addr)) ||
      (ar_latched_mmio.valid && ar_latched_mmio.to_llc && hits(ar_latched_mmio.upstream_addr)))
    return true;
  for (const auto &txn : r_pending)
    if (txn.to_llc && txn.beats_done < txn.total_beats && hits(txn.upstream_addr)) return true;
  return false;
}

uint32_t AXI_Interconnect::external_hazard_line_addr(uint32_t addr) const {
  constexpr uint32_t kHazardBytes = MAX_WRITE_TRANSACTION_BYTES;
  static_assert((kHazardBytes & (kHazardBytes - 1u)) == 0,
                "external hazard granule must be a power of two");
  return addr & ~(kHazardBytes - 1u);
}

bool AXI_Interconnect::has_external_pending_read_hazard(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  const auto hits = [&](uint32_t a) { return external_hazard_line_addr(a) == line; };
  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    const auto &meta = ar_issue_c[port_index(port)];
    if (downstream_io(port).ar.arvalid && meta.valid && hits(meta.upstream_addr)) return true;
  }
  if ((ar_latched.valid && hits(ar_latched.upstream_addr)) || (ar_latched_mmio.valid && hits(ar_latched_mmio.upstream_addr)))
    return true;
  for (const auto &txn : r_pending)
    if (txn.beats_done < txn.total_beats && hits(txn.upstream_addr)) return true;
  return false;
}

bool AXI_Interconnect::has_external_pending_write_hazard(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  const auto hits = [&](uint32_t a) { return external_hazard_line_addr(a) == line; };
  if ((aw_latched.valid && hits(aw_latched.addr)) || (aw_latched_mmio.valid && hits(aw_latched_mmio.addr)) ||
      (w_active && hits(w_current.addr)) || (w_active_mmio && hits(w_current_mmio.addr)))
    return true;
  for (const auto &txn : w_pending)
    if (hits(txn.addr)) return true;
  return false;
}

bool AXI_Interconnect::has_direct_read_response(uint8_t master_id) const {
  for (const auto &txn : r_pending)
    if (!txn.to_llc && txn.master_id == master_id && txn.beats_done == txn.total_beats) return true;
  return false;
}

bool AXI_Interconnect::can_issue_external_read(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  const AxiDualPortIssueGateResult gate = axi_dual_port_issue_gate(
      true, true, line, false, has_external_pending_write_hazard(addr), false,
      true, 0, false, false);
  return gate.axi_arvalid;
}

bool AXI_Interconnect::can_issue_external_write(uint32_t addr) const {
  const uint32_t line = external_hazard_line_addr(addr);
  const AxiDualPortIssueGateResult gate = axi_dual_port_issue_gate(
      false, true, 0, false, false, true, true, line, false,
      has_external_pending_read_hazard(addr));
  return gate.axi_awvalid;
}

int AXI_Interconnect::find_write_pending_by_axi_id(DownstreamPort port, uint8_t axi_id) const {
  for (size_t i = 0; i < w_pending.size(); ++i)
    if (w_pending[i].port == port && w_pending[i].axi_id == axi_id) return static_cast<int>(i);
  return -1;
}

int AXI_Interconnect::find_next_aw_pending() const {
  for (size_t i = 0; i < w_pending.size(); ++i)
    if (!w_pending[i].aw_done) return static_cast<int>(i);
  return -1;
}

int AXI_Interconnect::find_next_aw_pending(DownstreamPort port) const {
  for (size_t i = 0; i < w_pending.size(); ++i)
    if (!w_pending[i].aw_done && w_pending[i].port == port) return static_cast<int>(i);
  return -1;
}

int AXI_Interconnect::find_next_w_pending() const {
  for (size_t i = 0; i < w_pending.size(); ++i)
    if (w_pending[i].aw_done && !w_pending[i].w_done) return static_cast<int>(i);
  return -1;
}

int AXI_Interconnect::find_next_w_pending(DownstreamPort port) const {
  for (size_t i = 0; i < w_pending.size(); ++i)
    if (w_pending[i].port == port && w_pending[i].aw_done && !w_pending[i].w_done) return static_cast<int>(i);
  return -1;
}

void AXI_Interconnect::refresh_non_llc_w_active(DownstreamPort port) {
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

bool AXI_Interconnect::can_accept_read_master(uint8_t master_id) const {
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

// The same (master, id) is already in flight: responses are matched by id, so it must wait.
bool AXI_Interconnect::has_read_id_conflict(uint8_t master_id, uint8_t orig_id) const {
  for (const auto *latch : {&ar_latched, &ar_latched_mmio})
    if (latch->valid && !latch->to_llc && latch->master_id == master_id && latch->orig_id == orig_id) return true;
  for (const auto &txn : r_pending)
    if (txn.master_id == master_id && txn.orig_id == orig_id) return true;
  if (!llc_enabled()) return false;
  const auto &regs = llc.io.regs;
  if (llc_core_req_stage_.valid && !llc_core_req_stage_.is_write && llc_core_req_stage_.master == master_id &&
      llc_core_req_stage_.read.id == orig_id)
    return true;
  if (llc_upstream_req[master_id].valid && llc_upstream_req[master_id].id == orig_id) return true;
  if (llc_read_resp_valid_[master_id] && llc_read_resp_id_[master_id] == orig_id) return true;
  if (regs.read_resp_valid_r[master_id] && regs.read_resp_id_r[master_id] == orig_id) return true;
  if (regs.lookup_valid_r && regs.lookup_master_r == master_id && regs.lookup_id_r == orig_id) return true;
  for (uint32_t slot = 0; slot < llc_config.mshr_num && slot < MAX_OUTSTANDING; ++slot) {
    const auto &e = regs.mshr[slot];
    if (e.valid && !e.is_prefetch && e.master == master_id && e.id == orig_id) return true;
  }
  return false;
}

bool AXI_Interconnect::can_issue_llc_read_req() const {
  return !any_ar_latched() && count_total_read_inflight() < MAX_OUTSTANDING &&
         alloc_read_axi_id() != kInvalidAxiReadId;
}

void AXI_Interconnect::prepare_llc_inputs(bool sample_upstream_ready) {
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
      llc.io.ext_in.upstream.read_req[master] = to_llc_read_req(llc_core_req_stage_.read);
    }
    // The shared-fabric model adds a registered response boundary after the LLC
    // core. The C++ core keeps fresh responses visible for one full cycle;
    // consume them into the boundary only after that fresh cycle and only when the
    // boundary slot is free.
    llc.io.ext_in.upstream.read_resp[master].ready =
        sample_upstream_ready && !llc_read_resp_valid_[master] &&
        !llc.io.regs.read_resp_fresh_r[master] &&
        !has_direct_read_response(static_cast<uint8_t>(master));
  }

  for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
    any_upstream_req_visible = any_upstream_req_visible ||
                               llc_upstream_write_req[master].valid ||
                               !llc_upstream_write_q[master].empty() ||
                               write_ports[master].req.valid;
    if (llc_core_req_stage_.valid && llc_core_req_stage_.is_write &&
        llc_core_req_stage_.master == master) {
      llc.io.ext_in.upstream.write_req[master] = to_llc_write_req(llc_core_req_stage_.write);
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
void AXI_Interconnect::comb_outputs() {
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
void AXI_Interconnect::comb_inputs() {
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

// Per-cycle scratch of the read arbiter: which AR channels and AXI ids are taken by reads issued earlier in this
// comb pass, and how many reads each master has started.
uint8_t AXI_Interconnect::alloc_comb_read_axi_id(DownstreamPort port) {
  bool used[1u << sim_ddr::AXI_ID_WIDTH] = {false};
  for (const auto &txn : r_pending) {
    if (txn.port == port) used[txn.axi_id & kAxiIdMask] = true;
  }
  const auto &latch = ar_latch(port);
  if (latch.valid) used[latch.id & kAxiIdMask] = true;
  const uint8_t port_idx = port_index(port);
  for (uint8_t id = 0; id < (1u << sim_ddr::AXI_ID_WIDTH); ++id) {
    if (!used[id] && !read_comb_.id_used[port_idx][id]) {
      read_comb_.id_used[port_idx][id] = true;
      return id;
    }
  }
  return kInvalidAxiReadId;
}

bool AXI_Interconnect::can_accept_read_master_comb(uint8_t master_id) const {
  if (master_id >= NUM_READ_MASTERS) return false;
  const auto &started = read_comb_.master_count;
  return count_total_read_inflight() + started[0] + started[1] + started[2] + started[3] < MAX_OUTSTANDING &&
         count_master_read_pending(master_id) + started[master_id] < MAX_READ_OUTSTANDING_PER_MASTER;
}

// Drive a downstream AR; the response is matched back through `meta`.
void AXI_Interconnect::drive_read_ar(DownstreamPort port, uint32_t addr, uint8_t total_size, bool mode2_ddr_aligned,
                                     uint8_t axi_id, ARIssueMeta_t meta) {
  const auto issue = make_downstream_read_issue(port, addr, total_size, llc_config.line_bytes, mode2_ddr_aligned);
  meta.resp_extract_from_aligned_beat = issue.extract_from_aligned_beat;
  const int pidx = port_index(issue.port);
  auto &ar_io = downstream_io(issue.port);
  ar_io.ar.arvalid = true;
  ar_io.ar.araddr = issue.addr;
  ar_io.ar.arlen = calc_burst_len(issue.total_size);
  ar_io.ar.arsize = issue.port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize;
  ar_io.ar.arburst = sim_ddr::AXI_BURST_INCR;
  ar_io.ar.arid = axi_id;
  ar_issue_c[pidx] = meta;
  ar_issue_c[pidx].valid = true;
  read_comb_.port_busy[pidx] = true;
}

// A master's read goes out on `port`.
void AXI_Interconnect::issue_master_read(int master, DownstreamPort port, uint32_t addr, uint8_t total_size,
                                         uint8_t id, uint8_t axi_id) {
  ARIssueMeta_t meta{};
  meta.upstream_addr = addr;
  meta.upstream_total_size = total_size;
  meta.master_id = master;
  meta.orig_id = id;
  drive_read_ar(port, addr, total_size, false, axi_id, meta);
}

ReadReqHoldLatch AXI_Interconnect::read_req_snapshot(int master) const {
  const auto &req = read_ports[master].req;
  return ReadReqHoldLatch{true, static_cast<uint32_t>(req.addr), static_cast<uint8_t>(req.total_size),
                          static_cast<uint8_t>(req.id), static_cast<bool>(req.bypass)};
}

// Clear the cycle's combinational outputs and remember which masters saw ready last cycle.
void AXI_Interconnect::read_comb_begin() {
  ar_issue_c[0] = {};
  ar_issue_c[1] = {};
  read_comb_ = {};
  for (int i = 0; i < NUM_READ_MASTERS; ++i) {
    llc_upstream_accept_c[i] = false;
    llc_upstream_capture_c[i] = {};
    read_comb_.ready_curr[i] = req_ready_r[i];
    req_ready_r[i] = false;
  }

  // A ready pulse without a corresponding valid means the request was dropped.
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    if (read_comb_.ready_curr[i] && !read_ports[i].req.valid) read_req_hold[i] = {};
  }

  // Default: don't accept new requests
  for (int i = 0; i < NUM_READ_MASTERS; i++) read_ports[i].req.ready = false;
  axi_ddr_io.ar.arvalid = false;
  axi_mmio_io.ar.arvalid = false;
}

// An AR waiting for arready is held on its channel and keeps the issuing master's ready up.
void AXI_Interconnect::read_replay_latched_ar() {
  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    auto &latch = ar_latch(port);
    if (!latch.valid) continue;
    auto &ar_io = downstream_io(port);
    ar_io.ar.arvalid = true;
    ar_io.ar.araddr = latch.addr;
    ar_io.ar.arlen = latch.len;
    ar_io.ar.arsize = latch.size;
    ar_io.ar.arburst = latch.burst;
    ar_io.ar.arid = latch.id;
    read_comb_.port_busy[port_index(port)] = true;
    if (!latch.to_llc && !latch.accepted_upstream && latch.master_id < NUM_READ_MASTERS) {
      read_ports[latch.master_id].req.ready = true;
      req_ready_r[latch.master_id] = true;
    }
  }
}

// LLC enabled: the LLC's miss read goes downstream (once per distinct request).
void AXI_Interconnect::read_issue_llc_mem_read() {
  const auto &mem = llc.io.ext_out.mem;
  const uint8_t req_id = mem.read_req_id;
  const uint32_t req_addr = mem.read_req_addr;
  const uint8_t req_size = static_cast<uint8_t>(mem.read_req_size);
  const bool already_issued = llc_mem_read_issue_seen_valid_ && llc_mem_read_issue_seen_id_ == req_id &&
                              llc_mem_read_issue_seen_addr_ == req_addr && llc_mem_read_issue_seen_size_ == req_size;
  if (!mem.read_req_valid || already_issued || !can_issue_llc_read_req() || !can_issue_external_read(req_addr)) return;

  const auto port = classify_downstream_port(req_addr, req_size);
  if (read_comb_.port_busy[port_index(port)] || !downstream_request_supported(port, req_size)) return;
  const uint8_t axi_id = alloc_comb_read_axi_id(port);
  if (axi_id == kInvalidAxiReadId) return;

  ARIssueMeta_t meta{};
  meta.from_llc = true;
  meta.llc_mem_id = req_id;
  meta.upstream_addr = req_addr;
  meta.upstream_total_size = mem.read_req_size;
  meta.master_id = 0;
  meta.orig_id = req_id;
  drive_read_ar(port, req_addr, req_size, mem.read_req_mode2_ddr_aligned, axi_id, meta);
  llc_mem_read_issue_seen_valid_ = true;
  llc_mem_read_issue_seen_id_ = req_id;
  llc_mem_read_issue_seen_addr_ = req_addr;
  llc_mem_read_issue_seen_size_ = req_size;
}

// LLC enabled: cached requests are accepted into the LLC path (ready-first, except the L1D read master which may be
// accepted in the cycle its request appears); MMIO reads bypass the LLC and go straight out on the MMIO channel.
void AXI_Interconnect::read_route_masters_llc() {
  for (int master = 0; master < NUM_READ_MASTERS; ++master) {
    const bool stage_busy_for_master =
        llc_core_req_stage_.valid && !llc_core_req_stage_.is_write && llc_core_req_stage_.master == master;
    if (llc_upstream_req[master].valid || stage_busy_for_master || !read_ports[master].req.valid ||
        invalidate_all_requested() || mode_transition_needs_flush()) {
      continue;
    }
    if (has_read_id_conflict(static_cast<uint8_t>(master), static_cast<uint8_t>(read_ports[master].req.id))) continue;

    const uint32_t req_addr = static_cast<uint32_t>(read_ports[master].req.addr);
    const uint8_t req_total_size = static_cast<uint8_t>(read_ports[master].req.total_size);
    const DownstreamPort req_port = classify_downstream_port(req_addr, req_total_size);
    const bool req_supported = downstream_request_supported(req_port, req_total_size);
    if (!request_uses_direct_mapped_llc(req_addr, req_total_size) && !req_supported) continue;

    const bool direct_mmio_read = request_uses_mmio_port(req_addr, req_total_size) && req_supported;
    const bool same_cycle_accept = (master == MASTER_DCACHE_R);
    if (direct_mmio_read) {
      if (!can_issue_external_read(req_addr) || read_comb_.port_busy[port_index(DownstreamPort::MMIO)] ||
          !can_accept_read_master_comb(static_cast<uint8_t>(master))) {
        continue;
      }
      if (!read_comb_.ready_curr[master] && !same_cycle_accept) {
        req_ready_r[master] = true;
        read_ports[master].req.ready = true;
        read_req_hold[master] = read_req_snapshot(master);
        continue;
      }
      const ReadReqHoldLatch cap = read_req_hold[master].valid ? read_req_hold[master] : read_req_snapshot(master);
      if (has_read_id_conflict(static_cast<uint8_t>(master), cap.id)) continue;
      const uint8_t axi_id = alloc_comb_read_axi_id(DownstreamPort::MMIO);
      if (axi_id == kInvalidAxiReadId) continue;
      issue_master_read(master, DownstreamPort::MMIO, cap.addr, cap.total_size, cap.id, axi_id);
      read_ports[master].req.ready = true;
      read_req_hold[master] = {};
      read_comb_.master_count[master]++;
      if (read_comb_.port_busy[0] && read_comb_.port_busy[1]) break;
      continue;
    }

    if (read_comb_.ready_curr[master] || same_cycle_accept) {
      read_ports[master].req.ready = true;
      llc_upstream_accept_c[master] = true;
      llc_upstream_capture_c[master] = LlcUpstreamReqLatch{
          true, translate_llc_addr(req_addr, req_total_size), req_total_size, uint8_t(read_ports[master].req.id),
          effective_llc_bypass(req_addr, req_total_size, static_cast<bool>(read_ports[master].req.bypass)),
          request_uses_direct_mapped_llc(req_addr, req_total_size), mode2_ddr_aligned(req_addr, req_total_size)};
      continue;
    }
    req_ready_r[master] = true;
    read_ports[master].req.ready = true;
  }
}

// LLC disabled: a master that saw ready last cycle completes its handshake and issues the held request.
void AXI_Interconnect::read_complete_ready_handshakes() {
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    if (!read_comb_.ready_curr[i] || !read_ports[i].req.valid) continue;
    ReadReqHoldLatch cap = read_req_snapshot(i);
    const ReadReqHoldLatch &hold = read_req_hold[i];
    if (hold.valid) {
      const bool hold_matches_current = cap.addr == hold.addr && cap.total_size == hold.total_size &&
                                        cap.id == hold.id && cap.bypass == hold.bypass;
      if (hold_matches_current) cap = hold;
      else read_req_hold[i] = cap;
    }
    if (has_read_id_conflict(static_cast<uint8_t>(i), cap.id) || !can_issue_external_read(cap.addr)) continue;
    const auto cap_port = classify_downstream_port(cap.addr, cap.total_size);
    if (!downstream_request_supported(cap_port, cap.total_size) || read_comb_.port_busy[port_index(cap_port)] ||
        !can_accept_read_master_comb(static_cast<uint8_t>(i))) {
      continue;
    }
    const uint8_t axi_id = alloc_comb_read_axi_id(cap_port);
    if (axi_id == kInvalidAxiReadId) continue;
    issue_master_read(i, cap_port, cap.addr, cap.total_size, cap.id, axi_id);
    read_ports[i].req.ready = true;
    read_comb_.master_count[i]++;
  }
}

// LLC disabled: round-robin over the masters with a new request. Data-side masters may issue in the cycle the request
// appears; the others raise ready first and issue next cycle (the ICache relies on that two-cycle contract).
void AXI_Interconnect::read_arbitrate_new() {
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    const int idx = (r_arb_rr_idx + i) % NUM_READ_MASTERS;
    if (!read_ports[idx].req.valid) continue;
    if (!can_accept_read_master(static_cast<uint8_t>(idx))) continue;
    if (has_read_id_conflict(static_cast<uint8_t>(idx), static_cast<uint8_t>(read_ports[idx].req.id))) continue;
    const uint32_t req_addr = static_cast<uint32_t>(read_ports[idx].req.addr);
    if (!can_issue_external_read(req_addr)) continue;
    const uint8_t req_total_size = static_cast<uint8_t>(read_ports[idx].req.total_size);
    const auto req_port = classify_downstream_port(req_addr, req_total_size);
    if (!downstream_request_supported(req_port, req_total_size) || read_comb_.port_busy[port_index(req_port)] ||
        !can_accept_read_master_comb(static_cast<uint8_t>(idx))) {
      continue;
    }
    const uint8_t axi_id = alloc_comb_read_axi_id(req_port);
    if (axi_id == kInvalidAxiReadId) continue;

    if (idx != MASTER_DCACHE_R && !read_comb_.ready_curr[idx]) {
      req_ready_r[idx] = true;
      read_ports[idx].req.ready = true;
      read_req_hold[idx] = read_req_snapshot(idx);
      continue;
    }
    issue_master_read(idx, req_port, req_addr, req_total_size, static_cast<uint8_t>(read_ports[idx].req.id), axi_id);
    read_ports[idx].req.ready = true;
    read_comb_.master_count[idx]++;
    if (read_comb_.port_busy[0] && read_comb_.port_busy[1]) break;
  }
}

void AXI_Interconnect::comb_read_arbiter() {
  read_comb_begin();
  read_replay_latched_ar();
  if (llc_enabled()) {
    read_issue_llc_mem_read();
    read_route_masters_llc();
    return;
  }
  if (mode_transition_needs_flush()) return;
  read_complete_ready_handshakes();
  read_arbitrate_new();
}

void AXI_Interconnect::comb_read_response() {
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
  }
}

// ============================================================================
// Write Request with Latched AW (AXI Compliant)
// ============================================================================

void AXI_Interconnect::drive_aw(DownstreamPort port, uint32_t addr, uint8_t len, uint8_t size, uint8_t burst,
                                uint8_t id) {
  auto &aw_io = downstream_io(port);
  aw_io.aw.awvalid = true;
  aw_io.aw.awaddr = addr;
  aw_io.aw.awlen = len;
  aw_io.aw.awsize = size;
  aw_io.aw.awburst = burst;
  aw_io.aw.awid = id;
}

// An AW waiting for awready is held on its channel.
void AXI_Interconnect::drive_aw_latch(DownstreamPort port) {
  const auto &latch = aw_latch(port);
  if (latch.valid) drive_aw(port, latch.addr, latch.len, latch.size, latch.burst, latch.id);
}

// The W beat currently in flight on `port`.
void AXI_Interconnect::drive_w_current(DownstreamPort port) {
  const auto &current = w_current_ref(port);
  if (!w_active_ref(port) || !current.aw_done || current.w_done) return;
  auto &w_io = downstream_io(port);
  w_io.w.wvalid = true;
  w_io.w.wdata = pack_downstream_write_beat(current.wdata, current.beats_sent);
  w_io.w.wstrb = pack_downstream_write_strobe(current.wstrb, current.beats_sent);
  w_io.w.wlast = (current.beats_sent == current.total_beats - 1);
}

// Ready-first handshake: raise req.ready one cycle before accepting; with `fire` set the previous cycle's ready is
// consumed and the request fires now.
void AXI_Interconnect::write_ready_first(int idx, bool direct_fire) {
  write_ports[idx].req.ready = true;
  if (write_comb_ready_curr_[idx]) {
    if (direct_fire) write_req_fire_c[idx] = true;
  } else {
    w_req_ready_r[idx] = true;
  }
}

// LLC enabled: the LLC's write-back / bypass write goes downstream when its channel is free.
void AXI_Interconnect::write_issue_llc_mem_write() {
  const auto &mem = llc.io.ext_out.mem;
  if (llc_mem_write_resp_valid_ || !mem.write_req_valid || !can_issue_external_write(mem.write_req_addr)) return;
  const uint8_t issue_size = static_cast<uint8_t>(mem.write_req_size);
  const auto issue_port = classify_downstream_port(mem.write_req_addr, issue_size);
  if (w_active_ref(issue_port) || aw_latch(issue_port).valid || !downstream_request_supported(issue_port, issue_size)) {
    return;
  }
  const auto issue = make_downstream_write_issue(issue_port, mem.write_req_addr, issue_size, mem.write_req_data,
                                                 mem.write_req_strobe, llc_config.line_bytes,
                                                 mem.write_req_mode2_ddr_aligned);
  drive_aw(issue.port, issue.addr, calc_burst_len(issue.total_size),
           issue.port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize, sim_ddr::AXI_BURST_INCR,
           mem.write_req_id & kAxiIdMask);
}

// LLC enabled: the first master (round-robin) with an MMIO write bypasses the LLC. Returns whether one was selected,
// even if it has to wait this cycle.
bool AXI_Interconnect::write_route_direct_mmio() {
  for (int k = 0; k < NUM_WRITE_MASTERS; ++k) {
    const int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
    if (!write_ports[idx].req.valid) continue;
    const uint32_t req_addr = static_cast<uint32_t>(write_ports[idx].req.addr);
    if (has_external_pending_read_hazard(req_addr)) continue;
    const uint8_t req_total_size = static_cast<uint8_t>(write_ports[idx].req.total_size);
    const DownstreamPort req_port = classify_downstream_port(req_addr, req_total_size);
    if (!(request_uses_mmio_port(req_addr, req_total_size) && downstream_request_supported(req_port, req_total_size))) {
      continue;
    }
    const bool must_wait = llc.io.ext_out.upstream.write_resp[idx].valid || w_active_ref(req_port) ||
                           aw_latch(req_port).valid || !can_issue_external_write(req_addr);
    if (!must_wait) write_ready_first(idx, true);
    return true;
  }
  return false;
}

// LLC enabled: the first cacheable write that is not blocked by a line invalidate or a pending read enters the LLC
// write queue (ready-first).
void AXI_Interconnect::write_route_llc() {
  for (int k = 0; k < NUM_WRITE_MASTERS; ++k) {
    const int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
    if (!write_ports[idx].req.valid) continue;
    const uint32_t req_addr = static_cast<uint32_t>(write_ports[idx].req.addr);
    const uint8_t req_total_size = static_cast<uint8_t>(write_ports[idx].req.total_size);
    const uint32_t req_line = AXI_LLC::line_addr(llc_config, translate_llc_addr(req_addr, req_total_size));
    const bool blocked_by_line_invalidate =
        llc_invalidate_line_valid_ && req_line == AXI_LLC::line_addr(llc_config, llc_invalidate_line_addr_);
    const bool blocked_by_pending_read =
        !request_uses_mmio_port(req_addr, req_total_size) && has_same_line_read_hazard(req_line);
    if (blocked_by_line_invalidate || blocked_by_pending_read) continue;
    const DownstreamPort req_port = classify_downstream_port(req_addr, req_total_size);
    if (!request_uses_direct_mapped_llc(req_addr, req_total_size) &&
        !downstream_request_supported(req_port, req_total_size)) {
      continue;
    }
    write_ports[idx].req.ready = true;
    if (write_comb_ready_curr_[idx]) {
      llc_upstream_write_accept_c[idx] = true;
      llc_upstream_write_capture_c[idx] = LlcUpstreamWriteReqLatch{
          true, translate_llc_addr(req_addr, req_total_size), req_total_size, uint8_t(write_ports[idx].req.id),
          write_ports[idx].req.wdata, write_ports[idx].req.wstrb,
          effective_llc_bypass(req_addr, req_total_size, static_cast<bool>(write_ports[idx].req.bypass)),
          request_uses_direct_mapped_llc(req_addr, req_total_size), mode2_ddr_aligned(req_addr, req_total_size)};
    } else {
      w_req_ready_r[idx] = true;
    }
    return;
  }
}

// LLC disabled: pending AWs go out (a latched AW first, else the next pending transaction unless a response is
// buffered).
void AXI_Interconnect::write_issue_pending_aw(bool any_w_resp_valid) {
  for (DownstreamPort port : {DownstreamPort::DDR, DownstreamPort::MMIO}) {
    if (aw_latch(port).valid) {
      drive_aw_latch(port);
      continue;
    }
    if (any_w_resp_valid) continue;
    const int aw_idx = find_next_aw_pending(port);
    if (aw_idx < 0) continue;
    const auto &txn = w_pending[static_cast<size_t>(aw_idx)];
    if (can_issue_external_write(txn.addr)) {
      drive_aw(txn.port, txn.addr, txn.total_beats - 1, txn.port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize,
               sim_ddr::AXI_BURST_INCR, txn.axi_id);
    }
  }
}

// LLC disabled: the first supported write (round-robin) gets ready-first acceptance.
void AXI_Interconnect::write_accept_direct() {
  for (int k = 0; k < NUM_WRITE_MASTERS; k++) {
    const int idx = (w_arb_rr_idx + k) % NUM_WRITE_MASTERS;
    if (!write_ports[idx].req.valid || !write_size_supported(write_ports[idx].req.total_size)) continue;
    const uint8_t req_total_size = static_cast<uint8_t>(write_ports[idx].req.total_size);
    const DownstreamPort req_port = classify_downstream_port(static_cast<uint32_t>(write_ports[idx].req.addr), req_total_size);
    if (!downstream_request_supported(req_port, req_total_size)) continue;
    write_ready_first(idx, true);
    return;
  }
}

void AXI_Interconnect::comb_write_request() {
  const bool any_w_resp_valid = any_write_resp_buffered();
  for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
    write_comb_ready_curr_[i] = w_req_ready_r[i];
    w_req_ready_r[i] = false;
    write_req_fire_c[i] = false;
    llc_upstream_write_accept_c[i] = false;
    llc_upstream_write_capture_c[i] = {};
  }
  axi_ddr_io.aw.awvalid = false;
  axi_mmio_io.aw.awvalid = false;
  axi_ddr_io.w.wvalid = false;
  axi_mmio_io.w.wvalid = false;

  if (llc_enabled()) {
    drive_aw_latch(DownstreamPort::DDR);
    drive_aw_latch(DownstreamPort::MMIO);
    write_issue_llc_mem_write();
    bool direct_write_selected = false;
    if (!any_w_resp_valid && !llc_mem_write_resp_valid_ && !invalidate_all_requested() &&
        !mode_transition_needs_flush()) {
      direct_write_selected = write_route_direct_mmio();
    }
    if (!direct_write_selected && count_llc_write_pending() < MAX_WRITE_OUTSTANDING && !invalidate_all_requested() &&
        !mode_transition_needs_flush()) {
      write_route_llc();
    }
  } else {
    write_issue_pending_aw(any_w_resp_valid);
    if (!mode_transition_needs_flush() && can_accept_write_now() && !any_w_resp_valid) write_accept_direct();
  }
  drive_w_current(DownstreamPort::DDR);
  drive_w_current(DownstreamPort::MMIO);
}

void AXI_Interconnect::comb_write_response() {
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
void AXI_Interconnect::seq() {
  const SeqPrev prev{llc_core_req_stage_, llc_mem_write_resp_valid_, llc_synth_read_resp_valid_};
  for (int i = 0; i < NUM_READ_MASTERS; i++) {
    read_req_accepted[i] = false;
    read_req_accepted_id[i] = 0;
  }
  for (int i = 0; i < NUM_WRITE_MASTERS; i++) write_req_accepted[i] = false;

  seq_read_ar();
  if (llc_enabled()) seq_llc_dispatch(prev);
  seq_read_data(prev);
  if (llc_enabled()) seq_llc_write(prev);
  else seq_direct_write();
  seq_finish(prev);
}

// Self-check: a read the LLC consumed from the request stage must still be owned by some LLC structure.
void AXI_Interconnect::check_llc_consumed_reads(const SeqPrev &prev) {
  if (!llc_enabled()) return;
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
  if (!(prev.stage.valid &&
        !prev.stage.is_write)) {
    return;
  }
  const int master = prev.stage.master;
  if (master < 0 || master >= NUM_READ_MASTERS ||
      !llc.io.ext_out.upstream.read_req[master].ready) {
    return;
  }
  const auto &consumed_req = prev.stage.read;
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
  if (!retained) std::exit(1);
}

// AR handshakes: latch a stalled request, turn accepted ones into pending read transactions.
void AXI_Interconnect::seq_read_ar() {
  auto finish_ar_txn = [&](ReadPendingTxn &txn) {
    txn.beats_done = 0;
    txn.data.clear();
    txn.completion_queued = false;
    txn.completion_seq = 0;
    r_pending.push_back(txn);
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

}

// The LLC core takes one request per cycle from its stage; refill the stage round-robin from the per-master latches.
void AXI_Interconnect::seq_llc_dispatch(const SeqPrev &prev) {
  auto promote_write_head = [&]() {
    for (int master = 0; master < NUM_WRITE_MASTERS; ++master) {
      if (llc_upstream_write_req[master].valid ||
          llc_upstream_write_q[master].empty()) {
        continue;
      }
      llc_upstream_write_req[master] = llc_upstream_write_q[master].front();
      llc_upstream_write_q[master].pop_front();
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
      prev.stage.valid && !prev.stage.is_write &&
      prev.stage.master < NUM_READ_MASTERS &&
      llc.io.ext_out.upstream.read_req[prev.stage.master].ready;
  const bool core_stage_write_pop =
      prev.stage.valid && prev.stage.is_write &&
      prev.stage.master < NUM_WRITE_MASTERS &&
      llc.io.ext_out.upstream.write_req[prev.stage.master].ready;
  if (core_stage_read_pop || core_stage_write_pop) {
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
      write_req_accepted[master] = true;
    }
  }
}

// R data beats, retirement of delivered responses.
void AXI_Interconnect::seq_read_data(const SeqPrev &prev) {
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
        r_pending.erase(it);
      }
    }
  }
  if (llc_enabled() && !prev.synth_read_resp_valid &&
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
      r_pending.erase(exact_it);
    } else if (it != r_pending.end()) {
      r_pending.erase(it);
    }
  }

}

// Write path with the LLC: direct MMIO writes, the LLC's memory writes, B responses, upstream write responses.
void AXI_Interconnect::seq_llc_write(const SeqPrev &prev) {
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
    start_downstream_write(issue.port, issue.addr, issue.total_size, issue.wdata, issue.wstrb, axi_id,
                           static_cast<uint8_t>(idx), write_ports[idx].req.id, false, false, idx);
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
      start_downstream_write(issue.port, issue.addr, issue.total_size, issue.wdata, issue.wstrb,
                             llc.io.ext_out.mem.write_req_id & kAxiIdMask, 0, llc.io.ext_out.mem.write_req_id, true,
                             llc.io.regs.victim_wb_valid_r || llc.io.reg_write.victim_wb_valid_r, -1);
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
    if (llc_mem_ignored_b_count_ > 0) llc_mem_ignored_b_count_--;
  };
  handle_llc_b(DownstreamPort::DDR, axi_ddr_io);
  handle_llc_b(DownstreamPort::MMIO, axi_mmio_io);

  if (prev.mem_write_resp_valid && llc.io.ext_out.mem.write_resp_ready) {
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

  if (prev.synth_read_resp_valid &&
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
}

// Make `port` carry a new write: its current transaction (data beats) and the AW request latch.
void AXI_Interconnect::start_downstream_write(DownstreamPort port, uint32_t addr, uint8_t issue_size,
                                              const WideWriteData_t &wdata, const WideWriteStrb_t &wstrb, uint8_t axi_id,
                                              uint8_t master_id, uint8_t orig_id, bool to_llc_mem, bool victim_write,
                                              int current_master) {
  w_active_ref(port) = true;
  auto &current = w_current_ref(port);
  current = {};
  current.axi_id = axi_id;
  current.master_id = master_id;
  current.orig_id = orig_id;
  current.port = port;
  current.addr = addr;
  current.wdata = wdata;
  current.wstrb = wstrb;
  current.total_beats = calc_burst_len(issue_size) + 1;
  current.to_llc_mem = to_llc_mem;
  current.llc_victim_write = victim_write;
  w_current_master_ref(port) = current_master;

  auto &latch = aw_latch(port);
  latch.valid = true;
  latch.port = port;
  latch.addr = addr;
  latch.len = current.total_beats - 1;
  latch.size = port == DownstreamPort::MMIO ? kMmioAxiSize : kDownstreamAxiSize;
  latch.burst = sim_ddr::AXI_BURST_INCR;
  latch.id = axi_id;
}

// Write path without the LLC: pending queue, AW/W/B handshakes.
void AXI_Interconnect::seq_direct_write() {
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
        txn.beats_sent++;
        current.beats_sent = txn.beats_sent;
      }
      if (seq_w_io.w.wlast) {
        if (const int idx =
                find_write_pending_by_axi_id(port, current.axi_id);
            idx >= 0) {
          w_pending[static_cast<size_t>(idx)].w_done = true;
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
}

// Clock the LLC and advance the mode/offset reconfiguration handshake.
void AXI_Interconnect::seq_finish(const SeqPrev &prev) {
  llc.seq();
  if (!llc.io.ext_out.mem.read_req_valid) {
    llc_mem_read_issue_seen_valid_ = false;
    llc_mem_read_issue_seen_id_ = 0;
    llc_mem_read_issue_seen_addr_ = 0;
    llc_mem_read_issue_seen_size_ = 0;
  }
  if (requested_config_differs_from_active()) {
    reconfig_pending_ = true;
    reconfig_target_mode_ = requested_mode();
    reconfig_target_offset_ = requested_llc_mapped_offset();
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
  check_llc_consumed_reads(prev);
}

// ============================================================================
// Helpers
// ============================================================================
uint8_t AXI_Interconnect::calc_burst_len(uint8_t total_size) {
  return axi_dual_port_axi_len_for_beat_bytes(total_size, kDownstreamBeatBytes);
}

} // namespace axi_interconnect

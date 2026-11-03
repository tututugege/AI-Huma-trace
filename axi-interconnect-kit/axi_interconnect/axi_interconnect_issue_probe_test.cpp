/**
 * @file axi_interconnect_issue_probe_test.cpp
 * @brief Regression for actual AXI_Interconnect issue-shaping wrappers.
 */

#include "AXI_Interconnect.h"
#include "axi_dual_port_route_shape.h"
#include <cstdio>

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

uint8_t get_write_byte(const axi_interconnect::WideWriteData_t &data,
                       uint32_t byte_idx) {
  const uint32_t word_idx = byte_idx / sizeof(uint32_t);
  const uint32_t shift = (byte_idx % sizeof(uint32_t)) * 8u;
  return static_cast<uint8_t>((data.words[word_idx] >> shift) & 0xffu);
}

void set_write_byte(axi_interconnect::WideWriteData_t &data, uint32_t byte_idx,
                    uint8_t value) {
  const uint32_t word_idx = byte_idx / sizeof(uint32_t);
  const uint32_t shift = (byte_idx % sizeof(uint32_t)) * 8u;
  const uint32_t mask = 0xffu << shift;
  data.words[word_idx] =
      (data.words[word_idx] & ~mask) | (static_cast<uint32_t>(value) << shift);
}

axi_interconnect::WideWriteData_t make_payload(uint32_t salt) {
  axi_interconnect::WideWriteData_t data{};
  data.clear();
  for (uint32_t byte = 0; byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES;
      ++byte) {
    set_write_byte(data, byte,
                   static_cast<uint8_t>((salt + byte * 17u) & 0xffu));
  }
  return data;
}

axi_interconnect::WideWriteStrb_t make_strobe(uint32_t salt) {
  axi_interconnect::WideWriteStrb_t strobe{};
  strobe.clear();
  for (uint32_t byte = 0; byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES;
      ++byte) {
    const bool enable = ((salt + byte * 5u) % 7u) < 4u;
    strobe.set(byte, enable);
  }
  return strobe;
}

bool expect_write_payload(const axi_interconnect::WideWriteData_t &src_data,
                          const axi_interconnect::WideWriteStrb_t &src_strobe,
                          const axi_interconnect::DownstreamWriteIssueProbe
                              &issue,
                          axi_interconnect::DownstreamPort port, uint32_t addr,
                          uint8_t total_size, bool force_line_aligned) {
  const uint16_t bytes = static_cast<uint16_t>(total_size) + 1u;
  const bool should_align =
      port != axi_interconnect::DownstreamPort::MMIO &&
      (force_line_aligned || bytes <= sim_ddr::AXI_DATA_BYTES);
  const uint32_t byte_off = should_align ? (addr - issue.addr) : 0u;

  for (uint32_t byte = 0; byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES;
      ++byte) {
    const bool has_src = should_align ? byte >= byte_off : true;
    const uint32_t src_byte = should_align ? byte - byte_off : byte;
    const uint8_t expected_data =
        has_src && src_byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES
            ? get_write_byte(src_data, src_byte)
            : 0u;
    const bool expected_strobe =
        has_src && src_byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES
            ? src_strobe.test(src_byte)
            : false;

    if (get_write_byte(issue.wdata, byte) != expected_data ||
       issue.wstrb.test(byte) != expected_strobe) {
      std::printf(
          "FAIL: write payload mismatch port=%u addr=0x%08x size=%u "
          "force=%u byte=%u got_data=0x%02x exp_data=0x%02x got_strb=%u "
          "exp_strb=%u issue_addr=0x%08x\n",
          static_cast<unsigned>(port), addr, static_cast<unsigned>(total_size),
          static_cast<unsigned>(force_line_aligned), byte,
          static_cast<unsigned>(get_write_byte(issue.wdata, byte)),
          static_cast<unsigned>(expected_data),
          static_cast<unsigned>(issue.wstrb.test(byte)),
          static_cast<unsigned>(expected_strobe), issue.addr);
      return false;
    }
  }

  return true;
}

bool test_read_issue_probe_matches_helper() {
  constexpr uint32_t kDdrBase = 0x40000000u;
  constexpr uint32_t kMmioBase = 0x10000000u;
  const uint8_t sizes[] = {0u, 1u, 3u, 7u, 15u, 31u, 63u};
  const uint32_t line_bytes[] = {8u, 64u};

  for (const auto port : {axi_interconnect::DownstreamPort::DDR,
                         axi_interconnect::DownstreamPort::MMIO}) {
    for (const uint8_t size : sizes) {
      for (const uint32_t line : line_bytes) {
        for (const bool force : {false, true}) {
          for (uint32_t off = 0; off < 64u; ++off) {
            const uint32_t base =
                port == axi_interconnect::DownstreamPort::DDR ? kDdrBase
                                                              : kMmioBase;
            const uint32_t addr = base + off;
            const auto got = axi_interconnect::probe_downstream_read_issue(
                port, addr, size, line, force);
            const AxiBridgeDownstreamIssueShape ref =
                axi_bridge_downstream_read_issue_shape(
                    port == axi_interconnect::DownstreamPort::MMIO, addr, size,
                    static_cast<uint8_t>(line), sim_ddr::AXI_DATA_BYTES, force);

            if (got.port != port || got.addr != ref.issue_addr ||
               got.total_size != ref.issue_size ||
               got.extract_from_aligned_beat !=
                   ref.extract_from_aligned_beat) {
              std::printf(
                  "FAIL: read issue mismatch port=%u addr=0x%08x size=%u "
                  "line=%u force=%u got_addr=0x%08x ref_addr=0x%08x "
                  "got_size=%u ref_size=%u got_extract=%u ref_extract=%u\n",
                  static_cast<unsigned>(port), addr, static_cast<unsigned>(size),
                  static_cast<unsigned>(line), static_cast<unsigned>(force),
                  got.addr, ref.issue_addr, static_cast<unsigned>(got.total_size),
                  static_cast<unsigned>(ref.issue_size),
                  static_cast<unsigned>(got.extract_from_aligned_beat),
                  static_cast<unsigned>(ref.extract_from_aligned_beat));
              return false;
            }
          }
        }
      }
    }
  }

  return true;
}

bool test_write_issue_probe_matches_helper() {
  constexpr uint32_t kDdrBase = 0x40000000u;
  constexpr uint32_t kMmioBase = 0x10000000u;
  const uint8_t sizes[] = {0u, 1u, 3u, 7u, 15u, 31u, 63u};
  const uint32_t line_bytes[] = {8u, 64u};
  const auto data = make_payload(0x23u);
  const auto strobe = make_strobe(0x11u);

  for (const auto port : {axi_interconnect::DownstreamPort::DDR,
                         axi_interconnect::DownstreamPort::MMIO}) {
    for (const uint8_t size : sizes) {
      for (const uint32_t line : line_bytes) {
        for (const bool force : {false, true}) {
          for (uint32_t off = 0; off < 64u; ++off) {
            const uint32_t base =
                port == axi_interconnect::DownstreamPort::DDR ? kDdrBase
                                                              : kMmioBase;
            const uint32_t addr = base + off;
            const auto got = axi_interconnect::probe_downstream_write_issue(
                port, addr, size, data, strobe, line, force);
            const AxiBridgeDownstreamIssueShape ref =
                axi_bridge_downstream_write_issue_shape(
                    port == axi_interconnect::DownstreamPort::MMIO, addr, size,
                    static_cast<uint8_t>(line), sim_ddr::AXI_DATA_BYTES, force);

            if (got.port != port || got.addr != ref.issue_addr ||
               got.total_size != ref.issue_size) {
              std::printf(
                  "FAIL: write issue mismatch port=%u addr=0x%08x size=%u "
                  "line=%u force=%u got_addr=0x%08x ref_addr=0x%08x "
                  "got_size=%u ref_size=%u\n",
                  static_cast<unsigned>(port), addr, static_cast<unsigned>(size),
                  static_cast<unsigned>(line), static_cast<unsigned>(force),
                  got.addr, ref.issue_addr, static_cast<unsigned>(got.total_size),
                  static_cast<unsigned>(ref.issue_size));
              return false;
            }

            if (!expect_write_payload(data, strobe, got, port, addr, size,
                                     force)) {
              return false;
            }
          }
        }
      }
    }
  }

  return true;
}

} // namespace

int main() {
  if (!test_read_issue_probe_matches_helper()) {
    return 1;
  }
  if (!test_write_issue_probe_matches_helper()) {
    return 1;
  }
  return 0;
}

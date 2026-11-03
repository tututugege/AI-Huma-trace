/**
 * @file axi_interconnect_dual_port_test.cpp
 * @brief Focused coverage for DDR/MMIO split-port address routing.
 */

#include <cstdio>
#include <cstdlib>

#define private public
#include "AXI_Interconnect.h"
#undef private

static_assert(axi_interconnect::MAX_OUTSTANDING == 32,
              "dual-port contract requires 32 shared read outstanding slots");
static_assert(axi_interconnect::MAX_WRITE_OUTSTANDING == 32,
              "dual-port contract requires 32 shared write outstanding slots");

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {
uint32_t g_legacy_backing_words[32] = {};

void reset_legacy_backing() {
  for (auto &word : g_legacy_backing_words) {
    word = 0;
  }
}

uint32_t legacy_backing_index(uint32_t paddr) {
  return (paddr - 0x10000000u) >> 2;
}

} // namespace

uint32_t pmem_read(uint32_t paddr) {
  if (paddr >= 0x10000000u &&
      legacy_backing_index(paddr) <
          (sizeof(g_legacy_backing_words) / sizeof(g_legacy_backing_words[0]))) {
    return g_legacy_backing_words[legacy_backing_index(paddr)];
  }
  return 0;
}

void pmem_write(uint32_t paddr, uint32_t data) {
  if (paddr >= 0x10000000u &&
      legacy_backing_index(paddr) <
          (sizeof(g_legacy_backing_words) / sizeof(g_legacy_backing_words[0]))) {
    g_legacy_backing_words[legacy_backing_index(paddr)] = data;
  }
}

namespace {

void clear_inputs(axi_interconnect::AXI_Interconnect &dut) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    auto &port = dut.read_ports[i];
    port.req.valid = false;
    port.req.addr = 0;
    port.req.total_size = 0;
    port.req.id = 0;
    port.req.bypass = false;
    port.resp.ready = false;
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    auto &port = dut.write_ports[i];
    port.req.valid = false;
    port.req.addr = 0;
    port.req.wdata.clear();
    port.req.wstrb.clear();
    port.req.total_size = 0;
    port.req.id = 0;
    port.req.bypass = false;
    port.resp.ready = false;
  }
}

void set_downstream_ready(axi_interconnect::AXI_Interconnect &dut) {
  dut.axi_ddr_io.ar.arready = true;
  dut.axi_ddr_io.aw.awready = true;
  dut.axi_ddr_io.w.wready = true;
  dut.axi_mmio_io.ar.arready = true;
  dut.axi_mmio_io.aw.awready = true;
  dut.axi_mmio_io.w.wready = true;
}

void init_dut(axi_interconnect::AXI_Interconnect &dut) {
  unsetenv("AXI_SUBMODULE_MODE");
  unsetenv("AXI_SUBMODULE_OFFSET");
  reset_legacy_backing();
  axi_interconnect::AXI_LLCConfig cfg{};
  cfg.enable = false;
  dut.set_llc_config(cfg);
  dut.mode = 0;
  dut.init();
  set_downstream_ready(dut);
}

void init_llc_dut(axi_interconnect::AXI_Interconnect &dut) {
  unsetenv("AXI_SUBMODULE_MODE");
  unsetenv("AXI_SUBMODULE_OFFSET");
  reset_legacy_backing();
  axi_interconnect::AXI_LLCConfig cfg{};
  cfg.enable = true;
  dut.set_llc_config(cfg);
  dut.mode = 1;
  dut.init();
  set_downstream_ready(dut);
}

void init_llc_dut_mode(axi_interconnect::AXI_Interconnect &dut, uint8_t mode,
                       uint32_t offset = 0x30000000u) {
  unsetenv("AXI_SUBMODULE_MODE");
  unsetenv("AXI_SUBMODULE_OFFSET");
  reset_legacy_backing();
  axi_interconnect::AXI_LLCConfig cfg{};
  cfg.enable = true;
  cfg.size_bytes = 8u << 20;
  dut.set_llc_config(cfg);
  dut.mode = mode;
  dut.llc_mapped_offset = offset;
  dut.init();
  set_downstream_ready(dut);
}

void cycle_outputs(axi_interconnect::AXI_Interconnect &dut) {
  set_downstream_ready(dut);
  dut.comb_outputs();
}

void cycle_inputs(axi_interconnect::AXI_Interconnect &dut) {
  set_downstream_ready(dut);
  dut.comb_inputs();
  dut.seq();
  ++sim_time;
}

axi_interconnect::WideWriteData_t single_word_data(uint32_t value) {
  axi_interconnect::WideWriteData_t data{};
  data.clear();
  data[0] = value;
  return data;
}

axi_interconnect::WideWriteStrb_t byte_strobe(uint32_t mask) {
  axi_interconnect::WideWriteStrb_t strobe{};
  strobe.clear();
  for (uint32_t byte = 0; byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES;
       ++byte) {
    if (byte < 32 && ((mask & (uint32_t{1} << byte)) != 0)) {
      strobe.set(byte, true);
    }
  }
  return strobe;
}

sim_ddr::axi_data_t ddr_read_beat(uint32_t base) {
  sim_ddr::axi_data_t data{};
  for (uint8_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
    axi_compat::set_u32(data, word, base + word);
  }
  return data;
}

axi_interconnect::WideWriteData_t line_write_data(uint32_t base) {
  axi_interconnect::WideWriteData_t data{};
  data.clear();
  for (uint32_t word = 0; word < axi_interconnect::MAX_WRITE_TRANSACTION_WORDS;
       ++word) {
    data[word] = base + word;
  }
  return data;
}

axi_interconnect::WideWriteStrb_t full_line_strobe() {
  axi_interconnect::WideWriteStrb_t strobe{};
  strobe.clear();
  for (uint32_t byte = 0; byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES;
       ++byte) {
    strobe.set(byte, true);
  }
  return strobe;
}

bool enqueue_non_llc_write_payload(
    axi_interconnect::AXI_Interconnect &dut, uint8_t master, uint32_t addr,
    uint8_t total_size, const axi_interconnect::WideWriteData_t &data,
    const axi_interconnect::WideWriteStrb_t &strobe, uint8_t id) {
  for (int retry = 0; retry < 8; ++retry) {
    cycle_outputs(dut);
    const bool ready_snapshot = dut.write_ports[master].req.ready;
    auto &req = dut.write_ports[master].req;
    req.valid = true;
    req.addr = addr;
    req.total_size = total_size;
    req.id = id;
    req.wdata = data;
    req.wstrb = strobe;
    req.bypass = false;
    cycle_inputs(dut);
    if (ready_snapshot) {
      return true;
    }
  }
  return false;
}

bool enqueue_non_llc_write(axi_interconnect::AXI_Interconnect &dut,
                           uint8_t master, uint32_t addr, uint8_t total_size,
                           uint32_t data, uint32_t strobe, uint8_t id) {
  return enqueue_non_llc_write_payload(dut, master, addr, total_size,
                                       single_word_data(data),
                                       byte_strobe(strobe), id);
}

bool capture_llc_write(axi_interconnect::AXI_Interconnect &dut,
                       uint8_t master, uint32_t addr, uint8_t total_size,
                       uint32_t data, uint32_t strobe, uint8_t id) {
  for (int retry = 0; retry < 8; ++retry) {
    cycle_outputs(dut);
    const bool ready_snapshot = dut.write_ports[master].req.ready;
    auto &req = dut.write_ports[master].req;
    req.valid = true;
    req.addr = addr;
    req.total_size = total_size;
    req.id = id;
    req.wdata = single_word_data(data);
    req.wstrb = byte_strobe(strobe);
    req.bypass = false;
    cycle_inputs(dut);
    if (ready_snapshot) {
      return dut.llc_upstream_write_req[master].valid;
    }
  }
  return false;
}

void fill_read_outstanding(axi_interconnect::AXI_Interconnect &dut) {
  dut.r_pending.clear();
  for (uint32_t i = 0; i < axi_interconnect::MAX_OUTSTANDING; ++i) {
    const bool use_ddr = (i % 2u) == 0;
    const uint32_t addr =
        use_ddr ? (0x50000000u + i * 0x100u)
                : (0x10010000u + i * 0x100u);
    axi_interconnect::ReadPendingTxn txn{};
    txn.axi_id = static_cast<uint8_t>(i);
    txn.master_id =
        static_cast<uint8_t>(i % axi_interconnect::NUM_READ_MASTERS);
    txn.orig_id = static_cast<uint8_t>(i);
    txn.total_beats = 1;
    txn.beats_done = 0;
    txn.port = use_ddr ? axi_interconnect::DownstreamPort::DDR
                       : axi_interconnect::DownstreamPort::MMIO;
    txn.addr = addr;
    txn.upstream_addr = addr;
    txn.upstream_total_size = 3;
    txn.data.clear();
    dut.r_pending.push_back(txn);
  }
}

void fill_write_outstanding(axi_interconnect::AXI_Interconnect &dut) {
  dut.w_pending.clear();
  for (uint32_t i = 0; i < axi_interconnect::MAX_WRITE_OUTSTANDING; ++i) {
    const bool use_ddr = (i % 2u) == 0;
    const uint32_t addr =
        use_ddr ? (0x50000000u + i * 0x100u)
                : (0x10010000u + i * 0x100u);
    axi_interconnect::WritePendingTxn txn{};
    txn.axi_id = static_cast<uint8_t>(i);
    txn.master_id =
        static_cast<uint8_t>(i % axi_interconnect::NUM_WRITE_MASTERS);
    txn.orig_id = static_cast<uint8_t>(i);
    txn.port = use_ddr ? axi_interconnect::DownstreamPort::DDR
                       : axi_interconnect::DownstreamPort::MMIO;
    txn.addr = addr;
    txn.wdata.clear();
    txn.wstrb.clear();
    txn.total_beats = 1;
    dut.w_pending.push_back(txn);
  }
}

bool test_ddr_read_routes_to_port0() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = 0x40000004u;
  req.total_size = 3;
  req.id = 1;
  dut.comb_inputs();

  if (!dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: DDR read did not route exclusively to DDR port\n");
    return false;
  }
  if (dut.axi_ddr_io.ar.araddr != 0x40000000u || dut.axi_ddr_io.ar.arlen != 0 ||
      dut.axi_ddr_io.ar.arsize != sim_ddr::AXI_SIZE_CODE) {
    std::printf("FAIL: DDR read alignment mismatch addr=0x%08x len=%u size=%u\n",
                static_cast<uint32_t>(dut.axi_ddr_io.ar.araddr),
                static_cast<unsigned>(dut.axi_ddr_io.ar.arlen),
                static_cast<unsigned>(dut.axi_ddr_io.ar.arsize));
    return false;
  }
  return true;
}

bool run_mode0_ddr_read_response_case(uint32_t addr, uint8_t total_size,
                                      uint8_t req_id,
                                      uint32_t expected_word0,
                                      uint32_t expected_word1) {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = addr;
  req.total_size = total_size;
  req.id = req_id;
  dut.comb_inputs();

  const uint32_t aligned_addr =
      (addr / sim_ddr::AXI_DATA_BYTES) * sim_ddr::AXI_DATA_BYTES;
  if (!dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid ||
      dut.axi_ddr_io.ar.araddr != aligned_addr ||
      dut.axi_ddr_io.ar.arlen != 0 ||
      dut.axi_ddr_io.ar.arsize != sim_ddr::AXI_SIZE_CODE ||
      !req.ready) {
    std::printf("FAIL: mode0 DDR read issue mismatch addr=0x%08x "
                "arvalid=%d mmio_arvalid=%d araddr=0x%08x len=%u "
                "size=%u ready=%d\n",
                addr, static_cast<int>(dut.axi_ddr_io.ar.arvalid),
                static_cast<int>(dut.axi_mmio_io.ar.arvalid),
                static_cast<uint32_t>(dut.axi_ddr_io.ar.araddr),
                static_cast<unsigned>(dut.axi_ddr_io.ar.arlen),
                static_cast<unsigned>(dut.axi_ddr_io.ar.arsize),
                static_cast<int>(req.ready));
    return false;
  }

  const uint8_t axi_id = static_cast<uint8_t>(dut.axi_ddr_io.ar.arid);
  dut.seq();
  ++sim_time;
  clear_inputs(dut);
  dut.comb_inputs();
  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: mode0 DDR read AR remained asserted after retire "
                "ddr_arvalid=%d mmio_arvalid=%d\n",
                static_cast<int>(dut.axi_ddr_io.ar.arvalid),
                static_cast<int>(dut.axi_mmio_io.ar.arvalid));
    return false;
  }
  if (dut.r_pending.size() != 1 ||
      dut.r_pending[0].addr != aligned_addr ||
      dut.r_pending[0].upstream_addr != addr ||
      dut.r_pending[0].upstream_total_size != total_size ||
      !dut.r_pending[0].resp_extract_from_aligned_beat) {
    std::printf("FAIL: mode0 DDR read pending mismatch pending=%zu\n",
                dut.r_pending.size());
    return false;
  }

  dut.axi_ddr_io.r.rvalid = true;
  dut.axi_ddr_io.r.rid = axi_id;
  dut.axi_ddr_io.r.rresp = sim_ddr::AXI_RESP_OKAY;
  dut.axi_ddr_io.r.rlast = true;
  dut.axi_ddr_io.r.rdata = ddr_read_beat(0x1000u);
  dut.comb_outputs();
  if (!dut.axi_ddr_io.r.rready) {
    std::printf("FAIL: mode0 DDR read R was backpressured\n");
    return false;
  }
  dut.seq();
  ++sim_time;

  dut.axi_ddr_io.r.rvalid = false;
  dut.read_ports[axi_interconnect::MASTER_DCACHE_R].resp.ready = true;
  dut.comb_outputs();
  const auto &resp = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].resp;
  if (!resp.valid || resp.id != req_id || resp.data[0] != expected_word0 ||
      resp.data[1] != expected_word1) {
    std::printf("FAIL: mode0 DDR read response mismatch valid=%d id=%u "
                "word0=0x%08x word1=0x%08x expected0=0x%08x "
                "expected1=0x%08x\n",
                static_cast<int>(resp.valid), static_cast<unsigned>(resp.id),
                resp.data[0], resp.data[1], expected_word0, expected_word1);
    return false;
  }

  dut.seq();
  ++sim_time;
  if (!dut.r_pending.empty()) {
    const auto &pending = dut.r_pending.front();
    std::printf("FAIL: mode0 DDR read response did not retire resp_valid=%d "
                "resp_ready=%d resp_id=%u pending_master=%u "
                "pending_orig_id=%u pending_beats=%u/%u\n",
                static_cast<int>(resp.valid), static_cast<int>(resp.ready),
                static_cast<unsigned>(resp.id),
                static_cast<unsigned>(pending.master_id),
                static_cast<unsigned>(pending.orig_id),
                static_cast<unsigned>(pending.beats_done),
                static_cast<unsigned>(pending.total_beats));
    return false;
  }
  return true;
}

bool test_mode0_ddr_read_response_slices_aligned_beat() {
  constexpr uint32_t kBaseWord = 0x1000u;
  return run_mode0_ddr_read_response_case(0x40000004u, 3, 0x1,
                                          kBaseWord + 1u, kBaseWord + 2u) &&
         run_mode0_ddr_read_response_case(0x40000000u, 7, 0x2,
                                          kBaseWord + 0u, kBaseWord + 1u);
}

bool test_mode0_ddr_cacheline_read_two_beat_response() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  constexpr uint32_t kAddr = 0x40000000u;
  constexpr uint8_t kReqId = 0x4u;
  constexpr uint8_t kMaster = axi_interconnect::MASTER_DCACHE_R;

  auto &req = dut.read_ports[kMaster].req;
  req.valid = true;
  req.addr = kAddr;
  req.total_size = 63;
  req.id = kReqId;
  dut.comb_inputs();
  if (!dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid ||
      dut.axi_ddr_io.ar.araddr != kAddr ||
      dut.axi_ddr_io.ar.arlen != 1 ||
      dut.axi_ddr_io.ar.arsize != sim_ddr::AXI_SIZE_CODE ||
      !req.ready) {
    std::printf("FAIL: mode0 DDR cacheline read AR mismatch arvalid=%d "
                "mmio_arvalid=%d araddr=0x%08x len=%u size=%u ready=%d\n",
                static_cast<int>(dut.axi_ddr_io.ar.arvalid),
                static_cast<int>(dut.axi_mmio_io.ar.arvalid),
                static_cast<uint32_t>(dut.axi_ddr_io.ar.araddr),
                static_cast<unsigned>(dut.axi_ddr_io.ar.arlen),
                static_cast<unsigned>(dut.axi_ddr_io.ar.arsize),
                static_cast<int>(req.ready));
    return false;
  }

  const uint8_t axi_id = static_cast<uint8_t>(dut.axi_ddr_io.ar.arid);
  dut.seq();
  ++sim_time;
  clear_inputs(dut);
  dut.comb_inputs();
  if (dut.r_pending.size() != 1 ||
      dut.r_pending[0].total_beats != 2 ||
      dut.r_pending[0].resp_extract_from_aligned_beat) {
    std::printf("FAIL: mode0 DDR cacheline read pending mismatch pending=%zu "
                "beats=%u extract=%d\n",
                dut.r_pending.size(),
                dut.r_pending.empty()
                    ? 0u
                    : static_cast<unsigned>(dut.r_pending[0].total_beats),
                dut.r_pending.empty()
                    ? 0
                    : static_cast<int>(
                          dut.r_pending[0].resp_extract_from_aligned_beat));
    return false;
  }

  for (uint32_t beat = 0; beat < 2; ++beat) {
    clear_inputs(dut);
    dut.comb_inputs();
    dut.axi_ddr_io.r.rvalid = true;
    dut.axi_ddr_io.r.rid = axi_id;
    dut.axi_ddr_io.r.rresp = sim_ddr::AXI_RESP_OKAY;
    dut.axi_ddr_io.r.rlast = beat == 1;
    dut.axi_ddr_io.r.rdata = ddr_read_beat(beat == 0 ? 0x2000u : 0x3000u);
    dut.comb_outputs();
    if (!dut.axi_ddr_io.r.rready) {
      std::printf("FAIL: mode0 DDR cacheline R beat %u was backpressured\n",
                  beat);
      return false;
    }
    dut.seq();
    ++sim_time;
    dut.axi_ddr_io.r.rvalid = false;
    dut.read_ports[kMaster].resp.ready = true;
    dut.comb_outputs();
    if (beat == 0 && dut.read_ports[kMaster].resp.valid) {
      std::printf("FAIL: mode0 DDR cacheline read responded before RLAST\n");
      return false;
    }
  }

  dut.read_ports[kMaster].resp.ready = true;
  dut.comb_outputs();
  const auto &resp = dut.read_ports[kMaster].resp;
  if (!resp.valid || resp.id != kReqId) {
    std::printf("FAIL: mode0 DDR cacheline read response missing valid=%d "
                "id=%u\n",
                static_cast<int>(resp.valid), static_cast<unsigned>(resp.id));
    return false;
  }
  for (uint32_t word = 0; word < 16; ++word) {
    const uint32_t expected =
        word < 8 ? (0x2000u + word) : (0x3000u + (word - 8u));
    if (resp.data[word] != expected) {
      std::printf("FAIL: mode0 DDR cacheline read data mismatch word=%u "
                  "got=0x%08x expected=0x%08x\n",
                  word, resp.data[word], expected);
      return false;
    }
  }

  dut.seq();
  ++sim_time;
  if (!dut.r_pending.empty()) {
    std::printf("FAIL: mode0 DDR cacheline read response did not retire\n");
    return false;
  }
  return true;
}

bool test_same_master_direct_read_response_completion_order_stable() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  constexpr uint8_t kMaster = axi_interconnect::MASTER_DCACHE_R;
  constexpr uint32_t kAddr0 = 0x40002000u;
  constexpr uint32_t kAddr1 = 0x40002020u;
  constexpr uint8_t kReqId0 = 0x6u;
  constexpr uint8_t kReqId1 = 0x7u;

  auto issue_read = [&](uint32_t addr, uint8_t req_id, uint8_t &axi_id) {
    clear_inputs(dut);
    auto &req = dut.read_ports[kMaster].req;
    req.valid = true;
    req.addr = addr;
    req.total_size = 3;
    req.id = req_id;
    dut.comb_inputs();
    if (!dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid ||
        !req.ready) {
      std::printf("FAIL: same-master read issue mismatch addr=0x%08x "
                  "ddr_arvalid=%d mmio_arvalid=%d ready=%d\n",
                  addr, static_cast<int>(dut.axi_ddr_io.ar.arvalid),
                  static_cast<int>(dut.axi_mmio_io.ar.arvalid),
                  static_cast<int>(req.ready));
      return false;
    }
    axi_id = static_cast<uint8_t>(dut.axi_ddr_io.ar.arid);
    dut.seq();
    ++sim_time;
    return true;
  };

  uint8_t axi_id0 = 0;
  uint8_t axi_id1 = 0;
  if (!issue_read(kAddr0, kReqId0, axi_id0) ||
      !issue_read(kAddr1, kReqId1, axi_id1)) {
    return false;
  }
  if (axi_id0 == axi_id1 || dut.r_pending.size() != 2) {
    std::printf("FAIL: same-master read setup mismatch axi_id0=%u axi_id1=%u "
                "pending=%zu\n",
                static_cast<unsigned>(axi_id0), static_cast<unsigned>(axi_id1),
                dut.r_pending.size());
    return false;
  }

  auto complete_read = [&](uint8_t axi_id, uint32_t base) {
    clear_inputs(dut);
    dut.comb_inputs();
    dut.axi_ddr_io.r.rvalid = true;
    dut.axi_ddr_io.r.rid = axi_id;
    dut.axi_ddr_io.r.rresp = sim_ddr::AXI_RESP_OKAY;
    dut.axi_ddr_io.r.rlast = true;
    dut.axi_ddr_io.r.rdata = ddr_read_beat(base);
    dut.comb_outputs();
    if (!dut.axi_ddr_io.r.rready) {
      std::printf("FAIL: same-master read R was backpressured axi_id=%u\n",
                  static_cast<unsigned>(axi_id));
      return false;
    }
    dut.seq();
    ++sim_time;
    dut.axi_ddr_io.r.rvalid = false;
    return true;
  };

  if (!complete_read(axi_id1, 0x2200u)) {
    return false;
  }

  clear_inputs(dut);
  dut.comb_outputs();
  auto &resp = dut.read_ports[kMaster].resp;
  if (!resp.valid || resp.id != kReqId1 || resp.data[0] != 0x2200u) {
    std::printf("FAIL: later-completed read was not first response valid=%d "
                "id=%u word0=0x%08x\n",
                static_cast<int>(resp.valid), static_cast<unsigned>(resp.id),
                resp.data[0]);
    return false;
  }

  if (!complete_read(axi_id0, 0x1100u)) {
    return false;
  }

  clear_inputs(dut);
  dut.comb_outputs();
  if (!resp.valid || resp.id != kReqId1 || resp.data[0] != 0x2200u) {
    std::printf("FAIL: held response changed under upstream backpressure "
                "valid=%d id=%u word0=0x%08x\n",
                static_cast<int>(resp.valid), static_cast<unsigned>(resp.id),
                resp.data[0]);
    return false;
  }

  dut.read_ports[kMaster].resp.ready = true;
  dut.comb_outputs();
  if (!resp.valid || resp.id != kReqId1) {
    std::printf("FAIL: first completed response missing before retire\n");
    return false;
  }
  dut.seq();
  ++sim_time;

  clear_inputs(dut);
  dut.comb_outputs();
  if (!resp.valid || resp.id != kReqId0 || resp.data[0] != 0x1100u) {
    std::printf("FAIL: older read did not become next response valid=%d "
                "id=%u word0=0x%08x\n",
                static_cast<int>(resp.valid), static_cast<unsigned>(resp.id),
                resp.data[0]);
    return false;
  }

  dut.read_ports[kMaster].resp.ready = true;
  dut.comb_outputs();
  dut.seq();
  ++sim_time;
  if (!dut.r_pending.empty()) {
    std::printf("FAIL: same-master reads did not retire pending=%zu\n",
                dut.r_pending.size());
    return false;
  }
  return true;
}

bool test_mmio_read_routes_to_port1() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = 0x10000000u;
  req.total_size = 3;
  req.id = 2;
  dut.comb_inputs();

  if (dut.axi_ddr_io.ar.arvalid || !dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: MMIO read did not route exclusively to MMIO port\n");
    return false;
  }
  if (dut.axi_mmio_io.ar.araddr != 0x10000000u ||
      dut.axi_mmio_io.ar.arlen != 0 || dut.axi_mmio_io.ar.arsize != 2) {
    std::printf("FAIL: MMIO read shape mismatch addr=0x%08x len=%u size=%u\n",
                static_cast<uint32_t>(dut.axi_mmio_io.ar.araddr),
                static_cast<unsigned>(dut.axi_mmio_io.ar.arlen),
                static_cast<unsigned>(dut.axi_mmio_io.ar.arsize));
    return false;
  }
  return true;
}

bool test_mmio_large_read_blocks() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = 0x10000000u;
  req.total_size = 63;
  req.id = 5;
  dut.comb_inputs();

  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid || req.ready) {
    std::printf("FAIL: unsupported MMIO cacheline read was accepted\n");
    return false;
  }
  return true;
}

bool test_mmio_large_write_blocks() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  auto &req = dut.write_ports[axi_interconnect::MASTER_DCACHE_W].req;
  req.valid = true;
  req.addr = 0x10000000u;
  req.total_size = 63;
  req.id = 6;
  req.wdata[0] = 0x55667788u;
  req.wstrb = 0xFu;
  dut.comb_inputs();

  if (dut.axi_ddr_io.aw.awvalid || dut.axi_mmio_io.aw.awvalid ||
      dut.axi_ddr_io.w.wvalid || dut.axi_mmio_io.w.wvalid || req.ready) {
    std::printf("FAIL: unsupported MMIO cacheline write was accepted\n");
    return false;
  }
  return true;
}

bool test_llc_unsupported_mmio_read_synthesizes_response() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);
  g_legacy_backing_words[0] = 0xaabbccddu;
  g_legacy_backing_words[1] = 0x11223344u;

  dut.llc.io.ext_out.mem.read_req_valid = true;
  dut.llc.io.ext_out.mem.read_req_addr = 0x10000000u;
  dut.llc.io.ext_out.mem.read_req_size = 63;
  dut.llc.io.ext_out.mem.read_req_id = 4;
  dut.prepare_llc_inputs(true);
  if (!dut.llc.io.ext_in.mem.read_req_ready) {
    std::printf("FAIL: unsupported LLC MMIO read did not complete internally\n");
    return false;
  }

  dut.comb_read_arbiter();
  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: unsupported LLC MMIO read escaped to external AXI\n");
    return false;
  }

  dut.seq();
  dut.prepare_llc_inputs(true);
  if (!dut.llc.io.ext_in.mem.read_resp_valid ||
      dut.llc.io.ext_in.mem.read_resp_id != 4 ||
      dut.llc.io.ext_in.mem.read_resp_data[0] != 0xaabbccddu ||
      dut.llc.io.ext_in.mem.read_resp_data[1] != 0x11223344u) {
    std::printf("FAIL: synthesized LLC MMIO read response missing\n");
    return false;
  }
  return true;
}

bool test_llc_unsupported_mmio_write_synthesizes_response() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  dut.llc.io.ext_out.mem.write_req_valid = true;
  dut.llc.io.ext_out.mem.write_req_addr = 0x10000000u;
  dut.llc.io.ext_out.mem.write_req_size = 63;
  dut.llc.io.ext_out.mem.write_req_id = 5;
  dut.llc.io.ext_out.mem.write_req_data[0] = 0x55667788u;
  dut.llc.io.ext_out.mem.write_req_data[1] = 0x99aabbccu;
  dut.llc.io.ext_out.mem.write_req_strobe.set(0, true);
  dut.llc.io.ext_out.mem.write_req_strobe.set(1, true);
  dut.llc.io.ext_out.mem.write_req_strobe.set(2, true);
  dut.llc.io.ext_out.mem.write_req_strobe.set(3, true);
  dut.llc.io.ext_out.mem.write_req_strobe.set(4, true);
  dut.llc.io.ext_out.mem.write_req_strobe.set(5, true);
  dut.llc.io.ext_out.mem.write_req_strobe.set(6, true);
  dut.llc.io.ext_out.mem.write_req_strobe.set(7, true);
  dut.llc.io.ext_out.mem.write_resp_ready = true;
  dut.prepare_llc_inputs(true);
  if (!dut.llc.io.ext_in.mem.write_req_ready) {
    std::printf("FAIL: unsupported LLC MMIO write did not complete internally\n");
    return false;
  }

  dut.comb_write_request();
  if (dut.axi_ddr_io.aw.awvalid || dut.axi_mmio_io.aw.awvalid ||
      dut.axi_ddr_io.w.wvalid || dut.axi_mmio_io.w.wvalid) {
    std::printf("FAIL: unsupported LLC MMIO write escaped to external AXI\n");
    return false;
  }

  dut.seq();
  dut.prepare_llc_inputs(true);
  if (!dut.llc.io.ext_in.mem.write_resp_valid ||
      dut.llc.io.ext_in.mem.write_resp != sim_ddr::AXI_RESP_OKAY ||
      g_legacy_backing_words[0] != 0x55667788u ||
      g_legacy_backing_words[1] != 0x99aabbccu) {
    std::printf("FAIL: synthesized LLC MMIO write response missing\n");
    return false;
  }
  return true;
}

bool test_llc_unsupported_mmio_upstream_read_blocks() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = 0x10000000u;
  req.total_size = 63;
  req.id = 9;
  req.bypass = false;
  dut.comb_read_arbiter();

  const auto &cap = dut.llc_upstream_capture_c[axi_interconnect::MASTER_DCACHE_R];
  if (cap.valid || dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req.ready) {
    std::printf("FAIL: unsupported MMIO read was accepted by LLC path\n");
    return false;
  }
  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: unsupported MMIO read escaped to external AXI\n");
    return false;
  }
  return true;
}

bool test_llc_dcache_read_capture_pulses_accepted_id() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  constexpr uint8_t kReqId = 7;
  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = 0x80001000u;
  req.total_size = 63;
  req.id = kReqId;
  req.bypass = false;
  dut.comb_inputs();

  const auto &cap = dut.llc_upstream_capture_c[axi_interconnect::MASTER_DCACHE_R];
  if (!req.ready || !cap.valid || cap.id != kReqId || cap.addr != req.addr) {
    std::printf("FAIL: LLC DCache read capture mismatch ready=%d cap_valid=%d "
                "cap_id=%u cap_addr=0x%08x\n",
                static_cast<int>(req.ready), static_cast<int>(cap.valid),
                static_cast<unsigned>(cap.id), cap.addr);
    return false;
  }
  if (req.accepted) {
    std::printf("FAIL: accepted pulse was exposed before seq\n");
    return false;
  }

  dut.seq();
  ++sim_time;
  clear_inputs(dut);
  dut.comb_outputs();
  const auto &accepted =
      dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  if (!accepted.accepted || accepted.accepted_id != kReqId) {
    std::printf("FAIL: accepted/id pulse missing accepted=%d id=%u\n",
                static_cast<int>(accepted.accepted),
                static_cast<unsigned>(accepted.accepted_id));
    return false;
  }

  dut.comb_inputs();
  dut.seq();
  ++sim_time;
  clear_inputs(dut);
  dut.comb_outputs();
  if (dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req.accepted) {
    std::printf("FAIL: accepted pulse did not clear after one cycle\n");
    return false;
  }
  return true;
}

bool test_llc_mmio_word_read_bypasses_llc_core() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = 0x10000000u;
  req.total_size = 3;
  req.id = 10;
  req.bypass = false;
  dut.comb_read_arbiter();

  if (dut.axi_ddr_io.ar.arvalid || !dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: LLC-on MMIO word read did not route to MMIO AXI\n");
    return false;
  }
  if (dut.llc_upstream_capture_c[axi_interconnect::MASTER_DCACHE_R].valid ||
      dut.ar_issue_c[1].from_llc) {
    std::printf("FAIL: LLC-on MMIO word read still entered LLC core path\n");
    return false;
  }

  dut.seq();
  if (dut.r_pending.empty() || dut.r_pending.back().to_llc ||
      dut.r_pending.back().port != axi_interconnect::DownstreamPort::MMIO ||
      dut.r_pending.back().upstream_addr != 0x10000000u) {
    std::printf("FAIL: direct MMIO read pending entry shape mismatch\n");
    return false;
  }
  return true;
}

bool test_llc_direct_mmio_read_resp_blocks_llc_resp_ready() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  axi_interconnect::ReadPendingTxn pending{};
  pending.axi_id = 11;
  pending.master_id = axi_interconnect::MASTER_DCACHE_R;
  pending.orig_id = 11;
  pending.total_beats = 1;
  pending.beats_done = 1;
  pending.port = axi_interconnect::DownstreamPort::MMIO;
  pending.addr = 0x10000000u;
  pending.upstream_addr = 0x10000000u;
  pending.upstream_total_size = 3;
  pending.to_llc = false;
  pending.data.clear();
  pending.data[0] = 0xa5a55a5au;
  dut.r_pending.push_back(pending);
  dut.read_ports[axi_interconnect::MASTER_DCACHE_R].resp.ready = true;

  dut.prepare_llc_inputs(true);
  if (dut.llc.io.ext_in.upstream
          .read_resp[axi_interconnect::MASTER_DCACHE_R]
          .ready) {
    std::printf("FAIL: LLC read response ready overlapped direct MMIO resp\n");
    return false;
  }

  dut.comb_outputs();
  const auto &resp = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].resp;
  if (!resp.valid || resp.id != 11 || resp.data[0] != 0xa5a55a5au) {
    std::printf("FAIL: direct MMIO read response was not driven upstream\n");
    return false;
  }
  return true;
}

bool test_llc_mmio_word_write_bypasses_llc_core() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  constexpr uint8_t master = axi_interconnect::MASTER_UNCORE_LSU_W;
  dut.w_req_ready_r[master] = true;
  auto &req = dut.write_ports[master].req;
  req.valid = true;
  req.addr = 0x10000000u;
  req.total_size = 3;
  req.id = 12;
  req.wdata = single_word_data(0xc001d00du);
  req.wstrb = byte_strobe(0xfu);
  req.bypass = false;

  dut.comb_write_request();
  if (!req.ready || !dut.write_req_fire_c[master]) {
    std::printf("FAIL: LLC-on MMIO word write was not accepted as direct\n");
    return false;
  }
  if (dut.llc_upstream_write_capture_c[master].valid) {
    std::printf("FAIL: LLC-on MMIO word write still captured into LLC core\n");
    return false;
  }

  dut.seq();
  if (!dut.w_active_mmio || !dut.aw_latched_mmio.valid ||
      dut.w_current_mmio.to_llc_mem ||
      dut.w_current_mmio.master_id != master ||
      dut.w_current_mmio.orig_id != 12 ||
      dut.w_current_mmio.addr != 0x10000000u) {
    std::printf("FAIL: direct MMIO write state mismatch active=%d latch=%d "
                "to_llc=%d master=%u id=%u addr=0x%08x\n",
                static_cast<int>(dut.w_active_mmio),
                static_cast<int>(dut.aw_latched_mmio.valid),
                static_cast<int>(dut.w_current_mmio.to_llc_mem),
                static_cast<unsigned>(dut.w_current_mmio.master_id),
                static_cast<unsigned>(dut.w_current_mmio.orig_id),
                dut.w_current_mmio.addr);
    return false;
  }
  return true;
}

bool test_llc_direct_mmio_write_b_returns_upstream() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  constexpr uint8_t master = axi_interconnect::MASTER_UNCORE_LSU_W;
  dut.w_active_mmio = true;
  dut.w_current_mmio = {};
  dut.w_current_mmio.axi_id = 13;
  dut.w_current_mmio.master_id = master;
  dut.w_current_mmio.orig_id = 14;
  dut.w_current_mmio.port = axi_interconnect::DownstreamPort::MMIO;
  dut.w_current_mmio.addr = 0x10000000u;
  dut.w_current_mmio.total_beats = 1;
  dut.w_current_mmio.beats_sent = 1;
  dut.w_current_mmio.aw_done = true;
  dut.w_current_mmio.w_done = true;
  dut.w_current_mmio.to_llc_mem = false;
  dut.axi_mmio_io.b.bvalid = true;
  dut.axi_mmio_io.b.bid = 13;
  dut.axi_mmio_io.b.bresp = sim_ddr::AXI_RESP_OKAY;

  dut.comb_write_response();
  dut.seq();
  if (!dut.w_resp_valid[master] || dut.w_resp_id[master] != 14 ||
      dut.llc_mem_write_resp_valid_ || dut.w_active_mmio) {
    std::printf("FAIL: direct MMIO B did not return upstream resp_valid=%d "
                "id=%u llc_resp=%d active=%d\n",
                static_cast<int>(dut.w_resp_valid[master]),
                static_cast<unsigned>(dut.w_resp_id[master]),
                static_cast<int>(dut.llc_mem_write_resp_valid_),
                static_cast<int>(dut.w_active_mmio));
    return false;
  }
  return true;
}

bool test_llc_mem_write_b_stays_llc_owned() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  dut.w_active = true;
  dut.w_current = {};
  dut.w_current.axi_id = 15;
  dut.w_current.port = axi_interconnect::DownstreamPort::DDR;
  dut.w_current.addr = 0x40000000u;
  dut.w_current.total_beats = 1;
  dut.w_current.beats_sent = 1;
  dut.w_current.aw_done = true;
  dut.w_current.w_done = true;
  dut.w_current.to_llc_mem = true;
  dut.axi_ddr_io.b.bvalid = true;
  dut.axi_ddr_io.b.bid = 15;
  dut.axi_ddr_io.b.bresp = sim_ddr::AXI_RESP_OKAY;

  dut.comb_write_response();
  dut.seq();
  if (!dut.llc_mem_write_resp_valid_ || dut.w_resp_valid[0] ||
      dut.w_resp_valid[1] || dut.w_active) {
    std::printf("FAIL: LLC mem B ownership mismatch llc_resp=%d "
                "wresp0=%d wresp1=%d active=%d\n",
                static_cast<int>(dut.llc_mem_write_resp_valid_),
                static_cast<int>(dut.w_resp_valid[0]),
                static_cast<int>(dut.w_resp_valid[1]),
                static_cast<int>(dut.w_active));
    return false;
  }
  return true;
}

bool test_llc_direct_write_resp_blocks_llc_resp_ready() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  constexpr uint8_t master = axi_interconnect::MASTER_UNCORE_LSU_W;
  dut.w_resp_valid[master] = true;
  dut.w_resp_id[master] = 16;
  dut.w_resp_resp[master] = sim_ddr::AXI_RESP_OKAY;
  dut.write_ports[master].resp.ready = true;

  dut.prepare_llc_inputs(true);
  if (dut.llc.io.ext_in.upstream.write_resp[master].ready) {
    std::printf("FAIL: LLC write response ready overlapped direct write resp\n");
    return false;
  }

  dut.comb_outputs();
  const auto &resp = dut.write_ports[master].resp;
  if (!resp.valid || resp.id != 16 || resp.resp != sim_ddr::AXI_RESP_OKAY) {
    std::printf("FAIL: direct write response was not driven upstream\n");
    return false;
  }
  return true;
}

bool test_ddr_and_mmio_read_issue_same_cycle() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  dut.req_ready_r[axi_interconnect::MASTER_ICACHE] = true;
  auto &icache_req = dut.read_ports[axi_interconnect::MASTER_ICACHE].req;
  icache_req.valid = true;
  icache_req.addr = 0x40000200u;
  icache_req.total_size = 63;
  icache_req.id = 7;

  auto &dcache_req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  dcache_req.valid = true;
  dcache_req.addr = 0x10000000u;
  dcache_req.total_size = 3;
  dcache_req.id = 8;

  dut.comb_inputs();

  if (!dut.axi_ddr_io.ar.arvalid || !dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: DDR/MMIO AR did not issue in the same cycle\n");
    return false;
  }
  if (dut.axi_ddr_io.ar.araddr != 0x40000200u ||
      dut.axi_ddr_io.ar.arlen != 1 ||
      dut.axi_mmio_io.ar.araddr != 0x10000000u ||
      dut.axi_mmio_io.ar.arlen != 0) {
    std::printf("FAIL: same-cycle AR shape mismatch ddr_addr=0x%08x "
                "ddr_len=%u mmio_addr=0x%08x mmio_len=%u\n",
                static_cast<uint32_t>(dut.axi_ddr_io.ar.araddr),
                static_cast<unsigned>(dut.axi_ddr_io.ar.arlen),
                static_cast<uint32_t>(dut.axi_mmio_io.ar.araddr),
                static_cast<unsigned>(dut.axi_mmio_io.ar.arlen));
    return false;
  }
  return true;
}

bool test_ddr_and_mmio_aw_issue_same_cycle() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  axi_interconnect::WritePendingTxn ddr{};
  ddr.axi_id = 1;
  ddr.master_id = axi_interconnect::MASTER_DCACHE_W;
  ddr.orig_id = 1;
  ddr.port = axi_interconnect::DownstreamPort::DDR;
  ddr.addr = 0x40000400u;
  ddr.total_beats = 2;
  axi_interconnect::WritePendingTxn mmio{};
  mmio.axi_id = 2;
  mmio.master_id = axi_interconnect::MASTER_UNCORE_LSU_W;
  mmio.orig_id = 2;
  mmio.port = axi_interconnect::DownstreamPort::MMIO;
  mmio.addr = 0x10000004u;
  mmio.total_beats = 1;
  dut.w_pending.push_back(ddr);
  dut.w_pending.push_back(mmio);

  dut.comb_inputs();

  if (!dut.axi_ddr_io.aw.awvalid || !dut.axi_mmio_io.aw.awvalid) {
    std::printf("FAIL: DDR/MMIO AW did not issue in the same cycle\n");
    return false;
  }
  if (dut.axi_ddr_io.aw.awaddr != 0x40000400u ||
      dut.axi_ddr_io.aw.awlen != 1 ||
      dut.axi_mmio_io.aw.awaddr != 0x10000004u ||
      dut.axi_mmio_io.aw.awlen != 0) {
    std::printf("FAIL: same-cycle AW shape mismatch\n");
    return false;
  }
  return true;
}

bool test_ddr_and_mmio_w_issue_same_cycle() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  dut.w_active = true;
  dut.w_current = {};
  dut.w_current.port = axi_interconnect::DownstreamPort::DDR;
  dut.w_current.aw_done = true;
  dut.w_current.total_beats = 2;
  dut.w_current.wdata[0] = 0x11112222u;
  dut.w_current.wstrb.set(0, true);

  dut.w_active_mmio = true;
  dut.w_current_mmio = {};
  dut.w_current_mmio.port = axi_interconnect::DownstreamPort::MMIO;
  dut.w_current_mmio.aw_done = true;
  dut.w_current_mmio.total_beats = 1;
  dut.w_current_mmio.wdata[0] = 0x33334444u;
  dut.w_current_mmio.wstrb.set(0, true);

  dut.comb_inputs();

  if (!dut.axi_ddr_io.w.wvalid || !dut.axi_mmio_io.w.wvalid) {
    std::printf("FAIL: DDR/MMIO W did not issue in the same cycle\n");
    return false;
  }
  if (dut.axi_ddr_io.w.wlast || !dut.axi_mmio_io.w.wlast) {
    std::printf("FAIL: same-cycle W last mismatch ddr_last=%u mmio_last=%u\n",
                static_cast<unsigned>(dut.axi_ddr_io.w.wlast),
                static_cast<unsigned>(dut.axi_mmio_io.w.wlast));
    return false;
  }
  return true;
}

bool test_same_line_write_waits_for_read_return() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  auto &read_req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  read_req.valid = true;
  read_req.addr = 0x40000100u;
  read_req.total_size = 63;
  read_req.id = 3;
  dut.comb_inputs();
  if (!dut.axi_ddr_io.ar.arvalid) {
    std::printf("FAIL: setup read did not issue AR\n");
    return false;
  }
  dut.seq();

  clear_inputs(dut);
  auto &write_req = dut.write_ports[axi_interconnect::MASTER_DCACHE_W].req;
  write_req.valid = true;
  write_req.addr = 0x40000110u;
  write_req.total_size = 3;
  write_req.id = 4;
  write_req.wdata[0] = 0x11223344u;
  write_req.wstrb = 0xFu;
  dut.comb_inputs();
  dut.seq();

  clear_inputs(dut);
  auto &write_req_held = dut.write_ports[axi_interconnect::MASTER_DCACHE_W].req;
  write_req_held.valid = true;
  write_req_held.addr = 0x40000110u;
  write_req_held.total_size = 3;
  write_req_held.id = 4;
  write_req_held.wdata[0] = 0x11223344u;
  write_req_held.wstrb = 0xFu;
  dut.comb_inputs();
  dut.seq();

  clear_inputs(dut);
  dut.comb_inputs();
  if (dut.axi_ddr_io.aw.awvalid || dut.axi_mmio_io.aw.awvalid) {
    std::printf("FAIL: same-line AW issued before read R returned\n");
    return false;
  }
  return true;
}

bool test_same_line_write_releases_after_r_buffered() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  constexpr uint32_t kLineAddr = 0x40000600u;
  constexpr uint8_t kReadAxiId = 9;
  constexpr uint8_t kReadOrigId = 9;
  constexpr uint8_t kWriteId = 10;
  const uint8_t write_master = axi_interconnect::MASTER_DCACHE_W;
  const uint8_t read_master = axi_interconnect::MASTER_DCACHE_R;

  axi_interconnect::ReadPendingTxn pending{};
  pending.axi_id = kReadAxiId;
  pending.master_id = read_master;
  pending.orig_id = kReadOrigId;
  pending.total_beats = 2;
  pending.beats_done = 0;
  pending.port = axi_interconnect::DownstreamPort::DDR;
  pending.addr = kLineAddr;
  pending.upstream_addr = kLineAddr;
  pending.upstream_total_size = 63;
  pending.to_llc = false;
  pending.data.clear();
  dut.r_pending.push_back(pending);

  auto &write_req = dut.write_ports[write_master].req;
  write_req.valid = true;
  write_req.addr = kLineAddr + 4u;
  write_req.total_size = 3;
  write_req.id = kWriteId;
  write_req.wdata = single_word_data(0x55667788u);
  write_req.wstrb = byte_strobe(0xFu);

  dut.w_req_ready_r[write_master] = true;
  dut.comb_inputs();
  if (dut.axi_ddr_io.aw.awvalid || dut.axi_ddr_io.w.wvalid) {
    std::printf("FAIL: same-line write issued before any R beat\n");
    return false;
  }

  dut.read_ports[read_master].resp.ready = false;
  dut.axi_ddr_io.r.rvalid = true;
  dut.axi_ddr_io.r.rid = kReadAxiId;
  dut.axi_ddr_io.r.rresp = sim_ddr::AXI_RESP_OKAY;
  dut.axi_ddr_io.r.rlast = false;
  dut.axi_ddr_io.r.rdata = sim_ddr::axi_data_t{};
  dut.comb_outputs();
  if (!dut.axi_ddr_io.r.rready) {
    std::printf("FAIL: DDR R was backpressured while upstream resp blocked\n");
    return false;
  }
  dut.seq();
  if (dut.r_pending.empty() || dut.r_pending[0].beats_done != 1) {
    std::printf("FAIL: first R beat was not buffered\n");
    return false;
  }
  if (!dut.has_external_pending_read_hazard(write_req.addr)) {
    std::printf("FAIL: same-line read hazard released before R last\n");
    return false;
  }

  dut.axi_ddr_io.r.rvalid = false;
  dut.w_req_ready_r[write_master] = true;
  dut.comb_inputs();
  if (dut.axi_ddr_io.aw.awvalid || dut.axi_ddr_io.w.wvalid) {
    std::printf("FAIL: same-line write issued after only first R beat\n");
    return false;
  }

  dut.axi_ddr_io.r.rvalid = true;
  dut.axi_ddr_io.r.rid = kReadAxiId;
  dut.axi_ddr_io.r.rresp = sim_ddr::AXI_RESP_OKAY;
  dut.axi_ddr_io.r.rlast = true;
  dut.axi_ddr_io.r.rdata = sim_ddr::axi_data_t{};
  dut.comb_outputs();
  if (!dut.axi_ddr_io.r.rready) {
    std::printf("FAIL: DDR R last was backpressured by blocked upstream resp\n");
    return false;
  }
  dut.seq();
  if (dut.r_pending.empty() || dut.r_pending[0].beats_done != 2) {
    std::printf("FAIL: final R beat was not buffered\n");
    return false;
  }
  if (dut.has_external_pending_read_hazard(write_req.addr)) {
    std::printf("FAIL: same-line read hazard stayed after R last was buffered\n");
    return false;
  }

  dut.axi_ddr_io.r.rvalid = false;
  dut.comb_outputs();
  if (!dut.read_ports[read_master].resp.valid) {
    std::printf("FAIL: buffered read response was not presented upstream\n");
    return false;
  }
  dut.w_req_ready_r[write_master] = true;
  dut.comb_inputs();
  if (!dut.write_req_fire_c[write_master]) {
    std::printf("FAIL: same-line write was not accepted after R last buffered\n");
    return false;
  }
  dut.seq();
  dut.comb_inputs();
  if (!dut.axi_ddr_io.aw.awvalid) {
    std::printf("FAIL: same-line AW did not issue after R last buffered\n");
    return false;
  }
  return true;
}

bool test_same_line_read_waits_for_write_b() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  axi_interconnect::WritePendingTxn pending{};
  pending.axi_id = 3;
  pending.master_id = axi_interconnect::MASTER_DCACHE_W;
  pending.orig_id = 3;
  pending.port = axi_interconnect::DownstreamPort::DDR;
  pending.addr = 0x40000200u;
  pending.total_beats = 1;
  pending.beats_sent = 1;
  pending.aw_done = true;
  pending.w_done = true;
  dut.w_pending.push_back(pending);

  auto &read_req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  read_req.valid = true;
  read_req.addr = 0x40000210u;
  read_req.total_size = 3;
  read_req.id = 5;
  dut.comb_inputs();

  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: same-line AR issued before write B returned\n");
    return false;
  }
  return true;
}

bool test_same_line_read_releases_after_b_buffered() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  constexpr uint32_t kLineAddr = 0x40000700u;
  constexpr uint8_t kWriteAxiId = 11;
  constexpr uint8_t kReadId = 12;
  const uint8_t write_master = axi_interconnect::MASTER_DCACHE_W;
  const uint8_t read_master = axi_interconnect::MASTER_DCACHE_R;

  axi_interconnect::WritePendingTxn pending{};
  pending.axi_id = kWriteAxiId;
  pending.master_id = write_master;
  pending.orig_id = kWriteAxiId;
  pending.port = axi_interconnect::DownstreamPort::DDR;
  pending.addr = kLineAddr;
  pending.total_beats = 1;
  pending.beats_sent = 1;
  pending.aw_done = true;
  pending.w_done = true;
  dut.w_pending.push_back(pending);

  auto &read_req = dut.read_ports[read_master].req;
  read_req.valid = true;
  read_req.addr = kLineAddr + 8u;
  read_req.total_size = 3;
  read_req.id = kReadId;
  dut.comb_inputs();
  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid) {
    std::printf("FAIL: same-line read issued before B returned\n");
    return false;
  }
  if (!dut.has_external_pending_write_hazard(read_req.addr)) {
    std::printf("FAIL: same-line write hazard missing before B\n");
    return false;
  }

  dut.write_ports[write_master].resp.ready = false;
  dut.axi_ddr_io.b.bvalid = true;
  dut.axi_ddr_io.b.bid = kWriteAxiId;
  dut.axi_ddr_io.b.bresp = sim_ddr::AXI_RESP_OKAY;
  dut.comb_outputs();
  if (!dut.axi_ddr_io.b.bready) {
    std::printf("FAIL: DDR B was backpressured while upstream resp blocked\n");
    return false;
  }
  dut.seq();
  if (!dut.w_pending.empty() || !dut.w_resp_valid[write_master]) {
    std::printf("FAIL: B was not buffered as upstream write response\n");
    return false;
  }
  if (dut.has_external_pending_write_hazard(read_req.addr)) {
    std::printf("FAIL: same-line write hazard stayed after B was buffered\n");
    return false;
  }

  dut.axi_ddr_io.b.bvalid = false;
  dut.comb_inputs();
  if (!dut.axi_ddr_io.ar.arvalid) {
    std::printf("FAIL: same-line AR did not issue after B buffered\n");
    return false;
  }
  return true;
}

bool test_different_port_read_not_blocked_by_ddr_write() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  axi_interconnect::WritePendingTxn pending{};
  pending.axi_id = 4;
  pending.master_id = axi_interconnect::MASTER_DCACHE_W;
  pending.orig_id = 4;
  pending.port = axi_interconnect::DownstreamPort::DDR;
  pending.addr = 0x40000300u;
  pending.total_beats = 1;
  pending.aw_done = false;
  pending.w_done = false;
  dut.w_pending.push_back(pending);

  auto &read_req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  read_req.valid = true;
  read_req.addr = 0x10000000u;
  read_req.total_size = 3;
  read_req.id = 6;
  dut.comb_inputs();

  if (!dut.axi_mmio_io.ar.arvalid || dut.axi_ddr_io.ar.arvalid) {
    std::printf("FAIL: MMIO AR was incorrectly blocked by DDR write\n");
    return false;
  }
  if (!dut.axi_ddr_io.aw.awvalid) {
    std::printf("FAIL: DDR AW setup was not driven alongside MMIO AR\n");
    return false;
  }
  return true;
}

bool test_different_port_write_not_blocked_by_ddr_read() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  axi_interconnect::ReadPendingTxn pending{};
  pending.axi_id = 5;
  pending.master_id = axi_interconnect::MASTER_DCACHE_R;
  pending.orig_id = 5;
  pending.total_beats = 2;
  pending.beats_done = 0;
  pending.port = axi_interconnect::DownstreamPort::DDR;
  pending.addr = 0x40000400u;
  pending.upstream_addr = 0x40000400u;
  pending.upstream_total_size = 63;
  pending.to_llc = false;
  dut.r_pending.push_back(pending);

  axi_interconnect::WritePendingTxn write{};
  write.axi_id = 6;
  write.master_id = axi_interconnect::MASTER_UNCORE_LSU_W;
  write.orig_id = 6;
  write.port = axi_interconnect::DownstreamPort::MMIO;
  write.addr = 0x10000004u;
  write.total_beats = 1;
  write.aw_done = false;
  write.w_done = false;
  dut.w_pending.push_back(write);

  dut.comb_inputs();

  if (!dut.axi_mmio_io.aw.awvalid || dut.axi_ddr_io.aw.awvalid) {
    std::printf("FAIL: MMIO AW was incorrectly blocked by DDR read\n");
    return false;
  }
  return true;
}

bool test_llc_mem_write_waits_for_read_return() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  axi_interconnect::ReadPendingTxn pending{};
  pending.axi_id = 1;
  pending.master_id = 0;
  pending.orig_id = 1;
  pending.total_beats = 2;
  pending.beats_done = 1;
  pending.port = axi_interconnect::DownstreamPort::DDR;
  pending.addr = 0x40000800u;
  pending.upstream_addr = 0x40000800u;
  pending.upstream_total_size = 63;
  pending.to_llc = true;
  dut.r_pending.push_back(pending);

  dut.llc.io.ext_out.mem.write_req_valid = true;
  dut.llc.io.ext_out.mem.write_req_addr = 0x40000820u;
  dut.llc.io.ext_out.mem.write_req_size = 63;
  dut.prepare_llc_inputs(true);
  if (dut.llc.io.ext_in.mem.write_req_ready) {
    std::printf("FAIL: LLC mem write was ready before same-line R returned\n");
    return false;
  }

  dut.r_pending[0].beats_done = dut.r_pending[0].total_beats;
  dut.prepare_llc_inputs(true);
  if (!dut.llc.io.ext_in.mem.write_req_ready) {
    std::printf("FAIL: LLC mem write stayed blocked after R returned\n");
    return false;
  }
  return true;
}

bool test_llc_mem_read_waits_for_write_b() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut(dut);
  clear_inputs(dut);

  dut.w_active = true;
  dut.w_current = {};
  dut.w_current.port = axi_interconnect::DownstreamPort::DDR;
  dut.w_current.addr = 0x40000900u;
  dut.w_current.total_beats = 1;
  dut.w_current.aw_done = true;
  dut.w_current.w_done = true;

  dut.llc.io.ext_out.mem.read_req_valid = true;
  dut.llc.io.ext_out.mem.read_req_addr = 0x40000920u;
  dut.llc.io.ext_out.mem.read_req_size = 63;
  dut.llc.io.ext_out.mem.read_req_id = 2;
  dut.prepare_llc_inputs(true);
  if (dut.llc.io.ext_in.mem.read_req_ready) {
    std::printf("FAIL: LLC mem read was ready before same-line B returned\n");
    return false;
  }

  dut.w_active = false;
  dut.w_current = {};
  dut.prepare_llc_inputs(true);
  if (!dut.llc.io.ext_in.mem.read_req_ready) {
    std::printf("FAIL: LLC mem read stayed blocked after write completion\n");
    return false;
  }
  return true;
}

bool test_shared_read_outstanding_budget_blocks_both_ports() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  fill_read_outstanding(dut);

  auto &ddr_req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  ddr_req.valid = true;
  ddr_req.addr = 0x40000004u;
  ddr_req.total_size = 3;
  ddr_req.id = 0x31;
  dut.comb_inputs();
  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid ||
      ddr_req.ready) {
    std::printf("FAIL: full shared read budget still accepted DDR read\n");
    return false;
  }

  clear_inputs(dut);
  auto &mmio_req = dut.read_ports[axi_interconnect::MASTER_UNCORE_LSU_R].req;
  mmio_req.valid = true;
  mmio_req.addr = 0x10000000u;
  mmio_req.total_size = 3;
  mmio_req.id = 0x32;
  dut.comb_inputs();
  if (dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid ||
      mmio_req.ready) {
    std::printf("FAIL: full shared read budget still accepted MMIO read\n");
    return false;
  }
  return true;
}

bool test_read_outstanding_does_not_block_write_budget() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  fill_read_outstanding(dut);

  if (!enqueue_non_llc_write(dut, axi_interconnect::MASTER_DCACHE_W,
                             0x40000004u, 3, 0x11223344u, 0xfu, 0x33)) {
    std::printf("FAIL: full read budget incorrectly blocked write request\n");
    return false;
  }
  if (dut.w_pending.empty()) {
    std::printf("FAIL: write was accepted but no write pending entry exists\n");
    return false;
  }
  return true;
}

bool test_shared_write_outstanding_budget_blocks_both_ports() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  fill_write_outstanding(dut);

  auto &ddr_req = dut.write_ports[axi_interconnect::MASTER_DCACHE_W].req;
  ddr_req.valid = true;
  ddr_req.addr = 0x40000004u;
  ddr_req.total_size = 3;
  ddr_req.id = 0x34;
  ddr_req.wdata = single_word_data(0xaabbccddu);
  ddr_req.wstrb = byte_strobe(0xfu);
  dut.comb_inputs();
  if (ddr_req.ready ||
      dut.w_pending.size() != axi_interconnect::MAX_WRITE_OUTSTANDING) {
    std::printf("FAIL: full shared write budget still accepted DDR write\n");
    return false;
  }

  clear_inputs(dut);
  auto &mmio_req = dut.write_ports[axi_interconnect::MASTER_UNCORE_LSU_W].req;
  mmio_req.valid = true;
  mmio_req.addr = 0x10000000u;
  mmio_req.total_size = 3;
  mmio_req.id = 0x35;
  mmio_req.wdata = single_word_data(0xddccbbaau);
  mmio_req.wstrb = byte_strobe(0xfu);
  dut.comb_inputs();
  if (mmio_req.ready ||
      dut.w_pending.size() != axi_interconnect::MAX_WRITE_OUTSTANDING) {
    std::printf("FAIL: full shared write budget still accepted MMIO write\n");
    return false;
  }
  return true;
}

bool test_write_outstanding_does_not_block_read_budget() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);
  fill_write_outstanding(dut);

  auto &req = dut.read_ports[axi_interconnect::MASTER_DCACHE_R].req;
  req.valid = true;
  req.addr = 0x40000004u;
  req.total_size = 3;
  req.id = 0x36;
  dut.comb_inputs();
  if (!dut.axi_ddr_io.ar.arvalid || dut.axi_mmio_io.ar.arvalid ||
      !req.ready) {
    std::printf("FAIL: full write budget incorrectly blocked DDR read\n");
    return false;
  }
  return true;
}

bool test_mode0_ddr_partial_write_aligns_to_256b() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  if (!enqueue_non_llc_write(dut, axi_interconnect::MASTER_DCACHE_W,
                             0x40000004u, 3, 0xaabbccddu, 0xfu, 0x11)) {
    std::printf("FAIL: mode0 DDR partial write was not accepted\n");
    return false;
  }
  if (dut.w_pending.empty()) {
    std::printf("FAIL: mode0 DDR write was not enqueued\n");
    return false;
  }
  const auto &txn = dut.w_pending.back();
  if (txn.port != axi_interconnect::DownstreamPort::DDR ||
      txn.addr != 0x40000000u || txn.total_beats != 1 ||
      txn.wdata[1] != 0xaabbccddu) {
    std::printf("FAIL: mode0 DDR write alignment mismatch addr=0x%08x "
                "beats=%u data1=0x%08x\n",
                txn.addr, static_cast<unsigned>(txn.total_beats),
                txn.wdata[1]);
    return false;
  }
  for (int byte = 0; byte < 8; ++byte) {
    const bool expected = byte >= 4;
    if (txn.wstrb.test(byte) != expected) {
      std::printf("FAIL: mode0 DDR shifted strobe mismatch byte=%d got=%d\n",
                  byte, static_cast<int>(txn.wstrb.test(byte)));
      return false;
    }
  }
  return true;
}

bool test_mode0_ddr_partial_write_b_response_retires() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  constexpr uint32_t kAddr = 0x40000004u;
  constexpr uint32_t kAlignedAddr = 0x40000000u;
  constexpr uint32_t kData = 0xaabbccddu;
  constexpr uint8_t kReqId = 0x3u;
  constexpr uint8_t kMaster = axi_interconnect::MASTER_DCACHE_W;

  if (!enqueue_non_llc_write(dut, kMaster, kAddr, 3, kData, 0xfu, kReqId)) {
    std::printf("FAIL: mode0 DDR partial write was not accepted\n");
    return false;
  }
  if (dut.w_pending.size() != 1) {
    std::printf("FAIL: mode0 DDR write pending count mismatch pending=%zu\n",
                dut.w_pending.size());
    return false;
  }
  const uint8_t axi_id = dut.w_pending.front().axi_id;

  clear_inputs(dut);
  dut.comb_inputs();
  if (!dut.axi_ddr_io.aw.awvalid || dut.axi_mmio_io.aw.awvalid ||
      dut.axi_ddr_io.aw.awaddr != kAlignedAddr ||
      dut.axi_ddr_io.aw.awlen != 0 ||
      dut.axi_ddr_io.aw.awsize != sim_ddr::AXI_SIZE_CODE ||
      dut.axi_ddr_io.aw.awid != axi_id) {
    std::printf("FAIL: mode0 DDR write AW mismatch awvalid=%d "
                "mmio_awvalid=%d awaddr=0x%08x len=%u size=%u id=%u "
                "expected_id=%u\n",
                static_cast<int>(dut.axi_ddr_io.aw.awvalid),
                static_cast<int>(dut.axi_mmio_io.aw.awvalid),
                static_cast<uint32_t>(dut.axi_ddr_io.aw.awaddr),
                static_cast<unsigned>(dut.axi_ddr_io.aw.awlen),
                static_cast<unsigned>(dut.axi_ddr_io.aw.awsize),
                static_cast<unsigned>(dut.axi_ddr_io.aw.awid),
                static_cast<unsigned>(axi_id));
    return false;
  }
  if (dut.axi_ddr_io.w.wvalid || dut.axi_mmio_io.w.wvalid) {
    std::printf("FAIL: mode0 DDR write W issued before AW retired\n");
    return false;
  }

  dut.seq();
  ++sim_time;
  clear_inputs(dut);
  dut.comb_inputs();
  if (!dut.axi_ddr_io.w.wvalid || dut.axi_mmio_io.w.wvalid ||
      !dut.axi_ddr_io.w.wlast ||
      axi_compat::get_u32(dut.axi_ddr_io.w.wdata, 1) != kData) {
    std::printf("FAIL: mode0 DDR write W mismatch wvalid=%d mmio_wvalid=%d "
                "wlast=%d data1=0x%08x\n",
                static_cast<int>(dut.axi_ddr_io.w.wvalid),
                static_cast<int>(dut.axi_mmio_io.w.wvalid),
                static_cast<int>(dut.axi_ddr_io.w.wlast),
                axi_compat::get_u32(dut.axi_ddr_io.w.wdata, 1));
    return false;
  }
  for (uint32_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
    const bool expected = byte >= 4 && byte < 8;
    const bool got = axi_compat::test_bit(dut.axi_ddr_io.w.wstrb, byte);
    if (got != expected) {
      std::printf("FAIL: mode0 DDR write WSTRB mismatch byte=%u got=%d "
                  "expected=%d\n",
                  byte, static_cast<int>(got), static_cast<int>(expected));
      return false;
    }
  }

  dut.seq();
  ++sim_time;
  clear_inputs(dut);
  dut.comb_inputs();
  dut.axi_ddr_io.b.bvalid = true;
  dut.axi_ddr_io.b.bid = axi_id;
  dut.axi_ddr_io.b.bresp = sim_ddr::AXI_RESP_OKAY;
  dut.comb_outputs();
  if (!dut.axi_ddr_io.b.bready) {
    std::printf("FAIL: mode0 DDR write B was backpressured\n");
    return false;
  }

  dut.seq();
  ++sim_time;
  if (!dut.w_pending.empty()) {
    std::printf("FAIL: mode0 DDR write pending did not retire pending=%zu\n",
                dut.w_pending.size());
    return false;
  }

  dut.axi_ddr_io.b.bvalid = false;
  dut.write_ports[kMaster].resp.ready = true;
  dut.comb_outputs();
  const auto &resp = dut.write_ports[kMaster].resp;
  if (!resp.valid || resp.id != kReqId || resp.resp != sim_ddr::AXI_RESP_OKAY) {
    std::printf("FAIL: mode0 DDR write response mismatch valid=%d id=%u "
                "resp=%u\n",
                static_cast<int>(resp.valid), static_cast<unsigned>(resp.id),
                static_cast<unsigned>(resp.resp));
    return false;
  }

  dut.seq();
  ++sim_time;
  if (dut.w_resp_valid[kMaster]) {
    std::printf("FAIL: mode0 DDR write response did not retire upstream\n");
    return false;
  }
  return true;
}

bool test_mode0_ddr_cacheline_write_two_beat_b_response_retires() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  constexpr uint32_t kAddr = 0x40000000u;
  constexpr uint8_t kReqId = 0x5u;
  constexpr uint8_t kMaster = axi_interconnect::MASTER_DCACHE_W;
  const auto line_data = line_write_data(0x5000u);
  const auto line_strobe = full_line_strobe();

  if (!enqueue_non_llc_write_payload(dut, kMaster, kAddr, 63, line_data,
                                     line_strobe, kReqId)) {
    std::printf("FAIL: mode0 DDR cacheline write was not accepted\n");
    return false;
  }
  if (dut.w_pending.size() != 1 || dut.w_pending.front().total_beats != 2) {
    std::printf("FAIL: mode0 DDR cacheline write pending mismatch pending=%zu "
                "beats=%u\n",
                dut.w_pending.size(),
                dut.w_pending.empty()
                    ? 0u
                    : static_cast<unsigned>(dut.w_pending.front().total_beats));
    return false;
  }
  const uint8_t axi_id = dut.w_pending.front().axi_id;

  clear_inputs(dut);
  dut.comb_inputs();
  if (!dut.axi_ddr_io.aw.awvalid || dut.axi_mmio_io.aw.awvalid ||
      dut.axi_ddr_io.aw.awaddr != kAddr ||
      dut.axi_ddr_io.aw.awlen != 1 ||
      dut.axi_ddr_io.aw.awsize != sim_ddr::AXI_SIZE_CODE ||
      dut.axi_ddr_io.aw.awid != axi_id) {
    std::printf("FAIL: mode0 DDR cacheline write AW mismatch awvalid=%d "
                "mmio_awvalid=%d awaddr=0x%08x len=%u size=%u id=%u\n",
                static_cast<int>(dut.axi_ddr_io.aw.awvalid),
                static_cast<int>(dut.axi_mmio_io.aw.awvalid),
                static_cast<uint32_t>(dut.axi_ddr_io.aw.awaddr),
                static_cast<unsigned>(dut.axi_ddr_io.aw.awlen),
                static_cast<unsigned>(dut.axi_ddr_io.aw.awsize),
                static_cast<unsigned>(dut.axi_ddr_io.aw.awid));
    return false;
  }

  dut.seq();
  ++sim_time;
  for (uint32_t beat = 0; beat < 2; ++beat) {
    clear_inputs(dut);
    dut.comb_inputs();
    if (!dut.axi_ddr_io.w.wvalid || dut.axi_mmio_io.w.wvalid ||
        static_cast<bool>(dut.axi_ddr_io.w.wlast) != (beat == 1)) {
      std::printf("FAIL: mode0 DDR cacheline write W control mismatch "
                  "beat=%u wvalid=%d mmio_wvalid=%d wlast=%d\n",
                  beat, static_cast<int>(dut.axi_ddr_io.w.wvalid),
                  static_cast<int>(dut.axi_mmio_io.w.wvalid),
                  static_cast<int>(dut.axi_ddr_io.w.wlast));
      return false;
    }
    for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
      const uint32_t expected = 0x5000u + beat * sim_ddr::AXI_DATA_WORDS + word;
      const uint32_t got = axi_compat::get_u32(dut.axi_ddr_io.w.wdata, word);
      if (got != expected) {
        std::printf("FAIL: mode0 DDR cacheline write WDATA mismatch beat=%u "
                    "word=%u got=0x%08x expected=0x%08x\n",
                    beat, word, got, expected);
        return false;
      }
    }
    for (uint32_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
      if (!axi_compat::test_bit(dut.axi_ddr_io.w.wstrb, byte)) {
        std::printf("FAIL: mode0 DDR cacheline write WSTRB dropped byte=%u "
                    "beat=%u\n",
                    byte, beat);
        return false;
      }
    }
    dut.seq();
    ++sim_time;
  }

  clear_inputs(dut);
  dut.comb_inputs();
  dut.axi_ddr_io.b.bvalid = true;
  dut.axi_ddr_io.b.bid = axi_id;
  dut.axi_ddr_io.b.bresp = sim_ddr::AXI_RESP_OKAY;
  dut.comb_outputs();
  if (!dut.axi_ddr_io.b.bready) {
    std::printf("FAIL: mode0 DDR cacheline write B was backpressured\n");
    return false;
  }
  dut.seq();
  ++sim_time;

  dut.axi_ddr_io.b.bvalid = false;
  dut.write_ports[kMaster].resp.ready = true;
  dut.comb_outputs();
  const auto &resp = dut.write_ports[kMaster].resp;
  if (!dut.w_pending.empty() || !resp.valid || resp.id != kReqId ||
      resp.resp != sim_ddr::AXI_RESP_OKAY) {
    std::printf("FAIL: mode0 DDR cacheline write response mismatch "
                "pending=%zu valid=%d id=%u resp=%u\n",
                dut.w_pending.size(), static_cast<int>(resp.valid),
                static_cast<unsigned>(resp.id),
                static_cast<unsigned>(resp.resp));
    return false;
  }

  dut.seq();
  ++sim_time;
  if (dut.w_resp_valid[kMaster]) {
    std::printf("FAIL: mode0 DDR cacheline write response did not retire "
                "upstream\n");
    return false;
  }
  return true;
}

bool test_mode0_mmio_word_write_uses_mmio_port() {
  axi_interconnect::AXI_Interconnect dut;
  init_dut(dut);
  clear_inputs(dut);

  if (!enqueue_non_llc_write(dut, axi_interconnect::MASTER_UNCORE_LSU_W,
                             0x10000004u, 3, 0x11223344u, 0xfu, 0x12)) {
    std::printf("FAIL: mode0 MMIO word write was not accepted\n");
    return false;
  }
  if (dut.w_pending.empty()) {
    std::printf("FAIL: mode0 MMIO write was not enqueued\n");
    return false;
  }
  const auto &txn = dut.w_pending.back();
  if (txn.port != axi_interconnect::DownstreamPort::MMIO ||
      txn.addr != 0x10000004u || txn.total_beats != 1 ||
      txn.wdata[0] != 0x11223344u || txn.wstrb.slice_u32(0) != 0xfu) {
    std::printf("FAIL: mode0 MMIO write shape mismatch addr=0x%08x "
                "beats=%u data0=0x%08x strb=0x%llx\n",
                txn.addr, static_cast<unsigned>(txn.total_beats),
                txn.wdata[0],
                static_cast<unsigned long long>(txn.wstrb.slice_u32(0)));
    return false;
  }
  return true;
}

bool test_mode2_mapped_write_captures_direct_mapped_llc() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut_mode(dut, 2, 0x30000000u);
  clear_inputs(dut);

  if (!capture_llc_write(dut, axi_interconnect::MASTER_DCACHE_W,
                         0x30000004u, 3, 0x55667788u, 0xfu, 0x13)) {
    std::printf("FAIL: mode2 mapped write was not captured\n");
    return false;
  }
  const auto &cap = dut.llc_upstream_write_req[axi_interconnect::MASTER_DCACHE_W];
  if (cap.addr != 0x00000004u || !cap.direct_mapped || cap.bypass ||
      cap.mode2_ddr_aligned) {
    std::printf("FAIL: mode2 mapped write flags mismatch addr=0x%08x "
                "direct=%d bypass=%d aligned=%d\n",
                cap.addr, static_cast<int>(cap.direct_mapped),
                static_cast<int>(cap.bypass),
                static_cast<int>(cap.mode2_ddr_aligned));
    return false;
  }
  return true;
}

bool test_mode3_aliases_mode0_ddr_write() {
  axi_interconnect::AXI_Interconnect dut;
  init_llc_dut_mode(dut, 3, 0x30000000u);
  clear_inputs(dut);

  if (dut.active_mode() != 0 || dut.llc_enabled()) {
    std::printf("FAIL: encoded mode3 did not alias LLC_OFF active=%u "
                "llc_enabled=%d\n",
                static_cast<unsigned>(dut.active_mode()),
                static_cast<int>(dut.llc_enabled()));
    return false;
  }

  if (!enqueue_non_llc_write(dut, axi_interconnect::MASTER_DCACHE_W,
                             0x40000004u, 3, 0x99aabbccu, 0xfu, 0x14)) {
    std::printf("FAIL: encoded mode3 DDR write was not accepted as mode0\n");
    return false;
  }
  if (dut.w_pending.empty() ||
      dut.w_pending.back().port != axi_interconnect::DownstreamPort::DDR) {
    std::printf("FAIL: encoded mode3 DDR write did not use non-LLC DDR path\n");
    return false;
  }
  return true;
}

} // namespace

int main() {
  int passed = 0;
  int failed = 0;
  auto run = [&](const char *name, bool (*fn)()) {
    std::printf("=== %s ===\n", name);
    if (fn()) {
      std::printf("PASS\n");
      ++passed;
    } else {
      ++failed;
    }
  };

  run("DDR read routes to port0", test_ddr_read_routes_to_port0);
  run("mode0 DDR read response slices aligned beat",
      test_mode0_ddr_read_response_slices_aligned_beat);
  run("mode0 DDR cacheline read 2-beat response",
      test_mode0_ddr_cacheline_read_two_beat_response);
  run("same master read response completion order is stable",
      test_same_master_direct_read_response_completion_order_stable);
  run("MMIO read routes to port1", test_mmio_read_routes_to_port1);
  run("MMIO cacheline read blocks", test_mmio_large_read_blocks);
  run("MMIO cacheline write blocks", test_mmio_large_write_blocks);
  run("LLC unsupported MMIO read synthesizes response",
      test_llc_unsupported_mmio_read_synthesizes_response);
  run("LLC unsupported MMIO write synthesizes response",
      test_llc_unsupported_mmio_write_synthesizes_response);
  run("LLC unsupported MMIO upstream read blocks",
      test_llc_unsupported_mmio_upstream_read_blocks);
  run("LLC DCache read capture pulses accepted/id",
      test_llc_dcache_read_capture_pulses_accepted_id);
  run("LLC MMIO word read bypasses LLC core",
      test_llc_mmio_word_read_bypasses_llc_core);
  run("LLC direct MMIO read resp blocks LLC resp ready",
      test_llc_direct_mmio_read_resp_blocks_llc_resp_ready);
  run("LLC MMIO word write bypasses LLC core",
      test_llc_mmio_word_write_bypasses_llc_core);
  run("LLC direct MMIO write B returns upstream",
      test_llc_direct_mmio_write_b_returns_upstream);
  run("LLC mem write B stays LLC-owned",
      test_llc_mem_write_b_stays_llc_owned);
  run("LLC direct write resp blocks LLC resp ready",
      test_llc_direct_write_resp_blocks_llc_resp_ready);
  run("DDR and MMIO AR issue same-cycle",
      test_ddr_and_mmio_read_issue_same_cycle);
  run("DDR and MMIO AW issue same-cycle",
      test_ddr_and_mmio_aw_issue_same_cycle);
  run("DDR and MMIO W issue same-cycle", test_ddr_and_mmio_w_issue_same_cycle);
  run("same-line AW waits for R", test_same_line_write_waits_for_read_return);
  run("same-line AW releases after R buffered",
      test_same_line_write_releases_after_r_buffered);
  run("same-line AR waits for B", test_same_line_read_waits_for_write_b);
  run("same-line AR releases after B buffered",
      test_same_line_read_releases_after_b_buffered);
  run("MMIO AR not blocked by DDR write",
      test_different_port_read_not_blocked_by_ddr_write);
  run("MMIO AW not blocked by DDR read",
      test_different_port_write_not_blocked_by_ddr_read);
  run("LLC mem write waits for same-line R",
      test_llc_mem_write_waits_for_read_return);
  run("LLC mem read waits for same-line B",
      test_llc_mem_read_waits_for_write_b);
  run("shared read budget blocks both ports",
      test_shared_read_outstanding_budget_blocks_both_ports);
  run("read outstanding does not block write budget",
      test_read_outstanding_does_not_block_write_budget);
  run("shared write budget blocks both ports",
      test_shared_write_outstanding_budget_blocks_both_ports);
  run("write outstanding does not block read budget",
      test_write_outstanding_does_not_block_read_budget);
  run("mode0 DDR partial write aligns to 256b",
      test_mode0_ddr_partial_write_aligns_to_256b);
  run("mode0 DDR partial write B response retires",
      test_mode0_ddr_partial_write_b_response_retires);
  run("mode0 DDR cacheline write 2-beat B response retires",
      test_mode0_ddr_cacheline_write_two_beat_b_response_retires);
  run("mode0 MMIO word write uses MMIO port",
      test_mode0_mmio_word_write_uses_mmio_port);
  run("mode2 mapped write captures direct-mapped LLC",
      test_mode2_mapped_write_captures_direct_mapped_llc);
  run("encoded mode3 aliases mode0 DDR write",
      test_mode3_aliases_mode0_ddr_write);

  std::printf("dual-port routing results: %d passed, %d failed\n", passed,
              failed);
  return failed == 0 ? 0 : 1;
}

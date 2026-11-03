/**
 * @file axi_interconnect_32b_wstall_test.cpp
 * @brief Focused 32B-beat write-stall coverage for AXI_Interconnect.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#define private public
#include "AXI_Interconnect.h"
#undef private
#include "SimDDR.h"

uint32_t *p_memory = nullptr;
long long sim_time = 0;

namespace {

constexpr uint32_t TEST_MEM_SIZE_WORDS = 0x100000;
constexpr int kTimeout = sim_ddr::SIM_DDR_LATENCY * 80;

static_assert(sim_ddr::AXI_DATA_BYTES == 32,
              "axi_interconnect_32b_wstall_test must use 32B beats");
static_assert(sim_ddr::SIM_DDR_WRITE_ACCEPT_GAP == 1,
              "axi_interconnect_32b_wstall_test expects a 1-cycle write accept gap");

struct AwEvent {
  uint32_t addr = 0;
  uint8_t id = 0;
  uint8_t len = 0;
};

struct WEvent {
  uint32_t words[sim_ddr::AXI_DATA_WORDS] = {};
  uint32_t strb = 0;
  bool last = false;
};

struct TestEnv {
  axi_interconnect::AXI_Interconnect interconnect;
  sim_ddr::SimDDR ddr;
  std::vector<AwEvent> aw_events{};
  std::vector<WEvent> w_events{};
};

void clear_upstream_inputs(axi_interconnect::AXI_Interconnect &interconnect) {
  for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
    interconnect.read_ports[i].req.valid = false;
    interconnect.read_ports[i].req.addr = 0;
    interconnect.read_ports[i].req.total_size = 0;
    interconnect.read_ports[i].req.id = 0;
    interconnect.read_ports[i].req.bypass = false;
    interconnect.read_ports[i].resp.ready = false;
  }
  for (int i = 0; i < axi_interconnect::NUM_WRITE_MASTERS; ++i) {
    interconnect.write_ports[i].req.valid = false;
    interconnect.write_ports[i].req.addr = 0;
    interconnect.write_ports[i].req.wdata.clear();
    interconnect.write_ports[i].req.wstrb.clear();
    interconnect.write_ports[i].req.total_size = 0;
    interconnect.write_ports[i].req.id = 0;
    interconnect.write_ports[i].req.bypass = false;
    interconnect.write_ports[i].resp.ready = false;
  }
}

void cycle_outputs(TestEnv &env) {
  clear_upstream_inputs(env.interconnect);

  env.ddr.comb_outputs();
  env.interconnect.axi_io.ar.arready = env.ddr.io.ar.arready;
  env.interconnect.axi_io.r.rvalid = env.ddr.io.r.rvalid;
  env.interconnect.axi_io.r.rid = env.ddr.io.r.rid;
  env.interconnect.axi_io.r.rdata = env.ddr.io.r.rdata;
  env.interconnect.axi_io.r.rlast = env.ddr.io.r.rlast;
  env.interconnect.axi_io.r.rresp = env.ddr.io.r.rresp;
  env.interconnect.axi_io.aw.awready = env.ddr.io.aw.awready;
  env.interconnect.axi_io.w.wready = env.ddr.io.w.wready;
  env.interconnect.axi_io.b.bvalid = env.ddr.io.b.bvalid;
  env.interconnect.axi_io.b.bid = env.ddr.io.b.bid;
  env.interconnect.axi_io.b.bresp = env.ddr.io.b.bresp;

  env.interconnect.comb_outputs();
}

void comb_inputs_and_record(TestEnv &env) {
  env.interconnect.comb_inputs();

  if (env.interconnect.axi_io.aw.awvalid && env.interconnect.axi_io.aw.awready) {
    env.aw_events.push_back(
        {.addr = env.interconnect.axi_io.aw.awaddr,
         .id = static_cast<uint8_t>(env.interconnect.axi_io.aw.awid),
         .len = static_cast<uint8_t>(env.interconnect.axi_io.aw.awlen)});
  }
  if (env.interconnect.axi_io.w.wvalid && env.interconnect.axi_io.w.wready) {
    WEvent event{};
    for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
      event.words[word] =
          axi_compat::get_u32(env.interconnect.axi_io.w.wdata, word);
    }
    for (uint8_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
      if (axi_compat::test_bit(env.interconnect.axi_io.w.wstrb, byte)) {
        event.strb |= (1u << byte);
      }
    }
    event.last = env.interconnect.axi_io.w.wlast;
    env.w_events.push_back(event);
  }
}

void step_fabric(TestEnv &env) {
  env.ddr.io.ar.arvalid = env.interconnect.axi_io.ar.arvalid;
  env.ddr.io.ar.araddr = env.interconnect.axi_io.ar.araddr;
  env.ddr.io.ar.arid = env.interconnect.axi_io.ar.arid;
  env.ddr.io.ar.arlen = env.interconnect.axi_io.ar.arlen;
  env.ddr.io.ar.arsize = env.interconnect.axi_io.ar.arsize;
  env.ddr.io.ar.arburst = env.interconnect.axi_io.ar.arburst;

  env.ddr.io.aw.awvalid = env.interconnect.axi_io.aw.awvalid;
  env.ddr.io.aw.awaddr = env.interconnect.axi_io.aw.awaddr;
  env.ddr.io.aw.awid = env.interconnect.axi_io.aw.awid;
  env.ddr.io.aw.awlen = env.interconnect.axi_io.aw.awlen;
  env.ddr.io.aw.awsize = env.interconnect.axi_io.aw.awsize;
  env.ddr.io.aw.awburst = env.interconnect.axi_io.aw.awburst;

  env.ddr.io.w.wvalid = env.interconnect.axi_io.w.wvalid;
  env.ddr.io.w.wdata = env.interconnect.axi_io.w.wdata;
  env.ddr.io.w.wstrb = env.interconnect.axi_io.w.wstrb;
  env.ddr.io.w.wlast = env.interconnect.axi_io.w.wlast;

  env.ddr.io.r.rready = env.interconnect.axi_io.r.rready;
  env.ddr.io.b.bready = env.interconnect.axi_io.b.bready;

  env.ddr.comb_inputs();
  env.ddr.seq();
  env.interconnect.seq();
  sim_time++;
}

void cycle_inputs(TestEnv &env) {
  comb_inputs_and_record(env);
  step_fabric(env);
}

void init_env(TestEnv &env) {
  std::memset(p_memory, 0, TEST_MEM_SIZE_WORDS * sizeof(uint32_t));
  sim_time = 0;
  env.interconnect.init();
  env.ddr.init();
  env.aw_events.clear();
  env.w_events.clear();
}

axi_interconnect::WideWriteData_t make_line_write_data(uint32_t base_word) {
  axi_interconnect::WideWriteData_t data;
  data.clear();
  for (uint32_t word = 0; word < axi_interconnect::MAX_WRITE_TRANSACTION_WORDS;
       ++word) {
    data[word] = base_word + word;
  }
  return data;
}

axi_interconnect::WideWriteStrb_t make_full_strobe() {
  axi_interconnect::WideWriteStrb_t strobe;
  strobe.clear();
  for (uint32_t byte = 0; byte < axi_interconnect::MAX_WRITE_TRANSACTION_BYTES;
       ++byte) {
    strobe.set(byte, true);
  }
  return strobe;
}

bool issue_write(TestEnv &env, uint8_t master, uint32_t addr,
                 const axi_interconnect::WideWriteData_t &wdata,
                 const axi_interconnect::WideWriteStrb_t &wstrb,
                 uint8_t total_size, uint8_t id) {
  int timeout = kTimeout;
  while (timeout-- > 0) {
    cycle_outputs(env);
    const bool ready_snapshot = env.interconnect.write_ports[master].req.ready;

    auto &wp = env.interconnect.write_ports[master];
    wp.req.valid = true;
    wp.req.addr = addr;
    wp.req.wdata = wdata;
    wp.req.wstrb = wstrb;
    wp.req.total_size = total_size;
    wp.req.id = id;
    wp.req.bypass = false;

    cycle_inputs(env);
    if (ready_snapshot) {
      return true;
    }
  }
  return false;
}

bool wait_write_resp(TestEnv &env, uint8_t master, uint8_t id) {
  int timeout = kTimeout;
  while (timeout-- > 0) {
    cycle_outputs(env);
    auto &resp = env.interconnect.write_ports[master].resp;
    if (resp.valid) {
      if (resp.id != id || resp.resp != sim_ddr::AXI_RESP_OKAY) {
        std::printf("FAIL: write resp mismatch id=%u resp=%u\n",
                    static_cast<unsigned>(resp.id),
                    static_cast<unsigned>(resp.resp));
        return false;
      }
      resp.ready = true;
      cycle_inputs(env);
      return true;
    }
    cycle_inputs(env);
  }
  return false;
}

bool test_write_split_64b_to_32b_under_w_stall(TestEnv &env) {
  std::printf("=== Test: Split 64B write holds payload under W stall ===\n");
  init_env(env);

  const uint32_t addr = 0x8800;
  const auto wdata = make_line_write_data(0x92000000u);
  const auto wstrb = make_full_strobe();

  if (!issue_write(env, axi_interconnect::MASTER_DCACHE_W, addr, wdata, wstrb, 63,
                   0x7u)) {
    std::printf("FAIL: write request was not accepted\n");
    return false;
  }

  bool saw_stalled_second_beat = false;
  bool saw_replayed_second_beat = false;
  WEvent stalled_payload{};
  int timeout = kTimeout;
  while (!saw_replayed_second_beat && timeout-- > 0) {
    cycle_outputs(env);
    comb_inputs_and_record(env);

    if (!saw_stalled_second_beat && env.aw_events.size() == 1 &&
        env.w_events.size() == 1 && env.interconnect.axi_io.w.wvalid &&
        !env.interconnect.axi_io.w.wready) {
      for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
        stalled_payload.words[word] =
            axi_compat::get_u32(env.interconnect.axi_io.w.wdata, word);
      }
      for (uint8_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
        if (axi_compat::test_bit(env.interconnect.axi_io.w.wstrb, byte)) {
          stalled_payload.strb |= (1u << byte);
        }
      }
      stalled_payload.last = env.interconnect.axi_io.w.wlast;
      step_fabric(env);
      if (env.w_events.size() != 1) {
        std::printf("FAIL: stalled cycle unexpectedly consumed a second W beat\n");
        return false;
      }
      saw_stalled_second_beat = true;
      continue;
    }

    if (saw_stalled_second_beat && env.interconnect.axi_io.w.wvalid &&
        env.interconnect.axi_io.w.wready) {
      for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
        const uint32_t got =
            axi_compat::get_u32(env.interconnect.axi_io.w.wdata, word);
        if (got != stalled_payload.words[word]) {
          std::printf("FAIL: replayed W payload word[%u] exp=0x%08x got=0x%08x\n",
                      word, stalled_payload.words[word], got);
          return false;
        }
      }
      uint32_t got_strb = 0;
      for (uint8_t byte = 0; byte < sim_ddr::AXI_DATA_BYTES; ++byte) {
        if (axi_compat::test_bit(env.interconnect.axi_io.w.wstrb, byte)) {
          got_strb |= (1u << byte);
        }
      }
      if (got_strb != stalled_payload.strb ||
          env.interconnect.axi_io.w.wlast != stalled_payload.last) {
        std::printf("FAIL: replayed W control changed strb=0x%08x->0x%08x "
                    "last=%d->%d\n",
                    stalled_payload.strb, got_strb,
                    static_cast<int>(stalled_payload.last),
                    static_cast<int>(env.interconnect.axi_io.w.wlast));
        return false;
      }
      step_fabric(env);
      saw_replayed_second_beat = true;
      continue;
    }

    step_fabric(env);
  }

  if (!saw_stalled_second_beat || !saw_replayed_second_beat) {
    std::printf("FAIL: did not observe stalled then replayed second beat\n");
    return false;
  }

  if (!wait_write_resp(env, axi_interconnect::MASTER_DCACHE_W, 0x7u)) {
    std::printf("FAIL: write response timeout\n");
    return false;
  }

  if (env.aw_events.size() != 1 || env.aw_events[0].len != 1u) {
    std::printf("FAIL: expected one AW len=1, got count=%zu len=%u\n",
                env.aw_events.size(),
                env.aw_events.empty() ? 0u
                                      : static_cast<unsigned>(env.aw_events[0].len));
    return false;
  }
  if (env.w_events.size() != 2) {
    std::printf("FAIL: expected 2 W beats, got %zu\n", env.w_events.size());
    return false;
  }

  for (uint32_t beat = 0; beat < env.w_events.size(); ++beat) {
    if (env.w_events[beat].strb != 0xFFFFFFFFu) {
      std::printf("FAIL: beat %u strobe mismatch exp=0xffffffff got=0x%08x\n",
                  beat, env.w_events[beat].strb);
      return false;
    }
    if (env.w_events[beat].last != (beat == 1u)) {
      std::printf("FAIL: beat %u last mismatch got=%d\n", beat,
                  static_cast<int>(env.w_events[beat].last));
      return false;
    }
    for (uint32_t word = 0; word < sim_ddr::AXI_DATA_WORDS; ++word) {
      const uint32_t exp = 0x92000000u + beat * sim_ddr::AXI_DATA_WORDS + word;
      if (env.w_events[beat].words[word] != exp) {
        std::printf("FAIL: beat %u word %u exp=0x%08x got=0x%08x\n", beat, word,
                    exp, env.w_events[beat].words[word]);
        return false;
      }
    }
  }

  for (uint32_t word = 0; word < axi_interconnect::MAX_WRITE_TRANSACTION_WORDS;
       ++word) {
    const uint32_t exp = 0x92000000u + word;
    const uint32_t got = p_memory[(addr >> 2) + word];
    if (got != exp) {
      std::printf("FAIL: memory word[%u] exp=0x%08x got=0x%08x\n", word, exp,
                  got);
      return false;
    }
  }

  std::printf("PASS\n");
  return true;
}

} // namespace

int main() {
  p_memory = static_cast<uint32_t *>(
      std::calloc(TEST_MEM_SIZE_WORDS, sizeof(uint32_t)));
  if (p_memory == nullptr) {
    std::printf("FAIL: could not allocate test memory\n");
    return 1;
  }

  TestEnv env;
  const int rc = test_write_split_64b_to_32b_under_w_stall(env) ? 0 : 1;
  std::free(p_memory);
  return rc;
}

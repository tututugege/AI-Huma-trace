#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "AXI_Interconnect.h"
#include "SimDDR.h"

extern uint32_t *p_memory;
extern long long sim_time;

namespace axi_test {

using namespace axi_interconnect;

struct ArEvent {
  uint32_t addr = 0;
  uint8_t id = 0;
  uint8_t len = 0;
};

struct AwEvent {
  uint32_t addr = 0;
  uint8_t id = 0;
  uint8_t len = 0;
};

struct FakeLlcTables {
  AXI_LLCConfig config{};
  AXI_LLC_LookupIn_t lookup_in{};
  std::vector<AXI_LLC_Bytes_t> data_sets{};
  std::vector<AXI_LLC_Bytes_t> meta_sets{};
  std::vector<AXI_LLC_Bytes_t> valid_sets{};
  std::vector<AXI_LLC_Bytes_t> repl_sets{};

  bool pending_data = false;
  bool pending_meta = false;
  bool pending_valid = false;
  bool pending_repl = false;
  uint32_t pending_data_index = 0;
  uint32_t pending_meta_index = 0;
  uint32_t pending_valid_index = 0;
  uint32_t pending_repl_index = 0;

  void init(const AXI_LLCConfig &cfg) {
    config = cfg;
    const uint32_t sets = config.set_count();
    data_sets.assign(sets, {});
    meta_sets.assign(sets, {});
    valid_sets.assign(sets, {});
    repl_sets.assign(sets, {});
    for (uint32_t set = 0; set < sets; ++set) {
      data_sets[set].resize(static_cast<size_t>(config.ways) * config.line_bytes);
      meta_sets[set].resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
      valid_sets[set].resize(AXI_LLC::valid_row_bytes(config));
      repl_sets[set].resize(AXI_LLC_REPL_BYTES);
    }
    lookup_in = {};
    pending_data = pending_meta = pending_valid = pending_repl = false;
    pending_data_index = pending_meta_index = pending_valid_index =
        pending_repl_index = 0;
  }

  void comb_outputs() {
    lookup_in = {};
    if (pending_data && pending_data_index < data_sets.size()) {
      lookup_in.data_valid = true;
      lookup_in.data = data_sets[pending_data_index];
    }
    if (pending_meta && pending_meta_index < meta_sets.size()) {
      lookup_in.meta_valid = true;
      lookup_in.meta = meta_sets[pending_meta_index];
    }
    if (pending_valid && pending_valid_index < valid_sets.size()) {
      lookup_in.valid_valid = true;
      lookup_in.valid = valid_sets[pending_valid_index];
    }
    if (pending_repl && pending_repl_index < repl_sets.size()) {
      lookup_in.repl_valid = true;
      lookup_in.repl = repl_sets[pending_repl_index];
    }
  }

  static void write_way_payload(AXI_LLC_Bytes_t &set_payload, uint32_t way,
                                uint32_t bytes_per_way,
                                const AXI_LLC_Bytes_t &payload,
                                const std::vector<uint8_t> &byte_enable) {
    const size_t base = static_cast<size_t>(way) * bytes_per_way;
    for (uint32_t i = 0; i < bytes_per_way; ++i) {
      if (base + i >= set_payload.size() || i >= payload.size()) {
        break;
      }
      if (!byte_enable.empty() && i < byte_enable.size() && byte_enable[i] == 0) {
        continue;
      }
      set_payload.data()[base + i] = payload.data()[i];
    }
  }

  static void write_plain_payload(AXI_LLC_Bytes_t &dst,
                                  const AXI_LLC_Bytes_t &payload,
                                  const std::vector<uint8_t> &byte_enable) {
    const size_t limit = std::min(dst.size(), payload.size());
    for (size_t i = 0; i < limit; ++i) {
      if (!byte_enable.empty() && i < byte_enable.size() && byte_enable[i] == 0) {
        continue;
      }
      dst.data()[i] = payload.data()[i];
    }
  }

  static void write_valid_payload(AXI_LLC_Bytes_t &dst,
                                  const AXI_LLC_Bytes_t &payload,
                                  const std::vector<uint8_t> &byte_enable) {
    write_plain_payload(dst, payload, byte_enable);
  }

  void seq(const AXI_LLC_TableOut_t &table_out) {
    pending_data = pending_meta = pending_valid = pending_repl = false;

    if (table_out.invalidate_all) {
      for (auto &valid : valid_sets) {
        valid.resize(AXI_LLC::valid_row_bytes(config));
        std::fill(valid.data(), valid.data() + valid.size(), 0);
      }
    }

    if (table_out.data.write && table_out.data.index < data_sets.size()) {
      write_way_payload(data_sets[table_out.data.index], table_out.data.way,
                        config.line_bytes, table_out.data.payload,
                        table_out.data.byte_enable);
    } else if (table_out.data.enable && table_out.data.index < data_sets.size()) {
      pending_data = true;
      pending_data_index = table_out.data.index;
    }

    if (table_out.meta.write && table_out.meta.index < meta_sets.size()) {
      write_way_payload(meta_sets[table_out.meta.index], table_out.meta.way,
                        AXI_LLC_META_ENTRY_BYTES, table_out.meta.payload,
                        table_out.meta.byte_enable);
    } else if (table_out.meta.enable && table_out.meta.index < meta_sets.size()) {
      pending_meta = true;
      pending_meta_index = table_out.meta.index;
    }

    if (table_out.valid.write && table_out.valid.index < valid_sets.size()) {
      write_valid_payload(valid_sets[table_out.valid.index],
                          table_out.valid.payload,
                          table_out.valid.byte_enable);
    } else if (table_out.valid.enable && table_out.valid.index < valid_sets.size()) {
      pending_valid = true;
      pending_valid_index = table_out.valid.index;
    }

    if (table_out.repl.write && table_out.repl.index < repl_sets.size()) {
      write_plain_payload(repl_sets[table_out.repl.index], table_out.repl.payload,
                          table_out.repl.byte_enable);
    } else if (table_out.repl.enable && table_out.repl.index < repl_sets.size()) {
      pending_repl = true;
      pending_repl_index = table_out.repl.index;
    }
  }
};

struct Axi4LlcTestEnv {
  AXI_Interconnect interconnect;
  sim_ddr::SimDDR ddr;
  FakeLlcTables tables;
  std::vector<ArEvent> ar_events{};
  std::vector<AwEvent> aw_events{};
};

inline AXI_LLCConfig make_small_llc_config() {
  AXI_LLCConfig cfg;
  cfg.enable = true;
  cfg.size_bytes = 512;
  cfg.line_bytes = 64;
  cfg.ways = 2;
  cfg.mshr_num = 2;
  cfg.prefetch_enable = false;
  return cfg;
}

inline void clear_upstream_inputs(AXI_Interconnect &interconnect) {
  for (int i = 0; i < NUM_READ_MASTERS; ++i) {
    interconnect.read_ports[i].req.valid = false;
    interconnect.read_ports[i].req.addr = 0;
    interconnect.read_ports[i].req.total_size = 0;
    interconnect.read_ports[i].req.id = 0;
    interconnect.read_ports[i].req.bypass = false;
    interconnect.read_ports[i].resp.ready = false;
  }
  for (int i = 0; i < NUM_WRITE_MASTERS; ++i) {
    interconnect.write_ports[i].req.valid = false;
    interconnect.write_ports[i].req.addr = 0;
    interconnect.write_ports[i].req.total_size = 0;
    interconnect.write_ports[i].req.id = 0;
    interconnect.write_ports[i].req.bypass = false;
    interconnect.write_ports[i].req.wdata.clear();
    interconnect.write_ports[i].req.wstrb.clear();
    interconnect.write_ports[i].resp.ready = false;
  }
}

inline void apply_downstream_outputs(Axi4LlcTestEnv &env) {
  env.tables.comb_outputs();
  env.interconnect.set_llc_lookup_in(env.tables.lookup_in);

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
}

inline void cycle_outputs(Axi4LlcTestEnv &env) {
  clear_upstream_inputs(env.interconnect);
  apply_downstream_outputs(env);
  env.interconnect.comb_outputs();
}

inline void commit_cycle_inputs(Axi4LlcTestEnv &env) {
  env.interconnect.comb_inputs();

  if (env.interconnect.axi_io.ar.arvalid && env.interconnect.axi_io.ar.arready) {
    env.ar_events.push_back({env.interconnect.axi_io.ar.araddr,
                             static_cast<uint8_t>(env.interconnect.axi_io.ar.arid),
                             static_cast<uint8_t>(env.interconnect.axi_io.ar.arlen)});
  }
  if (env.interconnect.axi_io.aw.awvalid && env.interconnect.axi_io.aw.awready) {
    env.aw_events.push_back({env.interconnect.axi_io.aw.awaddr,
                             static_cast<uint8_t>(env.interconnect.axi_io.aw.awid),
                             static_cast<uint8_t>(env.interconnect.axi_io.aw.awlen)});
  }

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
  env.tables.seq(env.interconnect.get_llc_table_out());
  env.ddr.seq();
  env.interconnect.seq();
  ++sim_time;
}

inline void cycle_inputs(Axi4LlcTestEnv &env) { commit_cycle_inputs(env); }

inline void init_env(Axi4LlcTestEnv &env) {
  const AXI_LLCConfig cfg = make_small_llc_config();
  env.interconnect.set_llc_config(cfg);
  env.interconnect.init();
  env.ddr.init();
  env.tables.init(cfg);
  env.ar_events.clear();
  env.aw_events.clear();
}

inline void init_env(Axi4LlcTestEnv &env, const AXI_LLCConfig &cfg) {
  env.interconnect.set_llc_config(cfg);
  env.interconnect.init();
  env.ddr.init();
  env.tables.init(cfg);
  env.ar_events.clear();
  env.aw_events.clear();
}

inline uint32_t read_mem_word(uint32_t addr) { return p_memory[addr >> 2]; }

inline void write_memory_line(uint32_t line_addr, uint32_t base_word) {
  for (uint32_t i = 0; i < 16; ++i) {
    p_memory[(line_addr >> 2) + i] = base_word + i;
  }
}

inline bool wait_mem_word(Axi4LlcTestEnv &env, uint32_t addr,
                          uint32_t expected,
                          int timeout = sim_ddr::SIM_DDR_LATENCY * 80) {
  while (timeout-- > 0) {
    if (read_mem_word(addr) == expected) {
      return true;
    }
    cycle_outputs(env);
    cycle_inputs(env);
  }
  return read_mem_word(addr) == expected;
}

inline WideWriteStrb_t make_full_write_strobe(uint32_t bytes = 64) {
  WideWriteStrb_t wstrb;
  wstrb.clear();
  for (uint32_t i = 0; i < bytes; ++i) {
    wstrb.set(i, true);
  }
  return wstrb;
}

inline WideWriteData_t make_line_write_data(uint32_t base_word) {
  WideWriteData_t wdata;
  wdata.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    wdata[i] = base_word + i;
  }
  return wdata;
}

inline bool issue_read(Axi4LlcTestEnv &env, uint8_t master, uint32_t addr,
                       uint8_t total_size, uint8_t id, bool bypass) {
  int timeout = 200;
  while (timeout-- > 0) {
    cycle_outputs(env);

    auto &rp = env.interconnect.read_ports[master];
    rp.req.valid = true;
    rp.req.addr = addr;
    rp.req.total_size = total_size;
    rp.req.id = id;
    rp.req.bypass = bypass;

    cycle_inputs(env);
    if (env.interconnect.read_req_accepted[master]) {
      return true;
    }
  }
  return false;
}

inline bool issue_write(Axi4LlcTestEnv &env, uint8_t master, uint32_t addr,
                        const WideWriteData_t &wdata,
                        const WideWriteStrb_t &wstrb, uint8_t total_size,
                        uint8_t id, bool bypass) {
  int timeout = 200;
  while (timeout-- > 0) {
    cycle_outputs(env);

    auto &wp = env.interconnect.write_ports[master];
    wp.req.valid = true;
    wp.req.addr = addr;
    wp.req.wdata = wdata;
    wp.req.wstrb = wstrb;
    wp.req.total_size = total_size;
    wp.req.id = id;
    wp.req.bypass = bypass;

    cycle_inputs(env);
    if (env.interconnect.write_req_accepted[master]) {
      return true;
    }
  }
  return false;
}

inline bool wait_read_resp(Axi4LlcTestEnv &env, uint8_t master, uint8_t id,
                           uint32_t exp_word0, uint32_t exp_word1) {
  int timeout = sim_ddr::SIM_DDR_LATENCY * 80;
  while (timeout-- > 0) {
    cycle_outputs(env);
    auto &resp = env.interconnect.read_ports[master].resp;
    if (resp.valid) {
      if (resp.id != id || resp.data[0] != exp_word0 || resp.data[1] != exp_word1) {
        std::printf("FAIL: read resp mismatch id=%u d0=0x%x d1=0x%x\n", resp.id,
                    resp.data[0], resp.data[1]);
        return false;
      }
      resp.ready = true;
      cycle_inputs(env);
      cycle_outputs(env);
      env.interconnect.read_ports[master].resp.ready = true;
      cycle_inputs(env);
      return true;
    }
    cycle_inputs(env);
  }
  std::printf("FAIL: read resp timeout master=%u id=%u\n", master, id);
  return false;
}

inline bool wait_write_resp(Axi4LlcTestEnv &env, uint8_t master, uint8_t id) {
  int timeout = sim_ddr::SIM_DDR_LATENCY * 80;
  while (timeout-- > 0) {
    cycle_outputs(env);
    auto &resp = env.interconnect.write_ports[master].resp;
    if (resp.valid) {
      if (resp.id != id || resp.resp != sim_ddr::AXI_RESP_OKAY) {
        std::printf("FAIL: write resp mismatch id=%u resp=%u\n", resp.id, resp.resp);
        return false;
      }
      resp.ready = true;
      cycle_inputs(env);
      cycle_outputs(env);
      env.interconnect.write_ports[master].resp.ready = true;
      cycle_inputs(env);
      return true;
    }
    cycle_inputs(env);
  }
  std::printf("FAIL: write resp timeout master=%u id=%u\n", master, id);
  env.interconnect.debug_print();
  env.ddr.print_state();
  return false;
}

} // namespace axi_test

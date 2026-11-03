#include "SimDDR.h"
#include "config.h"
#include "sc_axi4_sim_api.h"

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <getopt.h>
#include <iostream>
#include <limits>
#include <string>

namespace {

struct SimConfig {
  std::string image_path;
  uint64_t max_inst = MAX_COMMIT_INST;
  uint64_t max_cycles = 12000000000ULL;
};

bool parse_u64(const char *str, uint64_t &value) {
  if (str == nullptr || *str == '\0') {
    return false;
  }
  char *end = nullptr;
  value = std::strtoull(str, &end, 0);
  return end != nullptr && *end == '\0';
}

void print_help(const char *argv0) {
  std::cout << "Usage: " << argv0 << " [options] <binary_image>\n"
            << "Options:\n"
            << "  --max-inst <N>    Maximum executed instructions\n"
            << "  --max-cycles <N>  Maximum simulated cycles\n"
            << "  -h, --help        Show this message\n";
}

bool parse_args(int argc, char **argv, SimConfig &cfg) {
  static struct option long_options[] = {
      {"max-inst", required_argument, nullptr, 'i'},
      {"max-cycles", required_argument, nullptr, 'c'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, 0, nullptr, 0},
  };

  int opt = 0;
  int long_idx = 0;
  while ((opt = getopt_long(argc, argv, "h", long_options, &long_idx)) != -1) {
    switch (opt) {
    case 'i':
      if (!parse_u64(optarg, cfg.max_inst)) {
        std::cerr << "Invalid --max-inst: " << optarg << std::endl;
        return false;
      }
      break;
    case 'c':
      if (!parse_u64(optarg, cfg.max_cycles)) {
        std::cerr << "Invalid --max-cycles: " << optarg << std::endl;
        return false;
      }
      break;
    case 'h':
      print_help(argv[0]);
      std::exit(0);
    default:
      print_help(argv[0]);
      return false;
    }
  }

  if (optind >= argc) {
    print_help(argv[0]);
    return false;
  }
  cfg.image_path = argv[optind];

  if (const char *env_target_inst = std::getenv("TARGET_INST")) {
    uint64_t parsed = 0;
    if (parse_u64(env_target_inst, parsed) && parsed > 0) {
      cfg.max_inst = parsed;
    }
  }
  return true;
}

struct AxiTraceWriter {
  bool enabled = false;
  uint64_t emitted = 0;
  uint64_t max_cycles = std::numeric_limits<uint64_t>::max();
  std::ofstream file;

  void init_from_env() {
    const char *trace_flag = std::getenv("AXI_TRACE");
    const char *trace_path = std::getenv("AXI_TRACE_FILE");
    const char *trace_max = std::getenv("AXI_TRACE_MAX_CYCLES");

    if (trace_flag && std::string(trace_flag) == "0") {
      enabled = false;
      return;
    }
    if ((trace_flag && std::string(trace_flag) == "1") || trace_path) {
      enabled = true;
    }
    if (!enabled) {
      return;
    }
    if (trace_max) {
      uint64_t parsed = 0;
      if (parse_u64(trace_max, parsed) && parsed > 0) {
        max_cycles = parsed;
      }
    }

    std::string path = trace_path ? std::string(trace_path) : "axi4_trace.csv";
    file.open(path, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
      std::cerr << "Warning: cannot open AXI trace file: " << path << std::endl;
      enabled = false;
      return;
    }
    file << "cycle,"
         << "arvalid,arready,arid,araddr,arlen,arsize,"
         << "awvalid,awready,awid,awaddr,awlen,awsize,"
         << "wvalid,wready,wdata,wstrb,wlast,"
         << "rvalid,rready,rid,rdata,rlast,"
         << "bvalid,bready,bid,bresp\n";
  }

  void emit(const sc_sim_status_t &status, const sc_axi4_in_t &in,
            const sc_axi4_out_t &out) {
    if (!enabled || !file.is_open() || emitted >= max_cycles) {
      return;
    }
    file << status.sim_time << ","
         << static_cast<uint32_t>(out.arvalid) << ","
         << static_cast<uint32_t>(in.arready) << ","
         << static_cast<uint32_t>(out.arid) << ","
         << static_cast<uint32_t>(out.araddr) << ","
         << static_cast<uint32_t>(out.arlen) << ","
         << static_cast<uint32_t>(out.arsize) << ","
         << static_cast<uint32_t>(out.awvalid) << ","
         << static_cast<uint32_t>(in.awready) << ","
         << static_cast<uint32_t>(out.awid) << ","
         << static_cast<uint32_t>(out.awaddr) << ","
         << static_cast<uint32_t>(out.awlen) << ","
         << static_cast<uint32_t>(out.awsize) << ","
         << static_cast<uint32_t>(out.wvalid) << ","
         << static_cast<uint32_t>(in.wready) << ","
         << static_cast<uint32_t>(out.wdata) << ","
         << static_cast<uint32_t>(out.wstrb) << ","
         << static_cast<uint32_t>(out.wlast) << ","
         << static_cast<uint32_t>(in.rvalid) << ","
         << static_cast<uint32_t>(out.rready) << ","
         << static_cast<uint32_t>(in.rid) << ","
         << static_cast<uint32_t>(in.rdata) << ","
         << static_cast<uint32_t>(in.rlast) << ","
         << static_cast<uint32_t>(in.bvalid) << ","
         << static_cast<uint32_t>(out.bready) << ","
         << static_cast<uint32_t>(in.bid) << ","
         << static_cast<uint32_t>(in.bresp) << "\n";
    emitted++;
  }
};

void sample_ddr_outputs(const sim_ddr::SimDDR_IO_t &ddr_io, sc_axi4_in_t &in) {
  in.arready = ddr_io.ar.arready;
  in.awready = ddr_io.aw.awready;
  in.wready = ddr_io.w.wready;
  in.rvalid = ddr_io.r.rvalid;
  in.rid = ddr_io.r.rid;
  in.rdata = ddr_io.r.rdata;
  in.rresp = ddr_io.r.rresp;
  in.rlast = ddr_io.r.rlast;
  in.bvalid = ddr_io.b.bvalid;
  in.bid = ddr_io.b.bid;
  in.bresp = ddr_io.b.bresp;
}

void drive_ddr_inputs(sim_ddr::SimDDR_IO_t &ddr_io, const sc_axi4_out_t &out) {
  ddr_io.ar.arvalid = out.arvalid;
  ddr_io.ar.arid = out.arid;
  ddr_io.ar.araddr = out.araddr;
  ddr_io.ar.arlen = out.arlen;
  ddr_io.ar.arsize = out.arsize;
  ddr_io.ar.arburst = out.arburst;

  ddr_io.aw.awvalid = out.awvalid;
  ddr_io.aw.awid = out.awid;
  ddr_io.aw.awaddr = out.awaddr;
  ddr_io.aw.awlen = out.awlen;
  ddr_io.aw.awsize = out.awsize;
  ddr_io.aw.awburst = out.awburst;

  ddr_io.w.wvalid = out.wvalid;
  ddr_io.w.wdata = out.wdata;
  ddr_io.w.wstrb = out.wstrb;
  ddr_io.w.wlast = out.wlast;

  ddr_io.r.rready = out.rready;
  ddr_io.b.bready = out.bready;
}

} // namespace

int main(int argc, char **argv) {
  SimConfig cfg{};
  if (!parse_args(argc, argv, cfg)) {
    return 1;
  }

  sc_sim_handle *sim = sc_sim_create();
  if (sim == nullptr) {
    std::cerr << "Error: failed to create simulator handle" << std::endl;
    return 1;
  }
  sc_sim_set_limits(sim, cfg.max_inst, cfg.max_cycles);

  uint64_t image_size = 0;
  if (sc_sim_load_image(sim, cfg.image_path.c_str(), &image_size) != 0) {
    std::cerr << "Error: " << sc_sim_last_error(sim) << std::endl;
    sc_sim_destroy(sim);
    return 1;
  }

  sim_ddr::SimDDR ddr;
  ddr.init();
  ddr.comb_outputs();

  AxiTraceWriter trace_writer;
  trace_writer.init_from_env();

  std::cout << "[single-cycle-axi4] image=" << cfg.image_path
            << " size=" << image_size
            << " max_inst=" << cfg.max_inst
            << " max_cycles=" << cfg.max_cycles
            << " ddr_latency=" << sim_ddr::SIM_DDR_LATENCY << std::endl;

  sc_axi4_in_t axi_in{};
  sc_axi4_out_t axi_out{};
  sc_sim_status_t status{};

  int rc = 0;
  uint64_t last_progress_inst = 0;
  while (true) {
    sample_ddr_outputs(ddr.io, axi_in);

    rc = sc_sim_step(sim, &axi_in, &axi_out, &status);
    trace_writer.emit(status, axi_in, axi_out);

    if (status.uart_valid) {
      std::cout << static_cast<char>(status.uart_ch) << std::flush;
    }

    drive_ddr_inputs(ddr.io, axi_out);
    ddr.comb_inputs();
    ddr.seq();
    ddr.comb_outputs();

    if (status.inst_count / 5000000ull != last_progress_inst / 5000000ull) {
      std::cout << "[single-cycle-axi4] inst=" << status.inst_count
                << " sim_time=" << status.sim_time << std::endl;
      last_progress_inst = status.inst_count;
    }

    if (rc != 0) {
      break;
    }
  }

  if (rc > 0 && status.success) {
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Success!!!!" << std::endl;
    if (status.inst_count >= cfg.max_inst) {
      std::cout << "reason=max_inst_reached" << std::endl;
    }
    std::cout << "inst_count=" << status.inst_count
              << " sim_time=" << status.sim_time << std::endl;
    std::cout << "-----------------------------" << std::endl;
    sc_sim_destroy(sim);
    return 0;
  }

  std::cout << "------------------------------" << std::endl;
  std::cout << "TIME OUT / ABORT" << std::endl;
  std::cout << "inst_count=" << status.inst_count
            << " sim_time=" << status.sim_time << std::endl;
  const char *last_error = sc_sim_last_error(sim);
  if (last_error != nullptr && last_error[0] != '\0') {
    std::cout << "error=" << last_error << std::endl;
  }
  std::cout << "------------------------------" << std::endl;
  sc_sim_destroy(sim);
  return 1;
}

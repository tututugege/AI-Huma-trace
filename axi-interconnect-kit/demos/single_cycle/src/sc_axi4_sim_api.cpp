#include "sc_axi4_sim_api.h"

#include "AXI_Interconnect.h"
#include "CSR.h"
#include "RISCV.h"
#include "config.h"
#include "single_cycle_cpu.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <new>
#include <string>

long long sim_time = 0;
uint32_t *p_memory = nullptr;

namespace {

constexpr uint32_t kImageBase = 0x80000000u;
constexpr uint8_t kFetchReqId = 0;
constexpr uint8_t kDataReqId = 1;
constexpr uint8_t kMmuReqId = 2;

class SingleCycleAxi4Sim;
static SingleCycleAxi4Sim *g_active_sim = nullptr;
static CpuMemReadResult cpu_mem_read_hook(uint32_t paddr, uint32_t *data);
static bool cpu_mem_read_now_hook(uint32_t paddr, uint32_t *data);
static bool cpu_mem_write_now_hook(uint32_t paddr, uint32_t data,
                                   uint32_t wstrb);

inline int32_t sext(uint32_t value, int bits) {
  const uint32_t sign_bit = 1u << (bits - 1);
  return static_cast<int32_t>((value ^ sign_bit) - sign_bit);
}

inline uint8_t calc_beats(uint8_t total_size) {
  const uint8_t bytes = static_cast<uint8_t>(total_size + 1);
  return static_cast<uint8_t>((bytes + 3) / 4);
}

inline void apply_wstrb_write(uint32_t addr, uint32_t data, uint8_t wstrb) {
  if (p_memory == nullptr) {
    return;
  }
  const uint32_t word_addr = addr >> 2;
  uint32_t old_data = p_memory[word_addr];
  uint32_t mask = 0;
  if (wstrb & 0x1) {
    mask |= 0x000000FFu;
  }
  if (wstrb & 0x2) {
    mask |= 0x0000FF00u;
  }
  if (wstrb & 0x4) {
    mask |= 0x00FF0000u;
  }
  if (wstrb & 0x8) {
    mask |= 0xFF000000u;
  }
  p_memory[word_addr] = (data & mask) | (old_data & ~mask);
}

struct DecodedMemReq {
  bool valid = false;
  bool is_read = true;
  uint32_t paddr = 0;
  uint8_t total_size = 0;
  uint32_t wdata = 0;
  uint32_t wstrb = 0;
};

struct ReadReqState {
  bool active = false;
  bool issued = false;
  uint8_t master = 0;
  uint8_t id = 0;
  uint32_t addr = 0;
  uint8_t total_size = 0;
  uint8_t beats_total = 0;
  uint8_t beats_seen = 0;
};

struct WriteReqState {
  bool active = false;
  bool issued = false;
  uint8_t id = 0;
  uint32_t addr = 0;
  uint32_t wdata = 0;
  uint8_t wstrb = 0;
  uint8_t total_size = 0;
  uint8_t beats_total = 0;
  uint8_t beats_seen = 0;
};

enum class ExecStage : uint8_t {
  kPrepareFetch = 0,
  kWaitFetch = 1,
  kPrepareData = 2,
  kWaitData = 3,
  kExecute = 4,
  kWaitAmoWrite = 5,
  kHalted = 6,
};

static const char *stage_name(ExecStage stage) {
  switch (stage) {
  case ExecStage::kPrepareFetch:
    return "PrepareFetch";
  case ExecStage::kWaitFetch:
    return "WaitFetch";
  case ExecStage::kPrepareData:
    return "PrepareData";
  case ExecStage::kWaitData:
    return "WaitData";
  case ExecStage::kExecute:
    return "Execute";
  case ExecStage::kWaitAmoWrite:
    return "WaitAmoWrite";
  case ExecStage::kHalted:
    return "Halted";
  }
  return "Unknown";
}

struct MmuHookState {
  bool pending = false;
  bool response_valid = false;
  uint32_t addr = 0;
  uint32_t data = 0;
};

class SingleCycleAxi4Sim {
public:
  SingleCycleAxi4Sim() { init_runtime(); }

  ~SingleCycleAxi4Sim() {
    if (g_active_sim == this) {
      g_active_sim = nullptr;
      g_cpu_mem_read32_hook = nullptr;
      g_cpu_mem_read32_now_hook = nullptr;
      g_cpu_mem_write32_now_hook = nullptr;
    }
    if (p_memory != nullptr) {
      delete[] p_memory;
      p_memory = nullptr;
    }
  }

  int load_image(const char *image_path, uint64_t *image_size_out) {
    if (image_path == nullptr) {
      set_error("image path is null");
      return -1;
    }
    if (p_memory == nullptr) {
      set_error("memory not initialized");
      return -1;
    }

    std::ifstream image(image_path, std::ios::binary);
    if (!image.is_open()) {
      set_error(std::string("image not found: ") + image_path);
      return -1;
    }

    image.seekg(0, std::ios::end);
    const size_t image_size = static_cast<size_t>(image.tellg());
    image.seekg(0, std::ios::beg);

    char *dst = reinterpret_cast<char *>(p_memory + (kImageBase >> 2));
    if (!image.read(dst, static_cast<std::streamsize>(image_size))) {
      set_error(std::string("failed to read image: ") + image_path);
      return -1;
    }

    p_memory[0x0u / 4] = 0xf1402573;
    p_memory[0x4u / 4] = 0x83e005b7;
    p_memory[0x8u / 4] = 0x800002b7;
    p_memory[0xcu / 4] = 0x00028067;
    p_memory[0x10000004u / 4] = 0x00006000;

    if (image_size_out != nullptr) {
      *image_size_out = static_cast<uint64_t>(image_size);
    }

    image_loaded_ = true;
    reset_machine_state();
    return 0;
  }

  void set_limits(uint64_t max_inst, uint64_t max_cycles) {
    max_inst_ = max_inst;
    max_cycles_ = max_cycles;
  }

  int step(const sc_axi4_in_t &axi_in, sc_axi4_out_t &axi_out,
           sc_sim_status_t &status) {
    clear_uart_event();
    clear_error_if_running();
    std::memset(&axi_out, 0, sizeof(axi_out));

    if (!image_loaded_) {
      set_error("image not loaded");
      fill_status(status);
      return -1;
    }

    if (stage_ == ExecStage::kHalted) {
      fill_axi_outputs(axi_out);
      fill_status(status);
      return success_ ? 1 : -1;
    }

    apply_axi_inputs(axi_in);
    interconnect_.comb_outputs();

    clear_master_inputs();

    bool req_ready = false;
    bool resp_valid = false;
    drive_current_stage(req_ready, resp_valid);

    interconnect_.comb_inputs();
    fill_axi_outputs(axi_out);
    mirror_read_data(axi_in, axi_out);
    mirror_write_data(axi_in, axi_out);

    interconnect_.seq();
    sim_time++;

    update_stage_after_cycle(req_ready, resp_valid);
    check_limits();
    fill_status(status);

    if (stage_ == ExecStage::kHalted) {
      return success_ ? 1 : -1;
    }
    return 0;
  }

  void get_status(sc_sim_status_t &status) const { fill_status(status); }

  const char *last_error() const {
    if (last_error_.empty()) {
      return "";
    }
    return last_error_.c_str();
  }

  CpuMemReadResult on_cpu_mem_read(uint32_t paddr, uint32_t *data) {
    if (data == nullptr || p_memory == nullptr) {
      return CPU_MEM_READ_FAULT;
    }

    const uint32_t aligned_addr = paddr & ~0x3u;
    if (mmu_hook_.response_valid && mmu_hook_.addr != aligned_addr) {
      mmu_hook_.response_valid = false;
      mmu_hook_.pending = false;
    }

    if (mmu_hook_.response_valid && mmu_hook_.addr == aligned_addr) {
      *data = mmu_hook_.data;
      mmu_hook_.response_valid = false;
      mmu_hook_.pending = false;
      return CPU_MEM_READ_OK;
    }

    if (mmu_hook_.pending && !mmu_req_.active && !mmu_hook_.response_valid) {
      mmu_hook_.pending = false;
    }

    if (mmu_hook_.pending) {
      return CPU_MEM_READ_PENDING;
    }

    mmu_hook_.pending = true;
    mmu_hook_.response_valid = false;
    mmu_hook_.addr = aligned_addr;
    mmu_hook_.data = 0;
    setup_read(mmu_req_, axi_interconnect::MASTER_MMU, kMmuReqId, aligned_addr,
               3);
    return CPU_MEM_READ_PENDING;
  }

private:
  void init_runtime() {
    if (p_memory == nullptr) {
      p_memory = new (std::nothrow) uint32_t[PHYSICAL_MEMORY_LENGTH];
    }
    if (p_memory == nullptr) {
      set_error("failed to allocate physical memory");
      stage_ = ExecStage::kHalted;
      success_ = false;
      return;
    }
    image_loaded_ = false;
    max_inst_ = MAX_COMMIT_INST;
    max_cycles_ = 12000000000ULL;
    reset_machine_state();
  }

  void reset_machine_state() {
    sim_time = 0;
    inst_count_ = 0;
    success_ = false;
    halted_reason_max_inst_ = false;
    halted_reason_ebreak_ = false;
    stage_ = ExecStage::kPrepareFetch;
    fetch_ok_ = false;
    fetch_vaddr_ = 0;
    fetch_paddr_ = 0;
    inst_word_ = 0;
    pre_req_ = {};
    fetch_req_ = {};
    data_req_ = {};
    mmu_req_ = {};
    write_req_ = {};
    mmu_req_ready_ = false;
    mmu_resp_valid_ = false;
    mmu_hook_ = {};
    uart_valid_ = false;
    uart_ch_ = 0;
    last_inst_count_ = 0;
    last_progress_time_ = 0;
    stall_reported_ = false;

    cpu_core_.init(0);
    cpu_core_.memory = p_memory;
    g_active_sim = this;
    g_cpu_mem_read32_hook = cpu_mem_read_hook;
    g_cpu_mem_read32_now_hook = cpu_mem_read_now_hook;
    g_cpu_mem_write32_now_hook = cpu_mem_write_now_hook;
    interconnect_.init();
  }

  void clear_error_if_running() {
    if (stage_ != ExecStage::kHalted) {
      last_error_.clear();
    }
  }

  void clear_uart_event() {
    uart_valid_ = false;
    uart_ch_ = 0;
  }

  void set_error(const std::string &error) { last_error_ = error; }

  static bool translate_addr(SingleCycleCpu &cpu_core, uint32_t vaddr, uint32_t type,
                             uint32_t &paddr) {
    if ((cpu_core.state.csr[csr_satp] & 0x80000000u) &&
        cpu_core.privilege != RISCV_MODE_M) {
      return cpu_core.va2pa(paddr, vaddr, type);
    }
    paddr = vaddr;
    return true;
  }

  static DecodedMemReq decode_mem_req_pre_exec(SingleCycleCpu &cpu_core,
                                               uint32_t inst_word) {
    DecodedMemReq req{};
    const uint32_t opcode = inst_word & 0x7f;
    const uint32_t rs1 = (inst_word >> 15) & 0x1f;
    const uint32_t rs2 = (inst_word >> 20) & 0x1f;
    const uint32_t funct3 = (inst_word >> 12) & 0x7;
    uint32_t vaddr = 0;
    uint32_t paddr = 0;

    if (opcode == 0x03) {
      const int32_t imm_i = sext((inst_word >> 20) & 0xfff, 12);
      vaddr = cpu_core.state.gpr[rs1] + static_cast<uint32_t>(imm_i);
      if (!translate_addr(cpu_core, vaddr, 1, paddr)) {
        return req;
      }
      req.valid = true;
      req.is_read = true;
      req.paddr = paddr;
      switch (funct3) {
      case 0:
      case 4:
        req.total_size = 0;
        break;
      case 1:
      case 5:
        req.total_size = 1;
        break;
      case 2:
        req.total_size = 3;
        break;
      default:
        req.valid = false;
        break;
      }
      return req;
    }

    if (opcode == 0x23) {
      const uint32_t imm = ((inst_word >> 25) << 5) | ((inst_word >> 7) & 0x1f);
      const int32_t imm_s = sext(imm, 12);
      const uint32_t rs2_data = cpu_core.state.gpr[rs2];
      vaddr = cpu_core.state.gpr[rs1] + static_cast<uint32_t>(imm_s);
      if (!translate_addr(cpu_core, vaddr, 2, paddr)) {
        return req;
      }
      const uint32_t offset = paddr & 0x3u;
      req.valid = true;
      req.is_read = false;
      req.paddr = paddr;
      switch (funct3) {
      case 0:
        req.total_size = 0;
        req.wstrb = (1u << offset);
        req.wdata = (rs2_data & 0xffu) << (offset * 8);
        break;
      case 1:
        req.total_size = 1;
        req.wstrb = (0x3u << offset) & 0xfu;
        req.wdata = (rs2_data & 0xffffu) << (offset * 8);
        break;
      case 2:
        req.total_size = 3;
        req.wstrb = (0xfu << offset) & 0xfu;
        req.wdata = rs2_data << (offset * 8);
        break;
      default:
        req.valid = false;
        break;
      }
      return req;
    }

    if (opcode == 0x2f) {
      const uint32_t vaddr_amo = cpu_core.state.gpr[rs1];
      if (!translate_addr(cpu_core, vaddr_amo, 1, paddr)) {
        return req;
      }
      req.valid = true;
      req.is_read = true;
      req.paddr = paddr;
      req.total_size = 3;
      return req;
    }

    return req;
  }

  static uint8_t encode_axi_id(uint8_t master, uint8_t id) {
    return static_cast<uint8_t>((master << 2) | (id & 0x3));
  }

  void setup_read(ReadReqState &read_req, uint8_t master, uint8_t id,
                  uint32_t addr, uint8_t total_size) {
    read_req = {};
    read_req.active = true;
    read_req.master = master;
    read_req.id = id;
    read_req.addr = addr;
    read_req.total_size = total_size;
    read_req.beats_total = calc_beats(total_size);
  }

  void setup_write(WriteReqState &write_req, uint8_t id, uint32_t addr,
                   uint32_t wdata, uint8_t wstrb, uint8_t total_size) {
    write_req = {};
    write_req.active = true;
    write_req.id = id;
    write_req.addr = addr;
    write_req.wdata = wdata;
    write_req.wstrb = wstrb;
    write_req.total_size = total_size;
    write_req.beats_total = calc_beats(total_size);
  }

  void apply_axi_inputs(const sc_axi4_in_t &axi_in) {
    interconnect_.axi_io.ar.arready = (axi_in.arready != 0);
    interconnect_.axi_io.aw.awready = (axi_in.awready != 0);
    interconnect_.axi_io.w.wready = (axi_in.wready != 0);
    interconnect_.axi_io.r.rvalid = (axi_in.rvalid != 0);
    interconnect_.axi_io.r.rid = axi_in.rid;
    interconnect_.axi_io.r.rdata = axi_in.rdata;
    interconnect_.axi_io.r.rresp = axi_in.rresp;
    interconnect_.axi_io.r.rlast = (axi_in.rlast != 0);
    interconnect_.axi_io.b.bvalid = (axi_in.bvalid != 0);
    interconnect_.axi_io.b.bid = axi_in.bid;
    interconnect_.axi_io.b.bresp = axi_in.bresp;
  }

  void fill_axi_outputs(sc_axi4_out_t &axi_out) const {
    axi_out.arvalid = interconnect_.axi_io.ar.arvalid;
    axi_out.arid = interconnect_.axi_io.ar.arid;
    axi_out.araddr = interconnect_.axi_io.ar.araddr;
    axi_out.arlen = interconnect_.axi_io.ar.arlen;
    axi_out.arsize = interconnect_.axi_io.ar.arsize;
    axi_out.arburst = interconnect_.axi_io.ar.arburst;

    axi_out.awvalid = interconnect_.axi_io.aw.awvalid;
    axi_out.awid = interconnect_.axi_io.aw.awid;
    axi_out.awaddr = interconnect_.axi_io.aw.awaddr;
    axi_out.awlen = interconnect_.axi_io.aw.awlen;
    axi_out.awsize = interconnect_.axi_io.aw.awsize;
    axi_out.awburst = interconnect_.axi_io.aw.awburst;

    axi_out.wvalid = interconnect_.axi_io.w.wvalid;
    axi_out.wdata = interconnect_.axi_io.w.wdata;
    axi_out.wstrb = interconnect_.axi_io.w.wstrb;
    axi_out.wlast = interconnect_.axi_io.w.wlast;

    axi_out.rready = interconnect_.axi_io.r.rready;
    axi_out.bready = interconnect_.axi_io.b.bready;
  }

  void clear_master_inputs() {
    for (int i = 0; i < axi_interconnect::NUM_READ_MASTERS; ++i) {
      interconnect_.read_ports[i].req.valid = false;
      interconnect_.read_ports[i].req.addr = 0;
      interconnect_.read_ports[i].req.total_size = 0;
      interconnect_.read_ports[i].req.id = 0;
      interconnect_.read_ports[i].resp.ready = false;
    }

    interconnect_.write_port.req.valid = false;
    interconnect_.write_port.req.addr = 0;
    interconnect_.write_port.req.wdata.clear();
    interconnect_.write_port.req.wstrb = 0;
    interconnect_.write_port.req.total_size = 0;
    interconnect_.write_port.req.id = 0;
    interconnect_.write_port.resp.ready = false;
  }

  void drive_mmu_request() {
    mmu_req_ready_ = false;
    mmu_resp_valid_ = false;

    if (!mmu_req_.active) {
      return;
    }

    auto &port = interconnect_.read_ports[axi_interconnect::MASTER_MMU];
    mmu_req_ready_ = port.req.ready;
    mmu_resp_valid_ = port.resp.valid;
    port.resp.ready = true;

    // Keep req.valid asserted while active to avoid missing ready-first pulses.
    port.req.valid = true;
    port.req.addr = mmu_req_.addr;
    port.req.total_size = mmu_req_.total_size;
    port.req.id = mmu_req_.id;
  }

  void drive_current_stage(bool &req_ready, bool &resp_valid) {
    req_ready = false;
    resp_valid = false;

    switch (stage_) {
    case ExecStage::kWaitFetch: {
      auto &port = interconnect_.read_ports[axi_interconnect::MASTER_ICACHE];
      req_ready = port.req.ready;
      resp_valid = port.resp.valid;
      port.resp.ready = true;
      if (!fetch_req_.issued) {
        port.req.valid = true;
        port.req.addr = fetch_req_.addr;
        port.req.total_size = fetch_req_.total_size;
        port.req.id = fetch_req_.id;
      }
      break;
    }
    case ExecStage::kWaitData: {
      if (pre_req_.is_read) {
        auto &port = interconnect_.read_ports[axi_interconnect::MASTER_DCACHE_R];
        req_ready = port.req.ready;
        resp_valid = port.resp.valid;
        port.resp.ready = true;
        if (!data_req_.issued) {
          port.req.valid = true;
          port.req.addr = data_req_.addr;
          port.req.total_size = data_req_.total_size;
          port.req.id = data_req_.id;
        }
      } else {
        auto &port = interconnect_.write_port;
        req_ready = port.req.ready;
        resp_valid = port.resp.valid;
        port.resp.ready = true;
        if (!write_req_.issued) {
          port.req.valid = true;
          port.req.addr = write_req_.addr;
          port.req.wdata.clear();
          port.req.wdata[0] = write_req_.wdata;
          port.req.wstrb = write_req_.wstrb;
          port.req.total_size = write_req_.total_size;
          port.req.id = write_req_.id;
        }
      }
      break;
    }
    case ExecStage::kWaitAmoWrite: {
      auto &port = interconnect_.write_port;
      req_ready = port.req.ready;
      resp_valid = port.resp.valid;
      port.resp.ready = true;
      if (!write_req_.issued) {
        port.req.valid = true;
        port.req.addr = write_req_.addr;
        port.req.wdata.clear();
        port.req.wdata[0] = write_req_.wdata;
        port.req.wstrb = write_req_.wstrb;
        port.req.total_size = write_req_.total_size;
        port.req.id = write_req_.id;
      }
      break;
    }
    default:
      break;
    }

    drive_mmu_request();
  }

  void mirror_read_data(const sc_axi4_in_t &axi_in, const sc_axi4_out_t &axi_out) {
    if (axi_in.rvalid == 0 || axi_out.rready == 0 || p_memory == nullptr) {
      return;
    }

    if (fetch_req_.active && fetch_req_.issued &&
        axi_in.rid == encode_axi_id(fetch_req_.master, fetch_req_.id) &&
        fetch_req_.beats_seen < fetch_req_.beats_total) {
      const uint32_t word_addr = (fetch_req_.addr >> 2) + fetch_req_.beats_seen;
      p_memory[word_addr] = axi_in.rdata;
      fetch_req_.beats_seen++;
      return;
    }

    if (mmu_req_.active && mmu_req_.issued &&
        axi_in.rid == encode_axi_id(mmu_req_.master, mmu_req_.id) &&
        mmu_req_.beats_seen < mmu_req_.beats_total) {
      const uint32_t word_addr = (mmu_req_.addr >> 2) + mmu_req_.beats_seen;
      p_memory[word_addr] = axi_in.rdata;
      mmu_hook_.data = axi_in.rdata;
      mmu_req_.beats_seen++;
      return;
    }

    if (data_req_.active && data_req_.issued &&
        axi_in.rid == encode_axi_id(data_req_.master, data_req_.id) &&
        data_req_.beats_seen < data_req_.beats_total) {
      const uint32_t word_addr = (data_req_.addr >> 2) + data_req_.beats_seen;
      p_memory[word_addr] = axi_in.rdata;
      data_req_.beats_seen++;
    }
  }

  void mirror_write_data(const sc_axi4_in_t &axi_in,
                         const sc_axi4_out_t &axi_out) {
    if (axi_out.wvalid == 0 || axi_in.wready == 0 || !write_req_.active) {
      return;
    }
    const uint32_t current_addr =
        write_req_.addr + static_cast<uint32_t>(write_req_.beats_seen) * 4u;
    apply_wstrb_write(current_addr, axi_out.wdata, axi_out.wstrb);

    const uint32_t word_base = current_addr & ~0x3u;
    for (uint32_t lane = 0; lane < 4; ++lane) {
      if ((axi_out.wstrb & (1u << lane)) == 0) {
        continue;
      }
      const uint32_t byte_addr = word_base + lane;
      if (byte_addr == UART_BASE) {
        uart_valid_ = true;
        uart_ch_ = static_cast<uint8_t>((axi_out.wdata >> (lane * 8)) & 0xffu);
      }
    }

    if (write_req_.beats_seen < std::numeric_limits<uint8_t>::max()) {
      write_req_.beats_seen++;
    }
  }

  void update_mmu_request_state() {
    if (!mmu_req_.active) {
      return;
    }

    if (!mmu_req_.issued && mmu_req_ready_) {
      mmu_req_.issued = true;
    }
    if (mmu_req_.issued && mmu_resp_valid_) {
      mmu_req_.active = false;
      mmu_hook_.response_valid = true;
      if (p_memory != nullptr) {
        mmu_hook_.data = p_memory[mmu_hook_.addr >> 2];
      }
    }
  }

  void update_stage_after_cycle(bool req_ready, bool resp_valid) {
    update_mmu_request_state();

    switch (stage_) {
    case ExecStage::kPrepareFetch:
      prepare_fetch();
      break;
    case ExecStage::kWaitFetch:
      if (!fetch_req_.issued && req_ready) {
        fetch_req_.issued = true;
      }
      if (fetch_req_.issued && resp_valid) {
        fetch_req_.active = false;
        inst_word_ = fetch_ok_ ? p_memory[fetch_paddr_ >> 2] : 0u;
        stage_ = ExecStage::kPrepareData;
      }
      break;
    case ExecStage::kPrepareData:
      prepare_data_request();
      break;
    case ExecStage::kWaitData:
      if (pre_req_.is_read) {
        if (!data_req_.issued && req_ready) {
          data_req_.issued = true;
        }
        if (data_req_.issued && resp_valid) {
          data_req_.active = false;
          stage_ = ExecStage::kExecute;
        }
      } else {
        if (!write_req_.issued && req_ready) {
          write_req_.issued = true;
        }
        if (write_req_.issued && resp_valid) {
          write_req_.active = false;
          stage_ = ExecStage::kExecute;
        }
      }
      break;
    case ExecStage::kExecute:
      cpu_core_.exec();
      if (cpu_core_.translation_pending) {
        break;
      }
      inst_count_++;
      last_progress_time_ = static_cast<uint64_t>(sim_time);
      if (inst_count_ != last_inst_count_) {
        last_inst_count_ = inst_count_;
        stall_reported_ = false;
      }

      if (inst_word_ == INST_EBREAK) {
        halted_reason_ebreak_ = true;
        stage_ = ExecStage::kHalted;
        success_ = true;
        break;
      }

      if (((inst_word_ & 0x7f) == 0x2f) && cpu_core_.state.store) {
        const uint8_t amo_wstrb =
            static_cast<uint8_t>(cpu_core_.state.store_strb & 0xfu);
        setup_write(write_req_, kDataReqId, cpu_core_.state.store_addr,
                    cpu_core_.state.store_data,
                    static_cast<uint8_t>(amo_wstrb == 0 ? 0xfu : amo_wstrb), 3);
        stage_ = ExecStage::kWaitAmoWrite;
        break;
      }

      stage_ = ExecStage::kPrepareFetch;
      break;
    case ExecStage::kWaitAmoWrite:
      if (!write_req_.issued && req_ready) {
        write_req_.issued = true;
      }
      if (write_req_.issued && resp_valid) {
        write_req_.active = false;
        stage_ = ExecStage::kPrepareFetch;
      }
      break;
    case ExecStage::kHalted:
      break;
    }
  }

  void prepare_fetch() {
    fetch_vaddr_ = cpu_core_.state.pc;
    fetch_ok_ = translate_addr(cpu_core_, fetch_vaddr_, 0, fetch_paddr_);
    if (cpu_core_.translation_pending) {
      return;
    }
    if (!fetch_ok_) {
      inst_word_ = 0;
      pre_req_ = {};
      stage_ = ExecStage::kExecute;
      return;
    }
    setup_read(fetch_req_, axi_interconnect::MASTER_ICACHE, kFetchReqId,
               fetch_paddr_, 3);
    stage_ = ExecStage::kWaitFetch;
  }

  void prepare_data_request() {
    pre_req_ = decode_mem_req_pre_exec(cpu_core_, inst_word_);
    if (cpu_core_.translation_pending) {
      return;
    }
    if (pre_req_.valid) {
      if (pre_req_.is_read) {
        setup_read(data_req_, axi_interconnect::MASTER_DCACHE_R, kDataReqId,
                   pre_req_.paddr, pre_req_.total_size);
      } else {
        setup_write(write_req_, kDataReqId, pre_req_.paddr, pre_req_.wdata,
                    static_cast<uint8_t>(pre_req_.wstrb), pre_req_.total_size);
      }
      stage_ = ExecStage::kWaitData;
      return;
    }
    stage_ = ExecStage::kExecute;
  }

  void check_limits() {
    if (stage_ == ExecStage::kHalted) {
      return;
    }

    if (inst_count_ >= max_inst_) {
      halted_reason_max_inst_ = true;
      stage_ = ExecStage::kHalted;
      success_ = true;
      return;
    }

    if (static_cast<uint64_t>(sim_time) >= max_cycles_) {
      set_error("max_cycles reached");
      stage_ = ExecStage::kHalted;
      success_ = false;
    }

    constexpr uint64_t kStallCycles = 2000000ULL;
    if (!stall_reported_ &&
        static_cast<uint64_t>(sim_time) > last_progress_time_ + kStallCycles &&
        stage_ != ExecStage::kHalted) {
      stall_reported_ = true;
      std::fprintf(
          stderr,
          "[sc-axi4][stall] time=%llu inst=%llu stage=%s "
          "mmu_pending=%d mmu_resp=%d mmu_addr=0x%08x mmu_req_active=%d "
          "mmu_req_issued=%d mmu_beats=%u/%u mmu_req_ready=%d "
          "arvalid=%d arready=%d arid=%u araddr=0x%08x\n",
          static_cast<unsigned long long>(sim_time),
          static_cast<unsigned long long>(inst_count_), stage_name(stage_),
          mmu_hook_.pending ? 1 : 0, mmu_hook_.response_valid ? 1 : 0,
          mmu_hook_.addr, mmu_req_.active ? 1 : 0, mmu_req_.issued ? 1 : 0,
          mmu_req_.beats_seen, mmu_req_.beats_total, mmu_req_ready_ ? 1 : 0,
          interconnect_.axi_io.ar.arvalid ? 1 : 0,
          interconnect_.axi_io.ar.arready ? 1 : 0,
          static_cast<unsigned>(interconnect_.axi_io.ar.arid),
          interconnect_.axi_io.ar.araddr);
      interconnect_.debug_print();
    }
  }

  void fill_status(sc_sim_status_t &status) const {
    status.sim_time = static_cast<uint64_t>(sim_time);
    status.inst_count = inst_count_;
    status.halted = (stage_ == ExecStage::kHalted) ? 1 : 0;
    status.success = success_ ? 1 : 0;
    status.wait_axi =
        (stage_ == ExecStage::kWaitFetch || stage_ == ExecStage::kWaitData ||
         stage_ == ExecStage::kWaitAmoWrite || mmu_req_.active)
            ? 1
            : 0;
    status.uart_valid = uart_valid_ ? 1 : 0;
    status.uart_ch = uart_ch_;
  }

private:
  SingleCycleCpu cpu_core_{};
  axi_interconnect::AXI_Interconnect interconnect_{};

  ExecStage stage_ = ExecStage::kHalted;
  bool image_loaded_ = false;
  bool success_ = false;
  bool halted_reason_max_inst_ = false;
  bool halted_reason_ebreak_ = false;
  uint64_t max_inst_ = MAX_COMMIT_INST;
  uint64_t max_cycles_ = 12000000000ULL;
  uint64_t inst_count_ = 0;
  uint64_t last_inst_count_ = 0;
  uint64_t last_progress_time_ = 0;
  bool stall_reported_ = false;

  bool fetch_ok_ = false;
  uint32_t fetch_vaddr_ = 0;
  uint32_t fetch_paddr_ = 0;
  uint32_t inst_word_ = 0;
  DecodedMemReq pre_req_{};
  ReadReqState fetch_req_{};
  ReadReqState data_req_{};
  ReadReqState mmu_req_{};
  WriteReqState write_req_{};
  bool mmu_req_ready_ = false;
  bool mmu_resp_valid_ = false;
  MmuHookState mmu_hook_{};

  bool uart_valid_ = false;
  uint8_t uart_ch_ = 0;

  std::string last_error_{};
};

static CpuMemReadResult cpu_mem_read_hook(uint32_t paddr, uint32_t *data) {
  if (g_active_sim == nullptr) {
    return CPU_MEM_READ_FAULT;
  }
  return g_active_sim->on_cpu_mem_read(paddr, data);
}

static bool cpu_mem_read_now_hook(uint32_t paddr, uint32_t *data) {
  if (p_memory == nullptr || data == nullptr) {
    return false;
  }
  *data = p_memory[(paddr & ~0x3u) >> 2];
  return true;
}

static bool cpu_mem_write_now_hook(uint32_t paddr, uint32_t data,
                                   uint32_t wstrb) {
  if (p_memory == nullptr) {
    return false;
  }
  apply_wstrb_write(paddr, data, wstrb);
  return true;
}

} // namespace

struct sc_sim_handle {
  SingleCycleAxi4Sim sim;
};

extern "C" {

sc_sim_handle *sc_sim_create(void) { return new (std::nothrow) sc_sim_handle(); }

void sc_sim_destroy(sc_sim_handle *handle) { delete handle; }

int sc_sim_load_image(sc_sim_handle *handle, const char *image_path,
                      uint64_t *image_size_out) {
  if (handle == nullptr) {
    return -1;
  }
  return handle->sim.load_image(image_path, image_size_out);
}

void sc_sim_set_limits(sc_sim_handle *handle, uint64_t max_inst,
                       uint64_t max_cycles) {
  if (handle == nullptr) {
    return;
  }
  handle->sim.set_limits(max_inst, max_cycles);
}

int sc_sim_step(sc_sim_handle *handle, const sc_axi4_in_t *axi_in,
                sc_axi4_out_t *axi_out, sc_sim_status_t *status_out) {
  if (handle == nullptr || axi_in == nullptr || axi_out == nullptr ||
      status_out == nullptr) {
    return -1;
  }
  return handle->sim.step(*axi_in, *axi_out, *status_out);
}

void sc_sim_get_status(const sc_sim_handle *handle,
                       sc_sim_status_t *status_out) {
  if (handle == nullptr || status_out == nullptr) {
    return;
  }
  handle->sim.get_status(*status_out);
}

const char *sc_sim_last_error(const sc_sim_handle *handle) {
  if (handle == nullptr) {
    return "handle is null";
  }
  return handle->sim.last_error();
}

} // extern "C"

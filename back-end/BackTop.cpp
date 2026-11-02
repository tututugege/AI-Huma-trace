#include "trace_hook.h"
#include "BackTop.h"
#include "Csr.h"
#include "IO.h"
#include "PhysMemory.h"
#include "RealLsu.h"
#ifdef LSU_SHADOW
#include "LsuShadow.h"
#else
#define LSU(f) lsu->f()
#endif
#ifdef ROB_SHADOW
#include "RobShadow.h"
#else
#define ROB(f) rob->f()
#endif
#ifdef IDU_SHADOW
#include "IduShadow.h"
#else
#define IDU(f) idu->f()
#endif
#ifdef CSR_SHADOW
#include "CsrShadow.h"
#else
#define CSR(f) csr->f()
#endif
#ifdef REN_SHADOW
#include "RenShadow.h"
#else
#define REN(f) rename->f()
#endif
#ifdef ISU_SHADOW
#include "IsuShadow.h"
#else
#define ISU(f) isu->f()
#endif
#ifdef EXU_SHADOW
#include "ExuShadow.h"
#else
#define EXU(f) exu->f()
#endif
#ifdef DIS_SHADOW
#include "DisShadow.h"
#else
#define DIS(f) dis->f()
#endif
#include "config.h"
#include "diff.h"
#include "mmio_address_map.h"
#include "oracle.h"
#include "ref.h"
#include "util.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>
#include <zlib.h>

void init_diff_ckpt(CPU_state ckpt_state, uint8_t privilege);

void BackTop::init() {
  pre = new PreIduQueue(ctx);
  idu = new Idu(ctx, MAX_BR_PER_CYCLE);
  rename = new Ren(ctx);
  dis = new Dispatch(ctx);
  isu = new Isu(ctx);
  prf = new Prf(ctx);
  exu = new Exu(ctx);
  csr = new Csr();
  rob = new Rob(ctx);
  lsu = new RealLsu(ctx);
  dtlb_mmu =
      std::make_unique<TlbMmu>(ctx, nullptr, DTLB_ENTRIES, TlbMmu::Kind::DTLB);

  out.fire = pre2front.fire;



















  pre->wires = this;
  idu->wires = this;
  rename->wires = this;
  dis->wires = this;
  isu->wires = this;
  prf->wires = this;
  exu->wires = this;
  rob->wires = this;
  csr->wires = this;
  lsu->wires = this;
  idu_br_latch = &idu->br_latch;

  pre->init();
  idu->init();
  rename->init();
  dis->init();
  isu->init();
  prf->init();
  exu->init();
  csr->init();
  rob->init();
  lsu->init();
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->flush();
    dtlb_mmu->seq();
  }
}

void BackTop::set_lsu_ptw_mem_port(PtwMemPort *port) {
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->set_ptw_mem_port(port);
  }
}

void BackTop::set_lsu_ptw_walk_port(PtwWalkPort *port) {
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->set_ptw_walk_port(port);
  }
}

void BackTop::comb_lsu_mmu() {
  mmu2lsu_io = {};
  if (dtlb_mmu == nullptr) {
    return;
  }

  if (rob_bcast.fence) {
    dtlb_mmu->flush();
    return;
  }
  if (rob_bcast.flush || dec_bcast.mispred) {
    dtlb_mmu->cancel_pending_walk();
    return;
  }

  dtlb_mmu->translate_lsu_ports(lsu2mmu_io, mmu2lsu_io);
}

void BackTop::comb_csr_status() {
  CSR(comb_csr_status);
  out.sstatus = csr_status.sstatus;
  out.mstatus = csr_status.mstatus;
  out.satp = csr_status.satp;
  out.privilege = csr_status.privilege;
}

void BackTop::clear_comb_inputs() { in = Back_in{}; }

// ---------------------------------------------------------------------------------------------
// One back-end cycle is evaluated in dependency order: every stage reads only signals that an
// earlier stage (or registered state) already produced. The call order in BackTop::comb() is that order.
// ---------------------------------------------------------------------------------------------

// Working copies and default outputs of every module.
void BackTop::comb_begin_all() {
  pre->comb_begin();
  IDU(comb_begin);
  REN(comb_begin);
  DIS(comb_begin);
  ISU(comb_begin);
  prf->comb_begin();
  EXU(comb_begin);
  ROB(comb_begin);
  CSR(comb_begin);
}

// Branch/decode results, free resources (rename registers, issue queues, LDQ/STQ) and the wakeups/write-backs completing this cycle.
void BackTop::comb_stage_decode_and_resources() {
  IDU(comb_branch);
  pre->comb_accept_front();
  IDU(comb_decode);
  CSR(comb_interrupt);
  REN(comb_alloc);
  prf->comb_complete();
  prf->comb_awake();
  prf->comb_write();
  ISU(comb_ready);
  LSU(comb_cal);
  LSU(comb_lsu2dis);
  LSU(comb_lsu2rob);
}

// ROB head -> commit group; committed stores release their STQ entries in the LSU.
void BackTop::comb_stage_commit() {
  ROB(comb_ready);
  ROB(comb_ftq_pc_req);
  pre->comb_ftq_lookup_rob();
  ROB(comb_commit);
  pre->comb_commit_info();
  LSU(comb_stq_commit);
}

// MMIO/DCache responses finish loads and stores and produce write-backs; CSR read data.
void BackTop::comb_stage_memory_responses() {
  DIS(comb_alloc);
  LSU(comb_mmio_in);
  LSU(comb_dcache2lsu_ldq);
  LSU(comb_dcache2lsu_stq);
  LSU(comb_lsu2exe);
  EXU(comb_to_csr);
  CSR(comb_csr_read);
}

// Functional units execute; their results complete ROB entries.
void BackTop::comb_stage_execute() {
  EXU(comb_exec);
  ROB(comb_complete);
}

// Issue selection, then LSU address translation (the MMU sits between the LSU's request and response phases).
void BackTop::comb_stage_issue_and_translate() {
  EXU(comb_ready);
  ISU(comb_issue);
  LSU(comb_exe2lsu);
  LSU(comb_tlb_out);
  comb_lsu_mmu();
  LSU(comb_tlb_in_ldq);
  LSU(comb_tlb_in_stq);
  LSU(comb_stlf);
}

// Register reads, next-cycle wakeups, exceptions, rename and dispatch of the instructions leaving decode.
void BackTop::comb_stage_operands_and_dispatch() {
  prf->comb_req_ftq();
  pre->comb_ftq_lookup_prf();
  prf->comb_read();

  ISU(comb_awake);
  ISU(comb_calc_latency_next);
  CSR(comb_exception);
  CSR(comb_csr_write);
  DIS(comb_wake);
  REN(comb_rename);

  DIS(comb_dispatch);
}

// Handshakes that move instructions (fire), the LSU's DCache/MMIO requests, queue enqueue, and recovery of ROB/IQs from a mispredict or global flush (flush wins).
void BackTop::comb_stage_fire_and_recover() {
  DIS(comb_fire);
  LSU(comb_dis2lsu);
  REN(comb_fire);
  IDU(comb_fire);
  // Kill flushed LSU work before driving new DCache/MMIO requests.
  LSU(comb_flush);
  LSU(comb_lsu2dcache_ldq);
  LSU(comb_lsu2dcache_stq);
  LSU(comb_mmio_out);
  LSU(comb_check);
  ISU(comb_enq);
  // Rob recovery was split out of comb_fire(); keep the original priority:
  // branch rollback happens before enqueue, and global flush overrides both.
  ROB(comb_branch);
  ROB(comb_fire);
  ROB(comb_flush);
  ISU(comb_flush);

  pre->comb_fire();
  prf->comb_pipeline();
  EXU(comb_pipeline);
  DIS(comb_pipeline);
  REN(comb_pipeline);
  CSR(comb_interrupt_inject);
}

void BackTop::comb() {
  // CSR 状态由 SimCpu::cycle() 在进入 front/back 组合逻辑前统一刷新。
#if CONFIG_BE_IO_CLEAR_AT_COMB_BEGIN
  // Diagnostic mode: clear backend internal stage IOs to expose hidden
  // dependence on previous-cycle combinational values.
  pre2front = {};
  pre_issue = {};
  ftq_prf_pc_req = {};
  ftq_prf_pc_resp = {};
  ftq_rob_pc_req = {};
  ftq_rob_pc_resp = {};
  ftq_commit_info = {};

  dec2ren = {};
  dec_bcast = {};

  ren2dec = {};
  idu_consume = {};
  ren2dis = {};

  dis2ren = {};
  dis2iss = {};
  dis2rob = {};
  dis2lsu = {};

  iss_awake = {};
  iss2prf = {};
  iss2dis = {};

  prf2exe = {};
  prf_awake = {};

  exu2id = {};
  // Known issue path: ROB consumes exu2rob before EXU refreshes it in current
  // comb order. Keep it out of global clear for now and handle EXU separately.
  exe2prf = {};
  exe2iss = {};
  exe2lsu = {};
  exe2csr = {};

  rob2dis = {};
  rob2csr = {};
  rob_bcast = {};
  rob_commit = {};

  csr2exe = {};
  csr2rob = {};
  csr2front = {};

  lsu2exe = {};
  lsu2dis = {};
  lsu2rob = {};
  out.peripheral_req = {};
  out.lsu2dcache = {};
  lsu2mmu_io = {};
  mmu2lsu_io = {};
#endif

  comb_begin_all();
  comb_stage_decode_and_resources();
  comb_stage_commit();
  comb_stage_memory_responses();
  comb_stage_execute();
  comb_stage_issue_and_translate();
  comb_stage_operands_and_dispatch();

  // 用于调试
  // 修正pc_next 以及difftest对应的pc_next
  out.flush = rob_bcast.flush;
  if (out.flush && tbhook::enabled) {
    int64_t committing = -1;
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      if (rob_commit.commit_entry[i].valid) {
        committing = (int64_t)rob_commit.commit_entry[i].uop->dbg.inst_idx;
        break;
      }
    }
    tbhook::on_flush(committing, rob_bcast.interrupt);
  }
  out.fence_i = rob_bcast.fence_i;
  out.itlb_flush = rob_bcast.fence;

  // 1. Normal case (No Rob flush)
  if (!rob_bcast.flush) {
    out.mispred = dec_bcast.mispred;
    out.stall = !pre2front.ready;
    out.redirect_pc = idu->br_latch.redirect_pc;
  } else {
    out.mispred = true;
    if (rob_bcast.mret || rob_bcast.sret) {
      out.redirect_pc = csr2front.epc;
    } else if (rob_bcast.exception) {
      out.redirect_pc = csr2front.trap_pc;
    } else {
      out.redirect_pc = rob_bcast.pc + 4;
    }
    BE_LOG("flush redirect_pc=0x%08x", (uint32_t)out.redirect_pc);
  }

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    out.commit_entry[i] =
        rob_commit.commit_entry[i].uop.to_inst_entry(
            rob_commit.commit_entry[i].valid);
    if (tbhook::enabled && rob_commit.commit_entry[i].valid &&
        !rob_bcast.interrupt) {
      tbhook::on_commit(
          (uint64_t)rob_commit.commit_entry[i].uop->dbg.inst_idx);
    }
    if (out.commit_entry[i].valid && out.flush) {
      // Flush: override extra_data.pc_next with redirect_pc for ALL
      // instructions so Difftest can read the correct next-PC (trap vector,
      // epc, etc.)
      out.commit_entry[i].uop.diag_val = out.redirect_pc;
      rob_commit.commit_entry[i].uop.diag_val = out.redirect_pc;
    }
  }

  comb_stage_fire_and_recover();
}

void BackTop::seq() {
  // rename -> isu/stq/rob
  // exu -> prf
  pre->seq();
  REN(seq);
  DIS(seq);
  IDU(seq);
  ISU(seq);
  EXU(seq);
  prf->seq();
  ROB(seq);
  CSR(seq);
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->seq();
  }
  LSU(seq);
}

// --- 辅助函数：简化 zlib 读写 POD 类型 ---
template <typename T>
void gz_write_pod(gzFile file, const T &data) {
  if (gzwrite(file, &data, sizeof(T)) != sizeof(T)) {
    Assert(0 && "Error writing data to gzip file.");
  }
}

template <typename T>
void gz_read_pod(gzFile file, T &data) {
  if (gzread(file, &data, sizeof(T)) != sizeof(T)) {
    Assert(0 && "Error reading data from gzip file.");
  }
}

namespace {
void seed_uart_io_words() {
  constexpr uint32_t kUartLsrThre = 0x20u;
  constexpr uint32_t kUartLsrTemt = 0x40u;
  constexpr uint32_t kUartIirNoInt = 0x01u;
  constexpr uint32_t kUartIirFifoEnabled = 0xc0u;
  constexpr uint32_t kUartMsrCts = 0x10u;
  constexpr uint32_t kUartMsrDsr = 0x20u;
  constexpr uint32_t kUartMsrDcd = 0x80u;

  pmem_write(UART_ADDR_BASE + 0x00u,
             (kUartIirNoInt | kUartIirFifoEnabled) << 16);
  pmem_write(UART_ADDR_BASE + 0x04u,
             0x00000003u |
                 ((kUartLsrThre | kUartLsrTemt) << 8) |
                 ((kUartMsrCts | kUartMsrDsr | kUartMsrDcd) << 16));
  pmem_write(UART_ADDR_BASE + 0x08u, 0);
}

constexpr uint64_t kCkptSimpointRamBytes = RAM_SIZE;
constexpr uint64_t kGzChunkSize = 1ULL * 1024 * 1024 * 1024;
constexpr uint64_t kMaxCkptIoRangeSize = 64ULL * 1024 * 1024;
constexpr uint32_t kCkptMagic = 0x006d6552u; // "Rem\0" little-endian
constexpr uint16_t kCkptFormatVersion = 3u;
constexpr uint16_t kCkptLegacyFormatVersion = 2u;

typedef struct CkptHeader {
  uint32_t magic;
  uint16_t format_ver;
  uint16_t privilege_mode;
  uint32_t ram_size;
  uint32_t io_range_count;
} CkptHeader;

typedef struct CkptIoRange {
  uint32_t base;
  uint32_t size;
} CkptIoRange;

void gz_read_exact(gzFile file, uint8_t *dst, uint64_t total_bytes) {
  uint64_t remain = total_bytes;
  while (remain > 0) {
    const unsigned int chunk = static_cast<unsigned int>(
        remain > kGzChunkSize ? kGzChunkSize : remain);
    const int read_bytes = gzread(file, dst, chunk);
    if (read_bytes < 0) {
      Assert(0 && "Error: gzread failed during checkpoint restore.");
    }
    if (read_bytes == 0) {
      Assert(0 && "Error: Unexpected EOF during checkpoint restore.");
    }
    dst += read_bytes;
    remain -= static_cast<uint64_t>(read_bytes);
  }
}

} // namespace

void BackTop::load_image(const std::string &filename) {
  std::ifstream inst_data(filename, std::ios::binary);
  if (!inst_data.is_open()) {
    Assert(0 && "Error: Image does not exist");
  }

  inst_data.seekg(0, std::ios::end);
  std::streamsize size = inst_data.tellg();
  inst_data.seekg(0, std::ios::beg);

  if (size < 0 || static_cast<uint64_t>(size) > RAM_SIZE) {
    Assert(0 && "Image too large for configured RAM window.");
  }
  if (!inst_data.read(reinterpret_cast<char *>(pmem_ram_ptr()), size)) {
    Assert(0 && "读取文件失败！");
  }

  inst_data.close();

  pmem_write(0x0, 0xf1402573);
  pmem_write(0x4, 0x83e005b7);
  pmem_write(0x8, 0x800002b7);
  pmem_write(0xc, 0x00028067);
  seed_uart_io_words();

  init_difftest(size);

}

void BackTop::load_flash_image(const std::string &filename) {
  std::ifstream flash_data(filename, std::ios::binary);
  if (!flash_data.is_open()) {
    Assert(0 && "Error: Flash image does not exist");
  }

  flash_data.seekg(0, std::ios::end);
  std::streamsize size = flash_data.tellg();
  flash_data.seekg(0, std::ios::beg);

  if (size < 0 || static_cast<uint64_t>(size) > BOOT_IO_SIZE) {
    Assert(0 && "Flash image too large for configured boot window.");
  }

  std::vector<uint8_t> bytes(static_cast<size_t>(size));
  if (size > 0 &&
      !flash_data.read(reinterpret_cast<char *>(bytes.data()), size)) {
    Assert(0 && "读取 flash 文件失败！");
  }

  for (size_t i = 0; i < bytes.size(); ++i) {
    const uint32_t addr = static_cast<uint32_t>(i);
    const uint32_t word_addr = addr & ~0x3u;
    const uint32_t shift = (addr & 0x3u) * 8u;
    const uint32_t word = (pmem_read(word_addr) & ~(0xFFu << shift)) |
                          (static_cast<uint32_t>(bytes[i]) << shift);
    pmem_write(word_addr, word);
  }

  constexpr uint32_t kEmbeddedSdLoaderOffset = 0x60u;
  constexpr uint32_t kSdLoaderLoadAddr = 0x83800000u;
  if (bytes.size() > kEmbeddedSdLoaderOffset) {
    const size_t loader_size = bytes.size() - kEmbeddedSdLoaderOffset;
    if (!pmem_is_ram_addr(kSdLoaderLoadAddr,
                          static_cast<uint32_t>(loader_size))) {
      Assert(0 && "Embedded SD loader does not fit in RAM window.");
    }
    pmem_memcpy_to_ram(kSdLoaderLoadAddr,
                       bytes.data() + kEmbeddedSdLoaderOffset, loader_size);

    pmem_write(0x0, 0xf1402573); // csrrs a0, mhartid, zero
    pmem_write(0x4, 0x83e005b7); // lui a1, 0x83e00
    pmem_write(0x8, 0x838002b7); // lui t0, 0x83800
    pmem_write(0xc, 0x00028067); // jalr zero, 0(t0)
  }

  seed_uart_io_words();
}

void BackTop::load_sd_boot_blob(const std::string &sdcard_image,
                                uint32_t sector_offset) {
  std::ifstream image(sdcard_image, std::ios::binary);
  if (!image.is_open()) {
    Assert(0 && "Error: SD card image does not exist");
  }

  constexpr uint32_t kSectorSize = 512u;
  constexpr uint32_t kEntryCount = 4u;
  std::array<uint32_t, kEntryCount * 3> header{};

  image.seekg(static_cast<std::streamoff>(sector_offset) * kSectorSize,
              std::ios::beg);
  if (!image.read(reinterpret_cast<char *>(header.data()), sizeof(header))) {
    Assert(0 && "Error: Failed to read SD boot blob header");
  }

  for (uint32_t i = 0; i < kEntryCount; ++i) {
    const uint32_t load_addr = header[i * 3 + 0];
    const uint32_t card_sector = header[i * 3 + 1];
    const uint32_t len = header[i * 3 + 2];
    if (load_addr == 0 || len == 0) {
      continue;
    }

    const size_t byte_len = static_cast<size_t>(len) * kSectorSize;
    if (!pmem_is_ram_addr(load_addr, static_cast<uint32_t>(byte_len))) {
      Assert(0 && "SD boot blob segment does not fit in RAM");
    }

    std::vector<uint8_t> payload(byte_len);
    const std::streamoff payload_off =
        static_cast<std::streamoff>(sector_offset + card_sector) * kSectorSize;
    image.seekg(payload_off, std::ios::beg);
    if (!image.read(reinterpret_cast<char *>(payload.data()), payload.size())) {
      Assert(0 && "Error: Failed to read SD boot blob payload");
    }
    pmem_memcpy_to_ram(load_addr, payload.data(), payload.size());
  }

  pmem_write(0x0, 0xf1402573); // csrrs a0, mhartid, zero
  pmem_write(0x4, 0x83e005b7); // lui a1, 0x83e00
  pmem_write(0x8, 0x800002b7); // lui t0, 0x80000
  pmem_write(0xc, 0x00028067); // jalr zero, 0(t0)
  seed_uart_io_words();
}

void BackTop::restore_from_ref() {
  CPU_state state;
  uint8_t privilege;
  get_state(state, privilege);
  number_PC = state.pc;

  csr->privilege = csr->privilege_1 = privilege;

  for (int i = 0; i < CSR_NUM; i++) {
    csr->CSR_RegFile[i] = state.csr[i];
    csr->CSR_RegFile_1[i] = state.csr[i];
  }

  // 同步 dut_cpu，全局参考路径（oracle/refetch 校验）依赖该镜像状态。
  std::memcpy(dut_cpu.gpr, state.gpr, sizeof(dut_cpu.gpr));
  std::memcpy(dut_cpu.csr, state.csr, sizeof(dut_cpu.csr));
  dut_cpu.pc = state.pc;
  dut_cpu.store = state.store;
  dut_cpu.store_addr = state.store_addr;
  dut_cpu.store_data = state.store_data;
  dut_cpu.store_strb = state.store_strb;
  dut_cpu.reserve_valid = state.reserve_valid;
  dut_cpu.reserve_addr = state.reserve_addr;
  lsu->cur.lrsc_unit.reserve_valid = state.reserve_valid;
  lsu->cur.lrsc_unit.reserve_addr = state.reserve_addr;
  lsu->nxt.lrsc_unit.reserve_valid = state.reserve_valid;
  lsu->nxt.lrsc_unit.reserve_addr = state.reserve_addr;

  // Ensure the pipeline starts with a refetch from the restored PC
  out.flush = true;
  out.redirect_pc = state.pc;

}

void BackTop::restore_checkpoint(const std::string &filename) {

  std::string final_name = filename;
  gzFile file = gzopen(final_name.c_str(), "rb");
  if (!file && final_name.find(".gz") == std::string::npos) {
    final_name += ".gz";
    file = gzopen(final_name.c_str(), "rb");
  }

  if (!file) {
    Assert(0 && "Error: Could not open checkpoint file");
  }

  // 与 SimpleSim/save_checkpoint 写入格式保持一致，确保反序列化对齐。
  // 注意：reserve_* 当前在本模拟器中暂未使用，但必须读取以避免流错位。
  typedef struct Ckpt_CPU_state {
    uint32_t gpr[32];
    uint32_t csr[21];
    uint32_t pc;

    uint32_t store_addr;
    uint32_t store_data;
    uint32_t store_strb;
    bool store;
    bool reserve_valid;
    uint32_t reserve_addr;
  } Ckpt_CPU_state;

  Ckpt_CPU_state ckpt_state;
  CkptHeader ckpt_header = {};
  uint64_t interval_inst_count;

  // 1. 恢复 header + 状态
  gz_read_pod(file, ckpt_header);
  if (ckpt_header.magic != kCkptMagic) {
    std::cerr << "Error: Invalid checkpoint magic." << std::endl;
    std::exit(1);
  }
  if (ckpt_header.format_ver != kCkptFormatVersion &&
      ckpt_header.format_ver != kCkptLegacyFormatVersion) {
    std::cerr << "Error: Unsupported checkpoint version: "
              << ckpt_header.format_ver << std::endl;
    std::exit(1);
  }
  uint32_t checkpoint_sdram_size = 0;
  if (ckpt_header.format_ver >= kCkptFormatVersion) {
    gz_read_pod(file, checkpoint_sdram_size);
    if (checkpoint_sdram_size != PMEM_SDRAM_SIZE) {
      std::cerr << "Error: SDRAM checkpoint size is incompatible: file=0x"
                << std::hex << checkpoint_sdram_size << " sim=0x"
                << PMEM_SDRAM_SIZE << std::dec << std::endl;
      std::exit(1);
    }
  }
  if (ckpt_header.ram_size == 0 ||
      ckpt_header.ram_size > kCkptSimpointRamBytes) {
    std::cerr << "Error: RAM-only checkpoint size is incompatible: file=0x"
              << std::hex << ckpt_header.ram_size << " sim=0x"
              << kCkptSimpointRamBytes << std::dec << std::endl;
    std::exit(1);
  }

  gz_read_pod(file, ckpt_state);
  gz_read_pod(file, interval_inst_count);
  ckpt_interval_inst_count = interval_inst_count;

  CPU_state state;
  memcpy(state.gpr, ckpt_state.gpr, sizeof(state.gpr));
  memcpy(state.csr, ckpt_state.csr, sizeof(state.csr));
  state.pc = ckpt_state.pc;
  state.store_addr = ckpt_state.store_addr;
  state.store_data = ckpt_state.store_data;
  state.store_strb = ckpt_state.store_strb;
  state.store = ckpt_state.store;
  state.reserve_valid = ckpt_state.reserve_valid;
  state.reserve_addr = ckpt_state.reserve_addr;

  // 初始化新字段
  state.instruction = 0;
  state.page_fault_inst = false;
  state.page_fault_load = false;
  state.page_fault_store = false;
  state.inst_idx = 0;

  number_PC = state.pc;
  const uint8_t restored_privilege =
      static_cast<uint8_t>(ckpt_header.privilege_mode & 0x3u);
  csr->privilege = csr->privilege_1 = restored_privilege;


  for (int i = 0; i < CSR_NUM; i++) {
    csr->CSR_RegFile[i] = state.csr[i];
    csr->CSR_RegFile_1[i] = state.csr[i];
  }

  // Populate global dut_cpu for Oracle state comparison
  memcpy(dut_cpu.gpr, state.gpr, sizeof(dut_cpu.gpr));
  memcpy(dut_cpu.csr, state.csr, sizeof(dut_cpu.csr));
  dut_cpu.pc = state.pc;
  dut_cpu.store = state.store;
  dut_cpu.store_addr = state.store_addr;
  dut_cpu.store_data = state.store_data;
  dut_cpu.store_strb = state.store_strb;
  dut_cpu.reserve_valid = state.reserve_valid;
  dut_cpu.reserve_addr = state.reserve_addr;
  lsu->cur.lrsc_unit.reserve_valid = state.reserve_valid;
  lsu->cur.lrsc_unit.reserve_addr = state.reserve_addr;
  lsu->nxt.lrsc_unit.reserve_valid = state.reserve_valid;
  lsu->nxt.lrsc_unit.reserve_addr = state.reserve_addr;

  // 2. 恢复内存
  if (pmem_ram_ptr() == nullptr || pmem_sdram_ptr() == nullptr) {
    Assert(0 &&
           "Error: Memory backends not allocated during checkpoint restore.");
  }
  pmem_clear_all();

  std::cout << "Restoring Memory... format=simpoint-v"
            << ckpt_header.format_ver << "(header+ranges)"
            << " file_ram=0x" << std::hex << ckpt_header.ram_size
            << " sim_ram=0x" << kCkptSimpointRamBytes << std::dec
            << std::endl;

  uint8_t *ram_dst = reinterpret_cast<uint8_t *>(pmem_ram_ptr());
  gz_read_exact(file, ram_dst, ckpt_header.ram_size);
  if (checkpoint_sdram_size != 0) {
    uint8_t *sdram_dst = reinterpret_cast<uint8_t *>(pmem_sdram_ptr());
    gz_read_exact(file, sdram_dst, checkpoint_sdram_size);
  }

  if (ckpt_header.io_range_count > 64u) {
    Assert(0 && "Error: unreasonable checkpoint MMIO range count");
  }
  if (ckpt_header.format_ver >= kCkptFormatVersion &&
      ckpt_header.io_range_count !=
          sizeof(kCheckpointMmioRanges) / sizeof(kCheckpointMmioRanges[0])) {
    Assert(0 && "Error: checkpoint v3 MMIO range count mismatch");
  }

  uint32_t restored_mmio_ranges = 0;
  for (uint32_t i = 0; i < ckpt_header.io_range_count; ++i) {
    CkptIoRange range{};
    gz_read_pod(file, range);
    if (range.size > kMaxCkptIoRangeSize) {
      Assert(0 && "Error: checkpoint MMIO range is unreasonably large");
    }

    const SimMmioRange *expected = nullptr;
    if (ckpt_header.format_ver >= kCkptFormatVersion) {
      expected = &kCheckpointMmioRanges[i];
      if (range.base != expected->base || range.size != expected->size) {
        Assert(0 && "Error: checkpoint v3 MMIO layout mismatch");
      }
    } else {
      for (const auto &candidate : kCheckpointMmioRanges) {
        if (range.base == candidate.base && range.size == candidate.size) {
          expected = &candidate;
          break;
        }
      }
    }

    std::vector<uint8_t> payload(range.size, 0);
    gz_read_exact(file, payload.data(), payload.size());
    if (expected == nullptr) {
      std::cout << "[CKPT][MMIO] Skipping legacy range base=0x" << std::hex
                << range.base << " size=0x" << range.size << std::dec
                << std::endl;
      continue;
    }
    if ((range.size & 0x3u) != 0) {
      Assert(0 && "Error: checkpoint MMIO range is not word aligned");
    }
    for (uint32_t off = 0; off < range.size; off += 4u) {
      const uint32_t word = static_cast<uint32_t>(payload[off + 0]) |
                            (static_cast<uint32_t>(payload[off + 1]) << 8) |
                            (static_cast<uint32_t>(payload[off + 2]) << 16) |
                            (static_cast<uint32_t>(payload[off + 3]) << 24);
      pmem_write(range.base + off, word);
    }
    ++restored_mmio_ranges;
  }

  gzclose(file);
  std::cout << "[CKPT] Restored DDR, SDRAM and " << restored_mmio_ranges
            << " MMIO ranges." << std::endl;
  std::cout << "Checkpoint restored from " << final_name << std::endl;

  init_diff_ckpt(state, restored_privilege);

  // Ensure the pipeline starts with a refetch from the restored PC
  out.flush = true;
  out.redirect_pc = state.pc;
}

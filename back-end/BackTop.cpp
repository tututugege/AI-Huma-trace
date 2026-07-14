#include "BackTop.h"
#include "Csr.h"
#include "IO.h"
#include "PhysMemory.h"
#include "RealLsu.h"
#include "config.h"
#include "diff.h"
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

  pre->out.pre2front = &pre2front;
  pre->out.issue = &pre_issue;
  pre->out.ftq_prf_pc_resp = &ftq_prf_pc_resp;
  pre->out.ftq_rob_pc_resp = &ftq_rob_pc_resp;
  pre->out.ftq_commit_info = &ftq_commit_info;
  pre->in.front2pre = &in;
  pre->in.idu_consume = &idu_consume;
  pre->in.rob_bcast = &rob_bcast;
  pre->in.rob_commit = &rob_commit;
  pre->in.idu_br_latch = &idu->br_latch;
  pre->in.ftq_prf_pc_req = &ftq_prf_pc_req;
  pre->in.ftq_rob_pc_req = &ftq_rob_pc_req;

  idu->out.dec2ren = &dec2ren;
  idu->out.dec_bcast = &dec_bcast;
  idu->out.idu_consume = &idu_consume;
  idu->in.issue = &pre_issue;
  idu->in.ren2dec = &ren2dec;
  idu->in.rob_bcast = &rob_bcast;
  idu->in.exu2id = &exu2id;

  rename->in.dec2ren = &dec2ren;
  rename->in.dis2ren = &dis2ren;
  rename->in.dec_bcast = &dec_bcast;
  rename->in.rob_bcast = &rob_bcast;
  rename->in.rob_commit = &rob_commit;

  rename->out.ren2dec = &ren2dec;
  rename->out.ren2dis = &ren2dis;

  dis->out.dis2ren = &dis2ren;
  dis->out.dis2iss = &dis2iss;
  dis->out.dis2rob = &dis2rob;
  dis->out.dis2lsu = &dis2lsu;

  dis->in.lsu2dis = &lsu2dis;
  dis->in.ren2dis = &ren2dis;
  dis->in.iss2dis = &iss2dis;
  dis->in.rob2dis = &rob2dis;
  dis->in.prf_awake = &prf_awake;
  dis->in.iss_awake = &iss_awake;
  dis->in.rob_bcast = &rob_bcast;
  dis->in.dec_bcast = &dec_bcast;

  isu->in.rob_bcast = &rob_bcast;
  isu->in.dec_bcast = &dec_bcast;
  isu->in.dis2iss = &dis2iss;
  isu->in.prf_awake = &prf_awake;
  isu->in.exe2iss = &exe2iss;

  isu->out.iss2dis = &iss2dis;
  isu->out.iss2prf = &iss2prf;
  isu->out.iss_awake = &iss_awake;

  prf->out.prf2exe = &prf2exe;
  prf->out.prf_awake = &prf_awake;
  prf->out.ftq_prf_pc_req = &ftq_prf_pc_req;

  prf->in.iss2prf = &iss2prf;
  prf->in.exe2prf = &exe2prf;
  prf->in.dec_bcast = &dec_bcast;
  prf->in.rob_bcast = &rob_bcast;
  prf->in.ftq_prf_pc_resp = &ftq_prf_pc_resp;

  exu->in.prf2exe = &prf2exe;
  exu->in.dec_bcast = &dec_bcast;
  exu->in.rob_bcast = &rob_bcast;
  exu->in.lsu2exe = &lsu2exe;
  exu->in.csr2exe = &csr2exe;
  exu->in.csr_status = &csr_status;

  exu->out.exe2prf = &exe2prf;
  exu->out.exe2iss = &exe2iss;
  exu->out.exe2lsu = &exe2lsu;
  exu->out.exe2lsu = &exe2lsu;
  exu->out.exe2csr = &exe2csr;
  exu->out.exu2id = &exu2id;
  exu->out.exu2rob = &exu2rob;

  rob->in.dis2rob = &dis2rob;
  rob->in.dec_bcast = &dec_bcast;
  rob->in.lsu2rob = &lsu2rob;
  rob->in.csr2rob = &csr2rob;
  rob->in.exu2rob = &exu2rob;
  rob->in.ftq_pc_resp = &ftq_rob_pc_resp;
  rob->in.front_stall = &in.front_stall;

  rob->out.rob_bcast = &rob_bcast;
  rob->out.rob_commit = &rob_commit;
  rob->out.rob2dis = &rob2dis;
  rob->out.rob2csr = &rob2csr;
  rob->out.ftq_pc_req = &ftq_rob_pc_req;

  csr->in.exe2csr = &exe2csr;
  csr->in.rob2csr = &rob2csr;
  csr->in.rob_bcast = &rob_bcast;
  csr->in.interrupt_inject = &csr_interrupt_inject_io;

  csr->out.csr2exe = &csr2exe;
  csr->out.csr2rob = &csr2rob;
  csr->out.csr2front = &csr2front;
  csr->out.csr_status = &csr_status;

  lsu->in.dis2lsu = &dis2lsu;
  lsu->in.exe2lsu = &exe2lsu;
  lsu->in.csr_status = &csr_status;
  lsu->in.rob_bcast = &rob_bcast;
  lsu->in.dec_bcast = &dec_bcast;
  lsu->in.rob_commit = &rob_commit;
  lsu->in.peripheral_resp = &in.peripheral_resp;
  lsu->in.dcache2lsu = &in.dcache2lsu;
  lsu->in.mmu2lsu = &mmu2lsu_io;

  lsu->out.lsu2exe = &lsu2exe;
  lsu->out.lsu2dis = &lsu2dis;
  lsu->out.lsu2rob = &lsu2rob;
  lsu->out.peripheral_req = &out.peripheral_req;
  lsu->out.lsu2dcache = &out.lsu2dcache;
  lsu->out.lsu2mmu = &lsu2mmu_io;

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
  csr->comb_csr_status();
  out.sstatus = csr->out.csr_status->sstatus;
  out.mstatus = csr->out.csr_status->mstatus;
  out.satp = csr->out.csr_status->satp;
  out.privilege = csr->out.csr_status->privilege;
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

  pre->comb_begin();
  idu->comb_begin();
  rename->comb_begin();
  dis->comb_begin();
  isu->comb_begin();
  prf->comb_begin();
  exu->comb_begin();
  rob->comb_begin();
  csr->comb_begin();

  // 每个空行表示分层  下层会依赖上层产生的某个信号
  idu->comb_branch();
  pre->comb_accept_front();
  idu->comb_decode();
  csr->comb_interrupt();
  rename->comb_alloc();
  prf->comb_complete();
  prf->comb_awake();
  prf->comb_write();
  isu->comb_ready();
  lsu->comb_cal();
  lsu->comb_lsu2dis();
  lsu->comb_lsu2rob();

  rob->comb_ready();
  rob->comb_ftq_pc_req();
  pre->comb_ftq_lookup_rob();
  rob->comb_commit();
  pre->comb_commit_info();
  lsu->comb_stq_commit();

  dis->comb_alloc();
  lsu->comb_mmio_in();
  lsu->comb_dcache2lsu_ldq();
  lsu->comb_dcache2lsu_stq();
  lsu->comb_lsu2exe();
  exu->comb_to_csr();
  csr->comb_csr_read();

  exu->comb_exec();
  rob->comb_complete();

  exu->comb_ready();
  isu->comb_issue();
  lsu->comb_exe2lsu();
  lsu->comb_tlb_out();
  comb_lsu_mmu();
  lsu->comb_tlb_in_ldq();
  lsu->comb_tlb_in_stq();
  lsu->comb_stlf();

  prf->comb_req_ftq();
  pre->comb_ftq_lookup_prf();
  prf->comb_read();

  isu->comb_awake();
  isu->comb_calc_latency_next();
  csr->comb_exception();
  csr->comb_csr_write();
  dis->comb_wake();
  rename->comb_rename();

  dis->comb_dispatch();

  // 用于调试
  // 修正pc_next 以及difftest对应的pc_next
  out.flush = rob->out.rob_bcast->flush;
  out.fence_i = rob->out.rob_bcast->fence_i;
  out.itlb_flush = rob->out.rob_bcast->fence;

  // 1. Normal case (No Rob flush)
  if (!rob->out.rob_bcast->flush) {
    out.mispred = idu->out.dec_bcast->mispred;
    out.stall = !pre2front.ready;
    out.redirect_pc = idu->br_latch.redirect_pc;
  } else {
    out.mispred = true;
    if (rob->out.rob_bcast->mret || rob->out.rob_bcast->sret) {
      out.redirect_pc = csr->out.csr2front->epc;
    } else if (rob->out.rob_bcast->exception) {
      out.redirect_pc = csr->out.csr2front->trap_pc;
    } else {
      out.redirect_pc = rob->out.rob_bcast->pc + 4;
    }
    BE_LOG("flush redirect_pc=0x%08x", (uint32_t)out.redirect_pc);
  }

  for (int i = 0; i < COMMIT_WIDTH; i++) {
    out.commit_entry[i] =
        rob->out.rob_commit->commit_entry[i].uop.to_inst_entry(
            rob->out.rob_commit->commit_entry[i].valid);
    if (out.commit_entry[i].valid && out.flush) {
      // Flush: override extra_data.pc_next with redirect_pc for ALL
      // instructions so Difftest can read the correct next-PC (trap vector,
      // epc, etc.)
      out.commit_entry[i].uop.diag_val = out.redirect_pc;
      rob->out.rob_commit->commit_entry[i].uop.diag_val = out.redirect_pc;
    }
  }

  dis->comb_fire();
  lsu->comb_dis2lsu();
  rename->comb_fire();
  idu->comb_fire();
  // Kill flushed LSU work before driving new DCache/MMIO requests.
  lsu->comb_flush();
  lsu->comb_lsu2dcache_ldq();
  lsu->comb_lsu2dcache_stq();
  lsu->comb_mmio_out();
  lsu->comb_check();
  isu->comb_enq();
  // Rob recovery was split out of comb_fire(); keep the original priority:
  // branch rollback happens before enqueue, and global flush overrides both.
  rob->comb_branch();
  rob->comb_fire();
  rob->comb_flush();
  isu->comb_flush();

  pre->comb_fire();
  prf->comb_pipeline();
  exu->comb_pipeline();
  dis->comb_pipeline();
  rename->comb_pipeline();
  csr->comb_interrupt_inject();
}

void BackTop::seq() {
  // rename -> isu/stq/rob
  // exu -> prf
  pre->seq();
  rename->seq();
  dis->seq();
  idu->seq();
  isu->seq();
  exu->seq();
  prf->seq();
  rob->seq();
  csr->seq();
  if (dtlb_mmu != nullptr) {
    dtlb_mmu->seq();
  }
  lsu->seq();
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
constexpr uint32_t kCkptMagic = 0x006d6552u; // "Rem\0" little-endian
constexpr uint16_t kCkptFormatVersion = 2u;

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

constexpr std::array<CkptIoRange, 5> kExpectedIoLayout = {
    CkptIoRange{BOOT_IO_BASE, BOOT_IO_SIZE},
    CkptIoRange{UART_ADDR_BASE, UART_MMIO_SIZE},
    CkptIoRange{XPS_INTC_ADDR_BASE, XPS_INTC_MMIO_SIZE},
    CkptIoRange{TIMER_BASE, TIMER_MMIO_SIZE},
    CkptIoRange{OCSDC_ADDR_BASE, OCSDC_MMIO_SIZE},
};

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

#ifdef CONFIG_DIFFTEST
  init_difftest(size);
#endif

#ifndef CONFIG_BPU
  init_oracle(size);
#endif
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
  for (int i = 0; i < ARF_NUM; i++) {
    prf->reg_file[i] = state.gpr[i];
    prf->reg_file_1[i] = state.gpr[i];
  }

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

#ifndef CONFIG_BPU
  // FAST 模式从 ref 切换到 O3+oracle 时，oracle 也必须恢复到同一状态，
  // 否则首拍 refetch 会因为 PC 不一致触发断言。
  init_oracle_ckpt(state, privilege);
#endif
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
  if (ckpt_header.format_ver != kCkptFormatVersion) {
    std::cerr << "Error: Unsupported checkpoint version: "
              << ckpt_header.format_ver << std::endl;
    std::exit(1);
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

  for (int i = 0; i < ARF_NUM; i++) {
    prf->reg_file[i] = state.gpr[i];
    prf->reg_file_1[i] = state.gpr[i];
  }

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
  if (pmem_ram_ptr() == nullptr) {
    Assert(0 && "Error: Memory not allocated during checkpoint restore.");
  }
  pmem_clear_all();

  std::cout << "Restoring Memory... format=simpoint-v2(header+ranges)"
            << " file_ram=0x" << std::hex << ckpt_header.ram_size
            << " sim_ram=0x" << kCkptSimpointRamBytes << std::dec
            << std::endl;

  uint8_t *ram_dst = reinterpret_cast<uint8_t *>(pmem_ram_ptr());
  gz_read_exact(file, ram_dst, ckpt_header.ram_size);

  seed_uart_io_words();
  pmem_write(OPENSBI_TIMERCMP_LOW_ADDR, UINT32_MAX);
  pmem_write(OPENSBI_TIMERCMP_HIGH_ADDR, UINT32_MAX);
  gzclose(file);
  std::cout << "[CKPT][RAM-ONLY] Ignored " << ckpt_header.io_range_count
            << " serialized MMIO ranges; UART is transmit-ready and all "
               "interrupt sources are reset."
            << std::endl;
  std::cout << "Checkpoint restored from " << final_name << std::endl;

  init_diff_ckpt(state, restored_privilege);
#ifndef CONFIG_BPU
  init_oracle_ckpt(state, restored_privilege);
#endif

  // Ensure the pipeline starts with a refetch from the restored PC
  out.flush = true;
  out.redirect_pc = state.pc;
}

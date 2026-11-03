#include "BackTop.h"
#include "PhysMemory.h"
#include "RealLsu.h"
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

// Image / flash / SD loading and checkpoint restore.

void init_diff_ckpt(CPU_state ckpt_state, uint8_t privilege);

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

// Architectural state taken from the reference model / a checkpoint: PC, privilege, CSRs, the mirror the oracle compares
// against, and the LR/SC reservation.
void BackTop::install_arch_state(const CPU_state &state, uint8_t privilege) {
  number_PC = state.pc;
  csr->privilege = csr->privilege_1 = privilege;
  for (int i = 0; i < CSR_NUM; i++) csr->CSR_RegFile[i] = csr->CSR_RegFile_1[i] = state.csr[i];

  std::memcpy(dut_cpu.gpr, state.gpr, sizeof(dut_cpu.gpr));
  std::memcpy(dut_cpu.csr, state.csr, sizeof(dut_cpu.csr));
  dut_cpu.pc = state.pc;
  dut_cpu.store = state.store;
  dut_cpu.store_addr = state.store_addr;
  dut_cpu.store_data = state.store_data;
  dut_cpu.store_strb = state.store_strb;
  dut_cpu.reserve_valid = state.reserve_valid;
  dut_cpu.reserve_addr = state.reserve_addr;
  lsu->cur.lrsc_unit.reserve_valid = lsu->nxt.lrsc_unit.reserve_valid = state.reserve_valid;
  lsu->cur.lrsc_unit.reserve_addr = lsu->nxt.lrsc_unit.reserve_addr = state.reserve_addr;
}

void BackTop::restore_checkpoint_front_door(const std::string &filename, const std::string &stub_file) {
  std::string final_name = filename;
  gzFile file = gzopen(final_name.c_str(), "rb");
  if (!file && final_name.find(".gz") == std::string::npos) {
    final_name += ".gz";
    file = gzopen(final_name.c_str(), "rb");
  }
  if (!file) {
    Assert(0 && "Error: Could not open checkpoint file");
  }

  struct CkptCpuState {  // layout of CPU_state in the checkpoint; read and discarded
    uint32_t gpr[32];
    uint32_t csr[21];
    uint32_t pc;
    uint32_t store_addr;
    uint32_t store_data;
    uint32_t store_strb;
    bool store;
    bool reserve_valid;
    uint32_t reserve_addr;
  };

  CkptHeader ckpt_header = {};
  gz_read_pod(file, ckpt_header);
  if (ckpt_header.magic != kCkptMagic) {
    std::cerr << "Error: Invalid checkpoint magic." << std::endl;
    std::exit(1);
  }
  if (ckpt_header.format_ver != kCkptFormatVersion && ckpt_header.format_ver != kCkptLegacyFormatVersion) {
    std::cerr << "Error: Unsupported checkpoint version: " << ckpt_header.format_ver << std::endl;
    std::exit(1);
  }
  uint32_t checkpoint_sdram_size = 0;
  if (ckpt_header.format_ver >= kCkptFormatVersion) {
    gz_read_pod(file, checkpoint_sdram_size);
    if (checkpoint_sdram_size != PMEM_SDRAM_SIZE) {
      std::cerr << "Error: SDRAM checkpoint size is incompatible: file=0x" << std::hex << checkpoint_sdram_size
                << " sim=0x" << PMEM_SDRAM_SIZE << std::dec << std::endl;
      std::exit(1);
    }
  }
  if (ckpt_header.ram_size == 0 || ckpt_header.ram_size > kCkptSimpointRamBytes) {
    std::cerr << "Error: RAM-only checkpoint size is incompatible: file=0x" << std::hex << ckpt_header.ram_size
              << " sim=0x" << kCkptSimpointRamBytes << std::dec << std::endl;
    std::exit(1);
  }

  // The CPU state and the devices are not injected: they are set by the restore program. Read the state only to get to
  // the interval length (0 for a special checkpoint).
  CkptCpuState discarded_state;
  uint64_t interval_inst_count = 0;
  gz_read_pod(file, discarded_state);
  gz_read_pod(file, interval_inst_count);
  ckpt_interval_inst_count = interval_inst_count;

  if (pmem_ram_ptr() == nullptr || pmem_sdram_ptr() == nullptr) {
    Assert(0 && "Error: Memory backends not allocated during checkpoint restore.");
  }
  pmem_clear_all();
  std::cout << "Restoring Memory (front-door restore: the CPU state is set by the restore program)... format=simpoint-v"
            << ckpt_header.format_ver << std::endl;
  gz_read_exact(file, reinterpret_cast<uint8_t *>(pmem_ram_ptr()), ckpt_header.ram_size);
  if (checkpoint_sdram_size != 0) {
    gz_read_exact(file, reinterpret_cast<uint8_t *>(pmem_sdram_ptr()), checkpoint_sdram_size);
  }
  gzclose(file);

  // The restore program goes into the boot window (words at 0x0; the core fetches the reset code from there).
  std::ifstream stub(stub_file, std::ios::binary);
  if (!stub.is_open()) {
    Assert(0 && "Error: restore program (stub) does not exist");
  }
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(stub)), std::istreambuf_iterator<char>());
  if (bytes.empty() || bytes.size() > BOOT_IO_SIZE || (bytes.size() & 3u) != 0) {
    Assert(0 && "Error: restore program must be a non-empty word array that fits the boot window");
  }
  for (size_t off = 0; off < bytes.size(); off += 4) {
    const uint32_t word = static_cast<uint32_t>(bytes[off]) | (static_cast<uint32_t>(bytes[off + 1]) << 8) |
                          (static_cast<uint32_t>(bytes[off + 2]) << 16) | (static_cast<uint32_t>(bytes[off + 3]) << 24);
    pmem_write(static_cast<uint32_t>(off), word);
  }
  seed_uart_io_words();  // device defaults, as for a normal boot
  std::cout << "[CKPT] Restored DDR and SDRAM; restore program: " << (bytes.size() / 4) << " instructions from "
            << stub_file << std::endl;

  init_difftest(static_cast<int>(RAM_SIZE > 0x7fffffffu ? 0x7fffffff : RAM_SIZE));
}

void BackTop::restore_from_ref() {
  CPU_state state;
  uint8_t privilege;
  get_state(state, privilege);
  install_arch_state(state, privilege);

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

  const uint8_t restored_privilege = static_cast<uint8_t>(ckpt_header.privilege_mode & 0x3u);
  install_arch_state(state, restored_privilege);

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

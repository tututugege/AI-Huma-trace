// Loading a program image or a checkpoint into the model (the function model's RAM, its devices, the timing model's
// architectural registers). The checkpoint file itself is read by the function model (refcpu_restore_checkpoint).
#include "Load.h"
#include "SimCpu.h"
#include "IoBacking.h"
#include "RefCore.h"
#include "config.h"
#include "mmio_address_map.h"
#include "util.h"
#include <cstdio>
#include <cstdlib>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

namespace {
// Boot stub at address 0 (the CPU starts there): a0 = mhartid, a1 = the devicetree address, jump to `jump`.  On rv64 `lui`
// sign-extends (0x80000000 would become 0xffffffff80000000), so the constants are built with addi + slli instead; the
// function model's own stub does the same.
void write_boot_stub(uint32_t a1_value, uint32_t jump_value) {
  uint32_t at = 0;
  io_write(at, 0xf1402573);  // csrrs a0, mhartid, zero
  at += 4;
  auto li_hi = [&](uint32_t reg, uint32_t v) {
    uint32_t sh = 21;   // v = imm << sh with imm < 2048 (the platform addresses are multiples of 2 MiB)
    while (sh > 0 && (v & ((1u << sh) - 1))) sh--;
    const uint32_t imm = v >> sh;
    io_write(at, (imm << 20) | (reg << 7) | 0x13);   // addi reg, zero, imm
    at += 4;
    io_write(at, (sh << 20) | (reg << 15) | (1u << 12) | (reg << 7) | 0x13);   // slli reg, reg, sh
    at += 4;
  };
  li_hi(11, a1_value);  // a1
  li_hi(5, jump_value);  // t0
  io_write(at, 0x00028067);  // jalr zero, 0(t0)
}

void seed_uart_io_words() {
  constexpr uint32_t kUartLsrThre = 0x20u;
  constexpr uint32_t kUartLsrTemt = 0x40u;
  constexpr uint32_t kUartIirNoInt = 0x01u;
  constexpr uint32_t kUartIirFifoEnabled = 0xc0u;
  constexpr uint32_t kUartMsrCts = 0x10u;
  constexpr uint32_t kUartMsrDsr = 0x20u;
  constexpr uint32_t kUartMsrDcd = 0x80u;

  io_write(UART_ADDR_BASE + 0x00u,
             (kUartIirNoInt | kUartIirFifoEnabled) << 16);
  io_write(UART_ADDR_BASE + 0x04u,
             0x00000003u |
                 ((kUartLsrThre | kUartLsrTemt) << 8) |
                 ((kUartMsrCts | kUartMsrDsr | kUartMsrDcd) << 16));
  io_write(UART_ADDR_BASE + 0x08u, 0);
}
// An input error must stop the simulation in every build: Assert() is compiled out under NDEBUG (the normal build), so a missing
// image used to be loaded as an empty one and the simulation ran until its time limit.
[[noreturn]] void fatal(const std::string &what, const std::string &name) {
  std::fprintf(stderr, "Error: %s: %s\n", what.c_str(), name.c_str());
  std::exit(1);
}
} // namespace

void load_image(const std::string &filename) {
  std::ifstream inst_data(filename, std::ios::binary);
  if (!inst_data.is_open()) {
    fatal("image does not exist", filename);
  }

  inst_data.seekg(0, std::ios::end);
  std::streamsize size = inst_data.tellg();
  inst_data.seekg(0, std::ios::beg);

  if (size < 0 || static_cast<uint64_t>(size) > RAM_SIZE) {
    fatal("image too large for the configured RAM window", filename);
  }
  if (!inst_data.read(reinterpret_cast<char *>(refcore_ram_ptr()), size)) {
    fatal("failed to read the image", filename);
  }

  inst_data.close();

  write_boot_stub(0x83e00000u, 0x80000000u);
  seed_uart_io_words();

  refcore_seed_devices();
}

uint64_t restore_checkpoint(SimCpu &cpu, const std::string &filename) {
  refcore_clear_memory();
  CPU_state state{};
  uint8_t privilege = 0;
  const uint64_t interval = refcore_restore_ckpt(filename, state, privilege);
  cpu.install_arch_state(state, privilege);
  return interval;
}

uint64_t restore_checkpoint_front_door(const std::string &filename, const std::string &stub_file) {
  refcore_clear_memory();
  const uint64_t interval = refcore_restore_ckpt_memory(filename);
  // The restore program goes into the boot window (words at 0x0; the core fetches the reset code from there).
  std::ifstream stub(stub_file, std::ios::binary);
  if (!stub.is_open()) {
    fatal("restore program (stub) does not exist", stub_file);
  }
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(stub)), std::istreambuf_iterator<char>());
  if (bytes.empty() || bytes.size() > BOOT_IO_SIZE || (bytes.size() & 3u) != 0) {
    fatal("restore program must be a non-empty word array that fits the boot window", stub_file);
  }
  for (size_t off = 0; off < bytes.size(); off += 4) {
    const uint32_t word = static_cast<uint32_t>(bytes[off]) | (static_cast<uint32_t>(bytes[off + 1]) << 8) |
                          (static_cast<uint32_t>(bytes[off + 2]) << 16) | (static_cast<uint32_t>(bytes[off + 3]) << 24);
    io_write(static_cast<uint32_t>(off), word);
  }
  seed_uart_io_words();  // device defaults, as for a normal boot
  std::cout << "[CKPT] Restored DDR and SDRAM; restore program: " << (bytes.size() / 4) << " instructions from "
            << stub_file << std::endl;

  refcore_seed_devices();
  return interval;
}

void restore_from_ref(SimCpu &cpu) {
  CPU_state state;
  uint8_t privilege;
  refcore_get_state(state, privilege);
  cpu.install_arch_state(state, privilege);
}

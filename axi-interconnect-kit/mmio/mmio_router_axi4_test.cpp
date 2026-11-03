/**
 * @file mmio_router_axi4_test.cpp
 * @brief AXI4 MMIO router + MMIO bus unit tests.
 */

#include "AXI_Router_AXI4.h"
#include "MMIO_Bus_AXI4.h"
#include "SimDDR.h"
#include "axi_mmio_map.h"
#include <cstdio>
#include <vector>

uint32_t *p_memory = nullptr;
long long sim_time = 0;
constexpr uint32_t TEST_MEM_WORDS = 0x100000; // 4MB
constexpr int TIMEOUT = 1000;

class DummyDevice : public mmio::MMIO_Device {
public:
  explicit DummyDevice(uint32_t size) : mem(size, 0) {}

  void read(uint32_t addr, uint8_t *data, uint32_t len) override {
    for (uint32_t i = 0; i < len; i++) {
      data[i] = mem[(addr - base) + i];
    }
  }

  void write(uint32_t addr, const uint8_t *data, uint32_t len,
             uint32_t wstrb) override {
    for (uint32_t i = 0; i < len; i++) {
      if ((wstrb >> i) & 1u) {
        mem[(addr - base) + i] = data[i];
      }
    }
  }

  void set_base(uint32_t b) { base = b; }
  uint8_t &at(uint32_t addr) { return mem[(addr - base)]; }

private:
  uint32_t base = 0;
  std::vector<uint8_t> mem;
};

struct TestEnv {
  axi_interconnect::AXI_Router_AXI4 router;
  sim_ddr::SimDDR ddr;
  mmio::MMIO_Bus_AXI4 mmio;
  sim_ddr::SimDDR_IO_t up; // upstream AXI master
};

static void clear_upstream(TestEnv &env) {
  env.up.ar.arvalid = false;
  env.up.ar.araddr = 0;
  env.up.ar.arid = 0;
  env.up.ar.arlen = 0;
  env.up.ar.arsize = 2;
  env.up.ar.arburst = sim_ddr::AXI_BURST_INCR;

  env.up.aw.awvalid = false;
  env.up.aw.awaddr = 0;
  env.up.aw.awid = 0;
  env.up.aw.awlen = 0;
  env.up.aw.awsize = 2;
  env.up.aw.awburst = sim_ddr::AXI_BURST_INCR;

  env.up.w.wvalid = false;
  env.up.w.wdata = 0;
  env.up.w.wstrb = 0;
  env.up.w.wlast = false;

  env.up.r.rready = true;
  env.up.b.bready = true;
}

static void cycle_outputs(TestEnv &env) {
  env.ddr.comb_outputs();
  env.mmio.comb_outputs();
  env.router.comb_outputs(env.up, env.ddr.io, env.mmio.io);
}

static void cycle_seq(TestEnv &env) {
  env.ddr.comb_inputs();
  env.mmio.comb_inputs();
  env.router.seq(env.up, env.ddr.io, env.mmio.io);
  env.ddr.seq();
  env.mmio.seq();
  sim_time++;
}

static bool test_router_ram_read(TestEnv &env) {
  printf("=== Test 1: AXI4 router RAM read ===\n");

  uint32_t addr = 0x2000;
  uint32_t id = 1;
  p_memory[addr >> 2] = 0xAABBCCDDu;

  bool issued = false;
  bool done = false;
  bool mmio_saw_ar = false;
  int timeout = TIMEOUT;
  while (timeout-- > 0 && !done) {
    clear_upstream(env);
    cycle_outputs(env);

    env.up.ar.arvalid = true;
    env.up.ar.araddr = addr;
    env.up.ar.arid = id;
    env.up.ar.arlen = 0;
    env.up.ar.arsize = 2;
    env.up.ar.arburst = sim_ddr::AXI_BURST_INCR;

    env.router.comb_inputs(env.up, env.ddr.io, env.mmio.io);
    if (env.mmio.io.ar.arvalid) {
      mmio_saw_ar = true;
    }

    if (!issued && env.up.ar.arvalid && env.up.ar.arready) {
      issued = true;
    }
    if (env.up.r.rvalid && env.up.r.rready) {
      done = true;
      if (env.up.r.rdata != 0xAABBCCDDu) {
        printf("FAIL: RAM read mismatch exp=0x%08x got=0x%08x\n", 0xAABBCCDDu,
               env.up.r.rdata);
        return false;
      }
    }

    cycle_seq(env);
  }

  if (!issued || !done) {
    printf("FAIL: RAM read timeout\n");
    return false;
  }
  if (mmio_saw_ar) {
    printf("FAIL: MMIO saw AR during RAM read\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

static bool test_router_mmio_read(TestEnv &env, DummyDevice &dev) {
  printf("=== Test 2: AXI4 router MMIO read ===\n");

  uint32_t addr = MMIO_RANGE_BASE + 0x10;
  uint32_t id = 2;

  dev.at(addr) = 0x12;
  dev.at(addr + 1) = 0x34;
  dev.at(addr + 2) = 0x56;
  dev.at(addr + 3) = 0x78;

  bool issued = false;
  bool done = false;
  bool ddr_saw_ar = false;
  int timeout = TIMEOUT;
  while (timeout-- > 0 && !done) {
    clear_upstream(env);
    cycle_outputs(env);

    env.up.ar.arvalid = true;
    env.up.ar.araddr = addr;
    env.up.ar.arid = id;
    env.up.ar.arlen = 0;
    env.up.ar.arsize = 2;
    env.up.ar.arburst = sim_ddr::AXI_BURST_FIXED;

    env.router.comb_inputs(env.up, env.ddr.io, env.mmio.io);
    if (env.ddr.io.ar.arvalid) {
      ddr_saw_ar = true;
    }

    if (!issued && env.up.ar.arvalid && env.up.ar.arready) {
      issued = true;
    }
    if (env.up.r.rvalid && env.up.r.rready) {
      done = true;
      if (env.up.r.rdata != 0x78563412u) {
        printf("FAIL: MMIO read mismatch exp=0x%08x got=0x%08x\n", 0x78563412u,
               env.up.r.rdata);
        return false;
      }
    }

    cycle_seq(env);
  }

  if (!issued || !done) {
    printf("FAIL: MMIO read timeout\n");
    return false;
  }
  if (ddr_saw_ar) {
    printf("FAIL: DDR saw AR during MMIO read\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

static bool test_router_mmio_write(TestEnv &env, DummyDevice &dev) {
  printf("=== Test 3: AXI4 router MMIO write ===\n");

  uint32_t addr = MMIO_RANGE_BASE + 0x20;
  uint32_t id = 3;

  bool aw_issued = false;
  bool w_issued = false;
  bool done = false;
  bool ddr_saw_aw = false;
  int timeout = TIMEOUT;
  while (timeout-- > 0 && !done) {
    clear_upstream(env);
    cycle_outputs(env);

    env.up.aw.awvalid = !aw_issued;
    env.up.aw.awaddr = addr;
    env.up.aw.awid = id;
    env.up.aw.awlen = 0;
    env.up.aw.awsize = 2;
    env.up.aw.awburst = sim_ddr::AXI_BURST_FIXED;

    env.up.w.wvalid = !w_issued;
    env.up.w.wdata = 0xDEADBEEFu;
    env.up.w.wstrb = 0xF;
    env.up.w.wlast = true;

    env.router.comb_inputs(env.up, env.ddr.io, env.mmio.io);
    if (env.ddr.io.aw.awvalid) {
      ddr_saw_aw = true;
    }

    if (!aw_issued && env.up.aw.awvalid && env.up.aw.awready) {
      aw_issued = true;
    }
    if (!w_issued && env.up.w.wvalid && env.up.w.wready) {
      w_issued = true;
    }
    if (env.up.b.bvalid && env.up.b.bready) {
      done = true;
    }

    cycle_seq(env);
  }

  if (!aw_issued || !w_issued || !done) {
    printf("FAIL: MMIO write timeout\n");
    return false;
  }
  if (ddr_saw_aw) {
    printf("FAIL: DDR saw AW during MMIO write\n");
    return false;
  }
  if (dev.at(addr) != 0xEF || dev.at(addr + 1) != 0xBE ||
      dev.at(addr + 2) != 0xAD || dev.at(addr + 3) != 0xDE) {
    printf("FAIL: MMIO write data mismatch got=%02x %02x %02x %02x\n",
           dev.at(addr), dev.at(addr + 1), dev.at(addr + 2), dev.at(addr + 3));
    return false;
  }
  printf("PASS\n");
  return true;
}

int main() {
  std::vector<uint32_t> memory(TEST_MEM_WORDS, 0);
  p_memory = memory.data();

  TestEnv env;
  env.router.init();
  env.ddr.init();
  env.mmio.init();

  DummyDevice dev(0x1000);
  dev.set_base(MMIO_RANGE_BASE);
  env.mmio.add_device(MMIO_RANGE_BASE, 0x1000, &dev);

  bool pass = true;
  pass &= test_router_ram_read(env);

  env.router.init();
  env.ddr.init();
  env.mmio.init();
  env.mmio.add_device(MMIO_RANGE_BASE, 0x1000, &dev);
  pass &= test_router_mmio_read(env, dev);

  env.router.init();
  env.ddr.init();
  env.mmio.init();
  env.mmio.add_device(MMIO_RANGE_BASE, 0x1000, &dev);
  pass &= test_router_mmio_write(env, dev);

  if (pass) {
    printf("\nAll AXI4 MMIO router tests passed!\n");
    return 0;
  }
  printf("\nSome AXI4 MMIO router tests failed!\n");
  return 1;
}

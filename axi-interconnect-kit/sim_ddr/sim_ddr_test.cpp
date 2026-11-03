/**
 * @file sim_ddr_test.cpp
 * @brief Standalone test harness for SimDDR AXI4 module
 *
 * This test program verifies the correctness of the SimDDR AXI4 interface.
 * It can be compiled and run independently of the main simulator.
 *
 * Build: cmake .. && make sim_ddr_test
 * Run: ./sim_ddr_test
 */

#include "SimDDR.h"
#include <cassert>
#include <cstdio>
#include <cstring>

// ============================================================================
// Test Memory (standalone, not using main simulator's p_memory)
// ============================================================================
uint32_t *p_memory = nullptr;
long long sim_time = 0;
constexpr uint32_t TEST_MEM_SIZE = 1024 * 1024; // 1MB for testing
constexpr int HANDSHAKE_TIMEOUT = 20;
constexpr int DATA_TIMEOUT = sim_ddr::SIM_DDR_LATENCY * 8 + 200;

// ============================================================================
// Test Helper Functions
// ============================================================================

// Simulate one clock cycle
void sim_cycle(sim_ddr::SimDDR &ddr) {
  ddr.comb();
  ddr.seq();
  sim_time++;
}

void advance_seq(sim_ddr::SimDDR &ddr) {
  ddr.seq();
  sim_time++;
}

// Simulate multiple cycles
void sim_cycles(sim_ddr::SimDDR &ddr, int n) {
  for (int i = 0; i < n; i++) {
    sim_cycle(ddr);
  }
}

// Clear all master signals
void clear_master_signals(sim_ddr::SimDDR &ddr) {
  // AW channel (master -> slave)
  ddr.io.aw.awvalid = false;
  ddr.io.aw.awid = 0;
  ddr.io.aw.awaddr = 0;
  ddr.io.aw.awlen = 0;
  ddr.io.aw.awsize = 2; // 4 bytes (32-bit)
  ddr.io.aw.awburst = sim_ddr::AXI_BURST_INCR;

  // W channel (master -> slave)
  ddr.io.w.wvalid = false;
  ddr.io.w.wdata = 0;
  ddr.io.w.wstrb = 0xF; // All bytes enabled
  ddr.io.w.wlast = false;

  // B channel (slave -> master, master provides ready)
  ddr.io.b.bready = true;

  // AR channel (master -> slave)
  ddr.io.ar.arvalid = false;
  ddr.io.ar.arid = 0;
  ddr.io.ar.araddr = 0;
  ddr.io.ar.arlen = 0;
  ddr.io.ar.arsize = 2; // 4 bytes
  ddr.io.ar.arburst = sim_ddr::AXI_BURST_INCR;

  // R channel (slave -> master, master provides ready)
  ddr.io.r.rready = true;
}

void reset_testbench(sim_ddr::SimDDR &ddr) {
  ddr.init();
  sim_time = 0;
  std::memset(p_memory, 0, TEST_MEM_SIZE * sizeof(uint32_t));
  clear_master_signals(ddr);
}

bool wait_aw_ready(sim_ddr::SimDDR &ddr, int timeout = HANDSHAKE_TIMEOUT) {
  ddr.comb();
  while (!ddr.io.aw.awready && timeout > 0) {
    advance_seq(ddr);
    ddr.comb();
    timeout--;
  }
  return timeout > 0;
}

bool wait_w_ready(sim_ddr::SimDDR &ddr, int timeout = HANDSHAKE_TIMEOUT) {
  ddr.comb();
  while (!ddr.io.w.wready && timeout > 0) {
    advance_seq(ddr);
    ddr.comb();
    timeout--;
  }
  return timeout > 0;
}

bool wait_ar_ready(sim_ddr::SimDDR &ddr, int timeout = HANDSHAKE_TIMEOUT) {
  ddr.comb();
  while (!ddr.io.ar.arready && timeout > 0) {
    advance_seq(ddr);
    ddr.comb();
    timeout--;
  }
  return timeout > 0;
}

bool wait_b_valid(sim_ddr::SimDDR &ddr, int timeout = DATA_TIMEOUT) {
  ddr.comb();
  while (!ddr.io.b.bvalid && timeout > 0) {
    advance_seq(ddr);
    ddr.comb();
    timeout--;
  }
  return timeout > 0;
}

bool wait_r_valid(sim_ddr::SimDDR &ddr, int timeout = DATA_TIMEOUT) {
  ddr.comb();
  while (!ddr.io.r.rvalid && timeout > 0) {
    advance_seq(ddr);
    ddr.comb();
    timeout--;
  }
  return timeout > 0;
}

// ============================================================================
// Test Cases
// ============================================================================

// Test 1: Single word write
bool test_single_write(sim_ddr::SimDDR &ddr) {
  printf("=== Test 1: Single Word Write ===\n");

  uint32_t test_addr = 0x1000;
  uint32_t test_data = 0xDEADBEEF;

  clear_master_signals(ddr);

  // Phase 1: Issue AW (write address)
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awaddr = test_addr;
  ddr.io.aw.awlen = 0;  // Single beat (len + 1 = 1)
  ddr.io.aw.awsize = 2; // 4 bytes

  // Wait for awready
  int timeout = 10;
  while (!ddr.io.aw.awready && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  if (timeout == 0) {
    printf("FAIL: AW handshake timeout\n");
    return false;
  }
  sim_cycle(ddr); // Complete handshake
  ddr.io.aw.awvalid = false;

  // Phase 2: Issue W (write data)
  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = test_data;
  ddr.io.w.wstrb = 0xF;
  ddr.io.w.wlast = true; // Single beat, so last

  // Wait for wready
  timeout = 10;
  while (!ddr.io.w.wready && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  if (timeout == 0) {
    printf("FAIL: W handshake timeout\n");
    return false;
  }
  sim_cycle(ddr); // Complete handshake
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  // Phase 3: Wait for B (write response)
  ddr.io.b.bready = true;
  timeout = DATA_TIMEOUT;
  while (!ddr.io.b.bvalid && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  if (timeout == 0) {
    printf("FAIL: B response timeout\n");
    return false;
  }

  if (ddr.io.b.bresp != sim_ddr::AXI_RESP_OKAY) {
    printf("FAIL: Write response not OKAY: %d\n", ddr.io.b.bresp);
    return false;
  }
  sim_cycle(ddr); // Complete handshake

  // Verify memory content
  uint32_t word_addr = test_addr >> 2;
  if (p_memory[word_addr] != test_data) {
    printf("FAIL: Memory content mismatch: expected 0x%08x, got 0x%08x\n",
           test_data, p_memory[word_addr]);
    return false;
  }

  printf("PASS: Single write test completed. mem[0x%x] = 0x%08x\n", test_addr,
         p_memory[word_addr]);
  return true;
}

// Test 2: Single word read
bool test_single_read(sim_ddr::SimDDR &ddr) {
  printf("=== Test 2: Single Word Read ===\n");

  uint32_t test_addr = 0x2000;
  uint32_t expected_data = 0xCAFEBABE;

  // Pre-initialize memory
  p_memory[test_addr >> 2] = expected_data;

  clear_master_signals(ddr);

  // Phase 1: Issue AR (read address)
  ddr.io.ar.arvalid = true;
  ddr.io.ar.araddr = test_addr;
  ddr.io.ar.arlen = 0;  // Single beat
  ddr.io.ar.arsize = 2; // 4 bytes

  // Wait for arready
  int timeout = 10;
  while (!ddr.io.ar.arready && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  if (timeout == 0) {
    printf("FAIL: AR handshake timeout\n");
    return false;
  }
  sim_cycle(ddr); // Complete handshake
  ddr.io.ar.arvalid = false;

  // Phase 2: Wait for R (read data)
  ddr.io.r.rready = true;
  timeout = DATA_TIMEOUT;
  while (!ddr.io.r.rvalid && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  if (timeout == 0) {
    printf("FAIL: R data timeout\n");
    return false;
  }

  if (ddr.io.r.rdata != expected_data) {
    printf("FAIL: Read data mismatch: expected 0x%08x, got 0x%08x\n",
           expected_data, ddr.io.r.rdata);
    return false;
  }

  if (!ddr.io.r.rlast) {
    printf("FAIL: Expected rlast to be set for single beat\n");
    return false;
  }

  if (ddr.io.r.rresp != sim_ddr::AXI_RESP_OKAY) {
    printf("FAIL: Read response not OKAY: %d\n", ddr.io.r.rresp);
    return false;
  }

  sim_cycle(ddr); // Complete handshake

  printf("PASS: Single read test completed. read(0x%x) = 0x%08x\n", test_addr,
         expected_data);
  return true;
}

// Test 3: Burst write (4 beats)
bool test_burst_write(sim_ddr::SimDDR &ddr) {
  printf("=== Test 3: Burst Write (4 beats) ===\n");

  uint32_t base_addr = 0x3000;
  uint32_t test_data[4] = {0x11111111, 0x22222222, 0x33333333, 0x44444444};

  clear_master_signals(ddr);

  // Phase 1: Issue AW
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awaddr = base_addr;
  ddr.io.aw.awlen = 3;  // 4 beats (len + 1)
  ddr.io.aw.awsize = 2; // 4 bytes

  if (!wait_aw_ready(ddr)) {
    printf("FAIL: AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  // Phase 2: Send 4 W beats
  for (int i = 0; i < 4; i++) {
    ddr.io.w.wvalid = true;
    ddr.io.w.wdata = test_data[i];
    ddr.io.w.wstrb = 0xF;
    ddr.io.w.wlast = (i == 3);

    if (!wait_w_ready(ddr)) {
      printf("FAIL: W handshake timeout at beat %d\n", i);
      return false;
    }
    advance_seq(ddr);
  }
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  // Phase 3: Wait for B
  if (!wait_b_valid(ddr)) {
    printf("FAIL: B response timeout\n");
    return false;
  }
  sim_cycle(ddr);

  // Verify memory
  for (int i = 0; i < 4; i++) {
    uint32_t word_addr = (base_addr >> 2) + i;
    if (p_memory[word_addr] != test_data[i]) {
      printf("FAIL: Memory[%d] mismatch: expected 0x%08x, got 0x%08x\n", i,
             test_data[i], p_memory[word_addr]);
      return false;
    }
  }

  printf("PASS: Burst write test completed\n");
  return true;
}

// Test 4: Burst read (4 beats)
bool test_burst_read(sim_ddr::SimDDR &ddr) {
  printf("=== Test 4: Burst Read (4 beats) ===\n");

  uint32_t base_addr = 0x4000;
  uint32_t expected_data[4] = {0xAAAA0001, 0xAAAA0002, 0xAAAA0003, 0xAAAA0004};

  // Pre-initialize memory
  for (int i = 0; i < 4; i++) {
    p_memory[(base_addr >> 2) + i] = expected_data[i];
  }

  clear_master_signals(ddr);

  // Phase 1: Issue AR
  ddr.io.ar.arvalid = true;
  ddr.io.ar.araddr = base_addr;
  ddr.io.ar.arlen = 3; // 4 beats
  ddr.io.ar.arsize = 2;

  int timeout = 10;
  while (!ddr.io.ar.arready && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  sim_cycle(ddr);
  ddr.io.ar.arvalid = false;

  // Phase 2: Receive 4 R beats
  for (int i = 0; i < 4; i++) {
    timeout = DATA_TIMEOUT;
    while (!ddr.io.r.rvalid && timeout > 0) {
      sim_cycle(ddr);
      timeout--;
    }
    if (timeout == 0) {
      printf("FAIL: R data timeout at beat %d\n", i);
      return false;
    }

    if (ddr.io.r.rdata != expected_data[i]) {
      printf("FAIL: Read data[%d] mismatch: expected 0x%08x, got 0x%08x\n", i,
             expected_data[i], ddr.io.r.rdata);
      return false;
    }

    bool expected_last = (i == 3);
    if (ddr.io.r.rlast != expected_last) {
      printf("FAIL: rlast mismatch at beat %d: expected %d, got %d\n", i,
             expected_last, ddr.io.r.rlast);
      return false;
    }

    sim_cycle(ddr);
  }

  printf("PASS: Burst read test completed\n");
  return true;
}

// Test 5: Write with partial strobe
bool test_partial_strobe(sim_ddr::SimDDR &ddr) {
  printf("=== Test 5: Partial Write Strobe ===\n");

  uint32_t test_addr = 0x5000;

  // Initialize memory to known value
  p_memory[test_addr >> 2] = 0x12345678;

  clear_master_signals(ddr);

  // Write only upper 2 bytes (0xABCD -> position 2,3)
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awaddr = test_addr;
  ddr.io.aw.awlen = 0;

  if (!wait_aw_ready(ddr)) {
    printf("FAIL: AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = 0xABCD0000;
  ddr.io.w.wstrb = 0xC; // Only bytes 2,3 (upper half-word)
  ddr.io.w.wlast = true;

  if (!wait_w_ready(ddr)) {
    printf("FAIL: W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  // Wait for B
  if (!wait_b_valid(ddr)) {
    printf("FAIL: B response timeout\n");
    return false;
  }
  sim_cycle(ddr);

  // Expected: 0x12345678 & 0x0000FFFF | 0xABCD0000 = 0xABCD5678
  uint32_t expected = 0xABCD5678;
  if (p_memory[test_addr >> 2] != expected) {
    printf("FAIL: Partial strobe mismatch: expected 0x%08x, got 0x%08x\n",
           expected, p_memory[test_addr >> 2]);
    return false;
  }

  printf("PASS: Partial strobe test completed. mem[0x%x] = 0x%08x\n", test_addr,
         p_memory[test_addr >> 2]);
  return true;
}

// Test 6: Back-to-back transactions
bool test_back_to_back(sim_ddr::SimDDR &ddr) {
  printf("=== Test 6: Back-to-back Transactions ===\n");

  uint32_t addr1 = 0x6000;
  uint32_t addr2 = 0x6010;
  uint32_t data1 = 0x11110000;
  uint32_t data2 = 0x22220000;

  clear_master_signals(ddr);

  // Transaction 1: Write
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awaddr = addr1;
  ddr.io.aw.awlen = 0;

  if (!wait_aw_ready(ddr)) {
    printf("FAIL: Transaction 1 AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data1;
  ddr.io.w.wlast = true;

  if (!wait_w_ready(ddr)) {
    printf("FAIL: Transaction 1 W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  if (!wait_b_valid(ddr)) {
    printf("FAIL: Transaction 1 B response timeout\n");
    return false;
  }
  sim_cycle(ddr);

  // Transaction 2: Write (immediately after)
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awaddr = addr2;
  ddr.io.aw.awlen = 0;

  if (!wait_aw_ready(ddr)) {
    printf("FAIL: Transaction 2 AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data2;
  ddr.io.w.wlast = true;

  if (!wait_w_ready(ddr)) {
    printf("FAIL: Transaction 2 W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  if (!wait_b_valid(ddr)) {
    printf("FAIL: Transaction 2 B response timeout\n");
    return false;
  }
  sim_cycle(ddr);

  // Verify both writes
  if (p_memory[addr1 >> 2] != data1) {
    printf("FAIL: First write mismatch\n");
    return false;
  }
  if (p_memory[addr2 >> 2] != data2) {
    printf("FAIL: Second write mismatch\n");
    return false;
  }

  printf("PASS: Back-to-back test completed\n");
  return true;
}

// Test 7: ID signal verification
bool test_id_signals(sim_ddr::SimDDR &ddr) {
  printf("=== Test 7: ID Signal Verification ===\n");

  clear_master_signals(ddr);

  // Test write with specific ID
  uint8_t test_awid = 0x5;
  uint32_t test_addr = 0x7000;
  uint32_t test_data = 0xABCD1234;

  ddr.io.aw.awvalid = true;
  ddr.io.aw.awid = test_awid;
  ddr.io.aw.awaddr = test_addr;
  ddr.io.aw.awlen = 0;

  if (!wait_aw_ready(ddr)) {
    printf("FAIL: Write AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  // Send W data
  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = test_data;
  ddr.io.w.wlast = true;

  if (!wait_w_ready(ddr)) {
    printf("FAIL: Write W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  // Wait for B response and check bid
  if (!wait_b_valid(ddr)) {
    printf("FAIL: Write B response timeout\n");
    return false;
  }

  if (ddr.io.b.bid != test_awid) {
    printf("FAIL: Write bid mismatch: expected 0x%x, got 0x%x\n", test_awid,
           ddr.io.b.bid);
    return false;
  }
  sim_cycle(ddr);

  // Test read with specific ID
  uint8_t test_arid = 0xA;
  p_memory[test_addr >> 2] = test_data;

  ddr.io.ar.arvalid = true;
  ddr.io.ar.arid = test_arid;
  ddr.io.ar.araddr = test_addr;
  ddr.io.ar.arlen = 0;

  if (!wait_ar_ready(ddr)) {
    printf("FAIL: Read AR handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.ar.arvalid = false;

  // Wait for R data and check rid
  if (!wait_r_valid(ddr)) {
    printf("FAIL: Read R data timeout\n");
    return false;
  }

  if (ddr.io.r.rid != test_arid) {
    printf("FAIL: Read rid mismatch: expected 0x%x, got 0x%x\n", test_arid,
           ddr.io.r.rid);
    return false;
  }
  sim_cycle(ddr);

  printf("PASS: ID signal test completed. bid=0x%x, rid=0x%x\n", test_awid,
         test_arid);
  return true;
}

// Test 8: Stress test - multiple sequential transactions
bool test_stress_sequential(sim_ddr::SimDDR &ddr) {
  printf("=== Test 8: Stress Test (100 sequential transactions) ===\n");

  clear_master_signals(ddr);

  const int NUM_TRANSACTIONS = 100;
  uint32_t base_addr = 0x10000;
  // Phase 1: Write 100 values
  for (int i = 0; i < NUM_TRANSACTIONS; i++) {
    uint32_t addr = base_addr + (i * 4);
    uint32_t data = 0xDEAD0000 | i;
    uint8_t id = i & 0xF;

    // AW phase
    ddr.io.aw.awvalid = true;
    ddr.io.aw.awid = id;
    ddr.io.aw.awaddr = addr;
    ddr.io.aw.awlen = 0;

    if (!wait_aw_ready(ddr)) {
      printf("FAIL: Write %d AW handshake timeout\n", i);
      return false;
    }
    advance_seq(ddr);
    ddr.io.aw.awvalid = false;

    // W phase
    ddr.io.w.wvalid = true;
    ddr.io.w.wdata = data;
    ddr.io.w.wlast = true;

    if (!wait_w_ready(ddr)) {
      printf("FAIL: Write %d W handshake timeout\n", i);
      return false;
    }
    advance_seq(ddr);
    ddr.io.w.wvalid = false;
    ddr.io.w.wlast = false;

    // B phase
    if (!wait_b_valid(ddr)) {
      printf("FAIL: Write %d B response timeout\n", i);
      return false;
    }
    if (ddr.io.b.bid != id) {
      printf("FAIL: Write %d bid mismatch\n", i);
      return false;
    }
    sim_cycle(ddr);
  }

  // Phase 2: Read back and verify all 100 values
  for (int i = 0; i < NUM_TRANSACTIONS; i++) {
    uint32_t addr = base_addr + (i * 4);
    uint32_t expected_data = 0xDEAD0000 | i;
    uint8_t id = (i + 7) & 0xF; // Different ID pattern

    // AR phase
    ddr.io.ar.arvalid = true;
    ddr.io.ar.arid = id;
    ddr.io.ar.araddr = addr;
    ddr.io.ar.arlen = 0;

    if (!wait_ar_ready(ddr)) {
      printf("FAIL: Read %d AR handshake timeout\n", i);
      return false;
    }
    advance_seq(ddr);
    ddr.io.ar.arvalid = false;

    // R phase
    if (!wait_r_valid(ddr)) {
      printf("FAIL: Read %d R data timeout\n", i);
      return false;
    }
    if (ddr.io.r.rdata != expected_data) {
      printf("FAIL: Read %d data mismatch: expected 0x%08x, got 0x%08x\n", i,
             expected_data, ddr.io.r.rdata);
      return false;
    }
    if (ddr.io.r.rid != id) {
      printf("FAIL: Read %d rid mismatch: expected 0x%x, got 0x%x\n", i, id,
             ddr.io.r.rid);
      return false;
    }
    sim_cycle(ddr);
  }

  printf("PASS: Stress test completed (100 writes + 100 reads)\n");
  return true;
}

// Test 9: Outstanding read transactions (multiple in-flight reads)
bool test_outstanding_reads(sim_ddr::SimDDR &ddr) {
  printf("=== Test 9: Outstanding Read Transactions ===\n");

  clear_master_signals(ddr);

  constexpr int NUM_OUTSTANDING = sim_ddr::SIM_DDR_MAX_OUTSTANDING;
  uint32_t base_addr = 0x20000;
  uint32_t expected_data[NUM_OUTSTANDING];
  uint8_t expected_id[NUM_OUTSTANDING];

  // Pre-initialize memory with test data
  for (int i = 0; i < NUM_OUTSTANDING; i++) {
    expected_data[i] = 0xBEEF0000 | i;
    expected_id[i] = i + 1;
    p_memory[(base_addr >> 2) + i] = expected_data[i];
  }

  // Issue multiple AR requests back-to-back before waiting for R data
  printf("  Issuing %d outstanding read requests...\n", NUM_OUTSTANDING);
  for (int i = 0; i < NUM_OUTSTANDING; i++) {
    ddr.io.ar.arvalid = true;
    ddr.io.ar.arid = expected_id[i];
    ddr.io.ar.araddr = base_addr + (i * 4);
    ddr.io.ar.arlen = 0;
    ddr.io.ar.arsize = 2;

    ddr.comb();
    int timeout = 10;
    while (!ddr.io.ar.arready && timeout > 0) {
      ddr.seq();
      sim_time++;
      ddr.comb();
      timeout--;
    }
    if (timeout == 0) {
      printf("FAIL: AR handshake timeout for request %d\n", i);
      return false;
    }
    ddr.seq();
    sim_time++;
    ddr.io.ar.arvalid = false;

    // Small gap between requests, but not waiting for response
    sim_cycle(ddr);
  }

  printf("  All AR requests accepted. Waiting for R responses...\n");

  bool received[NUM_OUTSTANDING] = {false};
  int received_cnt = 0;
  int timeout = DATA_TIMEOUT * NUM_OUTSTANDING;
  while (received_cnt < NUM_OUTSTANDING && timeout-- > 0) {
    while (!ddr.io.r.rvalid && timeout-- > 0) {
      sim_cycle(ddr);
    }
    if (timeout <= 0) {
      printf("FAIL: R data timeout (received %d/%d)\n", received_cnt,
             NUM_OUTSTANDING);
      return false;
    }

    // Match by rid (responses may interleave)
    int idx = -1;
    for (int i = 0; i < NUM_OUTSTANDING; i++) {
      if (ddr.io.r.rid == expected_id[i]) {
        idx = i;
        break;
      }
    }
    if (idx < 0) {
      printf("FAIL: Unknown rid 0x%x\n", ddr.io.r.rid);
      return false;
    }
    if (received[idx]) {
      printf("FAIL: Duplicate response for rid=0x%x\n", ddr.io.r.rid);
      return false;
    }
    if (ddr.io.r.rdata != expected_data[idx]) {
      printf("FAIL: Response rid=0x%x data mismatch: expected 0x%08x, got 0x%08x\n",
             ddr.io.r.rid, expected_data[idx], ddr.io.r.rdata);
      return false;
    }
    if (!ddr.io.r.rlast) {
      printf("FAIL: Response rid=0x%x rlast should be true for single-beat\n",
             ddr.io.r.rid);
      return false;
    }

    received[idx] = true;
    received_cnt++;
    sim_cycle(ddr);
  }
  if (received_cnt != NUM_OUTSTANDING) {
    printf("FAIL: Outstanding reads incomplete (%d/%d)\n", received_cnt,
           NUM_OUTSTANDING);
    return false;
  }

  printf("PASS: Outstanding reads test completed (%d in-flight)\n",
         NUM_OUTSTANDING);
  return true;
}

// Test 10: Read interleaving verification (burst reads that interleave)
bool test_interleaving(sim_ddr::SimDDR &ddr) {
  printf("=== Test 10: Read Interleaving ===\n");

  clear_master_signals(ddr);

  // Issue 2 burst reads of 4 beats each
  const int NUM_READS = 2;
  const int BEATS_PER_READ = 4;
  uint32_t base_addrs[NUM_READS] = {0x30000, 0x30100};
  uint8_t ids[NUM_READS] = {0x1, 0x2};

  // Pre-initialize memory
  for (int r = 0; r < NUM_READS; r++) {
    for (int b = 0; b < BEATS_PER_READ; b++) {
      p_memory[(base_addrs[r] >> 2) + b] = (ids[r] << 24) | (b << 16) | 0xDA7A;
    }
  }

  // Issue both AR requests back-to-back
  printf("  Issuing %d burst read requests (len=%d each)...\n", NUM_READS,
         BEATS_PER_READ);

  for (int r = 0; r < NUM_READS; r++) {
    ddr.io.ar.arvalid = true;
    ddr.io.ar.arid = ids[r];
    ddr.io.ar.araddr = base_addrs[r];
    ddr.io.ar.arlen = BEATS_PER_READ - 1; // len = beats - 1
    ddr.io.ar.arsize = 2;

    ddr.comb();
    int timeout = 10;
    while (!ddr.io.ar.arready && timeout > 0) {
      ddr.seq();
      sim_time++;
      ddr.comb();
      timeout--;
    }
    ddr.seq();
    sim_time++;
    ddr.io.ar.arvalid = false;
    sim_cycle(ddr);
  }

  printf("  Waiting for R responses (total %d beats)...\n",
         NUM_READS * BEATS_PER_READ);

  // Receive all beats, tracking which IDs we see
  int beats_received[NUM_READS] = {0, 0};
  int total_beats = NUM_READS * BEATS_PER_READ;
  int interleave_switches = 0;
  uint8_t last_id = 0;

  for (int beat = 0; beat < total_beats; beat++) {
    int timeout = DATA_TIMEOUT;
    while (!ddr.io.r.rvalid && timeout > 0) {
      sim_cycle(ddr);
      timeout--;
    }
    if (timeout == 0) {
      printf("FAIL: R data timeout at beat %d\n", beat);
      return false;
    }

    // Identify which read this beat belongs to
    uint8_t rid = ddr.io.r.rid;
    int read_idx = -1;
    for (int r = 0; r < NUM_READS; r++) {
      if (ids[r] == rid) {
        read_idx = r;
        break;
      }
    }
    if (read_idx < 0) {
      printf("FAIL: Unknown rid 0x%x\n", rid);
      return false;
    }

    int beat_idx = beats_received[read_idx];
    uint32_t expected_data =
        p_memory[(base_addrs[read_idx] >> 2) + beat_idx];
    if (ddr.io.r.rdata != expected_data) {
      printf("FAIL: rdata mismatch rid=0x%x beat=%d exp=0x%08x got=0x%08x\n",
             rid, beat_idx, expected_data, ddr.io.r.rdata);
      return false;
    }

    // Track interleaving switches
    if (beat > 0 && rid != last_id) {
      interleave_switches++;
    }
    last_id = rid;

    beats_received[read_idx]++;

    // Check rlast
    bool expected_last = (beats_received[read_idx] == BEATS_PER_READ);
    if (ddr.io.r.rlast != expected_last) {
      printf("FAIL: rlast mismatch at beat %d\n", beat);
      return false;
    }

    sim_cycle(ddr);
  }

  // Verify all beats received
  for (int r = 0; r < NUM_READS; r++) {
    if (beats_received[r] != BEATS_PER_READ) {
      printf("FAIL: Read %d received %d beats, expected %d\n", r,
             beats_received[r], BEATS_PER_READ);
      return false;
    }
  }

  printf("  Interleave switches: %d\n", interleave_switches);
  printf("PASS: Interleaving test completed (switches=%d)\n",
         interleave_switches);
  return true;
}

// Test 11: R channel backpressure via rready
bool test_rready_backpressure(sim_ddr::SimDDR &ddr) {
  printf("=== Test 11: R channel backpressure (rready) ===\n");

  clear_master_signals(ddr);

  const int BEATS = 4;
  uint32_t base_addr = 0x40000;
  uint8_t id = 0x3;
  uint32_t expected[BEATS] = {0xAAAABBBB, 0xCCCCDDDD, 0x11112222, 0x33334444};
  for (int i = 0; i < BEATS; i++) {
    p_memory[(base_addr >> 2) + i] = expected[i];
  }

  // Issue AR burst
  ddr.io.ar.arvalid = true;
  ddr.io.ar.arid = id;
  ddr.io.ar.araddr = base_addr;
  ddr.io.ar.arlen = BEATS - 1;
  ddr.io.ar.arsize = 2;

  int timeout = HANDSHAKE_TIMEOUT;
  while (!ddr.io.ar.arready && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  if (timeout == 0) {
    printf("FAIL: AR handshake timeout\n");
    return false;
  }
  sim_cycle(ddr);
  ddr.io.ar.arvalid = false;

  // Stall on first beat
  ddr.io.r.rready = false;
  timeout = DATA_TIMEOUT;
  while (!ddr.io.r.rvalid && timeout > 0) {
    sim_cycle(ddr);
    timeout--;
  }
  if (timeout == 0) {
    printf("FAIL: R data timeout\n");
    return false;
  }

  if (ddr.io.r.rid != id || ddr.io.r.rdata != expected[0] || ddr.io.r.rlast) {
    printf("FAIL: Unexpected first beat rid=0x%x data=0x%08x rlast=%d\n",
           ddr.io.r.rid, ddr.io.r.rdata, ddr.io.r.rlast);
    return false;
  }

  uint32_t stalled_data = ddr.io.r.rdata;
  for (int i = 0; i < 5; i++) {
    sim_cycle(ddr);
    if (!ddr.io.r.rvalid) {
      printf("FAIL: rvalid dropped while rready=0\n");
      return false;
    }
    if (ddr.io.r.rid != id || ddr.io.r.rdata != stalled_data ||
        ddr.io.r.rlast) {
      printf("FAIL: R changed under stall rid=0x%x data=0x%08x rlast=%d\n",
             ddr.io.r.rid, ddr.io.r.rdata, ddr.io.r.rlast);
      return false;
    }
  }

  // Now accept all beats
  ddr.io.r.rready = true;
  for (int beat = 0; beat < BEATS; beat++) {
    timeout = DATA_TIMEOUT;
    while (!ddr.io.r.rvalid && timeout > 0) {
      sim_cycle(ddr);
      timeout--;
    }
    if (timeout == 0) {
      printf("FAIL: R beat %d timeout\n", beat);
      return false;
    }
    sim_cycle(ddr); // consume this beat and advance
    bool expected_last = (beat == BEATS - 1);
    if (ddr.io.r.rid != id || ddr.io.r.rdata != expected[beat] ||
        ddr.io.r.rlast != expected_last) {
      printf("FAIL: R beat %d mismatch rid=0x%x data=0x%08x rlast=%d\n", beat,
             ddr.io.r.rid, ddr.io.r.rdata, ddr.io.r.rlast);
      return false;
    }
  }

  printf("PASS: rready backpressure test completed\n");
  return true;
}

// Test 12: B channel backpressure via bready
bool test_bready_backpressure(sim_ddr::SimDDR &ddr) {
  printf("=== Test 12: B channel backpressure (bready) ===\n");

  clear_master_signals(ddr);

  uint8_t id = 0x9;
  uint32_t addr = 0x50000;
  uint32_t data = 0xDEADBEEF;
  p_memory[addr >> 2] = 0x12345678;

  // AW
  ddr.io.aw.awvalid = true;
  ddr.io.aw.awid = id;
  ddr.io.aw.awaddr = addr;
  ddr.io.aw.awlen = 0;
  ddr.io.aw.awsize = 2;
  ddr.io.aw.awburst = sim_ddr::AXI_BURST_INCR;

  if (!wait_aw_ready(ddr)) {
    printf("FAIL: AW handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.aw.awvalid = false;

  // W
  ddr.io.w.wvalid = true;
  ddr.io.w.wdata = data;
  ddr.io.w.wstrb = 0xF;
  ddr.io.w.wlast = true;

  if (!wait_w_ready(ddr)) {
    printf("FAIL: W handshake timeout\n");
    return false;
  }
  advance_seq(ddr);
  ddr.io.w.wvalid = false;
  ddr.io.w.wlast = false;

  // Stall B
  ddr.io.b.bready = false;
  if (!wait_b_valid(ddr)) {
    printf("FAIL: B response timeout\n");
    return false;
  }
  if (ddr.io.b.bid != id || ddr.io.b.bresp != sim_ddr::AXI_RESP_OKAY) {
    printf("FAIL: Unexpected B bid=0x%x bresp=%d\n", ddr.io.b.bid,
           ddr.io.b.bresp);
    return false;
  }

  for (int i = 0; i < 5; i++) {
    sim_cycle(ddr);
    if (!ddr.io.b.bvalid) {
      printf("FAIL: bvalid dropped while bready=0\n");
      return false;
    }
    if (ddr.io.b.bid != id) {
      printf("FAIL: bid changed under stall exp=0x%x got=0x%x\n", id,
             ddr.io.b.bid);
      return false;
    }
  }

  // Accept B
  ddr.io.b.bready = true;
  sim_cycle(ddr); // handshake + pop
  sim_cycle(ddr); // update outputs
  if (ddr.io.b.bvalid) {
    printf("FAIL: bvalid not cleared after handshake\n");
    return false;
  }

  if (p_memory[addr >> 2] != data) {
    printf("FAIL: Memory mismatch exp=0x%08x got=0x%08x\n", data,
           p_memory[addr >> 2]);
    return false;
  }

  printf("PASS: bready backpressure test completed\n");
  return true;
}

// Test 13: Max outstanding reads (arready deassert at limit)
bool test_max_outstanding_limit(sim_ddr::SimDDR &ddr) {
  printf("=== Test 13: Max outstanding read limit ===\n");

  clear_master_signals(ddr);
  ddr.io.r.rready = false; // prevent completion

  constexpr int N = sim_ddr::SIM_DDR_MAX_OUTSTANDING;
  uint32_t base_addr = 0x60000;
  for (int i = 0; i < N + 1; i++) {
    p_memory[(base_addr >> 2) + i] = 0xBEEF0000 | i;
  }

  // Fill up to the limit
  for (int i = 0; i < N; i++) {
    ddr.io.ar.arvalid = true;
    ddr.io.ar.arid = i;
    ddr.io.ar.araddr = base_addr + (i * 4);
    ddr.io.ar.arlen = 0;
    ddr.io.ar.arsize = 2;

    ddr.comb();
    int timeout = HANDSHAKE_TIMEOUT;
    while (!ddr.io.ar.arready && timeout > 0) {
      ddr.seq();
      sim_time++;
      ddr.comb();
      timeout--;
    }
    if (timeout == 0) {
      printf("FAIL: AR handshake timeout at idx %d\n", i);
      return false;
    }
    ddr.seq();
    sim_time++;
    ddr.io.ar.arvalid = false;
    sim_cycle(ddr);
  }

  // One more should be backpressured
  ddr.io.ar.arvalid = true;
  ddr.io.ar.arid = 0xFF;
  ddr.io.ar.araddr = base_addr + (N * 4);
  ddr.io.ar.arlen = 0;
  ddr.io.ar.arsize = 2;

  ddr.comb();
  if (ddr.io.ar.arready) {
    printf("FAIL: arready should be 0 when outstanding is full\n");
    return false;
  }
  ddr.io.ar.arvalid = false;

  // Drain all reads (enable rready and accept responses)
  ddr.io.r.rready = true;
  bool received[N] = {false};
  int received_cnt = 0;
  int timeout = DATA_TIMEOUT * N;
  while (received_cnt < N && timeout-- > 0) {
    while (!ddr.io.r.rvalid && timeout-- > 0) {
      sim_cycle(ddr);
    }
    if (timeout <= 0) {
      printf("FAIL: Drain timeout (received %d/%d)\n", received_cnt, N);
      return false;
    }

    uint8_t rid = ddr.io.r.rid;
    if (rid >= N) {
      printf("FAIL: unexpected rid=0x%x while draining\n", rid);
      return false;
    }
    if (received[rid]) {
      printf("FAIL: duplicate rid=0x%x while draining\n", rid);
      return false;
    }
    uint32_t exp = p_memory[(base_addr >> 2) + rid];
    if (ddr.io.r.rdata != exp || !ddr.io.r.rlast) {
      printf("FAIL: drain mismatch rid=0x%x data=0x%08x exp=0x%08x rlast=%d\n",
             rid, ddr.io.r.rdata, exp, ddr.io.r.rlast);
      return false;
    }

    received[rid] = true;
    received_cnt++;
    sim_cycle(ddr);
  }
  if (received_cnt != N) {
    printf("FAIL: drain incomplete (%d/%d)\n", received_cnt, N);
    return false;
  }

  // After draining, arready should be reasserted.
  ddr.io.ar.arvalid = true;
  ddr.io.ar.arid = 0x1;
  ddr.io.ar.araddr = base_addr;
  ddr.io.ar.arlen = 0;
  ddr.io.ar.arsize = 2;
  ddr.comb();
  if (!ddr.io.ar.arready) {
    printf("FAIL: arready should be 1 after draining\n");
    return false;
  }
  ddr.io.ar.arvalid = false;

  printf("PASS: max outstanding limit test completed\n");
  return true;
}

// ============================================================================
// Main Test Runner
// ============================================================================
int main() {
  printf("====================================\n");
  printf("SimDDR AXI4 Module Test Suite\n");
  printf("====================================\n\n");

  // Allocate test memory
  p_memory = new uint32_t[TEST_MEM_SIZE];
  memset(p_memory, 0, TEST_MEM_SIZE * sizeof(uint32_t));

  // Create and initialize SimDDR
  sim_ddr::SimDDR ddr;
  ddr.init();

  int passed = 0;
  int failed = 0;

  // Run tests
  reset_testbench(ddr);
  if (test_single_write(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5); // Gap between tests

  reset_testbench(ddr);
  if (test_single_read(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_burst_write(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_burst_read(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_partial_strobe(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_back_to_back(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_id_signals(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_stress_sequential(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_outstanding_reads(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_interleaving(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_rready_backpressure(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_bready_backpressure(ddr))
    passed++;
  else
    failed++;
  sim_cycles(ddr, 5);

  reset_testbench(ddr);
  if (test_max_outstanding_limit(ddr))
    passed++;
  else
    failed++;

  // Summary
  printf("\n====================================\n");
  printf("Test Results: %d passed, %d failed\n", passed, failed);
  printf("====================================\n");

  // Cleanup
  delete[] p_memory;

  return failed == 0 ? 0 : 1;
}

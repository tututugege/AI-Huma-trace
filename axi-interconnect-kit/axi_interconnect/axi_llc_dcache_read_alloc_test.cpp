#include "AXI_LLC.h"
#include <cstdio>

using namespace axi_interconnect;

long long sim_time = 0;

namespace {

AXI_LLCConfig make_config() {
  AXI_LLCConfig config;
  config.enable = true;
  config.size_bytes = 512;
  config.line_bytes = 64;
  config.ways = 2;
  config.mshr_num = 2;
  return config;
}

void clear_inputs(AXI_LLC &llc) {
  llc.io.ext_in = {};
  llc.io.lookup_in = {};
}

void cycle(AXI_LLC &llc) {
  llc.comb();
  llc.seq();
}

AXI_LLC_Bytes_t make_repl(uint32_t way) {
  AXI_LLC_Bytes_t repl;
  repl.resize(AXI_LLC_REPL_BYTES);
  repl.data()[0] = static_cast<uint8_t>(way & 0xFFu);
  repl.data()[1] = static_cast<uint8_t>((way >> 8) & 0xFFu);
  repl.data()[2] = static_cast<uint8_t>((way >> 16) & 0xFFu);
  repl.data()[3] = static_cast<uint8_t>((way >> 24) & 0xFFu);
  return repl;
}

WideReadData_t make_line_data(uint32_t base_word) {
  WideReadData_t data;
  data.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    data[i] = base_word + i;
  }
  return data;
}

bool test_dcache_read_miss_policy() {
  printf("=== LLC Test: dcache read miss policy ===\n");
  AXI_LLC llc;
  const auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x1A0 + 12;
  const uint32_t expected_word0 =
      AXI_LLC_EXPECT_DCACHE_READ_MISS_NOALLOC != 0
          ? 0x7000
          : (0x7000 + ((addr % config.line_bytes) / 4));

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].addr = addr;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].id = 0x44;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_DCACHE_R].ready) {
    printf("FAIL: dcache read miss was not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: lookup table read not issued for dcache miss\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.valid_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) *
                               config.line_bytes);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  llc.io.lookup_in.valid.resize(AXI_LLC::valid_row_bytes(config));
  llc.io.lookup_in.valid.data()[0] = 0;
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  if (!llc.io.regs.mshr[0].valid) {
    printf("FAIL: miss did not allocate an MSHR\n");
    return false;
  }
  if (llc.io.regs.mshr[0].bypass !=
      static_cast<bool>(AXI_LLC_EXPECT_DCACHE_READ_MISS_NOALLOC)) {
    printf("FAIL: unexpected bypass policy actual=%d expected=%d\n",
           static_cast<int>(llc.io.regs.mshr[0].bypass),
           AXI_LLC_EXPECT_DCACHE_READ_MISS_NOALLOC);
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x7000);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  const bool install_writes = llc.io.table_out.data.write &&
                              llc.io.table_out.meta.write &&
                              llc.io.table_out.repl.write;
  if (AXI_LLC_EXPECT_DCACHE_READ_MISS_NOALLOC != 0) {
    if (install_writes) {
      printf("FAIL: no-allocate dcache read miss incorrectly installed into LLC\n");
      return false;
    }
    llc.seq();
  } else {
    if (!install_writes) {
      printf("FAIL: allocate-on-miss dcache read did not install into LLC\n");
      return false;
    }
    llc.seq();
    clear_inputs(llc);
    cycle(llc);
  }

  clear_inputs(llc);
  llc.comb();
  const auto &resp = llc.io.ext_out.upstream.read_resp[MASTER_DCACHE_R];
  if (!resp.valid || resp.id != 0x44 || resp.data[0] != expected_word0) {
    printf("FAIL: miss response mismatch valid=%d id=%u d0=0x%x expected=0x%x\n",
           static_cast<int>(resp.valid), static_cast<unsigned>(resp.id),
           resp.data[0], expected_word0);
    return false;
  }

  printf("PASS\n");
  return true;
}

} // namespace

int main() {
  if (!test_dcache_read_miss_policy()) {
    return 1;
  }
  return 0;
}

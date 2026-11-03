#include "AXI_LLC.h"
#include <cstdio>
#include <cstring>

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

AXI_LLC_Bytes_t make_meta_set(const AXI_LLCConfig &config, uint32_t hit_way,
                              uint32_t tag) {
  AXI_LLC_Bytes_t meta;
  meta.resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
  for (uint32_t way = 0; way < config.ways; ++way) {
    AXI_LLCMetaEntry_t entry{};
    if (way == hit_way) {
      entry.tag = tag;
      entry.flags = AXI_LLC_META_VALID;
    }
    AXI_LLC_Bytes_t enc;
    AXI_LLC::encode_meta(entry, enc);
    std::memcpy(meta.data() + static_cast<size_t>(way) * AXI_LLC_META_ENTRY_BYTES,
                enc.data(), AXI_LLC_META_ENTRY_BYTES);
  }
  return meta;
}

AXI_LLC_Bytes_t make_meta_set_with_flags(const AXI_LLCConfig &config,
                                         uint32_t hit_way, uint32_t tag,
                                         uint8_t flags) {
  AXI_LLC_Bytes_t meta;
  meta.resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
  for (uint32_t way = 0; way < config.ways; ++way) {
    AXI_LLCMetaEntry_t entry{};
    if (way == hit_way) {
      entry.tag = tag;
      entry.flags = flags;
    }
    AXI_LLC_Bytes_t enc;
    AXI_LLC::encode_meta(entry, enc);
    std::memcpy(meta.data() + static_cast<size_t>(way) * AXI_LLC_META_ENTRY_BYTES,
                enc.data(), AXI_LLC_META_ENTRY_BYTES);
  }
  return meta;
}

void write_line_words(AXI_LLC_Bytes_t &data, const AXI_LLCConfig &config,
                      uint32_t way, uint32_t base_word) {
  const uint32_t words = AXI_LLC::line_words(config);
  for (uint32_t word = 0; word < words; ++word) {
    const uint32_t value = base_word + word;
    const size_t offset = static_cast<size_t>(way) * config.line_bytes +
                          static_cast<size_t>(word) * sizeof(uint32_t);
    data.data()[offset + 0] = static_cast<uint8_t>(value & 0xFFu);
    data.data()[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    data.data()[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    data.data()[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
  }
}

uint32_t read_line_word(const AXI_LLC_Bytes_t &line, uint32_t word_idx) {
  const size_t offset = static_cast<size_t>(word_idx) * sizeof(uint32_t);
  return static_cast<uint32_t>(line.data()[offset + 0]) |
         (static_cast<uint32_t>(line.data()[offset + 1]) << 8) |
         (static_cast<uint32_t>(line.data()[offset + 2]) << 16) |
         (static_cast<uint32_t>(line.data()[offset + 3]) << 24);
}

AXI_LLC_Bytes_t make_data_set(const AXI_LLCConfig &config, uint32_t hit_way,
                              uint32_t base_word) {
  AXI_LLC_Bytes_t data;
  data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  const uint32_t words = AXI_LLC::line_words(config);
  for (uint32_t word = 0; word < words; ++word) {
    const uint32_t value = base_word + word;
    const size_t offset = static_cast<size_t>(hit_way) * config.line_bytes +
                          static_cast<size_t>(word) * sizeof(uint32_t);
    data.data()[offset + 0] = static_cast<uint8_t>(value & 0xFFu);
    data.data()[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
    data.data()[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
    data.data()[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
  }
  return data;
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

AXI_LLC_Bytes_t make_valid_set_from_meta(const AXI_LLCConfig &config,
                                         const AXI_LLC_Bytes_t &meta) {
  AXI_LLC_Bytes_t valid;
  valid.resize(AXI_LLC::valid_row_bytes(config));
  for (uint32_t way = 0; way < config.ways; ++way) {
    const auto entry = AXI_LLC::decode_meta(meta, way);
    if ((entry.flags & AXI_LLC_META_VALID) == 0) {
      continue;
    }
    const size_t byte_idx = static_cast<size_t>(way);
    if (byte_idx < valid.size()) {
      valid.data()[byte_idx] = 1;
    }
  }
  return valid;
}

AXI_LLC_Bytes_t make_valid_bits(const AXI_LLCConfig &config, uint32_t bits) {
  AXI_LLC_Bytes_t valid;
  valid.resize(AXI_LLC::valid_row_bytes(config));
  for (uint32_t way = 0; way < config.ways; ++way) {
    if (((bits >> way) & 0x1u) == 0) {
      continue;
    }
    const size_t byte_idx = static_cast<size_t>(way);
    if (byte_idx < valid.size()) {
      valid.data()[byte_idx] = 1;
    }
  }
  return valid;
}

void derive_lookup_valid_from_meta(AXI_LLC &llc,
                                   const AXI_LLCConfig &config) {
  llc.io.lookup_in.valid_valid = true;
  llc.io.lookup_in.valid = make_valid_set_from_meta(config, llc.io.lookup_in.meta);
}

WideReadData_t make_line_data(uint32_t base_word) {
  WideReadData_t data;
  data.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    data[i] = base_word + i;
  }
  return data;
}

WideWriteData_t make_write_line_data(uint32_t base_word) {
  WideWriteData_t data;
  data.clear();
  for (uint32_t i = 0; i < 16; ++i) {
    data[i] = base_word + i;
  }
  return data;
}

WideWriteStrb_t make_full_line_strobe(const AXI_LLCConfig &config) {
  WideWriteStrb_t strobe;
  strobe.clear();
  for (uint32_t i = 0;
       i < config.line_bytes && i < MAX_WRITE_TRANSACTION_BYTES; ++i) {
    strobe.set(i, true);
  }
  return strobe;
}

void drive_lookup_miss(AXI_LLC &llc, const AXI_LLCConfig &config, uint32_t repl_way = 0) {
  clear_inputs(llc);
  llc.comb();
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(repl_way);
  cycle(llc);
}

bool test_hit_path() {
  printf("=== LLC Test 1: cacheable hit ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x120 + 8;
  const uint32_t tag = AXI_LLC::tag_of(config, addr);
  const uint32_t exp_word0 = 0x1000 + ((addr % config.line_bytes) / 4);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 3;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: hit req not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: lookup table read not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 1, 0x1000);
  llc.io.lookup_in.meta = make_meta_set(config, 1, tag);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.read_resp[MASTER_ICACHE];
  if (!resp.valid || resp.id != 3 || resp.data[0] != exp_word0 ||
      resp.data[1] != exp_word0 + 1) {
    printf("FAIL: hit response mismatch id=%u d0=0x%x d1=0x%x\n", resp.id,
           resp.data[0], resp.data[1]);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_miss_refill() {
  printf("=== LLC Test 2: miss refill ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x1A0 + 12;
  const uint32_t set = AXI_LLC::set_index(config, addr);
  const uint32_t exp_word0 = 0x3000 + ((addr % config.line_bytes) / 4);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 5;
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 0, 0x2000);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid ||
      llc.io.ext_out.mem.read_req_addr != AXI_LLC::line_addr(config, addr) ||
      llc.io.ext_out.mem.read_req_size != config.line_bytes - 1) {
    printf("FAIL: miss mem req mismatch valid=%d addr=0x%x size=%u\n",
           llc.io.ext_out.mem.read_req_valid, llc.io.ext_out.mem.read_req_addr,
           llc.io.ext_out.mem.read_req_size);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x3000);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      !llc.io.table_out.repl.write || llc.io.table_out.data.index != set) {
    printf("FAIL: refill table writes missing\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.read_resp[MASTER_ICACHE];
  if (!resp.valid || resp.id != 5 || resp.data[0] != exp_word0 ||
      resp.data[1] != exp_word0 + 1) {
    printf("FAIL: refill resp mismatch id=%u d0=0x%x d1=0x%x\n", resp.id,
           resp.data[0], resp.data[1]);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_bypass_read() {
  printf("=== LLC Test 3: bypass read ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].addr = 0x404;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].total_size = 7;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].id = 6;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].bypass = true;
  cycle(llc);

  drive_lookup_miss(llc, config);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.read_req_addr != 0x404 ||
      llc.io.ext_out.mem.read_req_size != 7) {
    printf("FAIL: bypass mem req mismatch\n");
    return false;
  }
  if (llc.io.table_out.data.enable || llc.io.table_out.meta.enable) {
    printf("FAIL: bypass unexpectedly touched table\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x4000);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.read_resp[MASTER_UNCORE_LSU_R];
  if (!resp.valid || resp.id != 6 || resp.data[0] != 0x4000) {
    printf("FAIL: bypass resp mismatch id=%u d0=0x%x\n", resp.id, resp.data[0]);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_write_passthrough() {
  printf("=== LLC Test 4: bypass write miss does not allocate ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = 0x800;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size = 3;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 1;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[0] = 0xDEADBEEF;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb = 0xFu;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].bypass = true;
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready ||
      llc.io.ext_out.mem.write_req_valid) {
    printf("FAIL: write passthrough req mismatch\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: bypass write lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  llc.comb();
  if (llc.io.table_out.data.write || llc.io.table_out.meta.write ||
      llc.io.table_out.repl.write) {
    printf("FAIL: bypass write miss should not allocate LLC line\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.write_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.write_req_valid ||
      llc.io.ext_out.mem.write_req_addr != 0x800 ||
      llc.io.ext_out.mem.write_req_data[0] != 0xDEADBEEF) {
    printf("FAIL: bypass write miss mem req mismatch valid=%d addr=0x%x data0=0x%x\n",
           static_cast<int>(llc.io.ext_out.mem.write_req_valid),
           llc.io.ext_out.mem.write_req_addr,
           llc.io.ext_out.mem.write_req_data[0]);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.write_resp_valid = true;
  llc.io.ext_in.mem.write_resp = 0;
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W];
  if (!resp.valid || resp.id != 1 || resp.resp != 0) {
    printf("FAIL: write passthrough resp mismatch\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_bypass_write_hit_updates_table() {
  printf("=== LLC Test 19: bypass write hit updates resident line ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t line_addr = 0x880;
  const uint32_t addr = line_addr + 4;
  const uint32_t tag = AXI_LLC::tag_of(config, addr);
  const uint32_t write_value = 0xABCD1234;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].addr = addr;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].total_size = 3;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].id = 15;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].bypass = true;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].wdata[0] = write_value;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].wstrb = 0xFu;
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_UNCORE_LSU_W].ready ||
      llc.io.ext_out.mem.write_req_valid) {
    printf("FAIL: bypass write-hit request not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: bypass write-hit lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 1, 0x5100);
  llc.io.lookup_in.meta = make_meta_set_with_flags(
      config, 1, tag,
      static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  llc.comb();
  if (!llc.io.table_out.data.write || llc.io.table_out.data.way != 1 ||
      !llc.io.table_out.meta.write || llc.io.table_out.meta.way != 1) {
    printf("FAIL: bypass write-hit table update missing way=%u meta_way=%u\n",
           llc.io.table_out.data.way, llc.io.table_out.meta.way);
    return false;
  }
  if (read_line_word(llc.io.table_out.data.payload, 0) != 0x5100 ||
      read_line_word(llc.io.table_out.data.payload, 1) != write_value ||
      read_line_word(llc.io.table_out.data.payload, 2) != 0x5102) {
    printf("FAIL: bypass write-hit data merge mismatch w0=0x%x w1=0x%x w2=0x%x\n",
           read_line_word(llc.io.table_out.data.payload, 0),
           read_line_word(llc.io.table_out.data.payload, 1),
           read_line_word(llc.io.table_out.data.payload, 2));
    return false;
  }
  const auto meta = AXI_LLC::decode_meta(llc.io.table_out.meta.payload, 0);
  if (meta.tag != tag ||
      meta.flags != static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY)) {
    printf("FAIL: bypass write-hit meta mismatch tag=0x%x flags=0x%x\n",
           meta.tag, meta.flags);
    return false;
  }
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.write_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.write_req_valid ||
      llc.io.ext_out.mem.write_req_addr != addr ||
      llc.io.ext_out.mem.write_req_data[0] != write_value) {
    printf("FAIL: bypass write-hit mem req mismatch valid=%d addr=0x%x data0=0x%x\n",
           static_cast<int>(llc.io.ext_out.mem.write_req_valid),
           llc.io.ext_out.mem.write_req_addr,
           llc.io.ext_out.mem.write_req_data[0]);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.write_resp_valid = true;
  llc.io.ext_in.mem.write_resp = 0;
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.write_resp[MASTER_UNCORE_LSU_W];
  if (!resp.valid || resp.id != 15 || resp.resp != 0) {
    printf("FAIL: bypass write-hit response mismatch id=%u resp=%u\n", resp.id,
           resp.resp);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_cacheable_write_updates_table() {
  printf("=== LLC Test 5: cacheable write updates LLC ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x840;
  const uint32_t tag = AXI_LLC::tag_of(config, addr);

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 2;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x7000 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready ||
      llc.io.ext_out.mem.write_req_valid) {
    printf("FAIL: cacheable write req not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: cacheable write lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.data.write ||
      !llc.io.table_out.meta.enable || !llc.io.table_out.meta.write) {
    printf("FAIL: cacheable write table update missing\n");
    return false;
  }
  if (llc.io.table_out.data.way != 0 || llc.io.table_out.meta.way != 0) {
    printf("FAIL: cacheable write replacement way mismatch data_way=%u meta_way=%u\n",
           llc.io.table_out.data.way, llc.io.table_out.meta.way);
    return false;
  }
  const auto meta = AXI_LLC::decode_meta(llc.io.table_out.meta.payload, 0);
  if (meta.tag != tag ||
      meta.flags != (AXI_LLC_META_VALID | AXI_LLC_META_DIRTY)) {
    printf("FAIL: cacheable write meta mismatch tag=0x%x flags=0x%x\n", meta.tag,
           meta.flags);
    return false;
  }
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W];
  if (!resp.valid || resp.id != 2 || resp.resp != 0) {
    printf("FAIL: cacheable write resp mismatch\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_cacheable_partial_write_miss_preserves_untouched_bytes() {
  printf("=== LLC Test 5b: partial cacheable write miss preserves untouched bytes ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t line = 0x8c0;
  const uint32_t addr = line + 8;
  const uint32_t write_value = 0xDEADBEEF;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size = 3;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x23;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[0] = write_value;
  for (uint32_t b = 0; b < 4; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: partial cacheable write miss not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: partial write miss lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  if (!llc.io.regs.mshr[0].valid || !llc.io.regs.mshr[0].is_write ||
      !llc.io.regs.write_ctx[MASTER_DCACHE_W].cache_pending) {
    printf("FAIL: partial write miss did not allocate write-backed MSHR valid=%d is_write=%d cache_pending=%d\n",
           static_cast<int>(llc.io.regs.mshr[0].valid),
           static_cast<int>(llc.io.regs.mshr[0].is_write),
           static_cast<int>(llc.io.regs.write_ctx[MASTER_DCACHE_W].cache_pending));
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.read_req_addr != line ||
      llc.io.ext_out.mem.read_req_size != config.line_bytes - 1) {
    printf("FAIL: partial write miss refill request mismatch valid=%d addr=0x%x size=%u\n",
           static_cast<int>(llc.io.ext_out.mem.read_req_valid),
           llc.io.ext_out.mem.read_req_addr, llc.io.ext_out.mem.read_req_size);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x1000);
  llc.comb();
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      !llc.io.table_out.repl.write) {
    printf("FAIL: partial write miss did not install merged line\n");
    return false;
  }
  const auto merged = llc.io.table_out.data.payload;
  if (read_line_word(merged, 0) != 0x1000 || read_line_word(merged, 1) != 0x1001 ||
      read_line_word(merged, 2) != write_value || read_line_word(merged, 3) != 0x1003) {
    printf("FAIL: partial write merge wrong w0=0x%x w1=0x%x w2=0x%x w3=0x%x\n",
           read_line_word(merged, 0), read_line_word(merged, 1),
           read_line_word(merged, 2), read_line_word(merged, 3));
    return false;
  }
  const auto meta = AXI_LLC::decode_meta(llc.io.table_out.meta.payload, 0);
  if (meta.flags != (AXI_LLC_META_VALID | AXI_LLC_META_DIRTY)) {
    printf("FAIL: partial write merge meta flags wrong flags=0x%x\n", meta.flags);
    return false;
  }
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W];
  if (!resp.valid || resp.id != 0x23 || resp.resp != 0) {
    printf("FAIL: partial write miss final response mismatch valid=%d id=%u resp=%u\n",
           static_cast<int>(resp.valid), resp.id, resp.resp);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_cacheable_write_dirty_victim_writeback() {
  printf("=== LLC Test 6: cacheable write dirty victim writeback ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_addr = 0x000;
  const uint32_t addr = 0x100;
  const uint32_t clean_addr = 0x080;
  const uint32_t victim_tag = AXI_LLC::tag_of(config, victim_addr);
  const uint32_t clean_tag = AXI_LLC::tag_of(config, clean_addr);
  const uint32_t tag = AXI_LLC::tag_of(config, addr);

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 3;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x8800 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready ||
      llc.io.ext_out.mem.write_req_valid) {
    printf("FAIL: dirty victim write req not accepted cleanly\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: dirty victim write lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  write_line_words(llc.io.lookup_in.data, config, 0, 0x3300);
  write_line_words(llc.io.lookup_in.data, config, 1, 0x4400);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  AXI_LLCMetaEntry_t clean_meta{};
  clean_meta.tag = clean_tag;
  clean_meta.flags = AXI_LLC_META_VALID;
  AXI_LLC_Bytes_t enc_clean;
  AXI_LLC::encode_meta(clean_meta, enc_clean);
  std::memcpy(llc.io.lookup_in.meta.data(), enc_clean.data(), AXI_LLC_META_ENTRY_BYTES);
  AXI_LLCMetaEntry_t dirty_meta{};
  dirty_meta.tag = victim_tag;
  dirty_meta.flags = AXI_LLC_META_VALID | AXI_LLC_META_DIRTY;
  AXI_LLC_Bytes_t enc_dirty;
  AXI_LLC::encode_meta(dirty_meta, enc_dirty);
  std::memcpy(llc.io.lookup_in.meta.data() + AXI_LLC_META_ENTRY_BYTES,
              enc_dirty.data(), AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.write_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.write_req_valid ||
      llc.io.ext_out.mem.write_req_addr != victim_addr ||
      llc.io.ext_out.mem.write_req_size != config.line_bytes - 1) {
    printf("FAIL: dirty victim wb req mismatch valid=%d addr=0x%x size=%u\n",
           llc.io.ext_out.mem.write_req_valid, llc.io.ext_out.mem.write_req_addr,
           llc.io.ext_out.mem.write_req_size);
    return false;
  }
  if (llc.io.table_out.data.enable || llc.io.table_out.meta.enable) {
    printf("FAIL: dirty victim prematurely updated tables\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.write_resp_valid = true;
  llc.io.ext_in.mem.write_resp = 0;
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.data.write ||
      !llc.io.table_out.meta.enable || !llc.io.table_out.meta.write ||
      llc.io.table_out.data.way != 1) {
    printf("FAIL: dirty victim post-wb table update missing way=%u\n",
           llc.io.table_out.data.way);
    return false;
  }
  const auto meta = AXI_LLC::decode_meta(llc.io.table_out.meta.payload, 0);
  if (meta.tag != tag ||
      meta.flags != (AXI_LLC_META_VALID | AXI_LLC_META_DIRTY)) {
    printf("FAIL: dirty victim meta mismatch tag=0x%x flags=0x%x\n", meta.tag,
           meta.flags);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W];
  if (!resp.valid || resp.id != 3 || resp.resp != 0) {
    printf("FAIL: dirty victim final resp mismatch\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_cacheable_partial_write_miss_dirty_victim_writes_back_correct_addr_data() {
  printf("=== LLC Test 6b: partial write miss writes back dirty victim correctly ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_addr = 0x000;
  const uint32_t clean_addr = 0x080;
  const uint32_t new_line = 0x100;
  const uint32_t new_addr = new_line + 8;
  const uint32_t victim_tag = AXI_LLC::tag_of(config, victim_addr);
  const uint32_t clean_tag = AXI_LLC::tag_of(config, clean_addr);
  const uint32_t write_value = 0xA5A5F00D;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = new_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size = 3;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x24;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[0] = write_value;
  for (uint32_t b = 0; b < 4; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: partial write dirty-victim lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  write_line_words(llc.io.lookup_in.data, config, 0, 0x3300);
  write_line_words(llc.io.lookup_in.data, config, 1, 0x4400);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  AXI_LLCMetaEntry_t clean_meta{};
  clean_meta.tag = clean_tag;
  clean_meta.flags = AXI_LLC_META_VALID;
  AXI_LLC_Bytes_t enc_clean;
  AXI_LLC::encode_meta(clean_meta, enc_clean);
  std::memcpy(llc.io.lookup_in.meta.data(), enc_clean.data(), AXI_LLC_META_ENTRY_BYTES);
  AXI_LLCMetaEntry_t dirty_meta{};
  dirty_meta.tag = victim_tag;
  dirty_meta.flags = static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY);
  AXI_LLC_Bytes_t enc_dirty;
  AXI_LLC::encode_meta(dirty_meta, enc_dirty);
  std::memcpy(llc.io.lookup_in.meta.data() + AXI_LLC_META_ENTRY_BYTES,
              enc_dirty.data(), AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  cycle(llc);

  if (!llc.io.regs.mshr[0].valid || !llc.io.regs.mshr[0].is_write ||
      !llc.io.regs.mshr[0].victim_dirty) {
    printf("FAIL: dirty-victim partial write did not create write MSHR victim_dirty=%d\n",
           static_cast<int>(llc.io.regs.mshr[0].victim_dirty));
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.read_req_addr != new_line) {
    printf("FAIL: partial write dirty-victim refill request mismatch valid=%d addr=0x%x\n",
           static_cast<int>(llc.io.ext_out.mem.read_req_valid),
           llc.io.ext_out.mem.read_req_addr);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x5500);
  llc.comb();
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      llc.io.table_out.data.way != 1) {
    printf("FAIL: partial write install missing after dirty victim snapshot\n");
    return false;
  }
  const auto merged = llc.io.table_out.data.payload;
  if (read_line_word(merged, 0) != 0x5500 || read_line_word(merged, 1) != 0x5501 ||
      read_line_word(merged, 2) != write_value || read_line_word(merged, 3) != 0x5503) {
    printf("FAIL: post-victim merged line wrong w0=0x%x w1=0x%x w2=0x%x w3=0x%x\n",
           read_line_word(merged, 0), read_line_word(merged, 1),
           read_line_word(merged, 2), read_line_word(merged, 3));
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.write_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.write_req_valid ||
      llc.io.ext_out.mem.write_req_addr != victim_addr ||
      llc.io.ext_out.mem.write_req_data[0] != 0x4400 ||
      llc.io.ext_out.mem.write_req_data[1] != 0x4401) {
    printf("FAIL: dirty victim writeback mismatch valid=%d addr=0x%x d0=0x%x d1=0x%x\n",
           static_cast<int>(llc.io.ext_out.mem.write_req_valid),
           llc.io.ext_out.mem.write_req_addr, llc.io.ext_out.mem.write_req_data[0],
           llc.io.ext_out.mem.write_req_data[1]);
    return false;
  }
  if (llc.io.table_out.data.write || llc.io.table_out.meta.write) {
    printf("FAIL: partial write duplicated install while issuing victim writeback\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.write_resp_valid = true;
  llc.io.ext_in.mem.write_resp = 0;
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W];
  if (!resp.valid || resp.id != 0x24 || resp.resp != 0) {
    printf("FAIL: partial write dirty-victim final response mismatch\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_same_master_multi_mshr_bypass() {
  printf("=== LLC Test 7: same-master multi-MSHR bypass ===\n");
  AXI_LLC llc;
  auto config = make_config();
  config.mshr_num = 2;
  llc.set_config(config);
  llc.reset();

  struct Req {
    uint32_t addr;
    uint8_t id;
    uint32_t base_word;
  };
  const Req reqs[2] = {
      {.addr = 0x500, .id = 7, .base_word = 0x5000},
      {.addr = 0x540, .id = 8, .base_word = 0x6000},
  };

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = reqs[0].addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 7;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = reqs[0].id;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].bypass = true;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: first bypass req not accepted\n");
    return false;
  }
  llc.seq();

  drive_lookup_miss(llc, config);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = reqs[1].addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 7;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = reqs[1].id;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].bypass = true;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: second bypass req not accepted while first outstanding\n");
    return false;
  }
  llc.seq();

  drive_lookup_miss(llc, config);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.read_req_id != 0 ||
      llc.io.ext_out.mem.read_req_addr != reqs[0].addr) {
    printf("FAIL: first mem read issue mismatch valid=%d id=%u addr=0x%x\n",
           llc.io.ext_out.mem.read_req_valid, llc.io.ext_out.mem.read_req_id,
           llc.io.ext_out.mem.read_req_addr);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.read_req_id != 1 ||
      llc.io.ext_out.mem.read_req_addr != reqs[1].addr) {
    printf("FAIL: second mem read issue mismatch valid=%d id=%u addr=0x%x\n",
           llc.io.ext_out.mem.read_req_valid, llc.io.ext_out.mem.read_req_id,
           llc.io.ext_out.mem.read_req_addr);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(reqs[0].base_word);
  cycle(llc);

  bool first_resp_latched = false;
  for (int i = 0; i < 3; ++i) {
    clear_inputs(llc);
    cycle(llc);
    if (llc.io.regs.read_resp_valid_r[MASTER_ICACHE]) {
      first_resp_latched = true;
      break;
    }
  }
  if (!first_resp_latched) {
    printf("FAIL: first bypass resp never latched\n");
    return false;
  }

  clear_inputs(llc);
  llc.comb();
  auto &resp0 = llc.io.ext_out.upstream.read_resp[MASTER_ICACHE];
  if (!resp0.valid || resp0.id != reqs[0].id || resp0.data[0] != reqs[0].base_word) {
    printf("FAIL: first bypass resp mismatch id=%u d0=0x%x\n", resp0.id,
           resp0.data[0]);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_resp[MASTER_ICACHE].ready = true;
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 1;
  llc.io.ext_in.mem.read_resp_data = make_line_data(reqs[1].base_word);
  cycle(llc);

  bool second_resp_latched = false;
  for (int i = 0; i < 3; ++i) {
    clear_inputs(llc);
    cycle(llc);
    if (llc.io.regs.read_resp_valid_r[MASTER_ICACHE]) {
      second_resp_latched = true;
      break;
    }
  }
  if (!second_resp_latched) {
    printf("FAIL: second bypass resp never latched\n");
    return false;
  }

  clear_inputs(llc);
  llc.comb();
  auto &resp1 = llc.io.ext_out.upstream.read_resp[MASTER_ICACHE];
  if (!resp1.valid || resp1.id != reqs[1].id || resp1.data[0] != reqs[1].base_word) {
    printf("FAIL: second bypass resp mismatch id=%u d0=0x%x\n", resp1.id,
           resp1.data[0]);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_read_resp_holds_until_ready() {
  printf("=== LLC Test 8: read response holds until ready ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = 0x900;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 7;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 9;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].bypass = true;
  cycle(llc);

  drive_lookup_miss(llc, config);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x9000);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  for (int i = 0; i < 3; ++i) {
    clear_inputs(llc);
    llc.comb();
    auto &resp = llc.io.ext_out.upstream.read_resp[MASTER_ICACHE];
    if (!resp.valid || resp.id != 9 || resp.data[0] != 0x9000) {
      printf("FAIL: response dropped before ready on hold cycle %d\n", i);
      return false;
    }
    llc.seq();
  }

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_resp[MASTER_ICACHE].ready = true;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].valid) {
    printf("FAIL: response not visible on ready handshake cycle\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].valid) {
    printf("FAIL: response not cleared after ready handshake\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_stream_prefetch_table_fill() {
  printf("=== LLC Test 9: stream prefetch table fill ===\n");
  AXI_LLC llc;
  auto config = make_config();
  config.prefetch_enable = true;
  config.mshr_num = 2;
  llc.set_config(config);
  llc.reset();

  const uint32_t addr_a = 0x200 + 16;
  const uint32_t line_b = AXI_LLC::line_addr(config, addr_a) + config.line_bytes;
  const uint32_t addr_b = line_b + 12;
  const uint32_t line_c = line_b + config.line_bytes;
  const uint32_t addr_c = line_c + 8;
  const uint32_t set_a = AXI_LLC::set_index(config, addr_a);
  const uint32_t set_b = AXI_LLC::set_index(config, addr_b);
  const uint32_t set_c = AXI_LLC::set_index(config, addr_c);
  const uint32_t word_a = 0x7000 + ((addr_a % config.line_bytes) / 4);
  const uint32_t word_b = 0x7100 + ((addr_b % config.line_bytes) / 4);
  const uint32_t tag_c = AXI_LLC::tag_of(config, addr_c);

  auto issue_miss = [&](uint32_t addr, uint8_t id) {
    clear_inputs(llc);
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr;
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = id;
    cycle(llc);

    clear_inputs(llc);
    cycle(llc);

    clear_inputs(llc);
    llc.io.lookup_in.data_valid = true;
    llc.io.lookup_in.meta_valid = true;
    llc.io.lookup_in.repl_valid = true;
    llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
    llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                                 AXI_LLC_META_ENTRY_BYTES);
    derive_lookup_valid_from_meta(llc, config);
    llc.io.lookup_in.repl = make_repl(0);
    cycle(llc);

    clear_inputs(llc);
    llc.io.ext_in.mem.read_req_ready = true;
    cycle(llc);
  };

  issue_miss(addr_a, 10);
  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x7000);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      llc.io.table_out.data.index != set_a) {
    printf("FAIL: demand A refill table write missing\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].valid ||
      llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].data[0] != word_a) {
    printf("FAIL: demand A response mismatch\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_resp[MASTER_ICACHE].ready = true;
  cycle(llc);

  issue_miss(addr_b, 11);
  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x7100);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      llc.io.table_out.data.index != set_b) {
    printf("FAIL: demand B refill table write missing\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].valid ||
      llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].data[0] != word_b) {
    printf("FAIL: demand B response mismatch\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_resp[MASTER_ICACHE].ready = true;
  cycle(llc);

  bool prefetch_lookup_seen = false;
  for (int i = 0; i < 8; ++i) {
    clear_inputs(llc);
    llc.comb();
    if (llc.io.table_out.data.enable && llc.io.table_out.meta.enable &&
        llc.io.table_out.repl.enable) {
      prefetch_lookup_seen = true;
      break;
    }
    llc.seq();
  }
  if (!prefetch_lookup_seen) {
    printf("FAIL: stream prefetch lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  const uint8_t prefetch_mem_id = llc.io.ext_out.mem.read_req_id;
  if (!llc.io.ext_out.mem.read_req_valid ||
      llc.io.ext_out.mem.read_req_addr != line_c) {
    printf("FAIL: stream prefetch mem req mismatch valid=%d addr=0x%x id=%u\n",
           llc.io.ext_out.mem.read_req_valid, llc.io.ext_out.mem.read_req_addr,
           llc.io.ext_out.mem.read_req_id);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = prefetch_mem_id;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x7200);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      llc.io.table_out.data.index != set_c) {
    printf("FAIL: prefetch refill table write missing data=%d meta=%d idx=%u state=%u\n",
           llc.io.table_out.data.write, llc.io.table_out.meta.write,
           llc.io.table_out.data.index, static_cast<unsigned>(llc.io.regs.state));
    return false;
  }
  const auto prefetch_meta = AXI_LLC::decode_meta(llc.io.table_out.meta.payload, 0);
  if ((prefetch_meta.flags & AXI_LLC_META_PREFETCH) == 0 || prefetch_meta.tag != tag_c) {
    printf("FAIL: prefetch meta flag/tag mismatch flags=0x%x tag=0x%x\n",
           prefetch_meta.flags, prefetch_meta.tag);
    return false;
  }
  const auto &perf = llc.perf_counters();
  if (perf.prefetch_issue != 1 || perf.read_access != 2 || perf.read_miss != 2) {
    printf("FAIL: prefetch perf mismatch issue=%llu hit=%llu acc=%llu rh=%llu rm=%llu\n",
           static_cast<unsigned long long>(perf.prefetch_issue),
           static_cast<unsigned long long>(perf.prefetch_hit),
           static_cast<unsigned long long>(perf.read_access),
           static_cast<unsigned long long>(perf.read_hit),
           static_cast<unsigned long long>(perf.read_miss));
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_prefetch_degree_two_queue() {
  printf("=== LLC Test 10: prefetch degree two queue ===\n");
  AXI_LLC llc;
  auto config = make_config();
  config.prefetch_enable = true;
  config.prefetch_degree = 2;
  config.mshr_num = 2;
  llc.set_config(config);
  llc.reset();

  const uint32_t addr_a = 0x200 + 16;
  const uint32_t line_b = AXI_LLC::line_addr(config, addr_a) + config.line_bytes;
  const uint32_t addr_b = line_b + 12;
  const uint32_t line_c = line_b + config.line_bytes;
  const uint32_t line_d = line_c + config.line_bytes;

  auto issue_demand_miss = [&](uint32_t addr, uint8_t id, uint32_t fill_base) {
    clear_inputs(llc);
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr;
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
    llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = id;
    cycle(llc);

    clear_inputs(llc);
    cycle(llc);

    clear_inputs(llc);
    llc.io.lookup_in.data_valid = true;
    llc.io.lookup_in.meta_valid = true;
    llc.io.lookup_in.repl_valid = true;
    llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
    llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                                 AXI_LLC_META_ENTRY_BYTES);
    derive_lookup_valid_from_meta(llc, config);
    llc.io.lookup_in.repl = make_repl(0);
    cycle(llc);

    clear_inputs(llc);
    llc.io.ext_in.mem.read_req_ready = true;
    llc.comb();
    const uint8_t mem_id = llc.io.ext_out.mem.read_req_id;
    llc.seq();

    clear_inputs(llc);
    llc.io.ext_in.mem.read_resp_valid = true;
    llc.io.ext_in.mem.read_resp_id = mem_id;
    llc.io.ext_in.mem.read_resp_data = make_line_data(fill_base);
    cycle(llc);

    clear_inputs(llc);
    llc.comb();
    llc.seq();

    clear_inputs(llc);
    llc.comb();
    llc.seq();

    clear_inputs(llc);
    llc.io.ext_in.upstream.read_resp[MASTER_ICACHE].ready = true;
    cycle(llc);

    clear_inputs(llc);
    llc.io.ext_in.upstream.read_resp[MASTER_ICACHE].ready = true;
    cycle(llc);
  };

  issue_demand_miss(addr_a, 1, 0x7000);
  issue_demand_miss(addr_b, 2, 0x7100);

  bool lookup_seen = false;
  for (int i = 0; i < 8; ++i) {
    clear_inputs(llc);
    llc.comb();
    if (llc.io.table_out.data.enable && llc.io.table_out.meta.enable &&
        llc.io.table_out.repl.enable) {
      lookup_seen = true;
      break;
    }
    llc.seq();
  }
  if (!lookup_seen) {
    printf("FAIL: first queued prefetch lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.read_req_addr != line_c) {
    printf("FAIL: first degree-two prefetch mem req mismatch addr=0x%x\n",
           llc.io.ext_out.mem.read_req_addr);
    return false;
  }
  const uint8_t pref_c_id = llc.io.ext_out.mem.read_req_id;
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = pref_c_id;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x7200);
  cycle(llc);

  bool queued_line_d = false;
  for (uint32_t i = 0; i < AXI_LLC_MAX_PREFETCH_QUEUE; ++i) {
    if (llc.io.regs.prefetch_q[i].valid && llc.io.regs.prefetch_q[i].line_addr == line_d) {
      queued_line_d = true;
      break;
    }
  }
  const auto &perf = llc.perf_counters();
  if (!queued_line_d || perf.prefetch_issue != 1 || perf.prefetch_drop_queue_full != 0) {
    printf("FAIL: degree-two queue mismatch queued_d=%d issue=%llu queue_drop=%llu\n",
           queued_line_d,
           static_cast<unsigned long long>(perf.prefetch_issue),
           static_cast<unsigned long long>(perf.prefetch_drop_queue_full));
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_demand_mem_issue_preempts_prefetch() {
  printf("=== LLC Test 11: demand mem issue preempts prefetch ===\n");
  AXI_LLC llc;
  auto config = make_config();
  config.prefetch_enable = true;
  config.mshr_num = 2;
  llc.set_config(config);
  llc.reset();

  clear_inputs(llc);
  llc.io.regs.mshr[0].valid = true;
  llc.io.regs.mshr[0].is_prefetch = true;
  llc.io.regs.mshr[0].line_addr = 0x400;
  llc.io.regs.mshr[1].valid = true;
  llc.io.regs.mshr[1].line_addr = 0x800;
  llc.comb();

  if (!llc.io.ext_out.mem.read_req_valid || llc.io.ext_out.mem.read_req_addr != 0x800 ||
      llc.io.ext_out.mem.read_req_id != 1) {
    printf("FAIL: demand mem issue was not prioritized addr=0x%x id=%u\n",
           llc.io.ext_out.mem.read_req_addr, llc.io.ext_out.mem.read_req_id);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_lookup_request_is_one_shot() {
  printf("=== LLC Test 12: lookup request is one-shot ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x180 + 4;
  const uint32_t tag = AXI_LLC::tag_of(config, addr);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 2;
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable || !llc.io.reg_write.lookup_issued_r) {
    printf("FAIL: initial lookup pulse missing\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (llc.io.table_out.data.enable || llc.io.table_out.meta.enable ||
      llc.io.table_out.repl.enable) {
    printf("FAIL: lookup request repeated while waiting for SRAM response\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 0, 0x2200);
  llc.io.lookup_in.meta = make_meta_set(config, 0, tag);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].valid ||
      llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].id != 2) {
    printf("FAIL: hit response missing after one-shot lookup\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_same_line_inflight_not_ready() {
  printf("=== LLC Test 13: same-line inflight blocks early ready ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x180;
  llc.io.regs.mshr[0].valid = true;
  llc.io.regs.mshr[0].line_addr = AXI_LLC::line_addr(config, addr);
  llc.io.regs.mshr[0].is_prefetch = true;

  if (llc.can_accept_read_now(MASTER_ICACHE, false, addr)) {
    printf("FAIL: early ready allowed while same line is inflight\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_line_invalidate_maintenance() {
  printf("=== LLC Test 14: line invalidate maintenance ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x240;
  const uint32_t tag = AXI_LLC::tag_of(config, addr);

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_line_valid = true;
  llc.io.ext_in.mem.invalidate_line_addr = addr;
  llc.comb();
  if (!llc.io.ext_out.mem.invalidate_line_accepted) {
    printf("FAIL: invalidate maintenance not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: invalidate lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 1, 0x4000);
  llc.io.lookup_in.meta = make_meta_set(config, 1, tag);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  llc.comb();
  if (llc.io.table_out.meta.enable || llc.io.table_out.data.enable ||
      llc.io.table_out.repl.enable) {
    printf("FAIL: invalidate should only update valid table meta=%d data=%d repl=%d\n",
           static_cast<int>(llc.io.table_out.meta.enable),
           static_cast<int>(llc.io.table_out.data.enable),
           static_cast<int>(llc.io.table_out.repl.enable));
    return false;
  }
  if (!llc.io.table_out.valid.enable || !llc.io.table_out.valid.write ||
      llc.io.table_out.valid.way != 1 || llc.io.table_out.valid.payload.size() == 0 ||
      (llc.io.table_out.valid.payload.data()[0] & 0x2u) != 0) {
    printf("FAIL: invalidate valid clear missing enable=%d write=%d way=%u bits=0x%x\n",
           static_cast<int>(llc.io.table_out.valid.enable),
           static_cast<int>(llc.io.table_out.valid.write), llc.io.table_out.valid.way,
           llc.io.table_out.valid.payload.size() == 0
               ? 0
               : llc.io.table_out.valid.payload.data()[0]);
    return false;
  }
  if (llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].valid) {
    printf("FAIL: invalidate maintenance incorrectly produced read response\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_line_invalidate_same_set_other_way_survives() {
  printf("=== LLC Test 15: line invalidate keeps other same-set way alive ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_addr = 0x240;
  const uint32_t survivor_addr = 0x340;
  const uint32_t victim_tag = AXI_LLC::tag_of(config, victim_addr);
  const uint32_t survivor_tag = AXI_LLC::tag_of(config, survivor_addr);
  const uint32_t exp_word0 = 0x5500 + ((survivor_addr % config.line_bytes) / 4);

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_line_valid = true;
  llc.io.ext_in.mem.invalidate_line_addr = victim_addr;
  llc.comb();
  if (!llc.io.ext_out.mem.invalidate_line_accepted) {
    printf("FAIL: targeted invalidate not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: targeted invalidate lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  write_line_words(llc.io.lookup_in.data, config, 0, 0x4400);
  write_line_words(llc.io.lookup_in.data, config, 1, 0x5500);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  AXI_LLCMetaEntry_t victim_meta{};
  victim_meta.tag = victim_tag;
  victim_meta.flags = static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY);
  AXI_LLC_Bytes_t enc_victim;
  AXI_LLC::encode_meta(victim_meta, enc_victim);
  std::memcpy(llc.io.lookup_in.meta.data(), enc_victim.data(),
              AXI_LLC_META_ENTRY_BYTES);
  AXI_LLCMetaEntry_t survivor_meta{};
  survivor_meta.tag = survivor_tag;
  survivor_meta.flags = AXI_LLC_META_VALID;
  AXI_LLC_Bytes_t enc_survivor;
  AXI_LLC::encode_meta(survivor_meta, enc_survivor);
  std::memcpy(llc.io.lookup_in.meta.data() + AXI_LLC_META_ENTRY_BYTES,
              enc_survivor.data(), AXI_LLC_META_ENTRY_BYTES);
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  llc.comb();
  if (llc.io.table_out.meta.enable || llc.io.table_out.data.enable ||
      llc.io.table_out.repl.enable) {
    printf("FAIL: targeted invalidate should only update valid table meta=%d data=%d repl=%d\n",
           static_cast<int>(llc.io.table_out.meta.enable),
           static_cast<int>(llc.io.table_out.data.enable),
           static_cast<int>(llc.io.table_out.repl.enable));
    return false;
  }
  if (!llc.io.table_out.valid.enable || !llc.io.table_out.valid.write ||
      llc.io.table_out.valid.way != 0 || llc.io.table_out.valid.payload.size() == 0 ||
      (llc.io.table_out.valid.payload.data()[0] & 0x1u) != 0) {
    printf("FAIL: targeted invalidate did not clear victim valid way=%u bits=0x%x\n",
           llc.io.table_out.valid.way,
           llc.io.table_out.valid.payload.size() == 0
               ? 0
               : llc.io.table_out.valid.payload.data()[0]);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = survivor_addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 12;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: survivor read not accepted after targeted invalidate\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: survivor lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
  write_line_words(llc.io.lookup_in.data, config, 0, 0x4400);
  write_line_words(llc.io.lookup_in.data, config, 1, 0x5500);
  llc.io.lookup_in.meta.resize(static_cast<size_t>(config.ways) *
                               AXI_LLC_META_ENTRY_BYTES);
  std::memcpy(llc.io.lookup_in.meta.data(), enc_victim.data(),
              AXI_LLC_META_ENTRY_BYTES);
  std::memcpy(llc.io.lookup_in.meta.data() + AXI_LLC_META_ENTRY_BYTES,
              enc_survivor.data(), AXI_LLC_META_ENTRY_BYTES);
  llc.io.lookup_in.valid_valid = true;
  llc.io.lookup_in.valid = make_valid_bits(config, 0x2u);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &resp = llc.io.ext_out.upstream.read_resp[MASTER_ICACHE];
  if (!resp.valid || resp.id != 12 || resp.data[0] != exp_word0 ||
      resp.data[1] != exp_word0 + 1) {
    printf("FAIL: survivor line was lost after targeted invalidate id=%u d0=0x%x d1=0x%x\n",
           resp.id, resp.data[0], resp.data[1]);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_line_invalidate_same_line_inflight_rejected() {
  printf("=== LLC Test 16: same-line invalidate blocked by inflight miss ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x240;
  llc.io.regs.mshr[0].valid = true;
  llc.io.regs.mshr[0].line_addr = AXI_LLC::line_addr(config, addr);
  llc.io.regs.mshr[0].master = MASTER_ICACHE;
  llc.io.regs.mshr[0].id = 3;

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_line_valid = true;
  llc.io.ext_in.mem.invalidate_line_addr = addr;
  llc.comb();
  if (llc.io.ext_out.mem.invalidate_line_accepted) {
    printf("FAIL: same-line invalidate should not be accepted while miss is inflight\n");
    return false;
  }
  if (llc.io.reg_write.lookup_valid_r) {
    printf("FAIL: same-line invalidate incorrectly armed lookup state\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_line_invalidate_same_line_active_write_ctx_rejected() {
  printf("=== LLC Test 16b: same-line invalidate blocked by active write ctx ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t line = 0x340;
  llc.io.regs.write_ctx[MASTER_DCACHE_W].valid = true;
  llc.io.regs.write_ctx[MASTER_DCACHE_W].bypass = false;
  llc.io.regs.write_ctx[MASTER_DCACHE_W].cache_done = false;
  llc.io.regs.write_ctx[MASTER_DCACHE_W].line_addr = line;
  llc.io.regs.write_ctx[MASTER_DCACHE_W].addr = line + 4;

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_line_valid = true;
  llc.io.ext_in.mem.invalidate_line_addr = line + 8;
  llc.comb();
  if (llc.io.ext_out.mem.invalidate_line_accepted) {
    printf("FAIL: invalidate should be rejected while active write ctx owns line\n");
    return false;
  }
  if (llc.io.reg_write.lookup_valid_r) {
    printf("FAIL: invalidate incorrectly armed lookup with active write ctx hazard\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_line_invalidate_same_line_queued_write_rejected() {
  printf("=== LLC Test 16c: same-line invalidate blocked by queued write ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t line = 0x3c0;
  llc.io.regs.write_q_head_r[MASTER_DCACHE_W] = 0;
  llc.io.regs.write_q_tail_r[MASTER_DCACHE_W] = 1;
  llc.io.regs.write_q_count_r[MASTER_DCACHE_W] = 1;
  auto &entry = llc.io.regs.write_q[MASTER_DCACHE_W][0];
  entry.valid = true;
  entry.bypass = false;
  entry.master = MASTER_DCACHE_W;
  entry.addr = line + 12;
  entry.total_size = 3;

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_line_valid = true;
  llc.io.ext_in.mem.invalidate_line_addr = line;
  llc.comb();
  if (llc.io.ext_out.mem.invalidate_line_accepted) {
    printf("FAIL: invalidate should be rejected while same-line queued write exists\n");
    return false;
  }
  if (llc.io.reg_write.lookup_is_invalidate_r) {
    printf("FAIL: queued write hazard should not promote maintenance lookup\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_line_invalidate_same_line_write_lookup_rejected() {
  printf("=== LLC Test 16d: same-line invalidate blocked by write lookup ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t line = 0x440;
  llc.io.regs.lookup_valid_r = true;
  llc.io.regs.lookup_addr_r = line + 4;
  llc.io.regs.lookup_is_write_r = true;
  llc.io.regs.lookup_master_r = MASTER_DCACHE_W;

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_line_valid = true;
  llc.io.ext_in.mem.invalidate_line_addr = line + 32;
  llc.comb();
  if (llc.io.ext_out.mem.invalidate_line_accepted) {
    printf("FAIL: invalidate should be rejected while same-line write lookup is pending\n");
    return false;
  }
  if (llc.io.reg_write.lookup_valid_r != llc.io.regs.lookup_valid_r) {
    printf("FAIL: invalidate unexpectedly modified existing lookup state\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_invalidate_all_drops_stale_refill_install() {
  printf("=== LLC Test 17: invalidate_all drops stale refill install ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x540;
  const uint32_t line_addr = AXI_LLC::line_addr(config, addr);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 0x41;
  cycle(llc);

  drive_lookup_miss(llc, config);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.read_req_valid ||
      llc.io.ext_out.mem.read_req_addr != line_addr) {
    printf("FAIL: stale-refill setup mem req mismatch valid=%d addr=0x%x\n",
           static_cast<int>(llc.io.ext_out.mem.read_req_valid),
           llc.io.ext_out.mem.read_req_addr);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_all = true;
  llc.comb();
  if (!llc.io.table_out.invalidate_all) {
    printf("FAIL: invalidate_all pulse was not emitted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0x9000);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (llc.io.table_out.data.write || llc.io.table_out.meta.write ||
      llc.io.table_out.repl.write) {
    printf("FAIL: stale refill incorrectly installed into LLC\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].valid ||
      llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].id != 0x41 ||
      llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].data[0] != 0x9000) {
    printf("FAIL: stale refill did not return correct demand response id=%u d0=0x%x\n",
           llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].id,
           llc.io.ext_out.upstream.read_resp[MASTER_ICACHE].data[0]);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_resp[MASTER_ICACHE].ready = true;
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].addr = addr;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].id = 0x42;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_DCACHE_R].ready) {
    printf("FAIL: post-invalidate demand re-read not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: post-invalidate re-read did not miss LLC\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_invalidate_all_busy_single_cycle_pulse_is_not_silent_accept() {
  printf("=== LLC Test 17c: invalidate_all busy single-cycle pulse is not silent accept ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t line = 0x600;
  const uint32_t addr = line + 8;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size = 3;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x51;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[0] = 0xCAFEBABE;
  for (uint32_t b = 0; b < 4; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 0, 0x1000);
  llc.io.lookup_in.meta = make_meta_set(config, 0, AXI_LLC::tag_of(config, addr));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_resp[MASTER_DCACHE_W].ready = true;
  cycle(llc);

  if (llc.io.regs.dirty_line_count_r == 0) {
    printf("FAIL: dirty resident line was not tracked\n");
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_all = true;
  llc.comb();
  if (llc.io.ext_out.mem.invalidate_all_accepted || llc.io.table_out.invalidate_all) {
    printf("FAIL: invalidate_all should stall while dirty resident line exists\n");
    return false;
  }
  if (llc.io.reg_write.dirty_line_count_r == 0) {
    printf("FAIL: invalidate_all stall path should preserve dirty tracking\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_invalidate_all_hold_until_accept_then_pulse() {
  printf("=== LLC Test 17d: invalidate_all hold-until-accept then pulse ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  llc.io.regs.mshr[0].valid = true;
  llc.io.regs.mshr[0].victim_dirty = true;
  llc.io.regs.mshr[0].victim_writeback_done = false;

  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_all = true;
  llc.comb();
  if (llc.io.ext_out.mem.invalidate_all_accepted || llc.io.table_out.invalidate_all) {
    printf("FAIL: invalidate_all should stall while dirty victim dependency exists\n");
    return false;
  }

  llc.io.regs.mshr[0] = {};
  clear_inputs(llc);
  llc.io.ext_in.mem.invalidate_all = true;
  llc.comb();
  if (!llc.io.ext_out.mem.invalidate_all_accepted || !llc.io.table_out.invalidate_all) {
    printf("FAIL: invalidate_all did not fire after dirty victim dependency cleared\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_same_master_cacheable_miss_serialized() {
  printf("=== LLC Test 17b: same-master cacheable miss is serialized ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr0 = 0x500 + 4;
  const uint32_t addr1 = 0x580 + 8;

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr0;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 0x31;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: first cacheable miss was not accepted\n");
    return false;
  }
  llc.seq();

  drive_lookup_miss(llc, config, 0);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr1;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 0x32;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].addr = addr1 + 0x80;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].id = 0x33;
  llc.comb();
  if (llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: second same-master cacheable miss should be blocked while first miss is inflight\n");
    return false;
  }
  if (!llc.io.ext_out.upstream.read_req[MASTER_DCACHE_R].ready) {
    printf("FAIL: different read master should still be able to issue cacheable miss\n");
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_cacheable_write_wins_lookup_slot() {
  printf("=== LLC Test 17: cacheable write wins shared lookup slot ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t write_addr = 0x840;
  const uint32_t read_addr = 0x180;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = write_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 4;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x9900 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = read_addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 7;
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: cacheable write not accepted\n");
    return false;
  }
  if (llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: read incorrectly accepted in same cycle as write lookup allocation\n");
    return false;
  }
  llc.seq();

  if (!llc.io.regs.write_ctx[MASTER_DCACHE_W].valid ||
      llc.io.regs.write_ctx[MASTER_DCACHE_W].bypass ||
      !llc.io.regs.lookup_valid_r || !llc.io.regs.lookup_is_write_r ||
      llc.io.regs.lookup_addr_r != write_addr) {
    printf(
        "FAIL: write lookup state corrupted active=%d bypass=%d lookup=%d is_write=%d addr=0x%x\n",
        static_cast<int>(llc.io.regs.write_ctx[MASTER_DCACHE_W].valid),
        static_cast<int>(llc.io.regs.write_ctx[MASTER_DCACHE_W].bypass),
        static_cast<int>(llc.io.regs.lookup_valid_r),
        static_cast<int>(llc.io.regs.lookup_is_write_r),
        llc.io.regs.lookup_addr_r);
    return false;
  }

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: write lookup request not issued after arbitration\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_refill_commit_and_new_lookup_do_not_clobber() {
  printf("=== LLC Test 18: refill commit and new lookup both survive ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t refill_addr = 0x180;
  const uint32_t refill_line = AXI_LLC::line_addr(config, refill_addr);
  const uint32_t lookup_addr = 0x440;

  auto &entry = llc.io.regs.mshr[0];
  entry = {};
  entry.valid = true;
  entry.bypass = false;
  entry.refill_valid = true;
  entry.refill_committed = false;
  entry.victim_writeback_done = true;
  entry.addr = refill_addr;
  entry.line_addr = refill_line;
  entry.set = AXI_LLC::set_index(config, refill_addr);
  entry.tag = AXI_LLC::tag_of(config, refill_addr);
  entry.way = 0;
  entry.total_size = 15;
  entry.master = MASTER_ICACHE;
  entry.id = 0x51;
  entry.refill_data = make_line_data(0xB000);
  llc.io.regs.state = AXI_LLCState::kRefill;

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].addr = lookup_addr;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_DCACHE_R].id = 0x52;
  llc.comb();
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      !llc.io.table_out.repl.write) {
    printf("FAIL: refill commit table writes missing\n");
    return false;
  }
  if (!llc.io.ext_out.upstream.read_req[MASTER_DCACHE_R].ready ||
      !llc.io.reg_write.lookup_valid_r ||
      llc.io.reg_write.lookup_addr_r != lookup_addr) {
    printf("FAIL: new lookup not preserved beside refill commit ready=%d lookup=%d addr=0x%x\n",
           static_cast<int>(llc.io.ext_out.upstream.read_req[MASTER_DCACHE_R].ready),
           static_cast<int>(llc.io.reg_write.lookup_valid_r),
           llc.io.reg_write.lookup_addr_r);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.reg_write.read_resp_valid_r[MASTER_ICACHE] ||
      llc.io.reg_write.read_resp_id_r[MASTER_ICACHE] != 0x51 ||
      llc.io.reg_write.read_resp_data_r[MASTER_ICACHE][0] != 0xB000) {
    printf("FAIL: refill response was not latched in reg_write id=%u d0=0x%x\n",
           llc.io.reg_write.read_resp_id_r[MASTER_ICACHE],
           llc.io.reg_write.read_resp_data_r[MASTER_ICACHE][0]);
    return false;
  }
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable ||
      llc.io.table_out.data.index != AXI_LLC::set_index(config, lookup_addr)) {
    printf("FAIL: queued lookup was not issued after refill commit\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_cacheable_write_hit_then_read_latest() {
  printf("=== LLC Test 19: cacheable write hit is immediately readable ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr = 0x840;
  const uint32_t tag = AXI_LLC::tag_of(config, addr);
  const uint32_t exp_word0 = 0x6600 + ((addr % config.line_bytes) / 4);

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 13;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x6600 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: write-hit request not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: write-hit lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 1, 0x2200);
  llc.io.lookup_in.meta = make_meta_set_with_flags(config, 1, tag,
                                                   static_cast<uint8_t>(AXI_LLC_META_VALID));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.data.write ||
      llc.io.table_out.data.way != 1 || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.meta.write || llc.io.table_out.meta.way != 1) {
    printf("FAIL: write-hit table update missing way=%u meta_way=%u\n",
           llc.io.table_out.data.way, llc.io.table_out.meta.way);
    return false;
  }
  const auto hit_meta = AXI_LLC::decode_meta(llc.io.table_out.meta.payload, 0);
  if (hit_meta.tag != tag ||
      hit_meta.flags != (AXI_LLC_META_VALID | AXI_LLC_META_DIRTY)) {
    printf("FAIL: write-hit meta mismatch tag=0x%x flags=0x%x\n", hit_meta.tag,
           hit_meta.flags);
    return false;
  }
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &wresp = llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W];
  if (!wresp.valid || wresp.id != 13 || wresp.resp != 0) {
    printf("FAIL: write-hit response mismatch id=%u resp=%u\n", wresp.id,
           wresp.resp);
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 14;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: post-write read request not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: post-write read lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 1, 0x6600);
  llc.io.lookup_in.meta = make_meta_set_with_flags(
      config, 1, tag, static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  auto &rresp = llc.io.ext_out.upstream.read_resp[MASTER_ICACHE];
  if (!rresp.valid || rresp.id != 14 || rresp.data[0] != exp_word0 ||
      rresp.data[1] != exp_word0 + 1) {
    printf("FAIL: post-write read saw stale data id=%u d0=0x%x d1=0x%x\n",
           rresp.id, rresp.data[0], rresp.data[1]);
    return false;
  }
  printf("PASS\n");
  return true;
}

bool test_write_hit_updates_pending_read_victim_snapshot() {
  printf("=== LLC Test 19b: write hit refreshes pending read victim snapshot ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_addr = 0x840;
  const uint32_t refill_addr = 0x940;
  const uint32_t victim_tag = AXI_LLC::tag_of(config, victim_addr);
  const uint32_t refill_tag = AXI_LLC::tag_of(config, refill_addr);
  const uint32_t victim_set = AXI_LLC::set_index(config, victim_addr);

  auto &pending = llc.io.regs.mshr[0];
  pending.valid = true;
  pending.bypass = false;
  pending.is_write = false;
  pending.refill_valid = false;
  pending.refill_committed = false;
  pending.addr = refill_addr;
  pending.line_addr = AXI_LLC::line_addr(config, refill_addr);
  pending.set = victim_set;
  pending.tag = refill_tag;
  pending.way = 0;
  pending.victim_dirty = true;
  pending.victim_writeback_done = false;
  pending.victim_addr = AXI_LLC::line_addr(config, victim_addr);
  pending.victim_data.clear();
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    pending.victim_data[i] = 0x1100 + i;
  }

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = victim_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x31;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x6600 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: pending-read victim refresh write-hit not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: pending-read victim refresh lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 0, 0x2200);
  llc.io.lookup_in.meta = make_meta_set_with_flags(
      config, 0, victim_tag,
      static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  llc.comb();

  const auto &updated = llc.io.reg_write.mshr[0];
  if (!updated.victim_dirty || updated.victim_writeback_done ||
      updated.victim_addr != AXI_LLC::line_addr(config, victim_addr)) {
    printf("FAIL: victim snapshot metadata not refreshed dirty=%d done=%d addr=0x%x\n",
           static_cast<int>(updated.victim_dirty),
           static_cast<int>(updated.victim_writeback_done), updated.victim_addr);
    return false;
  }
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    const uint32_t expected = 0x6600 + i;
    if (updated.victim_data[i] != expected) {
      printf("FAIL: victim snapshot stale at word %u got=0x%x exp=0x%x\n", i,
             updated.victim_data[i], expected);
      return false;
    }
  }

  printf("PASS\n");
  return true;
}

bool test_pending_upstream_write_blocks_same_line_read() {
  printf("=== LLC Test 20: pending upstream write blocks same-line read ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t line_addr = 0x840;
  const uint32_t same_line_read = line_addr + 8;
  const uint32_t other_line_read = 0x900;

  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].valid = true;
  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].bypass = true;
  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].addr = line_addr;
  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].line_addr = line_addr;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = line_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 9;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0xAA00 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = same_line_read;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 10;
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: queued write was not accepted while another write is active\n");
    return false;
  }
  if (llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: same-line read incorrectly accepted while queued write is pending\n");
    return false;
  }

  clear_inputs(llc);
  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].valid = true;
  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].bypass = true;
  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].addr = line_addr;
  llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].line_addr = line_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = line_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 9;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = other_line_read;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 11;
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: other-line case did not accept queued write\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_same_set_write_miss_then_read_miss_preserve_both_lines() {
  printf("=== LLC Test 20b: same-set write/read misses preserve separate ways ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim0_addr = 0x840;
  const uint32_t victim1_addr = 0x940;
  const uint32_t write_addr = 0xA40;
  const uint32_t read_addr = 0xB40;
  const uint32_t set = AXI_LLC::set_index(config, write_addr);
  const uint32_t victim0_tag = AXI_LLC::tag_of(config, victim0_addr);
  const uint32_t victim1_tag = AXI_LLC::tag_of(config, victim1_addr);

  auto make_resident_data = [&](uint32_t base0, uint32_t base1) {
    AXI_LLC_Bytes_t data;
    data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
    write_line_words(data, config, 0, base0);
    write_line_words(data, config, 1, base1);
    return data;
  };
  auto make_resident_meta = [&](uint32_t tag0, uint8_t flags0, uint32_t tag1,
                                uint8_t flags1) {
    AXI_LLC_Bytes_t meta;
    meta.resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
    AXI_LLCMetaEntry_t e0{};
    e0.tag = tag0;
    e0.flags = flags0;
    AXI_LLCMetaEntry_t e1{};
    e1.tag = tag1;
    e1.flags = flags1;
    AXI_LLC_Bytes_t enc0;
    AXI_LLC_Bytes_t enc1;
    AXI_LLC::encode_meta(e0, enc0);
    AXI_LLC::encode_meta(e1, enc1);
    std::memcpy(meta.data() + 0 * AXI_LLC_META_ENTRY_BYTES, enc0.data(),
                AXI_LLC_META_ENTRY_BYTES);
    std::memcpy(meta.data() + 1 * AXI_LLC_META_ENTRY_BYTES, enc1.data(),
                AXI_LLC_META_ENTRY_BYTES);
    return meta;
  };

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = write_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x61;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0xA000 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: write-miss lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_resident_data(0x1100, 0x2200);
  llc.io.lookup_in.meta = make_resident_meta(
      victim0_tag, static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY),
      victim1_tag, static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  if (!llc.io.regs.victim_wb_valid_r || !llc.io.regs.victim_wb_for_write_r ||
      !llc.io.regs.write_ctx[MASTER_DCACHE_W].cache_pending ||
      llc.io.regs.write_ctx[MASTER_DCACHE_W].way != 0 ||
      llc.io.regs.write_ctx[MASTER_DCACHE_W].set != set) {
    printf(
        "FAIL: write miss did not hold reserved victim way valid=%d for_write=%d "
        "cache_pending=%d way=%u set=%u\n",
        static_cast<int>(llc.io.regs.victim_wb_valid_r),
        static_cast<int>(llc.io.regs.victim_wb_for_write_r),
        static_cast<int>(llc.io.regs.write_ctx[MASTER_DCACHE_W].cache_pending),
        static_cast<unsigned>(llc.io.regs.write_ctx[MASTER_DCACHE_W].way),
        llc.io.regs.write_ctx[MASTER_DCACHE_W].set);
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.write_req_ready = true;
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = read_addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 0x62;
  llc.comb();
  if (!llc.io.ext_out.upstream.read_req[MASTER_ICACHE].ready) {
    printf("FAIL: same-set read miss not accepted while write miss pending\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: read-miss lookup not issued beside pending write victim\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_resident_data(0x1100, 0x2200);
  llc.io.lookup_in.meta = make_resident_meta(
      victim0_tag, static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY),
      victim1_tag, static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  int read_slot = -1;
  for (uint32_t slot = 0; slot < config.mshr_num; ++slot) {
    if (llc.io.regs.mshr[slot].valid) {
      read_slot = static_cast<int>(slot);
      break;
    }
  }
  if (read_slot < 0 || llc.io.regs.mshr[read_slot].way != 1 ||
      llc.io.regs.mshr[read_slot].set != set) {
    printf("FAIL: read miss did not avoid write-reserved way slot=%d way=%u set=%u\n",
           read_slot,
           read_slot < 0 ? 0u : static_cast<unsigned>(llc.io.regs.mshr[read_slot].way),
           read_slot < 0 ? 0u : llc.io.regs.mshr[read_slot].set);
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.write_resp_valid = true;
  cycle(llc);
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      llc.io.table_out.data.way != 0 || llc.io.table_out.data.index != set ||
      read_line_word(llc.io.table_out.data.payload, 0) != 0xA000) {
    printf("FAIL: write miss did not commit refill line into reserved way\n");
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  cycle(llc);
  if (!llc.io.regs.mshr[read_slot].mem_req_issued) {
    printf("FAIL: same-set read miss did not issue memory request\n");
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = static_cast<uint8_t>(read_slot);
  llc.io.ext_in.mem.read_resp_data = make_line_data(0xB000);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      llc.io.table_out.data.way != 1 || llc.io.table_out.data.index != set ||
      read_line_word(llc.io.table_out.data.payload, 0) != 0xB000) {
    printf("FAIL: read miss refill did not preserve second way after victim snapshot\n");
    return false;
  }
  if (!llc.io.regs.victim_wb_valid_r || llc.io.regs.victim_wb_for_write_r ||
      llc.io.regs.victim_wb_mshr_slot_r != static_cast<uint8_t>(read_slot) ||
      llc.io.regs.victim_wb_addr_r != AXI_LLC::line_addr(config, victim1_addr)) {
    printf(
        "FAIL: read miss did not schedule victim writeback slot=%u for_write=%d addr=0x%x\n",
        static_cast<unsigned>(llc.io.regs.victim_wb_mshr_slot_r),
        static_cast<int>(llc.io.regs.victim_wb_for_write_r),
        llc.io.regs.victim_wb_addr_r);
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.write_req_ready = true;
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.write_resp_valid = true;
  cycle(llc);

  printf("PASS\n");
  return true;
}

bool test_pending_read_victim_blocks_victim_line_read() {
  printf("=== LLC Test 20c: pending read victim blocks victim-line read ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim0_addr = 0x840;
  const uint32_t victim1_addr = 0x940;
  const uint32_t refill_addr = 0xA40;
  const uint32_t victim0_tag = AXI_LLC::tag_of(config, victim0_addr);
  const uint32_t victim1_tag = AXI_LLC::tag_of(config, victim1_addr);
  const uint32_t victim_line = AXI_LLC::line_addr(config, victim1_addr);

  auto make_resident_data = [&](uint32_t base0, uint32_t base1) {
    AXI_LLC_Bytes_t data;
    data.resize(static_cast<size_t>(config.ways) * config.line_bytes);
    write_line_words(data, config, 0, base0);
    write_line_words(data, config, 1, base1);
    return data;
  };
  auto make_resident_meta = [&](uint32_t tag0, uint8_t flags0, uint32_t tag1,
                                uint8_t flags1) {
    AXI_LLC_Bytes_t meta;
    meta.resize(static_cast<size_t>(config.ways) * AXI_LLC_META_ENTRY_BYTES);
    AXI_LLCMetaEntry_t e0{};
    e0.tag = tag0;
    e0.flags = flags0;
    AXI_LLCMetaEntry_t e1{};
    e1.tag = tag1;
    e1.flags = flags1;
    AXI_LLC_Bytes_t enc0;
    AXI_LLC_Bytes_t enc1;
    AXI_LLC::encode_meta(e0, enc0);
    AXI_LLC::encode_meta(e1, enc1);
    std::memcpy(meta.data() + 0 * AXI_LLC_META_ENTRY_BYTES, enc0.data(),
                AXI_LLC_META_ENTRY_BYTES);
    std::memcpy(meta.data() + 1 * AXI_LLC_META_ENTRY_BYTES, enc1.data(),
                AXI_LLC_META_ENTRY_BYTES);
    return meta;
  };

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].addr = refill_addr;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].total_size = 15;
  llc.io.ext_in.upstream.read_req[MASTER_ICACHE].id = 0x63;
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: refill lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_resident_data(0x1100, 0x2200);
  llc.io.lookup_in.meta = make_resident_meta(
      victim0_tag, static_cast<uint8_t>(AXI_LLC_META_VALID),
      victim1_tag, static_cast<uint8_t>(AXI_LLC_META_VALID | AXI_LLC_META_DIRTY));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(1);
  cycle(llc);

  if (!llc.io.regs.mshr[0].valid || !llc.io.regs.mshr[0].victim_dirty ||
      llc.io.regs.mshr[0].victim_addr != victim_line) {
    printf("FAIL: dirty victim read miss not allocated as expected\n");
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.read_req_ready = true;
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.mem.read_resp_valid = true;
  llc.io.ext_in.mem.read_resp_id = 0;
  llc.io.ext_in.mem.read_resp_data = make_line_data(0xB000);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  if (!llc.io.regs.victim_wb_valid_r || llc.io.regs.victim_wb_addr_r != victim_line ||
      llc.io.regs.victim_wb_for_write_r) {
    printf("FAIL: victim writeback not armed for pending read victim addr=0x%x for_write=%d\n",
           llc.io.regs.victim_wb_addr_r,
           static_cast<int>(llc.io.regs.victim_wb_for_write_r));
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].addr = victim1_addr + 8;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].total_size = 7;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].id = 0x64;
  llc.comb();
  if (llc.io.ext_out.upstream.read_req[MASTER_UNCORE_LSU_R].ready) {
    printf("FAIL: victim-line read incorrectly accepted while victim pending\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_pending_read_victim_blocks_victim_line_write() {
  printf("=== LLC Test 20d: pending read victim blocks victim-line write ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_line = AXI_LLC::line_addr(config, 0x940);

  llc.io.regs.mshr[0].valid = true;
  llc.io.regs.mshr[0].bypass = false;
  llc.io.regs.mshr[0].is_write = false;
  llc.io.regs.mshr[0].victim_dirty = true;
  llc.io.regs.mshr[0].victim_writeback_done = false;
  llc.io.regs.mshr[0].victim_addr = victim_line;
  llc.io.regs.victim_wb_valid_r = true;
  llc.io.regs.victim_wb_issued_r = false;
  llc.io.regs.victim_wb_for_write_r = false;
  llc.io.regs.victim_wb_mshr_slot_r = 0;
  llc.io.regs.victim_wb_addr_r = victim_line;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = victim_line + 8;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size = 7;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x65;
  llc.comb();
  if (llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: victim-line write incorrectly accepted while victim pending\n");
    return false;
  }

  llc.io.regs.write_q[MASTER_DCACHE_W][0] = {};
  llc.io.regs.write_q[MASTER_DCACHE_W][0].valid = true;
  llc.io.regs.write_q[MASTER_DCACHE_W][0].master = MASTER_DCACHE_W;
  llc.io.regs.write_q[MASTER_DCACHE_W][0].addr = victim_line + 12;
  llc.io.regs.write_q[MASTER_DCACHE_W][0].id = 0x66;
  llc.io.regs.write_q[MASTER_DCACHE_W][0].total_size = 3;
  llc.io.regs.write_q_head_r[MASTER_DCACHE_W] = 0;
  llc.io.regs.write_q_tail_r[MASTER_DCACHE_W] = 1;
  llc.io.regs.write_q_count_r[MASTER_DCACHE_W] = 1;

  clear_inputs(llc);
  llc.comb();
  if (llc.io.reg_write.write_ctx[MASTER_DCACHE_W].valid) {
    printf("FAIL: queued victim-line write incorrectly promoted to write_ctx\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_unarmed_refill_ready_victim_blocks_victim_line_access() {
  printf("=== LLC Test 20e: refill-ready read victim blocks victim-line access ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_line = AXI_LLC::line_addr(config, 0x940);

  llc.io.regs.mshr[0].valid = true;
  llc.io.regs.mshr[0].bypass = false;
  llc.io.regs.mshr[0].is_write = false;
  llc.io.regs.mshr[0].refill_valid = true;
  llc.io.regs.mshr[0].refill_committed = false;
  llc.io.regs.mshr[0].victim_dirty = true;
  llc.io.regs.mshr[0].victim_writeback_done = false;
  llc.io.regs.mshr[0].victim_addr = victim_line;
  llc.io.regs.victim_wb_valid_r = false;
  llc.io.regs.victim_wb_for_write_r = false;
  llc.io.regs.victim_wb_addr_r = 0;

  clear_inputs(llc);
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].valid = true;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].addr = victim_line + 8;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].total_size = 7;
  llc.io.ext_in.upstream.read_req[MASTER_UNCORE_LSU_R].id = 0x67;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = victim_line + 12;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size = 3;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x68;
  llc.comb();
  if (llc.io.ext_out.upstream.read_req[MASTER_UNCORE_LSU_R].ready) {
    printf("FAIL: victim-line read incorrectly accepted after refill before victim wb armed\n");
    return false;
  }
  if (llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: victim-line write incorrectly accepted after refill before victim wb armed\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_refill_ready_victim_waits_for_cache_resolved_victim_write() {
  printf("=== LLC Test 20f: refill-ready victim waits for cache-resolved victim-line write ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_line = AXI_LLC::line_addr(config, 0x940);
  const uint32_t fill_line = AXI_LLC::line_addr(config, 0x840);

  auto &pending = llc.io.regs.mshr[0];
  pending.valid = true;
  pending.bypass = false;
  pending.is_write = false;
  pending.refill_valid = true;
  pending.refill_committed = false;
  pending.addr = fill_line;
  pending.line_addr = fill_line;
  pending.set = AXI_LLC::set_index(config, fill_line);
  pending.way = 0;
  pending.victim_dirty = true;
  pending.victim_writeback_done = false;
  pending.victim_addr = victim_line;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    pending.victim_data[i] = 0x7700 + i;
  }

  auto &ctx = llc.io.regs.write_ctx[MASTER_DCACHE_W];
  ctx.valid = true;
  ctx.bypass = false;
  ctx.lookup_pending = false;
  ctx.cache_done = false;
  ctx.cache_pending = true;
  ctx.line_addr = victim_line;
  ctx.addr = victim_line;

  clear_inputs(llc);
  llc.comb();
  if (llc.io.reg_write.victim_wb_valid_r) {
    printf("FAIL: victim writeback armed while cache-resolved victim-line write still owns cache state\n");
    return false;
  }

  llc.io.regs.write_ctx[MASTER_DCACHE_W].cache_pending = false;
  clear_inputs(llc);
  llc.comb();
  if (!llc.io.reg_write.victim_wb_valid_r ||
      llc.io.reg_write.victim_wb_addr_r != victim_line) {
    printf("FAIL: victim writeback did not arm after cache-resolved victim write cleared\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_refill_ready_victim_commits_with_unissued_same_line_lookup() {
  printf("=== LLC Test 20g: refill-ready victim commits despite unissued same-line write lookup ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_line = AXI_LLC::line_addr(config, 0x940);
  const uint32_t fill_line = AXI_LLC::line_addr(config, 0x840);

  auto &pending = llc.io.regs.mshr[0];
  pending.valid = true;
  pending.bypass = false;
  pending.is_write = false;
  pending.refill_valid = true;
  pending.refill_committed = false;
  pending.addr = fill_line;
  pending.line_addr = fill_line;
  pending.set = AXI_LLC::set_index(config, fill_line);
  pending.tag = AXI_LLC::tag_of(config, fill_line);
  pending.way = 0;
  pending.victim_dirty = true;
  pending.victim_writeback_done = false;
  pending.victim_addr = victim_line;
  pending.refill_data = make_line_data(0x7b00);
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    pending.victim_data[i] = 0x7a00 + i;
  }

  auto &ctx = llc.io.regs.write_ctx[MASTER_DCACHE_W];
  ctx.valid = true;
  ctx.bypass = false;
  ctx.lookup_pending = false;
  ctx.cache_done = false;
  ctx.cache_pending = false;
  ctx.line_addr = victim_line;
  ctx.addr = victim_line;

  llc.io.regs.lookup_valid_r = true;
  llc.io.regs.lookup_issued_r = false;
  llc.io.regs.lookup_addr_r = victim_line;
  llc.io.regs.lookup_size_r = 3;
  llc.io.regs.lookup_master_r = MASTER_DCACHE_W;
  llc.io.regs.lookup_id_r = 0x69;
  llc.io.regs.lookup_is_write_r = true;
  llc.io.regs.lookup_is_bypass_r = false;

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.reg_write.victim_wb_valid_r ||
      llc.io.reg_write.victim_wb_addr_r != victim_line) {
    printf("FAIL: victim snapshot was not externalized while same-line write lookup was still unissued\n");
    return false;
  }
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      !llc.io.reg_write.mshr[0].refill_committed) {
    printf("FAIL: refill commit still stalled behind unissued same-line write lookup\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_refill_ready_victim_queues_snapshot_when_global_slot_busy() {
  printf("=== LLC Test 20h: refill-ready victim queues snapshot when global slot is busy ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t victim_line = AXI_LLC::line_addr(config, 0x940);
  const uint32_t fill_line = AXI_LLC::line_addr(config, 0x840);
  const uint32_t busy_victim_line = AXI_LLC::line_addr(config, 0xa40);

  auto &pending = llc.io.regs.mshr[0];
  pending.valid = true;
  pending.bypass = false;
  pending.is_write = false;
  pending.refill_valid = true;
  pending.refill_committed = false;
  pending.addr = fill_line;
  pending.line_addr = fill_line;
  pending.set = AXI_LLC::set_index(config, fill_line);
  pending.tag = AXI_LLC::tag_of(config, fill_line);
  pending.way = 0;
  pending.victim_dirty = true;
  pending.victim_writeback_done = false;
  pending.victim_addr = victim_line;
  pending.refill_data = make_line_data(0x7d00);
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    pending.victim_data[i] = 0x7c00 + i;
  }

  llc.io.regs.victim_wb_valid_r = true;
  llc.io.regs.victim_wb_issued_r = false;
  llc.io.regs.victim_wb_for_write_r = true;
  llc.io.regs.victim_wb_write_master_r = MASTER_DCACHE_W;
  llc.io.regs.victim_wb_addr_r = busy_victim_line;
  llc.io.regs.victim_wb_data_r = make_write_line_data(0x1234);
  llc.io.regs.victim_wb_strobe_r = make_full_line_strobe(config);

  clear_inputs(llc);
  llc.comb();
  if (llc.io.reg_write.read_victim_wb_q_count_r != 1) {
    printf("FAIL: victim snapshot was not queued while global victim slot was busy count=%u\n",
           static_cast<unsigned>(llc.io.reg_write.read_victim_wb_q_count_r));
    return false;
  }
  const auto &queued = llc.io.reg_write.read_victim_wb_q[0];
  if (!queued.valid || queued.victim_addr != victim_line ||
      queued.owner_slot != 0) {
    printf("FAIL: queued victim snapshot metadata mismatch valid=%d addr=0x%x owner=%u\n",
           static_cast<int>(queued.valid), queued.victim_addr,
           static_cast<unsigned>(queued.owner_slot));
    return false;
  }
  if (!llc.io.table_out.data.write || !llc.io.table_out.meta.write ||
      !llc.io.reg_write.mshr[0].refill_committed) {
    printf("FAIL: refill commit still stalled while global victim slot was busy\n");
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_same_master_multi_write_queue() {
  printf("=== LLC Test 21: same-master writes queue inside LLC ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr0 = 0x840;
  const uint32_t addr1 = 0x8c0;
  const uint32_t tag0 = AXI_LLC::tag_of(config, addr0);
  const uint32_t tag1 = AXI_LLC::tag_of(config, addr1);

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr0;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x21;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x5500 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: first LLC write not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr1;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x22;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x6600 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: second LLC write not accepted while first is active\n");
    return false;
  }
  llc.seq();
  if (llc.io.regs.write_q_count_r[MASTER_DCACHE_W] != 1 ||
      !llc.io.regs.write_q[MASTER_DCACHE_W]
                          [llc.io.regs.write_q_head_r[MASTER_DCACHE_W]]
                              .valid) {
    printf("FAIL: second LLC write not queued count=%u head_valid=%d\n",
           llc.io.regs.write_q_count_r[MASTER_DCACHE_W],
           static_cast<int>(llc.io.regs.write_q[MASTER_DCACHE_W]
                                               [llc.io.regs.write_q_head_r[MASTER_DCACHE_W]]
                                                   .valid));
    return false;
  }
  if (!llc.io.table_out.data.enable || !llc.io.table_out.meta.enable ||
      !llc.io.table_out.repl.enable) {
    printf("FAIL: first queued write lookup not issued\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 0, 0x1100);
  llc.io.lookup_in.meta = make_meta_set_with_flags(
      config, 0, tag0, static_cast<uint8_t>(AXI_LLC_META_VALID));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].valid ||
      llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].id != 0x21) {
    printf("FAIL: first queued write response missing id=%u valid=%d\n",
           llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].id,
           static_cast<int>(llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].valid));
    return false;
  }
  clear_inputs(llc);
  llc.io.ext_in.upstream.write_resp[MASTER_DCACHE_W].ready = true;
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  bool promoted = false;
  for (int i = 0; i < 8; ++i) {
    clear_inputs(llc);
    llc.comb();
    if (llc.io.reg_write.write_ctx[MASTER_DCACHE_W].valid &&
        llc.io.reg_write.lookup_valid_r &&
        llc.io.reg_write.lookup_addr_r == addr1) {
      promoted = true;
    }
    if (promoted) {
      break;
    }
    llc.seq();
  }
  if (!promoted) {
    printf("FAIL: second queued write was not promoted within retry window\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 1, 0x2200);
  llc.io.lookup_in.meta = make_meta_set_with_flags(
      config, 1, tag1, static_cast<uint8_t>(AXI_LLC_META_VALID));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].valid ||
      llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].id != 0x22) {
    printf("FAIL: second queued write response missing id=%u valid=%d\n",
           llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].id,
           static_cast<int>(llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].valid));
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_same_master_queue_waits_for_resp_slot() {
  printf("=== LLC Test 22: queued write waits for same-master resp slot ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t addr0 = 0x940;
  const uint32_t addr1 = 0x9c0;
  const uint32_t tag0 = AXI_LLC::tag_of(config, addr0);

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr0;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x31;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x7100 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  cycle(llc);

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = addr1;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x32;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x7200 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready ||
      !llc.io.table_out.data.enable) {
    printf("FAIL: setup did not accept second write and issue first lookup\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.lookup_in.data_valid = true;
  llc.io.lookup_in.meta_valid = true;
  llc.io.lookup_in.repl_valid = true;
  llc.io.lookup_in.data = make_data_set(config, 0, 0x1100);
  llc.io.lookup_in.meta = make_meta_set_with_flags(
      config, 0, tag0, static_cast<uint8_t>(AXI_LLC_META_VALID));
  derive_lookup_valid_from_meta(llc, config);
  llc.io.lookup_in.repl = make_repl(0);
  cycle(llc);

  clear_inputs(llc);
  cycle(llc);

  clear_inputs(llc);
  llc.comb();
  if (!llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].valid ||
      llc.io.ext_out.upstream.write_resp[MASTER_DCACHE_W].id != 0x31) {
    printf("FAIL: first write response not visible before resp-slot test\n");
    return false;
  }
  if (llc.io.reg_write.write_ctx[MASTER_DCACHE_W].valid ||
      llc.io.reg_write.lookup_valid_r ||
      llc.io.reg_write.write_q_count_r[MASTER_DCACHE_W] != 1) {
    printf("FAIL: queued write promoted before resp ready active=%d lookup=%d q=%u\n",
           static_cast<int>(llc.io.reg_write.write_ctx[MASTER_DCACHE_W].valid),
           static_cast<int>(llc.io.reg_write.lookup_valid_r),
           llc.io.reg_write.write_q_count_r[MASTER_DCACHE_W]);
    return false;
  }

  printf("PASS\n");
  return true;
}

bool test_cross_master_multi_active_write_contexts() {
  printf("=== LLC Test 23: cross-master write contexts can be active together ===\n");
  AXI_LLC llc;
  auto config = make_config();
  llc.set_config(config);
  llc.reset();

  const uint32_t cacheable_addr = 0xa40;
  const uint32_t bypass_addr = 0xac8;

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].addr = cacheable_addr;
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].total_size =
      static_cast<uint8_t>(config.line_bytes - 1);
  llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].id = 0x41;
  for (uint32_t i = 0; i < config.line_bytes / sizeof(uint32_t); ++i) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wdata[i] = 0x8100 + i;
  }
  for (uint32_t b = 0; b < config.line_bytes; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_DCACHE_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_DCACHE_W].ready) {
    printf("FAIL: cacheable write not accepted\n");
    return false;
  }
  llc.seq();

  clear_inputs(llc);
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].valid = true;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].addr = bypass_addr;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].total_size = 3;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].id = 0x42;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].bypass = true;
  llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].wdata[0] = 0xdeadbeef;
  for (uint32_t b = 0; b < 4; ++b) {
    llc.io.ext_in.upstream.write_req[MASTER_UNCORE_LSU_W].wstrb.set(b, true);
  }
  llc.comb();
  if (!llc.io.ext_out.upstream.write_req[MASTER_UNCORE_LSU_W].ready) {
    printf("FAIL: bypass write not accepted while cacheable context active\n");
    return false;
  }
  llc.seq();

  if (!llc.io.regs.write_ctx[MASTER_DCACHE_W].valid ||
      !llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].valid) {
    printf("FAIL: cross-master contexts not both active dc=%d uc=%d\n",
           static_cast<int>(llc.io.regs.write_ctx[MASTER_DCACHE_W].valid),
           static_cast<int>(llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].valid));
    return false;
  }
  if (llc.io.regs.write_ctx[MASTER_DCACHE_W].bypass ||
      !llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].bypass) {
    printf("FAIL: cross-master context bypass tags wrong dc=%d uc=%d\n",
           static_cast<int>(llc.io.regs.write_ctx[MASTER_DCACHE_W].bypass),
           static_cast<int>(llc.io.regs.write_ctx[MASTER_UNCORE_LSU_W].bypass));
    return false;
  }
  if (!llc.io.regs.lookup_valid_r ||
      llc.io.regs.lookup_master_r != MASTER_DCACHE_W ||
      llc.io.regs.lookup_addr_r != cacheable_addr) {
    printf("FAIL: cacheable lookup ownership lost master=%u addr=0x%x valid=%d\n",
           llc.io.regs.lookup_master_r, llc.io.regs.lookup_addr_r,
           static_cast<int>(llc.io.regs.lookup_valid_r));
    return false;
  }

  clear_inputs(llc);
  llc.io.ext_in.mem.write_req_ready = true;
  llc.comb();
  if (!llc.io.ext_out.mem.write_req_valid ||
      llc.io.ext_out.mem.write_req_addr != bypass_addr ||
      llc.io.ext_out.mem.write_req_size != 3) {
    printf("FAIL: bypass write mem issue mismatch valid=%d addr=0x%x size=%u\n",
           static_cast<int>(llc.io.ext_out.mem.write_req_valid),
           llc.io.ext_out.mem.write_req_addr, llc.io.ext_out.mem.write_req_size);
    return false;
  }

  printf("PASS\n");
  return true;
}

} // namespace

int main() {
  int passed = 0;
  int failed = 0;

  if (test_hit_path())
    passed++;
  else
    failed++;

  if (test_miss_refill())
    passed++;
  else
    failed++;

  if (test_bypass_read())
    passed++;
  else
    failed++;

  if (test_write_passthrough())
    passed++;
  else
    failed++;

  if (test_cacheable_write_updates_table())
    passed++;
  else
    failed++;

  if (test_cacheable_partial_write_miss_preserves_untouched_bytes())
    passed++;
  else
    failed++;

  if (test_cacheable_partial_write_miss_dirty_victim_writes_back_correct_addr_data())
    passed++;
  else
    failed++;

  if (test_cacheable_write_dirty_victim_writeback())
    passed++;
  else
    failed++;

  if (test_same_master_multi_mshr_bypass())
    passed++;
  else
    failed++;

  if (test_read_resp_holds_until_ready())
    passed++;
  else
    failed++;

  if (test_stream_prefetch_table_fill())
    passed++;
  else
    failed++;

  if (test_prefetch_degree_two_queue())
    passed++;
  else
    failed++;

  if (test_demand_mem_issue_preempts_prefetch())
    passed++;
  else
    failed++;

  if (test_lookup_request_is_one_shot())
    passed++;
  else
    failed++;

  if (test_same_line_inflight_not_ready())
    passed++;
  else
    failed++;

  if (test_line_invalidate_maintenance())
    passed++;
  else
    failed++;

  if (test_line_invalidate_same_set_other_way_survives())
    passed++;
  else
    failed++;

  if (test_line_invalidate_same_line_inflight_rejected())
    passed++;
  else
    failed++;

  if (test_line_invalidate_same_line_active_write_ctx_rejected())
    passed++;
  else
    failed++;

  if (test_line_invalidate_same_line_queued_write_rejected())
    passed++;
  else
    failed++;

  if (test_line_invalidate_same_line_write_lookup_rejected())
    passed++;
  else
    failed++;

  if (test_invalidate_all_drops_stale_refill_install())
    passed++;
  else
    failed++;

  if (test_invalidate_all_busy_single_cycle_pulse_is_not_silent_accept())
    passed++;
  else
    failed++;

  if (test_invalidate_all_hold_until_accept_then_pulse())
    passed++;
  else
    failed++;

  if (test_same_master_cacheable_miss_serialized())
    passed++;
  else
    failed++;

  if (test_cacheable_write_wins_lookup_slot())
    passed++;
  else
    failed++;

  if (test_refill_commit_and_new_lookup_do_not_clobber())
    passed++;
  else
    failed++;

  if (test_cacheable_write_hit_then_read_latest())
    passed++;
  else
    failed++;

  if (test_write_hit_updates_pending_read_victim_snapshot())
    passed++;
  else
    failed++;

  if (test_bypass_write_hit_updates_table())
    passed++;
  else
    failed++;

  if (test_pending_upstream_write_blocks_same_line_read())
    passed++;
  else
    failed++;

  if (test_same_set_write_miss_then_read_miss_preserve_both_lines())
    passed++;
  else
    failed++;

  if (test_pending_read_victim_blocks_victim_line_read())
    passed++;
  else
    failed++;

  if (test_pending_read_victim_blocks_victim_line_write())
    passed++;
  else
    failed++;

  if (test_unarmed_refill_ready_victim_blocks_victim_line_access())
    passed++;
  else
    failed++;

  if (test_refill_ready_victim_waits_for_cache_resolved_victim_write())
    passed++;
  else
    failed++;

  if (test_refill_ready_victim_commits_with_unissued_same_line_lookup())
    passed++;
  else
    failed++;

  if (test_refill_ready_victim_queues_snapshot_when_global_slot_busy())
    passed++;
  else
    failed++;

  if (test_same_master_multi_write_queue())
    passed++;
  else
    failed++;

  if (test_same_master_queue_waits_for_resp_slot())
    passed++;
  else
    failed++;

  if (test_cross_master_multi_active_write_contexts())
    passed++;
  else
    failed++;

  printf("LLC results: %d passed, %d failed\n", passed, failed);
  return failed == 0 ? 0 : 1;
}

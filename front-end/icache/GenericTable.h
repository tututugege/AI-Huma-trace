#ifndef GENERIC_TABLE_H
#define GENERIC_TABLE_H
// Two table models:
//   GenericTable<Rows, Chunks, ChunkBits>  register file: the read data of the requested row in the same cycle (the I-cache tag / valid arrays)
//   DynamicSramTable                       SRAM with a fixed read latency, sized at run time (the LLC meta / valid / replacement arrays)
// seq(read, write) is the clock edge; a write is chunk-masked.

#include "base_types.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

// ---------------------------------------------------------------------------------------------------------------------
// DynamicSramTable
// ---------------------------------------------------------------------------------------------------------------------
struct DynamicTableConfig {
  uint32_t rows = 0;
  uint32_t chunks = 0;
  uint32_t chunk_bits = 0;
  uint32_t latency = 1;   // read latency in cycles (>= 1)
};

struct DynamicTablePayload {
  std::vector<uint8_t> bytes{};

  void reset(size_t size) { bytes.assign(size, 0); }
  size_t size() const { return bytes.size(); }
  uint8_t *data() { return bytes.data(); }
  const uint8_t *data() const { return bytes.data(); }
};

struct DynamicTableReadReq {
  bool enable = false;
  uint32_t address = 0;
};

struct DynamicTableReadResp {
  bool valid = false;
  DynamicTablePayload payload{};
};

struct DynamicTableWriteReq {
  bool enable = false;
  uint32_t address = 0;
  DynamicTablePayload payload{};
  std::vector<uint8_t> chunk_enable{};
};

class DynamicSramTable {
public:
  explicit DynamicSramTable(const DynamicTableConfig &config = {}) { configure(config); }

  void configure(const DynamicTableConfig &config) {
    config_ = config;
    storage_.assign(size_t(config_.rows) * row_bytes(), 0);
  }

  void reset() {
    pending_valid_ = false;
    pending_addr_ = 0;
    delay_left_ = 0;
    std::fill(storage_.begin(), storage_.end(), 0);
  }

  // the row requested `latency` cycles ago
  void comb(const DynamicTableReadReq &, DynamicTableReadResp &resp) const {
    resp.valid = false;
    resp.payload.reset(row_bytes());
    if (!pending_valid_ || delay_left_ != 0 || !valid_address(pending_addr_)) return;
    resp.valid = true;
    copy_row(pending_addr_, resp.payload);
  }

  void seq(const DynamicTableReadReq &read_req, const DynamicTableWriteReq &write_req) {
    write_chunks(write_req);
    if (read_req.enable && !pending_valid_ && valid_address(read_req.address)) {
      pending_valid_ = true;
      pending_addr_ = read_req.address;
      delay_left_ = std::max<uint32_t>(config_.latency, 1u) - 1u;
    } else if (!read_req.enable && pending_valid_ && delay_left_ == 0) {
      pending_valid_ = false;
      pending_addr_ = 0;
      delay_left_ = 0;
    } else if (pending_valid_ && delay_left_ > 0) {
      delay_left_--;
    }
  }

  // the row as stored now (no port, no latency)
  bool debug_read_row(uint32_t address, DynamicTablePayload &payload) const {
    payload.reset(row_bytes());
    if (!valid_address(address)) return false;
    copy_row(address, payload);
    return true;
  }

private:
  uint32_t chunk_bytes() const { return (config_.chunk_bits + 7u) / 8u; }
  uint32_t row_bytes() const { return config_.chunks * chunk_bytes(); }
  bool valid_address(uint32_t address) const { return address < config_.rows; }
  void copy_row(uint32_t address, DynamicTablePayload &payload) const {
    if (row_bytes()) std::memcpy(payload.data(), storage_.data() + size_t(address) * row_bytes(), row_bytes());
  }
  void write_chunks(const DynamicTableWriteReq &w) {
    const uint32_t cbytes = chunk_bytes();
    if (!w.enable || !valid_address(w.address) || cbytes == 0 || w.payload.size() < row_bytes() || w.chunk_enable.size() < config_.chunks)
      return;
    uint8_t *dst = storage_.data() + size_t(w.address) * row_bytes();
    for (uint32_t c = 0; c < config_.chunks; ++c)
      if (w.chunk_enable[c]) std::memcpy(dst + size_t(c) * cbytes, w.payload.data() + size_t(c) * cbytes, cbytes);
  }

  DynamicTableConfig config_{};
  std::vector<uint8_t> storage_{};
  bool pending_valid_ = false;
  uint32_t pending_addr_ = 0;
  uint32_t delay_left_ = 0;
};

// ---------------------------------------------------------------------------------------------------------------------
// GenericTable (register file)
// ---------------------------------------------------------------------------------------------------------------------
template <int Chunks, int ChunkBits> struct GenericTablePayload {
  static constexpr int kWordBits = 32;
  static constexpr int kWordsPerChunk = (ChunkBits + kWordBits - 1) / kWordBits;
  std::array<std::array<uint32_t, kWordsPerChunk>, Chunks> chunks{};
};

template <int AddrBits> struct GenericTableReadReq {
  wire<1> enable = false;
  wire<AddrBits> address = 0;
};

template <int Chunks, int ChunkBits> struct GenericTableReadResp {
  wire<1> valid = false;
  GenericTablePayload<Chunks, ChunkBits> payload{};
};

template <int AddrBits, int Chunks, int ChunkBits> struct GenericTableWriteReq {
  wire<1> enable = false;
  wire<AddrBits> address = 0;
  GenericTablePayload<Chunks, ChunkBits> payload{};
  std::array<wire<1>, Chunks> chunk_enable{};
};

template <int Rows, int Chunks, int ChunkBits>
class GenericTable {
public:
  static constexpr int kAddrBits = (Rows <= 1) ? 1 : __builtin_ctz(Rows);
  using ReadReq = GenericTableReadReq<kAddrBits>;
  using ReadResp = GenericTableReadResp<Chunks, ChunkBits>;
  using WriteReq = GenericTableWriteReq<kAddrBits, Chunks, ChunkBits>;
  using Payload = GenericTablePayload<Chunks, ChunkBits>;

  void reset() { storage_.fill(Payload{}); }

  void comb(const ReadReq &req, ReadResp &resp) const {
    resp = {};
    if (!req.enable) return;
    resp.valid = true;
    resp.payload = storage_[req.address];
  }

  void seq(const ReadReq &, const WriteReq &write_req) {
    if (!write_req.enable) return;
    for (int chunk = 0; chunk < Chunks; ++chunk)
      if (write_req.chunk_enable[chunk]) storage_[write_req.address].chunks[chunk] = write_req.payload.chunks[chunk];
  }

  const Payload &peek_row(uint32_t address) const { return storage_[address]; }

private:
  std::array<Payload, Rows> storage_{};
};

#endif

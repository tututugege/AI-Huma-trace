#include "front_fifo.h"
#include "../train_IO.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static void fetch_address_FIFO_comb_impl(const fetch_address_FIFO_in &inp, const fetch_address_FIFO_read_data &rd, FetchAddrCombOut &out) {
  std::memset(&out, 0, sizeof(FetchAddrCombOut));

  out.out_regs.full = false;
  out.out_regs.empty = true;
  out.out_regs.read_valid = false;
  out.out_regs.fetch_address = 0;
  out.out_regs.pair = FetchPair{};

  if (inp.reset) {
    out.clear_fifo = true;
    return;
  }

  bool has_data_before_read = (rd.size > 0);
  uint8_t queue_size_before = rd.size;
  if (inp.refetch) {
    out.clear_fifo = true;
    has_data_before_read = false;
    queue_size_before = 0;
  }

  const bool do_write = inp.write_enable;
  const bool do_read = inp.read_enable && (has_data_before_read || do_write);

  if (do_read) {
    if (has_data_before_read && !rd.head_valid) {
      std::printf("[fetch_address_FIFO] ERROR: missing head snapshot for same-cycle reread\n");
      std::exit(1);
    }
    out.out_regs.read_valid = true;
    out.out_regs.fetch_address =
        has_data_before_read ? rd.head_entry : inp.fetch_address;
    out.out_regs.pair = has_data_before_read ? rd.head_pair : inp.pair;
  }

  out.push_en = do_write;
  out.push_data = inp.fetch_address;
  out.push_pair = inp.pair;
  out.pop_en = do_read;
  out.drop_tail_n = inp.refetch ? 0 : inp.drop_tail_n;

  int next_size = static_cast<int>(queue_size_before);
  next_size -= out.drop_tail_n < next_size ? out.drop_tail_n : next_size;
  next_size += out.push_en ? 1 : 0;
  next_size -= out.pop_en ? 1 : 0;
  if (next_size < 0) {
    next_size = 0;
  }
  out.out_regs.full = (next_size >= (FETCH_ADDR_FIFO_SIZE - 1));
  out.out_regs.empty = (next_size == 0);
}

namespace {

class FetchAddressFifoModel {
public:
  void seq_read(const fetch_address_FIFO_in &inp, fetch_address_FIFO_read_data &rd) const {
    (void)inp;
    std::memset(&rd, 0, sizeof(rd));
    rd.size = size_;
    rd.head_valid = (size_ > 0);
    if (rd.head_valid) {
      rd.head_entry = entries_[0];
      rd.head_pair = pairs_[0];
      rd.head_age = ages_[0];
    }
  }

  void build_next_read_data(const fetch_address_FIFO_read_data &cur,
                            const FetchAddrCombOut &comb_out,
                            fetch_address_FIFO_read_data &next_rd) const {
    if (&next_rd != &cur) {
      next_rd = cur;
    }
    if (comb_out.clear_fifo) {
      next_rd = {};
    }
    if (comb_out.drop_tail_n) {
      next_rd.size -= comb_out.drop_tail_n < next_rd.size ? comb_out.drop_tail_n : next_rd.size;
      if (next_rd.size == 0) { next_rd.head_entry = 0; next_rd.head_valid = false; next_rd.head_age = 0; }
    }
    const fetch_addr_fifo_size_t size_before_ops = next_rd.size;
    if (comb_out.push_en) {
      if (next_rd.size >= FETCH_ADDR_FIFO_SIZE) {
        std::printf("[fetch_address_FIFO] ERROR: fifo overflow\n");
        std::exit(1);
      }
      if (next_rd.size == 0) {
        next_rd.head_entry = comb_out.push_data;
        next_rd.head_pair = comb_out.push_pair;
        next_rd.head_valid = true;
      }
      ++next_rd.size;
    }
    if (comb_out.pop_en) {
      if (next_rd.size == 0) {
        std::printf("[fetch_address_FIFO] ERROR: fifo underflow on read\n");
        std::exit(1);
      }
      --next_rd.size;
      if (next_rd.size == 0) {
        next_rd.head_entry = 0;
        next_rd.head_valid = false;
      } else if (size_before_ops > 1) {
        next_rd.head_entry = 0;
        next_rd.head_valid = false;
      } else if (comb_out.push_en) {
        next_rd.head_entry = comb_out.push_data;
        next_rd.head_pair = comb_out.push_pair;
        next_rd.head_valid = true;
      }
    }
  }

  // read port (first use in the cycle): clear / drop the youngest entries / pop, the cycle's request req (fresh here)
  void read(const fetch_address_FIFO_in &inp, fetch_address_FIFO_read_data &rd, fetch_address_FIFO_out &out,
            FetchAddrCombOut &req) const {
    fetch_address_FIFO_comb_impl(inp, rd, req);
    out = req.out_regs;
    build_next_read_data(rd, req, rd);
  }

  // write port (after the read): push the address (and its FE_PAIR second block)
  void write(const fetch_address_FIFO_in &inp, fetch_address_FIFO_read_data &rd, FetchAddrCombOut &req) const {
    if (req.push_en) {
      std::printf("[fetch_address_FIFO] ERROR: two pushes in one cycle\n");
      std::exit(1);
    }
    req.push_en = true;
    req.push_data = inp.fetch_address;
    req.push_pair = inp.pair;
    FetchAddrCombOut c{};
    c.push_en = true;
    c.push_data = inp.fetch_address;
    c.push_pair = inp.pair;
    build_next_read_data(rd, c, rd);
  }

  void seq_write(const FetchAddrCombOut &req) {
    if (req.clear_fifo) {
      size_ = 0;
      std::memset(entries_, 0, sizeof(entries_));
      for (auto &q : pairs_) q = FetchPair{};
      std::memset(ages_, 0, sizeof(ages_));
    }
    if (req.drop_tail_n) {
      const fetch_addr_fifo_size_t n = req.drop_tail_n < size_ ? req.drop_tail_n : size_;
      for (fetch_addr_fifo_size_t i = size_ - n; i < size_; ++i) { entries_[i] = 0; pairs_[i] = FetchPair{}; ages_[i] = 0; }
      size_ -= n;
    }
    for (fetch_addr_fifo_size_t i = 0; i < size_; ++i) if (ages_[i] < 3) ages_[i]++;  // one cycle older
    if (req.push_en) {
      if (size_ >= FETCH_ADDR_FIFO_SIZE) {
        std::printf("[fetch_address_FIFO] ERROR: fifo overflow\n");
        std::exit(1);
      }
      ages_[size_] = 0;
      pairs_[size_] = req.push_pair;
      entries_[size_++] = req.push_data;
    }
    if (req.pop_en) {
      if (size_ == 0) {
        std::printf("[fetch_address_FIFO] ERROR: fifo underflow on read\n");
        std::exit(1);
      }
      for (fetch_addr_fifo_size_t i = 1; i < size_; ++i) {
        entries_[i - 1] = entries_[i];
        pairs_[i - 1] = pairs_[i];
        ages_[i - 1] = ages_[i];
      }
      --size_;
      entries_[size_] = 0;
      pairs_[size_] = FetchPair{};
      ages_[size_] = 0;
    }
  }

private:
  fetch_addr_t entries_[FETCH_ADDR_FIFO_SIZE]{};
  FetchPair pairs_[FETCH_ADDR_FIFO_SIZE]{};
  uint8_t ages_[FETCH_ADDR_FIFO_SIZE]{};
  fetch_addr_fifo_size_t size_ = 0;
};

FetchAddressFifoModel g_fetch_addr_fifo_model;

} // namespace

void fetch_address_FIFO_seq_read(struct fetch_address_FIFO_in *in,
                                 struct fetch_address_FIFO_read_data *rd) {
  assert(in);
  assert(rd);
  g_fetch_addr_fifo_model.seq_read(*in, *rd);
}

void fetch_address_FIFO_read(const struct fetch_address_FIFO_in *in, struct fetch_address_FIFO_read_data *rd,
                             struct fetch_address_FIFO_out *out, FetchAddrCombOut *req) {
  g_fetch_addr_fifo_model.read(*in, *rd, *out, *req);
}

void fetch_address_FIFO_write(const struct fetch_address_FIFO_in *in, struct fetch_address_FIFO_read_data *rd,
                              FetchAddrCombOut *req) {
  g_fetch_addr_fifo_model.write(*in, *rd, *req);
}

void fetch_address_FIFO_seq_write(const FetchAddrCombOut *req) {
  assert(req);
  g_fetch_addr_fifo_model.seq_write(*req);
}

#include "front_fifo.h"
#include "../train_IO.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "frontend.h"

constexpr uint32_t bit_mask_u32(int bits) {
  return (bits >= 32) ? 0xffffffffu : ((1u << bits) - 1u);
}

static void PTAB_comb_impl(const PTAB_in &inp, const PTAB_read_data &rd, PtabCombOut &out) {
  std::memset(&out, 0, sizeof(PtabCombOut));
  out.out_regs.full = false;
  out.out_regs.empty = true;

  if (inp.reset) {
    out.clear_ptab = true;
    return;
  }

  bool has_data_before_read = (rd.size > 0);
  uint8_t queue_size_before = rd.size;
  if (inp.refetch) {
    out.clear_ptab = true;
    has_data_before_read = false;
    queue_size_before = 0;
  }

  const bool do_write = inp.write_enable;
  const bool do_read = inp.read_enable && (has_data_before_read || do_write);

  PTAB_entry write_entry{};
  if (do_write) {
    for (int i = 0; i < FETCH_WIDTH; i++) {
      write_entry.predict_dir[i] = inp.predict_dir[i];
      write_entry.predict_base_pc[i] = inp.predict_base_pc[i];
    }
    write_entry.meta = inp.meta;
    write_entry.meta.mask();
    write_entry.predict_next_fetch_address = inp.predict_next_fetch_address;
  }

  if (do_read) {
    if (has_data_before_read && !rd.head_valid) {
      std::printf("[PTAB_TOP] ERROR!!: missing head snapshot for same-cycle reread\n");
      std::exit(1);
    }
    const PTAB_entry &read_entry =
        has_data_before_read ? rd.head_entry : write_entry;
    for (int i = 0; i < FETCH_WIDTH; i++) {
      out.out_regs.predict_dir[i] = read_entry.predict_dir[i];
      out.out_regs.predict_base_pc[i] = read_entry.predict_base_pc[i];
    }
    out.out_regs.meta = read_entry.meta;
    out.out_regs.meta.mask();
    out.out_regs.predict_next_fetch_address = read_entry.predict_next_fetch_address;
  }

  out.push_write_en = do_write;
  out.push_write_entry = write_entry;
  out.pop_en = do_read;

  int next_size = static_cast<int>(queue_size_before);
  next_size += out.push_write_en ? 1 : 0;
  next_size -= out.pop_en ? 1 : 0;
  if (next_size < 0) {
    next_size = 0;
  }
  out.out_regs.full = (next_size >= (PTAB_SIZE - 1));
  out.out_regs.empty = (next_size == 0);
}

void PTAB_comb(const PtabCombIn &in, PtabCombOut &out) { PTAB_comb_impl(in.inp, in.rd, out); }

namespace {

class PTABModel {
public:
  void seq_read(const PTAB_in &inp, PTAB_read_data &rd) const {
    (void)inp;
    std::memset(&rd, 0, sizeof(rd));
    rd.size = size_;
    rd.head_valid = (size_ > 0);
    if (rd.head_valid) {
      rd.head_entry = entries_[head_];
    }
  }

  void build_next_read_data(const PTAB_read_data &cur, const PtabCombOut &comb_out,
                            PTAB_read_data &next_rd) const {
    if (&next_rd != &cur) {
      next_rd = cur;
    }
    if (comb_out.clear_ptab) {
      next_rd = {};
    }
    const ptab_size_t size_before_ops = next_rd.size;
    bool first_push_recorded = false;
    PTAB_entry first_push_entry{};
    if (comb_out.push_write_en) {
      if (next_rd.size >= PTAB_SIZE) {
        std::printf("[PTAB_TOP] ERROR!!: ptab.size() >= PTAB_SIZE\n");
        std::exit(1);
      }
      if (next_rd.size == 0) {
        next_rd.head_entry = comb_out.push_write_entry;
        next_rd.head_valid = true;
      }
      first_push_entry = comb_out.push_write_entry;
      first_push_recorded = true;
      ++next_rd.size;
    }
    if (comb_out.pop_en) {
      if (next_rd.size == 0) {
        std::printf("[PTAB_TOP] ERROR!!: ptab underflow on read\n");
        std::exit(1);
      }
      --next_rd.size;
      if (next_rd.size == 0) {
        next_rd.head_entry = PTAB_entry{};
        next_rd.head_valid = false;
      } else if (size_before_ops > 1) {
        next_rd.head_entry = PTAB_entry{};
        next_rd.head_valid = false;
      } else if (first_push_recorded) {
        next_rd.head_entry = first_push_entry;
        next_rd.head_valid = true;
      }
    }
  }

  void comb_calc(const PTAB_in &inp, const PTAB_read_data &rd, PTAB_out &out,
                 PTAB_read_data &next_rd, PtabCombOut &step_req) const {
    PTAB_comb_impl(inp, rd, step_req);
    out = step_req.out_regs;
    build_next_read_data(rd, step_req, next_rd);
  }

  void seq_write(const PtabCombOut &req) {
    if (req.clear_ptab) {
      size_ = 0;
      head_ = 0;
    }
    if (req.push_write_en) {
      if (size_ >= PTAB_SIZE) {
        std::printf("[PTAB_TOP] ERROR!!: ptab.size() >= PTAB_SIZE\n");
        std::exit(1);
      }
      entries_[(head_ + size_) % PTAB_SIZE] = req.push_write_entry;
      ++size_;
    }
    if (req.pop_en) {
      if (size_ == 0) {
        std::printf("[PTAB_TOP] ERROR!!: ptab underflow on read\n");
        std::exit(1);
      }
      head_ = (head_ + 1) % PTAB_SIZE;
      --size_;
    }
  }


private:
  PTAB_entry entries_[PTAB_SIZE]{};
  ptab_size_t size_ = 0;
  ptab_size_t head_ = 0;
};

PTABModel g_ptab_model;

} // namespace

void PTAB_seq_read(struct PTAB_in *in, struct PTAB_read_data *rd) {
  assert(in);
  assert(rd);
  g_ptab_model.seq_read(*in, *rd);
}

void PTAB_comb_calc(struct PTAB_in *in, const struct PTAB_read_data *rd,
                    struct PTAB_out *out, struct PTAB_read_data *next_rd,
                    PtabCombOut *step_req) {
  assert(in);
  assert(rd);
  assert(out);
  assert(next_rd);
  assert(step_req);
  g_ptab_model.comb_calc(*in, *rd, *out, *next_rd, *step_req);
}

void PTAB_seq_write(const PtabCombOut *req) {
  assert(req);
  g_ptab_model.seq_write(*req);
}

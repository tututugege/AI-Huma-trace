#ifndef FRONT_FIFO_H
#define FRONT_FIFO_H

#include "../front_IO.h"
#include "../frontend.h"
#include "../front_module.h"
#include "../BPU/BPU_configs.h"
#include "../train_IO.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

// The four FIFOs of the front end. The fetch-address FIFO (addresses, FE_PAIR second blocks, BPU_PIPE ages) and the
// three entry FIFOs of the front end (instruction FIFO, PTAB, front-to-back FIFO): one registered ring of N entries, one push
// and one pop per cycle; a read of an empty FIFO in the cycle of a write returns the written entry (bypass); reset / refetch empty it.
// read() (first: clear / pop, the head out) and write() (push) fill the cycle's request from the registered read data and
// advance it; seq_write() applies the request.


// Per FIFO: its IO types, depth, "full" threshold and how an entry is taken from the input / put on the output.
struct InstFifo {
  using In = instruction_FIFO_in; using Out = instruction_FIFO_out; using Rd = instruction_FIFO_read_data;
  using Comb = InstructionCombOut; using Entry = instruction_FIFO_entry;
  static constexpr int N = INSTRUCTION_FIFO_SIZE, FULL_AT = N;
  static constexpr const char *name = "instruction FIFO";
  static void load(const In &in, Entry &e) {
    for (int i = 0; i < FETCH_WIDTH; i++) {
      e.instructions[i] = in.fetch_group[i]; e.pc[i] = in.pc[i]; e.page_fault_inst[i] = in.page_fault_inst[i];
      e.inst_valid[i] = in.inst_valid[i]; e.predecode_type[i] = in.predecode_type[i];
      e.predecode_target_address[i] = in.predecode_target_address[i]; e.is_rvc[i] = in.is_rvc[i];
      e.start_a[i] = in.start_a[i]; e.start_b[i] = in.start_b[i]; e.raw16[i] = in.raw16[i];
    }
    e.up16 = in.up16; e.up_ok = in.up_ok; e.seq_next_pc = in.seq_next_pc;
  }
  static void store(const Entry &e, Out &o) {
    for (int i = 0; i < FETCH_WIDTH; i++) {
      o.instructions[i] = e.instructions[i]; o.pc[i] = e.pc[i]; o.page_fault_inst[i] = e.page_fault_inst[i];
      o.inst_valid[i] = e.inst_valid[i]; o.predecode_type[i] = e.predecode_type[i];
      o.predecode_target_address[i] = e.predecode_target_address[i]; o.is_rvc[i] = e.is_rvc[i];
      o.start_a[i] = e.start_a[i]; o.start_b[i] = e.start_b[i]; o.raw16[i] = e.raw16[i];
    }
    o.up16 = e.up16; o.up_ok = e.up_ok; o.seq_next_pc = e.seq_next_pc;
    o.FIFO_valid = true;
  }
};

struct Ptab {
  using In = PTAB_in; using Out = PTAB_out; using Rd = PTAB_read_data; using Comb = PtabCombOut; using Entry = PTAB_entry;
  static constexpr int N = PTAB_SIZE, FULL_AT = N - 1;
  static constexpr const char *name = "PTAB";
  static void load(const In &in, Entry &e) {
    for (int i = 0; i < FETCH_WIDTH; i++) { e.predict_dir[i] = in.predict_dir[i]; e.predict_base_pc[i] = in.predict_base_pc[i]; }
    e.meta = in.meta; e.meta.mask();
    e.predict_next_fetch_address = in.predict_next_fetch_address;
  }
  static void store(const Entry &e, Out &o) {
    for (int i = 0; i < FETCH_WIDTH; i++) { o.predict_dir[i] = e.predict_dir[i]; o.predict_base_pc[i] = e.predict_base_pc[i]; }
    o.meta = e.meta; o.meta.mask();
    o.predict_next_fetch_address = e.predict_next_fetch_address;
  }
};

struct F2bFifo {
  using In = front2back_FIFO_in; using Out = front2back_FIFO_out; using Rd = front2back_FIFO_read_data;
  using Comb = Front2BackCombOut; using Entry = front2back_FIFO_entry;
  static constexpr int N = FRONT2BACK_FIFO_SIZE, FULL_AT = N;
  static constexpr const char *name = "front-to-back FIFO";
  static void load(const In &in, Entry &e) {
    for (int i = 0; i < FETCH_WIDTH; i++) {
      e.fetch_group[i] = in.fetch_group[i]; e.page_fault_inst[i] = in.page_fault_inst[i]; e.inst_valid[i] = in.inst_valid[i];
      e.is_rvc[i] = in.is_rvc[i]; e.predict_dir_corrected[i] = in.predict_dir_corrected[i]; e.predict_base_pc[i] = in.predict_base_pc[i];
    }
    e.meta = in.meta; e.meta.mask();
    e.predict_next_fetch_address_corrected = in.predict_next_fetch_address_corrected;
  }
  static void store(const Entry &e, Out &o) {
    for (int i = 0; i < FETCH_WIDTH; i++) {
      o.fetch_group[i] = e.fetch_group[i]; o.page_fault_inst[i] = e.page_fault_inst[i]; o.inst_valid[i] = e.inst_valid[i];
      o.is_rvc[i] = e.is_rvc[i]; o.predict_dir_corrected[i] = e.predict_dir_corrected[i]; o.predict_base_pc[i] = e.predict_base_pc[i];
    }
    o.meta = e.meta; o.meta.mask();
    o.predict_next_fetch_address_corrected = e.predict_next_fetch_address_corrected;
    o.front2back_FIFO_valid = true;
  }
};

[[noreturn]] inline void fifo_die(const char *name, const char *what) {
  std::printf("[%s] ERROR: %s\n", name, what);
  std::exit(1);
}

template <class F>
class FrontFifo {
  using Entry = typename F::Entry;
  Entry entries_[F::N]{};
  int size_ = 0, head_ = 0;

 public:
  void seq_read(typename F::Rd &rd) const {
    std::memset(&rd, 0, sizeof(rd));
    rd.size = size_;
    rd.head_valid = size_ > 0;
    if (rd.head_valid) rd.head_entry = entries_[head_];
  }

  // Read port, the first use of the FIFO in the cycle: reset / refetch clear it, a read pops the head (from an empty FIFO
  // the entry written in the same cycle, if any: bypass). Fills the cycle's request c (fresh at this point) and advances
  // the read data rd past it.
  void read(const typename F::In &in, typename F::Rd &rd, typename F::Out &out, typename F::Comb &c) const {
    std::memset(&c, 0, sizeof(c));
    c.out_regs.empty = true;
    if (in.reset) {
      c.clear_fifo = true;
    } else {
      bool has_data = rd.size > 0;
      int size = rd.size;
      if (in.refetch) { c.clear_fifo = true; has_data = false; size = 0; }
      const bool do_read = in.read_enable && (has_data || in.write_enable);
      Entry we{};
      if (in.write_enable) F::load(in, we);
      if (do_read) {
        if (has_data && !rd.head_valid) fifo_die(F::name, "missing head snapshot for same-cycle reread");
        F::store(has_data ? rd.head_entry : we, c.out_regs);
      }
      c.push_en = in.write_enable;
      c.push_entry = we;
      c.pop_en = do_read;
      size += (c.push_en ? 1 : 0) - (c.pop_en ? 1 : 0);
      if (size < 0) size = 0;
      c.out_regs.empty = size == 0;
      c.out_regs.full = size >= F::FULL_AT;
    }
    out = c.out_regs;
    advance(rd, c);
  }

  // Write port, after the read: the entry of `in` is pushed (one push per cycle).
  void write(const typename F::In &in, typename F::Rd &rd, typename F::Comb &req) const {
    if (req.push_en) fifo_die(F::name, "two pushes in one cycle");
    req.push_en = true;
    F::load(in, req.push_entry);
    typename F::Comb c{};
    c.push_en = true;
    c.push_entry = req.push_entry;
    advance(rd, c);
  }

  // the read data after the request c
  static void advance(typename F::Rd &rd, const typename F::Comb &c) {
    if (c.clear_fifo) rd = {};
    const int size_before = rd.size;
    if (c.push_en) {
      if (rd.size >= F::N) fifo_die(F::name, "overflow");
      if (rd.size == 0) { rd.head_entry = c.push_entry; rd.head_valid = true; }
      ++rd.size;
    }
    if (c.pop_en) {
      if (rd.size == 0) fifo_die(F::name, "underflow");
      --rd.size;
      if (rd.size == 0 || size_before > 1) { rd.head_entry = Entry{}; rd.head_valid = false; }
      else if (c.push_en) { rd.head_entry = c.push_entry; rd.head_valid = true; }
    }
  }

  void seq_write(const typename F::Comb &c) {
    if (c.clear_fifo) size_ = head_ = 0;
    if (c.push_en) {
      if (size_ >= F::N) fifo_die(F::name, "overflow");
      entries_[(head_ + size_) % F::N] = c.push_entry;
      ++size_;
    }
    if (c.pop_en) {
      if (size_ == 0) fifo_die(F::name, "underflow");
      head_ = (head_ + 1) % F::N;
      --size_;
    }
  }
};

// The fetch-address FIFO: entries in order (index 0 = head), with each entry's FE_PAIR second block and its age (BPU_PIPE).
// Same port protocol as FrontFifo: seq_read (q), read / write (the cycle's request, the read data advanced), seq_write.
class FetchAddrFifo {
public:
  static void comb_impl(const fetch_address_FIFO_in &inp, const fetch_address_FIFO_read_data &rd, FetchAddrCombOut &out) {
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

  void seq_read(fetch_address_FIFO_read_data &rd) const {
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
    comb_impl(inp, rd, req);
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

#endif

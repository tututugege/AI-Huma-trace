#ifndef FRONT_FIFO_H
#define FRONT_FIFO_H

#include "../front_IO.h"
#include "fatal.h"
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
// advance it; seq() applies the request. The read data and the request point at entries (the storage, the written
// entry): only the head's output and the pushed entry at the edge are copies.


// Per FIFO: its IO types, depth, "full" threshold and how an entry is taken from the input / put on the output.
struct InstFifo {
  using In = InstFifoIn; using Out = InstFifoOut; using Rd = InstFifoReadData;
  using Comb = InstructionCombOut; using Entry = InstFifoEntry;
  static constexpr int N = INSTRUCTION_FIFO_SIZE, FULL_AT = N;
  static constexpr const char *name = "instruction FIFO";
  static void load(const Entry &src, Entry &e) { e = src; }
  static void store(const Entry &e, Out &o) {
    for (int i = 0; i < FETCH_WIDTH; i++) {
      o.instructions[i] = e.instructions[i]; o.pc[i] = e.pc[i]; o.page_fault_inst[i] = e.page_fault_inst[i];
      o.inst_valid[i] = e.inst_valid[i]; o.predecode_type[i] = e.predecode_type[i];
      o.predecode_target_address[i] = e.predecode_target_address[i]; o.is_rvc[i] = e.is_rvc[i];
      o.start_a[i] = e.start_a[i]; o.start_b[i] = e.start_b[i]; o.raw16[i] = e.raw16[i];
    }
    o.up16 = e.up16; o.up_ok = e.up_ok; o.seq_next_pc = e.seq_next_pc;
    o.has_alt_a = e.has_alt_a;
    if (e.has_alt_a) {
      for (int i = 0; i < FETCH_WIDTH; i++) {
        o.alt_instructions[i] = e.alt_instructions[i]; o.alt_pc[i] = e.alt_pc[i];
        o.alt_page_fault_inst[i] = e.alt_page_fault_inst[i]; o.alt_inst_valid[i] = e.alt_inst_valid[i];
        o.alt_predecode_type[i] = e.alt_predecode_type[i]; o.alt_predecode_target_address[i] = e.alt_predecode_target_address[i];
        o.alt_is_rvc[i] = e.alt_is_rvc[i]; o.alt_start_a[i] = e.alt_start_a[i];
        o.alt_start_b[i] = e.alt_start_b[i]; o.alt_raw16[i] = e.alt_raw16[i];
      }
      o.alt_up16 = e.alt_up16; o.alt_up_ok = e.alt_up_ok; o.alt_seq_next_pc = e.alt_seq_next_pc;
    }
    o.FIFO_valid = true;
  }
};

[[noreturn]] inline void fifo_die(const char *name, const char *what) {
  fatal("[%s] ERROR: %s\n", name, what);
}

template <class F>
class FrontFifo {
  using Entry = typename F::Entry;
  Entry entries_[F::N]{};
  int size_ = 0, head_ = 0;

 public:
  void read_q(typename F::Rd &rd) const {
    rd.size = size_;
    rd.head_valid = size_ > 0;
    rd.head = rd.head_valid ? &entries_[head_] : nullptr;
  }

  // Read port, the first use of the FIFO in the cycle: reset / refetch clear it, a read pops the head (from an empty FIFO
  // the entry written in the same cycle, if any: bypass). Fills the cycle's request c (fresh at this point) and advances
  // the read data rd past it.
  void read(const typename F::In &in, typename F::Rd &rd, typename F::Out &out, typename F::Comb &c) const {
    c = {};
    out = {};
    out.empty = true;
    if (in.reset) {
      c.clear_fifo = true;
    } else {
      bool has_data = rd.size > 0;
      int size = rd.size;
      if (in.refetch) { c.clear_fifo = true; has_data = false; size = 0; }
      const bool do_read = in.read_enable && (has_data || in.write_enable);
      if (do_read) {
        if (has_data && !rd.head_valid) fifo_die(F::name, "missing head snapshot for same-cycle reread");
        F::store(has_data ? *rd.head : in.wdata, out);
      }
      c.push_en = in.write_enable;
      c.push = in.write_enable ? &in.wdata : nullptr;
      c.pop_en = do_read;
      size += (c.push_en ? 1 : 0) - (c.pop_en ? 1 : 0);
      if (size < 0) size = 0;
      out.empty = size == 0;
      out.full = size >= F::FULL_AT;
    }
    advance(rd, c);
  }

  // Write port, after the read: the entry of `in` is pushed (one push per cycle).
  void write(const typename F::In &in, typename F::Rd &rd, typename F::Comb &req) const {
    if (req.push_en) fifo_die(F::name, "two pushes in one cycle");
    req.push_en = true;
    req.push = &in.wdata;
    typename F::Comb c{};
    c.push_en = true;
    c.push = &in.wdata;
    advance(rd, c);
  }

  // the read data after the request c
  static void advance(typename F::Rd &rd, const typename F::Comb &c) {
    if (c.clear_fifo) rd = {};
    const int size_before = rd.size;
    if (c.push_en) {
      if (rd.size >= F::N) fifo_die(F::name, "overflow");
      if (rd.size == 0) { rd.head = c.push; rd.head_valid = true; }
      ++rd.size;
    }
    if (c.pop_en) {
      if (rd.size == 0) fifo_die(F::name, "underflow");
      --rd.size;
      if (rd.size == 0 || size_before > 1) { rd.head = nullptr; rd.head_valid = false; }
      else if (c.push_en) { rd.head = c.push; rd.head_valid = true; }
    }
  }

  void seq(const typename F::Comb &c) {
    if (c.clear_fifo) size_ = head_ = 0;
    if (c.push_en) {
      if (size_ >= F::N) fifo_die(F::name, "overflow");
      F::load(*c.push, entries_[(head_ + size_) % F::N]);
      ++size_;
    }
    if (c.pop_en) {
      if (size_ == 0) fifo_die(F::name, "underflow");
      head_ = (head_ + 1) % F::N;
      --size_;
    }
  }
};

#endif

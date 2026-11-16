// The three entry FIFOs of the front end (instruction FIFO, PTAB, front-to-back FIFO): one registered ring of N entries, one push
// and one pop per cycle; a read of an empty FIFO in the cycle of a write returns the written entry (bypass); reset / refetch empty it.
// comb_calc() computes the outputs and the update request from the registered read data; seq_write() applies the request.
#include "front_fifo.h"
#include "../train_IO.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

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

[[noreturn]] void fifo_die(const char *name, const char *what) {
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

  // outputs and update request from the registered read data; next_rd = the read data after the request (for a second
  // pass in the same cycle; may alias rd)
  void comb_calc(const typename F::In &in, const typename F::Rd &rd, typename F::Out &out, typename F::Rd &next_rd,
                 typename F::Comb &c) const {
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
    // the read data after this request
    if (&next_rd != &rd) next_rd = rd;
    if (c.clear_fifo) next_rd = {};
    const int size_before = next_rd.size;
    if (c.push_en) {
      if (next_rd.size >= F::N) fifo_die(F::name, "overflow");
      if (next_rd.size == 0) { next_rd.head_entry = c.push_entry; next_rd.head_valid = true; }
      ++next_rd.size;
    }
    if (c.pop_en) {
      if (next_rd.size == 0) fifo_die(F::name, "underflow");
      --next_rd.size;
      if (next_rd.size == 0 || size_before > 1) { next_rd.head_entry = Entry{}; next_rd.head_valid = false; }
      else if (c.push_en) { next_rd.head_entry = c.push_entry; next_rd.head_valid = true; }
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

FrontFifo<InstFifo> g_inst_fifo;
FrontFifo<Ptab> g_ptab;
FrontFifo<F2bFifo> g_f2b_fifo;

}  // namespace

void instruction_FIFO_seq_read(instruction_FIFO_in *, instruction_FIFO_read_data *rd) { g_inst_fifo.seq_read(*rd); }
void instruction_FIFO_comb_calc(instruction_FIFO_in *in, const instruction_FIFO_read_data *rd, instruction_FIFO_out *out,
                                instruction_FIFO_read_data *next_rd, InstructionCombOut *c) { g_inst_fifo.comb_calc(*in, *rd, *out, *next_rd, *c); }
void instruction_FIFO_seq_write(const InstructionCombOut *c) { g_inst_fifo.seq_write(*c); }

void PTAB_seq_read(PTAB_in *, PTAB_read_data *rd) { g_ptab.seq_read(*rd); }
void PTAB_comb_calc(PTAB_in *in, const PTAB_read_data *rd, PTAB_out *out, PTAB_read_data *next_rd, PtabCombOut *c) {
  g_ptab.comb_calc(*in, *rd, *out, *next_rd, *c);
}
void PTAB_seq_write(const PtabCombOut *c) { g_ptab.seq_write(*c); }

void front2back_FIFO_seq_read(front2back_FIFO_in *, front2back_FIFO_read_data *rd) { g_f2b_fifo.seq_read(*rd); }
void front2back_FIFO_comb_calc(front2back_FIFO_in *in, const front2back_FIFO_read_data *rd, front2back_FIFO_out *out,
                               front2back_FIFO_read_data *next_rd, Front2BackCombOut *c) { g_f2b_fifo.comb_calc(*in, *rd, *out, *next_rd, *c); }
void front2back_FIFO_seq_write(const Front2BackCombOut *c) { g_f2b_fifo.seq_write(*c); }

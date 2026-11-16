#include "Isu.h"
#include "Wires.h"
#include "config.h"
#include <cstdint>
#include <cstring>
#include <vector>

// Completion-cycle table of the issue ports (EXP-034): a port completes one uop per cycle (one ROB / write-back entry), so the
// issue stage reserves the cycle in which a granted uop completes and a uop that would collide with an earlier one on the same
// port stays in the queue (like XiangShan's WbFuBusyTable).
namespace {
struct WbBusy {
  static constexpr int kRing = 64;
  uint8_t busy[ISSUE_WIDTH][kRing] = {};
  long long last_cycle = -1;
  void new_cycle(bool flush) {
    if (sim_time == last_cycle) return;
    last_cycle = sim_time;
    for (int p = 0; p < ISSUE_WIDTH; p++) busy[p][(sim_time - 1) & (kRing - 1)] = 0;
    if (flush) std::memset(busy, 0, sizeof busy);
  }
  bool free(int port, int off) const {
    Assert(off >= 1 && off < kRing);
    return !busy[port][(sim_time + off) & (kRing - 1)];
  }
  void reserve(int port, int off) { busy[port][(sim_time + off) & (kRing - 1)] = 1; }
};
WbBusy g_wb;
// FP register-file write ports (EXP-032): at most FP_WB_PORTS writes of the pipelined / iterative FP units (FMA, MISC with an FP destination,
// FDIV, I2F) per cycle; the write cycle is known at issue, so it is reserved there.  (FP loads write through the LSU and are not limited.)
struct FpWbBusy {
  static constexpr int kRing = 64, FP_WB_PORTS = 2;
  uint8_t busy[kRing] = {};
  long long last_cycle = -1;
  void new_cycle(bool flush) {
    if (sim_time == last_cycle) return;
    last_cycle = sim_time;
    busy[(sim_time - 1) & (kRing - 1)] = 0;
    if (flush) std::memset(busy, 0, sizeof busy);
  }
  bool try_reserve(int lat) {
    uint8_t &b = busy[(sim_time + lat) & (kRing - 1)];
    if (b >= FP_WB_PORTS) return false;
    b++;
    return true;
  }
};
FpWbBusy g_fpwb;
}  // namespace

Isu::Isu(SimContext *context) : ctx(context) {}

void Isu::add_iq(const IssueQueueConfig &cfg) {
  iqs.emplace_back(cfg);
  configs.push_back(cfg);
}

// A register woken by this cycle's broadcast is no longer busy for an entering uop.
void Isu::apply_wakeup_to_uop(Uop &uop) const {
  for (int k = 0; k < WAKE_CH_ALL; k++) {
    if (!wires->iss_awake.wake[k].valid) continue;
    const uint32_t preg = wires->iss_awake.wake[k].preg;
    if (uop->src1_en && uop->src1_preg == preg) uop.src1_busy = false;
    if (uop->src2_en && uop->src2_preg == preg) uop.src2_busy = false;
  }
}

void Isu::clear_wake_state() {
  for (auto &stage : mul_wake_pipe) for (auto &e : stage) e = {};
  for (auto &e : div_wake_slots) e = {};
  for (auto &e : fp_wake_slots) e = {};
  for_each_wake_slot_1([](auto &e) { e = {}; });
}

// One issue queue per GLOBAL_IQ_CONFIG entry (the order matches the IQ enum); each claims its range of physical issue ports.
void Isu::init() {
  iqs.clear();
  configs.clear();
  clear_wake_state();
  for (int i = 0; i < IQ_NUM; i++) {
    const auto &iq = GLOBAL_IQ_CONFIG[i];
    IssueQueueConfig cfg;
    cfg.id = iq.id;
    cfg.size = iq.size;
    cfg.dispatch_width = iq.dispatch_width;
    cfg.supported_ops = iq.supported_ops;  // coarse mask Dispatch routes with
    for (int p = 0; p < iq.port_num; p++) {
      // GLOBAL_ISSUE_PORT_CONFIG's index is the canonical physical issue-port id (the same one Exu uses)
      const int port = iq.port_start_idx + p;
      cfg.ports.push_back({port, GLOBAL_ISSUE_PORT_CONFIG[port].support_mask});
    }
    add_iq(cfg);
  }
  for (auto &q : iqs) q.in.grant_gate = [this](const Uop &u, int port) { return grant_gate(u, port); };
}

// Working copies already equal the registered state (seq() copies them back); each IQ resets its default IO.
void Isu::comb_begin() {
  for (auto &q : iqs) q.comb_begin();
}

// Cycles until the result of a uop is available; 1 means fast wakeup (the consumer can issue next cycle).
int Isu::get_latency(UopType uop, wire<7> func7, bool word) {
  if (uop == UOP_MUL) return MUL_MAX_LATENCY;
  if (uop == UOP_DIV) return div_latency(word);
  if (uop == UOP_FMA) return FP_FMA_LAT;
  if (uop == UOP_FMISC) return FP_MISC_LAT;
  if (uop == UOP_I2F) return FP_I2F_LAT;
  if (uop == UOP_FDIV) {   // funct7[1:0] = fmt (1 = D), funct5 0x0b = fsqrt
    const bool d = func7 & 1, sq = (func7 >> 2) == 0x0b;
    return sq ? (d ? FP_FSQRT_D_LAT : FP_FSQRT_S_LAT) : (d ? FP_FDIV_D_LAT : FP_FDIV_S_LAT);
  }
  return 1;
}

// Free slots of each queue go to Dispatch.
void Isu::comb_ready() {
  for (int i = 0; i < IQ_NUM; i++) wires->iss2dis.ready_num[i] = iqs[i].out.free_slots;
}

// Dispatch requests enter their queue, with this cycle's wakeups applied first (a readable source must not enter busy).
void Isu::comb_enq() {
  for (int i = 0; i < IQ_NUM; i++) {
    auto &q = iqs[i];
    const int width = configs[i].dispatch_width;
    for (int w = 0; w < width; w++) {
      auto &slot = q.in.enq_reqs[w];
      slot.valid = false;  // the payload is only read when valid
      if (!wires->dis2iss.req[i][w].valid) continue;
      slot.uop = wires->dis2iss.req[i][w].uop;
      apply_wakeup_to_uop(slot.uop);
      slot.valid = true;
    }
    q.comb_enq();
  }
}

// Extra grant conditions of a pick: its completion cycle on the port is free (EXP-035), and the FP register-file write cycle of an
// FP-writing unit (EXP-032). The reservations are made here, only for a grant that is about to be made.
bool Isu::grant_gate(const Uop &u, int port) {
  const UopType t = decode_uop_type(u.op);
  const int lat = get_latency(t, u->func7, u->is_word);
  const int off = (t == UOP_DIV || t == UOP_FDIV) ? lat + 1 : lat;   // iterative units finish one cycle later than their wake-up latency
  if (!g_wb.free(port, off)) return false;   // another uop of the port completes in that cycle
  if (u.dest_en && u->dest_fp && (t == UOP_FMA || t == UOP_FMISC || t == UOP_FDIV || t == UOP_I2F) &&
      !g_fpwb.try_reserve(lat)) return false;
  g_wb.reserve(port, off);
  return true;
}

// Every queue schedules; a grant needs the port's FU capability. No issue during a flush or mispredict.
void Isu::comb_issue() {
  for (int i = 0; i < ISSUE_WIDTH; i++) wires->iss2prf.iss_entry[i].valid = false;
  g_wb.new_cycle(wires->rob_bcast.flush);
  g_fpwb.new_cycle(wires->rob_bcast.flush);
  for (auto &q : iqs) {
    q.in.issue_block = wires->rob_bcast.flush || wires->dec_bcast.mispred;
    for (int p = q.port_lo; p < q.port_hi; p++) q.in.port_fu_ready_mask[p] = wires->exe2iss.fu_ready_mask[p];
    q.comb_issue();
    for (int p = q.port_lo; p < q.port_hi; p++) {
      const auto &grant = q.out.issue_grants[p];
      if (!grant.valid) continue;
      Assert(!wires->iss2prf.iss_entry[p].valid);
      wires->iss2prf.iss_entry[p].valid = true;
      wires->iss2prf.iss_entry[p].uop = grant.uop;
    }
  }
}

// Next state of the delayed wakeups: DIV/FP count down (an entry that fired is dropped), the MUL pipeline shifts, and
// uops issued this cycle with a multi-cycle latency enter them.
void Isu::comb_calc_latency_next() {
  auto age = [](auto &cur, auto &nxt) {
    for (size_t i = 0; i < std::size(cur); i++) {
      nxt[i] = {};
      if (cur[i].valid && cur[i].countdown > 0) {
        nxt[i] = cur[i];
        nxt[i].countdown--;
      }
    }
  };
  age(div_wake_slots, div_wake_slots_1);
  age(fp_wake_slots, fp_wake_slots_1);

  for (auto &stage : mul_wake_pipe_1) for (auto &e : stage) e = {};
  if (MUL_MAX_LATENCY > 1)
    for (int stage = ISU_MUL_WAKE_DEPTH - 1; stage >= 1; stage--)
      for (int s = 0; s < ISU_MUL_WAKE_SLOT_NUM; s++) mul_wake_pipe_1[stage][s] = mul_wake_pipe[stage - 1][s];

  for (int i = 0; i < ISSUE_WIDTH; i++) {
    const auto &inst = wires->iss2prf.iss_entry[i];
    if (!(inst.valid && inst.uop.dest_en)) continue;
    const UopType op = decode_uop_type(inst.uop.op);
    const int lat = get_latency(op, inst.uop->func7, inst.uop->is_word);
    if (lat <= 1) continue;
    auto put = [&](auto &e, int countdown) {
      e.valid = true;
      e.dest_preg = inst.uop.dest_preg;
      e.br_mask = inst.uop.br_mask;
      if constexpr (requires { e.countdown; }) e.countdown = countdown;
    };
    auto free_slot = [&](auto &slots, int countdown) {
      for (auto &e : slots)
        if (!e.valid) { put(e, countdown); return true; }
      return false;
    };
    bool ok = true;  // the placement must not sit inside Assert (it has to run in every build)
    if (op == UOP_MUL) ok = MUL_MAX_LATENCY > 1 && free_slot(mul_wake_pipe_1[0], 0);
    else if (op == UOP_DIV) ok = free_slot(div_wake_slots_1, lat - 1);
    else if (op == UOP_FMA || op == UOP_FMISC || op == UOP_FDIV || op == UOP_I2F) ok = free_slot(fp_wake_slots_1, lat - 1);
    Assert(ok && "delayed-wakeup slots full");
  }
}

// Gather every wakeup source onto its own broadcast channel (so write-backs never collide) and wake the queues:
// load write-backs (slow), DIV/FP countdown reaching zero and the last MUL stage (delayed), and uops issued this
// cycle with single-cycle latency (fast, ALU ports only).
void Isu::comb_awake() {
  auto &bus = wires->iss_awake.wake;
  for (int i = 0; i < WAKE_CH_ALL; i++) {
    bus[i].valid = false;
    bus[i].preg = 0;
  }
  const int lsu_base = 0;
  const int mul_base = lsu_base + LSU_LOAD_WB_WIDTH;
  const int div_base = mul_base + count_ports_with_mask(OP_MASK_MUL);
  const int fast_base = div_base + count_ports_with_mask(OP_MASK_DIV);
  auto drive = [&](int ch, uint32_t preg, const char *collision) {
    Assert(!bus[ch].valid && collision);
    bus[ch].valid = true;
    bus[ch].preg = preg;
  };

  int fast_ch[ISSUE_WIDTH];
  int next_fast = fast_base;
  for (int i = 0; i < ISSUE_WIDTH; i++) fast_ch[i] = (get_port_capability(i) & OP_MASK_ALU) ? next_fast++ : -1;

  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++)
    if (wires->prf_awake.wake[i].valid) {
      bus[lsu_base + i].valid = true;
      bus[lsu_base + i].preg = wires->prf_awake.wake[i].preg;
    }
  for (int i = 0; i < ISU_DIV_WAKE_SLOT_NUM; i++)
    if (div_wake_slots[i].valid && div_wake_slots[i].countdown == 0) drive(div_base + i, div_wake_slots[i].dest_preg, "Delayed writeback collision");
  {  // the FP slots fire on the FPW_CH channels after the fast ones, in slot order
    int ch = fast_base + ALU_NUM;
    for (int i = 0; i < ISU_FP_WAKE_SLOT_NUM; i++)
      if (fp_wake_slots[i].valid && fp_wake_slots[i].countdown == 0) {
        Assert(ch < fast_base + ALU_NUM + FPW_CH && "more FP wake-ups in one cycle than channels");
        drive(ch++, fp_wake_slots[i].dest_preg, "Delayed writeback collision");
      }
  }
  if (MUL_MAX_LATENCY > 1)
    for (int i = 0; i < ISU_MUL_WAKE_SLOT_NUM; i++)
      if (mul_wake_pipe[ISU_MUL_WAKE_DEPTH - 1][i].valid) drive(mul_base + i, mul_wake_pipe[ISU_MUL_WAKE_DEPTH - 1][i].dest_preg, "Delayed writeback collision");
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    const auto &e = wires->iss2prf.iss_entry[i];
    if (!(e.valid && e.uop.dest_en)) continue;
    const UopType op = decode_uop_type(e.uop.op);
    if (get_latency(op, e.uop->func7, e.uop->is_word) <= 1 && op != UOP_LOAD && op != UOP_STA && fast_ch[i] != -1)
      drive(fast_ch[i], e.uop.dest_preg, "Fast writeback collision");
  }

  {  // LSU channels after the fast ones ([p] unused since the speculative L2 wake-up was removed, [LD + p] L3 register)
    const int early_base = MAX_WAKEUP_PORTS;  // config.h channels first
    for (int i = 0; i < LSU_EARLY_WAKE_PORTS; i++)
      if (wires->lsu_awake.wake[i].valid) {
        bus[early_base + i].valid = true;
        bus[early_base + i].preg = wires->lsu_awake.wake[i].preg;
      }
  }
  uint32_t woken[WAKE_CH_ALL];
  int n_woken = 0;
  for (int i = 0; i < WAKE_CH_ALL; i++)
    if (bus[i].valid) woken[n_woken++] = bus[i].preg;
  if (n_woken)
    for (auto &q : iqs) q.wakeup(woken, n_woken);
}

// Flush / mispredict / branch-resolve handling for the queues and the delayed wakeups. A flush empties everything,
// a mispredict kills the wakeups of squashed uops, and resolved branches free their mask bit in the survivors.
void Isu::comb_flush() {
  const bool flush_all = wires->rob_bcast.flush;
  const bool flush_br = !flush_all && wires->dec_bcast.mispred;
  const wire<BR_MASK_WIDTH> br_mask = wires->dec_bcast.br_mask, clear = wires->dec_bcast.clear_mask;

  for (auto &q : iqs) {
    q.in.flush_all = flush_all;
    q.in.flush_br = flush_br;
    q.in.flush_br_mask = br_mask;
    q.in.clear_mask = clear;
    q.comb_flush();
  }
  if (flush_all) {
    clear_wake_state();
  } else if (wires->dec_bcast.mispred) {
    for_each_wake_slot_1([&](auto &e) { if (e.valid && (e.br_mask & br_mask) != 0) e = {}; });
  }
  if (clear) for_each_wake_slot_1([&](auto &e) { if (e.valid) e.br_mask &= ~clear; });
}

void Isu::seq() {
  for (auto &q : iqs) q.seq();
  for (int d = 0; d < ISU_MUL_WAKE_DEPTH; d++)
    for (int s = 0; s < ISU_MUL_WAKE_SLOT_NUM; s++) mul_wake_pipe[d][s] = mul_wake_pipe_1[d][s];
  for (int i = 0; i < ISU_DIV_WAKE_SLOT_NUM; i++) div_wake_slots[i] = div_wake_slots_1[i];
  for (int i = 0; i < ISU_FP_WAKE_SLOT_NUM; i++) fp_wake_slots[i] = fp_wake_slots_1[i];
}

#include "BeginCheck.h"
#include "Isu.h"
#include "Wires.h"
#include "config.h"
#include <cstdint>
#include <cstring>
#include <vector>

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
}

// Working copies already equal the registered state (seq() copies them back); each IQ resets its default IO.
void Isu::comb_begin() {
  for (auto &q : iqs) q.comb_begin();
  BEGIN_EQ(mul_wake_pipe_1, mul_wake_pipe);
  BEGIN_EQ(div_wake_slots_1, div_wake_slots);
  BEGIN_EQ(fp_wake_slots_1, fp_wake_slots);
}

// Cycles until the result of a uop is available; 1 means fast wakeup (the consumer can issue next cycle).
int Isu::get_latency(UopType uop, wire<7> func7, bool word) {
  if (uop == UOP_MUL) return MUL_MAX_LATENCY;
  if (uop == UOP_DIV) return div_latency(word);
#if FP
  if (uop == UOP_FMA) return FP_FMA_LAT;
  if (uop == UOP_FMISC) return FP_MISC_LAT;
  if (uop == UOP_I2F) return FP_I2F_LAT;
  if (uop == UOP_FDIV) {   // funct7[1:0] = fmt (1 = D), funct5 0x0b = fsqrt
    const bool d = func7 & 1, sq = (func7 >> 2) == 0x0b;
    return sq ? (d ? FP_FSQRT_D_LAT : FP_FSQRT_S_LAT) : (d ? FP_FDIV_D_LAT : FP_FDIV_S_LAT);
  }
#endif
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

// Every queue schedules; a grant needs the port's FU capability. No issue during a flush or mispredict.
void Isu::comb_issue() {
  for (int i = 0; i < ISSUE_WIDTH; i++) wires->iss2prf.iss_entry[i].valid = false;
  for (auto &q : iqs) {
    q.in.issue_block = wires->rob_bcast.flush || wires->dec_bcast.mispred;
    for (int p = 0; p < ISSUE_WIDTH; p++) q.in.port_fu_ready_mask[p] = wires->exe2iss.fu_ready_mask[p];
    q.comb_issue();
    for (int p = 0; p < ISSUE_WIDTH; p++) {
      const auto &grant = q.out.issue_grants[p];
      if (!grant.valid) continue;
      Assert(!wires->iss2prf.iss_entry[p].valid);
      wires->iss2prf.iss_entry[p].valid = true;
      wires->iss2prf.iss_entry[p].uop = grant.uop;
#if FP
      {  // F/D statistics (EXP-030), printed at exit
        static struct FpStats {
          uint64_t issued[MAX_UOP_TYPE] = {0};
          uint64_t d_issued = 0;
          ~FpStats() {
            std::fprintf(stderr, "[FP] issued: fma=%llu fmisc=%llu fdiv=%llu i2f=%llu (double precision: %llu)\n",
                         (unsigned long long)issued[UOP_FMA], (unsigned long long)issued[UOP_FMISC],
                         (unsigned long long)issued[UOP_FDIV], (unsigned long long)issued[UOP_I2F], (unsigned long long)d_issued);
          }
        } st;
        const UopType t = decode_uop_type(grant.uop.op);
        st.issued[t]++;
        if (t == UOP_FMA || t == UOP_FMISC || t == UOP_FDIV) st.d_issued += grant.uop->func7 & 1;
      }
#endif
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
#if FP
  {  // the FP slots fire on the FPW_CH channels after the fast ones, in slot order
    int ch = fast_base + ALU_NUM;
    for (int i = 0; i < ISU_FP_WAKE_SLOT_NUM; i++)
      if (fp_wake_slots[i].valid && fp_wake_slots[i].countdown == 0) {
        Assert(ch < fast_base + ALU_NUM + FPW_CH && "more FP wake-ups in one cycle than channels");
        drive(ch++, fp_wake_slots[i].dest_preg, "Delayed writeback collision");
      }
  }
#endif
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

  {  // LSU_PIPE_L2 channels: after the fast ones (L2 speculative, then L3 register)
    const int early_base = MAX_WAKEUP_PORTS;  // config.h channels first
    for (int i = 0; i < LSU_EARLY_WAKE_PORTS; i++)
      if (wires->lsu_awake.wake[i].valid) {
        bus[early_base + i].valid = true;
        bus[early_base + i].preg = wires->lsu_awake.wake[i].preg;
      }
  }
  for (auto &q : iqs) {
    for (int i = 0; i < WAKE_CH_ALL; i++) {
      q.in.wake_valid[i] = bus[i].valid;
      q.in.wake_pregs[i] = bus[i].preg;
    }
    q.comb_wakeup();
  }
}

// Speculative load wake-up of the previous cycle was wrong (before the select of this cycle): woken entries wait again.
extern uint64_t g_lp_cancel_entries;
void Isu::comb_cancel() {
  for (int k = 0; k < LSU_LOAD_WB_WIDTH; k++)
    if (wires->lsu_awake.cancel[k].valid)
      for (auto &q : iqs) g_lp_cancel_entries += q.cancel_wake(wires->lsu_awake.cancel[k].preg);
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

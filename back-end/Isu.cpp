#include "Isu.h"
#include "AguPorts.h"
#include "BackPorts.h"
#include "config.h"
#include "debug_config.h"
#include "trace_win.h"
#include "Pmu.h"
#include <cstdio>
#include <cstdlib>
#include <array>
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
  if (uop->src1_en && out.iss_awake.woken(uop->src1_preg)) uop.src1_busy = false;
  if (uop->src2_en && out.iss_awake.woken(uop->src2_preg)) uop.src2_busy = false;
}

// Empties the wake slots (a global flush at the edge; init).
void Isu::clear_wake_state() {
  for_each_wake_slot([](auto &e) { e = {}; });
}

extern uint64_t g_dbg_cycle;
static bool isutr_on() {
  static long lo = -1, hi = -1; static bool init = false;
  if (!init) { init = true; if (const char *e = std::getenv("ROBTR")) std::sscanf(e, "%ld:%ld", &lo, &hi); }
  return lo >= 0 && (long)g_dbg_cycle >= lo && (long)g_dbg_cycle <= hi;
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
    // GLOBAL_ISSUE_PORT_CONFIG's index is the canonical physical issue-port id (the same one Exu uses)
    if (iq.port_mask) {
      for (int port = 0; port < ISSUE_WIDTH; port++)
        if (iq.port_mask >> port & 1) cfg.ports.push_back({port, GLOBAL_ISSUE_PORT_CONFIG[port].support_mask});
    } else {
      for (int p = 0; p < iq.port_num; p++) {
        const int port = iq.port_start_idx + p;
        cfg.ports.push_back({port, GLOBAL_ISSUE_PORT_CONFIG[port].support_mask});
      }
    }
    add_iq(cfg);
  }
  for (auto &q : iqs) {
    q.in.grant_gate = [this](const Uop &u, int port) { return grant_gate(u, port, true); };
    q.in.gate_probe = [this](const Uop &u, int port) { return grant_gate(u, port, false); };
    q.in.presel_ok = [this](const Uop &u, int port) {
      const UopType t = decode_uop_type(u.op);
      if (t == UOP_DIV) return true;  // variable-latency DIV reserves port 1 wb via exe2iss.div_wb_next
      const int lat = get_latency(t, u->func7, u->is_word);
      return wb_busy_.free(port, (t == UOP_FDIV) ? lat + 1 : lat);
    };
    q.in.on_refuse = [](int why, bool lost) {
      g_pmu_back.iss_ref[why - 1]++;
      if (lost) g_pmu_back.iss_lost[why - 1]++;
    };
  }
}

// Each IQ resets its default IO.
void Isu::begin_cycle() {
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
void Isu::ready() {
  begin_cycle();   // the first phase of the cycle: default outputs
  for (int i = 0; i < IQ_NUM; i++) out.iss2dis.ready_num[i] = iqs[i].out.free_slots;
}

// Dispatch requests enter their queue, with this cycle's wakeups applied first (a readable source must not enter busy).
void Isu::enq() {
  for (int i = 0; i < IQ_NUM; i++) {
    auto &q = iqs[i];
    const int width = configs[i].dispatch_width;
    for (int w = 0; w < width; w++) {
      auto &slot = q.in.enq_reqs[w];
      slot.valid = false;  // the payload is only read when valid
      if (!in.dis2iss->req[i][w].valid) continue;
      slot.uop = in.dis2iss->req[i][w].uop;
      apply_wakeup_to_uop(slot.uop);
      if (isutr_on() && slot.uop->rob_idx == 14) std::fprintf(stderr, "ISUTR %llu ENQ q=%d op=%d rob=%d src1=%d(en%d busy%d) src2=%d(en%d busy%d) dest=%d\n", (unsigned long long)g_dbg_cycle, i, (int)slot.uop.op, (int)slot.uop->rob_idx, (int)slot.uop->src1_preg, (int)slot.uop->src1_en, (int)slot.uop.src1_busy, (int)slot.uop->src2_preg, (int)slot.uop->src2_en, (int)slot.uop.src2_busy, (int)slot.uop->dest_preg);
      slot.valid = true;
    }
  }
}

// Extra grant conditions of a pick: its completion cycle on the port is free (EXP-035), and the FP register-file write cycle of an
// FP-writing unit (EXP-032). The reservations are made here, only for a grant that is about to be made.
int Isu::grant_gate(const Uop &u, int port, bool reserve) {
  // Shared AGUs (AguPorts.h): the physical AGU port of a load / STA grant; the load queue is scheduled first, so a load wins a
  // shared port and the STA grant is refused (its slot stays in the queue).
  int agu_port = -1;
  if (port >= IQ_LD_PORT_BASE && port < IQ_LD_PORT_BASE + LSU_LDU_COUNT) agu_port = agu::ld_port(port - IQ_LD_PORT_BASE);
  if (port >= IQ_STA_PORT_BASE && port < IQ_STA_PORT_BASE + LSU_STA_COUNT) agu_port = agu::st_port(port - IQ_STA_PORT_BASE);
  if (agu_port >= 0 && (agu_ports_used_ >> agu_port & 1)) return GATE_REF_AGU;
  const UopType t = decode_uop_type(u.op);
  const int lat = get_latency(t, u->func7, u->is_word);
  const int off = (t == UOP_FDIV) ? lat + 1 : lat;   // iterative FP units finish one cycle later than their wake-up latency
  if (t != UOP_DIV && !wb_busy_.free(port, off)) return GATE_REF_WB;   // another uop of the port completes in that cycle
  const bool fpw = u.dest_en && u->dest_fp && (t == UOP_FMA || t == UOP_FMISC || t == UOP_FDIV || t == UOP_I2F);
  if (!reserve) return fpw && fp_wb_busy_.full(lat) ? GATE_REF_FPWB : GATE_OK;
  if (fpw && !fp_wb_busy_.try_reserve(lat)) return GATE_REF_FPWB;
  if (t != UOP_DIV) wb_busy_.reserve(port, off);
  if (agu_port >= 0) agu_ports_used_ |= 1u << agu_port;
  return GATE_OK;
}

// Every queue schedules; a grant needs the port's FU capability. No issue during a flush or mispredict.
void Isu::issue() {
  for (int i = 0; i < ISSUE_WIDTH; i++) out.iss2prf.iss_entry[i].valid = false;
  wb_busy_.new_cycle(in.rob_bcast->flush);
  fp_wb_busy_.new_cycle(in.rob_bcast->flush);
  agu_ports_used_ = 0;
  for (int k = 0; k < 4; k++) g_pmu_back.iss_ref[k] = g_pmu_back.iss_lost[k] = 0;
  // The queues schedule in this order: the two INT queues first (the FP write-port reservation sees the I2F grants before the FP queue's)
  static constexpr int ORDER[IQ_NUM] = {IQ_INT, IQ_INT1, IQ_MEM, IQ_STD, IQ_BR, IQ_FP};
  uint32_t cn_pregs[LSU_LOAD_WB_WIDTH];
  int n_cn = 0;
  for (int p = 0; p < LSU_LOAD_WB_WIDTH; p++)
    if (in.lsu_awake->cancel[p].valid) cn_pregs[n_cn++] = in.lsu_awake->cancel[p].preg;
  for (int qi : ORDER) {
    auto &q = iqs[qi];
    q.set_cancel(cn_pregs, n_cn);
    q.in.issue_block = in.rob_bcast->flush || in.dec_bcast->mispred;
    for (int p = q.port_lo; p < q.port_hi; p++) q.in.port_fu_ready_mask[p] = in.exe2iss->fu_ready_mask[p];
    q.comb_issue();
    for (int p = q.port_lo; p < q.port_hi; p++) {
      const auto &grant = q.out.issue_grants[p];
      if (!grant.valid) continue;
      if (isutr_on() && grant.uop->rob_idx == 14) std::fprintf(stderr, "ISUTR %llu GRANT port=%d op=%d rob=%d\n", (unsigned long long)g_dbg_cycle, p, (int)grant.uop.op, (int)grant.uop->rob_idx);
      { static const bool tr = std::getenv("ISSUE_TRACE") != nullptr;   // ISSUE_TRACE=1 (+ FPR_DUMP window): every grant (queue, port, rob, pc, op) and every woken physical register
        if (tr && trace_win::ok()) std::fprintf(stderr, "[GR] cyc=%lld q=%d port=%d rob=%d pc=%llx op=%d dest=p%d src1=p%d/%d src2=p%d/%d\n", (long long)sim_time, qi, p, (int)grant.uop->rob_idx, (unsigned long long)grant.uop->pc, (int)grant.uop.op,
                                             (int)grant.uop->dest_preg, (int)grant.uop->src1_preg, (int)grant.uop.src1_busy, (int)grant.uop->src2_preg, (int)grant.uop.src2_busy); }
      Assert(!out.iss2prf.iss_entry[p].valid);
      out.iss2prf.iss_entry[p].valid = true;
      out.iss2prf.iss_entry[p].uop = grant.uop;
    }
  }
}

// Next state of the delayed wakeups (the edge): DIV/FP count down (an entry that fired is dropped), the MUL pipeline shifts,
// and uops issued this cycle with a multi-cycle latency enter them (q: the registers of this cycle).
void Isu::calc_latency_next(const Regs &q) {
  Regs &d = q_;   // updated in place
  auto age = [](auto &cur, auto &nxt) {
    for (size_t i = 0; i < std::size(cur); i++) {
      nxt[i] = {};
      if (cur[i].valid && cur[i].countdown > 0) {
        nxt[i] = cur[i];
        nxt[i].countdown--;
      }
    }
  };
  age(q.fp_wake_slots, d.fp_wake_slots);

  for (auto &stage : d.mul_wake_pipe) for (auto &e : stage) e = {};
  if (MUL_MAX_LATENCY > 1)
    for (int stage = ISU_MUL_WAKE_DEPTH - 1; stage >= 1; stage--)
      for (int s = 0; s < ISU_MUL_WAKE_SLOT_NUM; s++) d.mul_wake_pipe[stage][s] = q.mul_wake_pipe[stage - 1][s];

  for (int i = 0; i < ISSUE_WIDTH; i++) {
    const auto &inst = out.iss2prf.iss_entry[i];
    if (!(inst.valid && inst.uop.dest_en)) continue;
    const UopType op = decode_uop_type(inst.uop.op);
    const int lat = get_latency(op, inst.uop->func7, inst.uop->is_word);
    if (lat <= 1) continue;
    auto put = [&](auto &e, int countdown) {
      e.valid = true;
      e.dest_preg = inst.uop->dest_preg;
      e.rob_idx = inst.uop->rob_idx;
      e.rob_flag = inst.uop->rob_flag;
      if constexpr (requires { e.countdown; }) e.countdown = countdown;
    };
    auto free_slot = [&](auto &slots, int countdown) {
      for (auto &e : slots)
        if (!e.valid) { put(e, countdown); return true; }
      return false;
    };
    bool ok = true;  // the placement must not sit inside Assert (it has to run in every build)
    if (op == UOP_MUL) ok = MUL_MAX_LATENCY > 1 && free_slot(d.mul_wake_pipe[0], 0);
    else if (op == UOP_DIV) ok = true;   // woken dynamically by Exu::ready() one cycle before completion
    else if (op == UOP_FMA || op == UOP_FMISC || op == UOP_FDIV || op == UOP_I2F) ok = free_slot(d.fp_wake_slots, lat - 1);
    Assert(ok && "delayed-wakeup slots full");
  }
}

// Gather every wakeup source onto its own broadcast channel (so write-backs never collide) and wake the queues:
// load write-backs (slow), DIV/FP countdown reaching zero and the last MUL stage (delayed), and uops issued this
// cycle with single-cycle latency (fast, ALU ports only).
void Isu::awake() {
  const Regs &q = q_;
  auto &bus = out.iss_awake.wake;
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

  static const auto fast_ch = [](int next_fast) {   // the fast channel of each ALU port (a constant of the configuration)
    std::array<int, ISSUE_WIDTH> f{};
    for (int i = 0; i < ISSUE_WIDTH; i++) f[i] = (get_port_capability(i) & OP_MASK_ALU) ? next_fast++ : -1;
    return f;
  }(fast_base);

  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++)
    if (in.prf_awake->wake[i].valid) {
      bus[lsu_base + i].valid = true;
      bus[lsu_base + i].preg = in.prf_awake->wake[i].preg;
    }
  if (in.exe2iss->div_wake_valid) drive(div_base, in.exe2iss->div_wake_preg, "Delayed writeback collision");
  {  // the FP slots fire on the FPW_CH channels after the fast ones, in slot order
    int ch = fast_base + ALU_NUM;
    for (int i = 0; i < ISU_FP_WAKE_SLOT_NUM; i++)
      if (q.fp_wake_slots[i].valid && q.fp_wake_slots[i].countdown == 0) {
        Assert(ch < fast_base + ALU_NUM + FPW_CH && "more FP wake-ups in one cycle than channels");
        drive(ch++, q.fp_wake_slots[i].dest_preg, "Delayed writeback collision");
      }
  }
  if (MUL_MAX_LATENCY > 1)
    for (int i = 0; i < ISU_MUL_WAKE_SLOT_NUM; i++)
      if (q.mul_wake_pipe[ISU_MUL_WAKE_DEPTH - 1][i].valid) drive(mul_base + i, q.mul_wake_pipe[ISU_MUL_WAKE_DEPTH - 1][i].dest_preg, "Delayed writeback collision");
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    const auto &e = out.iss2prf.iss_entry[i];
    if (!(e.valid && e.uop.dest_en)) continue;
    const UopType op = decode_uop_type(e.uop.op);
    if (get_latency(op, e.uop->func7, e.uop->is_word) <= 1 && op != UOP_LOAD && op != UOP_STA && fast_ch[i] != -1)
      drive(fast_ch[i], e.uop->dest_preg, "Fast writeback collision");
  }

  {  // LSU channels after the fast ones ([p] unused since the speculative L2 wake-up was removed, [LD + p] L3 register)
    const int early_base = MAX_WAKEUP_PORTS;  // config.h channels first
    for (int i = 0; i < LSU_EARLY_WAKE_PORTS; i++)
      if (in.lsu_awake->wake[i].valid) {
        bus[early_base + i].valid = true;
        bus[early_base + i].preg = in.lsu_awake->wake[i].preg;
      }
  }
  uint32_t woken[WAKE_CH_ALL];
  int n_woken = 0;
  const uint64_t ep = ++out.iss_awake.epoch;
  for (int i = 0; i < WAKE_CH_ALL; i++)
    if (bus[i].valid) {
      woken[n_woken++] = bus[i].preg;
      if (bus[i].preg < TAG_NUM) out.iss_awake.woke_at[bus[i].preg] = ep;
    }
  { static const bool tr = std::getenv("ISSUE_TRACE") != nullptr;
    if (tr && trace_win::ok() && n_woken) { std::fprintf(stderr, "[WK] cyc=%lld p:", (long long)sim_time); for (int i = 0; i < n_woken; i++) std::fprintf(stderr, " %u", woken[i]); std::fprintf(stderr, "\n"); } }
  for (auto &q : iqs) q.set_wakeup(woken, n_woken);   // (applied at the edge)
}

// Flush / mispredict / branch-resolve handling for the queues (their inputs; applied by their seq) and the delayed wakeups. A flush empties everything,
// a mispredict kills the wakeups of squashed uops, and resolved branches free their mask bit in the survivors.
void Isu::flush() {
  const bool flush_all = in.rob_bcast->flush;
  const bool flush_br = !flush_all && in.dec_bcast->mispred;
  const auto r_idx = in.dec_bcast->redirect_rob_idx;
  const auto r_flag = in.dec_bcast->redirect_rob_flag;

  for (auto &q : iqs) {
    q.in.flush_all = flush_all;
    q.in.flush_br = flush_br;
    q.in.flush_rob_idx = r_idx;
    q.in.flush_rob_flag = r_flag;
  }
  if (flush_all) {
    clear_wake_state();
  } else if (in.dec_bcast->mispred) {
    for_each_wake_slot([&](auto &e) {
      if (e.valid && is_younger_rob(e.rob_idx, e.rob_flag, r_idx, r_flag)) e = {};
    });
  }
}

// The clock edge: the delayed wakeups, the dispatch requests into the queues (with this cycle's wake-ups applied), the
// flush / branch clear; then each queue's own edge.
void Isu::seq() {
  const Regs q = q_;   // the wake slots of this cycle
  calc_latency_next(q);
  enq();
  flush();
  for (auto &iq : iqs) iq.seq();
}

// free slots, select and issue, the wakeup bus
void Isu::comb_issue() {
  ready();
  issue();
  awake();
}

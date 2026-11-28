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
  for (auto &e : q_.wb_spec_q) e = {};
  q_.div_busy = 0;
  q_.div_rob_idx = 0;
  q_.div_rob_flag = 0;
  q_.div_spec_mask = 0;
  q_.fdiv_cnt = 0;
  q_.fdiv_rob_idx = 0;
  q_.fdiv_rob_flag = 0;
  q_.fdiv_spec_mask = 0;
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
  div_busy_eff_ = false;
  fdiv_cnt_eff_ = 0;
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
      if (port == 1) {
        if (t == UOP_DIV) return !div_busy_eff_;
        return !in.exe2iss->div_wb_in2;
      }
      if (t == UOP_FDIV && fdiv_cnt_eff_ > 0) return false;
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
      slot.spec_t1 = 0;
      if (const uint8_t m = in.lsu_awake->spec_l1_mask) {
        const auto &u = slot.uop;
        for (int k = 0; k < LSU_LOAD_WB_WIDTH; k++) {
          if (!(m & (1u << k))) continue;
          const uint32_t wp = in.lsu_awake->wake[k].preg;
          if ((u->src1_en && u->src1_preg == wp) ||
              (u->src2_en && u->src2_preg == wp) ||
              (u->src3_en && u->src3_preg == wp)) {
            slot.spec_t1 |= static_cast<uint8_t>(1u << k);
          }
        }
      }
      if (isutr_on() && slot.uop->rob_idx == 14) std::fprintf(stderr, "ISUTR %llu ENQ q=%d op=%d rob=%d src1=%d(en%d busy%d) src2=%d(en%d busy%d) dest=%d\n", (unsigned long long)g_dbg_cycle, i, (int)slot.uop.op, (int)slot.uop->rob_idx, (int)slot.uop->src1_preg, (int)slot.uop->src1_en, (int)slot.uop.src1_busy, (int)slot.uop->src2_preg, (int)slot.uop->src2_en, (int)slot.uop.src2_busy, (int)slot.uop->dest_preg);
      slot.valid = true;
    }
  }
}

// Extra grant conditions of a pick: its completion cycle on the port is free (EXP-035), and the FP register-file write cycle of an
// FP-writing unit (EXP-032). The reservations are made here, only for a grant that is about to be made.
int Isu::grant_gate(const Uop &u, int port, bool reserve) {
  const UopType t = decode_uop_type(u.op);
  const int lat = get_latency(t, u->func7, u->is_word);
  const int off = (t == UOP_FDIV) ? lat + 1 : lat;   // iterative FP units finish one cycle later than their wake-up latency
  if (t != UOP_DIV && !wb_busy_.free(port, off)) return GATE_REF_WB;   // another uop of the port completes in that cycle
  const bool fpw = u.dest_en && u->dest_fp && (t == UOP_FMA || t == UOP_FMISC || t == UOP_FDIV || t == UOP_I2F);
  if (!reserve) return fpw && fp_wb_busy_.full(off) ? GATE_REF_FPWB : GATE_OK;
  if (fpw && !fp_wb_busy_.try_reserve(off)) return GATE_REF_FPWB;
  if (t != UOP_DIV) wb_busy_.reserve(port, off);
  return GATE_OK;
}

// Every queue schedules; a grant needs the port's FU capability. No issue during a flush or mispredict.
void Isu::issue() {
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    out.iss2prf.iss_entry[i].valid = false;
    out.iss2prf.iss_entry[i].spec_mask = 0;
  }
  for (int a = 0; a < ALU_NUM; a++) {
    out.iss_awake.cancel[a].valid = false;
    out.iss_awake.cancel[a].preg = 0;
  }
  wb_busy_.new_cycle(in.rob_bcast->flush);
  fp_wb_busy_.new_cycle(in.rob_bcast->flush);
  for (int k = 0; k < 4; k++) g_pmu_back.iss_ref[k] = g_pmu_back.iss_lost[k] = 0;
  // The queues schedule in this order: the two INT queues first (the FP write-port reservation sees the I2F grants before the FP queue's)
  static constexpr int ORDER[IQ_NUM] = {IQ_INT, IQ_INT1, IQ_MEM, IQ_STD, IQ_BR, IQ_FP};
  uint32_t cn_pregs[LSU_LOAD_WB_WIDTH + ALU_NUM];
  int n_cn = 0;
  uint8_t cn_port_mask = 0;
  for (int p = 0; p < LSU_LOAD_WB_WIDTH; p++)
    if (in.lsu_awake->cancel[p].valid) {
      cn_pregs[n_cn++] = in.lsu_awake->cancel[p].preg;
      cn_port_mask |= static_cast<uint8_t>(1u << p);
    }
  div_busy_eff_ = q_.div_busy && !((q_.div_spec_mask & cn_port_mask) != 0);
  fdiv_cnt_eff_ = (q_.fdiv_cnt > 0 && (q_.fdiv_spec_mask & cn_port_mask) != 0) ? 0 : q_.fdiv_cnt;
  if (cn_port_mask != 0) {
    for (int a = 0; a < ALU_NUM; a++) {
      const auto &fs = q_.fast_spec_q[a];
      if (!fs.valid || !(fs.spec_mask & cn_port_mask)) continue;
      if (in.rob_bcast->flush ||
          (in.dec_bcast->mispred && is_younger_rob(fs.rob_idx, fs.rob_flag, in.dec_bcast->redirect_rob_idx, in.dec_bcast->redirect_rob_flag)))
        continue;
      cn_pregs[n_cn++] = fs.dest_preg;
      out.iss_awake.cancel[a].valid = true;
      out.iss_awake.cancel[a].preg = fs.dest_preg;
    }
    if (!in.rob_bcast->flush) {
      for (int p = 0; p < ISSUE_WIDTH; p++) {
        const auto &w = q_.wb_spec_q[p];
        if (!w.valid || !(w.spec_mask & cn_port_mask)) continue;
        if (w.has_wb) wb_busy_.busy[p][w.wb_slot] = 0;
        if (w.has_fpwb) {
          Assert(fp_wb_busy_.busy[w.wb_slot] > 0);
          fp_wb_busy_.busy[w.wb_slot]--;
        }
      }
    }
  }
  for (int qi : ORDER) {
    auto &q = iqs[qi];
    q.set_cancel(cn_pregs, n_cn, cn_port_mask);
    q.in.issue_block = in.rob_bcast->flush || in.dec_bcast->mispred;
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
      out.iss2prf.iss_entry[p].spec_mask = grant.spec_t1;
    }
  }
}

// Next state of the delayed wakeups (the edge): DIV/FP count down (cancelled/fired entries are dropped), the MUL pipeline shifts,
// and uops issued this cycle with a multi-cycle latency or unverified L1-spec wakeup enter tracking (q: the registers of this cycle).
void Isu::calc_latency_next(const Regs &q) {
  Regs &d = q_;   // updated in place
  uint8_t cn_port_mask = 0;
  for (int p = 0; p < LSU_LOAD_WB_WIDTH; p++)
    if (in.lsu_awake->cancel[p].valid) cn_port_mask |= static_cast<uint8_t>(1u << p);

  const auto &g1 = out.iss2prf.iss_entry[1];
  const bool div_iss = g1.valid && decode_uop_type(g1.uop.op) == UOP_DIV;
  Assert(!(div_busy_eff_ && div_iss));
  if (div_iss) {
    d.div_busy = 1;
    d.div_rob_idx = g1.uop->rob_idx;
    d.div_rob_flag = g1.uop->rob_flag;
    d.div_spec_mask = g1.spec_mask;
  } else if (div_busy_eff_ && !in.exe2iss->div_wb_in2) {
    d.div_busy = 1;
    d.div_rob_idx = q.div_rob_idx;
    d.div_rob_flag = q.div_rob_flag;
    d.div_spec_mask = 0;
  } else {
    d.div_busy = 0;
    d.div_rob_idx = 0;
    d.div_rob_flag = 0;
    d.div_spec_mask = 0;
  }
  const int fp_last = ISSUE_WIDTH - 1;
  const auto &gf = out.iss2prf.iss_entry[fp_last];
  const bool fdiv_iss = gf.valid && decode_uop_type(gf.uop.op) == UOP_FDIV;
  Assert(!(fdiv_cnt_eff_ > 0 && fdiv_iss));
  if (fdiv_iss) {
    d.fdiv_cnt = get_latency(UOP_FDIV, gf.uop->func7, gf.uop->is_word);
    d.fdiv_rob_idx = gf.uop->rob_idx;
    d.fdiv_rob_flag = gf.uop->rob_flag;
    d.fdiv_spec_mask = gf.spec_mask;
  } else if (fdiv_cnt_eff_ > 1) {
    d.fdiv_cnt = fdiv_cnt_eff_ - 1;
    d.fdiv_rob_idx = q.fdiv_rob_idx;
    d.fdiv_rob_flag = q.fdiv_rob_flag;
    d.fdiv_spec_mask = 0;
  } else {
    d.fdiv_cnt = 0;
    d.fdiv_rob_idx = 0;
    d.fdiv_rob_flag = 0;
    d.fdiv_spec_mask = 0;
  }

  for (size_t i = 0; i < std::size(q.fp_wake_slots); i++) {
    d.fp_wake_slots[i] = {};
    if (q.fp_wake_slots[i].valid && q.fp_wake_slots[i].countdown > 0 &&
        !(q.fp_wake_slots[i].spec_mask & cn_port_mask)) {
      d.fp_wake_slots[i] = q.fp_wake_slots[i];
      d.fp_wake_slots[i].countdown--;
      d.fp_wake_slots[i].spec_mask = 0;
    }
  }

  for (auto &e : d.fast_spec_q) e = {};
  for (auto &e : d.wb_spec_q) e = {};

  for (auto &stage : d.mul_wake_pipe) for (auto &e : stage) e = {};
  if (MUL_MAX_LATENCY > 1)
    for (int stage = ISU_MUL_WAKE_DEPTH - 1; stage >= 1; stage--)
      for (int s = 0; s < ISU_MUL_WAKE_SLOT_NUM; s++) {
        const auto &src = q.mul_wake_pipe[stage - 1][s];
        if (src.valid && !(src.spec_mask & cn_port_mask)) {
          d.mul_wake_pipe[stage][s] = src;
          d.mul_wake_pipe[stage][s].spec_mask = 0;
        }
      }

  int alu_k = 0;
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    const bool is_alu_port = (get_port_capability(i) & OP_MASK_ALU) != 0;
    const int my_alu_k = is_alu_port ? alu_k++ : -1;
    const auto &inst = out.iss2prf.iss_entry[i];
    if (!inst.valid) continue;
    const UopType op = decode_uop_type(inst.uop.op);
    const int lat = get_latency(op, inst.uop->func7, inst.uop->is_word);
    if (inst.spec_mask != 0) {
      const int off = (op == UOP_FDIV) ? lat + 1 : lat;
      auto &w = d.wb_spec_q[i];
      w.valid = true;
      w.spec_mask = inst.spec_mask;
      w.wb_slot = static_cast<uint8_t>((sim_time + off) & (WbBusy::kRing - 1));
      w.has_wb = (op != UOP_DIV);
      w.has_fpwb = inst.uop.dest_en && inst.uop->dest_fp &&
                   (op == UOP_FMA || op == UOP_FMISC || op == UOP_FDIV || op == UOP_I2F);
    }
    if (!inst.uop.dest_en) continue;
    auto put = [&](auto &e, int countdown) {
      e.valid = true;
      e.spec_mask = inst.spec_mask;
      e.dest_preg = inst.uop->dest_preg;
      e.rob_idx = inst.uop->rob_idx;
      e.rob_flag = inst.uop->rob_flag;
      if constexpr (requires { e.countdown; }) e.countdown = countdown;
    };
    if (lat <= 1) {
      if (inst.spec_mask != 0 && op != UOP_LOAD && op != UOP_STA && my_alu_k >= 0)
        put(d.fast_spec_q[my_alu_k], 0);
      continue;
    }
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
  uint8_t cn_port_mask = 0;
  for (int p = 0; p < LSU_LOAD_WB_WIDTH; p++)
    if (in.lsu_awake->cancel[p].valid) cn_port_mask |= static_cast<uint8_t>(1u << p);
  {  // the FP slots fire on the FPW_CH channels after the fast ones, in slot order
    int ch = fast_base + ALU_NUM;
    for (int i = 0; i < ISU_FP_WAKE_SLOT_NUM; i++)
      if (q.fp_wake_slots[i].valid && q.fp_wake_slots[i].countdown == 0 &&
          !(q.fp_wake_slots[i].spec_mask & cn_port_mask)) {
        Assert(ch < fast_base + ALU_NUM + FPW_CH && "more FP wake-ups in one cycle than channels");
        drive(ch++, q.fp_wake_slots[i].dest_preg, "Delayed writeback collision");
      }
  }
  if (MUL_MAX_LATENCY > 1)
    for (int i = 0; i < ISU_MUL_WAKE_SLOT_NUM; i++) {
      const auto &me = q.mul_wake_pipe[ISU_MUL_WAKE_DEPTH - 1][i];
      if (me.valid && !(me.spec_mask & cn_port_mask))
        drive(mul_base + i, me.dest_preg, "Delayed writeback collision");
    }
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    const auto &e = out.iss2prf.iss_entry[i];
    if (!(e.valid && e.uop.dest_en)) continue;
    const UopType op = decode_uop_type(e.uop.op);
    if (get_latency(op, e.uop->func7, e.uop->is_word) <= 1 && op != UOP_LOAD && op != UOP_STA && fast_ch[i] != -1)
      drive(fast_ch[i], e.uop->dest_preg, "Fast writeback collision");
  }

  {  // LSU channels after the fast ones ([p] L1 spec, [LD+p] L3, [2*LD+p] L2b STLF / L2a fallback, [3*LD..+1] V2 WBP)
    const int early_base = MAX_WAKEUP_PORTS;  // config.h channels first
    for (int i = 0; i < LSU_EARLY_WAKE_PORTS; i++)
      if (in.lsu_awake->wake[i].valid) {
        bus[early_base + i].valid = true;
        bus[early_base + i].preg = in.lsu_awake->wake[i].preg;
      }
  }
  uint32_t woken[WAKE_CH_ALL];
  uint8_t woken_spec[WAKE_CH_ALL];
  int n_woken = 0;
  const uint64_t ep = ++out.iss_awake.epoch;
  for (int i = 0; i < WAKE_CH_ALL; i++)
    if (bus[i].valid) {
      uint8_t sm = 0;
      if (i >= MAX_WAKEUP_PORTS && i < MAX_WAKEUP_PORTS + LSU_LOAD_WB_WIDTH) {
        const int lp = i - MAX_WAKEUP_PORTS;
        if (in.lsu_awake->spec_l1_mask & (1u << lp)) sm = static_cast<uint8_t>(1u << lp);
      }
      woken[n_woken] = bus[i].preg;
      woken_spec[n_woken++] = sm;
      if (bus[i].preg < TAG_NUM) out.iss_awake.woke_at[bus[i].preg] = ep;
    }
  { static const bool tr = std::getenv("ISSUE_TRACE") != nullptr;
    if (tr && trace_win::ok() && n_woken) { std::fprintf(stderr, "[WK] cyc=%lld p:", (long long)sim_time); for (int i = 0; i < n_woken; i++) std::fprintf(stderr, " %u", woken[i]); std::fprintf(stderr, "\n"); } }
  for (auto &q : iqs) q.set_wakeup(woken, n_woken, woken_spec);   // (applied at the edge)
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
    if (q_.div_busy && is_younger_rob(q_.div_rob_idx, q_.div_rob_flag, r_idx, r_flag)) {
      q_.div_busy = 0;
      q_.div_rob_idx = 0;
      q_.div_rob_flag = 0;
      q_.div_spec_mask = 0;
    }
    if (q_.fdiv_cnt > 0 && is_younger_rob(q_.fdiv_rob_idx, q_.fdiv_rob_flag, r_idx, r_flag)) {
      q_.fdiv_cnt = 0;
      q_.fdiv_rob_idx = 0;
      q_.fdiv_rob_flag = 0;
      q_.fdiv_spec_mask = 0;
    }
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

#include "BeginCheck.h"
#include "Exu.h"
#include "Wires.h"
#include "config.h"
#include "trace_hook.h"
#include "FpStats.h"

namespace {
// A uop whose branch was squashed by this cycle's mispredict is dead.
inline bool is_br_killed(const ExuInst &uop, const DecBroadcastIO &db) { return db.mispred && (uop.br_mask & db.br_mask) != 0; }
}  // namespace

Exu::Exu(SimContext *ctx) : ctx(ctx) {}

// Functional units are created per issue port, in write-back priority order (MUL first).
void Exu::init() {
  struct Spec { FuKind kind; uint64_t mask; };
  static const Spec kSpecs[] = {{FuKind::MUL, OP_MASK_MUL}, {FuKind::DIV, OP_MASK_DIV}, {FuKind::AGU, OP_MASK_LD},
                                {FuKind::AGU, OP_MASK_STA}, {FuKind::SDU, OP_MASK_STD}, {FuKind::BRU, OP_MASK_BR},
                                {FuKind::I2F, OP_MASK_I2F},   // integer -> FP: ahead of the ALU, which would otherwise starve it (write-back priority = this order)
                                {FuKind::ALU, OP_MASK_ALU}, {FuKind::CSR, OP_MASK_CSR},
                                {FuKind::FMA, OP_MASK_FMA}, {FuKind::FDIV, OP_MASK_FDIV}, {FuKind::FMISC, OP_MASK_FMISC}};
  int agu = 0, sdu = 0;
  units.clear();
  for (auto &m : port_fu_mask) m = 0;
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    port_fus[i].clear();
    for (const Spec &spec : kSpecs) {
      if (!(GLOBAL_ISSUE_PORT_CONFIG[i].support_mask & spec.mask)) continue;
      FuUnit u;
      u.kind = spec.kind;
      u.mode = fu_mode(spec.kind);
      u.port = i;
      u.support_mask = spec.mask;
      if (spec.kind == FuKind::AGU) u.lsu_agu_port = agu++;
      if (spec.kind == FuKind::SDU) u.lsu_sdu_port = sdu++;
      if (u.mode == FuMode::PIPE) {
        u.depth = fu_pipe_latency(spec.kind);
        u.ring.resize(u.depth);
      }
      port_fus[i].push_back(static_cast<int>(units.size()));
      port_fu_mask[i] |= 1ULL << units.size();
      units.push_back(std::move(u));
    }
  }
  Assert(static_cast<int>(units.size()) <= TOTAL_FU_COUNT && TOTAL_FU_COUNT <= 64);
}

// Default (idle) outputs.
void Exu::comb_begin() {
  Wires &w = *wires;
  w.exe2csr.we = w.exe2csr.re = false;
  w.exe2csr.idx = 0;
  w.exe2csr.wcmd = 0;
  w.exe2csr.wdata = 0;
  for (int i = 0; i < TOTAL_FU_COUNT; i++) w.exe2prf.bypass[i].valid = false;
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    w.exe2prf.entry[i].valid = false;
    w.exu2rob.entry[i].valid = false;
    w.exe2iss.fu_ready_mask[i] = 0;
  }
  for (int i = 0; i < LSU_AGU_COUNT; i++) w.exe2lsu.agu_req[i].valid = false;
  for (int i = 0; i < LSU_SDU_COUNT; i++) w.exe2lsu.sdu_req[i].valid = false;
  w.exu2id.clear_mask = 0;
  w.exu2id.mispred = false;
  w.exu2id.redirect_pc = 0;
  w.exu2id.redirect_rob_idx = 0;
  w.exu2id.br_id = 0;
  w.exu2id.ftq_idx = 0;
  w.exu2id.ftq_off = 0;
  w.exu2id.br_type = 0;
}

// Which operation classes each issue port can accept. After the collect phase an input slot is occupied only if its unit
// could not take the uop, so a free slot is what makes a unit ready. Nothing is accepted during a flush.
void Exu::comb_ready() {
  Wires &w = *wires;
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    uint64_t mask = 0;
    if (!w.rob_bcast.flush)
      for (int fu : port_fus[i])
        if (!units[fu].slot_valid) mask |= units[fu].support_mask;
    w.exe2iss.fu_ready_mask[i] = mask;
  }
}

// The CSR uop sitting in port 0's FU slot drives the CSR file's read/write request.
void Exu::comb_to_csr() {
  Wires &w = *wires;
  if (w.rob_bcast.flush) return;
  for (int fu : port_fus[0]) {
    const FuUnit &f = units[fu];
    if (!f.slot_valid) continue;
    const ExuInst &u = f.slot;
    if (u.op != UOP_CSR || is_br_killed(u, w.dec_bcast)) continue;

    const bool rw_family = u->func3 == 0b001 || u->func3 == 0b101;  // CSRRW / CSRRWI always write
    w.exe2csr.we = rw_family || u->imm != 0;
    w.exe2csr.re = !rw_family || u.dest_preg != 0;
    w.exe2csr.idx = u->csr_idx;
    w.exe2csr.wcmd = u->func3 & 0b11;
    w.exe2csr.wdata = 0;  // the written value is a fact of the trace (rs1 or zimm)
    if (const tb::Fact *f2 = tbhook::fact_by_rob(u.rob_idx)) w.exe2csr.wdata = f2->csr_wdata;
    return;
  }
}

// Issued uops enter the FU input slots (killed ones are dropped, resolved branch bits are cleared); a flush empties them.
void Exu::comb_pipeline() {
  Wires &w = *wires;
  if (w.rob_bcast.flush) {
    for (auto &f : units) f.slot_valid = false;
    return;
  }
  const auto clear = w.dec_bcast.clear_mask;
  for (auto &f : units) {
    if (!f.slot_valid) continue;
    if (is_br_killed(f.slot, w.dec_bcast)) f.slot_valid = false;
    else if (clear) f.slot.br_mask &= ~clear;
  }
  for (int p = 0; p < ISSUE_WIDTH; p++) {
    if (!w.prf2exe.iss_entry[p].valid) continue;
    ExuInst uop = w.prf2exe.iss_entry[p].uop;
    if (is_br_killed(uop, w.dec_bcast)) continue;
    if (clear) uop.br_mask &= ~clear;
    const uint64_t bit = 1ULL << uop.op;
    int target = -1;  // first FU of the port that supports the op and has a free input slot
    for (int fu : port_fus[p])
      if ((units[fu].support_mask & bit) && !units[fu].slot_valid) { target = fu; break; }
    Assert(target >= 0 && "EXU enqueue without free FU input slot");
    units[target].slot_valid = true;
    units[target].slot = uop;
  }
}

// FUs take their input slots and evaluate (units that hold nothing are skipped); then results are collected. Every completing
// FU is broadcast for bypass; each port's first completing FU wins the port and, unless flushed/killed, completes in the ROB,
// sends its LSU request, writes back (ALU ports) or takes part in the branch arbitration (the oldest mispredicted branch
// redirects). LSU write-backs come in on their own ports.
void Exu::comb_exec() {
  Wires &w = *wires;
  const bool flush_all = w.rob_bcast.flush, flush_br = !flush_all && w.dec_bcast.mispred;
  const bool flush_any = flush_all || flush_br, flush = flush_all;  // (results are dead only on a global flush or when killed by the branch mask)
  const wire<BR_MASK_WIDTH> all = ~wire<BR_MASK_WIDTH>(0);
  const wire<BR_MASK_WIDTH> flush_mask = flush_all ? all : flush_br ? w.dec_bcast.br_mask : wire<BR_MASK_WIDTH>(0);
  const wire<BR_MASK_WIDTH> clear = w.dec_bcast.clear_mask;
  const int64_t now = sim_time;

  uint64_t cmask = 0;  // completing units
  ExuInst *cinst[64];
  for (size_t i = 0; i < units.size(); i++) {
    FuUnit &f = units[i];
    if (!f.slot_valid && f.idle()) continue;

    // flush / branch-mask clear applied to the instructions held in the unit
    if (f.mode == FuMode::PIPE) {
      if (flush_any && f.count) {
        if (flush_mask == all) {
          f.count = 0;
        } else {
          int m = 0;
          for (int k = 0; k < f.count; k++) {
            const int s = f.head + k >= f.depth ? f.head + k - f.depth : f.head + k;
            if (f.ring[s].uop.br_mask & flush_mask) continue;
            if (m != k) f.ring[f.head + m >= f.depth ? f.head + m - f.depth : f.head + m] = f.ring[s];
            m++;
          }
          f.count = m;
        }
      }
      if (clear)
        for (int k = 0; k < f.count; k++) f.ring[f.head + k >= f.depth ? f.head + k - f.depth : f.head + k].uop.br_mask &= ~clear;
    } else if (f.mode == FuMode::ITER) {
      if (flush_any && f.busy && (flush_mask == all || (f.cur.br_mask & flush_mask) != 0)) {
        f.busy = false;
        f.done = 0;
      }
      if (clear && f.busy) f.cur.br_mask &= ~clear;
    }

    // the input slot goes to the unit unless flushed or killed (that frees it)
    bool en = false;
    if (f.slot_valid) {
      if (flush_all || is_br_killed(f.slot, w.dec_bcast)) f.slot_valid = false;
      else en = true;
    }

    switch (f.mode) {
    case FuMode::SINGLE:
      if (en) {
        f.out = f.slot;
        if (clear) f.out.br_mask &= ~clear;
        fu::compute(f.kind, f.out);
        cinst[i] = &f.out;
        cmask |= 1ULL << i;
      }
      break;
    case FuMode::PIPE:
      if (en && f.count < f.depth) {
        FuUnit::PipeEntry e;
        e.uop = f.slot;
        if (clear) e.uop.br_mask &= ~clear;
        fu::compute(f.kind, e.uop);
        e.done_cycle = now + f.depth - 1;
        f.push_back(e);
      }
      if (f.count && f.front().done_cycle <= now) {
        cinst[i] = &f.front().uop;
        cmask |= 1ULL << i;
      }
      break;
    case FuMode::ITER:
      if (en && !f.busy) {
        f.cur = f.slot;
        if (clear) f.cur.br_mask &= ~clear;
        fu::compute(f.kind, f.cur);
        f.done = now + fu::iter_latency(f.kind, f.cur);
        f.busy = true;
      }
      if (f.busy && f.done <= now) {
        cinst[i] = &f.cur;
        cmask |= 1ULL << i;
      }
      break;
    }
  }

  for (uint64_t m = cmask; m; m &= m - 1) {
    const int i = __builtin_ctzll(m);
    w.exe2prf.bypass[i].uop = *cinst[i];
    w.exe2prf.bypass[i].valid = true;
  }

  ExuEntry int_res[ALU_NUM] = {}, br_res[BRU_NUM] = {};
  uint64_t consumed = 0;
  for (int p = 0; p < ISSUE_WIDTH && cmask; p++) {
    const uint64_t pm = cmask & port_fu_mask[p];
    if (!pm) continue;
    const int fu = __builtin_ctzll(pm);  // first completing unit in write-back priority order
    FuUnit &winner = units[fu];
    ExuInst u = *cinst[fu];
    const bool dead = flush || is_br_killed(u, w.dec_bcast);

    if (u.op == UOP_CSR && w.exe2csr.re) u.result = w.csr2exe.rdata;
    if (!dead) {
      if (u.op != UOP_LOAD && u.op != UOP_STA) {  // loads/stores complete when the LSU writes back
        w.exu2rob.entry[p].valid = true;
        w.exu2rob.entry[p].uop = u;
      }
      if (u.op == UOP_STA || u.op == UOP_LOAD) {
        Assert(winner.lsu_agu_port >= 0 && winner.lsu_agu_port < LSU_AGU_COUNT);
        w.exe2lsu.agu_req[winner.lsu_agu_port].valid = true;
        w.exe2lsu.agu_req[winner.lsu_agu_port].uop = u;
      } else if (u.op == UOP_STD) {
        Assert(winner.lsu_sdu_port >= 0 && winner.lsu_sdu_port < LSU_SDU_COUNT);
        w.exe2lsu.sdu_req[winner.lsu_sdu_port].valid = true;
        w.exe2lsu.sdu_req[winner.lsu_sdu_port].uop = u;
      }
    }
    if (p >= IQ_ALU_PORT_BASE && p < IQ_ALU_PORT_BASE + ALU_NUM && u.dest_en) {
      int_res[p - IQ_ALU_PORT_BASE].uop = u;
      int_res[p - IQ_ALU_PORT_BASE].valid = true;
    }
    if (!dead && p >= IQ_BR_PORT_BASE && p < IQ_BR_PORT_BASE + BRU_NUM) {  // a squashed branch must not drive recovery
      br_res[p - IQ_BR_PORT_BASE].uop = u;
      br_res[p - IQ_BR_PORT_BASE].valid = true;
    }
    consumed |= 1ULL << fu;
  }

  // units pop what was consumed; a unit that can take more frees its input slot
  for (uint64_t m = consumed; m; m &= m - 1) {
    FuUnit &f = units[__builtin_ctzll(m)];
    if (f.mode == FuMode::PIPE) f.pop_front();
    else if (f.mode == FuMode::ITER) { f.busy = false; f.done = 0; }
  }
  for (auto &f : units) {
    if (!f.slot_valid) continue;
    const int i = static_cast<int>(&f - units.data());
    const bool ready = f.mode == FuMode::SINGLE ? ((consumed >> i) & 1) : f.mode == FuMode::PIPE ? f.count < f.depth : !f.busy;
    if (ready) f.slot_valid = false;
  }

  for (int i = 0; i < ALU_NUM; i++)
    if (int_res[i].valid) {
      w.exe2prf.entry[IQ_ALU_PORT_BASE + i].valid = true;
      w.exe2prf.entry[IQ_ALU_PORT_BASE + i].uop = int_res[i].uop;
    }
  // LSU write-backs feed both the register file and the ROB
  auto lsu_writeback = [&](int port, const Uop &u) {
    Assert(!w.exe2prf.entry[port].valid);
    w.exe2prf.entry[port].valid = true;
    w.exe2prf.entry[port].uop = u;
    if (!(flush || is_br_killed(u, w.dec_bcast))) {
      w.exu2rob.entry[port].valid = true;
      w.exu2rob.entry[port].uop = u;
    }
  };
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++)
    if (w.lsu2exe.wb_req[i].valid) lsu_writeback(IQ_LD_PORT_BASE + i, w.lsu2exe.wb_req[i].uop);
#if FP
  {  // FP load write-backs per cycle (the staged LSU's write-back uop carries only the tag: an FP destination has a tag >= FP_TAG_BASE)
    int n = 0;
    for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++)
      n += w.lsu2exe.wb_req[i].valid && w.lsu2exe.wb_req[i].uop.dest_en && w.lsu2exe.wb_req[i].uop.dest_preg >= FP_TAG_BASE;
    FpStats::bump(g_fp_stats.ld_hist, n);
  }
#endif
  for (int i = 0; i < LSU_STA_COUNT; i++)
    if (w.lsu2exe.sta_wb_req[i].valid) lsu_writeback(IQ_STA_PORT_BASE + i, w.lsu2exe.sta_wb_req[i].uop);

  // branch resolution: every resolved branch frees its mask bit, the oldest mispredict redirects
  bool mispred = false;
  const ExuInst *oldest = nullptr;
  wire<BR_MASK_WIDTH> clear_mask = 0;
  for (int i = 0; i < BRU_NUM; i++) {
    if (!br_res[i].valid) continue;
    clear_mask |= wire<BR_MASK_WIDTH>(1) << br_res[i].uop->br_id;
    if (br_res[i].uop.mispred && (!mispred || cmp_inst_age(*oldest, br_res[i].uop))) {
      mispred = true;
      oldest = &br_res[i].uop;
    }
  }

  if (tbhook::enabled)
    for (int p = 0; p < ISSUE_WIDTH; p++) {
      if (!w.exu2rob.entry[p].valid) continue;
      const auto &u = w.exu2rob.entry[p].uop;
      tbhook::ExecObs o;
      o.rob_idx = u.rob_idx;
      o.uid_dbg = (uint64_t)u->dbg.inst_idx;
      o.op = (int)decode_uop_type(u.op);
      o.result = u.result;
      o.diag_val = u.diag_val;
      o.br_taken = u.br_taken;
      o.mispred = u.mispred;
      o.page_fault = u.page_fault_inst || u.page_fault_load || u.page_fault_store;
      tbhook::on_exec(o);
    }

  w.exu2id.clear_mask = clear_mask;  // broadcast to the other modules by the IDU, not applied locally
  w.exu2id.mispred = mispred;
  w.exu2id.redirect_pc = mispred ? oldest->diag_val : 0;
  w.exu2id.redirect_rob_idx = mispred ? oldest->rob_idx : 0;
  w.exu2id.br_id = mispred ? (*oldest)->br_id : 0;
  w.exu2id.ftq_idx = mispred ? (*oldest)->ftq_idx : 0;
  w.exu2id.ftq_off = mispred ? (*oldest)->ftq_offset : 0;
  if (mispred) {   // type of the mispredicted branch in the BPU's encoding (0 cond, 1 call, 2 ret, 3 indirect, 5 jal, 6 indirect call)
    const auto &u = *oldest;
    int t = 0;
    if (u->type == JAL) t = (oldest->dest_en && u->dest_areg == 1) ? 1 : 5;
    else if (u->type == JALR) t = (u->src1_areg == 1 && u->dest_areg == 0 && u->imm == 0) ? 2 : (u->dest_areg == 1) ? 6 : 3;
    w.exu2id.br_type = t;
  }
}


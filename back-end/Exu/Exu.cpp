#include "BeginCheck.h"
#include "Exu.h"
#include "Wires.h"
#include "config.h"
#include "trace_hook.h"

namespace {
// A uop whose branch was squashed by this cycle's mispredict is dead.
inline bool is_br_killed(const ExuInst &uop, const DecBroadcastIO &db) { return db.mispred && (uop.br_mask & db.br_mask) != 0; }
}  // namespace

Exu::Exu(SimContext *ctx) : ctx(ctx) {}

Exu::~Exu() {
  for (auto fu : units) delete fu;
}

// Functional units are created per issue port, in write-back priority order (MUL first).
void Exu::init() {
  struct Spec { FuKind kind; uint64_t mask; };
  static const Spec kSpecs[] = {{FuKind::MUL, OP_MASK_MUL}, {FuKind::DIV, OP_MASK_DIV}, {FuKind::AGU, OP_MASK_LD},
                                {FuKind::AGU, OP_MASK_STA}, {FuKind::SDU, OP_MASK_STD}, {FuKind::BRU, OP_MASK_BR},
                                {FuKind::ALU, OP_MASK_ALU}, {FuKind::CSR, OP_MASK_CSR}, {FuKind::FPU, OP_MASK_FP}};
  int agu = 0, sdu = 0;
  port_mappings.resize(ISSUE_WIDTH);
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    for (const Spec &spec : kSpecs) {
      if (!(GLOBAL_ISSUE_PORT_CONFIG[i].support_mask & spec.mask)) continue;
      FuEntry e;
      e.fu = new Fu(spec.kind, i);
      e.fu_idx = static_cast<int>(units.size());
      e.support_mask = spec.mask;
      if (spec.kind == FuKind::AGU) e.lsu_agu_port = agu++;
      if (spec.kind == FuKind::SDU) e.lsu_sdu_port = sdu++;
      units.push_back(e.fu);
      port_mappings[i].entries.push_back(e);
    }
  }
  for (int i = 0; i < TOTAL_FU_COUNT; i++) {
    fu_inst_r[i] = fu_inst_r_1[i] = {};
    fu_to_port[i] = -1;
  }
  for (int i = 0; i < ISSUE_WIDTH; i++)
    for (auto &e : port_mappings[i].entries) fu_to_port[e.fu_idx] = i;
}

// Default (idle) outputs and FU request buses; each FU starts its cycle and publishes its state.
void Exu::comb_begin() {
  Wires &w = *wires;
  BEGIN_EQ(fu_inst_r_1, fu_inst_r);

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
  for (auto &e : exu2fu_io.entry) {
    e.en = e.consume = e.flush = false;
    e.flush_mask = e.clear_mask = 0;
  }
  for (size_t i = 0; i < units.size(); i++) {
    units[i]->comb_begin();
    fu2exu_io.entry[i] = units[i]->out;
  }
}

// Which operation classes each issue port can accept: an FU takes a new uop when its input slot is free or the FU frees it.
// Nothing is accepted during a flush.
void Exu::comb_ready() {
  Wires &w = *wires;
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    uint64_t mask = 0;
    if (!w.rob_bcast.flush)
      for (auto &e : port_mappings[i].entries)
        if (!fu_inst_r_1[e.fu_idx].valid || fu2exu_io.entry[e.fu_idx].ready) mask |= e.support_mask;
    w.exe2iss.fu_ready_mask[i] = mask;
  }
}

// The CSR uop sitting in port 0's FU slot drives the CSR file's read/write request.
void Exu::comb_to_csr() {
  Wires &w = *wires;
  if (w.rob_bcast.flush) return;
  for (int fu = 0; fu < TOTAL_FU_COUNT; fu++) {
    if (!fu_inst_r[fu].valid || fu_to_port[fu] != 0) continue;
    const ExuInst &u = fu_inst_r[fu].uop;
    if (u.op != UOP_CSR || is_br_killed(u, w.dec_bcast)) continue;

    const bool rw_family = u->func3 == 0b001 || u->func3 == 0b101;  // CSRRW / CSRRWI always write
    w.exe2csr.we = rw_family || u->imm != 0;
    w.exe2csr.re = !rw_family || u.dest_preg != 0;
    w.exe2csr.idx = u->csr_idx;
    w.exe2csr.wcmd = u->func3 & 0b11;
    w.exe2csr.wdata = 0;  // the written value is a fact of the trace (rs1 or zimm)
    if (const tb::Fact *f = tbhook::fact_by_rob(u.rob_idx)) w.exe2csr.wdata = f->csr_wdata;
    return;
  }
}

// Issued uops enter the FU input slots (killed ones are dropped, resolved branch bits are cleared); a flush empties them.
void Exu::comb_pipeline() {
  Wires &w = *wires;
  if (w.rob_bcast.flush) {
    for (auto &s : fu_inst_r_1) s.valid = false;
    return;
  }
  const auto clear = w.dec_bcast.clear_mask;
  for (auto &s : fu_inst_r_1) {
    if (!s.valid) continue;
    if (is_br_killed(s.uop, w.dec_bcast)) s.valid = false;
    else if (clear) s.uop.br_mask &= ~clear;
  }
  for (int p = 0; p < ISSUE_WIDTH; p++) {
    if (!w.prf2exe.iss_entry[p].valid) continue;
    ExuInst uop = w.prf2exe.iss_entry[p].uop;
    if (is_br_killed(uop, w.dec_bcast)) continue;
    if (clear) uop.br_mask &= ~clear;
    const uint64_t bit = 1ULL << uop.op;
    int target = -1;  // first FU of the port that supports the op and can take it
    for (const auto &e : port_mappings[p].entries)
      if ((e.support_mask & bit) && (!fu_inst_r_1[e.fu_idx].valid || fu2exu_io.entry[e.fu_idx].ready)) { target = e.fu_idx; break; }
    Assert(target >= 0 && "EXU enqueue without free FU input slot");
    fu_inst_r_1[target].valid = true;
    fu_inst_r_1[target].uop = uop;
  }
}

// Stage 0: every FU gets the cycle's flush/clear controls.
void Exu::comb_fu_ctrl() {
  Wires &w = *wires;
  const bool flush_all = w.rob_bcast.flush, flush_br = !flush_all && w.dec_bcast.mispred;
  const wire<BR_MASK_WIDTH> flush_mask = flush_all ? ~wire<BR_MASK_WIDTH>(0) : flush_br ? w.dec_bcast.br_mask : wire<BR_MASK_WIDTH>(0);
  for (size_t i = 0; i < units.size(); i++) {
    exu2fu_io.entry[i].flush = flush_all || flush_br;
    exu2fu_io.entry[i].flush_mask = flush_mask;
    exu2fu_io.entry[i].clear_mask = w.dec_bcast.clear_mask;
    units[i]->in = exu2fu_io.entry[i];
    units[i]->comb_ctrl();
  }
}

// Stage 1: valid input slots go to their FU unless flushed or killed (those slots are freed).
void Exu::comb_exu2fu_dispatch() {
  Wires &w = *wires;
  for (int fu = 0; fu < TOTAL_FU_COUNT; fu++) {
    if (!fu_inst_r[fu].valid || fu_to_port[fu] < 0 || fu_to_port[fu] >= ISSUE_WIDTH) continue;
    ExuInst u = fu_inst_r[fu].uop;
    if (w.rob_bcast.flush || is_br_killed(u, w.dec_bcast)) {
      fu_inst_r_1[fu].valid = false;
      continue;
    }
    if (w.dec_bcast.clear_mask) u.br_mask &= ~w.dec_bcast.clear_mask;
    exu2fu_io.entry[fu].en = true;
    exu2fu_io.entry[fu].inst = u;
  }
}

// Stage 2: the FUs evaluate.
void Exu::comb_fu_exec() {
  for (size_t i = 0; i < units.size(); i++) {
    units[i]->in = exu2fu_io.entry[i];
    units[i]->comb_issue();
    fu2exu_io.entry[i].complete = units[i]->out.complete;
    fu2exu_io.entry[i].inst = units[i]->out.inst;
  }
}

// Stage 3: collect results. Every completing FU is broadcast for bypass; each port's first completing FU wins the port
// and, unless flushed/killed, completes in the ROB, sends its LSU request, writes back (ALU ports) or takes part in the
// branch arbitration (the oldest mispredicted branch redirects). LSU write-backs come in on their own ports.
void Exu::comb_fu2exu_collect() {
  Wires &w = *wires;
  const bool flush = w.rob_bcast.flush;
  for (size_t i = 0; i < units.size(); i++)
    if (fu2exu_io.entry[i].complete) {
      w.exe2prf.bypass[i].uop = fu2exu_io.entry[i].inst;
      w.exe2prf.bypass[i].valid = true;
    }

  ExuEntry int_res[ALU_NUM] = {}, br_res[BRU_NUM] = {};
  for (int p = 0; p < ISSUE_WIDTH; p++) {
    const FuEntry *winner = nullptr;
    for (auto &e : port_mappings[p].entries)
      if (fu2exu_io.entry[e.fu_idx].complete) { winner = &e; break; }
    if (!winner) continue;
    ExuInst u = fu2exu_io.entry[winner->fu_idx].inst;
    const bool dead = flush || is_br_killed(u, w.dec_bcast);

    if (u.op == UOP_CSR && w.exe2csr.re) u.result = w.csr2exe.rdata;
    if (!dead) {
      if (u.op != UOP_LOAD && u.op != UOP_STA) {  // loads/stores complete when the LSU writes back
        w.exu2rob.entry[p].valid = true;
        w.exu2rob.entry[p].uop = u;
      }
      if (u.op == UOP_STA || u.op == UOP_LOAD) {
        Assert(winner->lsu_agu_port >= 0 && winner->lsu_agu_port < LSU_AGU_COUNT);
        w.exe2lsu.agu_req[winner->lsu_agu_port].valid = true;
        w.exe2lsu.agu_req[winner->lsu_agu_port].uop = u;
      } else if (u.op == UOP_STD) {
        Assert(winner->lsu_sdu_port >= 0 && winner->lsu_sdu_port < LSU_SDU_COUNT);
        w.exe2lsu.sdu_req[winner->lsu_sdu_port].valid = true;
        w.exe2lsu.sdu_req[winner->lsu_sdu_port].uop = u;
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
    exu2fu_io.entry[winner->fu_idx].consume = true;
  }

  // FUs pop what was consumed; a ready FU frees its input slot
  for (size_t i = 0; i < units.size(); i++) {
    units[i]->in = exu2fu_io.entry[i];
    units[i]->comb_consume();
    fu2exu_io.entry[i].ready = units[i]->out.ready;
  }
  for (int fu = 0; fu < TOTAL_FU_COUNT; fu++)
    if (fu_inst_r_1[fu].valid && fu2exu_io.entry[fu].ready) fu_inst_r_1[fu].valid = false;

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
}

// FU interaction in dependency order: controls -> dispatch to FUs -> FU evaluation -> collection.
void Exu::comb_exec() {
  comb_fu_ctrl();
  comb_exu2fu_dispatch();
  comb_fu_exec();
  comb_fu2exu_collect();
}

void Exu::seq() {
  for (int i = 0; i < TOTAL_FU_COUNT; i++) fu_inst_r[i] = fu_inst_r_1[i];
  for (auto fu : units) fu->seq();
}

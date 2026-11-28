#include "Exu.h"
#include "PreIduQueue.h"
#include "Csr.h"
#include "BackPorts.h"
#include "util.h"
#include "config.h"
#include "trace_hook.h"
#include <cstring>

namespace {
// A uop whose branch was squashed by this cycle's mispredict is dead.
inline bool is_br_killed(const ExuInst &uop, const DecBroadcastIO &db) {
  return db.mispred && is_younger_rob(uop->rob_idx, uop->rob_flag, db.redirect_rob_idx, db.redirect_rob_flag);
}

// The issue ports whose uop needs the instruction PC from the FTQ: an ALU port with a pc operand (auipc, jal), a BRU port.
inline bool needs_ftq_pc(int p, const Uop &uop) {
  const uint64_t bit = 1ULL << uop.op;
  if (p >= IQ_ALU_PORT_BASE && p < IQ_ALU_PORT_BASE + ALU_NUM && (bit & OP_MASK_ALU) && uop->src1_is_pc) return true;
  return p >= IQ_BR_PORT_BASE && p < IQ_BR_PORT_BASE + BRU_NUM && (bit & OP_MASK_BR);
}
}  // namespace

Exu::Exu(SimContext *ctx) : ctx(ctx) {}

// Functional units are created per issue port, in write-back priority order (MUL first).
void Exu::init() {
  struct Spec { FuKind kind; uint64_t mask; };
  static const Spec kSpecs[] = {{FuKind::MUL, OP_MASK_MUL}, {FuKind::DIV, OP_MASK_DIV},
                                {FuKind::AGU, OP_MASK_LD | OP_MASK_STA}, {FuKind::SDU, OP_MASK_STD},
                                {FuKind::BRU, OP_MASK_BR},
                                {FuKind::I2F, OP_MASK_I2F},   // integer -> FP: ahead of the ALU, which would otherwise starve it (write-back priority = this order)
                                {FuKind::ALU, OP_MASK_ALU}, {FuKind::CSR, OP_MASK_CSR},
                                {FuKind::FMA, OP_MASK_FMA}, {FuKind::FDIV, OP_MASK_FDIV}, {FuKind::FMISC, OP_MASK_FMISC}};
  int agu = 0, sdu = 0;
  div_fu_idx = -1;
  units.clear();
  for (int i = 0; i < ISSUE_WIDTH; i++) {
    port_slot_valid[i] = false;
    port_fus[i].clear();
    port_fu_mask[i] = 0;
    for (const Spec &spec : kSpecs) {
      if (!(GLOBAL_ISSUE_PORT_CONFIG[i].support_mask & spec.mask)) continue;
      FuUnit u;
      u.kind = spec.kind;
      u.mode = fu_mode(spec.kind);
      u.port = i;
      u.support_mask = spec.mask;
      if (spec.kind == FuKind::AGU) u.lsu_agu_port = agu++;
      if (spec.kind == FuKind::SDU) u.lsu_sdu_port = sdu++;
      if (spec.kind == FuKind::DIV) div_fu_idx = static_cast<int>(units.size());
      if (u.mode == FuMode::PIPE) {
        u.depth = fu_pipe_latency(spec.kind);
        u.ring.resize(u.depth);
      }
      port_fus[i].push_back(static_cast<int>(units.size()));
      port_fu_mask[i] |= 1ULL << units.size();
      units.push_back(std::move(u));
    }
  }
  Assert(static_cast<int>(units.size()) == TOTAL_FU_COUNT && TOTAL_FU_COUNT <= 64);
}

// Default (idle) outputs.
void Exu::begin_cycle() {
  for (int i = 0; i < ROB_WB_WIDTH; i++) out.exu2rob.entry[i].valid = false;
  out.exe2iss.div_wb_next = false;
  out.exe2iss.div_wake_valid = false;
  out.exe2iss.div_wake_preg = 0;
  for (int i = 0; i < LSU_AGU_COUNT; i++) out.exe2lsu.agu_req[i].valid = out.exe2lsu.agu_req[i].raw = false;
  for (int i = 0; i < LSU_SDU_COUNT; i++) out.exe2lsu.sdu_req[i].valid = false;
  out.exu2id.mispred = false;
  out.exu2id.redirect_pc = 0;
  out.exu2id.redirect_rob_idx = 0;
  out.exu2id.redirect_rob_flag = 0;
  out.exu2id.ftq_idx = 0;
  out.exu2id.ftq_off = 0;
  out.exu2id.br_type = 0;
}

// Reserve port 1 write-back and wake consumers one cycle before DIV finishes.
void Exu::ready() {
  out.exe2iss.div_wb_next = false;
  out.exe2iss.div_wake_valid = false;
  out.exe2iss.div_wake_preg = 0;
  if (!in.rob_bcast->flush && div_fu_idx >= 0) {
    const FuUnit &f = units[div_fu_idx];
    if (f.busy && f.done == sim_time + 1) {
      out.exe2iss.div_wb_next = true;
      if (f.cur.dest_en) {
        out.exe2iss.div_wake_valid = true;
        out.exe2iss.div_wake_preg = f.cur->dest_preg;
      }
    }
  }
}

// The CSR uop sitting in port 0's issue slot drives the CSR file's read/write request.
// The clock edge: the load write-back register; issued uops enter the per-port pipeline register (killed ones are dropped);
// a flush empties them.
void Exu::seq() {
  // the write-back register: loads that complete this cycle and were not woken early by the LSU
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    const auto &e = in.lsu2exe->wb_req[i];
    LoadWake &l = q_.lwake_r[i];
    l.valid = !in.rob_bcast->flush && e.valid && !is_br_killed(e.uop, (*in.dec_bcast)) && e.uop.dest_en &&
              !e.uop.early_woken;
    if (!l.valid) continue;
    l.preg = e.uop->dest_preg;
    l.rob_idx = e.uop->rob_idx;
    l.rob_flag = e.uop->rob_flag;
  }
  if (in.rob_bcast->flush) {
    for (int p = 0; p < ISSUE_WIDTH; p++) port_slot_valid[p] = false;
    return;
  }
  for (int p = 0; p < ISSUE_WIDTH; p++) {
    port_slot_valid[p] = false;
    if (!in.iss2prf->iss_entry[p].valid) continue;
    Uop iu = in.iss2prf->iss_entry[p].uop;   // (there is no register file: the operand values are facts of the trace)
    if (needs_ftq_pc(p, iu)) {
      const FtqPc r = ftq->ftq_pc(iu->ftq_idx, iu->ftq_offset);
      iu->pc = r.pc;
      iu->ftq_pred_taken = r.pred_taken;
      iu->ftq_next_pc = r.next_pc;
    } else {
      iu->pc = 0;
      iu->ftq_pred_taken = false;
      iu->ftq_next_pc = 0;
    }
    ExuInst uop = iu;
    if (is_br_killed(uop, (*in.dec_bcast))) continue;
    port_slot_valid[p] = true;
    port_slot[p] = uop;
  }
}

// FUs take their input slots and evaluate (units that hold nothing are skipped); then results are collected. Every completing
// FU is broadcast for bypass; each port's first completing FU wins the port and, unless flushed/killed, completes in the ROB,
// sends its LSU request, writes back (ALU ports) or takes part in the branch arbitration (the oldest mispredicted branch
// redirects). LSU write-backs come in on their own ports.
void Exu::exec() {
  const bool flush_all = in.rob_bcast->flush, flush_br = !flush_all && in.dec_bcast->mispred;
  const bool flush_any = flush_all || flush_br, flush = flush_all;
  const int64_t now = sim_time;

  // flush applied to the instructions held inside PIPE / ITER units
  if (flush_any) {
    for (FuUnit &f : units) {
      if (f.mode == FuMode::PIPE && f.count) {
        if (flush_all) {
          f.count = 0;
        } else {
          int m = 0;
          for (int k = 0; k < f.count; k++) {
            const int s = f.head + k >= f.depth ? f.head + k - f.depth : f.head + k;
            if (is_br_killed(f.ring[s].uop, (*in.dec_bcast))) continue;
            if (m != k) f.ring[f.head + m >= f.depth ? f.head + m - f.depth : f.head + m] = f.ring[s];
            m++;
          }
          f.count = m;
        }
      } else if (f.mode == FuMode::ITER && f.busy && (flush_all || is_br_killed(f.cur, (*in.dec_bcast)))) {
        f.busy = false;
        f.done = 0;
      }
    }
  }

  uint64_t cmask = 0;  // completing units
  ExuInst *cinst[64];
  // per-port issue pipeline registers feed their target FU (non-stalling)
  for (int p = 0; p < ISSUE_WIDTH; p++) {
    if (!port_slot_valid[p]) continue;
    ExuInst slot = port_slot[p];
    port_slot_valid[p] = false;
    const uint64_t bit = 1ULL << slot.op;
    int target = -1;
    for (int fu : port_fus[p])
      if (units[fu].support_mask & bit) { target = fu; break; }
    Assert(target >= 0 && "EXU enqueue without supporting FU");
    FuUnit &f = units[target];
    if (f.lsu_agu_port >= 0 && (slot.op == UOP_LOAD || slot.op == UOP_STA)) {   // the AGU is occupied, killed or not
      out.exe2lsu.agu_req[f.lsu_agu_port].raw = true;
      out.exe2lsu.agu_req[f.lsu_agu_port].uop.op = slot.op;
    }
    if (flush_all || is_br_killed(slot, (*in.dec_bcast))) continue;
    switch (f.mode) {
    case FuMode::SINGLE:
      f.out = slot;
      fu::compute(f.kind, f.out);
      cinst[target] = &f.out;
      cmask |= 1ULL << target;
      break;
    case FuMode::PIPE: {
      Assert(f.count < f.depth && "PIPE unit overflow");
      FuUnit::PipeEntry e;
      e.uop = slot;
      fu::compute(f.kind, e.uop);
      e.done_cycle = now + f.depth - 1;
      f.push_back(e);
      break;
    }
    case FuMode::ITER:
      Assert(!f.busy && "ITER unit occupied while busy");
      f.cur = slot;
      fu::compute(f.kind, f.cur);
      f.done = now + fu::iter_latency(f.kind, f.cur);
      f.busy = true;
      break;
    }
  }

  // completing PIPE / ITER units
  for (size_t i = 0; i < units.size(); i++) {
    FuUnit &f = units[i];
    if (f.mode == FuMode::PIPE && f.count && f.front().done_cycle <= now) {
      cinst[i] = &f.front().uop;
      cmask |= 1ULL << i;
    } else if (f.mode == FuMode::ITER && f.busy && f.done <= now) {
      cinst[i] = &f.cur;
      cmask |= 1ULL << i;
    }
  }

  auto rob_wb_ch_from_port = [](int p) -> int {
    if (p >= IQ_ALU_PORT_BASE && p < IQ_ALU_PORT_BASE + ALU_NUM) return ROB_WB_ALU_BASE + (p - IQ_ALU_PORT_BASE);
    if (p >= IQ_STD_PORT_BASE && p < IQ_STD_PORT_BASE + LSU_SDU_COUNT) return ROB_WB_STD_BASE + (p - IQ_STD_PORT_BASE);
    if (p >= IQ_BR_PORT_BASE && p < IQ_BR_PORT_BASE + BRU_NUM) return ROB_WB_BR_BASE + (p - IQ_BR_PORT_BASE);
    if (p >= IQ_FP_PORT_BASE && p < IQ_FP_PORT_BASE + GLOBAL_IQ_CONFIG[IQ_FP].port_num)
      return ROB_WB_FP_BASE + (p - IQ_FP_PORT_BASE);
    Assert(0 && "EXU: unexpected port for direct ROB completion");
    return 0;
  };

  ExuEntry br_res[BRU_NUM] = {};
  uint64_t consumed = 0;
  for (int p = 0; p < ISSUE_WIDTH && cmask; p++) {
    const uint64_t pm = cmask & port_fu_mask[p];
    if (!pm) continue;
    Assert(__builtin_popcountll(pm) == 1 && "EXU: multiple FUs completing on one port");
    const int fu = __builtin_ctzll(pm);
    FuUnit &winner = units[fu];
    ExuInst u = *cinst[fu];
    const bool dead = flush || is_br_killed(u, (*in.dec_bcast));

    if (u.op == UOP_CSR) {   // the read port (CSRRW / CSRRWI to x0 do not read)
      const bool rw_family = u->func3 == 0b001 || u->func3 == 0b101;
      if (!rw_family || u->dest_preg != 0) u.result = csr_state().read(u->csr_idx);
    }
    if (!dead) {
      if (u.op != UOP_LOAD && u.op != UOP_STA) {  // loads/stores complete when the LSU writes back
        const int ch = rob_wb_ch_from_port(p);
        out.exu2rob.entry[ch].valid = true;
        out.exu2rob.entry[ch].uop = u;
      }
      if (u.op == UOP_STA || u.op == UOP_LOAD) {
        Assert(winner.lsu_agu_port >= 0 && winner.lsu_agu_port < LSU_AGU_COUNT);
        out.exe2lsu.agu_req[winner.lsu_agu_port].valid = true;
        out.exe2lsu.agu_req[winner.lsu_agu_port].uop = u;
      } else if (u.op == UOP_STD) {
        Assert(winner.lsu_sdu_port >= 0 && winner.lsu_sdu_port < LSU_SDU_COUNT);
        out.exe2lsu.sdu_req[winner.lsu_sdu_port].valid = true;
        out.exe2lsu.sdu_req[winner.lsu_sdu_port].uop = u;
      }
    }
    if (!dead && p >= IQ_BR_PORT_BASE && p < IQ_BR_PORT_BASE + BRU_NUM) {  // a squashed branch must not drive recovery
      br_res[p - IQ_BR_PORT_BASE].uop = u;
      br_res[p - IQ_BR_PORT_BASE].valid = true;
    }
    consumed |= 1ULL << fu;
  }

  // units pop what was consumed
  for (uint64_t m = consumed; m; m &= m - 1) {
    FuUnit &f = units[__builtin_ctzll(m)];
    if (f.mode == FuMode::PIPE) f.pop_front();
    else if (f.mode == FuMode::ITER) { f.busy = false; f.done = 0; }
  }

  // LSU write-backs: a load (or SC) and a store address complete in the ROB
  auto rob_complete = [&](int ch, const Uop &u) {
    if (!(flush || is_br_killed(u, (*in.dec_bcast)))) {
      out.exu2rob.entry[ch].valid = true;
      out.exu2rob.entry[ch].uop = u;
    }
  };
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    if (!in.lsu2exe->wb_req[i].valid) continue;
    rob_complete(ROB_WB_LD_BASE + i, in.lsu2exe->wb_req[i].uop);
  }
  for (int i = 0; i < LSU_STA_COUNT; i++)
    if (in.lsu2exe->sta_wb_req[i].valid) rob_complete(ROB_WB_STA_BASE + i, in.lsu2exe->sta_wb_req[i].uop);

  // branch resolution: every resolved branch frees its mask bit, the oldest mispredict redirects
  bool mispred = false;
  const ExuInst *oldest = nullptr;
  for (int i = 0; i < BRU_NUM; i++) {
    if (!br_res[i].valid) continue;
    if (br_res[i].uop.mispred && (!mispred || cmp_inst_age(*oldest, br_res[i].uop))) {
      mispred = true;
      oldest = &br_res[i].uop;
    }
  }

  for (int p = 0; p < ROB_WB_WIDTH; p++) {
    if (!out.exu2rob.entry[p].valid) continue;
    const auto &u = out.exu2rob.entry[p].uop;
    const int op = (int)decode_uop_type(u.op);
    if (op != UOP_LOAD && op != UOP_CSR) continue;
    if (in.rob_bcast->flush_now) continue;   // a flush decided in this cycle (CFG_FLUSH_REG delays its effect): everything completing now is younger than the flush point, dead; its uid is about to be re-used by the refetch
    tbhook::on_exec({(uint32_t)u->rob_idx, op, (xlen_t)u.result, (bool)(u.page_fault_inst || u.page_fault_load || u.page_fault_store)});
  }

  out.exu2id.mispred = mispred;
  out.exu2id.redirect_pc = mispred ? oldest->diag_val : 0;
  out.exu2id.redirect_rob_idx = mispred ? (*oldest)->rob_idx : 0;
  out.exu2id.redirect_rob_flag = mispred ? (*oldest)->rob_flag : 0;
  out.exu2id.ftq_idx = mispred ? (*oldest)->ftq_idx : 0;
  out.exu2id.ftq_off = mispred ? (*oldest)->ftq_offset : 0;
  if (mispred) {   // type of the mispredicted branch in the BPU's encoding (0 cond, 1 call, 2 ret, 3 indirect, 5 jal, 6 indirect call)
    const auto &u = *oldest;
    int t = 0;
    if (u->type == JAL) t = (oldest->dest_en && u->dest_areg == 1) ? 1 : 5;
    else if (u->type == JALR) t = (u->src1_areg == 1 && u->dest_areg == 0 && u->imm == 0) ? 2 : (u->dest_areg == 1) ? 6 : 3;
    out.exu2id.br_type = t;
  }
}


// Late load wakeup: the loads of the write-back register that survive this cycle's mispredict, in lane order, at most
// LSU_LOAD_WB_WIDTH of them.
void Exu::load_wake() {
  const Regs &q = q_;
  begin_cycle();   // the first phase of the cycle: default outputs, d starts equal to q
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) out.prf_awake.wake[i].valid = false;
  int n = 0;
  for (int i = 0; i < LSU_LOAD_WB_WIDTH; i++) {
    const LoadWake &l = q.lwake_r[i];
    if (!l.valid || (in.dec_bcast->mispred && is_younger_rob(l.rob_idx, l.rob_flag, in.dec_bcast->redirect_rob_idx, in.dec_bcast->redirect_rob_flag))) continue;
    out.prf_awake.wake[n].valid = true;
    out.prf_awake.wake[n].preg = l.preg;
    n++;
  }
}


// late load wakeup, the FUs, the port readiness for issue
void Exu::comb_exec() {
  load_wake();
  exec();
  ready();
}

#include "Dispatch.h"
#include "Rob.h"
#include "BackPorts.h"
#include "config.h"
#include "util.h"

// A source woken by this cycle's wake-up bus (it carries the load write-backs too) is no longer busy.
void Dispatch::apply_wakeup_to_uop(Uop &uop) const {
  const IssAwakeIO &w = *in.iss_awake;
  if (uop->src1_en && w.woken(uop->src1_preg)) uop.src1_busy = false;
  if (uop->src2_en && w.woken(uop->src2_preg)) uop.src2_busy = false;
  if (uop->src3_en && w.woken(uop->src3_preg)) uop.src3_busy = false;
}

void Dispatch::init() {
  for (int i = 0; i < DECODE_WIDTH; i++) {
    q_.inst_r[i] = {};
    q_.inst_valid[i] = false;
    fit_[i] = true;
  }
  for (int &o : stq_port_owner) o = -1;
  for (int &o : ldq_port_owner) o = -1;
  std::memset(q_.busy_table, 0, sizeof(q_.busy_table));
}

// The instructions of the stage go to the ROB's input (out.dis2rob.uop, also this cycle's working copy of each slot) with
// their ROB entry, and loads / stores take their LDQ / STQ entry and request port. A load records the STQ tail it must
// see (the stores before it); a load or store that finds no entry or port is not valid for the ROB.
void Dispatch::comb_alloc() {
  const Regs &q = q_;
  const auto clear_mask = in.dec_bcast->clear_mask;
  const LsuDisIO &lsu = *in.lsu2dis;
  int stores = 0, loads = 0, rob_k = 0;   // rob_k: position among the valid slots (the ROB entry is the tail + rob_k)
  for (int &o : stq_port_owner) o = -1;
  for (int &o : ldq_port_owner) o = -1;
  auto stq_slot = [&](Uop &u) {   // the STQ entry after the stores allocated before it in this group
    const int pos = lsu.stq_tail + stores;
    u->stq_idx = pos % STQ_SIZE;
    u->stq_flag = lsu.stq_tail_flag ^ ((pos / STQ_SIZE) & 1);
  };
  for (int i = 0; i < DECODE_WIDTH; i++) {
    iq_blk[i] = -1;
    iq_blk_width[i] = false;
    Uop &u = out.dis2rob.uop[i];
    u = q.inst_r[i];
    u.br_mask &= ~clear_mask;  // branches resolved this cycle must not enter the back-end buffers
    u.cplt_mask = 0;
    out.dis2rob.valid[i] = q.inst_valid[i];
    if (!q.inst_valid[i]) continue;   // (an invalid slot's handle names an instruction that has left: its pool entry is not written)
    const int pos = static_cast<int>(in.rob2dis->enq_idx) + rob_k++;
    u->rob_idx = pos % ROB_NUM;
    u->rob_flag = static_cast<bool>(in.rob2dis->rob_flag) ^ (pos >= ROB_NUM);
    u->expect_mask = 0;
    if (is_load(u)) {
      stq_slot(u);
      if (loads < lsu.ldq_free && loads < GLOBAL_IQ_CONFIG[IQ_LD].dispatch_width && loads < MAX_LDQ_DISPATCH_WIDTH && lsu.ldq_alloc_valid[loads]) {
        u->ldq_idx = lsu.ldq_alloc_idx[loads];
        ldq_port_owner[loads++] = i;
      } else {
        if (loads < lsu.ldq_free && loads >= GLOBAL_IQ_CONFIG[IQ_LD].dispatch_width) { iq_blk[i] = IQ_LD; iq_blk_width[i] = true; }
        out.dis2rob.valid[i] = false;
      }
    }
    if (is_store(u)) {
      if (stores < lsu.stq_free && stores < MAX_STQ_DISPATCH_WIDTH) {
        stq_slot(u);
        stq_port_owner[stores++] = i;
      } else {
        out.dis2rob.valid[i] = false;
      }
    }
  }
}

// Source-operand busy bits of slot i: busy table, this cycle's wakeups, and a register written by an older slot of the
// same group stays busy (the older instruction has not issued yet).
void Dispatch::wake(int i) {
  const Regs &q = q_;
  Uop &u = out.dis2rob.uop[i];
  u.src1_busy = u->src1_en ? q.busy_table[u->src1_preg] : false;
  u.src2_busy = u->src2_en ? q.busy_table[u->src2_preg] : false;
  u.src3_busy = u->src3_en ? q.busy_table[u->src3_preg] : false;
  apply_wakeup_to_uop(u);
  for (int j = 0; j < i; j++) {
    const Uop &o = out.dis2rob.uop[j];
    if (!out.dis2rob.valid[j] || !o.dest_en || (o->dest_areg == 0 && !o->dest_fp)) continue;
    if (is_eliminated_move(o)) continue;  // its register is its source's: no writer to wait for
    if (u->src1_en && u->src1_preg == o->dest_preg) u.src1_busy = true;
    if (u->src2_en && u->src2_preg == o->dest_preg) u.src2_busy = true;
    if (u->src3_en && u->src3_preg == o->dest_preg) u.src3_busy = true;
  }
}

// One pass in slot order: each slot's busy bits; then, up to the first slot that does not fire, its uops, whether they
// fit the issue queues, and whether it fires. A slot fires when it has its LSQ entry, its uops fit, the ROB has room and
// takes instructions, nothing flushes, and nothing older stalled; CSR and atomic instructions also need an empty ROB and
// nothing older firing; an atomic or WFI that fires is alone. Only fired slots place their uops (so nothing is taken back).
void Dispatch::comb_dispatch() {
  const Regs &q = q_;
  for (int i = 0; i < IQ_NUM; i++)
    for (int k = 0; k < MAX_IQ_DISPATCH_WIDTH; k++) out.dis2iss.req[i][k].valid = false;
  const bool global_flush = in.rob_bcast->flush || in.dec_bcast->mispred || in.rob2dis->stall;
  int usage[IQ_NUM] = {0};
  int rob_used = 0;  // ROB entries taken by the older slots of this group
  bool stalled = false, fired = false;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    out.dis2rob.dis_fire[i] = false;
    fit_[i] = true;
    if (out.dis2rob.valid[i]) wake(i);   // (a slot that waits keeps them in the stage register)
    if (!q.inst_valid[i] || stalled) continue;
    Uop &u = out.dis2rob.uop[i];
    const InstType type = decode_inst_type(u->type);

    UopPacket uops[MAX_UOPS_PER_INST];
    const int cnt = decompose_inst(u, uops);
    wire<ROB_CPLT_MASK_WIDTH> expect = 0;
    for (int k = 0; k < cnt; k++) expect |= rob_cplt_mask_from_iq(uops[k].iq_id);
    // SC returns its 0/1 result through the load-like write-back and still completes an STD
    if (type == AMO && (u->func7 >> 2) == AmoOp::SC) expect = ROB_CPLT_G0 | ROB_CPLT_G2;
    int lane_use[IQ_NUM] = {0};   // uops of this instruction already counted for each queue (a memory queue can take two: LD + STA of an AMO)
    for (int k = 0; k < cnt && fit_[i]; k++) {
      const int iq = route_iq(uops[k].iq_id);
      const int u_iq = usage[iq] + lane_use[iq]++;
      fit_[i] = u_iq < GLOBAL_IQ_CONFIG[iq].dispatch_width && u_iq < in.iss2dis->ready_num[iq];
      if (!fit_[i]) {
        iq_blk[i] = iq;
        iq_blk_width[i] = usage[iq] < in.iss2dis->ready_num[iq];  // room left: the enqueue width is what stopped it
      }
    }

    bool fire = out.dis2rob.valid[i] && fit_[i] && in.rob2dis->ready && !global_flush && rob_used < in.rob2dis->free_cnt;
    rob_used++;
    if ((is_CSR(type) || type == AMO) && (!in.rob2dis->empty || fired)) fire = false;   // serializing (CSR, xRET, ECALL, EBREAK, AMO)
    if (!fire) { stalled = true; continue; }

    out.dis2rob.dis_fire[i] = true;
    fired = true;
    u->expect_mask = expect;
    for (int k = 0; k < cnt; k++) {
      const int iq = route_iq(uops[k].iq_id);
      out.dis2iss.req[iq][usage[iq]].valid = true;
      out.dis2iss.req[iq][usage[iq]].uop = uops[k].uop;
      usage[iq]++;
    }
    if (type == AMO || type == WFI) stalled = true;  // exclusive issue slot
  }
  out.dis2ren.ready = !stalled;

  // the LSQ allocation requests of the fired loads and stores (the LSU reads the rest from the pool by id)
  const auto clear_mask = in.dec_bcast->clear_mask;
  for (int k = 0; k < MAX_STQ_DISPATCH_WIDTH; k++) {
    const int i = stq_port_owner[k];
    out.dis2lsu.alloc_req[k] = i >= 0 && out.dis2rob.dis_fire[i];
    if (!out.dis2lsu.alloc_req[k]) continue;
    out.dis2lsu.id[k] = out.dis2rob.uop[i].id;
    out.dis2lsu.br_mask[k] = out.dis2rob.uop[i].br_mask & ~clear_mask;
  }
  for (int k = 0; k < MAX_LDQ_DISPATCH_WIDTH; k++) {
    const int i = ldq_port_owner[k];
    out.dis2lsu.ldq_alloc_req[k] = i >= 0 && out.dis2rob.dis_fire[i];
    if (!out.dis2lsu.ldq_alloc_req[k]) continue;
    out.dis2lsu.ldq_id[k] = out.dis2rob.uop[i].id;
    out.dis2lsu.ldq_br_mask[k] = out.dis2rob.uop[i].br_mask & ~clear_mask;
  }
}

// The clock edge. Busy table: this cycle's wakeups clear, the fired destinations set. Stage register: cleared by a flush or
// mispredict, refilled from rename when the stage is free, otherwise the instructions that did not fire stay (with busy
// bits refreshed by this cycle's wakeups).
void Dispatch::seq() {
  Regs &d = q_;   // updated in place (each slot reads only its own old value first)
  for (int k = 0; k < WAKE_CH_ALL; k++)
    if (in.iss_awake->wake[k].valid) d.busy_table[in.iss_awake->wake[k].preg] = false;
  for (int i = 0; i < DECODE_WIDTH; i++)
    if (out.dis2rob.dis_fire[i] && d.inst_r[i].dest_en && !is_eliminated_move(d.inst_r[i]))
      d.busy_table[d.inst_r[i]->dest_preg] = true;
  const Regs &q = q_;
  const auto clear_mask = in.dec_bcast->clear_mask;
  const bool kill = in.rob_bcast->flush || in.dec_bcast->mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (kill) { d.inst_valid[i] = false; continue; }
    if (out.dis2ren.ready) {
      d.inst_valid[i] = in.ren2dis->valid[i];
      d.inst_r[i] = in.ren2dis->uop[i];
      d.inst_r[i].br_mask &= ~clear_mask;
      continue;
    }
    d.inst_valid[i] = q.inst_valid[i] && !out.dis2rob.dis_fire[i];
    if (d.inst_valid[i]) {
      d.inst_r[i].src1_busy = out.dis2rob.uop[i].src1_busy;
      d.inst_r[i].src2_busy = out.dis2rob.uop[i].src2_busy;
      d.inst_r[i].src3_busy = out.dis2rob.uop[i].src3_busy;
      apply_wakeup_to_uop(d.inst_r[i]);
      d.inst_r[i].br_mask &= ~clear_mask;
    }
  }
}


// A uop of `inst`: its own pool slot (the instruction's block, UOP_BLOCK in types.h), a copy of the instruction's (the pieces override a few static fields below), and the
// instruction's per-copy bits.
Uop Dispatch::make_dis_iss_uop(const Uop &inst, int k) const {
  Uop uop;
  uop.id = static_cast<uint16_t>(inst.id + 1 + k);
  uop_pool[uop.id] = uop_pool[inst.id];
  uop.dest_en = inst.dest_en;
  uop.src1_busy = inst.src1_busy;
  uop.src2_busy = inst.src2_busy;
  uop.src3_busy = inst.src3_busy;
  uop.br_mask = inst.br_mask;
  return uop;
}

namespace {
// One uop of a decomposition: target queue, operation, and the fields it overrides (-1 = keep the instruction's value).
struct Piece {
  int iq, op;
  int src1_en, src2_en, dest_en;
  bool imm4;    // immediate becomes the instruction length (the "PC + 4" link value; 2 for a compressed instruction)
  bool atomic;  // marks the STD of an atomic
};
}  // namespace

int Dispatch::decompose_inst(const Uop &inst, UopPacket *out) {
  const InstType type = decode_inst_type(inst->type);
  const int amo = inst->func7 >> 2;
  Piece pieces[MAX_UOPS_PER_INST];
  int n = 0;
  if (is_eliminated_move(inst)) return 0;  // renamed away by move elimination: completes in the ROB at dispatch
  auto add = [&](int iq, int op, int s1 = -1, int s2 = -1, int dst = -1, bool imm4 = false, bool atomic = false) {
    pieces[n++] = {iq, op, s1, s2, dst, imm4, atomic};
  };
  switch (type) {
  case ADD:   add(IQ_INT, UOP_ADD); break;
  case MUL:   add(IQ_INT, UOP_MUL); break;
  case DIV:   add(IQ_INT, UOP_DIV); break;
  case FPOP:
    switch (inst->fp_cls) {
    case FP_C_FMA:  add(IQ_FP, UOP_FMA); break;
    case FP_C_FDIV: add(IQ_FP, UOP_FDIV); break;
    case FP_C_MISC: add(IQ_FP, UOP_FMISC); break;
    case FP_C_I2F:  add(IQ_INT, UOP_I2F); break;
    default: Assert(0 && "FP instruction without a unit class");
    }
    break;
  case BR:    add(IQ_BR, UOP_BR); break;
  case LOAD:  add(IQ_LD, UOP_LOAD); break;
  case STORE:  // STA uses the base register, STD the data register
    add(IQ_STA, UOP_STA, -1, 0);
    add(IQ_STD, UOP_STD, 0, 1);
    break;
  case JALR:  // link value through the integer queue, then the jump (needs the base register)
    add(IQ_INT, UOP_ADD, 0, 0, -1, true);
    add(IQ_BR, UOP_JUMP, 1, -1, 0);
    break;
  case JAL:
    add(IQ_INT, UOP_ADD, 0, 0, -1, true);
    add(IQ_BR, UOP_JUMP, -1, -1, 0);
    break;
  case AMO:
    if (amo == AmoOp::LR) {
      add(IQ_LD, UOP_LOAD, -1, 0);
    } else if (amo == AmoOp::SC) {  // the 0/1 result comes back through the STA write-back port
      add(IQ_STA, UOP_STA, -1, 0, 1);
      add(IQ_STD, UOP_STD, 0, -1, 0, false, true);
    } else {  // read-modify-write: load, store address, store data (data source patched below)
      add(IQ_LD, UOP_LOAD, -1, 0);
      add(IQ_STA, UOP_STA, -1, 0, 0);
      add(IQ_STD, UOP_STD, -1, -1, 0, false, true);
    }
    break;
  case NOP:        add(IQ_INT, UOP_ADD); break;
  case CSR:        add(IQ_INT, UOP_CSR); break;
  case FENCE:      add(IQ_INT, UOP_FENCE); break;
  case ECALL:      add(IQ_INT, UOP_ECALL); break;
  case MRET:       add(IQ_INT, UOP_MRET); break;
  case SRET:       add(IQ_INT, UOP_SRET); break;
  case SFENCE_VMA: add(IQ_INT, UOP_SFENCE_VMA); break;
  case FENCE_I:    add(IQ_INT, UOP_FENCE_I); break;
  case EBREAK:     add(IQ_INT, UOP_EBREAK); break;
  case WFI:        add(IQ_INT, UOP_WFI); break;
  default: Assert(0 && "unknown instruction");
  }

  for (int k = 0; k < n; k++) {
    const Piece &p = pieces[k];
    out[k].iq_id = p.iq;
    Uop &u = out[k].uop;
    u = make_dis_iss_uop(inst, k);
    u.op = static_cast<UopType>(p.op);
    if (p.src1_en >= 0) u->src1_en = p.src1_en;
    if (p.src2_en >= 0) u->src2_en = p.src2_en;
    if (p.dest_en >= 0) u.dest_en = p.dest_en;
    if (p.imm4) u->imm = ilen_of(u->is_rvc);
    if (p.atomic) u->is_atomic = true;
  }
  if (type == AMO && amo != AmoOp::LR && amo != AmoOp::SC) {
    // the store data of an AMO comes from the loaded value (the load's destination), except for SWAP
    out[2].uop->src1_preg = inst->dest_preg;
    out[2].uop.src1_busy = amo != AmoOp::SWAP;
  }
  return n;
}


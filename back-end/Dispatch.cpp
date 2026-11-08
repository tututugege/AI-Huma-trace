#include "BeginCheck.h"
#include "Dispatch.h"
#include "Wires.h"
#include "config.h"
#include "util.h"

bool Dispatch::is_preg_woken(wire<PRF_IDX_WIDTH> preg) const {
  for (int w = 0; w < LSU_LOAD_WB_WIDTH; w++)
    if (wires->prf_awake.wake[w].valid && wires->prf_awake.wake[w].preg == preg) return true;
  for (int w = 0; w < WAKE_CH_ALL; w++)
    if (wires->iss_awake.wake[w].valid && wires->iss_awake.wake[w].preg == preg) return true;
  return false;
}

void Dispatch::apply_wakeup_to_uop(Uop &uop) const {
  if (uop->src1_en && is_preg_woken(uop->src1_preg)) uop.src1_busy = false;
  if (uop->src2_en && is_preg_woken(uop->src2_preg)) uop.src2_busy = false;
}

void Dispatch::init() {
  for (int i = 0; i < DECODE_WIDTH; i++) {
    inst_r[i] = inst_r_1[i] = inst_alloc[i] = {};
    inst_valid[i] = inst_valid_1[i] = false;
    dispatch_success_flags[i] = false;
    dispatch_cache[i] = {};
  }
  for (int &o : stq_port_owner) o = -1;
  for (int &o : ldq_port_owner) o = -1;
  std::memset(busy_table, 0, sizeof(busy_table));
  std::memset(busy_table_1, 0, sizeof(busy_table_1));
}

// The working copies already equal the registered state (seq() copies them back).
void Dispatch::comb_begin() {
  BEGIN_EQ(inst_r_1, inst_r);
  BEGIN_EQ(inst_valid_1, inst_valid);
  BEGIN_EQ(busy_table_1, busy_table);
}

// Allocate ROB ids and LDQ/STQ entries for the instructions waiting in the stage. A load records the STQ tail it must
// see (the stores before it); a load or store that finds no queue entry or port is marked invalid for the ROB.
void Dispatch::comb_alloc() {
  Wires &w = *wires;
  const auto clear_mask = w.dec_bcast.clear_mask;
  int stores = 0, loads = 0;

  for (int k = 0; k < MAX_STQ_DISPATCH_WIDTH; k++) {
    w.dis2lsu.alloc_req[k] = false;
    w.dis2lsu.br_mask[k] = 0;
    stq_port_owner[k] = -1;
  }
  for (int k = 0; k < MAX_LDQ_DISPATCH_WIDTH; k++) {
    w.dis2lsu.ldq_alloc_req[k] = false;
    w.dis2lsu.ldq_br_mask[k] = 0;
    ldq_port_owner[k] = -1;
  }

  for (int i = 0; i < DECODE_WIDTH; i++) { iq_blk[i] = -1; iq_blk_width[i] = false; }
  for (int i = 0; i < DECODE_WIDTH; i++) {
    Uop &u = inst_alloc[i];
    u = inst_r[i];
    u.br_mask &= ~clear_mask;  // branches resolved this cycle must not enter the back-end buffers
    u.rob_idx = make_rob_idx(w.rob2dis.enq_idx, i);
    u.rob_flag = w.rob2dis.rob_flag;
    u.expect_mask = 0;
    u.cplt_mask = 0;
    w.dis2rob.valid[i] = inst_valid[i];

    if (inst_valid[i] && is_load(inst_r[i])) {
      const int pos = w.lsu2dis.stq_tail + stores;  // stores older than this load, allocated this cycle
      u.stq_idx = pos % STQ_SIZE;
      u.stq_flag = w.lsu2dis.stq_tail_flag ^ ((pos / STQ_SIZE) & 1);
      if (loads < w.lsu2dis.ldq_free && loads < GLOBAL_IQ_CONFIG[IQ_LD].dispatch_width && loads < MAX_LDQ_DISPATCH_WIDTH &&
          w.lsu2dis.ldq_alloc_valid[loads]) {
        u.ldq_idx = w.lsu2dis.ldq_alloc_idx[loads];
        ldq_port_owner[loads++] = i;
      } else {
        if (loads < w.lsu2dis.ldq_free && loads >= GLOBAL_IQ_CONFIG[IQ_LD].dispatch_width) { iq_blk[i] = IQ_LD; iq_blk_width[i] = true; }
        w.dis2rob.valid[i] = false;
      }
    }
    if (inst_valid[i] && is_store(inst_r[i])) {
      if (stores < w.lsu2dis.stq_free && stores < MAX_STQ_DISPATCH_WIDTH) {
        const int pos = w.lsu2dis.stq_tail + stores;
        u.stq_idx = pos % STQ_SIZE;
        u.stq_flag = w.lsu2dis.stq_tail_flag ^ ((pos / STQ_SIZE) & 1);
        stq_port_owner[stores++] = i;
      } else {
        w.dis2rob.valid[i] = false;
      }
    }
    w.dis2rob.uop[i] = u;
  }
}

// Source-operand busy bits: busy table, this cycle's wakeups, and a register written by an older slot of the same group
// stays busy (the older instruction has not issued yet).
void Dispatch::comb_wake() {
  Wires &w = *wires;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!w.dis2rob.valid[i]) continue;
    Uop &u = inst_alloc[i];
    u.src1_busy = u->src1_en ? busy_table[u->src1_preg] : false;
    u.src2_busy = u->src2_en ? busy_table[u->src2_preg] : false;
    apply_wakeup_to_uop(u);
    for (int j = 0; j < i; j++) {
      if (!w.dis2rob.valid[j] || !inst_alloc[j].dest_en || inst_alloc[j]->dest_areg == 0) continue;
      if (is_eliminated_move(inst_alloc[j])) continue;  // its register is its source's: no writer to wait for
      if (u->src1_en && u->src1_preg == inst_alloc[j].dest_preg) u.src1_busy = true;
      if (u->src2_en && u->src2_preg == inst_alloc[j].dest_preg) u.src2_busy = true;
    }
  }
}

// A speculative load wake-up of the previous cycle was wrong: the destination register is busy again.
void Dispatch::comb_cancel() {
  for (int k = 0; k < LSU_LOAD_WB_WIDTH; k++)
    if (wires->lsu_awake.cancel[k].valid) busy_table[wires->lsu_awake.cancel[k].preg] = true;
}

// Split each instruction into uops and place them in the issue-queue request ports, in slot order; the first
// instruction that does not fit stops the group. What was placed is remembered so comb_fire can take it back.
void Dispatch::comb_dispatch() {
  Wires &w = *wires;
  for (int i = 0; i < IQ_NUM; i++)
    for (int k = 0; k < MAX_IQ_DISPATCH_WIDTH; k++) w.dis2iss.req[i][k].valid = false;

  int usage[IQ_NUM] = {0};
  for (int i = 0; i < DECODE_WIDTH; i++) {
    dispatch_success_flags[i] = false;
    dispatch_cache[i].count = 0;
    if (!inst_valid[i]) {
      dispatch_success_flags[i] = true;
      continue;
    }

    UopPacket uops[MAX_UOPS_PER_INST];
    const int cnt = decompose_inst(inst_alloc[i], uops);
    dispatch_cache[i].count = cnt;
    wire<ROB_CPLT_MASK_WIDTH> expect = 0;
    for (int k = 0; k < cnt; k++) {
      expect |= rob_cplt_mask_from_iq(uops[k].iq_id);
      dispatch_cache[i].iq_ids[k] = uops[k].iq_id;
    }
    // SC returns its 0/1 result through the load-like write-back and still completes an STD
    if (decode_inst_type(inst_alloc[i]->type) == AMO && (inst_alloc[i]->func7 >> 2) == AmoOp::SC)
      expect = ROB_CPLT_G0 | ROB_CPLT_G2;
    inst_alloc[i].expect_mask = expect;
    inst_alloc[i].cplt_mask = 0;

    bool fit = true;
    for (int k = 0; k < cnt && fit; k++) {
      const int q = uops[k].iq_id;
      fit = usage[q] < GLOBAL_IQ_CONFIG[q].dispatch_width && usage[q] < w.iss2dis.ready_num[q];
      if (!fit) {
        iq_blk[i] = q;
        iq_blk_width[i] = usage[q] < w.iss2dis.ready_num[q];  // room left: the enqueue width is what stopped it
      }
    }
    if (!fit) break;

    dispatch_success_flags[i] = true;
    w.dis2rob.uop[i].expect_mask = expect;
    w.dis2rob.uop[i].cplt_mask = 0;
    for (int k = 0; k < cnt; k++) {
      const int q = uops[k].iq_id;
      w.dis2iss.req[q][usage[q]].valid = true;
      w.dis2iss.req[q][usage[q]].uop = uops[k].uop;
      usage[q]++;
    }
  }
}

// Final fire decision per slot, busy-table update, and confirm/take back the issue-queue and LSU requests.
// Flushes and stalls block everything; CSR and atomic instructions only go when the ROB is empty and nothing
// older fires in the same cycle; an atomic or WFI that fires blocks the slots behind it; a slot that does not
// fire blocks the slots behind it.
void Dispatch::comb_fire() {
  Wires &w = *wires;
  const auto clear_mask = w.dec_bcast.clear_mask;
  const bool global_flush = w.rob_bcast.flush || w.dec_bcast.mispred || w.rob2dis.stall;

  std::memcpy(busy_table_1, busy_table, sizeof(busy_table_1));
  for (int k = 0; k < LSU_LOAD_WB_WIDTH; k++)
    if (w.prf_awake.wake[k].valid) busy_table_1[w.prf_awake.wake[k].preg] = false;
  for (int k = 0; k < WAKE_CH_ALL; k++)
    if (w.iss_awake.wake[k].valid) busy_table_1[w.iss_awake.wake[k].preg] = false;

  bool pre_stall = false, pre_fire = false;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    const InstType type = inst_valid[i] ? decode_inst_type(inst_r[i]->type) : NOP;
    const bool atomic = type == AMO, wfi = type == WFI;
    bool fire = w.dis2rob.valid[i] && dispatch_success_flags[i] && w.rob2dis.ready && !pre_stall && !global_flush;
    // serializing instructions need an empty ROB and nothing older firing this cycle
    if ((is_CSR(inst_r[i]->type) || atomic) && (!w.rob2dis.empty || pre_fire)) fire = false;
    w.dis2rob.dis_fire[i] = fire;
    if (inst_valid[i] && !fire) pre_stall = true;
    if (fire && (atomic || wfi)) pre_stall = true;  // exclusive issue slot
    if (fire) pre_fire = true;
  }
  for (int i = 0; i < DECODE_WIDTH; i++)
    if (w.dis2rob.dis_fire[i] && inst_r[i].dest_en && !is_eliminated_move(inst_r[i]))
      busy_table_1[inst_r[i].dest_preg] = true;
  w.dis2ren.ready = !pre_stall;

  // take back the issue requests of instructions that did not fire (ports were assigned in slot order)
  int next_slot[IQ_NUM] = {0};
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!inst_valid[i]) continue;
    if (!dispatch_success_flags[i]) break;
    for (int k = 0; k < dispatch_cache[i].count; k++) {
      const int q = dispatch_cache[i].iq_ids[k];
      if (!w.dis2rob.dis_fire[i]) w.dis2iss.req[q][next_slot[q]].valid = false;
      next_slot[q]++;
    }
  }

  for (int k = 0; k < MAX_STQ_DISPATCH_WIDTH; k++) {
    const int i = stq_port_owner[k];
    if (i < 0 || !w.dis2rob.dis_fire[i]) continue;
    const Uop &u = w.dis2rob.uop[i];
    w.dis2lsu.alloc_req[k] = true;
    w.dis2lsu.br_mask[k] = u.br_mask & ~clear_mask;
    w.dis2lsu.rob_idx[k] = u.rob_idx;
    w.dis2lsu.rob_flag[k] = u.rob_flag;
    w.dis2lsu.stq_flag[k] = u.stq_flag;
    w.dis2lsu.func3[k] = u->func3;
    w.dis2lsu.st_pc[k] = u->dbg.pc;
  }
  for (int k = 0; k < MAX_LDQ_DISPATCH_WIDTH; k++) {
    const int i = ldq_port_owner[k];
    if (i < 0 || !w.dis2rob.dis_fire[i]) continue;
    const Uop &u = w.dis2rob.uop[i];
    w.dis2lsu.ldq_alloc_req[k] = true;
    w.dis2lsu.ldq_idx[k] = u.ldq_idx;
    w.dis2lsu.ldq_br_mask[k] = u.br_mask & ~clear_mask;
    w.dis2lsu.ldq_rob_idx[k] = u.rob_idx;
    w.dis2lsu.ldq_rob_flag[k] = u.rob_flag;
    w.dis2lsu.ldq_pc[k] = u->dbg.pc;
  }
}

// Stage register: cleared by a flush or mispredict, refilled from rename when the stage is free, otherwise the
// instructions that did not fire stay (with busy bits refreshed by this cycle's wakeups).
void Dispatch::comb_pipeline() {
  Wires &w = *wires;
  const auto clear_mask = w.dec_bcast.clear_mask;
  const bool kill = w.rob_bcast.flush || w.dec_bcast.mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (kill) { inst_valid_1[i] = false; continue; }
    if (w.dis2ren.ready) {
      inst_valid_1[i] = w.ren2dis.valid[i];
      inst_r_1[i] = w.ren2dis.uop[i];
      inst_r_1[i].br_mask &= ~clear_mask;
      continue;
    }
    inst_valid_1[i] = inst_valid[i] && !w.dis2rob.dis_fire[i];
    if (inst_valid_1[i]) {
      inst_r_1[i].src1_busy = inst_alloc[i].src1_busy;
      inst_r_1[i].src2_busy = inst_alloc[i].src2_busy;
      apply_wakeup_to_uop(inst_r_1[i]);
      inst_r_1[i].br_mask &= ~clear_mask;
    }
  }
}

void Dispatch::seq() {
  for (int i = 0; i < DECODE_WIDTH; i++) {
    inst_r[i] = inst_r_1[i];
    inst_valid[i] = inst_valid_1[i];
  }
  std::memcpy(busy_table, busy_table_1, sizeof(busy_table));
}

// A uop of `inst`: it gets its own pool slot (static fields are copied; STA/STD, JAL/JALR pieces differ).
Uop Dispatch::make_dis_iss_uop(const Uop &inst, int k) const {
  Uop uop;
  uop.id = static_cast<uint16_t>(inst.id + 1 + k);
  uop_pool[uop.id] = uop_pool[inst.id];
  uop.dest_preg = inst.dest_preg;
  uop->src1_preg = inst->src1_preg;
  uop->src2_preg = inst->src2_preg;
  uop->ftq_idx = inst->ftq_idx;
  uop->ftq_offset = inst->ftq_offset;
  uop->is_atomic = inst->is_atomic;
  uop.dest_en = inst.dest_en;
  uop->src1_en = inst->src1_en;
  uop->src2_en = inst->src2_en;
  uop.src1_busy = inst.src1_busy;
  uop.src2_busy = inst.src2_busy;
  uop->src1_is_pc = inst->src1_is_pc;
  uop->src2_is_imm = inst->src2_is_imm;
  uop->func3 = inst->func3;
  uop->func7 = inst->func7;
  uop->imm = inst->imm;
  uop->br_id = inst->br_id;
  uop.br_mask = inst.br_mask;
  uop->csr_idx = inst->csr_idx;
  uop.rob_idx = inst.rob_idx;
  uop.stq_idx = inst.stq_idx;
  uop.stq_flag = inst.stq_flag;
  uop.ldq_idx = inst.ldq_idx;
  uop.rob_flag = inst.rob_flag;
  uop->dbg = inst->dbg;
  return uop;
}

namespace {
// One uop of a decomposition: target queue, operation, and the fields it overrides (-1 = keep the instruction's value).
struct Piece {
  int iq, op;
  int src1_en, src2_en, dest_en;
  bool imm4;    // immediate becomes 4 (the "PC + 4" link value)
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
  case FP:    add(IQ_INT, UOP_FP); break;
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
    if (p.imm4) u->imm = 4;
    if (p.atomic) u->is_atomic = true;
  }
  if (type == AMO && amo != AmoOp::LR && amo != AmoOp::SC) {
    // the store data of an AMO comes from the loaded value (the load's destination), except for SWAP
    out[2].uop->src1_preg = inst.dest_preg;
    out[2].uop.src1_busy = amo != AmoOp::SWAP;
  }
  return n;
}

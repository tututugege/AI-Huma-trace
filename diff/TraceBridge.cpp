#include "TraceBridge.h"
#include "refcpu_api.h"
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

extern long long sim_time;   // timing-model cycle counter (main.cpp)

namespace tb {

// Undo log of one dispatched instruction (~60 bytes, no full register/CSR copy):
//   pc / privilege / reservation before it, the old value of its destination register,
//   the CSRs a trap writes, the whole CSR file only for SYSTEM instructions,
//   and the old memory word of a store/AMO.
// Indices into RefCpuState::csr (enum_csr in function-model/include/CSR.h).
static constexpr int kTrapCsr[8] = {7 /*mstatus*/, 15 /*sstatus*/, 1 /*mepc*/, 2 /*mcause*/,
                                    5 /*mtval*/,   10 /*sepc*/,    12 /*scause*/, 14 /*stval*/};

struct Bridge::Entry {
  Fact f;
  xlen_t pc_before = 0;
  uint8_t priv_before = 0;
  bool resv_valid_before = false;
  uint32_t resv_addr_before = 0;
  uint8_t rd = 0;  // destination register (0: none)
  xlen_t rd_old = 0;
#if FP
  bool fp_rd = false;            // an FP register is written: undo it
  uint8_t frd = 0;
  uint64_t frd_old = 0;
  bool fcsr_saved = false;       // fflags/frm may change (OP-FP, fused multiply-add, SYSTEM)
  uint32_t fcsr_old = 0;
#endif
  xlen_t trap_csr[8] = {};
  bool full_csr = false;
  std::vector<xlen_t> csr_full;  // only for SYSTEM instructions
  bool has_undo = false;
  uint32_t undo_addr = 0, undo_old = 0, undo_old_hi = 0;
  uint8_t undo_words = 1;        // an 8-byte store (rv64 sd, amo*.d) changes two words
  std::unique_ptr<RefCpuState> trap_after;  // state the core reached by trapping (undone speculatively)
};

namespace {
// access size of a load/store from funct3[1:0]: 8 bytes (ld, sd) exist on rv64 only
inline uint8_t access_bytes(uint32_t f3_low) { return f3_low == 3 ? (XLEN == 64 ? 8 : 4) : (uint8_t)(1u << f3_low); }
inline uint8_t amo_bytes(uint32_t funct3) { return (funct3 == 3 && XLEN == 64) ? 8 : 4; }
inline uint32_t bits(uint32_t v, int hi, int lo) { return (v >> lo) & ((1u << (hi - lo + 1)) - 1); }
inline int32_t sext(uint32_t v, int n) { return (int32_t)(v << (32 - n)) >> (32 - n); }
// immediates sign-extended to the register width (xlen_t)
inline xlen_t xs(int32_t v) { return static_cast<xlen_t>(static_cast<sxlen_t>(v)); }
inline xlen_t immI(uint32_t i) { return xs(sext(bits(i, 31, 20), 12)); }
inline xlen_t immB(uint32_t i) {
  return xs(sext((bits(i, 31, 31) << 12) | (bits(i, 7, 7) << 11) | (bits(i, 30, 25) << 5) | (bits(i, 11, 8) << 1), 13));
}
inline xlen_t immJ(uint32_t i) {
  return xs(sext((bits(i, 31, 31) << 20) | (bits(i, 19, 12) << 12) | (bits(i, 20, 20) << 11) | (bits(i, 30, 21) << 1), 21));
}
inline xlen_t immS(uint32_t i) { return xs(sext((bits(i, 31, 25) << 5) | bits(i, 11, 7), 12)); }

// Undo one entry on a state copy (CSR/regs/pc/priv/reservation). Memory is the caller's job.
void undo_into(const Bridge::Stats *, RefCpuState &st, const xlen_t *trap_csr, bool full_csr,
               const xlen_t *csr_full, xlen_t pc_before, uint8_t priv_before, bool rv,
               uint32_t ra, uint8_t rd, xlen_t rd_old) {
  if (full_csr) std::memcpy(st.csr, csr_full, sizeof st.csr);
  else for (int i = 0; i < 8; i++) st.csr[kTrapCsr[i]] = trap_csr[i];
#ifndef TB_NEG_NO_RD_UNDO
  if (rd) st.gpr[rd] = rd_old;
#endif
  st.pc = pc_before;
  st.privilege = priv_before;
  st.reserve_valid = rv;
  st.reserve_addr = ra;
}
#if FP
// F/D: does the instruction write an FP register?  LOAD-FP, the four fused multiply-adds, and OP-FP except the compares
// (funct5 0x14), the conversions to integer (0x18) and fmv.x / fclass (0x1c).
inline bool writes_fpr(uint32_t inst) {
  const uint32_t op = inst & 0x7f, f5 = inst >> 27;
  if (op == 0x07 || op == 0x43 || op == 0x47 || op == 0x4b || op == 0x4f) return true;
  return op == 0x53 && f5 != 0x14 && f5 != 0x18 && f5 != 0x1c;
}
inline bool touches_fcsr(uint32_t inst) {
  const uint32_t op = inst & 0x7f;
  return op == 0x53 || op == 0x43 || op == 0x47 || op == 0x4b || op == 0x4f || op == 0x73;
}
#endif
} // namespace

// Control-flow outcome computed from the register file, independent of the core's execution
// (used when the instruction was not executed: blocked fetch, undone trap).
static bool eval_control(xlen_t pc, uint32_t inst, const xlen_t *gpr, bool &taken, xlen_t &next) {
  const uint32_t op = bits(inst, 6, 0), f3 = bits(inst, 14, 12);
  const xlen_t a = gpr[bits(inst, 19, 15)], b = gpr[bits(inst, 24, 20)];
  if (op == 0x6f) { taken = true; next = pc + immJ(inst); return true; }
  if (op == 0x67) { taken = true; next = (a + immI(inst)) & ~xlen_t(1); return true; }
  if (op != 0x63) return false;
  switch (f3) {
  case 0: taken = a == b; break;
  case 1: taken = a != b; break;
  case 4: taken = (sxlen_t)a < (sxlen_t)b; break;
  case 5: taken = (sxlen_t)a >= (sxlen_t)b; break;
  case 6: taken = a < b; break;
  case 7: taken = a >= b; break;
  default: taken = false; break;
  }
  next = taken ? pc + immB(inst) : pc + 4;
  return true;
}

Bridge::Bridge(RefCpuContext *ctx, uint32_t ram_base, uint32_t ram_bytes)
    : ctx_(ctx), ram_base_(ram_base), ram_bytes_(ram_bytes) {}

bool Bridge::in_ram(uint32_t addr, uint32_t size) const {
  return addr >= ram_base_ && (uint64_t)addr + size <= (uint64_t)ram_base_ + ram_bytes_;
}

bool Bridge::read_word(uint32_t paddr, uint32_t &word) const {
  const uint32_t a = paddr & ~3u;
  if (in_ram(a, 4)) {
    word = refcpu_load_word(ctx_, a);
    return true;
  }
  if (boot_bytes_ && a >= boot_base_ && (uint64_t)a + 4 <= (uint64_t)boot_base_ + boot_bytes_) {
    word = refcpu_get_io_word(ctx_, a);
    return true;
  }
  return false;
}

const Fact *Bridge::find(uint64_t uid) const {
  for (auto *e : chain_) if (e->f.uid == uid) return &e->f;
  return nullptr;
}

const Fact &Bridge::dispatch(uint64_t uid, xlen_t pc, uint32_t inst) {
  auto *e = new Entry();
  e->f.uid = uid;
  e->f.pc = pc;
  e->f.inst = inst;
  RefCpuState st;
  refcpu_get_state(ctx_, &st);
  // The undo log restores pc to what it was before this dispatch, forced jump included.
  e->pc_before = st.pc;
  e->priv_before = st.privilege;
  e->resv_valid_before = st.reserve_valid;
  e->resv_addr_before = st.reserve_addr;
  for (int i = 0; i < 8; i++) e->trap_csr[i] = st.csr[kTrapCsr[i]];

  if (st.pc != pc) {
    e->f.forced_jump = true;
    st.pc = pc;
    refcpu_set_state(ctx_, &st);
    stats.forced_jumps++;
  }
  stats.dispatched++;

  const uint32_t opcode = bits(inst, 6, 0);
  const uint32_t rd = bits(inst, 11, 7), rs1 = bits(inst, 19, 15), rs2 = bits(inst, 24, 20);
  const uint32_t funct3 = bits(inst, 14, 12);
  e->f.is_br = (opcode == 0x63);
  const bool is_amo = (opcode == 0x2f);
  const bool is_lr = is_amo && bits(inst, 31, 27) == 2;
  e->f.rs2_value = st.gpr[rs2];
  e->f.rs1_value = st.gpr[rs1];
  if (opcode == 0x73) {  // SYSTEM: save the whole CSR file
    e->full_csr = true;
    e->csr_full.assign(st.csr, st.csr + 21);
    e->f.csr_wdata = (funct3 & 4) ? rs1 : st.gpr[rs1];  // csrr*i: zimm in the rs1 field
  }
  // Destination register undo (x0 never changes)
#if FP
  const bool fp_dest = writes_fpr(inst);
  if (fp_dest) {
    e->fp_rd = true;
    e->frd = (uint8_t)rd;
    e->frd_old = st.fpr[rd];
  }
  if (touches_fcsr(inst)) { e->fcsr_saved = true; e->fcsr_old = st.fcsr; }
  const bool writes_rd = opcode != 0x63 && opcode != 0x23 && opcode != 0x27 && opcode != 0x0f && !fp_dest;
#else
  const bool writes_rd = opcode != 0x63 && opcode != 0x23 && opcode != 0x0f;
#endif
  if (writes_rd && rd != 0) {
    e->rd = (uint8_t)rd;
    e->rd_old = st.gpr[rd];
  }

#if FP
  const bool fp_ld = opcode == 0x07, fp_st = opcode == 0x27;   // flw/fld, fsw/fsd: funct3 2 / 3 as lw / ld
  if (fp_st) e->f.rs2_value = st.fpr[rs2];
#else
  constexpr bool fp_ld = false, fp_st = false;
#endif
  if (opcode == 0x03 || fp_ld || is_lr) {
    e->f.is_load = true;
    e->f.vaddr = (opcode == 0x03 || fp_ld) ? st.gpr[rs1] + immI(inst) : st.gpr[rs1];
    e->f.size = (opcode == 0x03 || fp_ld) ? access_bytes(funct3 & 3) : amo_bytes(funct3);
  } else if (opcode == 0x23 || fp_st || is_amo) {
    e->f.is_store = true;
    e->f.vaddr = (opcode == 0x23 || fp_st) ? st.gpr[rs1] + immS(inst) : st.gpr[rs1];
    e->f.size = (opcode == 0x23 || fp_st) ? (funct3 <= 3 ? access_bytes(funct3) : 4) : amo_bytes(funct3);
#if FP
    e->f.store_data = fp_st ? st.fpr[rs2] : st.gpr[rs2];
#else
    e->f.store_data = st.gpr[rs2];
#endif
    if (e->f.size == 1) e->f.store_data &= 0xFF;
    if (e->f.size == 2) e->f.store_data &= 0xFFFF;
    if (e->f.size == 4) e->f.store_data &= 0xFFFFFFFFu;
  }

  Blocked blk = Blocked::None;
  // Virtual -> physical for the checks below (identity when translation is off). A page fault
  // is not "blocked": the core executes the access and raises the exception itself.
  uint32_t pc_pa = static_cast<uint32_t>(pc);
  const bool pc_xlate_ok = refcpu_translate_data(ctx_, pc, 0, &pc_pa);
  uint32_t data_pa = static_cast<uint32_t>(e->f.vaddr);
  bool data_xlate_ok = true;
  if (e->f.is_load || e->f.is_store)
    data_xlate_ok = refcpu_translate_data(ctx_, e->f.vaddr, e->f.is_store ? 2 : 1, &data_pa);
  const bool in_boot = boot_bytes_ && pc >= boot_base_ && (uint64_t)pc + 4 <= (uint64_t)boot_base_ + boot_bytes_;   // (pc >= 2^32 is never in the boot ROM)
  if (pc_xlate_ok && !in_ram(pc_pa, 4) && !in_boot) blk = Blocked::PcOutOfRam;
  else if (inst == 0x00100073) blk = Blocked::Ebreak;
  else if (e->f.is_load || e->f.is_store) {
    const xlen_t a = e->f.vaddr;
    const uint32_t sz = e->f.size;
    if ((sz == 2 && (a & 1)) || (sz == 4 && (a & 3)) || (sz == 8 && (a & 7))) blk = Blocked::MemMisalign;
    else if (data_xlate_ok && !in_ram(data_pa, sz)) blk = Blocked::MemOutOfRam;
  }
  e->f.blocked = blk;

  if (blk != Blocked::None) {
    stats.blocked[(int)blk]++;
    // Not executed. The state stays as before (pc nominal +4); the next dispatch forces
    // a jump if the window continues elsewhere. rd keeps its old value: wrong-path only.
    e->rd = 0;
    bool tk; xlen_t nx;
    if (eval_control(pc, inst, st.gpr, tk, nx)) { e->f.br_taken = tk; e->f.control_valid = true; st.pc = nx; }
    else st.pc = pc + 4;
    refcpu_set_state(ctx_, &st);
    e->f.next_pc = st.pc;
    chain_.push_back(e);
    return e->f;
  }

  if (e->f.is_store && data_xlate_ok) {
    const uint32_t wa = data_pa & ~3u;
    e->has_undo = true;
    e->undo_addr = wa;
    e->undo_old = refcpu_load_word(ctx_, wa);
    if (e->f.size == 8) {   // sd / amo*.d: two aligned words
      e->undo_words = 2;
      e->undo_old_hi = refcpu_load_word(ctx_, wa + 4);
    }
  }

  refcpu_set_sim_end(ctx_, false);
  refcpu_set_sim_time(ctx_, static_cast<uint64_t>(sim_time));   // mtime/rdtime follow the timing model, otherwise machine time stays 0 and Linux never sees a tick
  refcpu_step(ctx_, 1);
  RefCpuStepInfo info{};
  refcpu_get_step_info(ctx_, &info);
  RefCpuState after{};
  refcpu_get_state(ctx_, &after);
#if FP
  e->f.fs_dirty = ((after.csr[7] >> 13) & 3) == 3 && ((st.csr[7] >> 13) & 3) != 3;   // (csr index 7 = mstatus)
#endif

  if (info.is_exception) {
    // Speculative trap: undo at once (state and store), continue at pc+4.
    e->f.trapped = true;
    e->trap_after.reset(new RefCpuState(after));
    e->f.trap_cause = (uint8_t)(((after.csr[1] == pc) ? after.csr[2] : after.csr[12]) & 0x3f);   // mcause if the trap went to M (mepc = pc), else scause
    stats.trapped++;
    if (e->has_undo) { refcpu_store_word(ctx_, e->undo_addr, e->undo_old); if (e->undo_words == 2) refcpu_store_word(ctx_, e->undo_addr + 4, e->undo_old_hi); }
    e->has_undo = false;
    // A trap writes only pc, privilege and the trap CSRs; rd is untouched.
    undo_into(&stats, after, e->trap_csr, e->full_csr, e->csr_full.data(), e->pc_before,
              e->priv_before, e->resv_valid_before, e->resv_addr_before, 0, 0);
    after.pc = pc + 4;
    { bool tk; xlen_t nx;
      if (eval_control(pc, inst, after.gpr, tk, nx)) { e->f.br_taken = tk; e->f.control_valid = true; after.pc = nx; } }
    refcpu_set_state(ctx_, &after);
    e->rd = 0;
    e->f.next_pc = after.pc;
    chain_.push_back(e);
    return e->f;
  }

  e->f.next_pc = after.pc;
  e->f.control_valid = true;  // executed: the core's outcome
  if (e->rd != 0 || (writes_rd && rd != 0)) {
    e->f.rd_valid = true;
    e->f.rd = rd;
    e->f.rd_value = after.gpr[rd];
  }
#if FP
  if (fp_dest) {
    e->f.rd_valid = true;
    e->f.rd_is_fp = true;
    e->f.rd = rd;
    e->f.rd_value = after.fpr[rd];
  }
#endif
  if (e->f.is_br) e->f.br_taken = info.br_taken;
  if (after.store && e->f.is_store) {  // plain stores and AMOs; a failed SC does not set store
    e->f.has_mem_write = true;
    e->f.mem_waddr = after.store_addr & ~3u;   // (debug only: the first word of an 8-byte store)
    e->f.mem_wdata = refcpu_load_word(ctx_, e->f.mem_waddr);
    e->f.mem_old = e->has_undo && e->undo_addr == e->f.mem_waddr ? e->undo_old : e->f.mem_wdata;
  }
  chain_.push_back(e);
  return e->f;
}

void Bridge::rewind_after(int64_t uid) {
  stats.rewinds++;
  // Position, not uid order: some wrong-path entries carry uid 0, so uids are not monotonic along the chain.
  size_t keep = 0;  // number of entries kept (the oldest ones, up to and including `uid`)
  if (uid >= 0) {
    keep = chain_.size();
    for (size_t i = 0; i < chain_.size(); i++)
      if ((int64_t)chain_[i]->f.uid == uid) { keep = i + 1; break; }
    if (keep == chain_.size() && (chain_.empty() || (int64_t)chain_.back()->f.uid <= uid)) return;
    if (keep == chain_.size() && chain_[keep - 1]->f.uid != (uint64_t)uid) {
      // not in the chain (already retired): keep the entries that are older by uid
      keep = 0;
      while (keep < chain_.size() && (int64_t)chain_[keep]->f.uid <= uid) keep++;
    }
  }
  if (chain_.size() <= keep) return;
  RefCpuState st;
  refcpu_get_state(ctx_, &st);
  // youngest first
  while (chain_.size() > keep) {
    Entry *e = chain_.back();
    chain_.pop_back();
#ifndef TB_NEG_NO_STORE_UNDO
    if (e->has_undo) { refcpu_store_word(ctx_, e->undo_addr, e->undo_old); if (e->undo_words == 2) refcpu_store_word(ctx_, e->undo_addr + 4, e->undo_old_hi); }
#endif
    undo_into(&stats, st, e->trap_csr, e->full_csr, e->csr_full.data(), e->pc_before, e->priv_before,
              e->resv_valid_before, e->resv_addr_before, e->rd, e->rd_old);
#if FP
    if (e->fp_rd) st.fpr[e->frd] = e->frd_old;
    if (e->fcsr_saved) st.fcsr = e->fcsr_old;
#endif
    stats.entries_dropped++;
    delete e;
  }
  refcpu_set_state(ctx_, &st);
}

void Bridge::commit(uint64_t uid) {
  if (chain_.empty() || chain_.front()->f.uid != uid) return;
  delete chain_.front();
  chain_.pop_front();
}

bool Bridge::patch_rd(uint64_t uid, xlen_t value) {
  Entry *t = nullptr;
  for (auto *e : chain_) if (e->f.uid == uid) { t = e; break; }
  if (!t) return false;
  const uint32_t rd = bits(t->f.inst, 11, 7);
  if (rd == 0) return false;
  struct Re { uint64_t uid; xlen_t pc; uint32_t inst; };
  std::vector<Re> younger;
  for (auto *e : chain_) if (e->f.uid > uid) younger.push_back({e->f.uid, e->f.pc, e->f.inst});
  const int64_t saved = stats.rewinds;
  rewind_after((int64_t)uid);
  stats.rewinds = saved;  // not a pipeline rewind
  RefCpuState st;
  refcpu_get_state(ctx_, &st);
  if (t->rd == 0) {  // not executed (blocked): make the write undoable
    t->rd = (uint8_t)rd;
    t->rd_old = st.gpr[rd];
  }
  st.gpr[rd] = value;
  refcpu_set_state(ctx_, &st);
  t->f.rd_valid = true;
  t->f.rd = (uint8_t)rd;
  t->f.rd_value = value;
  stats.patches++;
  const uint64_t d = stats.dispatched, fj = stats.forced_jumps;
  for (const auto &r : younger) dispatch(r.uid, r.pc, r.inst);
  stats.replayed += younger.size();
  stats.dispatched = d;  // replays are not new dispatches
  stats.forced_jumps = fj;
  return true;
}

void Bridge::commit_flush(uint64_t uid) {
  rewind_after((int64_t)uid);
  stats.rewinds--;  // counted as a flush, not a mispredict rewind
  stats.flush_commits++;
  if (chain_.empty() || chain_.back()->f.uid != uid) return;
  Entry *x = chain_.back();
  static const bool neg = std::getenv("TB_NEG_NO_TRAP_STATE") != nullptr;  // negative control
  if (x->trap_after && !neg) {
    refcpu_set_state(ctx_, x->trap_after.get());
    stats.trap_commits++;
  }
  // X retires: every older entry has already retired, so it is the only one left.
  while (!chain_.empty()) { delete chain_.front(); chain_.pop_front(); }
}

bool Bridge::take_interrupt(xlen_t cause, uint8_t target_priv, uint32_t pending_snapshot) {
  rewind_after(-1);
  stats.rewinds--;
  stats.interrupts++;
  return refcpu_take_forced_interrupt(ctx_, cause, target_priv, pending_snapshot);
}

uint64_t Bridge::oldest_uid() const { return chain_.front()->f.uid; }
size_t Bridge::kill_older_than(uint64_t uid) {
  size_t killed = 0;
  while (killed < chain_.size() && chain_[killed]->f.uid != uid) killed++;  // entries before the committing one
  if (killed == chain_.size()) return 0;  // not in the chain
  if (!killed) return 0;
  struct Re { uint64_t uid; xlen_t pc; uint32_t inst; };
  std::vector<Re> younger;
  for (size_t i = killed; i < chain_.size(); i++) younger.push_back({chain_[i]->f.uid, chain_[i]->f.pc, chain_[i]->f.inst});
  const int64_t saved = stats.rewinds;
  rewind_after(-1);  // undoes the killed entries and everything younger (the chain starts with the killed ones)
  stats.rewinds = saved;
  const uint64_t d = stats.dispatched, fj = stats.forced_jumps;
  for (const auto &r : younger) dispatch(r.uid, r.pc, r.inst);
  stats.replayed += younger.size();
  stats.dispatched = d;
  stats.forced_jumps = fj;
  return killed;
}

void Bridge::drop_oldest() { delete chain_.front(); chain_.pop_front(); }

size_t Bridge::undo_bytes() const { return sizeof(Entry) - sizeof(Fact); }  // fixed part; +84 B for SYSTEM

} // namespace tb

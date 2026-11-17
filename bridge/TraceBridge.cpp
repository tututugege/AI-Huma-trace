#include "TraceBridge.h"
#include "InstDecode.h"
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
  bool fp_rd = false;            // an FP register is written: undo it
  uint8_t frd = 0;
  uint64_t frd_old = 0;
  bool fcsr_saved = false;       // fflags/frm may change (OP-FP, fused multiply-add, SYSTEM)
  uint32_t fcsr_old = 0;
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
inline uint8_t access_bytes(uint32_t f3_low) { return f3_low == 3 ? 8 : (uint8_t)(1u << f3_low); }   // 8 bytes: ld / sd (rv64), fld / fsd (F/D)
inline uint8_t amo_bytes(uint32_t funct3) { return (funct3 == 3) ? 8 : 4; }
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
void undo_into(RefCpuState &st, const xlen_t *trap_csr, bool full_csr,
               const xlen_t *csr_full, xlen_t pc_before, uint8_t priv_before, bool rv,
               uint32_t ra, uint8_t rd, xlen_t rd_old) {
  if (full_csr) std::memcpy(st.csr, csr_full, sizeof st.csr);
  else for (int i = 0; i < 8; i++) st.csr[kTrapCsr[i]] = trap_csr[i];
  if (rd) st.gpr[rd] = rd_old;
  st.pc = pc_before;
  st.privilege = priv_before;
  st.reserve_valid = rv;
  st.reserve_addr = ra;
}
// F/D: does the instruction write an FP register?  LOAD-FP, the four fused multiply-adds, and OP-FP except the compares
// (funct5 0x14), the conversions to integer (0x18) and fmv.x / fclass (0x1c).
} // namespace

// Control-flow outcome computed from the register file, independent of the core's execution
// (used when the instruction was not executed: blocked fetch, undone trap).
static bool eval_control(xlen_t pc, uint32_t inst, const xlen_t *gpr, bool &taken, xlen_t &next, uint32_t len) {
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
  next = taken ? pc + immB(inst) : pc + len;
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

// The chain is in dispatch order, which is ascending uid (a uid is dispatched once): binary search.
const Fact *Bridge::find_slow(uint64_t uid) const {
  size_t lo = 0, hi = chain_.size();
  while (lo < hi) {
    const size_t mid = (lo + hi) / 2;
    if (chain_[mid]->f.uid < uid) lo = mid + 1; else hi = mid;
  }
  const Fact *r = lo < chain_.size() && chain_[lo]->f.uid == uid ? &chain_[lo]->f : nullptr;
  // The chain is not always sorted by uid (an instruction can be dispatched again out of order after a rewind / replay): a miss of the binary search
  // is confirmed by a linear scan (a hit is always right).  Without this, Linux lost facts and the kernel oopsed (EXP-036 regression, fixed 2026-11-13).
  if (!r) for (auto *e : chain_) if (e->f.uid == uid) { r = &e->f; break; }
  return r;
}

// Entries come from a free list: an entry lives from dispatch to commit / rewind, so the list stays as long as the window.
Bridge::Entry *Bridge::alloc_entry() {
  if (pool_.empty()) return new Entry();
  Entry *e = pool_.back();
  pool_.pop_back();
  *e = Entry();
  return e;
}

void Bridge::free_entry(Entry *e) {
  Entry *&s = slot_[e->f.uid & kSlotMask];
  if (s == e) s = nullptr;
  pool_.push_back(e);
}

void Bridge::push(Entry *e) {
  slot_[e->f.uid & kSlotMask] = e;
  chain_.push_back(e);
}

const Fact *Bridge::find(uint64_t uid) const {
  const Entry *s = slot_[uid & kSlotMask];
  return s && s->f.uid == uid ? &s->f : find_slow(uid);
}

const Fact &Bridge::dispatch(uint64_t uid, xlen_t pc, uint32_t inst, uint32_t len) {
  Entry *e = alloc_entry();
  e->f.uid = uid;
  e->f.pc = pc;
  e->f.inst = inst;
  e->f.len = static_cast<uint8_t>(len);
  // The function model's fields are read one by one (no copy of its state); gpr / fpr are its register arrays.
  const xlen_t *gpr = refcpu_gpr(ctx_);
  const uint64_t *fpr = refcpu_fpr(ctx_);
  const xlen_t mstatus_before = refcpu_csr(ctx_, 7);   // (csr index 7 = mstatus)
  // The undo log restores pc to what it was before this dispatch, forced jump included.
  e->pc_before = refcpu_pc(ctx_);
  e->priv_before = refcpu_privilege(ctx_);
  refcpu_reservation(ctx_, &e->resv_valid_before, &e->resv_addr_before);
  for (int i = 0; i < 8; i++) e->trap_csr[i] = refcpu_csr(ctx_, kTrapCsr[i]);

  if (e->pc_before != pc) {
    e->f.forced_jump = true;
    refcpu_set_pc(ctx_, pc);
  }

  const uint32_t opcode = bits(inst, 6, 0);
  const uint32_t rd = bits(inst, 11, 7), rs1 = bits(inst, 19, 15), rs2 = bits(inst, 24, 20);
  const uint32_t funct3 = bits(inst, 14, 12);
  const DecodedInst di = decode_inst(inst);   // the shared decoder (InstDecode.h)
  e->f.is_br = (opcode == 0x63);
  e->f.rs2_value = gpr[rs2];
  e->f.rs1_value = gpr[rs1];
  if (opcode == 0x73) {  // SYSTEM: save the whole CSR file
    e->full_csr = true;
    e->csr_full.resize(21);
    for (int i = 0; i < 21; i++) e->csr_full[i] = refcpu_csr(ctx_, i);
    e->f.csr_wdata = (funct3 & 4) ? rs1 : gpr[rs1];  // csrr*i: zimm in the rs1 field
  }
  // Destination register undo (x0 never changes)
  const bool fp_dest = di.writes_fpr;
  if (fp_dest) {
    e->fp_rd = true;
    e->frd = (uint8_t)rd;
    e->frd_old = fpr[rd];
  }
  if (di.touches_fcsr) { e->fcsr_saved = true; e->fcsr_old = refcpu_fcsr(ctx_); }
  const bool writes_rd = di.writes_xreg;
  if (writes_rd && rd != 0) {
    e->rd = (uint8_t)rd;
    e->rd_old = gpr[rd];
  }

  const bool fp_ld = opcode == 0x07, fp_st = opcode == 0x27;   // flw/fld, fsw/fsd: funct3 2 / 3 as lw / ld
  if (fp_st) e->f.rs2_value = fpr[rs2];
  if (di.mem_load) {
    e->f.is_load = true;
    e->f.vaddr = (opcode == 0x03 || fp_ld) ? gpr[rs1] + immI(inst) : gpr[rs1];
    e->f.size = (opcode == 0x03 || fp_ld) ? access_bytes(funct3 & 3) : amo_bytes(funct3);
  } else if (di.mem_store) {
    e->f.is_store = true;
    e->f.vaddr = (opcode == 0x23 || fp_st) ? gpr[rs1] + immS(inst) : gpr[rs1];
    e->f.size = (opcode == 0x23 || fp_st) ? (funct3 <= 3 ? access_bytes(funct3) : 4) : amo_bytes(funct3);
    e->f.store_data = fp_st ? fpr[rs2] : gpr[rs2];
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
  const bool in_boot = boot_bytes_ && pc >= boot_base_ && (uint64_t)pc + len <= (uint64_t)boot_base_ + boot_bytes_;   // (pc >= 2^32 is never in the boot ROM)
  if (pc_xlate_ok && !in_ram(pc_pa, 4) && !in_boot) blk = Blocked::PcOutOfRam;
  else if (inst == 0x00100073) blk = Blocked::Ebreak;
  else if (e->f.is_load || e->f.is_store) {
    const xlen_t a = e->f.vaddr;
    const uint32_t sz = e->f.size;
    // An FP load / store with mstatus.FS = Off is an illegal instruction before its address is looked at (the RTL flags it at completion, which stalls the
    // dispatch while it sits in the ROB window): it must reach the function model to trap, not be "blocked" for a misaligned / out-of-RAM address.
    bool fp_mem_fs_off = false;
    { const uint32_t opc = inst & 0x7f; fp_mem_fs_off = (opc == 0x07 || opc == 0x27) && ((mstatus_before >> 13) & 3) == 0; }   // (csr index 7 = mstatus)
    if (fp_mem_fs_off) { /* executed: traps */ }
    else if ((sz == 2 && (a & 1)) || (sz == 4 && (a & 3)) || (sz == 8 && (a & 7))) blk = Blocked::MemMisalign;
    else if (data_xlate_ok && !in_ram(data_pa, sz)) blk = Blocked::MemOutOfRam;
  }
  e->f.blocked = blk;

  if (blk != Blocked::None) {
    // Not executed. The state stays as before (pc nominal +4); the next dispatch forces
    // a jump if the window continues elsewhere. rd keeps its old value: wrong-path only.
    e->rd = 0;
    bool tk; xlen_t nx;
    xlen_t npc = pc + len;
    if (eval_control(pc, inst, gpr, tk, nx, len)) { e->f.br_taken = tk; e->f.control_valid = true; npc = nx; }
    refcpu_set_pc(ctx_, npc);
    e->f.next_pc = npc;
    push(e);
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
  const xlen_t mstatus_after = refcpu_csr(ctx_, 7);
  e->f.fs_dirty = ((mstatus_after >> 13) & 3) == 3 && ((mstatus_before >> 13) & 3) != 3;

  if (info.is_exception) {
    RefCpuState after{};   // (rare: the whole state, kept as the trap's result)
    refcpu_get_state(ctx_, &after);
    // Speculative trap: undo at once (state and store), continue at pc+4.
    e->f.trapped = true;
    e->trap_after.reset(new RefCpuState(after));
    e->f.trap_cause = (uint8_t)(((after.csr[1] == pc) ? after.csr[2] : after.csr[12]) & 0x3f);   // mcause if the trap went to M (mepc = pc), else scause
    if (e->has_undo) { refcpu_store_word(ctx_, e->undo_addr, e->undo_old); if (e->undo_words == 2) refcpu_store_word(ctx_, e->undo_addr + 4, e->undo_old_hi); }
    e->has_undo = false;
    // A trap writes only pc, privilege and the trap CSRs; rd is untouched.
    undo_into(after, e->trap_csr, e->full_csr, e->csr_full.data(), e->pc_before,
              e->priv_before, e->resv_valid_before, e->resv_addr_before, 0, 0);
    after.pc = pc + len;
    { bool tk; xlen_t nx;
      if (eval_control(pc, inst, after.gpr, tk, nx, len)) { e->f.br_taken = tk; e->f.control_valid = true; after.pc = nx; } }
    refcpu_set_state(ctx_, &after);
    e->rd = 0;
    e->f.next_pc = after.pc;
    push(e);
    return e->f;
  }

  bool did_store = false;
  uint32_t store_addr = 0;
  refcpu_store_info(ctx_, &did_store, &store_addr);
  e->f.next_pc = refcpu_pc(ctx_);
  e->f.control_valid = true;  // executed: the core's outcome
  if (e->rd != 0 || (writes_rd && rd != 0)) {
    e->f.rd_valid = true;
    e->f.rd = rd;
    e->f.rd_value = gpr[rd];
  }
  if (fp_dest) {
    e->f.rd_valid = true;
    e->f.rd_is_fp = true;
    e->f.rd = rd;
    e->f.rd_value = fpr[rd];
  }
  if (e->f.is_br) e->f.br_taken = info.br_taken;
  if (did_store && e->f.is_store) {  // plain stores and AMOs; a failed SC does not set store
    e->f.has_mem_write = true;
    constexpr uint32_t XB = XLEN / 8;   // the unit of the stale-value model (EXP-020): one word on rv32, one aligned dword on rv64
    e->f.mem_waddr = store_addr & ~(XB - 1);
    auto unit = [&](uint32_t a) { xlen_t v = refcpu_load_word(ctx_, a); v |= xlen_t(refcpu_load_word(ctx_, a + 4)) << 32; return v; };
    e->f.mem_wdata = unit(e->f.mem_waddr);
    e->f.mem_old = e->f.mem_wdata;   // the words the store undid hold the value before it
    if (e->has_undo) {
      for (uint32_t k = 0; k < XB; k += 4) {
        const uint32_t wa = e->f.mem_waddr + k;
        uint32_t old = 0; bool hit = false;
        if (e->undo_addr == wa) { old = e->undo_old; hit = true; }
        else if (e->undo_words == 2 && e->undo_addr + 4 == wa) { old = e->undo_old_hi; hit = true; }
        if (hit) e->f.mem_old = (e->f.mem_old & ~(xlen_t(0xffffffffu) << (8 * k))) | (xlen_t(old) << (8 * k));
      }
    }
  }
  push(e);
  return e->f;
}

void Bridge::rewind_after(int64_t uid) {
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
    if (e->has_undo) { refcpu_store_word(ctx_, e->undo_addr, e->undo_old); if (e->undo_words == 2) refcpu_store_word(ctx_, e->undo_addr + 4, e->undo_old_hi); }
    undo_into(st, e->trap_csr, e->full_csr, e->csr_full.data(), e->pc_before, e->priv_before,
              e->resv_valid_before, e->resv_addr_before, e->rd, e->rd_old);
    if (e->fp_rd) st.fpr[e->frd] = e->frd_old;
    if (e->fcsr_saved) st.fcsr = e->fcsr_old;
    free_entry(e);
  }
  refcpu_set_state(ctx_, &st);
}

void Bridge::commit(uint64_t uid) {
  if (chain_.empty() || chain_.front()->f.uid != uid) return;
  free_entry(chain_.front());
  chain_.pop_front();
}

bool Bridge::patch_rd(uint64_t uid, xlen_t value) {
  Entry *t = nullptr;
  for (auto *e : chain_) if (e->f.uid == uid) { t = e; break; }
  if (!t) return false;
  const uint32_t rd = bits(t->f.inst, 11, 7);
  if (rd == 0) return false;
  struct Re { uint64_t uid; xlen_t pc; uint32_t inst; uint32_t len; };
  std::vector<Re> younger;
  for (auto *e : chain_) if (e->f.uid > uid) younger.push_back({e->f.uid, e->f.pc, e->f.inst, e->f.len});
  rewind_after((int64_t)uid);
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
  for (const auto &r : younger) dispatch(r.uid, r.pc, r.inst, r.len);
  return true;
}

void Bridge::commit_flush(uint64_t uid) {
  rewind_after((int64_t)uid);
  if (chain_.empty() || chain_.back()->f.uid != uid) return;
  Entry *x = chain_.back();
  if (x->trap_after) refcpu_set_state(ctx_, x->trap_after.get());
  // X retires: every older entry has already retired, so it is the only one left.
  while (!chain_.empty()) { free_entry(chain_.front()); chain_.pop_front(); }
}

bool Bridge::take_interrupt(xlen_t cause, uint8_t target_priv, uint32_t pending_snapshot) {
  rewind_after(-1);
  return refcpu_take_forced_interrupt(ctx_, cause, target_priv, pending_snapshot);
}

size_t Bridge::kill_older_than(uint64_t uid) {
  size_t killed = 0;
  while (killed < chain_.size() && chain_[killed]->f.uid != uid) killed++;  // entries before the committing one
  if (killed == chain_.size()) return 0;  // not in the chain
  if (!killed) return 0;
  struct Re { uint64_t uid; xlen_t pc; uint32_t inst; uint32_t len; };
  std::vector<Re> younger;
  for (size_t i = killed; i < chain_.size(); i++) younger.push_back({chain_[i]->f.uid, chain_[i]->f.pc, chain_[i]->f.inst, chain_[i]->f.len});
  rewind_after(-1);  // undoes the killed entries and everything younger (the chain starts with the killed ones)
  for (const auto &r : younger) dispatch(r.uid, r.pc, r.inst, r.len);
  return killed;
}

} // namespace tb

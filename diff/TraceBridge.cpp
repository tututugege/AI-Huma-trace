#include "TraceBridge.h"
#include "refcpu_api.h"
#include <cstring>
#include <vector>

namespace tb {

// Undo log of one dispatched instruction (~60 bytes, no full register/CSR copy):
//   pc / privilege / reservation before it, the old value of its destination register,
//   the CSRs a trap writes, the whole CSR file only for SYSTEM instructions,
//   and the old memory word of a store/AMO.
// Indices into RefCpuState::csr (enum_csr in GenSimpoint/include/CSR.h).
static constexpr int kTrapCsr[8] = {7 /*mstatus*/, 15 /*sstatus*/, 1 /*mepc*/, 2 /*mcause*/,
                                    5 /*mtval*/,   10 /*sepc*/,    12 /*scause*/, 14 /*stval*/};

struct Bridge::Entry {
  Fact f;
  uint32_t pc_before = 0;
  uint8_t priv_before = 0;
  bool resv_valid_before = false;
  uint32_t resv_addr_before = 0;
  uint8_t rd = 0;  // destination register (0: none)
  uint32_t rd_old = 0;
  uint32_t trap_csr[8] = {};
  bool full_csr = false;
  std::vector<uint32_t> csr_full;  // only for SYSTEM instructions
  bool has_undo = false;
  uint32_t undo_addr = 0, undo_old = 0;
};

namespace {
inline uint32_t bits(uint32_t v, int hi, int lo) { return (v >> lo) & ((1u << (hi - lo + 1)) - 1); }
inline int32_t sext(uint32_t v, int n) { return (int32_t)(v << (32 - n)) >> (32 - n); }
inline uint32_t immI(uint32_t i) { return (uint32_t)sext(bits(i, 31, 20), 12); }
inline uint32_t immS(uint32_t i) { return (uint32_t)sext((bits(i, 31, 25) << 5) | bits(i, 11, 7), 12); }

// Undo one entry on a state copy (CSR/regs/pc/priv/reservation). Memory is the caller's job.
void undo_into(const Bridge::Stats *, RefCpuState &st, const uint32_t *trap_csr, bool full_csr,
               const uint32_t *csr_full, uint32_t pc_before, uint8_t priv_before, bool rv,
               uint32_t ra, uint8_t rd, uint32_t rd_old) {
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
} // namespace

Bridge::Bridge(RefCpuContext *ctx, uint32_t ram_base, uint32_t ram_bytes)
    : ctx_(ctx), ram_base_(ram_base), ram_bytes_(ram_bytes) {}

bool Bridge::in_ram(uint32_t addr, uint32_t size) const {
  return addr >= ram_base_ && (uint64_t)addr + size <= (uint64_t)ram_base_ + ram_bytes_;
}

const Fact *Bridge::find(uint64_t uid) const {
  for (auto *e : chain_) if (e->f.uid == uid) return &e->f;
  return nullptr;
}

const Fact &Bridge::dispatch(uint64_t uid, uint32_t pc, uint32_t inst) {
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
  if (opcode == 0x73) {  // SYSTEM: save the whole CSR file
    e->full_csr = true;
    e->csr_full.assign(st.csr, st.csr + 21);
  }
  // Destination register undo (x0 never changes)
  const bool writes_rd = opcode != 0x63 && opcode != 0x23 && opcode != 0x0f;
  if (writes_rd && rd != 0) {
    e->rd = (uint8_t)rd;
    e->rd_old = st.gpr[rd];
  }

  if (opcode == 0x03 || is_lr) {
    e->f.is_load = true;
    e->f.vaddr = opcode == 0x03 ? st.gpr[rs1] + immI(inst) : st.gpr[rs1];
    e->f.size = opcode == 0x03 ? ((funct3 & 3) == 0 ? 1 : (funct3 & 3) == 1 ? 2 : 4) : 4;
  } else if (opcode == 0x23 || is_amo) {
    e->f.is_store = true;
    e->f.vaddr = opcode == 0x23 ? st.gpr[rs1] + immS(inst) : st.gpr[rs1];
    e->f.size = opcode == 0x23 ? (funct3 == 0 ? 1 : funct3 == 1 ? 2 : 4) : 4;
    e->f.store_data = st.gpr[rs2];
    if (e->f.size == 1) e->f.store_data &= 0xFF;
    if (e->f.size == 2) e->f.store_data &= 0xFFFF;
  }

  Blocked blk = Blocked::None;
  if (!in_ram(pc, 4)) blk = Blocked::PcOutOfRam;
  else if (inst == 0x00100073) blk = Blocked::Ebreak;
  else if (e->f.is_load || e->f.is_store) {
    const uint32_t a = e->f.vaddr, sz = e->f.size;
    if ((sz == 2 && (a & 1)) || (sz == 4 && (a & 3))) blk = Blocked::MemMisalign;
    else if (!in_ram(a, sz)) blk = Blocked::MemOutOfRam;
  }
  e->f.blocked = blk;

  if (blk != Blocked::None) {
    stats.blocked[(int)blk]++;
    // Not executed. The state stays as before (pc nominal +4); the next dispatch forces
    // a jump if the window continues elsewhere. rd keeps its old value: wrong-path only.
    e->rd = 0;
    st.pc = pc + 4;
    refcpu_set_state(ctx_, &st);
    e->f.next_pc = st.pc;
    chain_.push_back(e);
    return e->f;
  }

  if (e->f.is_store) {
    const uint32_t wa = e->f.vaddr & ~3u;
    e->has_undo = true;
    e->undo_addr = wa;
    e->undo_old = refcpu_load_word(ctx_, wa);
  }

  refcpu_set_sim_end(ctx_, false);
  refcpu_step(ctx_, 1);
  RefCpuStepInfo info{};
  refcpu_get_step_info(ctx_, &info);
  RefCpuState after{};
  refcpu_get_state(ctx_, &after);

  if (info.is_exception) {
    // Speculative trap: undo at once (state and store), continue at pc+4.
    e->f.trapped = true;
    stats.trapped++;
    if (e->has_undo) refcpu_store_word(ctx_, e->undo_addr, e->undo_old);
    e->has_undo = false;
    // A trap writes only pc, privilege and the trap CSRs; rd is untouched.
    undo_into(&stats, after, e->trap_csr, e->full_csr, e->csr_full.data(), e->pc_before,
              e->priv_before, e->resv_valid_before, e->resv_addr_before, 0, 0);
    after.pc = pc + 4;
    refcpu_set_state(ctx_, &after);
    e->rd = 0;
    e->f.next_pc = after.pc;
    chain_.push_back(e);
    return e->f;
  }

  e->f.next_pc = after.pc;
  if (e->rd != 0 || (writes_rd && rd != 0)) {
    e->f.rd_valid = true;
    e->f.rd = rd;
    e->f.rd_value = after.gpr[rd];
  }
  if (e->f.is_br) e->f.br_taken = info.br_taken;
  chain_.push_back(e);
  return e->f;
}

void Bridge::rewind_after(int64_t uid) {
  stats.rewinds++;
  if (chain_.empty() || (int64_t)chain_.back()->f.uid <= uid) return;
  RefCpuState st;
  refcpu_get_state(ctx_, &st);
  // youngest first
  while (!chain_.empty() && (int64_t)chain_.back()->f.uid > uid) {
    Entry *e = chain_.back();
    chain_.pop_back();
#ifndef TB_NEG_NO_STORE_UNDO
    if (e->has_undo) refcpu_store_word(ctx_, e->undo_addr, e->undo_old);
#endif
    undo_into(&stats, st, e->trap_csr, e->full_csr, e->csr_full.data(), e->pc_before, e->priv_before,
              e->resv_valid_before, e->resv_addr_before, e->rd, e->rd_old);
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

bool Bridge::patch_rd(uint64_t uid, uint32_t value) {
  Entry *t = nullptr;
  for (auto *e : chain_) if (e->f.uid == uid) { t = e; break; }
  if (!t) return false;
  const uint32_t rd = bits(t->f.inst, 11, 7);
  if (rd == 0) return false;
  struct Re { uint64_t uid; uint32_t pc, inst; };
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

uint64_t Bridge::oldest_uid() const { return chain_.front()->f.uid; }
void Bridge::drop_oldest() { delete chain_.front(); chain_.pop_front(); }

size_t Bridge::undo_bytes() const { return sizeof(Entry) - sizeof(Fact); }  // fixed part; +84 B for SYSTEM

} // namespace tb

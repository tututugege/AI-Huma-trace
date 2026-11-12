#include "Wires.h"
#include "BeginCheck.h"
#include "Ren.h"
#include "Pmu.h"
#include "config.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "util.h"

// Shared by several phases of a cycle.
static wire<1> fire[DECODE_WIDTH];
static wire<PRF_IDX_WIDTH> alloc_reg[DECODE_WIDTH];
static bool elim[DECODE_WIDTH];  // move elimination: renamed away, takes no free register

static inline int ring_distance(int from, int to) { return to >= from ? to - from : to - from + PRF_NUM; }

// Free list invariant: free slots + registers held by in-flight instructions == the managed pool.
static inline void ren_freelist_sanity(int spec_head, int commit_head, int tail) {
  constexpr int kManagedPool = PRF_NUM - (ARF_NUM + 1);
  if (move_elim_enabled()) return;  // shared registers change the pool size
  Assert(spec_head >= 0 && spec_head < PRF_NUM && commit_head >= 0 && commit_head < PRF_NUM && tail >= 0 &&
         tail < PRF_NUM && "Ren: free list pointer out of range");
  Assert(ring_distance(spec_head, tail) + ring_distance(commit_head, spec_head) == kManagedPool &&
         "Ren: free slots + pending spec must equal managed pool");
}

void Ren::init() {
  int free_count = 0;
  for (int i = 0; i < PRF_NUM; i++) {
    if (i < ARF_NUM + 1) {
      spec_RAT[i] = arch_RAT[i] = i;
      arch_cnt[i] = 1;
    } else {
      arch_cnt[i] = 0;
      free_list[free_count++] = i;
    }
  }
  free_head = free_head_commit = 0;
  free_tail = free_count;
  for (int i = 0; i < MAX_BR_NUM; i++) alloc_checkpoint_head[i] = free_head;
  ren_freelist_sanity(free_head, free_head_commit, free_tail);

  for (int i = 0; i < DECODE_WIDTH; i++) {
    inst_r[i] = inst_r_1[i] = {};
    inst_valid[i] = inst_valid_1[i] = false;
  }
  memcpy(arch_RAT_1, arch_RAT, sizeof(arch_RAT));
  memcpy(spec_RAT_1, spec_RAT, sizeof(spec_RAT));
  memcpy(free_list_1, free_list, sizeof(free_list));
  free_head_1 = free_head;
  free_head_commit_1 = free_head_commit;
  free_tail_1 = free_tail;
  memcpy(alloc_checkpoint_head_1, alloc_checkpoint_head, sizeof(alloc_checkpoint_head));
}

// The working copies already equal the registered state (seq() copies them back).
void Ren::comb_begin() {
  BEGIN_EQ(inst_r_1, inst_r);
  BEGIN_EQ(inst_valid_1, inst_valid);
  BEGIN_EQ(spec_RAT_1, spec_RAT);
  BEGIN_EQ(arch_RAT_1, arch_RAT);
  BEGIN_EQ(RAT_checkpoint_1, RAT_checkpoint);
  BEGIN_EQ(free_list_1, free_list);
  BEGIN_EQ(free_head_1, free_head);
  BEGIN_EQ(free_head_commit_1, free_head_commit);
  BEGIN_EQ(free_tail_1, free_tail);
  BEGIN_EQ(alloc_checkpoint_head_1, alloc_checkpoint_head);
}

// Move elimination bound. The architectural state after a move commits equals the speculative RAT right after its
// rename, so the architectural reference count of the source register never exceeds the speculative one. A move is
// eliminated only when its source comes straight from the speculative RAT (no older instruction of the group writes
// it) and  (RAT entries naming it) + (older instructions of the group with a destination) + 1  <= MOVE_ELIM_MAX_REFS;
// older writers can add at most one reference each, so the bound holds without looking at what they write.
bool Ren::elim_refs_ok(int i) const {
  const int s1 = inst_r[i]->src1_areg;
  int older = 0;
  for (int j = 0; j < i; j++) {
    if (!inst_valid[j] || !inst_r[j].dest_en) continue;
    if (inst_r[j]->dest_areg != 0 && inst_r[j]->dest_areg == s1) return false;  // source bypassed inside the group
    older++;
  }
  int refs = 0;
  for (int e = 0; e < ARF_NUM + 1; e++)
    if (spec_RAT[e] == spec_RAT[s1]) refs++;
  const bool ok = refs + older + 1 <= MOVE_ELIM_MAX_REFS;
  return ok;
}

// Pick destination physical registers from the free list, in order. The first instruction that finds it empty stalls
// itself and everything behind it (ren2dis.valid = 0).
void Ren::comb_alloc() {
  ren_freelist_sanity(free_head, free_head_commit, free_tail);
  int head = free_head;
  int left = ring_distance(free_head, free_tail);
  bool stall = false;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    alloc_reg[i] = 0;
    elim[i] = false;
    wires->ren2dis.valid[i] = false;
    if (!inst_valid[i]) continue;
    if (move_elim_enabled() && is_move_shape(inst_r[i]) && elim_refs_ok(i)) {
      elim[i] = true;
      wires->ren2dis.valid[i] = !stall;
    } else if (!inst_r[i].dest_en) {
      wires->ren2dis.valid[i] = !stall;
    } else if (!stall && left > 0) {
      alloc_reg[i] = free_list[head];
      wires->ren2dis.valid[i] = true;
      LOOP_INC(head, PRF_NUM);
      left--;
    } else {
      stall = true;
    }
  }
  g_pmu_back.ren_no_preg = stall;
}

// Source/old-destination lookup in spec_RAT; an older instruction of the same group that writes the same architectural
// register (x0 excluded) takes precedence, the youngest such writer wins.
void Ren::comb_rename() {
  wire<PRF_IDX_WIDTH> dest_of[DECODE_WIDTH];  // final destination register of each slot (an eliminated move: its source)
  for (int i = 0; i < DECODE_WIDTH; i++) {
    wires->ren2dis.uop[i] = inst_r[i];
    wires->ren2dis.uop[i].dest_preg = alloc_reg[i];
    dest_of[i] = alloc_reg[i];
    // Static fields live in the uop pool: an invalid slot still holds the handle of an instruction that has long
    // left the stage, so it must not be written.
    if (!inst_valid[i]) continue;
    auto lookup = [&](int areg) {
      wire<PRF_IDX_WIDTH> p = spec_RAT[areg];
      for (int j = 0; j < i; j++)
        if (inst_valid[j] && inst_r[j].dest_en && inst_r[j]->dest_areg != 0 && inst_r[j]->dest_areg == areg)
          p = dest_of[j];
      return p;
    };
    wires->ren2dis.uop[i]->src1_preg = lookup(inst_r[i]->src1_areg);
    wires->ren2dis.uop[i]->src2_preg = lookup(inst_r[i]->src2_areg);
    wires->ren2dis.uop[i]->old_dest_preg = lookup(inst_r[i]->dest_areg);
    if (elim[i]) {
      dest_of[i] = wires->ren2dis.uop[i]->src1_preg;
      wires->ren2dis.uop[i].dest_preg = dest_of[i];
    }
  }
}

namespace {
// Commit statistics and the checkpoint warm-up / measurement window.
void count_commit(SimContext *ctx, const Uop &u) {
  ctx->perf.commit_num++;
  if (is_load(u)) ctx->perf.commit_load_num++;
  if (is_store(u)) ctx->perf.commit_store_num++;
  if (!ctx->is_ckpt) return;
  if (ctx->front_door_pending) {
    // The restore program ends with mret: the measurement counts from the checkpoint's first instruction. The commit
    // counters restart; the cycle count is kept (the memory system uses it as a clock) and its baseline moves.
    if (decode_inst_type(u->type) == MRET) {
      ctx->front_door_pending = false;
      std::cout << "[front-door] restore program committed: " << std::dec << ctx->perf.commit_num << " instructions, "
                << ctx->perf.cycle << " cycles; measuring from here" << std::endl;
      ctx->perf.commit_num = ctx->perf.commit_load_num = ctx->perf.commit_store_num = 0;
      ctx->perf.pmu.reset();
      ctx->perf.cycle_base = ctx->perf.cycle;
      if (ctx->ckpt_warmup_commit_target == 0) ctx->perf.perf_start = true;
    }
    return;
  }
  if (!ctx->perf.perf_start && ctx->perf.commit_num >= ctx->ckpt_warmup_commit_target) {
    std::cout << "[CKPT] O3 warmup done at commit " << std::dec << ctx->perf.commit_num << "/"
              << ctx->ckpt_warmup_commit_target << ". Resetting perf counters and starting measure phase." << std::endl;
    ctx->perf.perf_reset();
    ctx->perf.perf_start = true;
  }
  if (ctx->perf.perf_start && ctx->perf.commit_num >= ctx->ckpt_measure_commit_target)
    ctx->exit_reason = ExitReason::SIMPOINT;
}
}  // namespace

// ME_CHECK=1: every cycle, (1) arch_cnt equals the number of arch_RAT entries naming each register and (2) free
// registers + registers held by in-flight allocations + distinct architectural registers == PRF_NUM (no leak).
void Ren::me_check() const {
  static const bool on = std::getenv("ME_CHECK") != nullptr;
  if (!on) return;
  int seen[PRF_NUM] = {0}, distinct = 0;
  for (int a = 0; a < ARF_NUM + 1; a++) seen[arch_RAT_1[a]]++;
  for (int p = 0; p < PRF_NUM; p++) {
    Assert(seen[p] == arch_cnt[p] && "Ren(ME): arch_cnt differs from arch_RAT");
    distinct += seen[p] > 0;
  }
  Assert(ring_distance(free_head_1, free_tail_1) + ring_distance(free_head_commit_1, free_head_1) + distinct == PRF_NUM &&
         "Ren(ME): physical register leak");
  // reference-count statistics: the largest number of arch_RAT / spec_RAT entries naming one register, per cycle
  static struct Hist {
    uint64_t arch[ARF_NUM + 2] = {0}, spec[ARF_NUM + 2] = {0};
    ~Hist() {
      for (int k = 0; k < 2; k++) {
        const uint64_t *h = k ? spec : arch;
        fprintf(stderr, "[ME] max refs on one register (%s_RAT), cycles at each max:", k ? "spec" : "arch");
        for (int n = 1; n < ARF_NUM + 2; n++)
          if (h[n]) fprintf(stderr, " %d:%llu", n, (unsigned long long)h[n]);
        fprintf(stderr, "\n");
      }
    }
  } hist;
  int sp[PRF_NUM] = {0}, mx_a = 0, mx_s = 0;
  for (int a = 1; a < ARF_NUM + 1; a++) sp[spec_RAT_1[a]]++;
  for (int p = 0; p < PRF_NUM; p++) {
    if (p != 0) mx_a = std::max(mx_a, seen[p]);
    mx_s = std::max(mx_s, sp[p]);
  }
  hist.arch[std::max(mx_a, 1)]++;
  hist.spec[std::max(mx_s, 1)]++;
}

// Rename state updates: fired instructions take their register and checkpoint spec_RAT at branches, commits free old
// registers and update arch_RAT, then flush (restore from arch_RAT) or mispredict (restore a checkpoint) override
// the next-cycle spec state.
void Ren::comb_fire() {
  const bool flush = wires->rob_bcast.flush, mispred = wires->dec_bcast.mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) fire[i] = wires->ren2dis.valid[i] && wires->dis2ren.ready;

  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!fire[i]) continue;
    if (wires->ren2dis.uop[i].dest_en) {
      spec_RAT_1[inst_r[i]->dest_areg] = wires->ren2dis.uop[i].dest_preg;
      if (!elim[i]) {
        Assert(free_head_1 != free_tail_1 && "Ren: allocate when free list is empty");
        LOOP_INC(free_head_1, PRF_NUM);
      }
    }
    if (is_branch(inst_r[i]->type)) {  // the checkpoint includes this instruction's own rename, not younger ones
      const auto br_id = inst_r[i]->br_id;
      Assert(br_id < MAX_BR_NUM && "Ren: branch id out of range");
      memcpy(RAT_checkpoint_1[br_id], spec_RAT_1, sizeof(spec_RAT_1));
      ckpt_dirty |= uint64_t(1) << br_id;
      alloc_checkpoint_head_1[br_id] = free_head_1;
    }
  }

  wires->ren2dec.ready = !flush && !mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) wires->ren2dec.ready &= fire[i] || !inst_valid[i];

  const bool interrupt = wires->rob_bcast.interrupt;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (!wires->rob_commit.commit_entry[i].valid) continue;
    const auto &commit_uop = wires->rob_commit.commit_entry[i].uop;
    if (!interrupt) count_commit(ctx, commit_uop);

    InstEntry commit_entry = commit_uop.to_inst_entry(wires->rob_commit.commit_entry[i].valid);
    InstInfo *inst = &commit_entry.uop;
    // An exception instruction does not free its old destination: it looks as if it never committed.
    if (inst->dest_en && !is_exception(*inst) && !interrupt) {
      if (move_elim_enabled()) {
        // the old register is freed when no architectural register names it any more; an eliminated move
        // allocated nothing, so it does not advance the commit head
        Assert(!mispred && "Ren(ME): commit together with mispredict");
        static struct Stat {
          uint64_t elim = 0;
          ~Stat() { fprintf(stderr, "[ME] eliminated moves committed: %llu\n", (unsigned long long)elim); }
        } stat;
        if (is_eliminated_move(*inst)) stat.elim++;
        if (!is_eliminated_move(*inst)) {
          Assert(free_head_commit_1 != free_head_1 && "Ren: commit reclaim when no pending spec allocation");
          LOOP_INC(free_head_commit_1, PRF_NUM);
        }
        arch_cnt[inst->dest_preg]++;
        const auto old = (*inst)->old_dest_preg;
        if (--arch_cnt[old] == 0) {
          free_list_1[free_tail_1] = old;
          LOOP_INC(free_tail_1, PRF_NUM);
        }
      } else {
        Assert(free_head_commit_1 != free_head_1 && "Ren: commit reclaim when no pending spec allocation");
        free_list_1[free_tail_1] = (*inst)->old_dest_preg;
        LOOP_INC(free_tail_1, PRF_NUM);
        LOOP_INC(free_head_commit_1, PRF_NUM);
      }
      arch_RAT_1[(*inst)->dest_areg] = inst->dest_preg;
    }
    BE_LOG("ROB commit PC=0x%08x Inst=0x%08x idx=%ld", (*inst)->dbg.pc, (*inst)->dbg.instruction,
           (*inst)->dbg.inst_idx);
    if (!interrupt) ctx->run_commit_inst(&commit_entry, i);
  }
  if (interrupt) ctx->run_interrupt_hook();

  if (flush) {
    memcpy(spec_RAT_1, arch_RAT_1, sizeof(spec_RAT_1));
    free_head_1 = free_head_commit_1;
  } else if (mispred) {  // never together with flush
    const auto br = wires->dec_bcast.br_id;
    Assert(br < MAX_BR_NUM && br != 0 && "Ren: mispred br_id out of range");
    memcpy(spec_RAT_1, RAT_checkpoint[br], sizeof(spec_RAT_1));
    free_head_1 = alloc_checkpoint_head[br];
    free_tail_1 = free_tail;  // reclaims of this cycle are dropped, as in the original
  }
  ren_freelist_sanity(free_head_1, free_head_commit_1, free_tail_1);
  if (move_elim_enabled()) me_check();
}

// Pipeline register: emptied by flush/mispredict, reloaded from decode when ready, else keeps unfired instructions.
// Bits of branches resolved this cycle are cleared from every mask that stays.
void Ren::comb_pipeline() {
  const wire<BR_MASK_WIDTH> clear = wires->dec_bcast.clear_mask;
  const bool drain = wires->rob_bcast.flush || wires->dec_bcast.mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (drain) {
      inst_valid_1[i] = false;
    } else if (wires->ren2dec.ready) {
      inst_valid_1[i] = wires->dec2ren.valid[i];
      inst_r_1[i] = wires->dec2ren.uop[i];
      inst_r_1[i].br_mask &= ~clear;
    } else {
      inst_valid_1[i] = inst_valid[i] && !fire[i];
      if (inst_valid_1[i]) inst_r_1[i].br_mask &= ~clear;
    }
  }
}

void Ren::seq() {
  memcpy(inst_r, inst_r_1, sizeof(inst_r));
  memcpy(inst_valid, inst_valid_1, sizeof(inst_valid));
  memcpy(spec_RAT, spec_RAT_1, sizeof(spec_RAT));
  memcpy(arch_RAT, arch_RAT_1, sizeof(arch_RAT));
  memcpy(free_list, free_list_1, sizeof(free_list));
  free_head = free_head_1;
  free_head_commit = free_head_commit_1;
  free_tail = free_tail_1;
  // RAT_checkpoint_1 already equals RAT_checkpoint except for the rows written this cycle.
  for (uint64_t d = ckpt_dirty; d != 0; d &= d - 1) {
    const int row = __builtin_ctzll(d);
    memcpy(RAT_checkpoint[row], RAT_checkpoint_1[row], sizeof(RAT_checkpoint[row]));
  }
  ckpt_dirty = 0;
  memcpy(alloc_checkpoint_head, alloc_checkpoint_head_1, sizeof(alloc_checkpoint_head));
}

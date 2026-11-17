#include "BackPorts.h"
#include "Ren.h"
#include "Pmu.h"
#include "MsData.h"
#include "config.h"
#include <cstring>

#include "util.h"

// FP free list invariant: free slots + tags held by in-flight instructions == the managed pool (no move elimination here)
static void fp_freelist_sanity(const Ren::FpDomain &f) {
  Assert(f.dist(f.head, f.tail) + f.dist(f.head_commit, f.head) == FPRF_NUM - FP_ARF_NUM &&
         "Ren: FP free slots + pending spec must equal the managed pool (FP register leak)");
}

void Ren::init() {
  Regs &r = q_;
  r = Regs{};
  r.x.reset(0);
  r.f.reset(FP_TAG_BASE);
  for (int i = 0; i < PRF_NUM; i++) r.arch_cnt[i] = i < ARF_NUM + 1 ? 1 : 0;
  fp_freelist_sanity(r.f);
}

// Move elimination bound. The architectural state after a move commits equals the speculative RAT right after its
// rename, so the architectural reference count of the source register never exceeds the speculative one. A move is
// eliminated only when its source comes straight from the speculative RAT (no older instruction of the group writes
// it) and  (RAT entries naming it) + (older instructions of the group with a destination) + 1  <= MOVE_ELIM_MAX_REFS;
// older writers can add at most one reference each, so the bound holds without looking at what they write.
bool Ren::elim_refs_ok(int i) const {
  const Regs &q = q_;
  const int s1 = q.inst_r[i]->src1_areg;
  int older = 0;
  for (int j = 0; j < i; j++) {
    if (!q.inst_valid[j] || !q.inst_r[j].dest_en) continue;
    if (q.inst_r[j]->dest_areg != 0 && !q.inst_r[j]->dest_fp && q.inst_r[j]->dest_areg == s1) return false;  // source bypassed inside the group
    older++;
  }
  int refs = 0;
  for (int e = 0; e < ARF_NUM + 1; e++)
    if (q.x.spec[e] == q.x.spec[s1]) refs++;
  return refs + older + 1 <= MOVE_ELIM_MAX_REFS;
}

// Destination tags from the free lists, in order (each domain from its own; the first instruction that finds its list
// empty stalls itself and everything behind it: ren2dis.valid = 0), then the source / old-destination lookup in the
// speculative RAT, where an older instruction of the same group that writes the same architectural register (x0
// excluded) takes precedence, the youngest such writer winning.
void Ren::comb_rename() {
  const Regs &q = q_;
  fp_freelist_sanity(q.f);
  int left[2] = {q.x.free_count(), q.f.free_count()};
  int next[2] = {q.x.head, q.f.head};
  bool stall = false;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    alloc_reg_[i] = 0;
    elim_[i] = false;
    out.ren2dis.valid[i] = false;
    if (!q.inst_valid[i]) continue;
    const Uop &u = q.inst_r[i];
    if (is_move_shape(u) && elim_refs_ok(i)) {
      elim_[i] = true;
      out.ren2dis.valid[i] = !stall;
    } else if (!u.dest_en) {
      out.ren2dis.valid[i] = !stall;
    } else {
      const int fp = u->dest_fp ? 1 : 0;
      if (!stall && left[fp] > 0) {
        alloc_reg_[i] = fp ? q.f.free_list[next[1]] : q.x.free_list[next[0]];
        next[fp] = (next[fp] + 1) % (fp ? FPRF_NUM : PRF_NUM);
        left[fp]--;
        out.ren2dis.valid[i] = true;
      } else {
        stall = true;
      }
    }
  }
  g_pmu_back.ren_no_preg = stall;

  wire<PRF_IDX_WIDTH> dest_of[DECODE_WIDTH];  // final destination tag of each slot (an eliminated move: its source)
  for (int i = 0; i < DECODE_WIDTH; i++) {
    dest_of[i] = alloc_reg_[i];
    // static fields live in the uop pool: an invalid slot holds the handle of an instruction that has left the stage
    if (!q.inst_valid[i]) continue;
    Uop &r = out.ren2dis.uop[i];
    r = q.inst_r[i];
    r->dest_preg = alloc_reg_[i];
    // EXP-020: a newly allocated physical register holds no stale value; the taint of a squashed wrong-path producer (a mispredict does not clear the table) must not leak to its next owner
    if (r.dest_en && !elim_[i] && alloc_reg_[i] != 0 && alloc_reg_[i] < TAG_NUM) msd::taint[alloc_reg_[i]] = msd::TV{};
    auto lookup = [&](int areg, bool fp) {
      wire<PRF_IDX_WIDTH> p = fp ? q.f.spec[areg] : q.x.spec[areg];
      for (int j = 0; j < i; j++) {
        const Uop &o = q.inst_r[j];
        if (!q.inst_valid[j] || !o.dest_en || bool(o->dest_fp) != fp) continue;
        if (o->dest_areg == areg && (fp || areg != 0)) p = dest_of[j];
      }
      return p;
    };
    r->src1_preg = lookup(r->src1_areg, r->src1_fp);
    r->src2_preg = lookup(r->src2_areg, r->src2_fp);
    if (r->src3_en) r->src3_preg = lookup(r->src3_areg, r->src3_fp);
    r->old_dest_preg = lookup(r->dest_areg, r->dest_fp);
    if (elim_[i]) dest_of[i] = r->dest_preg = r->src1_preg;
  }
}

// The dispatch handshake: the fired slots and ren2dec.ready.
void Ren::comb_fire() {
  const Regs &q = q_;
  const bool flush = in.rob_bcast->flush, mispred = in.dec_bcast->mispred;
  out.ren2dec.ready = !flush && !mispred;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    fire_[i] = out.ren2dis.valid[i] && in.dis2ren->ready;
    out.ren2dec.ready &= fire_[i] || !q.inst_valid[i];
  }
}

// The clock edge. Fired instructions take their tag and checkpoint the speculative RAT at branches, commits free old
// tags and update the architectural RAT, then flush (restore from the architectural RAT) or mispredict (restore a
// checkpoint) override the speculative state. The pipeline register: emptied by flush / mispredict, reloaded from decode
// when ready, else keeps the unfired instructions; bits of branches resolved this cycle are cleared from every mask that
// stays.
void Ren::seq() {
  Regs &d = q_;   // updated in place
  const bool flush = in.rob_bcast->flush, mispred = in.dec_bcast->mispred;
  // a mispredict restores the branch's checkpoint as registered and the registered tails (this cycle's reclaims are dropped)
  const int br = mispred ? int(in.dec_bcast->br_id) : 0;
  Assert((!mispred || (br < MAX_BR_NUM && br != 0)) && "Ren: mispred br_id out of range");
  decltype(d.x.spec) mis_x;
  decltype(d.f.spec) mis_f;
  const int x_head = d.x.ckpt_head[br], f_head = d.f.ckpt_head[br], x_tail = d.x.tail, f_tail = d.f.tail;
  if (mispred) {
    std::memcpy(mis_x, d.x.ckpt[br], sizeof(mis_x));
    std::memcpy(mis_f, d.f.ckpt[br], sizeof(mis_f));
  }

  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (!fire_[i]) continue;
    const Uop &r = out.ren2dis.uop[i];
    if (r.dest_en) {
      if (r->dest_fp) {
        d.f.spec[r->dest_areg] = r->dest_preg;
        d.f.take();
      } else {
        d.x.spec[r->dest_areg] = r->dest_preg;
        if (!elim_[i]) d.x.take();
      }
    }
    if (is_branch(r->type)) {  // the checkpoint includes this instruction's own rename, not younger ones
      Assert(r->br_id < MAX_BR_NUM && "Ren: branch id out of range");
      d.x.checkpoint(r->br_id);
      d.f.checkpoint(r->br_id);
    }
  }

  // An instruction that raised an exception (or retires with an interrupt) does not free its old destination: it looks
  // as if it never committed.
  const bool interrupt = in.rob_bcast->interrupt;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    const auto &c = in.rob_commit->commit_entry[i];
    if (!c.valid || interrupt || !c.uop.dest_en || is_exception(c.uop)) continue;
    const Uop &u = c.uop;
    if (u->dest_fp) {   // FP: no move elimination
      d.f.commit_alloc();
      d.f.reclaim(u->old_dest_preg);
      d.f.arch[u->dest_areg] = u->dest_preg;
      continue;
    }
    // move elimination: the old tag is freed when no architectural register names it any more; an eliminated move
    // allocated nothing, so it does not advance the commit head
    Assert(!mispred && "Ren(ME): commit together with mispredict");
    if (!is_eliminated_move(u)) d.x.commit_alloc();
    d.arch_cnt[u->dest_preg]++;
    if (--d.arch_cnt[u->old_dest_preg] == 0) d.x.reclaim(u->old_dest_preg);
    d.x.arch[u->dest_areg] = u->dest_preg;
  }

  if (flush) {
    d.x.restore_arch();
    d.f.restore_arch();
  } else if (mispred) {  // never together with flush
    std::memcpy(d.x.spec, mis_x, sizeof(mis_x));
    std::memcpy(d.f.spec, mis_f, sizeof(mis_f));
    d.x.head = x_head; d.x.tail = x_tail;
    d.f.head = f_head; d.f.tail = f_tail;
  }
  fp_freelist_sanity(d.f);

  const wire<BR_MASK_WIDTH> clear = in.dec_bcast->clear_mask;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (flush || mispred) {
      d.inst_valid[i] = false;
    } else if (out.ren2dec.ready) {
      d.inst_valid[i] = in.dec2ren->valid[i];
      d.inst_r[i] = in.dec2ren->uop[i];
      d.inst_r[i].br_mask &= ~clear;
    } else {
      d.inst_valid[i] = d.inst_valid[i] && !fire_[i];
      if (d.inst_valid[i]) d.inst_r[i].br_mask &= ~clear;
    }
  }
}

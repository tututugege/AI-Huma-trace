#include "BackPorts.h"
#include "Ren.h"
#include "Pmu.h"
#include "MsData.h"
#include "config.h"
#include "DseCap.h"
#include <algorithm>
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

// Move elimination rule (1-hop intra-group bypass; unbounded arch_cnt up to RAT_NUM=33 fits in 6 bits):
// a move-shaped instruction is eliminated unless its source is produced by an older eliminated move in the same group.
bool Ren::elim_refs_ok(int i) const {
  const Regs &q = q_;
  const int s1 = q.inst_r[i]->src1_areg;
  int producer = -1;
  for (int j = 0; j < i; j++) {
    if (!q.inst_valid[j] || !q.inst_r[j].dest_en) continue;
    if (q.inst_r[j]->dest_areg != 0 && !q.inst_r[j]->dest_fp && q.inst_r[j]->dest_areg == s1) producer = j;
  }
  return producer < 0 || !elim_[producer];
}

// Destination tags from the free lists, in order (each domain from its own; the first instruction that finds its list
// or local RenLog empty stalls itself and everything behind it: ren2dis.valid = 0), then the source / old-destination
// lookup in the speculative RAT, where an older instruction of the same group that writes the same architectural
// register (x0 excluded) takes precedence, the youngest such writer winning.
void Ren::comb_rename() {
  const Regs &q = q_;
  fp_freelist_sanity(q.f);
  out.ren2rob.walking = q.walk_active;
  int left[2] = {dse_free(q.x.free_count() - (PRF_NUM - g_dse.prf), 0), dse_free(q.f.free_count() - (FPRF_NUM - g_dse.fprf), 0)};
  int next[2] = {q.x.head, q.f.head};
  int log_left = ROB_NUM - q.ren_log_used;
  bool stall = q.walk_active;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    alloc_reg_[i] = 0;
    elim_[i] = false;
    out.ren2dis.valid[i] = false;
    if (!q.inst_valid[i]) continue;
    if (!stall && log_left > 0) {
      log_left--;
    } else {
      stall = true;
    }
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
  g_pmu_back.ren_no_preg = stall && !q.walk_active;

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

// The dispatch handshake: the fired slots, S2 advance, and ren2dec.ready (2-stage Ren: IDU -> S1 -> S2 -> DIS).
void Ren::comb_fire() {
  const Regs &q = q_;
  const bool flush = in.rob_bcast->flush, mispred = in.dec_bcast->mispred;
  const bool walking = q.walk_active;
  bool s2_ready = !flush && !mispred && !walking;
  for (int i = 0; i < DECODE_WIDTH; i++) {
    fire_[i] = !walking && out.ren2dis.valid[i] && in.dis2ren->ready;
    s2_ready &= fire_[i] || !q.inst_valid[i];
  }
  s2_ready_ = s2_ready;
  bool s1_empty = true;
  for (int i = 0; i < DECODE_WIDTH; i++) if (q.s1_valid[i]) s1_empty = false;
  out.ren2dec.ready = !flush && !mispred && (s1_empty || s2_ready);
}

// The clock edge. Sparse REN_CKPT_NUM-entry FIFO checkpoints (branch spacing >= REN_CKPT_INTERVAL) +
// local ROB-indexed RenLog (1R1W: commit paused during walk) + 8-wide forward walk on mispredict.
void Ren::seq() {
  Regs &d = q_;   // updated in place
  const bool flush = in.rob_bcast->flush, mispred = in.dec_bcast->mispred;
  constexpr int K = REN_CKPT_NUM;
  const int old_rob_head = d.ren_rob_head;
  const int old_ckpt_count = d.ckpt_count;
  const int old_last_ckpt_idx = old_ckpt_count > 0 ? d.ckpt[(d.ckpt_tail + K - 1) % K].rob_idx : old_rob_head;
  auto age_of = [&](int idx) { return (idx - old_rob_head + ROB_NUM) % ROB_NUM; };

  // 0. Drain 1-cycle pipelined INT free-list reclaims from previous cycle's commit
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (d.reclaim_q_valid[i]) {
      d.x.reclaim(d.reclaim_q_preg[i]);
      d.reclaim_q_valid[i] = 0;
    }
  }

  // 1. Commit: read from local RenLog (assert lockstep with ROB), advance ren_rob_head, retire old checkpoints
  const bool interrupt = in.rob_bcast->interrupt;
  int commit_cnt = 0;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    const auto &c = in.rob_commit->commit_entry[i];
    if (!c.valid) break;
    const int ridx = (old_rob_head + commit_cnt) % ROB_NUM;
    const RenLogEntry &e = d.ren_log[ridx];
    commit_cnt++;
    if (interrupt || is_exception(c.uop)) continue;
    Assert(bool(e.dest_en) == bool(c.uop.dest_en) && "RenLog: dest_en mismatch at commit");
    if (!e.dest_en) continue;
    Assert(bool(e.dest_fp) == bool(c.uop->dest_fp) && "RenLog: dest_fp mismatch at commit");
    Assert(int(e.dest_areg) == int(c.uop->dest_areg) && "RenLog: dest_areg mismatch at commit");
    Assert(int(e.dest_preg) == int(c.uop->dest_preg) && "RenLog: dest_preg mismatch at commit");
    Assert(int(e.old_dest_preg) == int(c.uop->old_dest_preg) && "RenLog: old_dest_preg mismatch at commit");
    Assert(bool(e.me_elim) == is_eliminated_move(c.uop) && "RenLog: me_elim mismatch at commit");
    if (e.dest_fp) {
      d.f.commit_alloc();
      d.f.reclaim(e.old_dest_preg);
      d.f.arch[e.dest_areg] = e.dest_preg;
    } else {
      Assert(!mispred && "Ren(ME): commit together with mispredict");
      if (!e.me_elim) d.x.commit_alloc();
      d.arch_cnt[e.dest_preg]++;
      if (--d.arch_cnt[e.old_dest_preg] == 0) {
        d.reclaim_q_valid[i] = 1;
        d.reclaim_q_preg[i] = e.old_dest_preg;
      }
      d.x.arch[e.dest_areg] = e.dest_preg;
    }
  }
  if (commit_cnt > 0) {
    d.ren_rob_head = (old_rob_head + commit_cnt) % ROB_NUM;
    d.ren_log_used -= commit_cnt;
    Assert(d.ren_log_used >= 0 && "RenLog: underflow at commit");
    while (d.ckpt_count > 0 && age_of(d.ckpt[d.ckpt_head].rob_idx) < commit_cnt) {
      d.ckpt[d.ckpt_head].valid = false;
      d.ckpt_head = (d.ckpt_head + 1) % K;
      d.ckpt_count--;
    }
  }

  // 2. Speculative state transition: flush > mispred > walk > normal rename fire
  if (flush) {
    d.x.restore_arch();
    d.f.restore_arch();
    d.ren_rob_head = 0;
    d.ren_rob_tail = 0;
    d.ren_log_used = 0;
    for (int i = 0; i < K; i++) d.ckpt[i].valid = false;
    d.ckpt_head = d.ckpt_tail = d.ckpt_count = 0;
    d.walk_active = false;
    d.walk_ptr = 0;
    d.walk_rem = 0;
  } else if (mispred) {
    Assert(commit_cnt == 0 && "Ren: commit during mispredict");
    const int br_idx = static_cast<int>(in.dec_bcast->redirect_rob_idx);
    const int br_age = age_of(br_idx);
    Assert(br_age < d.ren_log_used && "RenLog: mispredicted branch outside valid range");

    int best_k = -1;
    for (int k = 0; k < d.ckpt_count; k++) {
      const int slot = (d.ckpt_head + k) % K;
      if (d.ckpt[slot].valid && age_of(d.ckpt[slot].rob_idx) <= br_age) best_k = k;
    }
    if (best_k >= 0) {
      const int best_slot = (d.ckpt_head + best_k) % K;
      const RenCkptEntry &ck = d.ckpt[best_slot];
      std::memcpy(d.x.spec, ck.x_spec, sizeof(d.x.spec));
      std::memcpy(d.f.spec, ck.f_spec, sizeof(d.f.spec));
      d.x.head = ck.x_head;
      d.f.head = ck.f_head;
      for (int k = best_k + 1; k < d.ckpt_count; k++) d.ckpt[(d.ckpt_head + k) % K].valid = false;
      d.ckpt_count = best_k + 1;
      d.ckpt_tail = (d.ckpt_head + d.ckpt_count) % K;
      const int best_age = age_of(ck.rob_idx);
      if (best_age == br_age) {
        d.walk_active = false;
        d.walk_rem = 0;
      } else {
        d.walk_active = true;
        d.walk_ptr = (ck.rob_idx + 1) % ROB_NUM;
        d.walk_rem = br_age - best_age;
      }
    } else {
      d.x.restore_arch();
      d.f.restore_arch();
      for (int i = 0; i < K; i++) d.ckpt[i].valid = false;
      d.ckpt_head = d.ckpt_tail = d.ckpt_count = 0;
      d.walk_active = true;
      d.walk_ptr = d.ren_rob_head;
      d.walk_rem = br_age + 1;
    }
    d.ren_rob_tail = (br_idx + 1) % ROB_NUM;
    d.ren_log_used = br_age + 1;
  } else if (d.walk_active) {
    const int step = std::min(d.walk_rem, DECODE_WIDTH);
    for (int k = 0; k < step; k++) {
      const RenLogEntry &e = d.ren_log[(d.walk_ptr + k) % ROB_NUM];
      if (!e.dest_en) continue;
      if (e.dest_fp) {
        d.f.spec[e.dest_areg] = e.dest_preg;
        d.f.take();
      } else {
        d.x.spec[e.dest_areg] = e.dest_preg;
        if (!e.me_elim) d.x.take();
      }
    }
    d.walk_ptr = (d.walk_ptr + step) % ROB_NUM;
    d.walk_rem -= step;
    if (d.walk_rem == 0) d.walk_active = false;
  } else {
    const bool can_alloc = (old_ckpt_count < K);
    const int anchor = old_last_ckpt_idx;
    bool ckpt_done = false;
    int fired_n = 0;
    for (int i = 0; i < DECODE_WIDTH; i++) {
      if (!fire_[i]) continue;
      const Uop &r = out.ren2dis.uop[i];
      const int ridx = (d.ren_rob_tail + fired_n) % ROB_NUM;
      RenLogEntry &e = d.ren_log[ridx];
      e.dest_en = r.dest_en;
      e.dest_fp = r->dest_fp;
      e.dest_areg = r->dest_areg;
      e.dest_preg = r->dest_preg;
      e.old_dest_preg = r->old_dest_preg;
      e.me_elim = elim_[i];

      if (r.dest_en) {
        if (r->dest_fp) {
          d.f.spec[r->dest_areg] = r->dest_preg;
          d.f.take();
        } else {
          d.x.spec[r->dest_areg] = r->dest_preg;
          if (!elim_[i]) d.x.take();
        }
      }

      if (can_alloc && !ckpt_done) {
        const int dist = (ridx - anchor + ROB_NUM) % ROB_NUM;
        if (is_branch(r->type) && (dist >= REN_CKPT_INTERVAL)) {
          RenCkptEntry &ck = d.ckpt[d.ckpt_tail];
          ck.valid = true;
          ck.rob_idx = ridx;
          std::memcpy(ck.x_spec, d.x.spec, sizeof(d.x.spec));
          std::memcpy(ck.f_spec, d.f.spec, sizeof(d.f.spec));
          ck.x_head = d.x.head;
          ck.f_head = d.f.head;
          d.ckpt_tail = (d.ckpt_tail + 1) % K;
          d.ckpt_count++;
          ckpt_done = true;
        }
      }
      fired_n++;
    }
    d.ren_rob_tail = (d.ren_rob_tail + fired_n) % ROB_NUM;
    d.ren_log_used += fired_n;
    Assert(d.ren_log_used <= ROB_NUM && "RenLog: overflow at fire");
  }
  fp_freelist_sanity(d.f);

  const wire<BR_MASK_WIDTH> clear = in.dec_bcast->clear_mask;
  const bool s2_take = s2_ready_;
  const bool s1_take = out.ren2dec.ready;
  // Advance S2 from S1 before overwriting S1
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (flush || mispred) {
      d.inst_valid[i] = false;
    } else if (s2_take) {
      d.inst_valid[i] = d.s1_valid[i];
      d.inst_r[i] = d.s1_inst[i];
      if (d.inst_valid[i]) d.inst_r[i].br_mask &= ~clear;
    } else {
      d.inst_valid[i] = d.inst_valid[i] && !fire_[i];
      if (d.inst_valid[i]) d.inst_r[i].br_mask &= ~clear;
    }
  }
  // Advance S1 from IDU
  for (int i = 0; i < DECODE_WIDTH; i++) {
    if (flush || mispred) {
      d.s1_valid[i] = false;
    } else if (s1_take) {
      d.s1_valid[i] = in.dec2ren->valid[i];
      d.s1_inst[i] = in.dec2ren->uop[i];
      if (d.s1_valid[i]) d.s1_inst[i].br_mask &= ~clear;
    } else if (s2_take) {
      d.s1_valid[i] = false;
    } else {
      if (d.s1_valid[i]) d.s1_inst[i].br_mask &= ~clear;
    }
  }
}

#pragma once
// EXP-058: Hardware-faithful LLC Bingo 4KB Spatial Prefetcher (1:1 mirror of rtl/memsys/axi_llc/axi_llc_bingo.sv)
//
// All tables are bounded, fixed-size, synthesizable, with zero 64-bit timestamps and zero multipliers:
//   - FT    (32 entries, FIFO ptr 5b): first access (reg, key, trig_off) to a 4KB region.
//   - AGT   (32 entries, 31b tree-PLRU): 64-bit spatial bitmask per active region; commits on AGT or LLC eviction.
//   - PHT_L (256 sets x 4 ways = 1024 entries, 3b tree-PLRU, 1RW SRAM): key = {pch[11:0], off[5:0]}, idx = key[7:0]^key[15:8], tag = key[17:8].
//   - PHT_S (128 sets x 4 ways = 512 entries,  3b tree-PLRU, 1RW SRAM): key = pch[11:0], idx = pch[6:0]^{3'b0,pch[11:8]}, tag = pch[11:7].
//   - 1RW SRAM FSM (ST_IDLE -> ST_PRED / ST_CMT): 1 SRAM read or write per cycle; 4-entry commit FIFO.
//   - 1-candidate/cycle burst emitter -> 16-entry deduplicated FIFO (bingo_q) launched at lower priority than BOP.
//   - Dense-stream filter (popcount >= 48) + accuracy throttler (win=256, min=26 => 10.16%, 1/8 probe with deg=2).

#include <array>
#include <cstdint>
#include <cstdio>

class BingoHw {
 public:
  static constexpr int FT_N       = 32;
  static constexpr int AGT_N      = 32;
  static constexpr int PHTL_SETS  = 256;
  static constexpr int PHTS_SETS  = 128;
  static constexpr int PHT_WAYS   = 4;
  static constexpr int CMTQ_N     = 4;
  static constexpr int BINGO_Q    = 16;
  static constexpr int DEG        = 16;
  static constexpr int THR_DEG    = 2;
  static constexpr int MAX_DENSE  = 48;
  static constexpr int THR_WIN    = 256;
  static constexpr int THR_MIN    = 26;   // 26 / 256 = 10.16% (zero multiplier)

  static uint16_t hash_pc(uint32_t pc) {
    const uint32_t h = (pc >> 1) ^ (pc >> 13) ^ (pc >> 21);
    return static_cast<uint16_t>(h & 0xFFFu);
  }

  BingoHw() { reset(); }

  void reset() {
    for (auto &e : ft_) e = {};
    ft_ptr_ = 0;
    for (auto &e : agt_) e = {};
    agt_plru_ = 0;
    for (auto &s : phtl_) for (auto &e : s) e = {};
    phtl_plru_.fill(0);
    for (auto &s : phts_) for (auto &e : s) e = {};
    phts_plru_.fill(0);
    fsm_ = Fsm::IDLE;
    pred_pend_ = {};
    pred_cur_ = {};
    cmt_cur_ = {};
    for (auto &e : cmt_q_) e = {};
    cmt_cnt_ = 0;
    for (auto &e : rd_phtl_) e = {};
    for (auto &e : rd_phts_) e = {};
    emit_v_ = false;
    emit_reg_ = 0;
    emit_off_ = 0;
    emit_mask_ = 0;
    emit_left_ = 0;
    for (auto &l : q_line_) l = 0;
    q_cnt_ = 0;
    win_sent_ = 0;
    win_useful_ = 0;
    throttled_ = false;
    probe_cnt_ = 0;
  }

  // Combinational queue head inspection (before tick)
  bool q_empty() const { return q_cnt_ == 0; }
  uint32_t q_head_line() const { return q_line_[0]; }
  bool is_idle() const {
    return q_cnt_ == 0 && !emit_v_ && fsm_ == Fsm::IDLE && !pred_pend_.v && cmt_cnt_ == 0;
  }

  // Advance one cycle (exact 1:1 mirror of axi_llc_bingo.sv always_comb -> always_ff)
  void tick(bool pop_head, bool pf_sent,
            bool trig_v, bool trig_pf, uint32_t trig_line, uint16_t trig_pch,
            bool evict_v, uint32_t evict_line) {
    // 1. Accuracy throttler
    const uint32_t ns = win_sent_ + (pf_sent ? 1u : 0u);
    const uint32_t nu = win_useful_ + ((trig_v && trig_pf) ? 1u : 0u);
    if (ns >= uint32_t(THR_WIN)) {
      throttled_ = (nu < uint32_t(THR_MIN));
      win_sent_ = 0;
      win_useful_ = 0;
    } else {
      win_sent_ = ns;
      win_useful_ = nu;
    }
    stats_sent += pf_sent ? 1u : 0u;
    stats_useful += (trig_v && trig_pf) ? 1u : 0u;

    // 2. Pop head of bingo_q + step 1-candidate/cycle burst emitter (reads old emit_*_q)
    int cnt_p = q_cnt_;
    uint32_t q_p[BINGO_Q];
    for (int i = 0; i < BINGO_Q; i++) {
      q_p[i] = (pop_head && (i + 1 < BINGO_Q)) ? q_line_[i + 1] : q_line_[i];
    }
    if (pop_head && cnt_p > 0) cnt_p--;

    bool emit_v_d = emit_v_;
    uint32_t emit_reg_d = emit_reg_;
    uint8_t emit_off_d = emit_off_;
    uint64_t emit_mask_d = emit_mask_;
    uint8_t emit_left_d = emit_left_;

    bool push_cand = false;
    uint32_t cand_line = 0;
    if (emit_v_ && emit_left_ > 0 && emit_mask_ != 0) {
      const uint32_t rot_sh = (uint32_t(emit_off_) + 1u) & 63u;
      const uint64_t rot = (rot_sh == 0) ? emit_mask_ : ((emit_mask_ >> rot_sh) | (emit_mask_ << (64u - rot_sh)));
      int d = 0;
      for (int i = 0; i < 64; i++) {
        if ((rot >> i) & 1ull) { d = i; break; }
      }
      const uint32_t idx = (rot_sh + uint32_t(d)) & 63u;
      cand_line = (emit_reg_ << 6) | idx;
      push_cand = true;
      emit_mask_d = emit_mask_ & ~(1ull << idx);
      emit_left_d = uint8_t(emit_left_ - 1u);
      if (emit_left_d == 0 || emit_mask_d == 0) emit_v_d = false;
      stats_issued++;
    }

    if (push_cand) {
      bool dup = false;
      for (int i = 0; i < cnt_p; i++) if (q_p[i] == cand_line) dup = true;
      if (!dup) {
        if (cnt_p == BINGO_Q) {
          for (int i = 0; i < BINGO_Q - 1; i++) q_p[i] = q_p[i + 1];
          q_p[BINGO_Q - 1] = cand_line;
        } else {
          q_p[cnt_p++] = cand_line;
        }
      }
    }
    for (int i = 0; i < BINGO_Q; i++) q_line_[i] = q_p[i];
    q_cnt_ = cnt_p;

    // 3. 1RW SRAM PHT FSM (operates on registered state from previous cycle)
    Fsm fsm_d = fsm_;
    PredReq pred_pend_d = pred_pend_;
    PredReq pred_cur_d = pred_cur_;
    CmtReq cmt_cur_d = cmt_cur_;
    bool cmt_pop = false;

    if (fsm_ == Fsm::PRED) {
      const uint16_t tag_l = uint16_t((pred_cur_.key >> 8) & 0x3FFu);
      const uint8_t  tag_s = uint8_t((pred_cur_.pch >> 7) & 0x1Fu);
      const uint8_t  idx_l = phtl_idx(pred_cur_.key);
      const uint8_t  idx_s = phts_idx(pred_cur_.pch);

      int hit_l = -1;
      for (int w = 0; w < PHT_WAYS; w++) {
        if (rd_phtl_[w].v && rd_phtl_[w].tag == tag_l) { hit_l = w; break; }
      }
      int hit_s = -1;
      if (hit_l < 0) {
        for (int w = 0; w < PHT_WAYS; w++) {
          if (rd_phts_[w].v && rd_phts_[w].tag == tag_s) { hit_s = w; break; }
        }
      }

      if (hit_l >= 0 || hit_s >= 0) {
        uint64_t hit_mask = 0;
        if (hit_l >= 0) {
          phtl_plru_[idx_l] = plru4_update(phtl_plru_[idx_l], uint8_t(hit_l));
          stats_pht_hit++;
          for (int b = 0; b < 64; b++) {
            if (rd_phtl_[hit_l].cnt[b] >= 1) hit_mask |= (1ull << b);
          }
        } else {
          phts_plru_[idx_s] = plru4_update(phts_plru_[idx_s], uint8_t(hit_s));
          stats_short_hit++;
          for (int d = 0; d < 64; d++) {
            if (rd_phts_[hit_s].cnt[d] >= 1) {
              const uint32_t abs_off = (uint32_t(pred_cur_.off) + uint32_t(d)) & 63u;
              hit_mask |= (1ull << abs_off);
            }
          }
        }
        int dense = 0;
        for (int b = 0; b < 64; b++) dense += int((hit_mask >> b) & 1ull);
        hit_mask &= ~(1ull << pred_cur_.off);
        if (dense >= MAX_DENSE) {
          stats_dense_skip++;
        } else if (hit_mask != 0) {
          bool allow = true;
          uint8_t deg = uint8_t(DEG);
          if (throttled_) {
            const uint8_t pc_old = probe_cnt_;
            probe_cnt_ = uint8_t((probe_cnt_ + 1u) & 7u);
            if (pc_old != 0) { allow = false; stats_thr_skip++; }
            else deg = uint8_t(THR_DEG);
          }
          if (allow) {
            emit_v_d = true;
            emit_reg_d = pred_cur_.reg;
            emit_off_d = pred_cur_.off;
            emit_mask_d = hit_mask;
            emit_left_d = deg;
          }
        }
      }
      fsm_d = Fsm::IDLE;
    } else if (fsm_ == Fsm::CMT) {
      // Write 1 way of PHT_L and 1 way of PHT_S (1RW write cycle)
      const uint8_t  idx_l = phtl_idx(cmt_cur_.key);
      const uint16_t tag_l = uint16_t((cmt_cur_.key >> 8) & 0x3FFu);
      const uint16_t pch   = uint16_t((cmt_cur_.key >> 6) & 0xFFFu);
      const uint8_t  idx_s = phts_idx(pch);
      const uint8_t  tag_s = uint8_t((pch >> 7) & 0x1Fu);

      // Update PHT_L
      int wl = -1;
      for (int w = 0; w < PHT_WAYS; w++) {
        if (rd_phtl_[w].v && rd_phtl_[w].tag == tag_l) { wl = w; break; }
      }
      bool hit_l = (wl >= 0);
      if (!hit_l) {
        for (int w = 0; w < PHT_WAYS; w++) if (!rd_phtl_[w].v) { wl = w; break; }
        if (wl < 0) wl = int(plru4_victim(phtl_plru_[idx_l]));
      }
      PhtE nl = hit_l ? rd_phtl_[wl] : PhtE{};
      nl.v = true;
      nl.tag = tag_l;
      for (int b = 0; b < 64; b++) {
        const bool bit = ((cmt_cur_.mask >> b) & 1ull) != 0;
        if (!hit_l) nl.cnt[b] = bit ? 2 : 0;
        else if (bit) { if (nl.cnt[b] < 3) nl.cnt[b]++; }
        else          { if (nl.cnt[b] > 0) nl.cnt[b]--; }
      }
      phtl_[idx_l][wl] = nl;
      phtl_plru_[idx_l] = plru4_update(phtl_plru_[idx_l], uint8_t(wl));

      // Relative mask for PHT_S
      uint64_t rel_mask = 0;
      for (int d = 0; d < 64; d++) {
        const uint32_t abs_off = (uint32_t(cmt_cur_.trig_off) + uint32_t(d)) & 63u;
        if ((cmt_cur_.mask >> abs_off) & 1ull) rel_mask |= (1ull << d);
      }
      int ws = -1;
      for (int w = 0; w < PHT_WAYS; w++) {
        if (rd_phts_[w].v && rd_phts_[w].tag == tag_s) { ws = w; break; }
      }
      bool hit_s = (ws >= 0);
      if (!hit_s) {
        for (int w = 0; w < PHT_WAYS; w++) if (!rd_phts_[w].v) { ws = w; break; }
        if (ws < 0) ws = int(plru4_victim(phts_plru_[idx_s]));
      }
      PhtE ns_e = hit_s ? rd_phts_[ws] : PhtE{};
      ns_e.v = true;
      ns_e.tag = tag_s;
      for (int b = 0; b < 64; b++) {
        const bool bit = ((rel_mask >> b) & 1ull) != 0;
        if (!hit_s) ns_e.cnt[b] = bit ? 2 : 0;
        else if (bit) { if (ns_e.cnt[b] < 3) ns_e.cnt[b]++; }
        else          { if (ns_e.cnt[b] > 0) ns_e.cnt[b]--; }
      }
      phts_[idx_s][ws] = ns_e;
      phts_plru_[idx_s] = plru4_update(phts_plru_[idx_s], uint8_t(ws));

      stats_pht_commit++;
      fsm_d = Fsm::IDLE;
    } else {
      // ST_IDLE: launch 1RW SRAM read (prediction has priority over commit)
      if (pred_pend_.v) {
        const uint8_t idx_l = phtl_idx(pred_pend_.key);
        const uint8_t idx_s = phts_idx(pred_pend_.pch);
        for (int w = 0; w < PHT_WAYS; w++) {
          rd_phtl_[w] = phtl_[idx_l][w];
          rd_phts_[w] = phts_[idx_s][w];
        }
        pred_cur_d = pred_pend_;
        pred_pend_d.v = false;
        fsm_d = Fsm::PRED;
      } else if (cmt_cnt_ > 0) {
        const CmtReq &cr = cmt_q_[0];
        const uint8_t idx_l = phtl_idx(cr.key);
        const uint8_t idx_s = phts_idx(uint16_t((cr.key >> 6) & 0xFFFu));
        for (int w = 0; w < PHT_WAYS; w++) {
          rd_phtl_[w] = phtl_[idx_l][w];
          rd_phts_[w] = phts_[idx_s][w];
        }
        cmt_cur_d = cr;
        cmt_pop = true;
        fsm_d = Fsm::CMT;
      }
    }

    // Commit queue post-pop state
    int cmt_cnt_d = cmt_cnt_;
    CmtReq cmt_q_d[CMTQ_N];
    for (int i = 0; i < CMTQ_N; i++) {
      cmt_q_d[i] = (cmt_pop && (i + 1 < CMTQ_N)) ? cmt_q_[i + 1] : cmt_q_[i];
    }
    if (cmt_pop && cmt_cnt_d > 0) cmt_cnt_d--;
    auto push_cmt = [&](uint32_t key, uint8_t trig_off, uint64_t mask) {
      if ((mask & ~(1ull << trig_off)) == 0) return;
      if (cmt_cnt_d < CMTQ_N) {
        cmt_q_d[cmt_cnt_d++] = {key, trig_off, mask};
      }
    };

    // 4. FT & AGT updates on evict_v and trig_v
    if (evict_v) {
      const uint32_t ereg = (evict_line >> 6) & 0xFFFFFu;
      for (int i = 0; i < AGT_N; i++) {
        if (agt_[i].v && agt_[i].reg == ereg) {
          push_cmt(agt_[i].key, agt_[i].trig_off, agt_[i].mask);
          agt_[i].v = false;
          stats_evict_commit++;
          break;
        }
      }
    }

    if (trig_v) {
      const uint32_t reg = (trig_line >> 6) & 0xFFFFFu;
      const uint8_t  off = uint8_t(trig_line & 63u);
      const uint16_t pch = uint16_t(trig_pch & 0xFFFu);
      const uint32_t key = (uint32_t(pch) << 6) | uint32_t(off);

      int ai = -1;
      for (int i = 0; i < AGT_N; i++) if (agt_[i].v && agt_[i].reg == reg) { ai = i; break; }
      if (ai >= 0) {
        agt_[ai].mask |= (1ull << off);
        agt_plru_ = plru32_update(agt_plru_, uint8_t(ai));
        stats_agt_hit++;
      } else {
        int fi = -1;
        for (int i = 0; i < FT_N; i++) if (ft_[i].v && ft_[i].reg == reg) { fi = i; break; }
        if (fi >= 0) {
          if (ft_[fi].trig_off != off) {
            int vict = -1;
            for (int i = 0; i < AGT_N; i++) if (!agt_[i].v) { vict = i; break; }
            if (vict < 0) vict = int(plru32_victim(agt_plru_));
            if (agt_[vict].v) push_cmt(agt_[vict].key, agt_[vict].trig_off, agt_[vict].mask);
            agt_[vict] = {true, reg, ft_[fi].key, ft_[fi].trig_off,
                          (1ull << ft_[fi].trig_off) | (1ull << off)};
            agt_plru_ = plru32_update(agt_plru_, uint8_t(vict));
            ft_[fi].v = false;
            stats_promote++;
          }
        } else {
          ft_[ft_ptr_] = {true, reg, key, off};
          ft_ptr_ = uint8_t((ft_ptr_ + 1u) & uint8_t(FT_N - 1));
          pred_pend_d = {true, reg, key, pch, off};
        }
      }
    }

    // Commit next state
    emit_v_ = emit_v_d;
    emit_reg_ = emit_reg_d;
    emit_off_ = emit_off_d;
    emit_mask_ = emit_mask_d;
    emit_left_ = emit_left_d;
    fsm_ = fsm_d;
    pred_pend_ = pred_pend_d;
    pred_cur_ = pred_cur_d;
    cmt_cur_ = cmt_cur_d;
    for (int i = 0; i < CMTQ_N; i++) cmt_q_[i] = cmt_q_d[i];
    cmt_cnt_ = cmt_cnt_d;
  }

  uint64_t stats_issued = 0, stats_sent = 0, stats_useful = 0;
  uint64_t stats_agt_hit = 0, stats_promote = 0, stats_pht_hit = 0, stats_short_hit = 0;
  uint64_t stats_pht_commit = 0, stats_evict_commit = 0, stats_dense_skip = 0, stats_thr_skip = 0;

  void print() const {
    std::printf("[BINGO] LLC: pht=%dx%d+%dx%d issued=%llu sent=%llu useful=%llu "
                "pht_hit=%llu short_hit=%llu dskip=%llu tskip=%llu commits=%llu(evict=%llu)\n",
                PHTL_SETS, PHT_WAYS, PHTS_SETS, PHT_WAYS,
                (unsigned long long)stats_issued, (unsigned long long)stats_sent, (unsigned long long)stats_useful,
                (unsigned long long)stats_pht_hit, (unsigned long long)stats_short_hit,
                (unsigned long long)stats_dense_skip, (unsigned long long)stats_thr_skip,
                (unsigned long long)stats_pht_commit, (unsigned long long)stats_evict_commit);
  }

 private:
  enum class Fsm : uint8_t { IDLE = 0, PRED = 1, CMT = 2 };
  struct FtE     { bool v = false; uint32_t reg = 0; uint32_t key = 0; uint8_t trig_off = 0; };
  struct AgtE    { bool v = false; uint32_t reg = 0; uint32_t key = 0; uint8_t trig_off = 0; uint64_t mask = 0; };
  struct PhtE    { bool v = false; uint16_t tag = 0; std::array<uint8_t, 64> cnt{}; };
  struct PredReq { bool v = false; uint32_t reg = 0; uint32_t key = 0; uint16_t pch = 0; uint8_t off = 0; };
  struct CmtReq  { uint32_t key = 0; uint8_t trig_off = 0; uint64_t mask = 0; };

  static uint8_t phtl_idx(uint32_t key) { return uint8_t((key ^ (key >> 8)) & 0xFFu); }
  static uint8_t phts_idx(uint16_t pch) { return uint8_t((pch ^ (pch >> 8)) & 0x7Fu); }

  static uint8_t plru4_update(uint8_t p, uint8_t way) {
    uint8_t n = p & 0x7u;
    const uint8_t w1 = (way >> 1) & 1u, w0 = way & 1u;
    n = uint8_t((n & ~1u) | (w1 ^ 1u));
    if (w1 == 0) n = uint8_t((n & ~2u) | ((w0 ^ 1u) << 1));
    else         n = uint8_t((n & ~4u) | ((w0 ^ 1u) << 2));
    return n;
  }
  static uint8_t plru4_victim(uint8_t p) {
    const uint8_t r = p & 1u;
    const uint8_t l = r ? ((p >> 2) & 1u) : ((p >> 1) & 1u);
    return uint8_t((r << 1) | l);
  }

  // 31-bit binary tree-PLRU for 32-entry AGT (node 0 = root; children 2*n+1, 2*n+2)
  static uint32_t plru32_update(uint32_t p, uint8_t slot) {
    int node = 0;
    for (int lvl = 4; lvl >= 0; lvl--) {
      const uint32_t bit = (slot >> lvl) & 1u;
      if (bit) p &= ~(1u << node);
      else     p |=  (1u << node);
      node = 2 * node + (bit ? 2 : 1);
    }
    return p & 0x7FFFFFFFu;
  }
  static uint8_t plru32_victim(uint32_t p) {
    int node = 0;
    uint8_t slot = 0;
    for (int lvl = 4; lvl >= 0; lvl--) {
      const uint8_t go_r = uint8_t((p >> node) & 1u);
      slot = uint8_t((slot << 1) | go_r);
      node = 2 * node + (go_r ? 2 : 1);
    }
    return slot;
  }

  std::array<FtE, FT_N> ft_{};
  uint8_t ft_ptr_ = 0;
  std::array<AgtE, AGT_N> agt_{};
  uint32_t agt_plru_ = 0;

  std::array<std::array<PhtE, PHT_WAYS>, PHTL_SETS> phtl_{};
  std::array<uint8_t, PHTL_SETS> phtl_plru_{};
  std::array<std::array<PhtE, PHT_WAYS>, PHTS_SETS> phts_{};
  std::array<uint8_t, PHTS_SETS> phts_plru_{};

  Fsm fsm_ = Fsm::IDLE;
  PredReq pred_pend_{};
  PredReq pred_cur_{};
  CmtReq cmt_cur_{};
  std::array<CmtReq, CMTQ_N> cmt_q_{};
  int cmt_cnt_ = 0;
  std::array<PhtE, PHT_WAYS> rd_phtl_{};
  std::array<PhtE, PHT_WAYS> rd_phts_{};

  bool emit_v_ = false;
  uint32_t emit_reg_ = 0;
  uint8_t emit_off_ = 0;
  uint64_t emit_mask_ = 0;
  uint8_t emit_left_ = 0;

  std::array<uint32_t, BINGO_Q> q_line_{};
  int q_cnt_ = 0;

  uint32_t win_sent_ = 0;
  uint32_t win_useful_ = 0;
  bool throttled_ = false;
  uint8_t probe_cnt_ = 0;
};

inline BingoHw *g_bingo_llc = nullptr;

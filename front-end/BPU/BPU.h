// Compact cycle-exact BPU: TAGE + BTB/indirect target cache + type predictor + history/RAS + update queue.
// Replaces the RTL-style BPU_TOP (kept as tests/bpu_ref in the trace repo, checked by tests/bpu_diff.cpp).
// comb() is pure; commit() applies the transaction it returned.
#pragma once
#include "../frontend.h"
#include "../wire_types.h"
#include "BPU_configs.h"
#include <cstdint>
#include <cstring>

namespace bpuf {

constexpr int NB = BPU_BANK_NUM, FW = FETCH_WIDTH, CW = COMMIT_WIDTH, TN = TN_MAX;
constexpr uint32_t kGhrLen[TN] = {8, 13, 32, 119};
constexpr uint32_t kFhLen[FH_N_MAX][TN] = {{8, 11, 11, 11}, {8, 8, 8, 8}, {7, 7, 7, 7}};

inline uint32_t bank_of(pc_t pc) { return (pc >> 2) % NB; }
inline pc_t bank_pc_of(pc_t pc) {
  if ((NB & (NB - 1)) != 0) return ((pc >> 2) / NB) << 2;
  return pc >> ceil_log2_u32(NB);
}

// ---------------------------------------------------------------------------
// Global history: GHR as a shift register plus the folded histories TAGE indexes with.
// ---------------------------------------------------------------------------
struct Hist {
  unsigned __int128 ghr = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  void push(bool dir) {
    for (int k = 0; k < FH_N_MAX; k++)
      for (int i = 0; i < TN; i++) {
        const uint32_t len = kFhLen[k][i];
        uint32_t v = fh[k][i];
        const uint32_t hi = (v >> (len - 1)) & 1;
        v = ((v << 1) & ((1u << len) - 1)) | (uint32_t(dir) ^ hi);
        v ^= uint32_t((ghr >> (kGhrLen[i] - 1)) & 1) << (kGhrLen[i] % len);
        fh[k][i] = v;
      }
    ghr = (ghr << 1) | dir;
  }
};

struct Ras {
  target_addr_t stack[RAS_DEPTH] = {};
  int count = 0;
  void push(target_addr_t v) {
    if (count < RAS_DEPTH) { stack[count++] = v; return; }
    for (int i = 1; i < RAS_DEPTH; i++) stack[i - 1] = stack[i];
    stack[RAS_DEPTH - 1] = v;
  }
  void pop() { if (count > 0) count--; }
};

// ---------------------------------------------------------------------------
// TAGE direction predictor (one per bank)
// ---------------------------------------------------------------------------
struct TageWr {
  bool upd = false;               // an update ran: LSFR steps, reset counter ticks
  uint8_t lsfr_next = 0;
  uint32_t reset_cnt_next = 0;
  bool ua_we = false;
  tage_use_alt_ctr_t ua_val = 0;
  bool base_we = false;
  uint32_t base_idx = 0;
  uint8_t base_val = 0;
  uint8_t cnt_we = 0, useful_we = 0, tag_we = 0, reset_we = 0;  // bitmasks over tables
  uint32_t idx[TN] = {};
  uint32_t reset_row = 0;
  uint8_t cnt_v[TN] = {}, useful_v[TN] = {}, reset_v[TN] = {};
  tage_tag_t tag_v[TN] = {};
};

struct TagePred {
  bool pred, alt;
  uint8_t pcpn, altpcpn;
  tage_tag_t tag[TN];
  tage_idx_t idx[TN];
};

struct TageUpd {
  pc_t bank_pc;
  bool real_dir, pred, alt;
  int pcpn;
  tage_tag_t tag[TN];
  tage_idx_t idx[TN];
  bool sc_used, loop_used;
};

class Tage {
  uint8_t base_[BASE_ENTRY_NUM];
  tage_tag_t tag_[TN][TN_ENTRY_NUM];
  uint8_t cnt_[TN][TN_ENTRY_NUM], useful_[TN][TN_ENTRY_NUM];
  uint8_t lsfr_ = 8;  // bit i = LSFR[i]; reset value {0,0,0,1}
  uint32_t reset_cnt_ = 0;
  tage_use_alt_ctr_t use_alt_ = static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_INIT);

  static uint8_t inc2(uint8_t v) { return v >= 3 ? 3 : v + 1; }
  static uint8_t dec2(uint8_t v) { return v == 0 ? 0 : v - 1; }
  static bool weak(uint8_t c) { return c >= TAGE_PROVIDER_WEAK_LOW && c <= TAGE_PROVIDER_WEAK_HIGH; }

 public:
  Tage() { std::memset(this, 0, sizeof(*this)); lsfr_ = 8; use_alt_ = static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_INIT); }

  void predict(pc_t pc, const uint32_t fh[FH_N_MAX][TN], TagePred &o) const {
    const uint32_t w = pc >> 2;
    uint32_t b = w & TAGE_BASE_IDX_MASK;
    int pcpn = TN, altpcpn = TN;
    for (int i = 0; i < TN; i++) {
      o.tag[i] = (fh[1][i] ^ fh[2][i] ^ w) & TAGE_TAG_MASK;
      o.idx[i] = (fh[0][i] ^ w) & TAGE_IDX_MASK;
    }
    uint8_t cnt[TN], useful[TN];
    for (int i = 0; i < TN; i++) { cnt[i] = cnt_[i][o.idx[i]]; useful[i] = useful_[i][o.idx[i]]; }
    for (int i = TN - 1; i >= 0; i--)
      if (tag_[i][o.idx[i]] == o.tag[i]) { pcpn = i; break; }
    for (int i = pcpn - 1; i >= 0; i--)
      if (tag_[i][o.idx[i]] == o.tag[i]) { altpcpn = i; break; }
    const bool base_pred = base_[b] >= 2;
    o.alt = altpcpn >= TN ? base_pred : cnt[altpcpn] >= 4;
    o.pcpn = pcpn;
    o.altpcpn = altpcpn;
    if (pcpn >= TN) { o.pred = base_pred; return; }
    o.pred = cnt[pcpn] >= 4;
    if (weak(cnt[pcpn]) && useful[pcpn] <= TAGE_USE_ALT_USEFUL_THRESHOLD &&
        use_alt_ <= static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_USE_ALT_THRESHOLD))
      o.pred = o.alt;
  }

  void update(const TageUpd &u, TageWr &w) const {
    w = TageWr{};
    w.upd = true;
    const bool dir = u.real_dir;
    const int pcpn = u.pcpn;
    uint8_t cnt[TN], useful[TN];
    for (int i = 0; i < TN; i++) {
      w.idx[i] = u.idx[i] & TAGE_IDX_MASK;
      cnt[i] = cnt_[i][w.idx[i]];
      useful[i] = useful_[i][w.idx[i]];
    }
    const uint32_t bidx = (u.bank_pc >> 2) & TAGE_BASE_IDX_MASK;
    const uint8_t base_cnt = base_[bidx];

    // LSFR (4-bit): shift in bit0^bit3
    const uint8_t ln = uint8_t(((lsfr_ << 1) & 0xE) | ((lsfr_ ^ (lsfr_ >> 3)) & 1));
    w.lsfr_next = ln;
    const int rnd = ((ln & 1) << 1) | ((ln >> 1) & 1);
    w.reset_cnt_next = reset_cnt_ + 1;

    if (pcpn < TN) {
      const bool praw = cnt[pcpn] >= 4;
      const bool provider_used = !(u.sc_used || u.loop_used);
      const bool alt_used = provider_used && (u.pred == u.alt);
      uint8_t nu = useful[pcpn];
      bool wu = false;
      if (!provider_used) { nu = praw == dir ? inc2(nu) : dec2(nu); wu = true; }
      else if (!alt_used) { nu = u.pred == dir ? inc2(nu) : dec2(nu); wu = true; }
      else if (praw != dir && u.alt == dir) { nu = dec2(nu); wu = true; }
      if (wu) { w.useful_we |= 1u << pcpn; w.useful_v[pcpn] = nu; }
      w.cnt_we |= 1u << pcpn;
      w.cnt_v[pcpn] = dir ? (cnt[pcpn] >= 7 ? 7 : cnt[pcpn] + 1) : (cnt[pcpn] == 0 ? 0 : cnt[pcpn] - 1);
      if (weak(cnt[pcpn]) && useful[pcpn] <= TAGE_USE_ALT_USEFUL_THRESHOLD && praw != u.alt &&
          ((praw == dir) ^ (u.alt == dir))) {
        constexpr tage_use_alt_ctr_t kMax = static_cast<tage_use_alt_ctr_t>((1u << TAGE_USE_ALT_CTR_BITS) - 1u);
        w.ua_we = true;
        w.ua_val = u.alt == dir ? (use_alt_ == 0 ? 0 : use_alt_ - 1) : (use_alt_ >= kMax ? kMax : use_alt_ + 1);
      }
    } else {
      w.base_we = true;
      w.base_idx = bidx;
      w.base_val = dir ? inc2(base_cnt) : dec2(base_cnt);
    }

    if (u.pred != dir && (pcpn <= TN - 2 || pcpn == TN)) {
      const int start = pcpn == TN ? 0 : pcpn + 1;
      int j = -1, k = -1;
      for (int i = start; i < TN; i++)
        if (useful[i] == 0) {
          if (j < 0) j = i;
          else { k = i; break; }
        }
      if (j < 0) {
        for (int i = start; i < TN; i++) { w.useful_we |= 1u << i; w.useful_v[i] = dec2(useful[i]); }
      } else {
        const int t = (k >= 0 && rnd % 3 == 0) ? k : j;
        w.tag_we |= 1u << t;  w.tag_v[t] = u.tag[t];
        w.cnt_we |= 1u << t;  w.cnt_v[t] = dir ? 4 : 3;
        w.useful_we |= 1u << t; w.useful_v[t] = 0;
      }
    }

    // periodic useful-bit reset sweep
    if ((reset_cnt_ & 0x7ff) == 0) {
      const uint32_t row = ((reset_cnt_ >> 11) & 0xfff) & TAGE_IDX_MASK;
      const bool msb_only = (reset_cnt_ >> 23) & 1;
      w.reset_row = row;
      for (int i = 0; i < TN; i++) {
        if ((w.useful_we >> i & 1) && w.idx[i] == row) continue;
        const uint8_t cur = useful_[i][row];
        w.reset_we |= 1u << i;
        w.reset_v[i] = msb_only ? (cur & 1) : (cur & 2);
      }
    }
  }

  void commit(const TageWr &w) {
    if (!w.upd) return;
    lsfr_ = w.lsfr_next;
    reset_cnt_ = w.reset_cnt_next;
    if (w.ua_we) use_alt_ = w.ua_val;
    if (w.base_we) base_[w.base_idx] = w.base_val;
    for (int i = 0; i < TN; i++) {
      if (w.cnt_we >> i & 1) cnt_[i][w.idx[i]] = w.cnt_v[i];
      if (w.useful_we >> i & 1) useful_[i][w.idx[i]] = w.useful_v[i];
      if (w.tag_we >> i & 1) tag_[i][w.idx[i]] = w.tag_v[i];
      if (w.reset_we >> i & 1) useful_[i][w.reset_row] = w.reset_v[i];
    }
  }
};

// ---------------------------------------------------------------------------
// BTB + indirect target cache (one per bank)
// ---------------------------------------------------------------------------
struct BtbWr {
  bool bht_we = false, btb_we = false, tc_we = false, mode_we = false;
  uint32_t bht_idx = 0;  bht_hist_t bht_v = 0;
  uint32_t btb_way = 0, btb_idx = 0;  btb_tag_t btb_tag = 0;  target_addr_t btb_bta = 0;  uint8_t btb_useful = 0;
  uint32_t tc_way = 0, tc_idx = 0, pre_way = 0, pre_idx = 0;
  tc_tag_t tc_tag = 0;  target_addr_t tc_target = 0;  uint8_t tc_useful = 0, pre_useful = 0;
  uint32_t mode_idx = 0;  wire3_t mode_v = 0;
};

class Btb {
  static constexpr wire3_t kModeBias = 3, kModeInit = 2, kModeMax = 6;
  struct Way { btb_tag_t tag; target_addr_t bta; bool valid; wire3_t useful; };
  struct Tc { target_addr_t target; tc_tag_t tag; bool valid; wire3_t useful; };
  Way btb_[BTB_ENTRY_NUM][BTB_WAY_NUM];
  bht_hist_t bht_[BHT_ENTRY_NUM];
  wire3_t mode_[BHT_ENTRY_NUM];
  Tc tc_[TC_ENTRY_NUM][TC_WAY_NUM], pre_[TC_ENTRY_NUM][TC_WAY_NUM];

  static constexpr int width(uint32_t m) { int w = 0; while (m) { w++; m >>= 1; } return w; }
  static uint32_t hash(uint32_t pcw, uint32_t mask, uint32_t salt) {
    const int wd = width(mask);
    uint32_t x = pcw;
    if (wd < 32) x ^= pcw >> wd;
    if (2 * wd < 32) x ^= pcw >> (2 * wd);
    if (3 * wd < 32) x ^= pcw >> (3 * wd);
    return (x ^ salt) & mask;
  }
  static uint32_t btb_idx(pc_t pc) { return hash(pc >> 2, BTB_IDX_MASK, 0x9e3779b9u); }
  static uint32_t bht_idx(pc_t pc) { return hash(pc >> 2, BHT_IDX_MASK, 0x6a5d39e1u); }
  static btb_tag_t btb_tag(pc_t pc) { return ((pc >> 2) >> ceil_log2_u32(BTB_ENTRY_NUM)) & BTB_TAG_MASK; }
  static uint32_t tc_idx(pc_t pc, bht_hist_t b) {
    const uint32_t h = b ^ (b >> 7) ^ (b >> 13) ^ (uint32_t(b) << 9);
    return hash((pc >> 2) ^ h ^ (uint32_t(b) >> 19), TC_ENTRY_MASK, 0x51ed270bu);
  }
  static tc_tag_t tc_tag(pc_t pc) { return hash(pc >> 2, TC_TAG_MASK, 0x3c6ef372u); }
  static uint8_t useful_next(uint8_t v, bool ok) { return ok ? (v < 7 ? v + 1 : v) : (v > 0 ? v - 1 : 0); }
  static bht_hist_t bht_next(bht_hist_t cur, br_type_t t, bool dir, target_addr_t tgt) {
    if (t == BR_IDIRECT) {
      const uint32_t tw = tgt >> 2, tf = tw ^ (tw >> 11) ^ (tw >> 19);
      return (cur ^ (uint32_t(cur) << 5) ^ (cur >> 2)) ^ tf ^ 0x9e3779b9u;
    }
    return (uint32_t(cur) << 1) | dir;
  }
  template <class S, int N>
  static int hit_way(const S (&set)[N], uint32_t tag) {
    for (int w = 0; w < N; w++) if (set[w].valid && set[w].tag == tag) return w;
    return -1;
  }
  template <class S, int N>
  static int victim(const S (&set)[N]) {
    for (int w = 0; w < N; w++) if (!set[w].valid) return w;
    int m = 255, mw = 0;
    for (int w = 0; w < N; w++) if (set[w].useful < m) { m = set[w].useful; mw = w; }
    return mw;
  }

 public:
  Btb() { std::memset(this, 0, sizeof(*this)); for (auto &m : mode_) m = kModeInit; }

  // bank_pc is the bank-local pc, exactly what the RTL hands the bank
  target_addr_t predict(pc_t pc, br_type_t type) const {
    const Way(&set)[BTB_WAY_NUM] = btb_[btb_idx(pc)];
    const int h = hit_way(set, btb_tag(pc));
    if (type == BR_IDIRECT) {
      const uint32_t bi = bht_idx(pc), ti = tc_idx(pc, bht_[bi]);
      const tc_tag_t tg = tc_tag(pc);
      const int a = hit_way(tc_[ti], tg), p = hit_way(pre_[ti], tg);
      if (p >= 0 && (a < 0 || mode_[bi] >= kModeBias)) return pre_[ti][p].target;
      if (a >= 0) return tc_[ti][a].target;
      return h >= 0 ? set[h].bta : pc + 4;
    }
    if (type == BR_DIRECT || type == BR_CALL || type == BR_JAL || type == BR_RET)
      return h >= 0 ? set[h].bta : pc + 4;
    return pc + 4;
  }

  void update(pc_t pc, br_type_t type, bool dir, target_addr_t addr, BtbWr &w) const {
    w = BtbWr{};
    const uint32_t bi = bht_idx(pc);
    const bht_hist_t cur = bht_[bi];
    const bht_hist_t nxt = type != BR_NONCTL ? bht_next(cur, type, dir, addr) : cur;
    if (type != BR_NONCTL) { w.bht_we = true; w.bht_idx = bi; w.bht_v = nxt; }

    const bool writes = type == BR_DIRECT || type == BR_CALL || type == BR_RET || type == BR_JAL || type == BR_IDIRECT;
    if (dir && writes) {
      const uint32_t ei = btb_idx(pc);
      const btb_tag_t tg = btb_tag(pc);
      const Way(&set)[BTB_WAY_NUM] = btb_[ei];
      const int h = hit_way(set, tg);
      const int way = h >= 0 ? h : victim(set);
      w.btb_we = true;
      w.btb_way = way; w.btb_idx = ei; w.btb_tag = tg; w.btb_bta = addr;
      w.btb_useful = type == BR_IDIRECT ? INDIRECT_BTB_INIT_USEFUL
                     : (h >= 0 ? useful_next(set[way].useful, set[way].bta == addr) : 1);
    }

    if (dir && type == BR_IDIRECT) {
      const uint32_t pi = tc_idx(pc, cur), wi = tc_idx(pc, nxt);
      const tc_tag_t tg = tc_tag(pc);
      const Tc(&ws)[TC_WAY_NUM] = tc_[wi];
      const int wh = hit_way(ws, tg);
      const int wway = wh >= 0 ? wh : victim(ws);
      w.tc_we = true;
      w.tc_way = wway; w.tc_idx = wi; w.tc_tag = tg; w.tc_target = addr;
      w.tc_useful = wh >= 0 ? useful_next(ws[wway].useful, ws[wway].target == addr) : INDIRECT_TC_INIT_USEFUL;

      const Tc(&ps)[TC_WAY_NUM] = pre_[pi];
      const int ph = hit_way(ps, tg);
      const int pway = ph >= 0 ? ph : victim(ps);
      const bool pre_ok = ph >= 0 && ps[pway].target == addr;
      w.pre_way = pway; w.pre_idx = pi;
      w.pre_useful = ph >= 0 ? useful_next(ps[pway].useful, pre_ok) : INDIRECT_TC_INIT_USEFUL;

      const int mh = hit_way(tc_[pi], tg);
      const bool main_ok = mh >= 0 && tc_[pi][mh].target == addr;
      if (main_ok != pre_ok) {
        const wire3_t m = mode_[bi];
        w.mode_we = true; w.mode_idx = bi;
        w.mode_v = pre_ok ? (m < kModeMax ? m + 1 : kModeMax) : (m > 0 ? m - 1 : 0);
      }
    }
  }

  void commit(const BtbWr &w) {
    if (w.bht_we) bht_[w.bht_idx] = w.bht_v;
    if (w.tc_we) {
      tc_[w.tc_idx][w.tc_way] = {w.tc_target, w.tc_tag, true, w.tc_useful};
      pre_[w.pre_idx][w.pre_way] = {w.tc_target, w.tc_tag, true, w.pre_useful};
    }
    if (w.mode_we) mode_[w.mode_idx] = w.mode_v;
    if (w.btb_we) btb_[w.btb_idx][w.btb_way] = {w.btb_tag, w.btb_bta, true, w.btb_useful};
  }
};

// ---------------------------------------------------------------------------
// Branch-type predictor (banked, set-associative)
// ---------------------------------------------------------------------------
struct TypeEntry { bool valid; type_pred_tag_t tag; br_type_t type; type_pred_conf_t conf; type_pred_age_t age; };
struct TypeWr { bool en = false; uint32_t bank = 0, set = 0, way = 0; TypeEntry e{}; };

class TypePred {
  TypeEntry t_[NB][TYPE_PRED_SET_NUM][TYPE_PRED_WAY_NUM];
  static void locate(pc_t pc, uint32_t &bank, uint32_t &set, type_pred_tag_t &tag) {
    bank = bank_of(pc);
    const uint32_t w = bank_pc_of(pc) >> 2;
    set = w & TYPE_PRED_SET_MASK;
    tag = (w >> ceil_log2_u32(TYPE_PRED_SET_NUM)) & TYPE_PRED_TAG_MASK;
  }
 public:
  TypePred() { reset(); }
  void reset() { std::memset(t_, 0, sizeof(t_)); }

  br_type_t predict(pc_t pc) const {
    uint32_t b, s; type_pred_tag_t tg; locate(pc, b, s, tg);
    for (auto &e : t_[b][s]) if (e.valid && e.tag == tg) return e.type;
    return BR_NONCTL;
  }

  void update(pc_t pc, br_type_t type, TypeWr &w) const {
    w = TypeWr{};
    uint32_t b, s; type_pred_tag_t tg; locate(pc, b, s, tg);
    const TypeEntry(&set)[TYPE_PRED_WAY_NUM] = t_[b][s];
    int hit = -1;
    for (int i = 0; i < TYPE_PRED_WAY_NUM; i++) if (set[i].valid && set[i].tag == tg) { hit = i; break; }
    w.bank = b; w.set = s;
    if (type == BR_NONCTL) {
      if (hit < 0) return;
      w.en = true; w.way = hit; w.e = TypeEntry{};
      return;
    }
    int vic = 0;
    bool found = false;
    for (int i = 0; i < TYPE_PRED_WAY_NUM; i++) if (!set[i].valid) { vic = i; found = true; break; }
    if (!found) for (int i = 1; i < TYPE_PRED_WAY_NUM; i++) if (set[i].age < set[vic].age) vic = i;
    w.en = true;
    w.way = hit >= 0 ? hit : vic;
    TypeEntry e = hit >= 0 ? set[w.way] : TypeEntry{};
    e.valid = true; e.tag = tg;
    if (hit >= 0 && e.type == type) {
      if (e.conf < TYPE_PRED_CONF_MAX) e.conf++;
      if (e.age < TYPE_PRED_AGE_MAX) e.age++;
    } else { e.type = type; e.conf = 1; e.age = 1; }
    w.e = e;
  }

  void commit(const TypeWr &w) { if (w.en) t_[w.bank][w.set][w.way] = w.e; }
};


// ---------------------------------------------------------------------------
// Top level
// ---------------------------------------------------------------------------
struct InputPayload {
  wire1_t refetch;
  fetch_addr_t refetch_address;
  wire1_t icache_read_ready;
  // commit-stream update
  pc_t in_update_base_pc[CW];
  wire1_t in_upd_valid[CW];
  wire1_t in_actual_dir[CW];
  br_type_t in_actual_br_type[CW];
  target_addr_t in_actual_targets[CW];
  wire1_t in_pred_dir[CW];
  wire1_t in_alt_pred[CW];
  pcpn_t in_pcpn[CW];
  pcpn_t in_altpcpn[CW];
  tage_tag_t in_tage_tags[CW][TN];
  tage_idx_t in_tage_idxs[CW][TN];
  wire1_t in_sc_used[CW];
  wire1_t in_loop_used[CW];
};

struct OutputPayload {
  fetch_addr_t fetch_address;
  wire1_t icache_read_valid;
  fetch_addr_t predict_next_fetch_address;
  wire1_t PTAB_write_enable;
  wire1_t out_pred_dir[FW];
  PredMeta meta;
  pc_t out_pred_base_pc;
  wire1_t update_queue_full;  // the update queue drops instead of stalling: always 0
};

class Bpu {
 public:
  struct QEntry {
    pc_t pc;
    bool dir, pred, alt, sc_used, loop_used;
    br_type_t type;
    target_addr_t target;
    pcpn_t pcpn, altpcpn;
    tage_tag_t tag[TN];
    tage_idx_t idx[TN];
  };

  struct Txn {
    bool do_spec;
    pc_t pc_next;
    uint8_t spec_n;
    bool spec_dir[FW];
    int8_t spec_ras;  // 0 none, 1 push, 2 pop
    target_addr_t spec_ras_val;
    TageWr tage[NB];
    BtbWr btb[NB];
    TypeWr type[CW];
    bool pop[NB], push[CW];
    uint32_t push_bank[CW];
    QEntry push_e[CW];
  };

 private:
  pc_t pc_ = RESET_PC;
  Hist spec_, arch_;
  Ras spec_ras_, arch_ras_;
  Tage tage_[NB];
  Btb btb_[NB];
  TypePred type_;
  QEntry q_[Q_DEPTH][NB];
  int wr_[NB] = {}, rd_[NB] = {}, cnt_[NB] = {};

 public:
  void reset() {  // TAGE/BTB arrays keep their contents, as in the RTL-style model
    pc_ = RESET_PC;
    spec_ = arch_ = Hist{};
    spec_ras_ = arch_ras_ = Ras{};
    type_.reset();
    for (int b = 0; b < NB; b++) wr_[b] = rd_[b] = cnt_[b] = 0;
  }

  void comb(const InputPayload &in, OutputPayload &out, Txn &t) const {
    std::memset(&out, 0, sizeof(out));
    std::memset(&t, 0, sizeof(t));
    const bool refetch = in.refetch;
    const pc_t base = refetch ? in.refetch_address : pc_;
    const uint32_t mask = ~(ICACHE_LINE_SIZE - 1);
    const uint32_t plus = base + FW * 4;
    const uint32_t boundary = (base & mask) != (plus & mask) ? (plus & mask) : plus;
    const bool go_pred = in.icache_read_ready || refetch;

    out.icache_read_valid = !refetch;
    out.fetch_address = out.out_pred_base_pc = base;
    out.PTAB_write_enable = go_pred && !refetch;
    uint32_t next = boundary;

    if (go_pred) {
      const Ras &ras = refetch ? arch_ras_ : spec_ras_;
      bool taken_found = false, spec_done = false;
      t.do_spec = !refetch;
      for (int i = 0; i < FW; i++) {
        const pc_t pc = base + i * 4;
        if (pc >= boundary) continue;
        const uint32_t b = bank_of(pc);
        const pc_t bpc = bank_pc_of(pc);
        const br_type_t ty = type_.predict(pc);
        TagePred tp;
        tage_[b].predict(bpc, spec_.fh, tp);
        const bool taken = ty == BR_NONCTL ? false
                           : (ty == BR_RET || ty == BR_CALL || ty == BR_IDIRECT || ty == BR_JAL) ? true
                           : tp.pred;
        out.out_pred_dir[i] = taken;
        out.meta.alt_pred[i] = tp.alt;
        out.meta.pcpn[i] = tp.pcpn;
        out.meta.altpcpn[i] = tp.altpcpn;
        for (int k = 0; k < TN; k++) { out.meta.tage_tag[i][k] = tp.tag[k]; out.meta.tage_idx[i][k] = tp.idx[k]; }
        if (taken && !taken_found) {
          target_addr_t tgt = btb_[b].predict(bpc, ty);
          if (ty == BR_RET && ras.count > 0) tgt = ras.stack[ras.count - 1];
          next = tgt;
          taken_found = true;
        }
        if (t.do_spec && !spec_done) {
          if (ty == BR_DIRECT) t.spec_dir[t.spec_n++] = taken;
          if (taken) {
            spec_done = true;
            if (ty == BR_CALL) { t.spec_ras = 1; t.spec_ras_val = pc + 4; }
            else if (ty == BR_RET) t.spec_ras = 2;
          }
        }
      }
    }
    out.predict_next_fetch_address = next;
    t.pc_next = refetch ? in.refetch_address : go_pred ? next : pc_;

    // update path: queue head -> TAGE/BTB; commit stream -> type predictor and queue
    for (int b = 0; b < NB; b++) {
      if (cnt_[b] == 0) continue;
      const QEntry &e = q_[rd_[b]][b];
      t.pop[b] = true;
      const pc_t bpc = bank_pc_of(e.pc);
      if (e.type == BR_DIRECT) {
        TageUpd u;
        u.bank_pc = bpc; u.real_dir = e.dir; u.pred = e.pred; u.alt = e.alt;
        u.pcpn = e.pcpn;
        for (int k = 0; k < TN; k++) { u.tag[k] = e.tag[k]; u.idx[k] = e.idx[k]; }
        u.sc_used = e.sc_used; u.loop_used = e.loop_used;
        tage_[b].update(u, t.tage[b]);
      }
      if (e.type != BR_NONCTL) btb_[b].update(bpc, e.type, e.dir, e.target, t.btb[b]);
    }
    int cnt_next[NB];
    for (int b = 0; b < NB; b++) cnt_next[b] = cnt_[b] - (t.pop[b] ? 1 : 0);
    bool enq[NB] = {};
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      type_.update(in.in_update_base_pc[i], in.in_actual_br_type[i], t.type[i]);
      const uint32_t b = bank_of(in.in_update_base_pc[i]);
      if (enq[b] || cnt_next[b] >= Q_DEPTH) continue;
      enq[b] = true;
      cnt_next[b]++;
      t.push[i] = true;
      t.push_bank[i] = b;
      QEntry &e = t.push_e[i];
      e.pc = in.in_update_base_pc[i];
      e.dir = in.in_actual_dir[i]; e.pred = in.in_pred_dir[i]; e.alt = in.in_alt_pred[i];
      e.sc_used = in.in_sc_used[i]; e.loop_used = in.in_loop_used[i];
      e.type = in.in_actual_br_type[i]; e.target = in.in_actual_targets[i];
      e.pcpn = in.in_pcpn[i]; e.altpcpn = in.in_altpcpn[i];
      for (int k = 0; k < TN; k++) { e.tag[k] = in.in_tage_tags[i][k]; e.idx[k] = in.in_tage_idxs[i][k]; }
    }
  }

  void commit(const InputPayload &in, const Txn &t) {
    for (int b = 0; b < NB; b++) { tage_[b].commit(t.tage[b]); btb_[b].commit(t.btb[b]); }
    for (int i = 0; i < CW; i++) type_.commit(t.type[i]);

    for (int i = 0; i < t.spec_n; i++) spec_.push(t.spec_dir[i]);
    if (t.spec_ras == 1) spec_ras_.push(t.spec_ras_val);
    else if (t.spec_ras == 2) spec_ras_.pop();
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const br_type_t ty = in.in_actual_br_type[i];
      if (ty == BR_DIRECT) {
        arch_.push(in.in_actual_dir[i]);
        if (in.in_pred_dir[i] != in.in_actual_dir[i]) spec_ = arch_;
      } else if (ty == BR_CALL) arch_ras_.push(in.in_update_base_pc[i] + 4);
      else if (ty == BR_RET) arch_ras_.pop();
    }
    if (in.refetch) { spec_ = arch_; spec_ras_ = arch_ras_; }
    pc_ = t.pc_next;

    for (int i = 0; i < CW; i++)
      if (t.push[i]) {
        const uint32_t b = t.push_bank[i];
        q_[wr_[b]][b] = t.push_e[i];
        wr_[b] = (wr_[b] + 1) % Q_DEPTH;
        cnt_[b]++;
      }
    for (int b = 0; b < NB; b++)
      if (t.pop[b]) { rd_[b] = (rd_[b] + 1) % Q_DEPTH; cnt_[b]--; }
  }
};

}  // namespace bpuf

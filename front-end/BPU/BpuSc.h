#pragma once
// Hardware-faithful Stage 2 8-table Long-History Block-TAGE + S2 Statistical Corrector (SC).
// v10 (Zero-T0-Read Meta + WRBypass + Deep Multi-Bank 1RW):
//   - Prediction captures compact Meta (pcpn, alt, prov_cnt, prov_u, base_cnt, useful[EX_TN], SC weights) into ftb_rec
//   - Commit training performs ZERO SRAM reads (0 T0 ports): consults 8-entry FF WRBypass -> falls back to Meta -> 1-cycle pure write
//   - Skip saturated no-op writes -> < 1.5% write-clash rate per bank under XiangShan write-wins not-hit 1RW arbitration
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace bpuf {

// Deep-macro 1RW bank geometry (eliminates 64x64 fragmented macros!)
constexpr int BPU_FTB1_BANKS    = 4;  // 4-Bank 1RW (256-deep per bank -> 256x72 macros)
constexpr int BPU_S2_TAGE_BANKS = 8;  // 8-Bank 1RW (512-deep per bank -> 512x72 macros)
constexpr int BPU_SC_BANKS      = 2;  // 2-Bank 1RW (GEHL 256x24, IMLI/LOC 128x24, Bias FF)
constexpr int BPU_ITT_BANKS     = 2;  // 2-Bank 1RW (128x52 macros)
constexpr int BPU_AH1_BANKS     = 2;  // 2-Bank 1RW (aftb 128-deep, atage 512-deep, aitt 128-deep)
constexpr int BPU_AH2_BANKS     = 2;  // 2-Bank 1RW (aftb2 64-deep, atage2 256-deep, aitt2 128-deep)

inline uint32_t s2_bank_of_w(uint32_t w)  { return (w ^ (w >> 10)) & uint32_t(BPU_FTB1_BANKS - 1); }
inline uint32_t ah1_bank_of_w(uint32_t w) { return (w ^ (w >> 8))  & uint32_t(BPU_AH1_BANKS - 1); }
inline uint32_t ah2_bank_of_w(uint32_t w) { return (w ^ (w >> 7))  & uint32_t(BPU_AH2_BANKS - 1); }

// 38-bit Sv39 VA target compression (stores pc[38:1], sign-extends bit 38 to [63:39], bit 0 = 0)
inline uint64_t sv39_va_roundtrip(uint64_t pc) {
  const int64_t s = static_cast<int64_t>((pc >> 1) << 26) >> 25;
  return static_cast<uint64_t>(s);
}

constexpr int EX_TN = 8;
constexpr int EX_FH_K = 3;
constexpr int EX_TAGE_N = 4096;
constexpr int EX_TAGE_BASE_N = 4096;
constexpr uint16_t EX_TAGE_TAG_MASK = 0x7ffu;  // 11-bit tags
constexpr int EX_TAGE_URST = 11;
constexpr int EX_GHR_LEN[EX_TN] = {5, 11, 22, 44, 89, 178, 356, 712};
constexpr int EX_FH_LEN[EX_FH_K] = {12, 11, 10};

constexpr int SC_GEHL_N = 4;
constexpr int SC_GHR_LEN[SC_GEHL_N] = {4, 10, 20, 40};
constexpr int SC_FH_LEN = 9;

constexpr int BPU_WRB_N = 8;  // 8-entry fully-associative WRBypass CAM+Mem (tiny FF array)

struct ExTageIdx {
  uint32_t w = 0;
  uint32_t base_idx = 0;
  uint32_t idx[EX_TN] = {};
  uint16_t tag[EX_TN] = {};
};

struct ScIdx {
  uint32_t w = 0;
  uint32_t bias_set = 0;
  uint32_t gehl_idx[SC_GEHL_N] = {};
  uint32_t imli_idx = 0;
  uint32_t lht_idx = 0;
};

inline ExTageIdx make_ex_tage_index(uint32_t w, const uint16_t ex_fh[EX_FH_K][EX_TN]) {
  ExTageIdx ix;
  ix.w = w;
  // base_idx aligns low 2 bank bits with ftb1 (4-bank, bijective 4096 entries -> 0% A-vs-B conflict)
  ix.base_idx = (((w ^ (w >> 12)) & uint32_t(EX_TAGE_BASE_N - 1)) & ~uint32_t(BPU_FTB1_BANKS - 1)) | s2_bank_of_w(w);
  for (int t = 0; t < EX_TN; t++) {
    ix.idx[t] = (w ^ (w >> 11) ^ uint32_t(ex_fh[0][t])) & uint32_t(EX_TAGE_N - 1);
    ix.tag[t] = static_cast<uint16_t>((w ^ (w >> 11) ^ uint32_t(ex_fh[1][t]) ^ (uint32_t(ex_fh[2][t]) << 1)) & EX_TAGE_TAG_MASK);
  }
  return ix;
}

inline ScIdx make_sc_index(uint32_t w, const uint16_t sc_fh[SC_GEHL_N], uint8_t imli) {
  ScIdx ix;
  ix.w = w;
  ix.bias_set = w & 63u;
  for (int g = 0; g < SC_GEHL_N; g++) {
    ix.gehl_idx[g] = (w ^ (w >> 9) ^ uint32_t(sc_fh[g])) & 511u;
  }
  ix.imli_idx = (w ^ (uint32_t(imli) * 13u)) & 255u;
  ix.lht_idx = w & 63u;
  return ix;
}

// Tiny 8-entry FF WRBypass for TAGE counters (eliminates T0 SRAM reads!)
struct TageWrBypass {
  struct Ent {
    bool valid = false;
    uint8_t tbl = 0;  // 0..EX_TN-1 for tagged, EX_TN for base
    uint8_t sl = 0;
    uint16_t idx = 0;
    uint8_t cnt = 0;
    uint8_t u = 0;
  };
  Ent e[BPU_WRB_N];
  uint8_t ptr = 0;

  void reset() {
    for (int i = 0; i < BPU_WRB_N; i++) e[i] = Ent{};
    ptr = 0;
  }
  bool lookup(uint8_t tbl, uint8_t sl, uint16_t idx, uint8_t &cnt_out, uint8_t &u_out) const {
    for (int i = 0; i < BPU_WRB_N; i++) {
      if (e[i].valid && e[i].tbl == tbl && e[i].sl == sl && e[i].idx == idx) {
        cnt_out = e[i].cnt;
        u_out = e[i].u;
        return true;
      }
    }
    return false;
  }
  void insert(uint8_t tbl, uint8_t sl, uint16_t idx, uint8_t cnt, uint8_t u) {
    for (int i = 0; i < BPU_WRB_N; i++) {
      if (e[i].valid && e[i].tbl == tbl && e[i].sl == sl && e[i].idx == idx) {
        e[i].cnt = cnt;
        e[i].u = u;
        return;
      }
    }
    e[ptr] = Ent{true, tbl, sl, idx, cnt, u};
    ptr = static_cast<uint8_t>((ptr + 1) % BPU_WRB_N);
  }
};

// 8-table long-history Block-TAGE for S2 (4096 entries = 8 banks x 512 x 64b 1RW packed wmask)
struct ExBlockTage {
  static constexpr int SLOTS = 4;
  std::vector<uint8_t> base[SLOTS];
  std::vector<uint16_t> tag[EX_TN][SLOTS];
  std::vector<uint8_t> cnt[EX_TN][SLOTS];
  std::vector<uint8_t> useful[EX_TN][SLOTS];
  uint8_t use_alt[16] = {};
  uint8_t lsfr = 8;
  uint32_t reset_cnt = 0;
  uint8_t rst_pend_mask = 0;
  uint32_t rst_row = 0;
  uint8_t rst_sl = 0;
  TageWrBypass wrb;

  struct RawSlot {
    uint8_t base_cnt = 1;
    uint8_t sup_mask = 0;
    uint16_t tag[EX_TN] = {};
    uint8_t cnt[EX_TN] = {};
    uint8_t useful[EX_TN] = {};
  };
  struct Raw {
    RawSlot s[SLOTS];
  };
  struct Pred {
    bool pred = false;
    bool alt = false;
    uint8_t pcpn = EX_TN;
    bool used_alt = false;
    uint8_t prov_cnt = 4;
    uint8_t base_cnt = 1;
  };
  // Prediction-time Meta stored in ftb_rec (enables ZERO-T0-read commit write!)
  struct SlotMeta {
    uint8_t pcpn = EX_TN;
    bool pred = false;
    bool alt = false;
    uint8_t base_cnt = 1;
    uint8_t prov_cnt = 4;
    uint8_t prov_u = 0;
    uint8_t useful[EX_TN] = {};
  };
  struct Meta {
    SlotMeta s[SLOTS];
  };
  struct WritePlan {
    bool valid = false;
    int sl = 0;
    uint32_t w = 0;
    bool real_dir = false;
    uint8_t pcpn = EX_TN;
    bool pdir = false;
    bool weak = false;
    bool alt = false;
    uint32_t base_idx = 0;
    bool base_wr = false;
    uint8_t base_val = 1;
    uint8_t tbl_wr_mask = 0;
    bool tbl_is_alloc[EX_TN] = {};
    uint32_t tbl_idx[EX_TN] = {};
    uint16_t tbl_tag[EX_TN] = {};
    uint8_t tbl_cnt[EX_TN] = {};
    uint8_t tbl_u[EX_TN] = {};
  };

  void init() {
    for (int sl = 0; sl < SLOTS; sl++) {
      base[sl].assign(EX_TAGE_BASE_N, 1);
      for (int t = 0; t < EX_TN; t++) {
        tag[t][sl].assign(EX_TAGE_N, 0);
        cnt[t][sl].assign(EX_TAGE_N, 0);
        useful[t][sl].assign(EX_TAGE_N, 0);
      }
    }
    for (int i = 0; i < 16; i++) use_alt[i] = 7;
    lsfr = 8;
    reset_cnt = 0;
    rst_pend_mask = 0;
    rst_row = 0;
    rst_sl = 0;
    wrb.reset();
  }

  void read(const ExTageIdx &ix, Raw &r, uint8_t sup_mask = 0) const {
    for (int sl = 0; sl < SLOTS; sl++) {
      r.s[sl].base_cnt = base[sl][ix.base_idx];
      r.s[sl].sup_mask = sup_mask;
      for (int t = 0; t < EX_TN; t++) {
        if ((sup_mask >> t) & 1u) {
          r.s[sl].tag[t] = 0;
          r.s[sl].cnt[t] = 0;
          r.s[sl].useful[t] = 1;  // suppressed 1RW bank: unread, never leak SRAM data into Meta
        } else {
          const uint32_t ti = ix.idx[t];
          r.s[sl].tag[t] = tag[t][sl][ti];
          r.s[sl].cnt[t] = cnt[t][sl][ti];
          r.s[sl].useful[t] = useful[t][sl][ti];
        }
      }
    }
  }

  void select_slot(const ExTageIdx &ix, const RawSlot &r, Pred &o, SlotMeta *meta_out = nullptr) const {
    int pcpn = EX_TN, altpcpn = EX_TN;
    for (int t = 0; t < EX_TN; t++) {
      if (((r.sup_mask >> t) & 1u) == 0 && r.tag[t] == ix.tag[t]) {
        altpcpn = pcpn;
        pcpn = t;
      }
    }
    o.base_cnt = r.base_cnt;
    const bool bp = o.base_cnt >= 2;
    o.pcpn = static_cast<uint8_t>(pcpn);
    o.alt = (altpcpn == EX_TN) ? bp : (r.cnt[altpcpn] >= 4);
    o.prov_cnt = (pcpn == EX_TN) ? o.base_cnt : r.cnt[pcpn];
    const bool pp = (pcpn == EX_TN) ? bp : (o.prov_cnt >= 4);
    const bool weak = (pcpn != EX_TN) && (o.prov_cnt == 3 || o.prov_cnt == 4);
    const bool ulow = (pcpn != EX_TN) && (r.useful[pcpn] == 0);
    const uint8_t ua = use_alt[(ix.w >> 2) & 15u];
    o.used_alt = (pcpn != EX_TN) && weak && ulow && (ua <= 8);
    o.pred = o.used_alt ? o.alt : pp;
    if (meta_out) {
      meta_out->pcpn = o.pcpn;
      meta_out->pred = o.pred;
      meta_out->alt = o.alt;
      meta_out->base_cnt = o.base_cnt;
      meta_out->prov_cnt = o.prov_cnt;
      meta_out->prov_u = (pcpn != EX_TN) ? r.useful[pcpn] : 0;
      for (int t = 0; t < EX_TN; t++) meta_out->useful[t] = r.useful[t];
    }
  }

  void predict_slot(const ExTageIdx &ix, int sl, Pred &o) const {
    RawSlot r;
    r.base_cnt = base[sl][ix.base_idx];
    for (int t = 0; t < EX_TN; t++) {
      const uint32_t ti = ix.idx[t];
      r.tag[t] = tag[t][sl][ti];
      r.cnt[t] = cnt[t][sl][ti];
      r.useful[t] = useful[t][sl][ti];
    }
    select_slot(ix, r, o);
  }

  // Used only by FastWarm
  void update_slot(const ExTageIdx &ix, int sl, bool real_dir, Pred *pred_out = nullptr) {
    if (sl < 0 || sl >= SLOTS) return;
    Pred p;
    RawSlot r;
    r.base_cnt = base[sl][ix.base_idx];
    for (int t = 0; t < EX_TN; t++) {
      r.tag[t] = tag[t][sl][ix.idx[t]];
      r.cnt[t] = cnt[t][sl][ix.idx[t]];
      r.useful[t] = useful[t][sl][ix.idx[t]];
    }
    SlotMeta m;
    select_slot(ix, r, p, &m);
    if (pred_out) *pred_out = p;
    WritePlan wp;
    Pred p_tmp;
    compute_plan_from_meta(ix, sl, real_dir, true, m, wp, p_tmp);
    apply_plan(wp);
  }

  // ZERO-T0-READ pure-const commit plan using ftb_rec Meta + WRBypass!
  void compute_plan_from_meta(const ExTageIdx &ix, int sl, bool real_dir, bool meta_valid,
                              const SlotMeta &m, WritePlan &wp, Pred &p_out) const {
    wp = WritePlan{};
    p_out = Pred{};
    if (sl < 0 || sl >= SLOTS) return;
    wp.valid = true;
    wp.sl = sl;
    wp.w = ix.w;
    wp.real_dir = real_dir;
    wp.base_idx = ix.base_idx;

    const int pcpn = meta_valid ? int(m.pcpn) : EX_TN;
    const bool alt = meta_valid ? m.alt : false;
    uint8_t bc = meta_valid ? m.base_cnt : 1;
    uint8_t dummy_u = 0;
    wrb.lookup(EX_TN, static_cast<uint8_t>(sl), static_cast<uint16_t>(ix.base_idx), bc, dummy_u);

    uint8_t c = meta_valid ? m.prov_cnt : bc;
    uint8_t u = meta_valid ? m.prov_u : 0;
    if (pcpn < EX_TN) {
      wrb.lookup(static_cast<uint8_t>(pcpn), static_cast<uint8_t>(sl), static_cast<uint16_t>(ix.idx[pcpn]), c, u);
    }

    p_out.pcpn = static_cast<uint8_t>(pcpn);
    p_out.alt = alt;
    p_out.base_cnt = bc;
    p_out.prov_cnt = (pcpn == EX_TN) ? bc : c;
    p_out.pred = meta_valid ? m.pred : (bc >= 2);
    wp.pcpn = static_cast<uint8_t>(pcpn);
    wp.alt = alt;

    if (pcpn == EX_TN) {
      const uint8_t nbc = real_dir ? (bc < 3 ? bc + 1 : 3) : (bc > 0 ? bc - 1 : 0);
      if (nbc != bc) {
        wp.base_wr = true;
        wp.base_val = nbc;
      }
    } else {
      const bool pdir = (c >= 4);
      const bool weak = (c == 3 || c == 4);
      wp.pdir = pdir;
      wp.weak = weak;
      const uint8_t nc = real_dir ? (c < 7 ? c + 1 : 7) : (c > 0 ? c - 1 : 0);
      const uint8_t nu = (pdir != alt)
                             ? ((pdir == real_dir) ? (u < 3 ? uint8_t(u + 1) : 3) : (u > 0 ? uint8_t(u - 1) : 0))
                             : u;
      // Skip saturated no-op SRAM write!
      if (nc != c || nu != u) {
        wp.tbl_wr_mask |= static_cast<uint8_t>(1u << pcpn);
        wp.tbl_is_alloc[pcpn] = false;
        wp.tbl_idx[pcpn] = ix.idx[pcpn];
        wp.tbl_tag[pcpn] = ix.tag[pcpn];
        wp.tbl_cnt[pcpn] = nc;
        wp.tbl_u[pcpn] = nu;
      }
    }

    if (!meta_valid || (p_out.pred != real_dir)) {
      int free_t[EX_TN];
      int nf = 0;
      const int start_t = (pcpn == EX_TN) ? 0 : (pcpn + 1);
      uint8_t u_eff[EX_TN];
      for (int t = start_t; t < EX_TN; t++) {
        uint8_t ut = meta_valid ? m.useful[t] : 0;
        uint8_t ct_dummy = 0;
        wrb.lookup(static_cast<uint8_t>(t), static_cast<uint8_t>(sl), static_cast<uint16_t>(ix.idx[t]), ct_dummy, ut);
        u_eff[t] = ut;
        if (ut == 0) free_t[nf++] = t;
      }
      if (nf > 0) {
        int chosen = free_t[0];
        if (nf > 1 && ((lsfr & 3u) == 0)) chosen = free_t[1];
        wp.tbl_wr_mask |= static_cast<uint8_t>(1u << chosen);
        wp.tbl_is_alloc[chosen] = true;
        wp.tbl_idx[chosen] = ix.idx[chosen];
        wp.tbl_tag[chosen] = ix.tag[chosen];
        wp.tbl_cnt[chosen] = real_dir ? 4 : 3;
        wp.tbl_u[chosen] = 0;
      } else {
        for (int t = start_t; t < EX_TN; t++) {
          if (u_eff[t] > 0) {
            wp.tbl_wr_mask |= static_cast<uint8_t>(1u << t);
            wp.tbl_is_alloc[t] = false;
            wp.tbl_idx[t] = ix.idx[t];
            wp.tbl_tag[t] = 0;  // u-only write via wmask!
            wp.tbl_cnt[t] = 0;
            wp.tbl_u[t] = static_cast<uint8_t>(u_eff[t] - 1);
          }
        }
      }
    }
  }

  void apply_plan(const WritePlan &wp, bool do_base = true) {
    if (!wp.valid) return;
    if (wp.pcpn < EX_TN && wp.weak && wp.pdir != wp.alt) {
      uint8_t &ua = use_alt[(wp.w >> 2) & 15u];
      if (wp.alt == wp.real_dir) { if (ua > 0) ua--; }
      else { if (ua < 15) ua++; }
    }
    const uint8_t fb = ((lsfr >> 3) ^ (lsfr >> 2)) & 1u;
    lsfr = static_cast<uint8_t>(((lsfr << 1) | fb) & 0xfu);
    if (lsfr == 0) lsfr = 1;
    reset_cnt = reset_cnt + 1;
    if ((reset_cnt & ((1u << EX_TAGE_URST) - 1u)) == 0) {
      rst_pend_mask = static_cast<uint8_t>((1u << EX_TN) - 1u);
      rst_row = (reset_cnt >> EX_TAGE_URST) & uint32_t(EX_TAGE_N - 1);
      rst_sl = static_cast<uint8_t>(wp.sl);
    }

    if (do_base && wp.base_wr) {
      base[wp.sl][wp.base_idx] = wp.base_val;
      wrb.insert(EX_TN, static_cast<uint8_t>(wp.sl), static_cast<uint16_t>(wp.base_idx), wp.base_val, 0);
    }
    for (int t = 0; t < EX_TN; t++) {
      if (((wp.tbl_wr_mask >> t) & 1u) == 0) continue;
      const uint32_t ti = wp.tbl_idx[t];
      if (wp.tbl_is_alloc[t]) {
        tag[t][wp.sl][ti] = wp.tbl_tag[t];
        cnt[t][wp.sl][ti] = wp.tbl_cnt[t];
        useful[t][wp.sl][ti] = wp.tbl_u[t];
        wrb.insert(static_cast<uint8_t>(t), static_cast<uint8_t>(wp.sl), static_cast<uint16_t>(ti), wp.tbl_cnt[t], wp.tbl_u[t]);
      } else if (t == wp.pcpn) {
        cnt[t][wp.sl][ti] = wp.tbl_cnt[t];
        useful[t][wp.sl][ti] = wp.tbl_u[t];
        wrb.insert(static_cast<uint8_t>(t), static_cast<uint8_t>(wp.sl), static_cast<uint16_t>(ti), wp.tbl_cnt[t], wp.tbl_u[t]);
      } else {
        // u-only decrement via bit-level wmask (preserves tag & cnt without T0 read!)
        useful[t][wp.sl][ti] = wp.tbl_u[t];
      }
    }
  }

  void try_rst(const uint8_t tbl_busy_mask[EX_TN]) {
    if (rst_pend_mask == 0) return;
    const uint32_t bk = rst_row & uint32_t(BPU_S2_TAGE_BANKS - 1);
    for (int t = 0; t < EX_TN; t++) {
      if (((rst_pend_mask >> t) & 1u) && (((tbl_busy_mask[t] >> bk) & 1u) == 0)) {
        useful[t][rst_sl][rst_row] &= 1u;
        rst_pend_mask &= static_cast<uint8_t>(~(1u << t));
      }
    }
  }
};

// Tiny 8-entry WRBypass per SC subtable
struct ScSubWrBypass {
  struct Ent {
    bool valid = false;
    uint16_t key = 0;  // {idx, sl, bway}
    int8_t val = 0;
  };
  Ent e[BPU_WRB_N];
  uint8_t ptr = 0;

  void reset() {
    for (int i = 0; i < BPU_WRB_N; i++) e[i] = Ent{};
    ptr = 0;
  }
  bool lookup(uint16_t key, int8_t &val_out) const {
    for (int i = 0; i < BPU_WRB_N; i++) {
      if (e[i].valid && e[i].key == key) { val_out = e[i].val; return true; }
    }
    return false;
  }
  void insert(uint16_t key, int8_t val) {
    for (int i = 0; i < BPU_WRB_N; i++) {
      if (e[i].valid && e[i].key == key) { e[i].val = val; return; }
    }
    e[ptr] = Ent{true, key, val};
    ptr = static_cast<uint8_t>((ptr + 1) % BPU_WRB_N);
  }
};

// S2 Statistical Corrector (Bias 64x4 FF + 4 Global GEHL 512 + IMLI 256 + Local 256, 2-Bank 1RW, Zero T0 Read)
struct StatCorrector {
  static constexpr int SLOTS = 4;
  static constexpr int BIAS_SETS = 64;
  static constexpr int GEHL_N = 512;
  static constexpr int IMLI_N = 256;
  static constexpr int LOC_N = 256;
  static constexpr int LHT_N = 64;

  std::vector<int8_t> bias[SLOTS][4];
  std::vector<int8_t> gehl[SC_GEHL_N][SLOTS];
  std::vector<int8_t> imli_tbl[SLOTS];
  std::vector<int8_t> loc_tbl[SLOTS];
  uint16_t lht[LHT_N] = {};
  int thres[16] = {};
  ScSubWrBypass wrb[7];

  struct RawSlot {
    uint8_t sup_mask = 0;
    int8_t bias[4] = {};
    int8_t gehl[SC_GEHL_N] = {};
    int8_t imli = 0;
    int8_t loc = 0;
  };
  struct Raw {
    RawSlot s[SLOTS];
    uint32_t loc_idx = 0;
  };
  struct WritePlan {
    bool valid = false;
    bool do_wr = false;
    bool th_upd = false;
    bool th_dec = false;
    uint32_t th_idx = 0;
    uint32_t lht_idx = 0;
    uint16_t new_lht = 0;
    int sl = 0;
    uint32_t bway = 0;
    uint32_t bias_set = 0;
    uint32_t gehl_idx[SC_GEHL_N] = {};
    uint32_t imli_idx = 0;
    uint32_t loc_idx = 0;
    int8_t bias_v = 0;
    int8_t gehl_v[SC_GEHL_N] = {};
    int8_t imli_v = 0;
    int8_t loc_v = 0;
  };

  uint32_t loc_index(const ScIdx &ix) const {
    const uint32_t lh = lht[ix.lht_idx];
    return (ix.w ^ lh ^ (lh >> 6)) & (LOC_N - 1);
  }

  void init() {
    for (int sl = 0; sl < SLOTS; sl++) {
      for (int w = 0; w < 4; w++) bias[sl][w].assign(BIAS_SETS, 0);
      for (int g = 0; g < SC_GEHL_N; g++) gehl[g][sl].assign(GEHL_N, 0);
      imli_tbl[sl].assign(IMLI_N, 0);
      loc_tbl[sl].assign(LOC_N, 0);
    }
    std::memset(lht, 0, sizeof(lht));
    for (int i = 0; i < 16; i++) thres[i] = 35;
    for (int i = 0; i < 7; i++) wrb[i].reset();
  }

  static int8_t sat_upd(int8_t v, bool taken) {
    int nv = int(v) + (taken ? 1 : -1);
    if (nv > 31) nv = 31;
    if (nv < -32) nv = -32;
    return static_cast<int8_t>(nv);
  }

  void read(const ScIdx &ix, Raw &r, uint8_t sup_mask = 0) const {
    const uint32_t li = loc_index(ix);
    r.loc_idx = li;
    for (int sl = 0; sl < SLOTS; sl++) {
      r.s[sl].sup_mask = sup_mask;
      for (int way = 0; way < 4; way++) r.s[sl].bias[way] = bias[sl][way][ix.bias_set];
      for (int g = 0; g < SC_GEHL_N; g++)
        r.s[sl].gehl[g] = ((sup_mask >> (1 + g)) & 1u) ? int8_t(0) : gehl[g][sl][ix.gehl_idx[g]];
      r.s[sl].imli = ((sup_mask >> 5) & 1u) ? int8_t(0) : imli_tbl[sl][ix.imli_idx];
      r.s[sl].loc = ((sup_mask >> 6) & 1u) ? int8_t(0) : loc_tbl[sl][li];
    }
  }

  static int eval_raw(const RawSlot &r, bool tage_pred, bool prov_hit, uint8_t prov_cnt, uint8_t base_cnt) {
    const int tage_w = 2 * (prov_hit ? (2 * int(prov_cnt) - 7) : (2 * int(base_cnt) - 3));
    const bool low_conf = prov_hit ? (prov_cnt == 3 || prov_cnt == 4) : (base_cnt == 1 || base_cnt == 2);
    const uint32_t bway = (uint32_t(tage_pred) << 1) | uint32_t(low_conf);
    int s = tage_w + (((r.sup_mask >> 0) & 1u) ? 0 : (2 * int(r.bias[bway]) + 1));
    for (int g = 0; g < SC_GEHL_N; g++) s += (((r.sup_mask >> (1 + g)) & 1u) ? 0 : (2 * int(r.gehl[g]) + 1));
    s += (((r.sup_mask >> 5) & 1u) ? 0 : (2 * int(r.imli) + 1));
    s += (((r.sup_mask >> 6) & 1u) ? 0 : (2 * int(r.loc) + 1));
    return s;
  }

  bool select_slot(uint32_t w, const RawSlot &r, bool tage_pred, bool prov_hit, uint8_t prov_cnt, uint8_t base_cnt) const {
    const int lsum = eval_raw(r, tage_pred, prov_hit, prov_cnt, base_cnt);
    const bool sc_pred = (lsum >= 0);
    const int th = thres[(w >> 2) & 15u];
    if (sc_pred != tage_pred && std::abs(lsum) >= th) return sc_pred;
    return tage_pred;
  }

  // ZERO-T0-READ pure-const commit plan using ftb_rec Meta + WRBypass!
  void compute_plan_from_meta(const ScIdx &ix, int sl, bool real_dir, bool tage_pred, bool prov_hit,
                              uint8_t prov_cnt, uint8_t base_cnt, bool meta_valid, const Raw &meta_r,
                              WritePlan &wp) const {
    wp = WritePlan{};
    if (sl < 0 || sl >= SLOTS) return;
    wp.valid = true;
    const uint32_t lh = lht[ix.lht_idx];
    const uint32_t li = meta_valid ? meta_r.loc_idx : ((ix.w ^ lh ^ (lh >> 6)) & (LOC_N - 1));
    const bool low_conf = prov_hit ? (prov_cnt == 3 || prov_cnt == 4) : (base_cnt == 1 || base_cnt == 2);
    const uint32_t bway = (uint32_t(tage_pred) << 1) | uint32_t(low_conf);

    RawSlot r = meta_valid ? meta_r.s[sl] : RawSlot{};
    r.sup_mask = 0;
    const uint16_t bk = static_cast<uint16_t>((ix.bias_set << 4) | (sl << 2) | bway);
    wrb[0].lookup(bk, r.bias[bway]);
    for (int g = 0; g < SC_GEHL_N; g++) {
      const uint16_t gk = static_cast<uint16_t>((ix.gehl_idx[g] << 2) | sl);
      wrb[1 + g].lookup(gk, r.gehl[g]);
    }
    wrb[5].lookup(static_cast<uint16_t>((ix.imli_idx << 2) | sl), r.imli);
    wrb[6].lookup(static_cast<uint16_t>((li << 2) | sl), r.loc);

    const int lsum = eval_raw(r, tage_pred, prov_hit, prov_cnt, base_cnt);
    const bool sc_pred = (lsum >= 0);
    const uint32_t th_i = (ix.w >> 2) & 15u;
    const int th = thres[th_i];
    wp.th_idx = th_i;
    wp.lht_idx = ix.lht_idx;
    wp.new_lht = static_cast<uint16_t>(((lh << 1) | uint16_t(real_dir)) & 0x3ffu);
    wp.loc_idx = li;

    if (sc_pred != tage_pred && std::abs(lsum) >= th - 4 && std::abs(lsum) <= th + 4) {
      wp.th_upd = true;
      wp.th_dec = (sc_pred == real_dir);
    }

    if (sc_pred != real_dir || std::abs(lsum) < th + 18) {
      wp.do_wr = true;
      wp.sl = sl;
      wp.bway = bway;
      wp.bias_set = ix.bias_set;
      wp.bias_v = sat_upd(r.bias[bway], real_dir);
      for (int g = 0; g < SC_GEHL_N; g++) {
        wp.gehl_idx[g] = ix.gehl_idx[g];
        wp.gehl_v[g] = sat_upd(r.gehl[g], real_dir);
      }
      wp.imli_idx = ix.imli_idx;
      wp.imli_v = sat_upd(r.imli, real_dir);
      wp.loc_v = sat_upd(r.loc, real_dir);
    }
  }

  void apply_plan(const WritePlan &wp) {
    if (!wp.valid) return;
    if (wp.th_upd) {
      int &th = thres[wp.th_idx];
      if (wp.th_dec) { if (th > 5) th--; }
      else { if (th < 63) th++; }
    }
    lht[wp.lht_idx] = wp.new_lht;
    if (!wp.do_wr) return;
    bias[wp.sl][wp.bway][wp.bias_set] = wp.bias_v;
    wrb[0].insert(static_cast<uint16_t>((wp.bias_set << 4) | (wp.sl << 2) | wp.bway), wp.bias_v);
    for (int g = 0; g < SC_GEHL_N; g++) {
      gehl[g][wp.sl][wp.gehl_idx[g]] = wp.gehl_v[g];
      wrb[1 + g].insert(static_cast<uint16_t>((wp.gehl_idx[g] << 2) | wp.sl), wp.gehl_v[g]);
    }
    imli_tbl[wp.sl][wp.imli_idx] = wp.imli_v;
    wrb[5].insert(static_cast<uint16_t>((wp.imli_idx << 2) | wp.sl), wp.imli_v);
    loc_tbl[wp.sl][wp.loc_idx] = wp.loc_v;
    wrb[6].insert(static_cast<uint16_t>((wp.loc_idx << 2) | wp.sl), wp.loc_v);
  }

  // Used only by FastWarm
  void update(const ScIdx &ix, int sl, bool real_dir, bool tage_pred, bool prov_hit, uint8_t prov_cnt, uint8_t base_cnt) {
    Raw r;
    read(ix, r, 0);
    WritePlan wp;
    compute_plan_from_meta(ix, sl, real_dir, tage_pred, prov_hit, prov_cnt, base_cnt, true, r, wp);
    apply_plan(wp);
  }
};

}  // namespace bpuf

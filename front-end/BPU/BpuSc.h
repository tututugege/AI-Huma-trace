#pragma once
// Hardware-faithful Stage 2 6-table Long-History Block-TAGE (Clean BPU v11: Plan A).
//   - Deep Multi-Bank 1RW SRAMs (ZERO T0 read ports):
//     * u_ftb1:    2-Bank 1RW (512-deep x 4w -> 3 x 512x72 macros/way = 24 macros total)
//     * u_s2_tage: 6 tables x 8-Bank 1RW (512x64b -> 512x72 macros = 48 macros) + 4-Bank Base (1024x8b)
//     * u_s2_itt:  3 tables x 2-Bank 1RW (128x52b -> 128x64 macros)
//     * u_aftb:    2-Bank 1RW (128x8w), u_atage: 4 tables x 2-Bank 1RW (512x60b), u_aitt: 3 tables x 2-Bank 1RW
//   - Unified single folded-history engine ex_fh[3][6] (lengths {5, 11, 22, 44, 89, 178}); NO StatCorrector / IMLI / 2-Ahead B.
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace bpuf {

// Deep-macro 1RW bank geometry (single-block 1-Taken -> ZERO intra-cycle A-vs-B conflicts!)
constexpr int BPU_FTB1_BANKS    = 2;  // 2-Bank 1RW (512-deep per bank -> 512x72 macros)
constexpr int BPU_S2_BASE_BANKS = 4;  // 4-Bank 1RW (1024x8b -> 2 x 512x8 macros)
constexpr int BPU_S2_TAGE_BANKS = 8;  // 8-Bank 1RW (512-deep per bank -> 512x72 macros)
constexpr int BPU_ITT_BANKS     = 2;  // 2-Bank 1RW (128x52 macros)
constexpr int BPU_AH1_BANKS     = 2;  // 2-Bank 1RW (aftb 128-deep, atage 512-deep, aitt 128-deep)

inline uint32_t s2_bank_of_w(uint32_t w)  { return (w ^ (w >> 10)) & uint32_t(BPU_FTB1_BANKS - 1); }
inline uint32_t ah1_bank_of_w(uint32_t w) { return (w ^ (w >> 8))  & uint32_t(BPU_AH1_BANKS - 1); }

// 38-bit Sv39 VA target compression (stores pc[38:1], sign-extends bit 38 to [63:39], bit 0 = 0)
inline uint64_t sv39_va_roundtrip(uint64_t pc) {
  const int64_t s = static_cast<int64_t>((pc >> 1) << 26) >> 25;
  return static_cast<uint64_t>(s);
}

constexpr int EX_TN = 6;
constexpr int EX_FH_K = 3;
constexpr int EX_TAGE_N = 4096;
constexpr int EX_TAGE_BASE_N = 4096;
constexpr uint16_t EX_TAGE_TAG_MASK = 0x7ffu;  // 11-bit tags
constexpr int EX_TAGE_URST = 11;
constexpr int EX_GHR_LEN[EX_TN] = {5, 11, 22, 44, 89, 178};
constexpr int EX_FH_LEN[EX_FH_K] = {12, 11, 10};
// Ahead 4-table TAGE & 3-table ITT directly project from the single unified ex_fh (lengths {5, 11, 44, 178})
constexpr int AH_EX_MAP[4] = {0, 1, 3, 5};

constexpr int BPU_WRB_N = 8;  // 8-entry fully-associative WRBypass CAM+Mem (tiny FF array)

struct ExTageIdx {
  uint32_t w = 0;
  uint32_t base_idx = 0;
  uint32_t idx[EX_TN] = {};
  uint16_t tag[EX_TN] = {};
};

inline ExTageIdx make_ex_tage_index(uint32_t w, const uint16_t ex_fh[EX_FH_K][EX_TN]) {
  ExTageIdx ix;
  ix.w = w;
  ix.base_idx = (w ^ (w >> 12)) & uint32_t(EX_TAGE_BASE_N - 1);
  for (int t = 0; t < EX_TN; t++) {
    ix.idx[t] = (w ^ (w >> 11) ^ uint32_t(ex_fh[0][t])) & uint32_t(EX_TAGE_N - 1);
    ix.tag[t] = static_cast<uint16_t>((w ^ (w >> 11) ^ uint32_t(ex_fh[1][t]) ^ (uint32_t(ex_fh[2][t]) << 1)) & EX_TAGE_TAG_MASK);
  }
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

// 6-table long-history Block-TAGE for S2 (4096 entries = 8 banks x 512 x 64b 1RW packed wmask)
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
          r.s[sl].useful[t] = 1;
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
            wp.tbl_tag[t] = 0;
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
    if (wp.base_wr && do_base) {
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

}  // namespace bpuf

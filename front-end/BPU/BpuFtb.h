#pragma once
// Clean Block-based 1-Ahead A FTB Branch Prediction Unit (Clean BPU v11: Plan A).
//   - Single-block 1-Taken 3-stage pipeline: S0 (1-Ahead A / uFTB) -> S1 (1RW SRAM read) -> S2 (Main eval/override)
//   - 6 clean predictor tables (ZERO 2-Ahead B, ZERO StatCorrector, ZERO S3 split):
//     * S0: uFTB (16x4 FF) + 1-Ahead A (aftb 2048x8 2B + atage 1024x4x4 2B + aitt 256x3 2B)
//     * S1/S2: Main FTB (ftb1 4096x4 2B) + 6-table ExBlockTage (4096x4x6 8B + 4B Base) + ITT (256x3 2B)
//   - Single unified folded-history engine ex_fh[3][6] (lengths {5,11,22,44,89,178}); Zero-T0-read Meta + WRBypass + PLRU
#include "BPU.h"
#include "BpuSc.h"
#include <algorithm>
#include <cstring>
#include <vector>

namespace bpuf {

// Fixed geometry
constexpr int FTB_SLOTS     = 4;
constexpr int FTB_MAXW      = 8;
constexpr int FTB_S0_N      = 64;    // 16 sets x 4 ways FF (cold-start fallback)
constexpr int FTB_S0_W      = 4;
constexpr int FTB_S0_TAGB   = 12;
constexpr int FTB_S0_TGTB   = 20;
constexpr int FTB_S1_N      = 4096;  // 1024 sets x 4 ways (2-Bank 1RW: 512x4w, 215b/way -> 3 x 512x72 macros!)
constexpr int FTB_S1_W      = 4;
constexpr int FTB_S1_TAGB   = 14;
constexpr int FTB_TAGE_N    = 4096;
constexpr int FTB_ITT_N     = 256;
constexpr int FTB_AH_N      = 2048;  // 256 sets x 8 ways (2-Bank 1RW: 128x8w)
constexpr int FTB_AH_W      = 8;
constexpr int FTB_AH_TAGE_N = 1024;  // 4 tables x 1024 x 4 slots (2-Bank 1RW: 512x60b)
constexpr int FTB_BRQ_N     = 16;
constexpr int FTB_REC_N     = 512;
constexpr uint16_t FTB_TAGE_TAG_MASK = 0x3ffu;

struct FtbSlot {
  bool valid = false;
  uint8_t offset = 0;       // 4b: halfword slot offset (0..15) from block base
  bool carried = false;     // 1b: slot 0 is a 32-bit instruction carried from base - 2
  bool is_rvc = false;      // 1b: 16-bit instruction (ilen = 2 else 4)
  uint8_t type = BR_NONCTL; // 3b: branch type
  uint8_t ctr = 2;          // 2b: saturating taken counter (0..3, taken >= 2)
  pc_t target = 0;          // 38-bit Sv39 sign-extended target address
  bool operator==(const FtbSlot &o) const {
    return valid == o.valid && offset == o.offset && carried == o.carried &&
           is_rvc == o.is_rvc && type == o.type && ctr == o.ctr && target == o.target;
  }
};

struct FtbEnt {
  bool valid = false;
  uint16_t tag = 0;         // 14b tag (normal FTB / uFTB)
  uint32_t tag32 = 0;       // 22b combined (key_base, cur_base) tag (1-Ahead FTB)
  FtbSlot s[FTB_SLOTS];
  bool payload_eq(const FtbEnt &o) const {
    if (valid != o.valid || tag != o.tag || tag32 != o.tag32) return false;
    for (int k = 0; k < FTB_SLOTS; k++) if (!(s[k] == o.s[k])) return false;
    return true;
  }
};

// Block-level index/tag bundle for 4-table 1-Ahead TAGE & 3/4-table ITT (derived from unified ex_fh)
constexpr int ITT_MAX_EXT = 4;
constexpr int ITT_EX_MAP4[ITT_MAX_EXT] = {1, 2, 3, 5};
struct FtbBlkIdx {
  uint32_t w = 0;
  uint32_t base_idx = 0;
  uint32_t idx[TN] = {};
  uint16_t tag[TN] = {};
  uint32_t itt_idx[ITT_MAX_EXT] = {};
  uint16_t itt_tag[ITT_MAX_EXT] = {};
};

// Tiny 8-entry FF WRBypass for FTB entries (allows single-way write without T0 SRAM read!)
struct FtbWrBypass {
  struct Ent {
    bool valid = false;
    uint32_t set = 0;
    uint32_t tag = 0;
    uint8_t way = 0;
    FtbEnt ent{};
  };
  Ent e[BPU_WRB_N];
  uint8_t ptr = 0;

  void reset() {
    for (int i = 0; i < BPU_WRB_N; i++) e[i] = Ent{};
    ptr = 0;
  }
  bool lookup(uint32_t set, uint32_t tag, uint8_t &way_out, FtbEnt &ent_out) const {
    if (cfg_fe_no_wrb()) return false;
    for (int i = 0; i < BPU_WRB_N; i++) {
      if (e[i].valid && e[i].set == set && e[i].tag == tag) {
        way_out = e[i].way;
        ent_out = e[i].ent;
        return true;
      }
    }
    return false;
  }
  void insert(uint32_t set, uint32_t tag, uint8_t way, const FtbEnt &ent) {
    if (cfg_fe_no_wrb()) return;
    for (int i = 0; i < BPU_WRB_N; i++) {
      if (e[i].valid && e[i].set == set && e[i].tag == tag) {
        e[i].way = way;
        e[i].ent = ent;
        return;
      }
    }
    e[ptr] = Ent{true, set, tag, way, ent};
    ptr = static_cast<uint8_t>((ptr + 1) % BPU_WRB_N);
  }
};

// Tree-PLRU helper for 4-way (3b/set) and 8-way (7b/set) FTB replacement (zero multi-way RMW!)
struct PlruArray {
  int W = 4;
  std::vector<uint8_t> bits;

  void init(int sets, int ways) {
    W = ways;
    bits.assign(sets, 0);
  }
  uint8_t victim(uint32_t s) const {
    const uint8_t b = bits[s];
    if (W == 4) {
      if ((b & 1u) == 0) return (b & 2u) ? 1 : 0;
      else return (b & 4u) ? 3 : 2;
    } else {
      if ((b & 1u) == 0) {
        if ((b & 2u) == 0) return (b & 8u) ? 1 : 0;
        else return (b & 16u) ? 3 : 2;
      } else {
        if ((b & 4u) == 0) return (b & 32u) ? 5 : 4;
        else return (b & 64u) ? 7 : 6;
      }
    }
  }
  void touch(uint32_t s, uint8_t w) {
    uint8_t &b = bits[s];
    if (W == 4) {
      if (w < 2) {
        b |= 1u;
        if (w == 0) b |= 2u; else b &= ~2u;
      } else {
        b &= ~1u;
        if (w == 2) b |= 4u; else b &= ~4u;
      }
    } else {
      if (w < 4) {
        b |= 1u;
        if (w < 2) {
          b |= 2u;
          if (w == 0) b |= 8u; else b &= ~8u;
        } else {
          b &= ~2u;
          if (w == 2) b |= 16u; else b &= ~16u;
        }
      } else {
        b &= ~1u;
        if (w < 6) {
          b |= 4u;
          if (w == 4) b |= 32u; else b &= ~32u;
        } else {
          b &= ~4u;
          if (w == 6) b |= 64u; else b &= ~64u;
        }
      }
    }
  }
};

// Set-associative FTB table (used for S0 uFTB FF and S1/S2 Main FTB 2-Bank 1RW)
struct FtbTab {
  int N = 0, W = 1, S = 1, set_bits = 0, tag_bits = 14, tgt_bits = 0;
  std::vector<FtbEnt> e;
  PlruArray plru;
  FtbWrBypass wrb;

  struct LookupMeta {
    bool hit = false;
    uint8_t way = 0;
    FtbEnt ent{};
  };

  void init(int n, int w, int tb, int tgtb) {
    N = n; W = w; S = (n > 0 && w > 0) ? (n / w) : 1;
    set_bits = ceil_log2_u32(static_cast<uint32_t>(S));
    tag_bits = tb;
    tgt_bits = tgtb;
    e.assign(static_cast<size_t>(S) * W, FtbEnt{});
    plru.init(S, W);
    wrb.reset();
  }

  uint32_t index(pc_t base) const {
    const uint32_t w = static_cast<uint32_t>(base >> SLOT_SHIFT);
    return (w ^ (w >> set_bits)) & static_cast<uint32_t>(S - 1);
  }
  uint16_t tag(pc_t base) const {
    const uint32_t w = static_cast<uint32_t>(base >> SLOT_SHIFT);
    return static_cast<uint16_t>((w >> set_bits) & ((1u << tag_bits) - 1u));
  }
  bool tgt_fits(pc_t base, pc_t target) const {
    if (tgt_bits <= 0) return true;
    const pc_t mask = ~((pc_t(1) << (tgt_bits + SLOT_SHIFT)) - 1u);
    return (base & mask) == (target & mask);
  }

  bool lookup(pc_t base, FtbEnt &out, LookupMeta *meta_out = nullptr) const {
    const uint32_t s = index(base);
    const uint16_t t = tag(base);
    int inv_w = -1;
    for (int w = 0; w < W; w++) {
      const FtbEnt &en = e[s * W + w];
      if (!en.valid && inv_w < 0) inv_w = w;
      if (en.valid && en.tag == t) {
        out = en;
        if (meta_out) { meta_out->hit = true; meta_out->way = static_cast<uint8_t>(w); meta_out->ent = en; }
        return true;
      }
    }
    if (meta_out) {
      meta_out->hit = false;
      meta_out->way = (inv_w >= 0) ? static_cast<uint8_t>(inv_w) : plru.victim(s);
      meta_out->ent = FtbEnt{};
    }
    return false;
  }

  void touch_lru(uint32_t s, int hit_w) {
    plru.touch(s, static_cast<uint8_t>(hit_w));
  }

  void clear_slot(pc_t base, pc_t br_pc) {
    const bool carried = (br_pc + SLOT_BYTES == base);
    if (!carried && br_pc < base) return;
    const uint32_t off = carried ? 0u : static_cast<uint32_t>((br_pc - base) >> SLOT_SHIFT);
    const uint32_t s = index(base);
    const uint16_t t = tag(base);
    for (int w = 0; w < W; w++) {
      FtbEnt &en = e[s * W + w];
      if (!(en.valid && en.tag == t)) continue;
      bool any_v = false;
      for (int k = 0; k < FTB_SLOTS; k++) {
        if (en.s[k].valid && en.s[k].offset == off) en.s[k] = FtbSlot{};
        if (en.s[k].valid) any_v = true;
      }
      if (!any_v) en.valid = false;
      wrb.insert(s, t, static_cast<uint8_t>(w), en);
      return;
    }
  }
};

// Block-indexed 4-slot TAGE for 1-Ahead A (2-Bank 1RW, Zero T0 Read)
struct BlockTage {
  int N = FTB_AH_TAGE_N;
  int nbanks = BPU_AH1_BANKS;
  uint32_t mask = FTB_AH_TAGE_N - 1;
  std::vector<uint16_t> tag[TN][FTB_SLOTS];
  std::vector<uint8_t> cnt[TN][FTB_SLOTS];
  std::vector<uint8_t> useful[TN][FTB_SLOTS];
  uint8_t lsfr = 8;
  uint32_t reset_cnt = 0;
  uint8_t use_alt = static_cast<uint8_t>(TAGE_USE_ALT_CTR_INIT);
  uint8_t rst_pend_mask = 0;
  uint32_t rst_row = 0;
  uint8_t rst_sl = 0;
  TageWrBypass wrb;

  struct RawSlot {
    uint8_t sup_mask = 0;
    uint16_t tag[TN] = {};
    uint8_t cnt[TN] = {};
    uint8_t useful[TN] = {};
  };
  struct Raw {
    RawSlot s[FTB_SLOTS];
  };
  struct Pred {
    bool pred = false;
    bool alt = false;
    uint8_t pcpn = TN;
    bool used_alt = false;
  };
  struct SlotMeta {
    uint8_t pcpn = TN;
    bool pred = false;
    bool alt = false;
    uint8_t prov_cnt = 4;
    uint8_t prov_u = 0;
    uint8_t useful[TN] = {};
  };
  struct Meta {
    SlotMeta s[FTB_SLOTS];
  };
  struct WritePlan {
    bool valid = false;
    int sl = 0;
    bool real_dir = false;
    uint8_t pcpn = TN;
    bool pdir = false;
    bool weak = false;
    bool alt = false;
    uint8_t tbl_wr_mask = 0;
    bool tbl_is_alloc[TN] = {};
    uint32_t tbl_idx[TN] = {};
    uint16_t tbl_tag[TN] = {};
    uint8_t tbl_cnt[TN] = {};
    uint8_t tbl_u[TN] = {};
  };

  void init(int entries, int banks = BPU_AH1_BANKS) {
    N = entries;
    nbanks = banks;
    mask = static_cast<uint32_t>(N - 1);
    for (int sl = 0; sl < FTB_SLOTS; sl++) {
      for (int t = 0; t < TN; t++) {
        tag[t][sl].assign(N, 0);
        cnt[t][sl].assign(N, 0);
        useful[t][sl].assign(N, 0);
      }
    }
    lsfr = 8;
    reset_cnt = 0;
    use_alt = static_cast<uint8_t>(TAGE_USE_ALT_CTR_INIT);
    rst_pend_mask = 0;
    rst_row = 0;
    rst_sl = 0;
    wrb.reset();
  }

  void read(const FtbBlkIdx &ix, Raw &r, uint8_t sup_mask = 0) const {
    for (int sl = 0; sl < FTB_SLOTS; sl++) {
      r.s[sl].sup_mask = sup_mask;
      for (int t = 0; t < TN; t++) {
        if ((sup_mask >> t) & 1u) {
          r.s[sl].tag[t] = 0;
          r.s[sl].cnt[t] = 0;
          r.s[sl].useful[t] = 1;
        } else {
          const uint32_t ti = ix.idx[t] & mask;
          r.s[sl].tag[t] = tag[t][sl][ti];
          r.s[sl].cnt[t] = cnt[t][sl][ti];
          r.s[sl].useful[t] = useful[t][sl][ti];
        }
      }
    }
  }

  void select_slot(const FtbBlkIdx &ix, const RawSlot &r, Pred &o, int base_override,
                   SlotMeta *meta_out = nullptr) const {
    int pcpn = TN, altpcpn = TN;
    for (int t = 0; t < TN; t++) {
      if (((r.sup_mask >> t) & 1u) == 0 && r.tag[t] == ix.tag[t]) {
        altpcpn = pcpn;
        pcpn = t;
      }
    }
    const bool bp = (base_override >= 2);
    o.pcpn = static_cast<uint8_t>(pcpn);
    o.alt = (altpcpn == TN) ? bp : (r.cnt[altpcpn] >= 4);
    const bool pp = (pcpn == TN) ? bp : (r.cnt[pcpn] >= 4);
    const bool weak = (pcpn != TN) && (r.cnt[pcpn] == 3);
    const bool ulow = (pcpn != TN) && (r.useful[pcpn] == 0);
    o.used_alt = (pcpn != TN) && weak && ulow && (use_alt <= 8);
    o.pred = o.used_alt ? o.alt : pp;
    if (meta_out) {
      meta_out->pcpn = o.pcpn;
      meta_out->pred = o.pred;
      meta_out->alt = o.alt;
      meta_out->prov_cnt = (pcpn != TN) ? r.cnt[pcpn] : 0;
      meta_out->prov_u = (pcpn != TN) ? r.useful[pcpn] : 0;
      for (int t = 0; t < TN; t++) meta_out->useful[t] = r.useful[t];
    }
  }

  void compute_plan_from_meta(const FtbBlkIdx &ix, int sl, bool real_dir, int base_override,
                              bool meta_valid, const SlotMeta &m, WritePlan &wp) const {
    wp = WritePlan{};
    if (sl < 0 || sl >= FTB_SLOTS) return;
    wp.valid = true;
    wp.sl = sl;
    wp.real_dir = real_dir;

    const int pcpn = meta_valid ? int(m.pcpn) : TN;
    const bool bp = (base_override >= 2);
    const bool alt = meta_valid ? m.alt : bp;
    const bool pred = meta_valid ? m.pred : bp;
    wp.pcpn = static_cast<uint8_t>(pcpn);
    wp.alt = alt;

    if (pcpn >= 0 && pcpn < TN) {
      const uint32_t ti = ix.idx[pcpn] & mask;
      uint8_t c = m.prov_cnt;
      uint8_t u = m.prov_u;
      wrb.lookup(static_cast<uint8_t>(pcpn), static_cast<uint8_t>(sl), static_cast<uint16_t>(ti), c, u);
      const bool pdir = (c >= 4);
      wp.pdir = pdir;
      wp.weak = (c == 3);
      const uint8_t nc = real_dir ? (c < 7 ? c + 1 : 7) : (c > 0 ? c - 1 : 0);
      const uint8_t nu = (pdir != alt)
                             ? ((pdir == real_dir) ? (u < 3 ? uint8_t(u + 1) : 3) : (u > 0 ? uint8_t(u - 1) : 0))
                             : u;
      if (nc != c || nu != u) {
        wp.tbl_wr_mask |= static_cast<uint8_t>(1u << pcpn);
        wp.tbl_is_alloc[pcpn] = false;
        wp.tbl_idx[pcpn] = ti;
        wp.tbl_tag[pcpn] = ix.tag[pcpn];
        wp.tbl_cnt[pcpn] = nc;
        wp.tbl_u[pcpn] = nu;
      }
    }

    if (!meta_valid || (pred != real_dir)) {
      int free_t[TN];
      int nf = 0;
      const int start_t = (pcpn == TN) ? 0 : (pcpn + 1);
      uint8_t u_eff[TN];
      for (int t = start_t; t < TN; t++) {
        const uint32_t ti = ix.idx[t] & mask;
        uint8_t ut = meta_valid ? m.useful[t] : 0;
        uint8_t ct_dummy = 0;
        wrb.lookup(static_cast<uint8_t>(t), static_cast<uint8_t>(sl), static_cast<uint16_t>(ti), ct_dummy, ut);
        u_eff[t] = ut;
        if (ut == 0) free_t[nf++] = t;
      }
      if (nf > 0) {
        int chosen = free_t[0];
        if (nf > 1 && ((lsfr & 3u) == 0)) chosen = free_t[1];
        const uint32_t ti = ix.idx[chosen] & mask;
        wp.tbl_wr_mask |= static_cast<uint8_t>(1u << chosen);
        wp.tbl_is_alloc[chosen] = true;
        wp.tbl_idx[chosen] = ti;
        wp.tbl_tag[chosen] = ix.tag[chosen];
        wp.tbl_cnt[chosen] = real_dir ? 4 : 3;
        wp.tbl_u[chosen] = 0;
      } else {
        for (int t = start_t; t < TN; t++) {
          if (u_eff[t] > 0) {
            const uint32_t ti = ix.idx[t] & mask;
            wp.tbl_wr_mask |= static_cast<uint8_t>(1u << t);
            wp.tbl_is_alloc[t] = false;
            wp.tbl_idx[t] = ti;
            wp.tbl_tag[t] = 0;
            wp.tbl_cnt[t] = 0;
            wp.tbl_u[t] = static_cast<uint8_t>(u_eff[t] - 1);
          }
        }
      }
    }
  }

  void apply_plan(const WritePlan &wp) {
    if (!wp.valid) return;
    if (wp.pcpn < TN && wp.weak && wp.pdir != wp.alt) {
      constexpr uint8_t uamax = static_cast<uint8_t>((1u << TAGE_USE_ALT_CTR_BITS) - 1u);
      if (wp.alt == wp.real_dir) { if (use_alt > 0) use_alt--; }
      else { if (use_alt < uamax) use_alt++; }
    }
    const uint8_t fb = ((lsfr >> 3) ^ (lsfr >> 2)) & 1u;
    lsfr = static_cast<uint8_t>(((lsfr << 1) | fb) & 0xfu);
    if (lsfr == 0) lsfr = 1;
    reset_cnt = reset_cnt + 1;
    if ((reset_cnt & ((1u << TageCfg::urst) - 1u)) == 0) {
      rst_pend_mask = static_cast<uint8_t>((1u << TN) - 1u);
      rst_row = (reset_cnt >> TageCfg::urst) & mask;
      rst_sl = static_cast<uint8_t>(wp.sl);
    }
    for (int t = 0; t < TN; t++) {
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

  void try_rst(const uint8_t tbl_busy_mask[TN]) {
    if (rst_pend_mask == 0) return;
    const uint32_t bk = rst_row & static_cast<uint32_t>(nbanks - 1);
    for (int t = 0; t < TN; t++) {
      if (((rst_pend_mask >> t) & 1u) && (((tbl_busy_mask[t] >> bk) & 1u) == 0)) {
        useful[t][rst_sl][rst_row] &= 1u;
        rst_pend_mask &= static_cast<uint8_t>(~(1u << t));
      }
    }
  }

  // Used only by FastWarm
  void update_slot(const FtbBlkIdx &ix, int sl, bool real_dir, int base_override) {
    Raw r;
    read(ix, r, 0);
    Pred p;
    SlotMeta m;
    select_slot(ix, r.s[sl], p, base_override, &m);
    WritePlan wp;
    compute_plan_from_meta(ix, sl, real_dir, base_override, true, m, wp);
    apply_plan(wp);
  }
};

// Single Block-level 3/4-table Indirect Target Table (2-Bank 1RW, Zero T0 Read via Meta)
struct BlockItt {
  struct Ent {
    bool valid = false;
    uint16_t tag = 0;
    uint8_t conf = 0;
    uint8_t u = 0;
    target_addr_t target = 0;
  };
  struct Raw {
    Ent e[ITT_MAX_EXT];
  };
  struct Meta {
    bool hit = false;
    uint8_t hit_k = 0;
    uint8_t conf = 0;
    uint8_t valid_mask = 0;
    uint8_t u_mask = 0;
    target_addr_t target = 0;
  };
  struct WritePlan {
    uint8_t wr_mask = 0;
    bool u_clr_only[ITT_MAX_EXT] = {};
    uint32_t idx[ITT_MAX_EXT] = {};
    Ent ent[ITT_MAX_EXT];
  };

  int N = FTB_ITT_N;
  int nt = ITT_MAX;
  uint32_t mask = FTB_ITT_N - 1;
  std::vector<Ent> tbl[ITT_MAX_EXT];

  void init(int entries, int ntables = ITT_MAX) {
    N = entries;
    nt = ntables;
    mask = static_cast<uint32_t>(N - 1);
    for (int k = 0; k < ITT_MAX_EXT; k++) tbl[k].assign(N, Ent{});
  }

  void read(const FtbBlkIdx &ix, Raw &r, uint8_t sup_mask = 0) const {
    for (int k = 0; k < nt; k++) {
      if ((sup_mask >> k) & 1u) {
        r.e[k] = Ent{};
      } else {
        r.e[k] = tbl[k][ix.itt_idx[k] & mask];
      }
    }
  }

  bool select(const FtbBlkIdx &ix, const Raw &r, target_addr_t &tgt, Meta *meta_out = nullptr) const {
    if (meta_out) {
      *meta_out = Meta{};
      for (int k = 0; k < nt; k++) {
        if (r.e[k].valid) meta_out->valid_mask |= static_cast<uint8_t>(1u << k);
        if (r.e[k].u) meta_out->u_mask |= static_cast<uint8_t>(1u << k);
      }
    }
    for (int k = nt - 1; k >= 0; k--) {
      if (r.e[k].valid && r.e[k].tag == ix.itt_tag[k]) {
        tgt = r.e[k].target;
        if (meta_out) {
          meta_out->hit = true;
          meta_out->hit_k = static_cast<uint8_t>(k);
          meta_out->conf = r.e[k].conf;
          meta_out->target = r.e[k].target;
        }
        return true;
      }
    }
    return false;
  }

  void compute_plan_from_meta(const FtbBlkIdx &ix, target_addr_t actual_tgt, bool meta_valid,
                              const Meta &m, WritePlan &wp) const {
    wp = WritePlan{};
    actual_tgt = sv39_va_roundtrip(actual_tgt);
    const int hit = (meta_valid && m.hit) ? int(m.hit_k) : -1;
    bool correct = false;
    if (hit >= 0 && hit < nt) {
      const uint32_t id = ix.itt_idx[hit] & mask;
      Ent en{};
      en.valid = true;
      en.tag = ix.itt_tag[hit];
      en.conf = m.conf;
      en.u = (m.u_mask >> hit) & 1u;
      en.target = m.target;
      correct = (en.target == actual_tgt);
      const uint8_t old_conf = en.conf;
      const uint8_t old_u = en.u;
      if (correct) {
        if (en.conf < 3) en.conf++;
        en.u = 1;
      } else {
        if (en.conf > 0) en.conf--;
        if (en.conf == 0) { en.target = actual_tgt; en.conf = 1; }
        en.u = 0;
      }
      if (!correct || en.conf != old_conf || en.u != old_u) {
        wp.wr_mask |= static_cast<uint8_t>(1u << hit);
        wp.idx[hit] = id;
        wp.ent[hit] = en;
      }
    }
    if (!correct) {
      const int start_k = (hit >= 0) ? (hit + 1) : 0;
      int alloc = -1;
      for (int k = start_k; k < nt; k++) {
        const bool v = meta_valid ? (((m.valid_mask >> k) & 1u) != 0) : false;
        const bool u = meta_valid ? (((m.u_mask >> k) & 1u) != 0) : false;
        if (!v || !u) { alloc = k; break; }
      }
      if (alloc >= 0) {
        const uint32_t id = ix.itt_idx[alloc] & mask;
        Ent en{};
        en.valid = true;
        en.tag = ix.itt_tag[alloc];
        en.target = actual_tgt;
        en.conf = 1;
        en.u = 0;
        wp.wr_mask |= static_cast<uint8_t>(1u << alloc);
        wp.idx[alloc] = id;
        wp.ent[alloc] = en;
      } else {
        for (int k = start_k; k < nt; k++) {
          wp.wr_mask |= static_cast<uint8_t>(1u << k);
          wp.u_clr_only[k] = true;
          wp.idx[k] = ix.itt_idx[k] & mask;
        }
      }
    }
  }

  void apply_plan(const WritePlan &wp) {
    for (int k = 0; k < nt; k++) {
      if (((wp.wr_mask >> k) & 1u) == 0) continue;
      if (wp.u_clr_only[k]) tbl[k][wp.idx[k]].u = 0;
      else tbl[k][wp.idx[k]] = wp.ent[k];
    }
  }

  void update(const FtbBlkIdx &ix, target_addr_t actual_tgt) {
    Raw r;
    read(ix, r, 0);
    Meta m;
    target_addr_t dummy = 0;
    select(ix, r, dummy, &m);
    WritePlan wp;
    compute_plan_from_meta(ix, actual_tgt, true, m, wp);
    apply_plan(wp);
  }
};

// Set-associative 1-Ahead FTB table (2-Bank 1RW, Zero T0 Read via Meta + WRBypass + PLRU)
struct AheadFtbTab {
  int N = FTB_AH_N, W = FTB_AH_W, S = FTB_AH_N / FTB_AH_W, set_bits = 8;
  std::vector<FtbEnt> e;
  PlruArray plru;
  FtbWrBypass wrb;

  struct SetRaw {
    FtbEnt ways[FTB_AH_W];
  };
  struct LookupMeta {
    bool hit = false;
    uint8_t way = 0;
    FtbEnt ent{};
  };

  void init(int n, int w) {
    W = w; N = n; S = n / w;
    set_bits = ceil_log2_u32(static_cast<uint32_t>(S));
    e.assign(static_cast<size_t>(S) * W, FtbEnt{});
    plru.init(S, W);
    wrb.reset();
  }

  uint32_t index(pc_t key_base) const {
    const uint32_t wk = static_cast<uint32_t>(key_base >> SLOT_SHIFT);
    return (wk ^ (wk >> set_bits)) & static_cast<uint32_t>(S - 1);
  }
  uint32_t tag(pc_t key_base, pc_t cur_base) const {
    const uint32_t wk = static_cast<uint32_t>(key_base >> SLOT_SHIFT);
    const uint32_t wc = static_cast<uint32_t>(cur_base >> SLOT_SHIFT);
    const uint32_t tk = (wk >> set_bits) & 0x3ffu;
    const uint32_t tc = (wc ^ (wc >> 12)) & 0xfffu;
    return (tk << 12) | tc;
  }

  void read_set(pc_t key_base, SetRaw &raw) const {
    const uint32_t s = index(key_base);
    for (int w = 0; w < W; w++) raw.ways[w] = e[s * W + w];
  }

  bool select_way(const SetRaw &raw, pc_t key_base, pc_t cur_base, FtbEnt &out,
                  LookupMeta *meta_out = nullptr) const {
    const uint32_t s = index(key_base);
    const uint32_t t = tag(key_base, cur_base);
    int inv_w = -1;
    for (int w = 0; w < W; w++) {
      if (!raw.ways[w].valid && inv_w < 0) inv_w = w;
      if (raw.ways[w].valid && raw.ways[w].tag32 == t) {
        out = raw.ways[w];
        if (meta_out) { meta_out->hit = true; meta_out->way = static_cast<uint8_t>(w); meta_out->ent = out; }
        return true;
      }
    }
    if (meta_out) {
      meta_out->hit = false;
      meta_out->way = (inv_w >= 0) ? static_cast<uint8_t>(inv_w) : plru.victim(s);
      meta_out->ent = FtbEnt{};
    }
    return false;
  }

  void touch_lru(uint32_t s, int hit_w) {
    plru.touch(s, static_cast<uint8_t>(hit_w));
  }

  void clear_slot(pc_t key_base, pc_t cur_base, pc_t br_pc) {
    const bool carried = (br_pc + SLOT_BYTES == cur_base);
    if (!carried && br_pc < cur_base) return;
    const uint32_t off = carried ? 0u : static_cast<uint32_t>((br_pc - cur_base) >> SLOT_SHIFT);
    const uint32_t s = index(key_base);
    const uint32_t t = tag(key_base, cur_base);
    for (int w = 0; w < W; w++) {
      FtbEnt &en = e[s * W + w];
      if (!(en.valid && en.tag32 == t)) continue;
      bool any_v = false;
      for (int k = 0; k < FTB_SLOTS; k++) {
        if (en.s[k].valid && en.s[k].offset == off) en.s[k] = FtbSlot{};
        if (en.s[k].valid) any_v = true;
      }
      if (!any_v) en.valid = false;
      wrb.insert(s, t, static_cast<uint8_t>(w), en);
      return;
    }
  }
};

struct FtbAhRaw {
  AheadFtbTab::SetRaw ftb_set{};
  BlockTage::Raw tage_raw{};
  ExBlockTage::Raw ex_tage_raw{};
  BlockItt::Raw itt_raw{};
};

struct FtbAhPipe {
  bool valid = false;
  pc_t key_base = 0;
  uint16_t key_ex_fh[EX_FH_K][EX_TN] = {};
  FtbAhRaw raw{};
};

struct FtbRaw {
  bool ftb_hit = false;
  FtbTab::LookupMeta ftb_meta{};
  FtbEnt ent{};
  ExBlockTage::Raw s2_tage_raw{};
  BlockItt::Raw itt_raw{};
  bool self0_v = false;
  FtbAhPipe self0_pipe{};
};

struct FtbTrkBr {
  uint8_t offset = 0;
  bool carried = false;
  bool is_rvc = false;
  uint8_t type = BR_NONCTL;
  bool actual_dir = false;
  bool pred_dir = false;
  bool alt_pred = false;
  uint8_t pcpn = TN;
  pc_t target = 0;
};

struct FtbTrainEv {
  pc_t start = 0;
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  bool ah_v = false;
  pc_t ah_key_base = 0;
  uint16_t ah_key_ex_fh[EX_FH_K][EX_TN] = {};
  uint8_t nbr = 0;
  FtbTrkBr br[FW];
};

// Compact prediction-time block metadata stored in single-bank 512-entry 1R1W ftb_rec_ (NO SC, NO 2-Ahead B!)
struct FtbBlkRec {
  bool valid = false;
  uint16_t id = 0;
  pc_t base = 0;
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  bool ah_v = false;
  bool ah_self0 = false;
  pc_t ah_key_base = 0;
  pc_t prev_base = 0;
  uint16_t ah_key_ex_fh[EX_FH_K][EX_TN] = {};
  FtbTab::LookupMeta ftb1_meta{};
  ExBlockTage::Meta s2_tage_meta{};
  BlockItt::Meta s2_itt_meta{};
  AheadFtbTab::LookupMeta ah_ftb_meta{};
  BlockTage::Meta ah_tage_meta{};
  ExBlockTage::Meta ah_ex_tage_meta{};
  BlockItt::Meta ah_itt_meta{};
};

struct FtbBrqEnt {
  uint16_t blk_id = 0;
  pc_t pc = 0;
  bool is_rvc = false;
  uint8_t type = BR_NONCTL;
  bool actual_dir = false;
  bool pred_dir = false;
  bool alt_pred = false;
  uint8_t pcpn = TN;
  pc_t target = 0;
};

struct FtbTrkState {
  bool v = false;
  pc_t start = 0;
  pc_t expect = 0;
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  uint8_t nbr = 0;
  FtbTrkBr br[FW];
  FtbBrqEnt brq[FTB_BRQ_N];
  uint8_t brq_h = 0, brq_n = 0;
};

struct FtbWrBuf {
  bool valid = false;
  uint32_t set = 0;
  uint8_t way = 0;
  FtbEnt ent{};
};

struct BpuFtb {
  FtbTab ftb0_;           // S0 uFTB (configurable FF, 20b compressed target)
  FtbTab ftb1_;           // S1/S2 Main FTB (4096 x 4, 2-Bank 1RW, 512-deep)
  ExBlockTage s2_tage_;   // S2 6-table Block-TAGE (4096 x 4 slots x 6 tables + 4-Bank Base, 8-Bank 1RW)
  BlockItt itt_;          // S2 Single Block-ITT (256x3 or 512x4)
  AheadFtbTab aftb_;      // Ahead FTB A (configurable 2048/4096 x 4/8, 2-Bank 1RW)
  BlockTage atage_;       // Ahead Block-TAGE A (4-table)
  ExBlockTage ah_ex_tage_;// Ahead 6-table ExBlockTage (used when FE_AH_EX_TAGE=1, base from AFTB ctr)
  BlockItt aitt_;         // Ahead Block-ITT A (256x3 or 512x4)

  struct AhMeta {
    AheadFtbTab::LookupMeta ftb{};
    BlockTage::Meta tage{};
    ExBlockTage::Meta ex_tage{};
    BlockItt::Meta itt{};
  };
  struct S2Meta {
    FtbTab::LookupMeta ftb{};
    ExBlockTage::Meta tage{};
    BlockItt::Meta itt{};
  };

  void reset() {
    ftb0_.init(cfg_fe_uftb_n(), FTB_S0_W, FTB_S0_TAGB, FTB_S0_TGTB);
    ftb1_.init(FTB_S1_N, FTB_S1_W, FTB_S1_TAGB, 0);
    if (cfg_fe_pred_arch() == 2 && cfg_fe_s2_sup_tage() > 0) {
      const int sup_m = cfg_fe_s2_sup_tage();
      const int sup_n = (sup_m == 2) ? 2048 : 1024;
      const int sup_b = (sup_n >= 2048) ? 4 : 2;
      s2_tage_.init(sup_n, sup_b, cfg_fe_sup_tbl_mask());
    } else {
      s2_tage_.init();
    }
    const int itt_n = cfg_fe_itt_cfg() ? 512 : FTB_ITT_N;
    const int itt_t = cfg_fe_itt_cfg() ? ITT_MAX_EXT : ITT_MAX;
    itt_.init(itt_n, itt_t);
    aftb_.init(cfg_fe_aftb_n(), cfg_fe_aftb_w());
    const int at_n = cfg_fe_atage_n();
    const int at_b = (at_n >= 4096) ? 8 : (at_n >= 2048 ? 4 : 2);
    atage_.init(at_n, at_b);
    const int ah_w = (cfg_fe_pred_arch() == 2) ? cfg_fe_ah_ways() : 1;
    const int ah_s = at_n / ah_w;
    const int ah_b = (ah_s >= 4096) ? 8 : (ah_s >= 2048 ? 4 : 2);
    ah_ex_tage_.init(at_n, ah_b, 0x3fu, ah_w);
    aitt_.init(itt_n, itt_t);
  }

  // Unified index builder for S2 ITT (projects directly from unified ex_fh[3][6])
  static FtbBlkIdx make_index(pc_t base, const uint16_t ex_fh[EX_FH_K][EX_TN]) {
    FtbBlkIdx ix;
    const uint32_t w = static_cast<uint32_t>(base >> SLOT_SHIFT);
    ix.w = w;
    ix.base_idx = w ^ (w >> 11);
    for (int t = 0; t < TN; t++) {
      const int e = AH_EX_MAP[t];
      ix.idx[t] = w ^ (w >> 10) ^ uint32_t(ex_fh[0][e]);
      ix.tag[t] = static_cast<uint16_t>((w ^ (w >> 10) ^ uint32_t(ex_fh[1][e]) ^ (uint32_t(ex_fh[2][e]) << 1)) & FTB_TAGE_TAG_MASK);
    }
    if (cfg_fe_itt_cfg() == 0) {
      for (int k = 0; k < ITT_MAX; k++) {
        ix.itt_idx[k] = ix.idx[IttCfg::h[k]];
        ix.itt_tag[k] = ix.tag[IttCfg::h[k]];
      }
    } else {
      for (int k = 0; k < ITT_MAX_EXT; k++) {
        const int e = ITT_EX_MAP4[k];
        ix.itt_idx[k] = w ^ (w >> 10) ^ uint32_t(ex_fh[0][e]);
        ix.itt_tag[k] = static_cast<uint16_t>((w ^ (w >> 10) ^ uint32_t(ex_fh[1][e]) ^ (uint32_t(ex_fh[2][e]) << 1)) & FTB_TAGE_TAG_MASK);
      }
    }
    return ix;
  }

  // Unified index builder for 1-Ahead/2-Ahead TAGE & ITT (projects directly from unified ex_fh[3][6])
  static FtbBlkIdx make_ahead_index(pc_t key_base, pc_t cur_base, const uint16_t ex_fh[EX_FH_K][EX_TN],
                                    const uint16_t (*tag_ex_fh)[EX_TN] = nullptr,
                                    uint64_t prev_base = 0, bool is_self0 = false) {
    FtbBlkIdx ix;
    const uint32_t wk = static_cast<uint32_t>(key_base >> SLOT_SHIFT);
    const uint32_t wc = static_cast<uint32_t>(cur_base >> SLOT_SHIFT);
    const uint16_t (*tfh)[EX_TN] = (cfg_fe_ah_cur_tag() && tag_ex_fh != nullptr) ? tag_ex_fh : ex_fh;
    const uint32_t sec = (!is_self0 && cfg_fe_ah_sec_tag() && prev_base != 0)
                             ? (static_cast<uint32_t>(prev_base >> 1) ^ static_cast<uint32_t>(prev_base >> 7))
                             : 0u;
    const uint32_t self_xor = is_self0 ? 0x2aau : 0u;
    ix.w = wk;
    ix.base_idx = wk ^ (wk >> 11);
    for (int t = 0; t < TN; t++) {
      const int e = AH_EX_MAP[t];
      ix.idx[t] = wk ^ (wk >> 10) ^ uint32_t(ex_fh[0][e]);
      ix.tag[t] = static_cast<uint16_t>((wc ^ (wc >> 10) ^ wk ^ uint32_t(tfh[1][e]) ^ (uint32_t(tfh[2][e]) << 1) ^ sec ^ self_xor) & FTB_TAGE_TAG_MASK);
    }
    if (cfg_fe_itt_cfg() == 0) {
      for (int k = 0; k < ITT_MAX; k++) {
        ix.itt_idx[k] = ix.idx[IttCfg::h[k]];
        ix.itt_tag[k] = ix.tag[IttCfg::h[k]];
      }
    } else {
      for (int k = 0; k < ITT_MAX_EXT; k++) {
        const int e = ITT_EX_MAP4[k];
        ix.itt_idx[k] = wk ^ (wk >> 10) ^ uint32_t(ex_fh[0][e]);
        ix.itt_tag[k] = static_cast<uint16_t>((wc ^ (wc >> 10) ^ wk ^ uint32_t(tfh[1][e]) ^ (uint32_t(tfh[2][e]) << 1) ^ sec ^ self_xor) & FTB_TAGE_TAG_MASK);
      }
    }
    return ix;
  }

  struct AhSlotTage {
    uint8_t pcpn = EX_TN;
    bool prov_dir = false;
    bool alt_is_tagged = false;
    bool alt_dir = false;
    bool use_alt = false;
  };
  struct AhDirIttOut {
    bool valid = false;
    AhSlotTage s[FTB_SLOTS];
    bool itt_hit = false;
    target_addr_t itt_tgt = 0;
  };

  static bool resolve_ah_dir(const AhSlotTage &ts, uint8_t ftb_ctr, bool &alt_out, bool &used_alt_out) {
    const bool bp = (ftb_ctr >= 2);
    alt_out = ts.alt_is_tagged ? ts.alt_dir : bp;
    if (ts.pcpn >= EX_TN) {
      used_alt_out = false;
      return bp;
    }
    used_alt_out = ts.use_alt;
    return ts.use_alt ? alt_out : ts.prov_dir;
  }

  void ahead_read_tage_itt(pc_t key_base, const uint16_t key_ex_fh[EX_FH_K][EX_TN], FtbAhRaw &r,
                           uint8_t tage_sup = 0, uint8_t itt_sup = 0) const {
    const ExTageIdx ex_ix = make_ahead_ex_tage_index(key_base, 0, key_ex_fh, ah_ex_tage_.S);
    ah_ex_tage_.read(ex_ix, r.ex_tage_raw, tage_sup);
    const FtbBlkIdx ix = make_ahead_index(key_base, 0, key_ex_fh);
    aitt_.read(ix, r.itt_raw, itt_sup);
  }

  void ahead_dir_itt_select(pc_t cur_base, const uint16_t cur_ex_fh[EX_FH_K][EX_TN],
                            const FtbAhPipe &ah, AhDirIttOut &out, AhMeta *meta_out = nullptr,
                            pc_t prev_base = 0, bool is_self0 = false) const {
    out = AhDirIttOut{};
    if (meta_out) *meta_out = AhMeta{};
    if (!ah.valid) return;
    out.valid = true;
    const ExTageIdx ex_ix = make_ahead_ex_tage_index(ah.key_base, cur_base, ah.key_ex_fh, ah_ex_tage_.S,
                                                     cur_ex_fh, prev_base, is_self0);
    const FtbBlkIdx ix = make_ahead_index(ah.key_base, cur_base, ah.key_ex_fh, cur_ex_fh, prev_base, is_self0);
    const uint8_t ua = ah_ex_tage_.use_alt[(ex_ix.w >> 2) & 15u];
    for (int k = 0; k < FTB_SLOTS; k++) {
      const ExBlockTage::RawSlot &r = ah.raw.ex_tage_raw.s[k];
      int pcpn = EX_TN, altpcpn = EX_TN;
      int prov_w = 0, alt_w = 0;
      for (int t = 0; t < EX_TN; t++) {
        if (((r.sup_mask >> t) & 1u) != 0) continue;
        for (int w = 0; w < ah_ex_tage_.W; w++) {
          if (r.tag[t][w] == ex_ix.tag[t]) {
            altpcpn = pcpn;
            alt_w = prov_w;
            pcpn = t;
            prov_w = w;
            break;
          }
        }
      }
      const uint8_t prov_cnt = (pcpn < EX_TN) ? r.cnt[pcpn][prov_w] : 0;
      const bool weak = (pcpn < EX_TN) && (prov_cnt == 3 || prov_cnt == 4);
      const bool ulow = (pcpn < EX_TN) && (r.useful[pcpn][prov_w] == 0);
      out.s[k].pcpn = static_cast<uint8_t>(pcpn);
      out.s[k].prov_dir = (pcpn < EX_TN) && (prov_cnt >= 4);
      out.s[k].alt_is_tagged = (altpcpn < EX_TN);
      out.s[k].alt_dir = (altpcpn < EX_TN) ? (r.cnt[altpcpn][alt_w] >= 4) : false;
      out.s[k].use_alt = (pcpn < EX_TN) && weak && ulow && (ua <= 8);
      if (meta_out) {
        auto &sm = meta_out->ex_tage.s[k];
        sm.pcpn = out.s[k].pcpn;
        sm.prov_way = static_cast<uint8_t>(prov_w);
        sm.prov_cnt = prov_cnt;
        sm.prov_u = (pcpn < EX_TN) ? r.useful[pcpn][prov_w] : 0;
        for (int t = 0; t < EX_TN; t++) {
          for (int w = 0; w < ah_ex_tage_.W; w++) sm.useful[t][w] = r.useful[t][w];
        }
      }
    }
    out.itt_hit = aitt_.select(ix, ah.raw.itt_raw, out.itt_tgt, meta_out ? &meta_out->itt : nullptr);
  }

  void ahead_read(pc_t key_base, const uint16_t key_ex_fh[EX_FH_K][EX_TN], FtbAhRaw &r,
                  uint8_t tage_sup = 0, uint8_t itt_sup = 0) const {
    const FtbBlkIdx ix = make_ahead_index(key_base, 0, key_ex_fh);
    aftb_.read_set(key_base, r.ftb_set);
    if (cfg_fe_ah_ex_tage()) {
      const ExTageIdx ex_ix = make_ahead_ex_tage_index(key_base, 0, key_ex_fh, ah_ex_tage_.S);
      ah_ex_tage_.read(ex_ix, r.ex_tage_raw, tage_sup);
    } else {
      atage_.read(ix, r.tage_raw, tage_sup);
    }
    aitt_.read(ix, r.itt_raw, itt_sup);
  }

  template <class S0GuessT>
  bool ahead_select(pc_t base, pc_t bnd, const FtbAhPipe &ah, uint8_t rcnt, target_addr_t rtop, S0GuessT &g,
                    AhMeta *meta_out = nullptr, bool *slot_dir_out = nullptr, PredMeta *pred_meta_out = nullptr,
                    pc_t *ret_bpc_out = nullptr) const {
    if (meta_out) *meta_out = AhMeta{};
    if (ret_bpc_out) *ret_bpc_out = 0;
    if (slot_dir_out) for (int i = 0; i < FW; i++) slot_dir_out[i] = false;
    if (pred_meta_out) {
      for (int i = 0; i < FW; i++) {
        pred_meta_out->pcpn[i] = TN;
        pred_meta_out->alt_pred[i] = false;
        pred_meta_out->ps_info[i] = 0;
      }
    }
    if (!ah.valid) return false;
    FtbEnt ent;
    const bool hit = aftb_.select_way(ah.raw.ftb_set, ah.key_base, base, ent, meta_out ? &meta_out->ftb : nullptr);
    const FtbBlkIdx ix = make_ahead_index(ah.key_base, base, ah.key_ex_fh);
    bool dirs[FTB_SLOTS] = {};
    bool tp_alt[FTB_SLOTS] = {};
    uint8_t tp_pcpn[FTB_SLOTS] = {};
    bool tp_ualt[FTB_SLOTS] = {};
    if (cfg_fe_ah_ex_tage()) {
      const ExTageIdx ex_ix = make_ahead_ex_tage_index(ah.key_base, base, ah.key_ex_fh, ah_ex_tage_.S);
      ExBlockTage::Pred etp[FTB_SLOTS];
      for (int k = 0; k < FTB_SLOTS; k++) {
        const int base_ctr = (hit && ent.s[k].valid) ? int(ent.s[k].ctr) : 0;
        ah_ex_tage_.select_slot(ex_ix, ah.raw.ex_tage_raw.s[k], etp[k],
                                meta_out ? &meta_out->ex_tage.s[k] : nullptr, base_ctr);
        dirs[k] = etp[k].pred;
        tp_alt[k] = etp[k].alt;
        tp_pcpn[k] = (etp[k].pcpn < EX_TN) ? static_cast<uint8_t>(std::min<int>(etp[k].pcpn, TN - 1)) : static_cast<uint8_t>(TN);
        tp_ualt[k] = etp[k].used_alt;
      }
    } else {
      BlockTage::Pred tp[FTB_SLOTS];
      for (int k = 0; k < FTB_SLOTS; k++) {
        atage_.select_slot(ix, ah.raw.tage_raw.s[k], tp[k], (hit && ent.s[k].valid) ? int(ent.s[k].ctr) : 0,
                           meta_out ? &meta_out->tage.s[k] : nullptr);
        dirs[k] = tp[k].pred;
        tp_alt[k] = tp[k].alt;
        tp_pcpn[k] = tp[k].pcpn;
        tp_ualt[k] = tp[k].used_alt;
      }
    }
    target_addr_t itt_tgt = 0;
    const bool itt_hit = aitt_.select(ix, ah.raw.itt_raw, itt_tgt, meta_out ? &meta_out->itt : nullptr);
    if (!hit) return false;
    g = S0GuessT{};
    g.hit = true;
    bool slot_tk[FTB_SLOTS] = {};
    eval_ent(base, bnd, ent, true, dirs, itt_hit, itt_tgt, rcnt, rtop, g,
             (slot_dir_out || pred_meta_out) ? slot_tk : nullptr, ret_bpc_out);
    if (slot_dir_out || pred_meta_out) {
      for (int k = 0; k < FTB_SLOTS; k++) {
        const FtbSlot &sl = ent.s[k];
        if (!sl.valid || sl.offset >= FW) continue;
        if (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES >= bnd) continue;
        const int off = sl.offset;
        if (slot_dir_out) slot_dir_out[off] = slot_tk[k];
        if (pred_meta_out) {
          pred_meta_out->alt_pred[off] = tp_alt[k];
          pred_meta_out->pcpn[off] = tp_pcpn[k];
          pred_meta_out->ps_info[off] = ps_pack(sl.type, (br_ind(sl.type) && itt_hit) ? 4 : (sl.type == BR_RET && rcnt > 0 ? 7 : 1), tp_ualt[k]);
        }
      }
    }
    return true;
  }

  template <class GuessT>
  static void eval_ent(pc_t base, pc_t bnd, const FtbEnt &ent,
                       bool use_dir_ovr, const bool dir_ovr[FTB_SLOTS],
                       bool itt_hit, target_addr_t itt_tgt,
                       uint8_t rcnt, target_addr_t rtop,
                       GuessT &g, bool *slot_dir_out = nullptr, pc_t *ret_bpc_out = nullptr) {
    g.next = bnd;
    g.taken = false;
    g.slot = 0;
    g.type = BR_NONCTL;
    g.nbits = 0;
    g.bits = 0;
    g.ras_op = 0;
    g.ras_val = 0;
    int best_k = -1;
    uint8_t best_off = 255;
    for (int k = 0; k < FTB_SLOTS; k++) {
      const FtbSlot &sl = ent.s[k];
      if (!sl.valid) continue;
      const pc_t bpc = (sl.offset == 0 && sl.carried)
                           ? (base - SLOT_BYTES)
                           : (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES);
      if (bpc >= bnd) continue;
      const bool tk = (sl.type == BR_DIRECT) ? (use_dir_ovr ? dir_ovr[k] : (sl.ctr >= 2)) : true;
      if (slot_dir_out) slot_dir_out[k] = tk;
      if (tk && sl.offset < best_off) {
        best_off = sl.offset;
        best_k = k;
      }
    }
    if (best_k >= 0) {
      const FtbSlot &sl = ent.s[best_k];
      const pc_t bpc = (sl.offset == 0 && sl.carried)
                           ? (base - SLOT_BYTES)
                           : (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES);
      if (ret_bpc_out) *ret_bpc_out = bpc;
      pc_t tgt = sl.target;
      if (sl.type == BR_RET && rcnt > 0) tgt = rtop;
      else if (br_ind(sl.type) && itt_hit) tgt = itt_tgt;
      g.taken = true;
      g.slot = sl.offset;
      g.type = sl.type;
      g.rvc = sl.is_rvc;
      g.carried = sl.carried;
      g.next = tgt;
      g.nbits = HSYM_BITS;
      g.bits = hist_sym(bpc, tgt);
      if (br_call(sl.type)) { g.ras_op = 1; g.ras_val = bpc + ilen_of(sl.is_rvc); }
      else if (sl.type == BR_RET) { g.ras_op = 2; }
    }
  }

  template <class S0GuessT>
  S0GuessT s0_lookup(pc_t base, pc_t bnd, uint8_t rcnt, target_addr_t rtop,
                     bool *slot_dir_out = nullptr, PredMeta *pred_meta_out = nullptr) const {
    if (slot_dir_out) for (int i = 0; i < FW; i++) slot_dir_out[i] = false;
    if (pred_meta_out) {
      for (int i = 0; i < FW; i++) {
        pred_meta_out->pcpn[i] = TN;
        pred_meta_out->alt_pred[i] = false;
        pred_meta_out->ps_info[i] = 0;
      }
    }
    S0GuessT g{};
    g.next = bnd;
    FtbEnt ent;
    if (!ftb0_.lookup(base, ent)) return g;
    g.hit = true;
    bool slot_tk[FTB_SLOTS] = {};
    eval_ent(base, bnd, ent, false, nullptr, false, 0, rcnt, rtop, g,
             (slot_dir_out || pred_meta_out) ? slot_tk : nullptr);
    if (slot_dir_out || pred_meta_out) {
      for (int k = 0; k < FTB_SLOTS; k++) {
        const FtbSlot &sl = ent.s[k];
        if (!sl.valid || sl.offset >= FW) continue;
        if (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES >= bnd) continue;
        const int off = sl.offset;
        if (slot_dir_out) slot_dir_out[off] = slot_tk[k];
        if (pred_meta_out) {
          pred_meta_out->alt_pred[off] = (sl.ctr >= 2);
          pred_meta_out->pcpn[off] = TN;
          pred_meta_out->ps_info[off] = ps_pack(sl.type, (sl.type == BR_RET && rcnt > 0) ? 7 : 1, false);
        }
      }
    }
    return g;
  }

  void s1_read(pc_t base, const FtbBlkIdx &ix, const ExTageIdx &ex_ix,
               FtbRaw &r, uint8_t tage_sup = 0, uint8_t itt_sup = 0) const {
    r = FtbRaw{};
    r.ftb_hit = ftb1_.lookup(base, r.ent, &r.ftb_meta);
    s2_tage_.read(ex_ix, r.s2_tage_raw, tage_sup);
    itt_.read(ix, r.itt_raw, itt_sup);
  }

  template <class MainOutT>
  void s2_select(pc_t base, pc_t bnd, const FtbBlkIdx &ix, const ExTageIdx &ex_ix,
                 const FtbRaw &r, uint8_t rcnt, target_addr_t rtop,
                 MainOutT &o, S2Meta *meta_out = nullptr) const {
    o = MainOutT{};
    if (meta_out) {
      *meta_out = S2Meta{};
      meta_out->ftb = r.ftb_meta;
    }
    o.next = bnd;
    for (int i = 0; i < FW; i++) {
      o.meta.pcpn[i] = TN;
      o.meta.alt_pred[i] = false;
    }

    bool dirs[FTB_SLOTS] = {};
    ExBlockTage::Pred tp[FTB_SLOTS];
    for (int k = 0; k < FTB_SLOTS; k++) {
      s2_tage_.select_slot(ex_ix, r.s2_tage_raw.s[k], tp[k], meta_out ? &meta_out->tage.s[k] : nullptr);
      dirs[k] = tp[k].pred;
    }
    target_addr_t itt_tgt = 0;
    const bool itt_hit = itt_.select(ix, r.itt_raw, itt_tgt, meta_out ? &meta_out->itt : nullptr);
    if (!r.ftb_hit) return;

    bool slot_tk[FTB_SLOTS] = {};
    eval_ent(base, bnd, r.ent, true, dirs, itt_hit, itt_tgt, rcnt, rtop, o, slot_tk);
    if (o.taken) {
      for (int k = 0; k < FTB_SLOTS; k++) {
        if (r.ent.s[k].valid && r.ent.s[k].offset == o.slot) { o.rvc = r.ent.s[k].is_rvc; break; }
      }
    }
    for (int k = 0; k < FTB_SLOTS; k++) {
      const FtbSlot &sl = r.ent.s[k];
      if (!sl.valid || sl.offset >= FW) continue;
      if (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES >= bnd) continue;
      const int off = sl.offset;
      o.dir[off] = slot_tk[k];
      o.meta.alt_pred[off] = tp[k].alt;
      o.meta.pcpn[off] = (tp[k].pcpn < EX_TN) ? static_cast<uint8_t>(std::min<int>(tp[k].pcpn, TN - 1)) : static_cast<uint8_t>(TN);
      o.meta.ps_info[off] = ps_pack(sl.type, (br_ind(sl.type) && itt_hit) ? 4 : (sl.type == BR_RET && rcnt > 0 ? 7 : 1), tp[k].used_alt);
    }
  }

  template <class S0GuessT>
  S0GuessT s0_lookup_with_ah(pc_t base, pc_t bnd, const AhDirIttOut &ah_di,
                             uint8_t rcnt, target_addr_t rtop) const {
    S0GuessT g{};
    g.next = bnd;
    FtbEnt ent;
    if (!ftb0_.lookup(base, ent)) return g;
    g.hit = true;
    bool dirs[FTB_SLOTS] = {};
    bool unused_alt = false, unused_ualt = false;
    for (int k = 0; k < FTB_SLOTS; k++) {
      dirs[k] = ah_di.valid ? resolve_ah_dir(ah_di.s[k], ent.s[k].ctr, unused_alt, unused_ualt)
                            : (ent.s[k].ctr >= 2);
    }
    eval_ent(base, bnd, ent, true, dirs, ah_di.valid && ah_di.itt_hit, ah_di.itt_tgt, rcnt, rtop, g);
    return g;
  }

  void s1_read_uni(pc_t base, const ExTageIdx &ex_ix, FtbRaw &r, uint8_t sup_tage_sup = 0) const {
    r = FtbRaw{};
    r.ftb_hit = ftb1_.lookup(base, r.ent, &r.ftb_meta);
    if (cfg_fe_s2_sup_tage() > 0) {
      s2_tage_.read(ex_ix, r.s2_tage_raw, sup_tage_sup);
    }
  }

  template <class MainOutT>
  void s2_select_uni(pc_t base, pc_t bnd, const ExTageIdx &ex_ix,
                     const FtbRaw &r, bool ah_v, const AhDirIttOut &ah_di, const AhMeta &s0_ah_meta,
                     const uint16_t cur_ex_fh[EX_FH_K][EX_TN],
                     uint8_t rcnt, target_addr_t rtop,
                     MainOutT &o, S2Meta *s2_meta_out = nullptr, AhMeta *ah_meta_out = nullptr,
                     bool *self0_used_out = nullptr) const {
    o = MainOutT{};
    if (self0_used_out) *self0_used_out = false;
    if (s2_meta_out) {
      *s2_meta_out = S2Meta{};
      s2_meta_out->ftb = r.ftb_meta;
    }
    bool eff_ah_v = ah_v;
    AhDirIttOut self0_di{};
    const AhDirIttOut *di_ptr = &ah_di;
    if (cfg_fe_ah_self0() && !ah_v && r.self0_v) {
      ahead_dir_itt_select(base, cur_ex_fh, r.self0_pipe, self0_di, ah_meta_out, 0, true);
      eff_ah_v = true;
      di_ptr = &self0_di;
      if (self0_used_out) *self0_used_out = true;
    } else if (ah_meta_out) {
      *ah_meta_out = s0_ah_meta;
    }
    const AhDirIttOut &eff_di = *di_ptr;
    o.next = bnd;
    for (int i = 0; i < FW; i++) {
      o.meta.pcpn[i] = TN;
      o.meta.alt_pred[i] = false;
    }
    if (!r.ftb_hit) return;

    const bool use_sup = (cfg_fe_s2_sup_tage() > 0);
    bool dirs[FTB_SLOTS] = {};
    bool slot_alt[FTB_SLOTS] = {};
    uint8_t slot_pcpn[FTB_SLOTS] = {};
    bool slot_ualt[FTB_SLOTS] = {};
    for (int k = 0; k < FTB_SLOTS; k++) {
      bool ah_alt = (r.ent.s[k].ctr >= 2), ah_ualt = false;
      const bool ah_dir = eff_ah_v ? resolve_ah_dir(eff_di.s[k], r.ent.s[k].ctr, ah_alt, ah_ualt)
                                   : (r.ent.s[k].ctr >= 2);
      if (ah_meta_out) {
        auto &am = ah_meta_out->ex_tage.s[k];
        am.base_cnt = r.ent.s[k].ctr;
        am.alt = ah_alt;
        am.pred = ah_dir;
        if (am.pcpn >= EX_TN) am.prov_cnt = r.ent.s[k].ctr;
      }
      uint8_t win_pcpn = (eff_ah_v && eff_di.s[k].pcpn < EX_TN)
                             ? static_cast<uint8_t>(std::min<int>(eff_di.s[k].pcpn, TN - 1))
                             : static_cast<uint8_t>(TN);
      bool win_alt = ah_alt;
      bool win_ualt = ah_ualt;
      bool win_dir = ah_dir;
      if (use_sup) {
        ExBlockTage::Pred sup_p;
        s2_tage_.select_slot(ex_ix, r.s2_tage_raw.s[k], sup_p,
                             s2_meta_out ? &s2_meta_out->tage.s[k] : nullptr,
                             int(r.ent.s[k].ctr));
        const bool sup_valid_hit = (sup_p.pcpn < EX_TN) && !sup_p.used_alt;
        const bool sup_wins = sup_valid_hit && (!eff_ah_v || eff_di.s[k].pcpn >= EX_TN || ah_ualt);
        if (sup_wins) {
          win_dir = sup_p.pred;
          win_alt = sup_p.alt;
          win_pcpn = static_cast<uint8_t>(std::min<int>(sup_p.pcpn, TN - 1));
          win_ualt = sup_p.used_alt;
        }
      }
      dirs[k] = win_dir;
      slot_alt[k] = win_alt;
      slot_pcpn[k] = win_pcpn;
      slot_ualt[k] = win_ualt;
    }
    const bool itt_hit = eff_ah_v && eff_di.itt_hit;
    const target_addr_t itt_tgt = eff_di.itt_tgt;
    bool slot_tk[FTB_SLOTS] = {};
    eval_ent(base, bnd, r.ent, true, dirs, itt_hit, itt_tgt, rcnt, rtop, o, slot_tk);
    if (o.taken) {
      for (int k = 0; k < FTB_SLOTS; k++) {
        if (r.ent.s[k].valid && r.ent.s[k].offset == o.slot) { o.rvc = r.ent.s[k].is_rvc; break; }
      }
    }
    for (int k = 0; k < FTB_SLOTS; k++) {
      const FtbSlot &sl = r.ent.s[k];
      if (!sl.valid || sl.offset >= FW) continue;
      if (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES >= bnd) continue;
      const int off = sl.offset;
      o.dir[off] = slot_tk[k];
      o.meta.alt_pred[off] = slot_alt[k];
      o.meta.pcpn[off] = slot_pcpn[k];
      o.meta.ps_info[off] = ps_pack(sl.type, (br_ind(sl.type) && itt_hit) ? 4 : (sl.type == BR_RET && rcnt > 0 ? 7 : 1), slot_ualt[k]);
    }
  }

  void mirror_ent_to_ftb0(pc_t base, const FtbEnt &main_ent, bool has_taken) {
    FtbEnt u{};
    u.tag = ftb0_.tag(base);
    bool any_v = false;
    for (int k = 0; k < FTB_SLOTS; k++) {
      if (!main_ent.s[k].valid) continue;
      if (main_ent.s[k].type != BR_RET && !ftb0_.tgt_fits(base, main_ent.s[k].target)) continue;
      u.s[k] = main_ent.s[k];
      any_v = true;
    }
    const uint32_t s = ftb0_.index(base);
    const uint16_t tg = u.tag;
    int hit = -1;
    for (int w = 0; w < ftb0_.W; w++) {
      if (ftb0_.e[s * ftb0_.W + w].valid && ftb0_.e[s * ftb0_.W + w].tag == tg) { hit = w; break; }
    }
    if (hit >= 0) {
      if (any_v) {
        u.valid = true;
        ftb0_.e[s * ftb0_.W + hit] = u;
        ftb0_.touch_lru(s, hit);
      } else {
        ftb0_.e[s * ftb0_.W + hit].valid = false;
      }
    } else if (has_taken && any_v) {
      int vic = -1;
      for (int w = 0; w < ftb0_.W; w++) if (!ftb0_.e[s * ftb0_.W + w].valid) { vic = w; break; }
      if (vic < 0) vic = ftb0_.plru.victim(s);
      u.valid = true;
      ftb0_.e[s * ftb0_.W + vic] = u;
      ftb0_.touch_lru(s, vic);
    }
  }

  static int upsert_branch(FtbEnt &ent, const FtbTrkBr &b) {
    const pc_t tgt38 = sv39_va_roundtrip(b.target);
    for (int k = 0; k < FTB_SLOTS; k++) {
      if (ent.s[k].valid && ent.s[k].offset == b.offset) {
        ent.s[k].type = b.type;
        ent.s[k].carried = b.carried;
        ent.s[k].is_rvc = b.is_rvc;
        if (b.actual_dir) ent.s[k].target = tgt38;
        if (b.type == BR_DIRECT) {
          ent.s[k].ctr = b.actual_dir ? (ent.s[k].ctr < 3 ? ent.s[k].ctr + 1 : 3)
                                      : (ent.s[k].ctr > 0 ? ent.s[k].ctr - 1 : 0);
        } else {
          ent.s[k].ctr = 3;
        }
        return k;
      }
    }
    int pos = -1;
    for (int k = 0; k < FTB_SLOTS; k++) if (!ent.s[k].valid) { pos = k; break; }
    if (pos < 0) {
      if (!b.actual_dir && b.type == BR_DIRECT) return -1;
      pos = FTB_SLOTS - 1;
      int min_c = 99;
      for (int k = 0; k < FTB_SLOTS; k++) {
        if (ent.s[k].type == BR_DIRECT && int(ent.s[k].ctr) < min_c) { min_c = ent.s[k].ctr; pos = k; }
      }
    }
    FtbSlot ns{};
    ns.valid = true;
    ns.offset = b.offset;
    ns.carried = b.carried;
    ns.is_rvc = b.is_rvc;
    ns.type = b.type;
    ns.ctr = (b.type == BR_DIRECT) ? (b.actual_dir ? 2 : 1) : 3;
    ns.target = tgt38;
    ent.s[pos] = ns;
    return pos;
  }

  static FtbEnt *update_tab(FtbTab &tab, const FtbTrainEv &ev, bool alloc_only_taken) {
    if (ev.nbr == 0) return nullptr;
    bool has_taken = false, any_fits = false;
    for (int i = 0; i < ev.nbr; i++) {
      if (ev.br[i].actual_dir) has_taken = true;
      if (!ev.br[i].actual_dir || ev.br[i].type == BR_RET || tab.tgt_fits(ev.start, ev.br[i].target)) any_fits = true;
    }
    if (!any_fits) return nullptr;
    const uint32_t s = tab.index(ev.start);
    const uint16_t t = tab.tag(ev.start);
    int hit = -1;
    for (int w = 0; w < tab.W; w++) if (tab.e[s * tab.W + w].valid && tab.e[s * tab.W + w].tag == t) { hit = w; break; }
    if (hit < 0) {
      if (alloc_only_taken && !has_taken) return nullptr;
      int vic = -1;
      for (int w = 0; w < tab.W; w++) if (!tab.e[s * tab.W + w].valid) { vic = w; break; }
      if (vic < 0) vic = tab.plru.victim(s);
      hit = vic;
      FtbEnt &en = tab.e[s * tab.W + hit];
      en = FtbEnt{};
      en.valid = true;
      en.tag = t;
    }

    FtbEnt &en = tab.e[s * tab.W + hit];
    for (int i = 0; i < ev.nbr; i++) {
      const FtbTrkBr &b = ev.br[i];
      if (b.actual_dir && b.type != BR_RET && !tab.tgt_fits(ev.start, b.target)) continue;
      upsert_branch(en, b);
    }
    bool any_v = false;
    for (int k = 0; k < FTB_SLOTS; k++) if (en.s[k].valid) any_v = true;
    if (!any_v) { en.valid = false; return nullptr; }
    tab.touch_lru(s, hit);
    return &en;
  }

  // Used only by FastWarm
  void apply_train_ev(const FtbTrainEv &ev) {
    const int pred_arch = cfg_fe_pred_arch();
    FtbEnt *main_ent = cfg_fe_single_pred() ? nullptr : update_tab(ftb1_, ev, false);
    if (pred_arch == 2) {
      bool any_taken = false;
      for (int i = 0; i < ev.nbr; i++) if (ev.br[i].actual_dir) any_taken = true;
      if (main_ent) mirror_ent_to_ftb0(ev.start, *main_ent, any_taken);
      if (main_ent && cfg_fe_s2_sup_tage() > 0) {
        const uint32_t w = static_cast<uint32_t>(ev.start >> SLOT_SHIFT);
        const ExTageIdx ex_ix = make_ex_tage_index(w, ev.ex_fh);
        for (int i = 0; i < ev.nbr; i++) {
          const FtbTrkBr &b = ev.br[i];
          if (b.type == BR_DIRECT) {
            int sl = -1;
            for (int k = 0; k < FTB_SLOTS; k++) if (main_ent->s[k].valid && main_ent->s[k].offset == b.offset) { sl = k; break; }
            if (sl >= 0) s2_tage_.update_slot(ex_ix, sl, b.actual_dir, nullptr, int(main_ent->s[sl].ctr));
          }
        }
      }
      if (main_ent && ev.ah_v && ev.nbr > 0) {
        const FtbBlkIdx aix = make_ahead_index(ev.ah_key_base, ev.start, ev.ah_key_ex_fh, ev.ex_fh);
        const ExTageIdx aex = make_ahead_ex_tage_index(ev.ah_key_base, ev.start, ev.ah_key_ex_fh, ah_ex_tage_.S, ev.ex_fh);
        for (int i = 0; i < ev.nbr; i++) {
          const FtbTrkBr &b = ev.br[i];
          if (b.type == BR_DIRECT) {
            int sl = -1;
            for (int k = 0; k < FTB_SLOTS; k++) if (main_ent->s[k].valid && main_ent->s[k].offset == b.offset) { sl = k; break; }
            if (sl >= 0) ah_ex_tage_.update_slot(aex, sl, b.actual_dir, nullptr, int(main_ent->s[sl].ctr));
          }
          if (br_ind(b.type) && b.actual_dir) aitt_.update(aix, b.target);
        }
      }
      return;
    }
    update_tab(ftb0_, ev, true);
    if (main_ent) {
      const uint32_t w = static_cast<uint32_t>(ev.start >> SLOT_SHIFT);
      const FtbBlkIdx ix = make_index(ev.start, ev.ex_fh);
      const ExTageIdx ex_ix = make_ex_tage_index(w, ev.ex_fh);
      for (int i = 0; i < ev.nbr; i++) {
        const FtbTrkBr &b = ev.br[i];
        if (b.type == BR_DIRECT) {
          int sl = -1;
          for (int k = 0; k < FTB_SLOTS; k++) if (main_ent->s[k].valid && main_ent->s[k].offset == b.offset) { sl = k; break; }
          if (sl >= 0) s2_tage_.update_slot(ex_ix, sl, b.actual_dir);
        }
        if (br_ind(b.type) && b.actual_dir) itt_.update(ix, b.target);
      }
    }
    if (pred_arch == 1) return;
    if (ev.ah_v && ev.nbr > 0) {
      const uint32_t s = aftb_.index(ev.ah_key_base);
      const uint32_t t = aftb_.tag(ev.ah_key_base, ev.start);
      int hit = -1;
      for (int ww = 0; ww < aftb_.W; ww++) if (aftb_.e[s * aftb_.W + ww].valid && aftb_.e[s * aftb_.W + ww].tag32 == t) { hit = ww; break; }
      if (hit < 0) {
        int vic = -1;
        for (int ww = 0; ww < aftb_.W; ww++) if (!aftb_.e[s * aftb_.W + ww].valid) { vic = ww; break; }
        if (vic < 0) vic = aftb_.plru.victim(s);
        hit = vic;
        FtbEnt &en = aftb_.e[s * aftb_.W + hit];
        en = FtbEnt{}; en.valid = true; en.tag32 = t;
      }
      FtbEnt &aen = aftb_.e[s * aftb_.W + hit];
      for (int i = 0; i < ev.nbr; i++) upsert_branch(aen, ev.br[i]);
      aftb_.touch_lru(s, hit);
      const FtbBlkIdx aix = make_ahead_index(ev.ah_key_base, ev.start, ev.ah_key_ex_fh);
      const ExTageIdx aex = cfg_fe_ah_ex_tage()
                                ? make_ahead_ex_tage_index(ev.ah_key_base, ev.start, ev.ah_key_ex_fh, ah_ex_tage_.S)
                                : ExTageIdx{};
      for (int i = 0; i < ev.nbr; i++) {
        const FtbTrkBr &b = ev.br[i];
        if (b.type == BR_DIRECT) {
          int sl = -1;
          for (int k = 0; k < FTB_SLOTS; k++) if (aen.s[k].valid && aen.s[k].offset == b.offset) { sl = k; break; }
          if (sl >= 0) {
            if (cfg_fe_ah_ex_tage()) ah_ex_tage_.update_slot(aex, sl, b.actual_dir, nullptr, int(aen.s[sl].ctr));
            else atage_.update_slot(aix, sl, b.actual_dir, int(aen.s[sl].ctr));
          }
        }
        if (br_ind(b.type) && b.actual_dir) aitt_.update(aix, b.target);
      }
    }
  }

  // ---- ZERO-T0-READ 1-Cycle Pure Commit Write Plan ----
  struct CommitPlan {
    bool valid = false;
    bool ev_has_taken = false;
    FtbTrainEv ev{};
    bool ftb1_wr = false;
    uint32_t ftb1_s = 0;
    uint16_t ftb1_tag = 0;
    uint8_t ftb1_way = 0;
    FtbEnt ftb1_ent{};
    ExBlockTage::WritePlan tage_wp{};
    BlockItt::WritePlan itt_wp{};
    bool ah_wr = false;
    uint32_t ah_s = 0;
    uint32_t ah_tag = 0;
    uint8_t ah_way = 0;
    FtbEnt ah_ent{};
    BlockTage::WritePlan atage_wp{};
    ExBlockTage::WritePlan aex_tage_wp{};
    BlockItt::WritePlan aitt_wp{};
  };

  void compute_commit_plan(const FtbBlkRec &rc, const FtbTrkBr &b, CommitPlan &p) const {
    p = CommitPlan{};
    p.valid = true;
    p.ev_has_taken = b.actual_dir;
    p.ev.start = rc.base;
    p.ev.nbr = 1;
    p.ev.br[0] = b;

    const int pred_arch = cfg_fe_pred_arch();
    if (!cfg_fe_single_pred()) {
      // 1. Main FTB1 (Meta + WRBypass + commit-time PLRU victim)
      const uint32_t s = ftb1_.index(rc.base);
      const uint16_t tg = ftb1_.tag(rc.base);
      uint8_t way = 0;
      FtbEnt old_ent{};
      bool had_hit = false;
      if (ftb1_.wrb.lookup(s, tg, way, old_ent)) {
        had_hit = old_ent.valid && (old_ent.tag == tg);
      } else if (rc.ftb1_meta.hit) {
        had_hit = true;
        old_ent = rc.ftb1_meta.ent;
        way = rc.ftb1_meta.way;
      } else {
        way = ftb1_.plru.victim(s);
      }
      FtbEnt en = had_hit ? old_ent : FtbEnt{};
      if (!had_hit) {
        en.valid = true;
        en.tag = tg;
      }
      const int sl = upsert_branch(en, b);
      bool any_v = false;
      for (int k = 0; k < FTB_SLOTS; k++) if (en.s[k].valid) any_v = true;
      if (!any_v) en.valid = false;
      p.ftb1_s = s;
      p.ftb1_tag = tg;
      p.ftb1_way = way;
      p.ftb1_ent = en;
      p.ftb1_wr = any_v && (!had_hit || !en.payload_eq(old_ent));

      const uint32_t w = static_cast<uint32_t>(rc.base >> SLOT_SHIFT);
      const FtbBlkIdx ix = make_index(rc.base, rc.ex_fh);
      const ExTageIdx ex_ix = make_ex_tage_index(w, rc.ex_fh);
      const bool slot_meta_ok = rc.ftb1_meta.hit && (sl >= 0) && rc.ftb1_meta.ent.s[sl].valid &&
                                (rc.ftb1_meta.ent.s[sl].offset == b.offset);

      if (pred_arch == 2) {
        const bool has_ah = rc.ah_v || rc.ah_self0;
        if (cfg_fe_s2_sup_tage() > 0 && b.type == BR_DIRECT && any_v && sl >= 0) {
          const bool ah_pred = has_ah ? rc.ah_ex_tage_meta.s[sl].pred
                                      : (slot_meta_ok ? (rc.ftb1_meta.ent.s[sl].ctr >= 2) : false);
          const bool sup_hit = slot_meta_ok && (rc.s2_tage_meta.s[sl].pcpn < EX_TN);
          const bool allow_alloc = (!has_ah) || (ah_pred != b.actual_dir);
          if (sup_hit || allow_alloc) {
            ExBlockTage::Pred ep;
            s2_tage_.compute_plan_from_meta(ex_ix, sl, b.actual_dir, slot_meta_ok,
                                            rc.s2_tage_meta.s[sl], p.tage_wp, ep, int(en.s[sl].ctr), allow_alloc);
          }
        }
        if (has_ah) {
          const bool is_s0 = !rc.ah_v && rc.ah_self0;
          const pc_t kbase = is_s0 ? rc.base : rc.ah_key_base;
          const uint16_t (*kfh)[EX_TN] = is_s0 ? rc.ex_fh : rc.ah_key_ex_fh;
          const pc_t pbase = is_s0 ? 0 : rc.prev_base;
          if (b.type == BR_DIRECT && any_v && sl >= 0) {
            const ExTageIdx aex = make_ahead_ex_tage_index(kbase, rc.base, kfh, ah_ex_tage_.S, rc.ex_fh, pbase, is_s0);
            ExBlockTage::Pred ep;
            ah_ex_tage_.compute_plan_from_meta(aex, sl, b.actual_dir, slot_meta_ok,
                                               rc.ah_ex_tage_meta.s[sl], p.aex_tage_wp, ep, int(en.s[sl].ctr));
          }
          if (br_ind(b.type) && b.actual_dir) {
            const FtbBlkIdx aix = make_ahead_index(kbase, rc.base, kfh, rc.ex_fh, pbase, is_s0);
            aitt_.compute_plan_from_meta(aix, b.target, true, rc.ah_itt_meta, p.aitt_wp);
          }
        }
        return;
      }

      // 2. S2 6-table ExBlockTage
      if (b.type == BR_DIRECT && any_v && sl >= 0) {
        ExBlockTage::Pred ep;
        s2_tage_.compute_plan_from_meta(ex_ix, sl, b.actual_dir, slot_meta_ok, rc.s2_tage_meta.s[sl], p.tage_wp, ep);
      }

      // 3. S2 ITT
      if (br_ind(b.type) && b.actual_dir) {
        itt_.compute_plan_from_meta(ix, b.target, true, rc.s2_itt_meta, p.itt_wp);
      }
    }

    if (pred_arch == 1) return;

    // 4. Ahead (aftb + atage/ah_ex_tage + aitt)
    if (rc.ah_v) {
      const uint32_t as = aftb_.index(rc.ah_key_base);
      const uint32_t at = aftb_.tag(rc.ah_key_base, rc.base);
      uint8_t away = 0;
      FtbEnt aold{};
      bool ahit = false;
      if (aftb_.wrb.lookup(as, at, away, aold)) {
        ahit = aold.valid && (aold.tag32 == at);
      } else if (rc.ah_ftb_meta.hit) {
        ahit = true;
        aold = rc.ah_ftb_meta.ent;
        away = rc.ah_ftb_meta.way;
      } else {
        away = aftb_.plru.victim(as);
      }
      FtbEnt aen = ahit ? aold : FtbEnt{};
      if (!ahit) {
        aen.valid = true;
        aen.tag32 = at;
      }
      const int asl = upsert_branch(aen, b);
      p.ah_s = as;
      p.ah_tag = at;
      p.ah_way = away;
      p.ah_ent = aen;
      p.ah_wr = (!ahit || !aen.payload_eq(aold));

      const FtbBlkIdx aix = make_ahead_index(rc.ah_key_base, rc.base, rc.ah_key_ex_fh);
      if (b.type == BR_DIRECT && asl >= 0) {
        const bool ah_slot_ok = rc.ah_ftb_meta.hit && rc.ah_ftb_meta.ent.s[asl].valid &&
                                (rc.ah_ftb_meta.ent.s[asl].offset == b.offset);
        if (cfg_fe_ah_ex_tage()) {
          const ExTageIdx aex = make_ahead_ex_tage_index(rc.ah_key_base, rc.base, rc.ah_key_ex_fh, ah_ex_tage_.S);
          ExBlockTage::Pred ep;
          ah_ex_tage_.compute_plan_from_meta(aex, asl, b.actual_dir, ah_slot_ok,
                                             rc.ah_ex_tage_meta.s[asl], p.aex_tage_wp, ep, int(aen.s[asl].ctr));
        } else {
          atage_.compute_plan_from_meta(aix, asl, b.actual_dir, int(aen.s[asl].ctr), ah_slot_ok,
                                        rc.ah_tage_meta.s[asl], p.atage_wp);
        }
      }
      if (br_ind(b.type) && b.actual_dir) {
        aitt_.compute_plan_from_meta(aix, b.target, true, rc.ah_itt_meta, p.aitt_wp);
      }
    }
  }

  void apply_ftb1_write(uint32_t s, uint8_t way, const FtbEnt &ent) {
    ftb1_.e[s * ftb1_.W + way] = ent;
  }
  void apply_aftb_write(uint32_t s, uint8_t way, const FtbEnt &ent) {
    aftb_.e[s * aftb_.W + way] = ent;
  }

  void apply_commit_non_ftb(const CommitPlan &p, bool do_s2_base) {
    if (!p.valid) return;
    const int pred_arch = cfg_fe_pred_arch();
    if (pred_arch == 2) {
      mirror_ent_to_ftb0(p.ev.start, p.ftb1_ent, p.ev_has_taken);
      if (p.ftb1_wr) {
        ftb1_.wrb.insert(p.ftb1_s, p.ftb1_tag, p.ftb1_way, p.ftb1_ent);
        ftb1_.plru.touch(p.ftb1_s, p.ftb1_way);
      }
      if (cfg_fe_s2_sup_tage() > 0 && p.tage_wp.valid) {
        s2_tage_.apply_plan(p.tage_wp, false);
      }
      if (p.aex_tage_wp.valid) ah_ex_tage_.apply_plan(p.aex_tage_wp, false);
      if (p.aitt_wp.wr_mask != 0) aitt_.apply_plan(p.aitt_wp);
      return;
    }
    update_tab(ftb0_, p.ev, true);
    if (!cfg_fe_single_pred()) {
      if (p.ftb1_wr) {
        ftb1_.wrb.insert(p.ftb1_s, p.ftb1_tag, p.ftb1_way, p.ftb1_ent);
        ftb1_.plru.touch(p.ftb1_s, p.ftb1_way);
      }
      s2_tage_.apply_plan(p.tage_wp, do_s2_base);
      itt_.apply_plan(p.itt_wp);
    }
    if (pred_arch == 1) return;
    if (p.ah_wr) {
      aftb_.wrb.insert(p.ah_s, p.ah_tag, p.ah_way, p.ah_ent);
      aftb_.plru.touch(p.ah_s, p.ah_way);
    }
    if (cfg_fe_ah_ex_tage()) {
      if (p.aex_tage_wp.valid) ah_ex_tage_.apply_plan(p.aex_tage_wp, false);
    } else {
      if (p.atage_wp.valid) atage_.apply_plan(p.atage_wp);
    }
    if (p.aitt_wp.wr_mask != 0) aitt_.apply_plan(p.aitt_wp);
  }

  void clear_false_branch_ftb0(pc_t base, pc_t nb_pc) {
    ftb0_.clear_slot(base, nb_pc);
  }
  void clear_false_branch_aftb(pc_t key_base, pc_t cur_base, pc_t nb_pc) {
    aftb_.clear_slot(key_base, cur_base, nb_pc);
  }
  void apply_ftb1_clr_write(uint32_t s, uint16_t tg, uint8_t way, const FtbEnt &ent) {
    ftb1_.e[s * ftb1_.W + way] = ent;
    ftb1_.wrb.insert(s, tg, way, ent);
  }
  void clear_false_branch(pc_t base, pc_t nb_pc) {
    ftb0_.clear_slot(base, nb_pc);
    ftb1_.clear_slot(base, nb_pc);
  }
};

}  // namespace bpuf

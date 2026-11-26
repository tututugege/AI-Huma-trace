#pragma once
// Block-based 1-Ahead FTB Branch Prediction Unit (EXP-050 Milestone A, ah1_4k_a2k1k + FTB_BRQ=2).
//
// Matches rtl/frontend/bpu/ cycle-for-cycle and bit-for-bit:
//   S0 (0 bubble)   1-ahead registered SRAM select (aftb 2048x8, atage 1024x4x4, aitt 256x3) with uFTB (256x4, 20b tgt)
//                   cold-start fallback; simultaneously launches the next-block 1-ahead SRAM read from (s0_base, s0_fh).
//   S1 (cold ovr)   Exact-PC SRAM read: Main FTB (4096x4, 4 slots/entry, no slot swapping) + S2 Block-TAGE (4096x4x4 + base)
//                   + Single Block-ITT (256x3). Triggers 1-bubble S1 override only on cold cycles (!r0.ah_hit).
//   S2 (fallback)   Exact-PC/exact-history Block-TAGE + Block-ITT + RAS final prediction -> PTAB write, ftb_rec_[512] write,
//                   2-bubble override on disagreement.
//   Commit side     16-entry 1-branch/cycle queue (brq[16], never backpressures ROB) + 1R1W ftb_rec_[512] -> single-branch RMW
//                   update to all 7 arrays.
#include "BPU.h"
#include "BpuSc.h"
#include <algorithm>
#include <cstring>
#include <vector>

namespace bpuf {

// Fixed geometry (matches rtl/frontend/bpu/bpu_types_pkg.sv)
constexpr int FTB_SLOTS     = 4;
constexpr int FTB_MAXW      = 8;
constexpr int FTB_S0_N      = 256;
constexpr int FTB_S0_W      = 4;
constexpr int FTB_S0_TAGB   = 12;
constexpr int FTB_S0_TGTB   = 20;
constexpr int FTB_S1_N      = 4096;
constexpr int FTB_S1_W      = 4;
constexpr int FTB_S1_TAGB   = 14;
constexpr int FTB_TAGE_N    = 4096;
constexpr int FTB_ITT_N     = 256;
constexpr int FTB_AH_N       = 2048;
constexpr int FTB_AH_W       = 8;
constexpr int FTB_AH_TAGE_N  = 1024;
constexpr int FTB_AH2_N      = 1024;
constexpr int FTB_AH2_W      = 8;
constexpr int FTB_AH2_TAGE_N = 512;
constexpr int FTB_BRQ_N      = 16;
constexpr int FTB_REC_N     = 512;
constexpr uint16_t FTB_TAGE_TAG_MASK = 0x3ffu;

struct FtbSlot {
  bool valid = false;
  uint8_t offset = 0;       // 4b: halfword slot offset (0..15) from block base
  bool carried = false;     // 1b: slot 0 is a 32-bit instruction carried from base - 2 (true PC = base - 2)
  bool is_rvc = false;      // 1b: 16-bit instruction (ilen = 2 else 4)
  uint8_t type = BR_NONCTL; // 3b: branch type
  uint8_t ctr = 2;          // 2b: saturating taken counter (0..3, taken >= 2)
  pc_t target = 0;          // target address
};

struct FtbEnt {
  bool valid = false;
  uint16_t tag = 0;         // up to 16b tag (normal FTB)
  uint32_t tag32 = 0;       // 22b combined (key_base, cur_base) tag (ahead FTB)
  pc_t b_base = 0;          // expected Block B base PC (used when AH2_PAR!=0)
  uint8_t lru = 0;          // replacement recency (0 = MRU, W-1 = LRU)
  FtbSlot s[FTB_SLOTS];
};

// Block-level index/tag bundle computed ONCE in S0 from (block_base, spec_fh).
struct FtbBlkIdx {
  uint32_t w = 0;
  uint32_t base_idx = 0;
  uint32_t idx[TN] = {};
  uint16_t tag[TN] = {};
};

// Prediction-time block metadata stored in 512-entry 1R1W ftb_rec_
struct FtbBlkRec {
  bool valid = false;
  uint16_t id = 0;
  pc_t base = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  uint16_t sc_fh[SC_GEHL_N] = {};
  uint8_t imli = 0;
  bool ah_v = false;
  uint8_t ah_kind = 1; // 1 = 1-ahead A (aftb_), 2 = 2-ahead B (aftb2_)
  pc_t ah_key_base = 0;
  pc_t ah_a_base = 0;
  uint32_t ah_key_fh[FH_N_MAX][TN] = {};
};

// Set-associative FTB table (used for S0 uFTB and S1 Main FTB)
struct FtbTab {
  int N = 0, W = 1, S = 1, set_bits = 0, tag_bits = 14, tgt_bits = 0;
  std::vector<FtbEnt> e;

  void init(int n, int w, int tb, int tgtb) {
    N = n; W = w; S = (n > 0 && w > 0) ? (n / w) : 1;
    set_bits = ceil_log2_u32(static_cast<uint32_t>(S));
    tag_bits = tb;
    tgt_bits = tgtb;
    e.assign(static_cast<size_t>(S) * W, FtbEnt{});
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

  bool lookup(pc_t base, FtbEnt &out) const {
    const uint32_t s = index(base);
    const uint16_t t = tag(base);
    for (int w = 0; w < W; w++) {
      const FtbEnt &en = e[s * W + w];
      if (en.valid && en.tag == t) {
        out = en;
        return true;
      }
    }
    return false;
  }

  void touch_lru(uint32_t s, int hit_w) {
    const uint8_t old_lru = e[s * W + hit_w].lru;
    for (int w = 0; w < W; w++) {
      if (w == hit_w) e[s * W + w].lru = 0;
      else if (e[s * W + w].valid && e[s * W + w].lru < old_lru) e[s * W + w].lru++;
    }
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
      return;
    }
  }
};

// Block-indexed 4-slot TAGE (used for S2 Block-TAGE and 1-Ahead Block-TAGE)
struct BlockTage {
  int N = FTB_TAGE_N;
  uint32_t mask = FTB_TAGE_N - 1;
  std::vector<uint8_t> base[FTB_SLOTS];
  std::vector<uint16_t> tag[TN][FTB_SLOTS];
  std::vector<uint8_t> cnt[TN][FTB_SLOTS];
  std::vector<uint8_t> useful[TN][FTB_SLOTS];
  uint8_t lsfr = 8;
  uint32_t reset_cnt = 0;
  uint8_t use_alt = static_cast<uint8_t>(TAGE_USE_ALT_CTR_INIT);

  struct RawSlot {
    uint8_t base_cnt = 0;
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

  void init(int entries) {
    N = entries;
    mask = static_cast<uint32_t>(N - 1);
    for (int sl = 0; sl < FTB_SLOTS; sl++) {
      base[sl].assign(N, 0);
      for (int t = 0; t < TN; t++) {
        tag[t][sl].assign(N, 0);
        cnt[t][sl].assign(N, 0);
        useful[t][sl].assign(N, 0);
      }
    }
    lsfr = 8;
    reset_cnt = 0;
    use_alt = static_cast<uint8_t>(TAGE_USE_ALT_CTR_INIT);
  }

  void read(const FtbBlkIdx &ix, Raw &r) const {
    const uint32_t bi = ix.base_idx & mask;
    for (int sl = 0; sl < FTB_SLOTS; sl++) {
      r.s[sl].base_cnt = base[sl][bi];
      for (int t = 0; t < TN; t++) {
        const uint32_t ti = ix.idx[t] & mask;
        r.s[sl].tag[t] = tag[t][sl][ti];
        r.s[sl].cnt[t] = cnt[t][sl][ti];
        r.s[sl].useful[t] = useful[t][sl][ti];
      }
    }
  }

  void select_slot(const FtbBlkIdx &ix, const RawSlot &r, Pred &o, int base_override = -1) const {
    int pcpn = TN, altpcpn = TN;
    for (int t = 0; t < TN; t++) {
      if (r.tag[t] == ix.tag[t]) {
        altpcpn = pcpn;
        pcpn = t;
      }
    }
    const bool bp = (base_override >= 0) ? (base_override >= 2) : (r.base_cnt >= 2);
    o.pcpn = static_cast<uint8_t>(pcpn);
    o.alt = (altpcpn == TN) ? bp : (r.cnt[altpcpn] >= 4);
    const bool pp = (pcpn == TN) ? bp : (r.cnt[pcpn] >= 4);
    const bool weak = (pcpn != TN) && (r.cnt[pcpn] == 3);
    const bool ulow = (pcpn != TN) && (r.useful[pcpn] == 0);
    o.used_alt = (pcpn != TN) && weak && ulow && (use_alt <= 8);
    o.pred = o.used_alt ? o.alt : pp;
  }

  void predict_slot(const FtbBlkIdx &ix, int sl, Pred &o, int base_override = -1) const {
    RawSlot r;
    const uint32_t bi = ix.base_idx & mask;
    r.base_cnt = base[sl][bi];
    for (int t = 0; t < TN; t++) {
      const uint32_t ti = ix.idx[t] & mask;
      r.tag[t] = tag[t][sl][ti];
      r.cnt[t] = cnt[t][sl][ti];
      r.useful[t] = useful[t][sl][ti];
    }
    select_slot(ix, r, o, base_override);
  }

  void update_slot(const FtbBlkIdx &ix, int sl, bool real_dir, bool pred_dir, bool alt_pred, int pcpn) {
    if (sl < 0 || sl >= FTB_SLOTS) return;
    const uint32_t next_rc = reset_cnt + 1;
    const bool do_rst = ((next_rc & ((1u << TageCfg::urst) - 1u)) == 0);
    auto set_u = [&](uint8_t &u, uint8_t nu) {
      // Single-write-port u_uhi SRAM: periodic reset takes priority on bit 1 on 1-in-2048 reset cycles
      u = do_rst ? static_cast<uint8_t>((u & 2u) | (nu & 1u)) : nu;
    };
    const uint32_t bi = ix.base_idx & mask;
    if (pcpn == TN) {
      uint8_t &bc = base[sl][bi];
      bc = real_dir ? (bc < 3 ? bc + 1 : 3) : (bc > 0 ? bc - 1 : 0);
    } else if (pcpn >= 0 && pcpn < TN) {
      const uint32_t ti = ix.idx[pcpn] & mask;
      uint8_t &c = cnt[pcpn][sl][ti];
      const bool pdir = c >= 4;
      const bool weak = (c == 3);
      if (weak && pdir != alt_pred) {
        constexpr uint8_t uamax = static_cast<uint8_t>((1u << TAGE_USE_ALT_CTR_BITS) - 1u);
        if (alt_pred == real_dir) { if (use_alt > 0) use_alt--; }
        else { if (use_alt < uamax) use_alt++; }
      }
      c = real_dir ? (c < 7 ? c + 1 : 7) : (c > 0 ? c - 1 : 0);
      if (pdir != alt_pred) {
        uint8_t &u = useful[pcpn][sl][ti];
        const uint8_t nu = (pdir == real_dir) ? (u < 3 ? uint8_t(u + 1) : 3) : (u > 0 ? uint8_t(u - 1) : 0);
        set_u(u, nu);
      }
    }

    if (pred_dir != real_dir) {
      int free_t[TN]; int nf = 0;
      const int start_t = (pcpn == TN) ? 0 : (pcpn + 1);
      for (int t = start_t; t < TN; t++) {
        if (useful[t][sl][ix.idx[t] & mask] == 0) free_t[nf++] = t;
      }
      if (nf > 0) {
        int chosen = free_t[0];
        if (nf > 1 && ((lsfr & 3u) == 0)) chosen = free_t[1];
        const uint32_t ti = ix.idx[chosen] & mask;
        tag[chosen][sl][ti] = ix.tag[chosen];
        cnt[chosen][sl][ti] = real_dir ? 4 : 3;
        set_u(useful[chosen][sl][ti], 0);
      } else {
        for (int t = start_t; t < TN; t++) {
          uint8_t &u = useful[t][sl][ix.idx[t] & mask];
          if (u > 0) set_u(u, uint8_t(u - 1));
        }
      }
    }

    const uint8_t fb = ((lsfr >> 3) ^ (lsfr >> 2)) & 1u;
    lsfr = static_cast<uint8_t>(((lsfr << 1) | fb) & 0xfu);
    if (lsfr == 0) lsfr = 1;
    reset_cnt = next_rc;
    if (do_rst) {
      const uint32_t row = (reset_cnt >> TageCfg::urst) & mask;
      for (int t = 0; t < TN; t++) useful[t][sl][row] &= 1u;
    }
  }
};

// Single Block-level 3-table Indirect Target Table
struct BlockItt {
  struct Ent {
    bool valid = false;
    uint16_t tag = 0;
    uint8_t conf = 0;
    uint8_t u = 0;
    target_addr_t target = 0;
  };
  struct Raw {
    Ent e[ITT_MAX];
  };

  int N = FTB_ITT_N;
  uint32_t mask = FTB_ITT_N - 1;
  std::vector<Ent> tbl[ITT_MAX];

  void init(int entries) {
    N = entries;
    mask = static_cast<uint32_t>(N - 1);
    for (int k = 0; k < ITT_MAX; k++) tbl[k].assign(N, Ent{});
  }

  void read(const FtbBlkIdx &ix, Raw &r) const {
    for (int k = 0; k < ITT_MAX; k++) {
      const int tk = IttCfg::h[k];
      r.e[k] = tbl[k][ix.idx[tk] & mask];
    }
  }

  bool select(const FtbBlkIdx &ix, const Raw &r, target_addr_t &tgt) const {
    for (int k = ITT_MAX - 1; k >= 0; k--) {
      const int tk = IttCfg::h[k];
      if (r.e[k].valid && r.e[k].tag == ix.tag[tk]) {
        tgt = r.e[k].target;
        return true;
      }
    }
    return false;
  }

  void update(const FtbBlkIdx &ix, target_addr_t actual_tgt) {
    int hit = -1;
    for (int k = ITT_MAX - 1; k >= 0; k--) {
      const int tk = IttCfg::h[k];
      const Ent &en = tbl[k][ix.idx[tk] & mask];
      if (en.valid && en.tag == ix.tag[tk]) { hit = k; break; }
    }
    bool correct = false;
    if (hit >= 0) {
      const int tk = IttCfg::h[hit];
      Ent &en = tbl[hit][ix.idx[tk] & mask];
      correct = (en.target == actual_tgt);
      if (correct) {
        if (en.conf < 3) en.conf++;
        en.u = 1;
      } else {
        if (en.conf > 0) en.conf--;
        if (en.conf == 0) { en.target = actual_tgt; en.conf = 1; }
        en.u = 0;
      }
    }
    if (!correct) {
      const int start_k = (hit >= 0) ? (hit + 1) : 0;
      int alloc = -1;
      for (int k = start_k; k < ITT_MAX; k++) {
        const int tk = IttCfg::h[k];
        const Ent &en = tbl[k][ix.idx[tk] & mask];
        if (!en.valid || en.u == 0) { alloc = k; break; }
      }
      if (alloc >= 0) {
        const int tk = IttCfg::h[alloc];
        Ent &en = tbl[alloc][ix.idx[tk] & mask];
        en.valid = true;
        en.tag = ix.tag[tk];
        en.target = actual_tgt;
        en.conf = 1;
        en.u = 0;
      } else {
        for (int k = start_k; k < ITT_MAX; k++) {
          const int tk = IttCfg::h[k];
          tbl[k][ix.idx[tk] & mask].u = 0;
        }
      }
    }
  }
};

// Set-associative 1-Ahead FTB table (indexed by key_base = pc_{t-1}, tagged by (key_base, cur_base))
struct AheadFtbTab {
  int N = FTB_AH_N, W = FTB_AH_W, S = FTB_AH_N / FTB_AH_W, set_bits = 8;
  std::vector<FtbEnt> e;

  struct SetRaw {
    FtbEnt ways[FTB_AH_W];
  };

  void init(int n, int w) {
    W = w; N = n; S = n / w;
    set_bits = ceil_log2_u32(static_cast<uint32_t>(S));
    e.assign(static_cast<size_t>(S) * W, FtbEnt{});
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

  bool select_way(const SetRaw &raw, pc_t key_base, pc_t cur_base, FtbEnt &out) const {
    const uint32_t t = tag(key_base, cur_base);
    for (int w = 0; w < W; w++) {
      if (raw.ways[w].valid && raw.ways[w].tag32 == t) {
        out = raw.ways[w];
        return true;
      }
    }
    return false;
  }

  void touch_lru(uint32_t s, int hit_w) {
    const uint8_t old_lru = e[s * W + hit_w].lru;
    for (int w = 0; w < W; w++) {
      if (w == hit_w) e[s * W + w].lru = 0;
      else if (e[s * W + w].valid && e[s * W + w].lru < old_lru) e[s * W + w].lru++;
    }
  }
};

struct FtbAhRaw {
  AheadFtbTab::SetRaw ftb_set{};
  BlockTage::Raw tage_raw{};
  BlockItt::Raw itt_raw{};
};

struct FtbAhPipe {
  bool valid = false;
  pc_t key_base = 0;
  uint32_t key_fh[FH_N_MAX][TN] = {};
  FtbAhRaw raw{};
};

struct FtbRaw {
  bool ftb_hit = false;
  FtbEnt ent{};
  ExBlockTage::Raw s2_tage_raw{};
  StatCorrector::Raw s2_sc_raw{};
  BlockItt::Raw itt_raw{};
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
  uint32_t fh[FH_N_MAX][TN] = {};
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  uint16_t sc_fh[SC_GEHL_N] = {};
  uint8_t imli = 0;
  bool ah_v = false;
  uint8_t ah_kind = 1; // 1 = 1-ahead A (aftb_), 2 = 2-ahead B (aftb2_)
  pc_t ah_key_base = 0;
  pc_t ah_a_base = 0;
  uint32_t ah_key_fh[FH_N_MAX][TN] = {};
  uint8_t nbr = 0;
  FtbTrkBr br[FW];
};

// 1-branch/cycle commit queue entry (matches rtl/frontend/bpu/bpu_ftb_brq.sv)
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
  // FastWarm block tracker state (used only by warm_commit)
  bool v = false;
  pc_t start = 0;
  pc_t expect = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  uint16_t sc_fh[SC_GEHL_N] = {};
  uint8_t imli = 0;
  uint8_t nbr = 0;
  FtbTrkBr br[FW];
  // Commit-side 1-branch/cycle queue
  FtbBrqEnt brq[FTB_BRQ_N];
  uint8_t brq_h = 0, brq_n = 0;
};

struct BpuFtb {
  FtbTab ftb0_;         // S0 uFTB (256 x 4, 20b compressed target)
  FtbTab ftb1_;         // S1/S2 Main FTB (4096 x 4, full target)
  ExBlockTage s2_tage_; // S2 8-table Block-TAGE (4096 x 4 slots x 8 tables + base)
  StatCorrector sc_;    // S2 Statistical Corrector (Bias 64x4 + 4 GEHL + IMLI + Local)
  BlockItt itt_;        // S2 Single Block-ITT (256 x 3 tables)
  AheadFtbTab aftb_;    // 1-Ahead FTB A (2048 x 8)
  BlockTage atage_;     // 1-Ahead Block-TAGE A (1024 x 4 slots x 4 tables, reuses aftb ctr as base)
  BlockItt aitt_;       // 1-Ahead Block-ITT A (256 x 3 tables)
  AheadFtbTab aftb2_;   // 2-Ahead FTB B (1024 x 8)
  BlockTage atage2_;    // 2-Ahead Block-TAGE B (512 x 4 slots x 4 tables)
  BlockItt aitt2_;      // 2-Ahead Block-ITT B (256 x 3 tables)

  void reset() {
    ftb0_.init(FTB_S0_N, FTB_S0_W, FTB_S0_TAGB, FTB_S0_TGTB);
    ftb1_.init(FTB_S1_N, FTB_S1_W, FTB_S1_TAGB, 0);
    s2_tage_.init();
    sc_.init();
    itt_.init(FTB_ITT_N);
    aftb_.init(FTB_AH_N, FTB_AH_W);
    atage_.init(FTB_AH_TAGE_N);
    aitt_.init(FTB_ITT_N);
    aftb2_.init(FTB_AH2_N, FTB_AH2_W);
    atage2_.init(FTB_AH2_TAGE_N);
    aitt2_.init(FTB_ITT_N);
  }

  static FtbBlkIdx make_index(pc_t base, const uint32_t fh[FH_N_MAX][TN]) {
    FtbBlkIdx ix;
    const uint32_t w = static_cast<uint32_t>(base >> SLOT_SHIFT);
    ix.w = w;
    ix.base_idx = w ^ (w >> 11);
    for (int t = 0; t < TN; t++) {
      ix.idx[t] = (w ^ (w >> 10) ^ fh[0][t] ^ ((fh[1][t] & 0x7u) << 9));
      ix.tag[t] = static_cast<uint16_t>((w ^ (w >> 10) ^ fh[1][t] ^ (fh[2][t] << 2)) & FTB_TAGE_TAG_MASK);
    }
    return ix;
  }

  static FtbBlkIdx make_ahead_index(pc_t key_base, pc_t cur_base, const uint32_t fh[FH_N_MAX][TN]) {
    FtbBlkIdx ix;
    const uint32_t wk = static_cast<uint32_t>(key_base >> SLOT_SHIFT);
    const uint32_t wc = static_cast<uint32_t>(cur_base >> SLOT_SHIFT);
    ix.w = wk;
    ix.base_idx = wk ^ (wk >> 11);
    for (int t = 0; t < TN; t++) {
      ix.idx[t] = (wk ^ (wk >> 10) ^ fh[0][t] ^ ((fh[1][t] & 0x7u) << 9));
      ix.tag[t] = static_cast<uint16_t>((wc ^ (wc >> 10) ^ wk ^ fh[1][t] ^ (fh[2][t] << 2)) & FTB_TAGE_TAG_MASK);
    }
    return ix;
  }

  void ahead_read(pc_t key_base, const uint32_t key_fh[FH_N_MAX][TN], FtbAhRaw &r) const {
    const FtbBlkIdx ix = make_ahead_index(key_base, 0, key_fh);
    aftb_.read_set(key_base, r.ftb_set);
    atage_.read(ix, r.tage_raw);
    aitt_.read(ix, r.itt_raw);
  }

  void ahead2_read(pc_t key_base, const uint32_t key_fh[FH_N_MAX][TN], FtbAhRaw &r) const {
    const FtbBlkIdx ix = make_ahead_index(key_base, 0, key_fh);
    aftb2_.read_set(key_base, r.ftb_set);
    atage2_.read(ix, r.tage_raw);
    aitt2_.read(ix, r.itt_raw);
  }

  template <class S0GuessT>
  bool ahead_select_impl(const AheadFtbTab &aftb, const BlockTage &atage, const BlockItt &aitt,
                         pc_t base, pc_t bnd, bool ah_valid, pc_t key_base,
                         const uint32_t key_fh[FH_N_MAX][TN], const FtbAhRaw &raw,
                         uint8_t rcnt, target_addr_t rtop, S0GuessT &g) const {
    if (!ah_valid) return false;
    FtbEnt ent;
    if (!aftb.select_way(raw.ftb_set, key_base, base, ent)) return false;

    const FtbBlkIdx ix = make_ahead_index(key_base, base, key_fh);
    bool dirs[FTB_SLOTS] = {};
    BlockTage::Pred tp[FTB_SLOTS];
    for (int k = 0; k < FTB_SLOTS; k++) {
      atage.select_slot(ix, raw.tage_raw.s[k], tp[k], ent.s[k].valid ? int(ent.s[k].ctr) : -1);
      dirs[k] = tp[k].pred;
    }
    target_addr_t itt_tgt = 0;
    const bool itt_hit = aitt.select(ix, raw.itt_raw, itt_tgt);

    g = S0GuessT{};
    g.hit = true;
    eval_ent(base, bnd, ent, true, dirs, itt_hit, itt_tgt, rcnt, rtop, g);
    return true;
  }

  template <class S0GuessT>
  bool ahead_select(pc_t base, pc_t bnd, const FtbAhPipe &ah, uint8_t rcnt, target_addr_t rtop, S0GuessT &g) const {
    return ahead_select_impl(aftb_, atage_, aitt_, base, bnd, ah.valid, ah.key_base, ah.key_fh, ah.raw, rcnt, rtop, g);
  }

  template <class S0GuessT>
  bool ahead2_select(pc_t base, pc_t bnd, bool valid, pc_t key_base, const uint32_t key_fh[FH_N_MAX][TN],
                     const FtbAhRaw &raw, uint8_t rcnt, target_addr_t rtop, S0GuessT &g, pc_t a_base = 0) const {
    if (!valid) return false;
    FtbEnt ent;
    // Keyed by (key_base, a_base) so ah2 runs 100% in parallel with Block A; verify ent.b_base == base at the end.
    if (!aftb2_.select_way(raw.ftb_set, key_base, a_base, ent)) return false;
    if (ent.b_base != base) return false;
    const FtbBlkIdx ix = make_ahead_index(key_base, a_base, key_fh);
    bool dirs[FTB_SLOTS] = {};
    BlockTage::Pred tp[FTB_SLOTS];
    for (int k = 0; k < FTB_SLOTS; k++) {
      atage2_.select_slot(ix, raw.tage_raw.s[k], tp[k], ent.s[k].valid ? int(ent.s[k].ctr) : -1);
      dirs[k] = tp[k].pred;
    }
    target_addr_t itt_tgt = 0;
    const bool itt_hit = aitt2_.select(ix, raw.itt_raw, itt_tgt);
    g = S0GuessT{};
    g.hit = true;
    eval_ent(base, bnd, ent, true, dirs, itt_hit, itt_tgt, rcnt, rtop, g);
    return true;
  }

  template <class GuessT>
  static void eval_ent(pc_t base, pc_t bnd, const FtbEnt &ent,
                       bool use_dir_ovr, const bool dir_ovr[FTB_SLOTS],
                       bool itt_hit, target_addr_t itt_tgt,
                       uint8_t rcnt, target_addr_t rtop,
                       GuessT &g, bool *slot_dir_out = nullptr) {
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
  S0GuessT s0_lookup(pc_t base, pc_t bnd, uint8_t rcnt, target_addr_t rtop) const {
    S0GuessT g{};
    g.next = bnd;
    FtbEnt ent;
    if (!ftb0_.lookup(base, ent)) return g;
    g.hit = true;
    const bool dummy[FTB_SLOTS] = {};
    eval_ent(base, bnd, ent, false, dummy, false, 0, rcnt, rtop, g);
    return g;
  }

  void s1_read(pc_t base, const FtbBlkIdx &ix, const ExTageIdx &ex_ix, const ScIdx &sc_ix, FtbRaw &r) const {
    r = FtbRaw{};
    r.ftb_hit = ftb1_.lookup(base, r.ent);
    s2_tage_.read(ex_ix, r.s2_tage_raw);
    sc_.read(sc_ix, r.s2_sc_raw);
    itt_.read(ix, r.itt_raw);
  }

  template <class S0GuessT>
  S0GuessT s1_predict(pc_t base, pc_t bnd, const FtbRaw &r, uint8_t rcnt, target_addr_t rtop) const {
    S0GuessT g{};
    g.next = bnd;
    if (!r.ftb_hit) return g;
    g.hit = true;
    const bool dummy[FTB_SLOTS] = {};
    eval_ent(base, bnd, r.ent, false, dummy, false, 0, rcnt, rtop, g);
    return g;
  }

  template <class MainOutT>
  void s2_select(pc_t base, pc_t bnd, const FtbBlkIdx &ix, const ExTageIdx &ex_ix, const FtbRaw &r,
                 uint8_t rcnt, target_addr_t rtop, MainOutT &o) const {
    o = MainOutT{};
    o.next = bnd;
    for (int i = 0; i < FW; i++) {
      o.meta.pcpn[i] = TN;
      o.meta.alt_pred[i] = false;
    }
    if (!r.ftb_hit) return;

    bool dirs[FTB_SLOTS] = {};
    ExBlockTage::Pred tp[FTB_SLOTS];
    for (int k = 0; k < FTB_SLOTS; k++) {
      s2_tage_.select_slot(ex_ix, r.s2_tage_raw.s[k], tp[k]);
      dirs[k] = sc_.select_slot(ex_ix.w, r.s2_sc_raw.s[k], tp[k].pred, tp[k].pcpn < EX_TN, tp[k].prov_cnt, tp[k].base_cnt);
    }
    target_addr_t itt_tgt = 0;
    const bool itt_hit = itt_.select(ix, r.itt_raw, itt_tgt);

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

  static int upsert_branch(FtbEnt &ent, const FtbTrkBr &b) {
    for (int k = 0; k < FTB_SLOTS; k++) {
      if (ent.s[k].valid && ent.s[k].offset == b.offset) {
        ent.s[k].type = b.type;
        ent.s[k].carried = b.carried;
        ent.s[k].is_rvc = b.is_rvc;
        if (b.actual_dir) ent.s[k].target = b.target;
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
    ns.target = b.target;
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
    for (int w = 0; w < tab.W; w++) {
      if (tab.e[s * tab.W + w].valid && tab.e[s * tab.W + w].tag == t) { hit = w; break; }
    }
    if (hit < 0) {
      if (alloc_only_taken && !has_taken) return nullptr;
      int vic = -1;
      for (int w = 0; w < tab.W; w++) if (!tab.e[s * tab.W + w].valid) { vic = w; break; }
      if (vic < 0) {
        vic = 0;
        for (int w = 1; w < tab.W; w++) if (tab.e[s * tab.W + w].lru > tab.e[s * tab.W + vic].lru) vic = w;
      }
      hit = vic;
      FtbEnt &en = tab.e[s * tab.W + hit];
      en = FtbEnt{};
      en.valid = true;
      en.tag = t;
      en.lru = static_cast<uint8_t>(tab.W - 1);
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

  void apply_train_ev(const FtbTrainEv &ev) {
    FtbEnt *main_ent = update_tab(ftb1_, ev, false);
    update_tab(ftb0_, ev, true);
    if (!main_ent) return;

    const FtbBlkIdx ix = make_index(ev.start, ev.fh);
    const uint32_t w = static_cast<uint32_t>(ev.start >> SLOT_SHIFT);
    const ExTageIdx ex_ix = make_ex_tage_index(w, ev.ex_fh);
    const ScIdx sc_ix = make_sc_index(w, ev.sc_fh, ev.imli);
    for (int i = 0; i < ev.nbr; i++) {
      const FtbTrkBr &b = ev.br[i];
      if (b.type == BR_DIRECT) {
        int sl = -1;
        for (int k = 0; k < FTB_SLOTS; k++) {
          if (main_ent->s[k].valid && main_ent->s[k].offset == b.offset) { sl = k; break; }
        }
        if (sl >= 0) {
          ExBlockTage::Pred ep;
          s2_tage_.update_slot(ex_ix, sl, b.actual_dir, &ep);
          sc_.update(sc_ix, sl, b.actual_dir, ep.pred, ep.pcpn < EX_TN, ep.prov_cnt, ep.base_cnt);
        }
      }
      if (br_ind(b.type) && b.actual_dir) itt_.update(ix, b.target);
    }

    if (ev.ah_v && ev.nbr > 0) {
      const bool use2 = (ev.ah_kind == 2);
      const pc_t cur_key = use2 ? ev.ah_a_base : ev.start;
      AheadFtbTab &atab = use2 ? aftb2_ : aftb_;
      BlockTage &atg = use2 ? atage2_ : atage_;
      BlockItt &ait = use2 ? aitt2_ : aitt_;
      const uint32_t s = atab.index(ev.ah_key_base);
      const uint32_t t = atab.tag(ev.ah_key_base, cur_key);
      int hit = -1;
      for (int w = 0; w < atab.W; w++) {
        if (atab.e[s * atab.W + w].valid && atab.e[s * atab.W + w].tag32 == t) { hit = w; break; }
      }
      if (hit < 0) {
        int vic = -1;
        for (int w = 0; w < atab.W; w++) if (!atab.e[s * atab.W + w].valid) { vic = w; break; }
        if (vic < 0) {
          vic = 0;
          for (int w = 1; w < atab.W; w++) if (atab.e[s * atab.W + w].lru > atab.e[s * atab.W + vic].lru) vic = w;
        }
        hit = vic;
        FtbEnt &en = atab.e[s * atab.W + hit];
        en = FtbEnt{};
        en.valid = true;
        en.tag32 = t;
        en.b_base = ev.start;
        en.lru = static_cast<uint8_t>(atab.W - 1);
      } else if (use2 && atab.e[s * atab.W + hit].b_base != ev.start) {
        FtbEnt &en = atab.e[s * atab.W + hit];
        const uint8_t keep_lru = en.lru;
        en = FtbEnt{};
        en.valid = true;
        en.tag32 = t;
        en.b_base = ev.start;
        en.lru = keep_lru;
      }
      FtbEnt &aen = atab.e[s * atab.W + hit];
      for (int i = 0; i < ev.nbr; i++) upsert_branch(aen, ev.br[i]);
      atab.touch_lru(s, hit);

      const FtbBlkIdx aix = make_ahead_index(ev.ah_key_base, cur_key, ev.ah_key_fh);
      for (int i = 0; i < ev.nbr; i++) {
        const FtbTrkBr &b = ev.br[i];
        if (b.type == BR_DIRECT) {
          int sl = -1;
          for (int k = 0; k < FTB_SLOTS; k++) {
            if (aen.s[k].valid && aen.s[k].offset == b.offset) { sl = k; break; }
          }
          if (sl >= 0) {
            BlockTage::Pred ap;
            atg.predict_slot(aix, sl, ap, int(aen.s[sl].ctr));
            atg.update_slot(aix, sl, b.actual_dir, ap.pred, ap.alt, ap.pcpn);
          }
        }
        if (br_ind(b.type) && b.actual_dir) ait.update(aix, b.target);
      }
    }
  }

  void clear_false_branch(pc_t base, pc_t nb_pc) {
    ftb0_.clear_slot(base, nb_pc);
    ftb1_.clear_slot(base, nb_pc);
  }
};

}  // namespace bpuf

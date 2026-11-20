#pragma once
// Block-based FTB Branch Prediction Unit (Phase 1 of docs/BPU_OPT_PLAN.md, EXP-050).
//
// Replaces the 16-bank per-slot Type + BTB + TAGE + ITT arrays (~325 KB) with a compact block-based hierarchy (~80 KB):
//   S0 (0 bubble)   uFTB (256 x 4, optional 20-bit compressed targets) -> fast guess g0, next PC, and one block-level index
//   S1 (main pred)  Main FTB (2K/4K x 4, up to 2 branch slots per block) + 2-slot fast history direction table (mini-TAGE)
//                   -> S1 prediction g1, S1 override on disagreement (1 bubble, or 0 bubble with FTB_OVR_FAST bit 1)
//   S2 (fallback)   2-slot Block-TAGE (4 tables x 2048 x 2) + single Block-ITT (3 tables x 256) + RAS
//                   -> final prediction fin, S2 override only when long-history TAGE or ITT disagrees with S1
//   Commit side     Commit-order block tracker -> block train queue -> updates uFTB, Main FTB, S1 mini-TAGE, S2 Block-TAGE, Block-ITT.
//
// All storage, indices, tags, counters and replacement policies are fixed-width and hardware-faithful.
#include "BPU.h"
#include "env.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace bpuf {

constexpr int FTB_SLOTS = 4;    // max branch slots stored per fetch block (runtime FTB_SLOTS in [2..4], NO slot swapping)
constexpr int FTB_MAXW  = 8;    // max set-associativity supported by FtbTab
constexpr int FTB_TQ_N  = 16;   // block-level commit train queue depth
constexpr uint16_t FTB_TAGE_TAG_MASK = 0x3ffu;  // 10-bit TAGE tag (4x fewer false tag collisions than 8-bit)

inline int ftb_active_slots() {
  const int s = sim_env().ftb_slots;
  return s < 2 ? 2 : (s > FTB_SLOTS ? FTB_SLOTS : s);
}

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
  uint8_t lru = 0;          // replacement recency (0 = MRU, W-1 = LRU)
  FtbSlot s[FTB_SLOTS];
};

// Block-level index/tag bundle computed ONCE in S0 from (block_base, spec_fh).
struct FtbBlkIdx {
  uint32_t w = 0;                 // low 32b of (base >> SLOT_SHIFT)
  uint32_t base_idx = 0;          // base table index (full width, masked per table)
  uint32_t idx[TN] = {};          // folded-history XOR pc index (full width, masked per table)
  uint16_t tag[TN] = {};          // 10-bit tag per TAGE table
};

// Exact prediction-time block record (shadow of FTQ per-block metadata for training)
struct FtbBlkRec {
  bool valid = false;
  uint16_t id = 0;
  pc_t base = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  // Phase 2 (FTB_AHEAD): exact prediction-time predecessor key & history
  bool ah_v = false;
  pc_t ah_key_base = 0;
  uint32_t ah_key_fh[FH_N_MAX][TN] = {};
};

// ---------------------------------------------------------------------------
// Set-associative FTB table (used for both S0 uFTB and S1 Main FTB)
// ---------------------------------------------------------------------------
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
    if (N <= 0) return false;
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

  // Invalidate a false branch slot (reported by predecode checker).
  void clear_slot(pc_t base, pc_t br_pc) {
    const bool carried = (br_pc + SLOT_BYTES == base);
    if (N <= 0 || (!carried && br_pc < base)) return;
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

// ---------------------------------------------------------------------------
// Block-indexed 2-slot TAGE (replaces 16-bank per-slot TAGE; also used for S1 mini-TAGE)
// ---------------------------------------------------------------------------
struct BlockTage {
  int N = 2048;
  uint32_t mask = 2047;
  uint32_t active_mask = 0xf;   // bitmask over TN tables (e.g. 0x3 for 2-table S1 mini-TAGE, 0xf for 4-table)
  std::vector<uint8_t> base[FTB_SLOTS];
  std::vector<uint16_t> tag[TN][FTB_SLOTS];
  std::vector<uint8_t> cnt[TN][FTB_SLOTS];
  std::vector<uint8_t> useful[TN][FTB_SLOTS];
  uint8_t lsfr = 8;
  uint32_t reset_cnt = 0;
  uint8_t use_alt[64] = {};

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

  void init(int entries, uint32_t amask = 0xf) {
    N = entries > 0 ? entries : 256;
    mask = static_cast<uint32_t>(N - 1);
    active_mask = amask;
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
    for (auto &u : use_alt) u = static_cast<uint8_t>(TAGE_USE_ALT_CTR_INIT);
  }

  void read(const FtbBlkIdx &ix, Raw &r) const {
    const uint32_t bi = ix.base_idx & mask;
    for (int sl = 0; sl < FTB_SLOTS; sl++) {
      r.s[sl].base_cnt = base[sl][bi];
      for (int t = 0; t < TN; t++) {
        if (!((active_mask >> t) & 1u)) continue;
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
      if (!((active_mask >> t) & 1u)) continue;
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
    const bool prefer_alt = use_alt[ix.base_idx & (tcfg().ualt - 1)] <= 8;
    o.used_alt = (pcpn != TN) && weak && ulow && prefer_alt;
    o.pred = o.used_alt ? o.alt : pp;
  }

  void predict_slot(const FtbBlkIdx &ix, int sl, Pred &o, int base_override = -1) const {
    RawSlot r;
    const uint32_t bi = ix.base_idx & mask;
    r.base_cnt = base[sl][bi];
    for (int t = 0; t < TN; t++) {
      if (!((active_mask >> t) & 1u)) continue;
      const uint32_t ti = ix.idx[t] & mask;
      r.tag[t] = tag[t][sl][ti];
      r.cnt[t] = cnt[t][sl][ti];
      r.useful[t] = useful[t][sl][ti];
    }
    select_slot(ix, r, o, base_override);
  }

  void update_slot(const FtbBlkIdx &ix, int sl, bool real_dir, bool pred_dir, bool alt_pred, int pcpn) {
    if (sl < 0 || sl >= FTB_SLOTS) return;
    const uint32_t bi = ix.base_idx & mask;
    if (pcpn == TN) {
      uint8_t &bc = base[sl][bi];
      bc = real_dir ? (bc < 3 ? bc + 1 : 3) : (bc > 0 ? bc - 1 : 0);
    } else if (pcpn >= 0 && pcpn < TN && ((active_mask >> pcpn) & 1u)) {
      const uint32_t ti = ix.idx[pcpn] & mask;
      uint8_t &c = cnt[pcpn][sl][ti];
      const bool pdir = c >= 4;
      const bool weak = (c == 3);
      if (weak && pdir != alt_pred) {
        uint8_t &ua = use_alt[bi & (tcfg().ualt - 1)];
        const uint8_t uamax = static_cast<uint8_t>((1u << TAGE_USE_ALT_CTR_BITS) - 1u);
        if (alt_pred == real_dir) { if (ua > 0) ua--; }
        else { if (ua < uamax) ua++; }
      }
      c = real_dir ? (c < 7 ? c + 1 : 7) : (c > 0 ? c - 1 : 0);
      if (pdir != alt_pred) {
        uint8_t &u = useful[pcpn][sl][ti];
        if (pdir == real_dir) { if (u < 3) u++; }
        else { if (u > 0) u--; }
      }
    }

    // Allocate on misprediction
    if (pred_dir != real_dir) {
      int free_t[TN]; int nf = 0;
      const int start_t = (pcpn == TN) ? 0 : (pcpn + 1);
      for (int t = start_t; t < TN; t++) {
        if (!((active_mask >> t) & 1u)) continue;
        if (useful[t][sl][ix.idx[t] & mask] == 0) free_t[nf++] = t;
      }
      if (nf > 0) {
        int chosen = free_t[0];
        if (nf > 1 && ((lsfr & 3u) == 0)) chosen = free_t[1];
        const uint32_t ti = ix.idx[chosen] & mask;
        tag[chosen][sl][ti] = ix.tag[chosen];
        cnt[chosen][sl][ti] = real_dir ? 4 : 3;
        useful[chosen][sl][ti] = 0;
      } else {
        for (int t = start_t; t < TN; t++) {
          if (!((active_mask >> t) & 1u)) continue;
          uint8_t &u = useful[t][sl][ix.idx[t] & mask];
          if (u > 0) u--;
        }
      }
    }

    // LFSR & periodic useful reset
    const uint8_t fb = ((lsfr >> 3) ^ (lsfr >> 2)) & 1u;
    lsfr = static_cast<uint8_t>(((lsfr << 1) | fb) & 0xfu);
    if (lsfr == 0) lsfr = 1;
    reset_cnt++;
    if ((reset_cnt & ((1u << TageCfg::urst) - 1u)) == 0) {
      const uint32_t row = (reset_cnt >> TageCfg::urst) & mask;
      for (int t = 0; t < TN; t++) {
        if (!((active_mask >> t) & 1u)) continue;
        useful[t][sl][row] &= 1u;
      }
    }
  }
};

// ---------------------------------------------------------------------------
// Single Block-level Indirect Target Table (replaces 16-bank ITT)
// ---------------------------------------------------------------------------
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

  int N = 256;
  uint32_t mask = 255;
  std::vector<Ent> tbl[ITT_MAX];

  void init(int entries) {
    N = entries > 0 ? entries : 256;
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

// ---------------------------------------------------------------------------
// Set-associative Ahead FTB table (Phase 2: indexed by key_base = pc_{t-k}, tagged by (key_base, cur_base))
// ---------------------------------------------------------------------------
struct AheadFtbTab {
  int N = 0, W = 8, S = 1, set_bits = 0;
  std::vector<FtbEnt> e;

  struct SetRaw {
    int W = 0;
    FtbEnt ways[FTB_MAXW];
  };

  void init(int n, int w) {
    W = w < 1 ? 1 : (w > FTB_MAXW ? FTB_MAXW : w);
    N = n; S = (n > 0) ? (n / W) : 1;
    if (S < 1) S = 1;
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
    const uint32_t tk = (wk >> set_bits) & 0x3ffu;   // 10b key_base tag
    const uint32_t tc = (wc ^ (wc >> 12)) & 0xfffu;  // 12b cur_base folded tag
    return (tk << 12) | tc;
  }

  // Stage t-k SRAM read: uses ONLY key_base!
  void read_set(pc_t key_base, SetRaw &raw) const {
    raw.W = W;
    if (N <= 0) { raw.W = 0; return; }
    const uint32_t s = index(key_base);
    for (int w = 0; w < W; w++) raw.ways[w] = e[s * W + w];
  }

  // Stage t (S0) tag compare: uses ONLY latched SetRaw + (key_base, cur_base)!
  bool select_way(const SetRaw &raw, pc_t key_base, pc_t cur_base, FtbEnt &out) const {
    const uint32_t t = tag(key_base, cur_base);
    for (int w = 0; w < raw.W; w++) {
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

// Latched ahead SRAM output (1-cycle or 2-cycle pipelined before S0)
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

// ---------------------------------------------------------------------------
// S1 -> S2 registered raw bundle for Block-based FTB BPU (much smaller than 16-bank MainRaw)
// ---------------------------------------------------------------------------
struct FtbRaw {
  bool ftb_hit = false;
  FtbEnt ent{};
  BlockTage::Raw s1_dir_raw{};
  BlockTage::Raw s2_tage_raw{};
  BlockItt::Raw itt_raw{};
};

// One committed branch within a tracked fetch block
struct FtbTrkBr {
  uint8_t offset = 0;
  bool carried = false;
  bool is_rvc = false;
  uint8_t type = BR_NONCTL;
  bool actual_dir = false;
  bool pred_dir = false;
  bool alt_pred = false;
  uint8_t pcpn = TN;
  tage_tag_t pred_tag0 = 0;
  pc_t target = 0;
};

// Block-level commit training event
struct FtbTrainEv {
  pc_t start = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  bool ah_v = false;
  pc_t ah_key_base = 0;
  uint32_t ah_key_fh[FH_N_MAX][TN] = {};
  uint8_t nbr = 0;
  FtbTrkBr br[FW];
  bool end_taken = false;
  bool end_mispred_nt = false;
};

// RTL-style 1-branch commit queue entry (FTB_BRQ=1: zero block-tracker FSM, 1R1W ftb_rec_[512], 1-branch/cycle RMW)
struct FtbBrqEnt {
  uint16_t blk_id = 0;
  pc_t pc = 0;
  bool is_rvc = false;
  uint8_t type = BR_NONCTL;
  bool actual_dir = false;
  bool pred_dir = false;
  bool alt_pred = false;
  uint8_t pcpn = TN;
  tage_tag_t pred_tag0 = 0;
  pc_t target = 0;
};

// Commit-order block tracker state (kept inside BpuPipe::State)
struct FtbTrkState {
  bool v = false;
  uint16_t cur_blk = 0;
  pc_t start = 0;
  pc_t expect = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  bool ah_v = false;
  pc_t ah_key_base = 0;
  uint32_t ah_key_fh[FH_N_MAX][TN] = {};
  uint8_t nbr = 0;
  FtbTrkBr br[FW];
  FtbTrainEv tq[FTB_TQ_N];
  uint8_t tq_h = 0, tq_n = 0;
  FtbBrqEnt brq[FTB_TQ_N];
  uint8_t brq_h = 0, brq_n = 0;
};

// ---------------------------------------------------------------------------
// Complete Block-based FTB BPU subsystem (owned by BpuPipe when BPU_FTB=1)
// ---------------------------------------------------------------------------
struct FtbStats {
  uint64_t s0_lookups = 0, s0_hits = 0;
  uint64_t ah_lookups = 0, ah_hits = 0, ah_cold = 0;
  uint64_t s1_lookups = 0, s1_hits = 0, s1_ovr = 0, s1_ovr_cap = 0, s1_ovr_dir = 0;
  uint64_t s2_lookups = 0, s2_ovr = 0, s2_ovr_dir = 0, s2_ovr_itt = 0, s2_ovr_miss = 0;
  uint64_t train_blocks = 0, train_drops = 0, slot_evicts = 0, pd_clears = 0;
  uint64_t br_upd = 0, tag_mismatch = 0, pcpn_tn = 0;
};

struct BpuFtb {
  FtbTab ftb0_;       // S0 uFTB
  FtbTab ftb1_;       // S1 Main FTB
  BlockTage s1_tage_; // S1 fast direction predictor (mini-TAGE)
  BlockTage s2_tage_; // S2 main Block-TAGE
  BlockItt itt_;      // S2 single Block-ITT
  // Phase 2: Ahead-pipelined FTB + Block-TAGE + Block-ITT (used when FTB_AHEAD > 0)
  AheadFtbTab aftb_;
  BlockTage atage_;
  BlockItt aitt_;
  mutable FtbStats stats_{};

  ~BpuFtb() {
    if (sim_env().bpu_ftb && sim_env().ftb_stats && stats_.s0_lookups > 0) {
      std::fprintf(stderr,
        "[FTB_STATS] s0=%llu hit=%llu(%.1f%%) ah=%llu/%llu(%.1f%% cold=%llu) | s1=%llu hit=%llu(%.1f%%) ovr=%llu(cap=%llu dir=%llu) | "
        "s2=%llu ovr=%llu(dir=%llu itt=%llu miss=%llu) | train=%llu drop=%llu evict3=%llu pdclr=%llu | br_upd=%llu tag_mm=%llu pcpn_tn=%llu\n",
        (unsigned long long)stats_.s0_lookups, (unsigned long long)stats_.s0_hits,
        100.0 * stats_.s0_hits / std::max<uint64_t>(1, stats_.s0_lookups),
        (unsigned long long)stats_.ah_hits, (unsigned long long)stats_.ah_lookups,
        100.0 * stats_.ah_hits / std::max<uint64_t>(1, stats_.ah_lookups),
        (unsigned long long)stats_.ah_cold,
        (unsigned long long)stats_.s1_lookups, (unsigned long long)stats_.s1_hits,
        100.0 * stats_.s1_hits / std::max<uint64_t>(1, stats_.s1_lookups),
        (unsigned long long)stats_.s1_ovr, (unsigned long long)stats_.s1_ovr_cap, (unsigned long long)stats_.s1_ovr_dir,
        (unsigned long long)stats_.s2_lookups, (unsigned long long)stats_.s2_ovr,
        (unsigned long long)stats_.s2_ovr_dir, (unsigned long long)stats_.s2_ovr_itt, (unsigned long long)stats_.s2_ovr_miss,
        (unsigned long long)stats_.train_blocks, (unsigned long long)stats_.train_drops,
        (unsigned long long)stats_.slot_evicts, (unsigned long long)stats_.pd_clears,
        (unsigned long long)stats_.br_upd, (unsigned long long)stats_.tag_mismatch, (unsigned long long)stats_.pcpn_tn);
    }
  }

  void reset() {
    const SimEnv &env = sim_env();
    ftb0_.init(env.ftb_s0_n, 4, 12, env.ftb_s0_tgtb);
    ftb1_.init(env.ftb_s1_n, env.ftb_s1_w, 14, 0);
    const uint32_t s1_mask = (env.ftb_s1_dir == 1) ? 0x3u : 0xfu;
    s1_tage_.init(env.ftb_s1_dir_n, s1_mask);
    s2_tage_.init(env.ftb_tage_n, 0xfu);
    itt_.init(env.ftb_itt_n);
    if (env.ftb_ahead > 0) {
      aftb_.init(env.ftb_ah_n, env.ftb_ah_w);
      atage_.init(env.ftb_ah_tage_n, 0xfu);
      aitt_.init(env.ftb_itt_n);
    }
    stats_ = FtbStats{};
  }

  // Compute block-level index & 10-bit tags ONCE per fetch block (in S0).
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

  // Phase 2: Ahead index depends ONLY on (key_base, key_fh); tags depend on (cur_base, key_base, key_fh).
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

  // Stage t-k ahead SRAM read: uses ONLY registered (key_base, key_fh)!
  void ahead_read(pc_t key_base, const uint32_t key_fh[FH_N_MAX][TN], FtbAhRaw &r) const {
    const FtbBlkIdx ix = make_ahead_index(key_base, 0, key_fh);
    aftb_.read_set(key_base, r.ftb_set);
    atage_.read(ix, r.tage_raw);
    aitt_.read(ix, r.itt_raw);
  }

  // Stage t (S0) ahead select: uses ONLY latched FtbAhPipe + current (base, bnd, rcnt, rtop) — ZERO SRAM access!
  template <class S0GuessT, class MainOutT>
  bool ahead_select(pc_t base, pc_t bnd, const FtbAhPipe &ah, uint8_t rcnt, target_addr_t rtop,
                    S0GuessT &g, MainOutT &mo) const {
    if (!ah.valid) { stats_.ah_cold++; return false; }
    stats_.ah_lookups++;
    FtbEnt ent;
    if (!aftb_.select_way(ah.raw.ftb_set, ah.key_base, base, ent)) return false;
    stats_.ah_hits++;

    const FtbBlkIdx ix = make_ahead_index(ah.key_base, base, ah.key_fh);
    const int nsl = ftb_active_slots();
    bool dirs[FTB_SLOTS] = {};
    BlockTage::Pred tp[FTB_SLOTS];
    for (int k = 0; k < nsl; k++) {
      atage_.select_slot(ix, ah.raw.tage_raw.s[k], tp[k], ent.s[k].valid ? int(ent.s[k].ctr) : -1);
      dirs[k] = tp[k].pred;
    }
    target_addr_t itt_tgt = 0;
    const bool itt_hit = sim_env().ftb_no_aitt ? false : aitt_.select(ix, ah.raw.itt_raw, itt_tgt);

    g = S0GuessT{};
    g.hit = true;
    bool slot_tk[FTB_SLOTS] = {};
    eval_ent(base, bnd, ent, true, dirs, itt_hit, itt_tgt, rcnt, rtop, g, slot_tk);

    // Populate MainOut as well (used when FTB_AH_ONLY=2 skips S2 override)
    mo = MainOutT{};
    mo.next = g.next; mo.slot = g.slot; mo.type = g.type; mo.taken = g.taken;
    mo.nbits = g.nbits; mo.bits = g.bits; mo.ras_op = g.ras_op; mo.ras_val = g.ras_val;
    if (g.taken) {
      for (int k = 0; k < nsl; k++) {
        if (ent.s[k].valid && ent.s[k].offset == g.slot) { mo.rvc = ent.s[k].is_rvc; break; }
      }
    }
    for (int i = 0; i < FW; i++) {
      mo.meta.pcpn[i] = TN;
      mo.meta.alt_pred[i] = false;
      for (int t = 0; t < TN; t++) {
        mo.meta.tage_idx[i][t] = static_cast<tage_idx_t>(ix.idx[t] & TAGE_IDX_MASK);
        mo.meta.tage_tag[i][t] = static_cast<tage_tag_t>(ix.tag[t] & tcfg().tagmask[t]);
      }
    }
    for (int k = 0; k < nsl; k++) {
      const FtbSlot &sl = ent.s[k];
      if (!sl.valid || sl.offset >= FW) continue;
      if (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES >= bnd) continue;
      const int off = sl.offset;
      mo.dir[off] = slot_tk[k];
      mo.meta.alt_pred[off] = tp[k].alt;
      mo.meta.pcpn[off] = tp[k].pcpn;
      mo.meta.ps_info[off] = ps_pack(sl.type, (br_ind(sl.type) && itt_hit) ? 4 : (sl.type == BR_RET && rcnt > 0 ? 7 : 1), tp[k].used_alt);
    }
    return true;
  }

  // Evaluate an FTB entry: scan all active slots and pick the valid taken slot with MINIMUM offset (no slot swapping!).
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
    const int nsl = ftb_active_slots();
    int best_k = -1;
    uint8_t best_off = 255;
    for (int k = 0; k < nsl; k++) {
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
      g.next = tgt;
      g.nbits = HSYM_BITS;
      g.bits = hist_sym(bpc, tgt);
      if (br_call(sl.type)) { g.ras_op = 1; g.ras_val = bpc + ilen_of(sl.is_rvc); }
      else if (sl.type == BR_RET) { g.ras_op = 2; }
    }
  }

  // S0 uFTB lookup (0 bubble)
  template <class S0GuessT>
  S0GuessT s0_lookup(pc_t base, pc_t bnd, uint8_t rcnt, target_addr_t rtop) const {
    stats_.s0_lookups++;
    S0GuessT g{};
    g.next = bnd;
    FtbEnt ent;
    if (!ftb0_.lookup(base, ent)) return g;
    stats_.s0_hits++;
    g.hit = true;
    const bool dummy[FTB_SLOTS] = {};
    eval_ent(base, bnd, ent, false, dummy, false, 0, rcnt, rtop, g);
    return g;
  }

  // S1 array reads: Main FTB + S1 mini-TAGE + S2 Block-TAGE + Single Block-ITT
  void s1_read(pc_t base, const FtbBlkIdx &ix, FtbRaw &r) const {
    stats_.s1_lookups++;
    r = FtbRaw{};
    r.ftb_hit = ftb1_.lookup(base, r.ent);
    if (r.ftb_hit) stats_.s1_hits++;
    if (sim_env().ftb_s1_dir == 1 || sim_env().ftb_s1_dir == 2) s1_tage_.read(ix, r.s1_dir_raw);
    s2_tage_.read(ix, r.s2_tage_raw);
    itt_.read(ix, r.itt_raw);
  }

  // S1 prediction using Main FTB + S1 fast direction table (mini-TAGE / bimodal / oracle)
  template <class S0GuessT>
  S0GuessT s1_predict(pc_t base, pc_t bnd, const FtbBlkIdx &ix, const FtbRaw &r, uint8_t rcnt, target_addr_t rtop) const {
    S0GuessT g{};
    g.next = bnd;
    if (!r.ftb_hit) return g;
    g.hit = true;
    const int dir_mode = sim_env().ftb_s1_dir;
    bool use_dir = false;
    bool dirs[FTB_SLOTS] = {};
    const int nsl = ftb_active_slots();
    if (dir_mode == 1 || dir_mode == 2) {
      use_dir = true;
      for (int k = 0; k < nsl; k++) {
        BlockTage::Pred p;
        s1_tage_.select_slot(ix, r.s1_dir_raw.s[k], p);
        dirs[k] = p.pred;
      }
    } else if (dir_mode == 3) {
      use_dir = true;
      for (int k = 0; k < nsl; k++) {
        BlockTage::Pred p;
        s2_tage_.select_slot(ix, r.s2_tage_raw.s[k], p);
        dirs[k] = p.pred;
      }
    }
    eval_ent(base, bnd, r.ent, use_dir, dirs, false, 0, rcnt, rtop, g);
    return g;
  }

  // S2 fallback prediction using Main FTB + 2-slot Block-TAGE + Single Block-ITT + RAS
  template <class MainOutT>
  void s2_select(pc_t base, pc_t bnd, const FtbBlkIdx &ix, const FtbRaw &r,
                 uint8_t rcnt, target_addr_t rtop, MainOutT &o) const {
    stats_.s2_lookups++;
    o = MainOutT{};
    o.next = bnd;
    // Initialize default meta so cold branches still carry valid pcpn=TN and pred tag
    for (int i = 0; i < FW; i++) {
      o.meta.pcpn[i] = TN;
      o.meta.alt_pred[i] = false;
      for (int t = 0; t < TN; t++) {
        o.meta.tage_idx[i][t] = static_cast<tage_idx_t>(ix.idx[t] & TAGE_IDX_MASK);
        o.meta.tage_tag[i][t] = static_cast<tage_tag_t>(ix.tag[t] & tcfg().tagmask[t]);
      }
    }
    if (!r.ftb_hit) return;

    const int nsl = ftb_active_slots();
    bool dirs[FTB_SLOTS] = {};
    BlockTage::Pred tp[FTB_SLOTS];
    for (int k = 0; k < nsl; k++) {
      s2_tage_.select_slot(ix, r.s2_tage_raw.s[k], tp[k]);
      dirs[k] = tp[k].pred;
    }
    target_addr_t itt_tgt = 0;
    const bool itt_hit = itt_.select(ix, r.itt_raw, itt_tgt);

    bool slot_tk[FTB_SLOTS] = {};
    eval_ent(base, bnd, r.ent, true, dirs, itt_hit, itt_tgt, rcnt, rtop, o, slot_tk);
    if (o.taken) {
      for (int k = 0; k < nsl; k++) {
        if (r.ent.s[k].valid && r.ent.s[k].offset == o.slot) { o.rvc = r.ent.s[k].is_rvc; break; }
      }
    }
    // Populate per-slot outputs for PTAB / PredecodeChecker / BpuTrain
    for (int k = 0; k < nsl; k++) {
      const FtbSlot &sl = r.ent.s[k];
      if (!sl.valid || sl.offset >= FW) continue;
      if (base + static_cast<pc_t>(sl.offset) * SLOT_BYTES >= bnd) continue;
      const int off = sl.offset;
      o.dir[off] = slot_tk[k];
      o.meta.alt_pred[off] = tp[k].alt;
      o.meta.pcpn[off] = tp[k].pcpn;
      o.meta.ps_info[off] = ps_pack(sl.type, (br_ind(sl.type) && itt_hit) ? 4 : (sl.type == BR_RET && rcnt > 0 ? 7 : 1), tp[k].used_alt);
    }
  }

  // ---- Commit-order block training -----------------------------------------------------------------------------------
  // Insert/update one branch into an FTB entry WITHOUT swapping slot indices (so BlockTage slot k stays stable!).
  int upsert_branch(FtbEnt &ent, const FtbTrkBr &b, bool count_evict) {
    const int nsl = ftb_active_slots();
    for (int k = 0; k < nsl; k++) {
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
    for (int k = 0; k < nsl; k++) if (!ent.s[k].valid) { pos = k; break; }
    if (pos < 0) {
      if (!b.actual_dir && b.type == BR_DIRECT) return -1;
      if (count_evict) stats_.slot_evicts++;
      // Evict the weakest conditional slot (lowest ctr), else last slot
      pos = nsl - 1;
      int min_c = 99;
      for (int k = 0; k < nsl; k++) {
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

  // Update an FtbTab with a block train event; returns pointer to updated entry (or nullptr if not allocated).
  FtbEnt *update_tab(FtbTab &tab, const FtbTrainEv &ev, bool alloc_only_taken, bool count_evict) {
    if (tab.N <= 0 || ev.nbr == 0) return nullptr;
    // Check if any branch is eligible for this table (e.g. target compression filter for uFTB; BR_RET uses RAS so target width does not apply)
    bool has_taken = false;
    bool any_fits = false;
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
      // Pick invalid way or LRU way
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
      upsert_branch(en, b, count_evict);
    }
    bool any_v = false;
    const int nsl = ftb_active_slots();
    for (int k = 0; k < nsl; k++) if (en.s[k].valid) any_v = true;
    if (!any_v) {
      en.valid = false;
      return nullptr;
    }
    tab.touch_lru(s, hit);
    return &en;
  }

  // Apply one block-level commit training event to all FTB/TAGE/ITT tables.
  void apply_train_ev(const FtbTrainEv &ev, bool use_live_s1_pred = true) {
    stats_.train_blocks++;
    // 1. Update Main FTB (allocates all branch blocks) and uFTB (allocates only taken blocks to avoid S0 pollution)
    FtbEnt *main_ent = update_tab(ftb1_, ev, /*alloc_only_taken=*/false, /*count_evict=*/true);
    update_tab(ftb0_, ev, /*alloc_only_taken=*/true, /*count_evict=*/false);
    if (!main_ent) return;

    const FtbBlkIdx ix = make_index(ev.start, ev.fh);
    const int s1_dir_mode = sim_env().ftb_s1_dir;
    const int nsl = ftb_active_slots();

    // 2. Update Block-TAGE, S1 mini-TAGE, and Single Block-ITT
    for (int i = 0; i < ev.nbr; i++) {
      const FtbTrkBr &b = ev.br[i];
      if (b.type == BR_DIRECT) {
        int sl = -1;
        for (int k = 0; k < nsl; k++) {
          if (main_ent->s[k].valid && main_ent->s[k].offset == b.offset) { sl = k; break; }
        }
        if (sl >= 0) {
          stats_.br_upd++;
          if (b.pcpn == TN) stats_.pcpn_tn++;
          if (b.pred_tag0 != static_cast<tage_tag_t>(ix.tag[0] & tcfg().tagmask[0])) stats_.tag_mismatch++;
          // Update S2 Block-TAGE using prediction-time pcpn/alt/pred
          s2_tage_.update_slot(ix, sl, b.actual_dir, b.pred_dir, b.alt_pred, b.pcpn);
          // Update S1 mini-TAGE (if enabled) using its own prediction for this slot
          if (s1_dir_mode == 1 || s1_dir_mode == 2) {
            BlockTage::Pred sp;
            s1_tage_.predict_slot(ix, sl, sp);
            (void)use_live_s1_pred;
            s1_tage_.update_slot(ix, sl, b.actual_dir, sp.pred, sp.alt, sp.pcpn);
          }
        }
      }
      if (br_ind(b.type) && b.actual_dir) {
        itt_.update(ix, b.target);
      }
    }
    // 3. Phase 2: Train Ahead FTB, Ahead Block-TAGE, and Ahead Block-ITT
    if (sim_env().ftb_ahead > 0 && ev.ah_v && aftb_.N > 0 && ev.nbr > 0) {
      const uint32_t s = aftb_.index(ev.ah_key_base);
      const uint32_t t = aftb_.tag(ev.ah_key_base, ev.start);
      int hit = -1;
      for (int w = 0; w < aftb_.W; w++) {
        if (aftb_.e[s * aftb_.W + w].valid && aftb_.e[s * aftb_.W + w].tag32 == t) { hit = w; break; }
      }
      if (hit < 0) {
        int vic = -1;
        for (int w = 0; w < aftb_.W; w++) if (!aftb_.e[s * aftb_.W + w].valid) { vic = w; break; }
        if (vic < 0) {
          vic = 0;
          for (int w = 1; w < aftb_.W; w++) if (aftb_.e[s * aftb_.W + w].lru > aftb_.e[s * aftb_.W + vic].lru) vic = w;
        }
        hit = vic;
        FtbEnt &en = aftb_.e[s * aftb_.W + hit];
        en = FtbEnt{};
        en.valid = true;
        en.tag32 = t;
        en.lru = static_cast<uint8_t>(aftb_.W - 1);
      }
      FtbEnt &aen = aftb_.e[s * aftb_.W + hit];
      for (int i = 0; i < ev.nbr; i++) upsert_branch(aen, ev.br[i], /*count_evict=*/false);
      aftb_.touch_lru(s, hit);

      const FtbBlkIdx aix = make_ahead_index(ev.ah_key_base, ev.start, ev.ah_key_fh);
      for (int i = 0; i < ev.nbr; i++) {
        const FtbTrkBr &b = ev.br[i];
        if (b.type == BR_DIRECT) {
          int sl = -1;
          for (int k = 0; k < nsl; k++) {
            if (aen.s[k].valid && aen.s[k].offset == b.offset) { sl = k; break; }
          }
          if (sl >= 0) {
            BlockTage::Pred ap;
            atage_.predict_slot(aix, sl, ap, int(aen.s[sl].ctr));
            atage_.update_slot(aix, sl, b.actual_dir, ap.pred, ap.alt, ap.pcpn);
          }
        }
        if (br_ind(b.type) && b.actual_dir) {
          aitt_.update(aix, b.target);
        }
      }
    }
  }

  void clear_false_branch(pc_t base, pc_t nb_pc) {
    stats_.pd_clears++;
    ftb0_.clear_slot(base, nb_pc);
    ftb1_.clear_slot(base, nb_pc);
  }
};

}  // namespace bpuf

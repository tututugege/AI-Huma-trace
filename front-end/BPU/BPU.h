// Compact cycle-exact BPU: TAGE + BTB/indirect target cache + type predictor + history/RAS + update queue.
// Replaces the RTL-style BPU_TOP (kept as tests/bpu_ref in the trace repo, checked by tests/bpu_diff.cpp).
// comb() is pure; commit() applies the transaction it returned.
#pragma once
#include "../frontend.h"
#include "../wire_types.h"
#include "BPU_configs.h"
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <vector>

namespace bpuf {

constexpr int NB = BPU_BANK_NUM, FW = FETCH_WIDTH, CW = COMMIT_WIDTH, TN = TN_MAX;

// EXP-010: realistic prediction latency. BPU_BUBBLE=B: after a predicted-taken fetch group the BPU idles B cycles (the main
// TAGE/BTB predictor is B+1 cycles deep and the fall-through address is fetched meanwhile). BPU_UBTB=N: an N-entry direct-mapped
// 1-cycle target table (tag 12 b) hides the bubble when it holds the same target. Both default to 0 (= old zero-bubble model).
inline int env_int(const char *n, int d) { const char *v = std::getenv(n); return v ? std::atoi(v) : d; }

// ---- docs/BPU_ACC.md: TAGE parameters ---------------------------------------------------------------------------------------------
// BPA_HLEN="8,13,32,119"  history length per tagged table (<= 255 bits; run time). The other parameters are fixed to the RTL design:
// TAGE_TAG_WIDTH-bit tags, 3-bit tagged counters, 2-bit useful counters, one use-alt counter, allocation in the lowest free table or (1/3)
// the next one, one useful-bit reset row every 2^11 updates. (The run-time sweeps BPA_TAGW / CNTW / UW / WEAK / UALT / ALLOC / URST and the
// statistical corrector BPA_SC* found no gain and were removed with EXP-028's switch clean-up.)
inline std::vector<int> env_list(const char *n, std::vector<int> d) {
  const char *v = std::getenv(n);
  if (!v || !*v) return d;
  std::vector<int> r;
  for (const char *c = v; *c;) { r.push_back(std::atoi(c)); while (*c && *c != ',') c++; if (*c == ',') c++; }
  while (!r.empty() && int(r.size()) < int(d.size())) r.push_back(r.back());
  return r;
}
struct TageCfg {
  uint32_t ghr[TN] = {}, fl[FH_N_MAX][TN] = {}, tagw[TN] = {}, tagmask[TN] = {};
  static constexpr int uw = 2, ualt = 1, urst = 11, cmax = 7, chalf = 4;   // 3-bit counters, 2-bit useful
  int hlen_max = 0;
  TageCfg() {
    const std::vector<int> dh = (TN == 4) ? std::vector<int>{8, 13, 32, 119} : (TN == 5) ? std::vector<int>{5, 9, 17, 40, 119} : (TN == 6) ? std::vector<int>{4, 7, 12, 22, 42, 119} : (TN == 3) ? std::vector<int>{8, 24, 119} : (TN == 2) ? std::vector<int>{13, 119} : std::vector<int>(TN, 32);
    const std::vector<int> h = env_list("BPA_HLEN", dh);
    for (int i = 0; i < TN; i++) {
      ghr[i] = h[i]; tagw[i] = TAGE_TAG_WIDTH;
      if (ghr[i] < 1 || ghr[i] > 255) { std::printf("[BPA] bad BPA_HLEN\n"); std::exit(1); }
      tagmask[i] = (1u << tagw[i]) - 1;
      if (int(ghr[i]) > hlen_max) hlen_max = ghr[i];
      // folded lengths: index fold 11 b (or the history length if shorter), tag folds = tag width and tag width - 1 (8 and 7 with the default 8-bit tags)
      fl[0][i] = ghr[i] < 11 ? ghr[i] : 11;
      fl[1][i] = ghr[i] < tagw[i] ? ghr[i] : tagw[i];
      fl[2][i] = ghr[i] < tagw[i] - 1 ? ghr[i] : tagw[i] - 1;
    }
  }
};
// BPA_ITT: history-indexed indirect target tables (ITTAGE-like) added to the BTB bank. BPA_ITTH="2" (or "1,3"): the TAGE history table(s) whose index/tag
// are reused (ascending history length), BPA_ITTN entries per table and bank.
// Default (EXP-025, the RTL design): three tables on TAGE history tables 1, 2, 3 (13 / 32 / 119 bits), 64 entries each per bank (192 entries, 42 bits each:
// 64.5 Kb for 8 banks; the single 256-entry table it replaces was 86 Kb). Compile-time defaults CFG_ITT_H / CFG_ITT_SIZE, env still overrides.
#ifndef CFG_ITT_H
#define CFG_ITT_H "1,2,3"
#endif
#ifndef CFG_ITT_SIZE
#define CFG_ITT_SIZE 64
#endif
constexpr int ITT_MAX = 3, ITT_NMAX = 1024;
struct IttCfg {
  int n = 0, size = CFG_ITT_SIZE, h[ITT_MAX] = {};
  IttCfg() {
    const char *v = std::getenv("BPA_ITTH");   // unset: the compile-time default; empty string: ITT off
    if (!v) v = CFG_ITT_H;
    for (const char *c = v; *c;) { if (n < ITT_MAX) h[n++] = std::atoi(c); while (*c && *c != ',') c++; if (*c == ',') c++; }
    size = env_int("BPA_ITTN", CFG_ITT_SIZE);
    if (n && (size < 2 || size > ITT_NMAX || (size & (size - 1)))) { std::printf("[BPA] bad BPA_ITTN\n"); std::exit(1); }
    for (int i = 0; i < n; i++) if (h[i] < 0 || h[i] >= TN) { std::printf("[BPA] bad BPA_ITTH\n"); std::exit(1); }
  }
};
inline const IttCfg &icfg() { static const IttCfg c; return c; }
inline const TageCfg &tcfg() { static const TageCfg c; return c; }
inline uint32_t ghr_len(int i) { return tcfg().ghr[i]; }
inline uint32_t fh_len(int k, int i) { return tcfg().fl[k][i]; }

// BPO_HIST=1 (docs/BPU_OPT.md): the global history holds one symbol of BPO_HSYM bits (folded from pc and target) per TAKEN branch;
// not-taken conditional branches push nothing. Default 0 = one direction bit per conditional branch.
inline int cfg_hsym() { static const int v = env_int("BPO_HSYM", 3); return v; }
inline uint8_t hist_sym(pc_t bpc, pc_t tgt) {
  static const int hs = cfg_hsym();
  const uint32_t v = (bpc >> SLOT_SHIFT) ^ (tgt >> SLOT_SHIFT) ^ ((bpc >> SLOT_SHIFT) >> 7);
  uint32_t r = 0;
  for (int i = 0; i < 32; i += hs) r ^= (v >> i) & ((1u << hs) - 1);
  return uint8_t(r);
}
inline uint64_t &pd_flush_count() { static uint64_t n = 0; return n; }   // statistics only: predecode-checker flushes seen by the BPU
// BPA_REPAIR=1: exact history / RAS repair at a backend branch mispredict. The BPU keeps a checkpoint record per fetch block (state before the
// block, predicted types of its slots) keyed by a block sequence number that travels in the PredMeta to the back end FTQ; the back end returns it with
// the slot and type of the mispredicted branch, and the BPU rebuilds the history / RAS as: checkpoint + earlier (not taken) slots + the branch's actual outcome.
// Default 0: spec history / RAS are copied from the committed state (which lacks every older in-flight branch, call and return).
inline uint64_t *repair_stats() { static uint64_t n[2] = {0, 0}; return n; }   // [0] repaired, [1] record not found / mismatch
// BPU_ICALL=1: a jalr with rd = ra that is not a return is a call (BR_ICALL: indirect target + RAS push). 0 = such a jalr is BR_IDIRECT (old design).
inline bool br_ind(int t) { return t == BR_IDIRECT || t == BR_ICALL; }
inline bool br_call(int t) { return t == BR_CALL || t == BR_ICALL; }

// statistics-only prediction record of one slot (BPA_STATS): predicted type (3 b), target source (3 b, see Btb::select; 7 = RAS), use-alt (1 b)
inline uint8_t ps_pack(int ty, int src, bool used_alt) { return uint8_t((ty & 7) | ((src & 7) << 3) | (used_alt ? 0x40 : 0)); }

inline uint32_t bank_of(pc_t pc) { return (pc >> SLOT_SHIFT) % NB; }
inline pc_t bank_pc_of(pc_t pc) {
  if ((NB & (NB - 1)) != 0) return ((pc >> SLOT_SHIFT) / NB) << SLOT_SHIFT;
  return pc >> ceil_log2_u32(NB);
}

// ---------------------------------------------------------------------------
// Global history: GHR as a shift register plus the folded histories TAGE indexes with.
// ---------------------------------------------------------------------------
struct Hist {
  uint64_t g[4] = {};   // 256-bit global history, newest bit = bit 0
  uint32_t fh[FH_N_MAX][TN] = {};
  bool bit(uint32_t k) const { return (g[k >> 6] >> (k & 63)) & 1; }
  void push(bool dir) {
    for (int k = 0; k < FH_N_MAX; k++)
      for (int i = 0; i < TN; i++) {
        const uint32_t len = fh_len(k, i);
        uint32_t v = fh[k][i];
        const uint32_t hi = (v >> (len - 1)) & 1;
        v = ((v << 1) & ((1u << len) - 1)) | (uint32_t(dir) ^ hi);
        v ^= uint32_t(bit(ghr_len(i) - 1)) << (ghr_len(i) % len);
        fh[k][i] = v;
      }
    g[3] = (g[3] << 1) | (g[2] >> 63); g[2] = (g[2] << 1) | (g[1] >> 63); g[1] = (g[1] << 1) | (g[0] >> 63); g[0] = (g[0] << 1) | dir;
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
  uint32_t ua_idx = 0;
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
  uint8_t pcpn;
  tage_tag_t tag[TN];
  tage_idx_t idx[TN];
  bool used_alt;   // statistics only (BPA_STATS): the weak-provider use-alt rule replaced the provider's prediction
};

struct TageUpd {
  pc_t bank_pc;
  bool real_dir, pred, alt;
  int pcpn;
  tage_tag_t tag[TN];
  tage_idx_t idx[TN];
  bool loop_used;
};

class Tage {
  uint8_t base_[BASE_ENTRY_NUM];
  tage_tag_t tag_[TN][TN_ENTRY_NUM];
  uint8_t cnt_[TN][TN_ENTRY_NUM], useful_[TN][TN_ENTRY_NUM];
  uint8_t lsfr_ = 8;  // bit i = LSFR[i]; reset value {0,0,0,1}
  uint32_t reset_cnt_ = 0;
  tage_use_alt_ctr_t use_alt_[64];

  static uint8_t inc2(uint8_t v) { return v >= 3 ? 3 : v + 1; }
  static uint8_t dec2(uint8_t v) { return v == 0 ? 0 : v - 1; }
  static uint8_t incu(uint8_t v) { const uint8_t m = uint8_t((1u << tcfg().uw) - 1); return v >= m ? m : v + 1; }
  static uint8_t decu(uint8_t v) { return v == 0 ? 0 : v - 1; }
  static bool taken(uint8_t c) { return c >= tcfg().chalf; }
  static bool weak(uint8_t c) { return c == TageCfg::chalf - 1; }
  static uint8_t cinc(uint8_t c) { return c >= tcfg().cmax ? c : c + 1; }
  static uint8_t cdec(uint8_t c) { return c == 0 ? 0 : c - 1; }
  static constexpr tage_use_alt_ctr_t kUaInit = static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_INIT);
  tage_use_alt_ctr_t ua(uint32_t base_idx) const { return use_alt_[base_idx & (tcfg().ualt - 1)]; }

 public:
  struct Idx { uint32_t base; tage_idx_t idx[TN]; tage_tag_t tag[TN]; };
  struct Raw { uint8_t base_cnt; tage_tag_t tag[TN]; uint8_t cnt[TN], useful[TN]; };
  Tage() { std::memset(this, 0, sizeof(*this)); lsfr_ = 8; for (auto &u : use_alt_) u = kUaInit; }
  static bool provider_weak(int pcpn, uint8_t cnt) { return pcpn >= TN || weak(cnt); }

  void predict(pc_t pc, const uint32_t fh[FH_N_MAX][TN], TagePred &o) const {
    const uint32_t w = static_cast<uint32_t>(pc >> SLOT_SHIFT);   // index hashes use the low 32 bits of the word address
    uint32_t b = w & TAGE_BASE_IDX_MASK;
    int pcpn = TN, altpcpn = TN;
    for (int i = 0; i < TN; i++) {
      o.tag[i] = (fh[1][i] ^ fh[2][i] ^ w) & tcfg().tagmask[i];
      o.idx[i] = (fh[0][i] ^ w) & TAGE_IDX_MASK;
    }
    uint8_t cnt[TN], useful[TN];
    for (int i = 0; i < TN; i++) { cnt[i] = cnt_[i][o.idx[i]]; useful[i] = useful_[i][o.idx[i]]; }
    for (int i = TN - 1; i >= 0; i--)
      if (tag_[i][o.idx[i]] == o.tag[i]) { pcpn = i; break; }
    for (int i = pcpn - 1; i >= 0; i--)
      if (tag_[i][o.idx[i]] == o.tag[i]) { altpcpn = i; break; }
    const bool base_pred = base_[b] >= 2;
    o.alt = altpcpn >= TN ? base_pred : taken(cnt[altpcpn]);
    o.pcpn = pcpn;
    o.used_alt = false;
    if (pcpn >= TN) { o.pred = base_pred; return; }
    o.pred = taken(cnt[pcpn]);
    if (weak(cnt[pcpn]) && useful[pcpn] <= TAGE_USE_ALT_USEFUL_THRESHOLD &&
        ua(b) <= static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_USE_ALT_THRESHOLD))
    { o.pred = o.alt; o.used_alt = true; }
  }
  // Pipelined access (BPU_PIPE): index generation (comb, S0), array read (S1), compare/select (comb, S2).
  static void index(pc_t pc, const uint32_t fh[FH_N_MAX][TN], Idx &o) {
    const uint32_t w = pc >> SLOT_SHIFT;
    o.base = w & TAGE_BASE_IDX_MASK;
    for (int i = 0; i < TN; i++) {
      o.tag[i] = (fh[1][i] ^ fh[2][i] ^ w) & tcfg().tagmask[i];
      o.idx[i] = (fh[0][i] ^ w) & TAGE_IDX_MASK;
    }
  }
  void read(const Idx &x, Raw &r) const {  // 1 read port: base[1], tag/cnt/useful[TN]
    r.base_cnt = base_[x.base];
    for (int i = 0; i < TN; i++) { r.tag[i] = tag_[i][x.idx[i]]; r.cnt[i] = cnt_[i][x.idx[i]]; r.useful[i] = useful_[i][x.idx[i]]; }
  }
  void select(const Idx &x, const Raw &r, TagePred &o) const {
    int pcpn = TN, altpcpn = TN;
    for (int i = 0; i < TN; i++) { o.tag[i] = x.tag[i]; o.idx[i] = x.idx[i]; }
    for (int i = TN - 1; i >= 0; i--)
      if (r.tag[i] == x.tag[i]) { pcpn = i; break; }
    for (int i = pcpn - 1; i >= 0; i--)
      if (r.tag[i] == x.tag[i]) { altpcpn = i; break; }
    const bool base_pred = r.base_cnt >= 2;
    o.alt = altpcpn >= TN ? base_pred : taken(r.cnt[altpcpn]);
    o.pcpn = pcpn;
    o.used_alt = false;
    if (pcpn >= TN) { o.pred = base_pred; return; }
    o.pred = taken(r.cnt[pcpn]);
    if (weak(r.cnt[pcpn]) && r.useful[pcpn] <= TAGE_USE_ALT_USEFUL_THRESHOLD &&
        ua(x.base) <= static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_USE_ALT_THRESHOLD))
    { o.pred = o.alt; o.used_alt = true; }
  }

  void update(const TageUpd &u, TageWr &w) const {
    const TageCfg &K = tcfg();
    w = TageWr{};
    w.upd = true;
    const bool dir = u.real_dir;
    const int pcpn = u.pcpn;
    const bool tpred = u.pred;
    uint8_t cnt[TN], useful[TN];
    for (int i = 0; i < TN; i++) {
      w.idx[i] = u.idx[i] & TAGE_IDX_MASK;
      cnt[i] = cnt_[i][w.idx[i]];
      useful[i] = useful_[i][w.idx[i]];
    }
    const uint32_t bidx = (u.bank_pc >> SLOT_SHIFT) & TAGE_BASE_IDX_MASK;
    const uint8_t base_cnt = base_[bidx];

    // LSFR (4-bit): shift in bit0^bit3
    const uint8_t ln = uint8_t(((lsfr_ << 1) & 0xE) | ((lsfr_ ^ (lsfr_ >> 3)) & 1));
    w.lsfr_next = ln;
    const int rnd = ((ln & 1) << 1) | ((ln >> 1) & 1);
    w.reset_cnt_next = reset_cnt_ + 1;

    if (pcpn < TN) {
      const bool praw = taken(cnt[pcpn]);
      const bool provider_used = !u.loop_used;
      const bool alt_used = provider_used && (tpred == u.alt);
      uint8_t nu = useful[pcpn];
      bool wu = false;
      if (!provider_used) { nu = praw == dir ? incu(nu) : decu(nu); wu = true; }
      else if (!alt_used) { nu = tpred == dir ? incu(nu) : decu(nu); wu = true; }
      else if (praw != dir && u.alt == dir) { nu = decu(nu); wu = true; }
      if (wu) { w.useful_we |= 1u << pcpn; w.useful_v[pcpn] = nu; }
      w.cnt_we |= 1u << pcpn;
      w.cnt_v[pcpn] = dir ? cinc(cnt[pcpn]) : cdec(cnt[pcpn]);
      if (weak(cnt[pcpn]) && useful[pcpn] <= TAGE_USE_ALT_USEFUL_THRESHOLD && praw != u.alt &&
          ((praw == dir) ^ (u.alt == dir))) {
        constexpr tage_use_alt_ctr_t kMax = static_cast<tage_use_alt_ctr_t>((1u << TAGE_USE_ALT_CTR_BITS) - 1u);
        const tage_use_alt_ctr_t cur = ua(bidx);
        w.ua_we = true;
        w.ua_idx = bidx & (K.ualt - 1);
        w.ua_val = u.alt == dir ? (cur == 0 ? 0 : cur - 1) : (cur >= kMax ? kMax : cur + 1);
      }
    } else {
      w.base_we = true;
      w.base_idx = bidx;
      w.base_val = dir ? inc2(base_cnt) : dec2(base_cnt);
    }

    if (tpred != dir && (pcpn <= TN - 2 || pcpn == TN)) {
      const int start = pcpn == TN ? 0 : pcpn + 1;
      int j = -1, k = -1;
      for (int i = start; i < TN; i++)
        if (useful[i] == 0) {
          if (j < 0) j = i;
          else { k = i; break; }
        }
      if (j < 0) {
        for (int i = start; i < TN; i++) { w.useful_we |= 1u << i; w.useful_v[i] = decu(useful[i]); }
      } else {
        auto alloc_at = [&](int t) {
          w.tag_we |= 1u << t;  w.tag_v[t] = u.tag[t];
          w.cnt_we |= 1u << t;  w.cnt_v[t] = dir ? K.chalf : K.chalf - 1;
          w.useful_we |= 1u << t; w.useful_v[t] = 0;
        };
        alloc_at((k >= 0 && rnd % 3 == 0) ? k : j);
      }
    }


    // periodic useful-bit reset sweep
    if ((reset_cnt_ & ((1u << K.urst) - 1)) == 0) {
      const uint32_t row = ((reset_cnt_ >> K.urst) & 0xfff) & TAGE_IDX_MASK;
      const bool msb_only = (reset_cnt_ >> (K.urst + 12)) & 1;
      w.reset_row = row;
      for (int i = 0; i < TN; i++) {
        if ((w.useful_we >> i & 1) && w.idx[i] == row) continue;
        const uint8_t cur = useful_[i][row];
        w.reset_we |= 1u << i;
        w.reset_v[i] = msb_only ? (cur & 1) : (cur & ((1u << K.uw) - 1) & 2);
      }
    }
  }

  void commit(const TageWr &w) {
    if (!w.upd) return;
    lsfr_ = w.lsfr_next;
    reset_cnt_ = w.reset_cnt_next;
    if (w.ua_we) use_alt_[w.ua_idx] = w.ua_val;
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
// BTB + indirect target tables (ITT) (one per bank)
// ---------------------------------------------------------------------------
struct IttEnt { target_addr_t target; uint16_t tag; uint8_t conf, useful; bool valid; };
struct BtbWr {
  uint8_t itt_we = 0; uint32_t itt_idx[ITT_MAX] = {}; IttEnt itt_v[ITT_MAX] = {};
  bool btb_we = false;
  uint32_t btb_way = 0, btb_idx = 0;  btb_tag_t btb_tag = 0;  target_addr_t btb_bta = 0;  uint8_t btb_useful = 0;
};

class Btb {
  struct Way { btb_tag_t tag; target_addr_t bta; bool valid; wire3_t useful; };
  Way btb_[BTB_ENTRY_NUM][BTB_WAY_NUM];
  IttEnt itt_[ITT_MAX][ITT_NMAX];

  static constexpr int width(uint32_t m) { int w = 0; while (m) { w++; m >>= 1; } return w; }
  static uint32_t hash(uint32_t pcw, uint32_t mask, uint32_t salt) {
    const int wd = width(mask);
    uint32_t x = pcw;
    if (wd < 32) x ^= pcw >> wd;
    if (2 * wd < 32) x ^= pcw >> (2 * wd);
    if (3 * wd < 32) x ^= pcw >> (3 * wd);
    return (x ^ salt) & mask;
  }
  static uint32_t btb_idx(pc_t pc) { return hash(static_cast<uint32_t>(pc >> SLOT_SHIFT), BTB_IDX_MASK, 0x9e3779b9u); }
  static btb_tag_t btb_tag(pc_t pc) { return static_cast<btb_tag_t>(((pc >> SLOT_SHIFT) >> ceil_log2_u32(BTB_ENTRY_NUM)) & BTB_TAG_MASK); }
  static uint8_t useful_next(uint8_t v, bool ok) { return ok ? (v < 7 ? v + 1 : v) : (v > 0 ? v - 1 : 0); }
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
  Btb() { std::memset(this, 0, sizeof(*this)); }
  // ITT lookup: the longest-history hit (index and tag come from the TAGE index generation, tags/indices carried in the prediction meta)
  const IttEnt *itt_hit(const tage_idx_t *idx, const tage_tag_t *tag, int *which = nullptr) const {
    const IttCfg &c = icfg();
    for (int k = c.n - 1; k >= 0; k--) {
      const IttEnt &e = itt_[k][idx[c.h[k]] & (c.size - 1)];
      if (e.valid && e.tag == tag[c.h[k]]) { if (which) *which = k; return &e; }
    }
    return nullptr;
  }

  // Pipelined access: one read of the BTB set and the ITT entries; select is pure compare.
  struct Raw { Way btb[BTB_WAY_NUM]; IttEnt it[ITT_MAX]; };
  void read(pc_t pc, Raw &r, const tage_idx_t *tidx = nullptr) const {
    if (tidx) for (int k = 0; k < icfg().n; k++) r.it[k] = itt_[k][tidx[icfg().h[k]] & (icfg().size - 1)];
    for (int w = 0; w < BTB_WAY_NUM; w++) r.btb[w] = btb_[btb_idx(pc)][w];
  }
  // src (statistics only): 1 BTB hit, 2 BTB miss (pc+4), 3 ITT, 5 indirect fallback to BTB, 6 indirect miss (pc+4)
  static target_addr_t select(pc_t pc, br_type_t type, const Raw &r, int *src, const tage_tag_t *ttag, bool *itt, pc_t fall) {   // pc: the bank-local pc (BTB tag); fall: the real fall-through address used when the BTB misses
    const int h = hit_way(r.btb, btb_tag(pc));
    if (br_ind(type)) {
      if (ttag) for (int k = icfg().n - 1; k >= 0; k--)
        if (r.it[k].valid && r.it[k].tag == ttag[icfg().h[k]]) { if (itt) *itt = true; if (src) *src = 3; return r.it[k].target; }
      if (src) *src = h >= 0 ? 5 : 6;
      return h >= 0 ? r.btb[h].bta : fall;
    }
    if (type == BR_DIRECT || type == BR_CALL || type == BR_JAL || type == BR_RET) {
      if (src) *src = h >= 0 ? 1 : 2;
      return h >= 0 ? r.btb[h].bta : fall;
    }
    return fall;
  }

  // bank_pc is the bank-local pc, exactly what the RTL hands the bank
  target_addr_t predict(pc_t pc, br_type_t type, int *src = nullptr, const tage_idx_t *tidx = nullptr, const tage_tag_t *ttag = nullptr, bool *itt = nullptr) const {
    const Way(&set)[BTB_WAY_NUM] = btb_[btb_idx(pc)];
    const int h = hit_way(set, btb_tag(pc));
    if (br_ind(type)) {
      if (tidx && icfg().n) { const IttEnt *e = itt_hit(tidx, ttag); if (e) { if (itt) *itt = true; if (src) *src = 3; return e->target; } }
      if (src) *src = h >= 0 ? 5 : 6;
      return h >= 0 ? set[h].bta : pc + 4;
    }
    if (type == BR_DIRECT || type == BR_CALL || type == BR_JAL || type == BR_RET) {
      if (src) *src = h >= 0 ? 1 : 2;
      return h >= 0 ? set[h].bta : pc + 4;
    }
    return pc + 4;
  }

  void update(pc_t pc, br_type_t type, bool dir, target_addr_t addr, BtbWr &w, const tage_idx_t *tidx = nullptr, const tage_tag_t *ttag = nullptr) const {
    w = BtbWr{};
    const bool writes = type == BR_DIRECT || type == BR_CALL || type == BR_RET || type == BR_JAL || br_ind(type);
    if (dir && writes) {
      const uint32_t ei = btb_idx(pc);
      const btb_tag_t tg = btb_tag(pc);
      const Way(&set)[BTB_WAY_NUM] = btb_[ei];
      const int h = hit_way(set, tg);
      const int way = h >= 0 ? h : victim(set);
      w.btb_we = true;
      w.btb_way = way; w.btb_idx = ei; w.btb_tag = tg; w.btb_bta = addr;
      w.btb_useful = br_ind(type) ? INDIRECT_BTB_INIT_USEFUL
                     : (h >= 0 ? useful_next(set[way].useful, set[way].bta == addr) : 1);
    }

    if (dir && br_ind(type) && tidx && icfg().n) {
      const IttCfg &c = icfg();
      int prov = -1;
      for (int k = c.n - 1; k >= 0; k--) { const IttEnt &e = itt_[k][tidx[c.h[k]] & (c.size - 1)]; if (e.valid && e.tag == ttag[c.h[k]]) { prov = k; break; } }
      int alloc_from = 0;
      if (prov >= 0) {
        const uint32_t i = tidx[c.h[prov]] & (c.size - 1);
        IttEnt e = itt_[prov][i];
        if (e.target == addr) { e.conf = e.conf < 3 ? e.conf + 1 : 3; e.useful = 1; alloc_from = c.n; }
        else { if (e.conf > 0) e.conf--; else e.target = addr; e.useful = 0; alloc_from = prov + 1; }
        w.itt_we |= 1u << prov; w.itt_idx[prov] = i; w.itt_v[prov] = e;
      }
      for (int k = alloc_from; k < c.n; k++) {   // allocate in the first longer table with a free / not useful entry, else age the entry
        const uint32_t i = tidx[c.h[k]] & (c.size - 1);
        IttEnt e = itt_[k][i];
        if (!e.valid || e.useful == 0) { e = IttEnt{addr, uint16_t(ttag[c.h[k]]), 0, 0, true}; w.itt_we |= 1u << k; w.itt_idx[k] = i; w.itt_v[k] = e; break; }
        e.useful = 0; w.itt_we |= 1u << k; w.itt_idx[k] = i; w.itt_v[k] = e;
      }
    }
  }

  void commit(const BtbWr &w) {
    for (int k = 0; k < ITT_MAX; k++) if (w.itt_we >> k & 1) itt_[k][w.itt_idx[k]] = w.itt_v[k];
    if (w.btb_we) btb_[w.btb_idx][w.btb_way] = {w.btb_tag, w.btb_bta, true, w.btb_useful};
  }
};

// ---------------------------------------------------------------------------
// Branch-type predictor (banked, set-associative)
// ---------------------------------------------------------------------------
struct TypeEntry { bool valid; type_pred_tag_t tag; br_type_t type; type_pred_conf_t conf; type_pred_age_t age; bool rvc; };   // rvc: the branch is a 16-bit instruction (RAS push value pc + 2)
struct TypeWr { bool en = false; uint32_t bank = 0, set = 0, way = 0; TypeEntry e{}; };

class TypePred {
  TypeEntry t_[NB][TYPE_PRED_SET_NUM][TYPE_PRED_WAY_NUM];
  static void locate(pc_t pc, uint32_t &bank, uint32_t &set, type_pred_tag_t &tag) {
    bank = bank_of(pc);
    const uint32_t w = static_cast<uint32_t>(bank_pc_of(pc) >> SLOT_SHIFT);
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
  // Pipelined access: read one set (S1), compare tags (S2).
  struct Raw { TypeEntry way[TYPE_PRED_WAY_NUM]; };
  void read(pc_t pc, Raw &r) const {
    uint32_t b, s; type_pred_tag_t tg; locate(pc, b, s, tg);
    for (int i = 0; i < TYPE_PRED_WAY_NUM; i++) r.way[i] = t_[b][s][i];
  }
  static br_type_t select(pc_t pc, const Raw &r) {
    uint32_t b, s; type_pred_tag_t tg; locate(pc, b, s, tg);
    for (auto &e : r.way) if (e.valid && e.tag == tg) return e.type;
    return BR_NONCTL;
  }
  static bool select_rvc(pc_t pc, const Raw &r) {
    uint32_t b, s; type_pred_tag_t tg; locate(pc, b, s, tg);
    for (auto &e : r.way) if (e.valid && e.tag == tg) return e.rvc;
    return false;
  }
  bool predict_rvc(pc_t pc) const {
    uint32_t b, s; type_pred_tag_t tg; locate(pc, b, s, tg);
    for (auto &e : t_[b][s]) if (e.valid && e.tag == tg) return e.rvc;
    return false;
  }

  void update(pc_t pc, br_type_t type, TypeWr &w, bool rvc = false) const {
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
    e.valid = true; e.tag = tg; e.rvc = rvc;
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
// Predecode-check result of the previous cycle (BPO_PDCKPT): the block whose PTAB entry was checked, and when the checker asks for a
// flush, the history / RAS effect of the corrected block (what the predecode stage knows: conditional branches, JAL, return).
struct PdInfo {
  wire1_t valid = false, flush = false;
  uint8_t nbits = 0, bits = 0, ras_op = 0;
  target_addr_t ras_val = 0;
  wire1_t nb_valid = false;   // the checked block had a taken prediction on an instruction that is not a branch: the type predictor entry of nb_pc is stale
  pc_t nb_pc = 0;
};

struct InputPayload {
  wire1_t refetch;
  fetch_addr_t refetch_address;
  wire1_t icache_read_ready;
  // commit-stream update
  pc_t in_update_base_pc[CW];
  wire1_t in_update_is_rvc[CW];   // the committed instruction is 16 bits (RVC=1): its fall-through / return address is pc + 2
  wire1_t in_upd_valid[CW];
  wire1_t in_actual_dir[CW];
  br_type_t in_actual_br_type[CW];
  target_addr_t in_actual_targets[CW];
  wire1_t in_pred_dir[CW];
  wire1_t in_alt_pred[CW];
  pcpn_t in_pcpn[CW];
  tage_tag_t in_tage_tags[CW][TN];
  tage_idx_t in_tage_idxs[CW][TN];
  wire1_t in_loop_used[CW];
  uint8_t ptab_size;  // BPU_PIPE: PTAB occupancy (credit for the blocks in flight)
  PdInfo pd;
  wire1_t rf_valid;    // BPA_REPAIR: the refetch is a backend branch mispredict
  uint16_t rf_blk; uint8_t rf_slot, rf_type; pc_t rf_pc; wire1_t rf_is_rvc;
  wire1_t refetch_pd;  // BPU_PIPE statistics: the refetch comes from the predecode checker (not the back end)
};

struct OutputPayload {
  fetch_addr_t fetch_address;
  wire1_t icache_read_valid;
  fetch_addr_t predict_next_fetch_address;
  wire1_t PTAB_write_enable;
  wire1_t out_pred_dir[FW];
  PredMeta meta;
  pc_t out_pred_base_pc;
  // FE_PAIR: a fetch group of two blocks (fetch across a predicted-taken branch). Fetch side (S0 push): second block of the entry.
  wire1_t pair_valid;
  fetch_addr_t pair_addr2;
  uint8_t pair_a_len, pair_b_len;
  // PTAB side (block leaving S_D): explicit pc of every slot (the second block's instructions follow the first block's taken branch)
  wire1_t ptab_pcs_valid;
  pc_t out_pred_pc[FW];
  wire1_t update_queue_full;  // the update queue drops instead of stalling: always 0
  // free entries of the update queue / uBTB train queue at the start of the cycle (registered): compact ROB commit
  // credits (EXP-028); 255 = no limit
  uint8_t upd_free = 255, tq_free = 255;
};

}  // namespace bpuf

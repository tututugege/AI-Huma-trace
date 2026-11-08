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

// ---- docs/BPU_ACC.md: TAGE parameters (runtime, all defaults = the original predictor) ----------------------------------------------
// BPA_HLEN="8,13,32,119"  history length per tagged table (<= 255 bits)      BPA_TAGW="8,8,8,8"  tag width per table (needs a build with
// -DTAGE_TAG_WIDTH >= the largest)   BPA_CNTW=3  tagged counter width (2..4)   BPA_UW=2  useful width (1..2)   BPA_WEAK=0  1: both weak
// states of the provider count as weak   BPA_UALT=1  number of use-alt counters (indexed by pc)   BPA_ALLOC=0  allocation policy (see Tage::update)
// BPA_URST=11  useful-bit reset sweep: one row every 2^N updates   BPA_ALTUPD=0  1: a weak new provider also trains the alternate prediction
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
  int cntw = 3, uw = 2, weak2 = 0, ualt = 1, alloc = 0, urst = 11, altupd = 0, cmax = 7, chalf = 4;
  int hlen_max = 0;
  int sc = 0, scn = 256, scw = 6, scth = 8, sct = 3;   // BPA_SC (0 off, 1 always, 2 only when the provider is weak / base), BPA_SCN, BPA_SCW, BPA_SCTH, BPA_SCT (history tables used: 1..3)
  int scmax = 31, scmin = -32;
  TageCfg() {
    const std::vector<int> dh = (TN == 4) ? std::vector<int>{8, 13, 32, 119} : (TN == 5) ? std::vector<int>{5, 9, 17, 40, 119} : (TN == 6) ? std::vector<int>{4, 7, 12, 22, 42, 119} : (TN == 3) ? std::vector<int>{8, 24, 119} : (TN == 2) ? std::vector<int>{13, 119} : std::vector<int>(TN, 32);
    const std::vector<int> h = env_list("BPA_HLEN", dh), tw = env_list("BPA_TAGW", std::vector<int>(TN, TAGE_TAG_WIDTH));
    for (int i = 0; i < TN; i++) {
      ghr[i] = h[i]; tagw[i] = tw[i];
      if (ghr[i] < 1 || ghr[i] > 255 || int(tagw[i]) > TAGE_TAG_WIDTH || tagw[i] < 2) { std::printf("[BPA] bad BPA_HLEN / BPA_TAGW (tag width <= %d)\n", TAGE_TAG_WIDTH); std::exit(1); }
      tagmask[i] = (1u << tagw[i]) - 1;
      if (int(ghr[i]) > hlen_max) hlen_max = ghr[i];
      // folded lengths: index fold 11 b (or the history length if shorter), tag folds = tag width and tag width - 1 (8 and 7 with the default 8-bit tags)
      fl[0][i] = ghr[i] < 11 ? ghr[i] : 11;
      fl[1][i] = ghr[i] < tagw[i] ? ghr[i] : tagw[i];
      fl[2][i] = ghr[i] < tagw[i] - 1 ? ghr[i] : tagw[i] - 1;
    }
    cntw = env_int("BPA_CNTW", 3); uw = env_int("BPA_UW", 2); weak2 = env_int("BPA_WEAK", 0); ualt = env_int("BPA_UALT", 1);
    alloc = env_int("BPA_ALLOC", 0); urst = env_int("BPA_URST", 11); altupd = env_int("BPA_ALTUPD", 0);
    if (cntw < 2 || cntw > 4 || uw < 1 || uw > 2 || ualt < 1 || (ualt & (ualt - 1))) { std::printf("[BPA] bad BPA_CNTW / BPA_UW / BPA_UALT\n"); std::exit(1); }
    cmax = (1 << cntw) - 1; chalf = 1 << (cntw - 1);
    sc = env_int("BPA_SC", 0); scn = env_int("BPA_SCN", 256); scw = env_int("BPA_SCW", 6); scth = env_int("BPA_SCTH", 8); sct = env_int("BPA_SCT", 3);
    if (sc && (scn < 2 || scn > 1024 || (scn & (scn - 1)) || scw < 3 || scw > 7 || sct < 1 || sct > 3 || TN < 3)) { std::printf("[BPA] bad BPA_SCN / BPA_SCW / BPA_SCT\n"); std::exit(1); }
    scmax = (1 << (scw - 1)) - 1; scmin = -(1 << (scw - 1));
  }
};
// BPA_ITT: history-indexed indirect target tables (ITTAGE-like) added to the BTB bank. BPA_ITTH="2" (or "1,3"): the TAGE history table(s) whose index/tag
// are reused (ascending history length), BPA_ITTN=256 entries per table and bank, BPA_ITTCONF=0 minimum confidence to use a hit.
constexpr int ITT_MAX = 3, ITT_NMAX = 1024;
struct IttCfg {
  int n = 0, size = 256, minconf = 0, h[ITT_MAX] = {};
  IttCfg() {
    const std::vector<int> l = env_list("BPA_ITTH", {});
    const char *v = std::getenv("BPA_ITTH");
    if (!v) { n = 1; h[0] = 1; }   // default (RTL-aligned configuration "S+R"): one table on TAGE history 1 (13 bits); BPA_ITTH="" turns it off
    else if (*v) { n = 0; for (const char *c = v; *c;) { if (n < ITT_MAX) h[n++] = std::atoi(c); while (*c && *c != ',') c++; if (*c == ',') c++; } }
    (void)l;
    size = env_int("BPA_ITTN", 256); minconf = env_int("BPA_ITTCONF", 0);
    if (n && (size < 2 || size > ITT_NMAX || (size & (size - 1)))) { std::printf("[BPA] bad BPA_ITTN\n"); std::exit(1); }
    for (int i = 0; i < n; i++) if (h[i] < 0 || h[i] >= TN) { std::printf("[BPA] bad BPA_ITTH\n"); std::exit(1); }
  }
};
inline const IttCfg &icfg() { static const IttCfg c; return c; }
inline const TageCfg &tcfg() { static const TageCfg c; return c; }
inline uint32_t ghr_len(int i) { return tcfg().ghr[i]; }
inline uint32_t fh_len(int k, int i) { return tcfg().fl[k][i]; }

inline int cfg_bubble() { static const int v = env_int("BPU_BUBBLE", 0); return v; }
// BPO_HIST=1 (docs/BPU_OPT.md): the global history holds one symbol of BPO_HSYM bits (folded from pc and target) per TAKEN branch;
// not-taken conditional branches push nothing. Default 0 = one direction bit per conditional branch.
inline int cfg_hist() { static const int v = env_int("BPO_HIST", 1); return v; }
inline int cfg_hsym() { static const int v = env_int("BPO_HSYM", 3); return v; }
inline uint8_t hist_sym(pc_t bpc, pc_t tgt) {
  static const int hs = cfg_hsym();
  const uint32_t v = (bpc >> 2) ^ (tgt >> 2) ^ ((bpc >> 2) >> 7);
  uint32_t r = 0;
  for (int i = 0; i < 32; i += hs) r ^= (v >> i) & ((1u << hs) - 1);
  return uint8_t(r);
}
// docs/BPU_ACC.md diagnostics (default 0 = unchanged behaviour). BPA_NOCMTSYNC=1: the zero-bubble model does not copy the architectural history
// over the speculative one when a mispredicted conditional branch commits (BPU_PIPE has no such copy). BPA_CMTSYNC=1: BPU_PIPE does it.
inline int cfg_nocmtsync() { static const int v = env_int("BPA_NOCMTSYNC", 0); return v; }
inline int cfg_cmtsync() { static const int v = env_int("BPA_CMTSYNC", 0); return v; }
inline uint64_t &pd_flush_count() { static uint64_t n = 0; return n; }   // statistics only: predecode-checker flushes seen by the BPU
// BPA_REPAIR=1: exact history / RAS repair at a backend branch mispredict. The BPU keeps a checkpoint record per fetch block (state before the
// block, predicted types of its slots) keyed by a block sequence number that travels in the PredMeta to the back end FTQ; the back end returns it with
// the slot and type of the mispredicted branch, and the BPU rebuilds the history / RAS as: checkpoint + earlier (not taken) slots + the branch's actual outcome.
// Default 0: spec history / RAS are copied from the committed state (which lacks every older in-flight branch, call and return).
inline int cfg_repair() { static const int v = env_int("BPA_REPAIR", 3); return v; }
inline uint64_t *repair_stats() { static uint64_t n[2] = {0, 0}; return n; }   // [0] repaired, [1] record not found / mismatch
inline int cfg_notc() { static const int v = env_int("BPA_NOTC", 1); return v; }   // 1: the BHT-indexed target cache / pre-TC are not used (indirect targets: ITT, else BTB)
// BPU_ICALL=1: a jalr with rd = ra that is not a return is a call (BR_ICALL: indirect target + RAS push). 0 = such a jalr is BR_IDIRECT (old design).
inline int cfg_icall() { static const int v = env_int("BPU_ICALL", 1); return v; }
inline bool br_ind(int t) { return t == BR_IDIRECT || t == BR_ICALL; }
inline bool br_call(int t) { return t == BR_CALL || t == BR_ICALL; }
inline int cfg_ubtb() { static const int v = env_int("BPU_UBTB", 0); return v; }

// statistics-only prediction record of one slot (BPA_STATS): predicted type (3 b), target source (3 b, see Btb::select; 7 = RAS), use-alt (1 b)
inline uint8_t ps_pack(int ty, int src, bool used_alt) { return uint8_t((ty & 7) | ((src & 7) << 3) | (used_alt ? 0x40 : 0)); }

inline uint32_t bank_of(pc_t pc) { return (pc >> 2) % NB; }
inline pc_t bank_pc_of(pc_t pc) {
  if ((NB & (NB - 1)) != 0) return ((pc >> 2) / NB) << 2;
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
  uint8_t sc_we = 0;                 // BPA_SC: bits 0..2 history tables, bit 3 bias
  uint32_t sc_i[3] = {}, sc_bi = 0;
  int8_t sc_v[3] = {}, sc_bv = 0;
  bool sc_bt = false;
};

struct TagePred {
  bool pred, alt;
  uint8_t pcpn, altpcpn;
  tage_tag_t tag[TN];
  tage_idx_t idx[TN];
  bool used_alt;   // statistics only (BPA_STATS): the weak-provider use-alt rule replaced the provider's prediction
  bool sc_used;    // BPA_SC: the statistical corrector overrode the TAGE prediction (travels in the meta as sc_used)
};

struct TageUpd {
  pc_t bank_pc;
  bool real_dir, pred, alt;
  int pcpn;
  int altpcpn;
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
  tage_use_alt_ctr_t use_alt_[64];
  int8_t sc_[3][1024];       // BPA_SC: signed counters indexed by the TAGE index of history table j (low bits)
  int8_t scb_[1024][2];      // BPA_SC: bias counters indexed by pc and the TAGE prediction

  static uint8_t inc2(uint8_t v) { return v >= 3 ? 3 : v + 1; }
  static uint8_t dec2(uint8_t v) { return v == 0 ? 0 : v - 1; }
  static uint8_t incu(uint8_t v) { const uint8_t m = uint8_t((1u << tcfg().uw) - 1); return v >= m ? m : v + 1; }
  static uint8_t decu(uint8_t v) { return v == 0 ? 0 : v - 1; }
  static bool taken(uint8_t c) { return c >= tcfg().chalf; }
  static bool weak(uint8_t c) { const TageCfg &k = tcfg(); return c == k.chalf - 1 || (k.weak2 && c == k.chalf); }
  static uint8_t cinc(uint8_t c) { return c >= tcfg().cmax ? c : c + 1; }
  static uint8_t cdec(uint8_t c) { return c == 0 ? 0 : c - 1; }
  static constexpr tage_use_alt_ctr_t kUaInit = static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_INIT);
  tage_use_alt_ctr_t ua(uint32_t base_idx) const { return use_alt_[base_idx & (tcfg().ualt - 1)]; }
  static int sc_sum(const int8_t c[3], int8_t b) { int s = 2 * b + 1; for (int j = 0; j < tcfg().sct; j++) s += 2 * c[j] + 1; return s; }
  // decide the override: returns true and flips pred when the corrector disagrees with a confident enough margin
  static bool sc_apply(const int8_t c[3], int8_t b, bool provider_weak, bool &pred) {
    const TageCfg &K = tcfg();
    if (!K.sc) return false;
    if (K.sc == 2 && !provider_weak) return false;
    const int sum = sc_sum(c, b);
    const bool sp = sum >= 0;
    if (sp != pred && (sum >= 0 ? sum : -sum) >= K.scth) { pred = sp; return true; }
    return false;
  }

 public:
  struct Idx { uint32_t base; tage_idx_t idx[TN]; tage_tag_t tag[TN]; };
  struct Raw { uint8_t base_cnt; tage_tag_t tag[TN]; uint8_t cnt[TN], useful[TN]; int8_t scc[3]; int8_t scb[2]; };
  Tage() { std::memset(this, 0, sizeof(*this)); lsfr_ = 8; for (auto &u : use_alt_) u = kUaInit; }
  static bool provider_weak(int pcpn, uint8_t cnt) { return pcpn >= TN || weak(cnt); }

  void predict(pc_t pc, const uint32_t fh[FH_N_MAX][TN], TagePred &o) const {
    const uint32_t w = pc >> 2;
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
    o.altpcpn = altpcpn;
    o.used_alt = false; o.sc_used = false;
    if (pcpn >= TN) { o.pred = base_pred; sc_stage(o, b, 0, nullptr); return; }
    o.pred = taken(cnt[pcpn]);
    if (weak(cnt[pcpn]) && useful[pcpn] <= TAGE_USE_ALT_USEFUL_THRESHOLD &&
        ua(b) <= static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_USE_ALT_THRESHOLD))
    { o.pred = o.alt; o.used_alt = true; }
    sc_stage(o, b, pcpn < TN ? cnt[pcpn] : 0, nullptr);
  }
  // corrector on top of the TAGE prediction in o (reads the counters directly, or the S1 copy in raw)
  void sc_stage(TagePred &o, uint32_t base_idx, uint8_t pcnt, const Raw *raw) const {
    o.sc_used = false;
    if (!tcfg().sc) return;
    const TageCfg &K = tcfg();
    int8_t c[3] = {0, 0, 0}, b;
    if (raw) { for (int j = 0; j < 3; j++) c[j] = raw->scc[j]; b = raw->scb[o.pred]; }
    else { for (int j = 0; j < K.sct; j++) c[j] = sc_[j][o.idx[j] & (K.scn - 1)]; b = scb_[base_idx & (K.scn - 1)][o.pred]; }
    bool p = o.pred;
    if (sc_apply(c, b, provider_weak(o.pcpn, pcnt), p)) { o.pred = p; o.sc_used = true; }
  }

  // Pipelined access (BPU_PIPE): index generation (comb, S0), array read (S1), compare/select (comb, S2).
  static void index(pc_t pc, const uint32_t fh[FH_N_MAX][TN], Idx &o) {
    const uint32_t w = pc >> 2;
    o.base = w & TAGE_BASE_IDX_MASK;
    for (int i = 0; i < TN; i++) {
      o.tag[i] = (fh[1][i] ^ fh[2][i] ^ w) & tcfg().tagmask[i];
      o.idx[i] = (fh[0][i] ^ w) & TAGE_IDX_MASK;
    }
  }
  void read(const Idx &x, Raw &r) const {  // 1 read port: base[1], tag/cnt/useful[TN]
    r.base_cnt = base_[x.base];
    for (int i = 0; i < TN; i++) { r.tag[i] = tag_[i][x.idx[i]]; r.cnt[i] = cnt_[i][x.idx[i]]; r.useful[i] = useful_[i][x.idx[i]]; }
    if (tcfg().sc) { const TageCfg &K = tcfg(); for (int j = 0; j < 3; j++) r.scc[j] = j < K.sct ? sc_[j][x.idx[j] & (K.scn - 1)] : 0; r.scb[0] = scb_[x.base & (K.scn - 1)][0]; r.scb[1] = scb_[x.base & (K.scn - 1)][1]; }
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
    o.altpcpn = altpcpn;
    o.used_alt = false; o.sc_used = false;
    if (pcpn >= TN) { o.pred = base_pred; sc_stage(o, x.base, 0, &r); return; }
    o.pred = taken(r.cnt[pcpn]);
    if (weak(r.cnt[pcpn]) && r.useful[pcpn] <= TAGE_USE_ALT_USEFUL_THRESHOLD &&
        ua(x.base) <= static_cast<tage_use_alt_ctr_t>(TAGE_USE_ALT_CTR_USE_ALT_THRESHOLD))
    { o.pred = o.alt; o.used_alt = true; }
    sc_stage(o, x.base, r.cnt[pcpn], &r);
  }

  void update(const TageUpd &u, TageWr &w) const {
    const TageCfg &K = tcfg();
    w = TageWr{};
    w.upd = true;
    const bool dir = u.real_dir;
    const int pcpn = u.pcpn;
    const bool tpred = (K.sc && u.sc_used) ? !u.pred : u.pred;   // the TAGE prediction before the corrector
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
      const bool praw = taken(cnt[pcpn]);
      const bool provider_used = !(u.sc_used || u.loop_used);
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
      if (K.altupd && weak(cnt[pcpn]) && useful[pcpn] == 0) {   // BPA_ALTUPD: a weak, never-useful provider also trains the alternate prediction
        if (u.altpcpn < TN) { w.cnt_we |= 1u << u.altpcpn; w.cnt_v[u.altpcpn] = dir ? cinc(cnt[u.altpcpn]) : cdec(cnt[u.altpcpn]); }
        else { w.base_we = true; w.base_idx = bidx; w.base_val = dir ? inc2(base_cnt) : dec2(base_cnt); }
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
        if (K.alloc == 1) alloc_at(j);                       // always the lowest free table
        else if (K.alloc == 2) { alloc_at(j); if (k >= 0) alloc_at(k); }   // up to two tables
        else alloc_at((k >= 0 && rnd % 3 == 0) ? k : j);
      }
    }

    if (K.sc) {   // corrector training: recompute its sum from the current counters, train on a miss or a small margin
      int8_t c[3] = {0, 0, 0};
      for (int j = 0; j < K.sct; j++) { w.sc_i[j] = u.idx[j] & (K.scn - 1); c[j] = sc_[j][w.sc_i[j]]; }
      w.sc_bi = bidx & (K.scn - 1); w.sc_bt = tpred;
      const int8_t b = scb_[w.sc_bi][tpred];
      const int sum = sc_sum(c, b);
      const bool sp = sum >= 0;
      if (sp != dir || (sum >= 0 ? sum : -sum) < K.scth * 2) {
        auto step = [&](int8_t v) { return int8_t(dir ? (v < K.scmax ? v + 1 : v) : (v > K.scmin ? v - 1 : v)); };
        for (int j = 0; j < K.sct; j++) { w.sc_v[j] = step(c[j]); w.sc_we |= 1u << j; }
        w.sc_bv = step(b); w.sc_we |= 8;
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
    for (int j = 0; j < 3; j++) if (w.sc_we >> j & 1) sc_[j][w.sc_i[j]] = w.sc_v[j];
    if (w.sc_we & 8) scb_[w.sc_bi][w.sc_bt] = w.sc_bv;
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
struct IttEnt { target_addr_t target; uint16_t tag; uint8_t conf, useful; bool valid; };
struct BtbWr {
  uint8_t itt_we = 0; uint32_t itt_idx[ITT_MAX] = {}; IttEnt itt_v[ITT_MAX] = {};
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
    if (br_ind(t)) {
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
  // ITT lookup: the longest-history hit (index and tag come from the TAGE index generation, tags/indices carried in the prediction meta)
  const IttEnt *itt_hit(const tage_idx_t *idx, const tage_tag_t *tag, int *which = nullptr) const {
    const IttCfg &c = icfg();
    for (int k = c.n - 1; k >= 0; k--) {
      const IttEnt &e = itt_[k][idx[c.h[k]] & (c.size - 1)];
      if (e.valid && e.tag == tag[c.h[k]]) { if (which) *which = k; return &e; }
    }
    return nullptr;
  }

  // Pipelined access: one read of the BTB set, BHT/mode, then the (BHT-indexed) TC / pre-TC sets; select is pure compare.
  struct Raw { Way btb[BTB_WAY_NUM]; bht_hist_t bht; wire3_t mode; Tc tc[TC_WAY_NUM], pre[TC_WAY_NUM]; IttEnt it[ITT_MAX]; };
  void read(pc_t pc, Raw &r, const tage_idx_t *tidx = nullptr) const {
    if (tidx) for (int k = 0; k < icfg().n; k++) r.it[k] = itt_[k][tidx[icfg().h[k]] & (icfg().size - 1)];
    const uint32_t bi = bht_idx(pc);
    for (int w = 0; w < BTB_WAY_NUM; w++) r.btb[w] = btb_[btb_idx(pc)][w];
    r.bht = bht_[bi]; r.mode = mode_[bi];
    const uint32_t ti = tc_idx(pc, r.bht);
    for (int w = 0; w < TC_WAY_NUM; w++) { r.tc[w] = tc_[ti][w]; r.pre[w] = pre_[ti][w]; }
  }
  // src (statistics only): 1 BTB hit, 2 BTB miss (pc+4), 3 TC, 4 pre-TC, 5 indirect fallback to BTB, 6 indirect miss (pc+4)
  static target_addr_t select(pc_t pc, br_type_t type, const Raw &r, int *src = nullptr, const tage_tag_t *ttag = nullptr, bool *itt = nullptr) {
    const int h = hit_way(r.btb, btb_tag(pc));
    if (br_ind(type)) {
      if (ttag) for (int k = icfg().n - 1; k >= 0; k--)
        if (r.it[k].valid && r.it[k].tag == ttag[icfg().h[k]] && r.it[k].conf >= icfg().minconf) { if (itt) *itt = true; if (src) *src = 3; return r.it[k].target; }
      const tc_tag_t tg = tc_tag(pc);
      const int a = cfg_notc() ? -1 : hit_way(r.tc, tg), p = cfg_notc() ? -1 : hit_way(r.pre, tg);
      if (p >= 0 && (a < 0 || r.mode >= kModeBias)) { if (src) *src = 4; return r.pre[p].target; }
      if (a >= 0) { if (src) *src = 3; return r.tc[a].target; }
      if (src) *src = h >= 0 ? 5 : 6;
      return h >= 0 ? r.btb[h].bta : pc + 4;
    }
    if (type == BR_DIRECT || type == BR_CALL || type == BR_JAL || type == BR_RET) {
      if (src) *src = h >= 0 ? 1 : 2;
      return h >= 0 ? r.btb[h].bta : pc + 4;
    }
    return pc + 4;
  }

  // bank_pc is the bank-local pc, exactly what the RTL hands the bank
  target_addr_t predict(pc_t pc, br_type_t type, int *src = nullptr, const tage_idx_t *tidx = nullptr, const tage_tag_t *ttag = nullptr, bool *itt = nullptr) const {
    const Way(&set)[BTB_WAY_NUM] = btb_[btb_idx(pc)];
    const int h = hit_way(set, btb_tag(pc));
    if (br_ind(type)) {
      if (tidx && icfg().n) { const IttEnt *e = itt_hit(tidx, ttag); if (e && e->conf >= icfg().minconf) { if (itt) *itt = true; if (src) *src = 3; return e->target; } }
      const uint32_t bi = bht_idx(pc), ti = tc_idx(pc, bht_[bi]);
      const tc_tag_t tg = tc_tag(pc);
      const int a = cfg_notc() ? -1 : hit_way(tc_[ti], tg), p = cfg_notc() ? -1 : hit_way(pre_[ti], tg);
      if (p >= 0 && (a < 0 || mode_[bi] >= kModeBias)) { if (src) *src = 4; return pre_[ti][p].target; }
      if (a >= 0) { if (src) *src = 3; return tc_[ti][a].target; }
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
    const uint32_t bi = bht_idx(pc);
    const bht_hist_t cur = bht_[bi];
    const bht_hist_t nxt = type != BR_NONCTL ? bht_next(cur, type, dir, addr) : cur;
    if (type != BR_NONCTL) { w.bht_we = true; w.bht_idx = bi; w.bht_v = nxt; }

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
    if (dir && br_ind(type) && !cfg_notc()) {
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
    for (int k = 0; k < ITT_MAX; k++) if (w.itt_we >> k & 1) itt_[k][w.itt_idx[k]] = w.itt_v[k];
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
// Predecode-check result of the previous cycle (BPO_PDCKPT): the block whose PTAB entry was checked, and when the checker asks for a
// flush, the history / RAS effect of the corrected block (what the predecode stage knows: conditional branches, JAL, return).
struct PdInfo {
  wire1_t valid = false, flush = false;
  uint8_t nbits = 0, bits = 0, ras_op = 0;
  target_addr_t ras_val = 0;
};

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
  uint8_t ptab_size;  // BPU_PIPE: PTAB occupancy (credit for the blocks in flight)
  PdInfo pd;
  wire1_t rf_valid;    // BPA_REPAIR: the refetch is a backend branch mispredict
  uint16_t rf_blk; uint8_t rf_slot, rf_type; pc_t rf_pc;
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
};

struct RecB {   // baseline BPU checkpoint record of one fetch block (BPA_REPAIR)
  Hist h; Ras r; pc_t base = 0; uint8_t ptype[FW] = {}; uint16_t id = 0; bool valid = false;
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
    int bub_next;
    int ub_idx;      // -1 none
    uint32_t ub_tag; pc_t ub_target; bool ub_inval;
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
    // update-path statistics (no architectural effect)
    bool rec_v;
    RecB rec;
    uint8_t st_ctl, st_ctl_drop_same_bank, st_ctl_drop_full, st_ctl_drop_blocked_by_nonctl, st_nonctl_enq;
  };
  // Counters of the one-entry-per-bank update rule (reported at exit).
  struct UpdStats { uint64_t commits = 0, cond = 0, cond_mispred = 0, ctl = 0, ctl_drop_same_bank = 0, ctl_drop_full = 0, ctl_drop_blocked_by_nonctl = 0, nonctl_enq = 0, cycles = 0; };
  static UpdStats &stats() { static UpdStats s; static bool reg = false; if (!reg) { reg = true; std::atexit([] {
      const UpdStats &x = stats();
      std::printf("[BPU_MPKI] commits=%llu cond_branches=%llu cond_mispredicts=%llu\n", (unsigned long long)x.commits,
                  (unsigned long long)x.cond, (unsigned long long)x.cond_mispred);
      std::printf("[BPU_UPD_OLD] control updates=%llu dropped(same bank in a cycle)=%llu dropped(bank queue full)=%llu "
                  "of which blocked by a non-control entry in the same cycle=%llu; non-control entries enqueued=%llu\n",
                  (unsigned long long)x.ctl, (unsigned long long)x.ctl_drop_same_bank, (unsigned long long)x.ctl_drop_full,
                  (unsigned long long)x.ctl_drop_blocked_by_nonctl, (unsigned long long)x.nonctl_enq); }); } return s; }

 private:
  pc_t pc_ = RESET_PC;
  int bub_ = 0;
  struct Ub { bool v = false; uint32_t tag = 0; pc_t target = 0; };
  std::vector<Ub> ub_;
  Hist spec_, arch_;
  Ras spec_ras_, arch_ras_;
  Tage tage_[NB];
  Btb btb_[NB];
  TypePred type_;
  RecB rec_[256];
  uint16_t blk_seq_ = 0;
  QEntry q_[Q_DEPTH][NB];
  int wr_[NB] = {}, rd_[NB] = {}, cnt_[NB] = {};

 public:
  bool bubble() const { return bub_ > 0; }  // the front end must not run the BPU this cycle
  void reset() {  // TAGE/BTB arrays keep their contents, as in the RTL-style model
    pc_ = RESET_PC;
    bub_ = 0;
    ub_.assign(cfg_ubtb(), Ub{});
    spec_ = arch_ = Hist{};
    spec_ras_ = arch_ras_ = Ras{};
    type_.reset();
    for (auto &r : rec_) r.valid = false;
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
                           : (ty == BR_RET || ty == BR_CALL || br_ind(ty) || ty == BR_JAL) ? true
                           : tp.pred;
        out.out_pred_dir[i] = taken;
        out.meta.alt_pred[i] = tp.alt;
        out.meta.pcpn[i] = tp.pcpn;
        out.meta.altpcpn[i] = tp.altpcpn;
        for (int k = 0; k < TN; k++) { out.meta.tage_tag[i][k] = tp.tag[k]; out.meta.tage_idx[i][k] = tp.idx[k]; }
        out.meta.ps_info[i] = ps_pack(ty, 0, tp.used_alt);
        out.meta.sc_used[i] = tp.sc_used;
        if (taken && !taken_found) {
          int src = 0; bool itt = false;
          target_addr_t tgt = btb_[b].predict(bpc, ty, &src, tp.idx, tp.tag, &itt);
          if (ty == BR_RET && ras.count > 0) { tgt = ras.stack[ras.count - 1]; src = 7; }
          out.meta.ps_info[i] = ps_pack(ty, src, tp.used_alt) | (itt ? 0x80 : 0);
          next = tgt;
          taken_found = true;
        }
        if (t.do_spec && !spec_done) {
          if (cfg_hist()) { if (taken) for (int k = 0; k < cfg_hsym(); k++) t.spec_dir[t.spec_n++] = (hist_sym(pc, next) >> k) & 1; }
          else if (ty == BR_DIRECT) t.spec_dir[t.spec_n++] = taken;
          if (taken) {
            spec_done = true;
            if (br_call(ty)) { t.spec_ras = 1; t.spec_ras_val = pc + 4; }
            else if (ty == BR_RET) t.spec_ras = 2;
          }
        }
      }
    }
    out.predict_next_fetch_address = next;
    if (cfg_repair() && go_pred && !refetch) {
      t.rec_v = true;
      t.rec.h = spec_; t.rec.r = spec_ras_; t.rec.base = base; t.rec.id = blk_seq_; t.rec.valid = true;
      for (int i = 0; i < FW; i++) { out.meta.blk_id[i] = blk_seq_; t.rec.ptype[i] = out.meta.ps_info[i] & 7; }
    }
    t.pc_next = refetch ? in.refetch_address : go_pred ? next : pc_;
    t.bub_next = (refetch || bub_ <= 0) ? 0 : bub_ - 1;
    t.ub_idx = -1;
    if (cfg_bubble() > 0 && go_pred && !refetch) {
      bool hit_ok = false;
      const int N = cfg_ubtb();
      if (N > 0) {
        const uint32_t w = base >> 3, idx = w % N, tag = (w / N) & 0xfff;
        const Ub &u = ub_.empty() ? Ub{} : ub_[idx];
        const bool hit = u.v && u.tag == tag;
        t.ub_idx = idx; t.ub_tag = tag;
        if (next != boundary) { hit_ok = hit && u.target == next; t.ub_target = next; t.ub_inval = false; }
        else { hit_ok = !hit; t.ub_inval = true; }   // main says fall-through: a stale uBTB hit costs a bubble and is dropped
        if (next == boundary && !hit) t.ub_idx = -1;
      }
      if (next != boundary && !hit_ok) t.bub_next = cfg_bubble();
      else if (next == boundary && N > 0 && !hit_ok) t.bub_next = cfg_bubble();
    }

    // update path: queue head -> TAGE/BTB; commit stream -> type predictor and queue
    for (int b = 0; b < NB; b++) {
      if (cnt_[b] == 0) continue;
      const QEntry &e = q_[rd_[b]][b];
      t.pop[b] = true;
      const pc_t bpc = bank_pc_of(e.pc);
      if (e.type == BR_DIRECT) {
        TageUpd u;
        u.bank_pc = bpc; u.real_dir = e.dir; u.pred = e.pred; u.alt = e.alt;
        u.pcpn = e.pcpn; u.altpcpn = e.altpcpn;
        for (int k = 0; k < TN; k++) { u.tag[k] = e.tag[k]; u.idx[k] = e.idx[k]; }
        u.sc_used = e.sc_used; u.loop_used = e.loop_used;
        tage_[b].update(u, t.tage[b]);
      }
      if (e.type != BR_NONCTL) btb_[b].update(bpc, e.type, e.dir, e.target, t.btb[b], e.idx, e.tag);
    }
    int cnt_next[NB];
    for (int b = 0; b < NB; b++) cnt_next[b] = cnt_[b] - (t.pop[b] ? 1 : 0);
    bool enq[NB] = {}, enq_nonctl[NB] = {};
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      type_.update(in.in_update_base_pc[i], in.in_actual_br_type[i], t.type[i]);
      const uint32_t b = bank_of(in.in_update_base_pc[i]);
      const bool is_ctl = in.in_actual_br_type[i] != BR_NONCTL;
      t.st_ctl += is_ctl;
      if (enq[b] || cnt_next[b] >= Q_DEPTH) {
        if (is_ctl) {
          if (enq[b]) { t.st_ctl_drop_same_bank++; t.st_ctl_drop_blocked_by_nonctl += enq_nonctl[b]; }
          else t.st_ctl_drop_full++;
        }
        continue;
      }
      enq[b] = true; enq_nonctl[b] = !is_ctl; t.st_nonctl_enq += !is_ctl;
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

  // Fast-warmup replay (EXP-011): one committed instruction in program order. The update path (type predictor,
  // TAGE, BTB/indirect target cache, histories, RAS) is applied immediately instead of through the update queue; the
  // prediction metadata comes from a predict() on the architectural history. warm_finish() copies the committed
  // history to the speculative one.
  void warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target) {
    const br_type_t pty = type_.predict(pc);
    if (type == BR_NONCTL) {
      if (pty != BR_NONCTL) { TypeWr tw; type_.update(pc, type, tw); type_.commit(tw); }
      return;
    }
    const uint32_t b = bank_of(pc);
    const pc_t bpc = bank_pc_of(pc);
    TagePred tp;
    tage_[b].predict(bpc, arch_.fh, tp);
    const bool pred = pty == BR_NONCTL ? false
                      : (pty == BR_RET || pty == BR_CALL || br_ind(pty) || pty == BR_JAL) ? true : tp.pred;
    TypeWr tw; type_.update(pc, type, tw); type_.commit(tw);
    if (cfg_hist() && dir) { const uint8_t sy = hist_sym(pc, target); for (int k = 0; k < cfg_hsym(); k++) arch_.push((sy >> k) & 1); }
    if (type == BR_DIRECT) {
      TageUpd u;
      std::memset(&u, 0, sizeof(u));
      u.bank_pc = bpc; u.real_dir = dir; u.pred = pred; u.alt = tp.alt; u.pcpn = tp.pcpn; u.altpcpn = tp.altpcpn;
      for (int k = 0; k < TN; k++) { u.tag[k] = tp.tag[k]; u.idx[k] = tp.idx[k]; }
      TageWr w; tage_[b].update(u, w); tage_[b].commit(w);
      if (!cfg_hist()) arch_.push(dir);
    } else if (br_call(type)) arch_ras_.push(pc + 4);
    else if (type == BR_RET) arch_ras_.pop();
    BtbWr bw; btb_[b].update(bpc, type, dir, target, bw, tp.idx, tp.tag); btb_[b].commit(bw);
  }
  void warm_finish() { spec_ = arch_; spec_ras_ = arch_ras_; }


  void commit(const InputPayload &in, const Txn &t) {
    { UpdStats &x = stats(); x.ctl += t.st_ctl; x.ctl_drop_same_bank += t.st_ctl_drop_same_bank; x.ctl_drop_full += t.st_ctl_drop_full;
      x.ctl_drop_blocked_by_nonctl += t.st_ctl_drop_blocked_by_nonctl; x.nonctl_enq += t.st_nonctl_enq; }
    for (int b = 0; b < NB; b++) { tage_[b].commit(t.tage[b]); btb_[b].commit(t.btb[b]); }
    for (int i = 0; i < CW; i++) type_.commit(t.type[i]);

    for (int i = 0; i < t.spec_n; i++) spec_.push(t.spec_dir[i]);
    if (t.spec_ras == 1) spec_ras_.push(t.spec_ras_val);
    else if (t.spec_ras == 2) spec_ras_.pop();
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const br_type_t ty = in.in_actual_br_type[i];
      stats().commits++;
      if (ty == BR_DIRECT) { stats().cond++; stats().cond_mispred += in.in_pred_dir[i] != in.in_actual_dir[i]; }
      if (cfg_hist() && ty != BR_NONCTL && in.in_actual_dir[i]) {
        const uint8_t sy = hist_sym(in.in_update_base_pc[i], in.in_actual_targets[i]);
        for (int k = 0; k < cfg_hsym(); k++) arch_.push((sy >> k) & 1);
      }
      if (ty == BR_DIRECT) {
        if (!cfg_hist()) arch_.push(in.in_actual_dir[i]);
        if (in.in_pred_dir[i] != in.in_actual_dir[i] && !cfg_nocmtsync()) spec_ = arch_;
      } else if (br_call(ty)) arch_ras_.push(in.in_update_base_pc[i] + 4);
      else if (ty == BR_RET) arch_ras_.pop();
    }
    if (in.refetch) {
      bool done = false;
      if (cfg_repair() && in.rf_valid) {
        const RecB &rc = rec_[in.rf_blk & 255];
        if (rc.valid && rc.id == in.rf_blk && rc.base + 4u * in.rf_slot == in.rf_pc) {
          spec_ = rc.h; spec_ras_ = cfg_repair() == 3 ? arch_ras_ : rc.r;   // 3: history repair only, RAS from the committed state
          for (int i = 0; i < in.rf_slot; i++) if (!cfg_hist() && rc.ptype[i] == BR_DIRECT) spec_.push(false);
          const bool tk = in.rf_type == BR_DIRECT ? (in.refetch_address != in.rf_pc + 4) : true;
          if (cfg_hist()) { if (tk) { const uint8_t sy = hist_sym(in.rf_pc, in.refetch_address); for (int k = 0; k < cfg_hsym(); k++) spec_.push((sy >> k) & 1); } }
          else if (in.rf_type == BR_DIRECT) spec_.push(tk);
          if (br_call(in.rf_type)) spec_ras_.push(in.rf_pc + 4); else if (in.rf_type == BR_RET) spec_ras_.pop();
          done = true; repair_stats()[0]++;
        } else repair_stats()[1]++;
      }
      if (!done) { spec_ = arch_; spec_ras_ = arch_ras_; }
      pd_flush_count() += in.refetch_pd;
    }
    if (t.rec_v) { rec_[blk_seq_ & 255] = t.rec; blk_seq_++; }
    pc_ = t.pc_next;
    bub_ = t.bub_next;
    if (t.ub_idx >= 0 && ub_.empty()) ub_.assign(cfg_ubtb(), Ub{});
    if (t.ub_idx >= 0 && !ub_.empty()) {
      Ub &u = ub_[t.ub_idx];
      if (t.ub_inval) u.v = false; else { u.v = true; u.tag = t.ub_tag; u.target = t.ub_target; }
    }

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

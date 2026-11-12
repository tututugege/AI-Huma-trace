// Pipelined branch predictor (BPU_PIPE); the stage-by-stage description the RTL is written from is rtl/docs/analysis/bpu-pipe-rtl.md.
//
//   S0 (1 cycle)   PC register -> uBTB lookup (indexed by fetch-block start) -> next fetch address; speculative history / RAS
//                  push; index generation for the main predictor; the block's fetch address enters the fetch-address FIFO.
//   S1 .. S_D      main predictor (type / TAGE / BTB / TC) on synchronous-read arrays. S1 reads the arrays with the indices
//                  registered at the end of S0; S_D (D = BPU_MAIN_LAT, 1..3) does tag compare / provider select / target mux and
//                  compares the result with what S0 sent. A disagreement is an override: the younger in-flight blocks are
//                  squashed, S0 is redirected, history / RAS are restored from the per-block checkpoint.
//   commit side    commit stream -> block tracker -> uBTB train queue -> uBTB write; commit stream -> table updates.
//
// BPU_PIPE=1  step (a): the main prediction is computed at S0 on the table state of that cycle with the exact history and only
//             delayed to S_D (same predictions as the zero-bubble model).
// BPU_PIPE=2  step (b): real S0 / S1 / S_D split; history and RAS are advanced at S0 from the uBTB guess and repaired on override.
// BPU_PIPE=3  step (c): (b) plus an update queue whose writes are arbitrated against the S1 reads (single-port arrays).
// comb() reads registers and arrays only and returns the next value of every register in a Txn; commit() is the clock edge.
#pragma once
#include "BPU.h"

namespace bpuf {

inline int cfg_pipe() { static const int v = env_int("BPU_PIPE", 3); return v; }
inline int cfg_s0_entries() { static const int v = env_int("BPU_S0_ENTRIES", 128); return v; }
inline int cfg_s0_ways() { static const int v = env_int("BPU_S0_WAYS", 4); return v; }
inline int cfg_main_lat() { static const int v = env_int("BPU_MAIN_LAT", 2); return v; }
inline int cfg_updq() { static const int v = env_int("BPU_UPDQ", 8); return v; }
inline int cfg_upd_ports() { static const int v = env_int("BPU_UPD_PORTS", 2); return v; }
inline int cfg_s0_train() { static const int v = env_int("BPU_S0_TRAIN", 1); return v; }  // 0: uBTB never trained (debug)

// docs/BPU_OPT.md switches (all default off = bpu-pipe behaviour)
inline int cfg_bankblk() { static const int v = env_int("BPO_BANKBLK", 4); return v; }     // N block banks (0: per-slot bank port model)
inline int cfg_upd_merge() { static const int v = env_int("BPO_UPD_MERGE", 1); return v; } // with BANKBLK: updates to one row share a write
inline int cfg_ub_main() { static const int v = env_int("BPO_UB_MAIN", 0); return v; }     // 1: also train uBTB/L1 from S_D result, 2: S_D only
inline int cfg_l1_entries() { static const int v = env_int("BPO_L1_ENTRIES", 512); return v; }
inline int cfg_l1_ways() { static const int v = env_int("BPO_L1_WAYS", 2); return v; }
inline int cfg_ub_tag() { static const int v = env_int("BPO_UB_TAG", 10); return v; }
inline int cfg_l1_hbits() { static const int v = env_int("BPO_L1_HBITS", 0); return v; }
inline int cfg_ub_hbits() { static const int v = env_int("BPO_UB_HBITS", 0); return v; }   // uBTB / L1 index and tag use this many bits of recent history
inline int cfg_loop() { static const int v = env_int("BPO_LOOP", 0); return v; }       // S0 single-block loop exit prediction (needs BPO_HIST=1)
inline int cfg_pdckpt() { static const int v = env_int("BPO_PDCKPT", 1); return v; }   // checkpoint per PTAB entry: exact history / RAS restore at a predecode flush
// FE_RFAST (FRONTEND_NOTES.md): bit0 = S0 issues the first block of a redirect in the refetch cycle itself (instead of one cycle later)
inline int cfg_rfast() { static const int v = env_int("FE_RFAST", CFG_FE_RFAST); return v; }
// FE_BFAST: bit0 = an S1-level (L1) override issues the corrected block in the same cycle (no bubble), bit1 = an S_D override issues the corrected block in the same cycle (1 bubble cycle less)
inline int cfg_bfast() { static const int v = env_int("FE_BFAST", CFG_FE_BFAST); return v; }
// FE_PAIR=1: dual-block fetch. When the S0 guess of a block is a taken branch, the block at its target is looked up in the same cycle
// (second uBTB read, second set of main-predictor reads in S1) and both are fetched as one group (<= FETCH_WIDTH slots in total).
inline int cfg_pair() { static const int v = env_int("FE_PAIR", 0); return v; }
inline int cfg_tq() { static const int v = env_int("BPO_TQ", 4); return v; }


constexpr int MAXW = 4;       // uBTB ways
constexpr int MAXD = 3;       // deepest main predictor (S1..S3)
constexpr int MAXUQ = 16;     // update queue capacity
constexpr int MAXBB = 8;      // block banks
constexpr int PQ = 64;        // PTAB checkpoint records (BPO_PDCKPT)
constexpr int TQ = 16;        // uBTB train queue capacity (BPO_TQ, default 4)
constexpr int MAXP = 4;       // update write ports (drain width) upper bound
constexpr int HR_BITS = 512;  // circular global-history buffer storage; the ring is HR_BITS_USED long (256, or 512 with FE_PAIR: two blocks per cycle keep twice as many speculative history bits in flight)
inline uint32_t hr_mask() { static const uint32_t m = (cfg_pair() ? 512u : 256u) - 1u; return m; }

constexpr uint32_t mask_of(int bits) { return bits >= 32 ? 0xffffffffu : ((1u << bits) - 1u); }
// uBTB entry field widths: every field is truncated to its declared width on write.
constexpr int UB_TAG_BITS = 10, UB_SLOT_BITS = 3, UB_TYPE_BITS = 3, UB_CTR_BITS = 2, UB_ND_BITS = 4, UB_TGT_BITS = 30;

// ---------------------------------------------------------------------------
// Circular global history: bit ring + head pointer + folded histories. Repair restores head and fh; no ghr copy.
// ---------------------------------------------------------------------------
struct HistRing {
  uint64_t bits[HR_BITS / 64] = {};
  uint32_t head = 0;  // 8 b: next write position
  uint32_t fh[FH_N_MAX][TN] = {};
  bool back(uint32_t k) const { const uint32_t p = (head - 1 - k) & hr_mask(); return (bits[p >> 6] >> (p & 63)) & 1; }
  void push(bool dir) {
    for (int k = 0; k < FH_N_MAX; k++)
      for (int i = 0; i < TN; i++) {
        const uint32_t len = fh_len(k, i);
        uint32_t v = fh[k][i];
        const uint32_t hi = (v >> (len - 1)) & 1;
        v = ((v << 1) & ((1u << len) - 1)) | (uint32_t(dir) ^ hi);
        v ^= uint32_t(back(ghr_len(i) - 1)) << (ghr_len(i) % len);
        fh[k][i] = v;
      }
    const uint32_t p = head;
    bits[p >> 6] = (bits[p >> 6] & ~(1ull << (p & 63))) | (uint64_t(dir) << (p & 63));
    head = (head + 1) & hr_mask();
  }
};

// Circular return-address stack: pointer + count; repaired from (ptr, cnt, top value).
struct RasC {
  target_addr_t st[RAS_DEPTH] = {};
  uint8_t ptr = 0, cnt = 0;  // 5 b, 6 b
  target_addr_t top() const { return st[ptr]; }
  void push(target_addr_t v) { ptr = (ptr + 1) % RAS_DEPTH; st[ptr] = v; if (cnt < RAS_DEPTH) cnt++; }
  void pop() { if (cnt > 0) { ptr = (ptr + RAS_DEPTH - 1) % RAS_DEPTH; cnt--; } }
};

// Per fetch block checkpoint, stored next to the block in the stage registers (state before the block's S0 push).
struct Ckpt {
  uint32_t head = 0;                    // 8 b
  uint32_t fh[FH_N_MAX][TN] = {};       // 12 folded histories (<= 11 b each)
  uint8_t ras_ptr = 0, ras_cnt = 0;     // 5 b, 6 b
  target_addr_t ras_top = 0;            // 32 b
  uint8_t lp_cnt = 0;                   // 4 b: consecutive S0 issues of the same single-block loop (BPO_LOOP)
};

// ---------------------------------------------------------------------------
// uBTB (S0): direct mapped, indexed by the fetch-block start; 1 read port (S0), 1 RMW read + 1 write port (training)
// ---------------------------------------------------------------------------
struct UbEnt {
  bool v = false;
  uint16_t tag = 0;       // 10 b
  bool taken = false;     // 1 b: the block ends with a taken branch (else: fall-through block that has conditional branches)
  uint8_t type = 0;       // 3 b
  uint8_t slot = 0;       // 3 b: word offset of the taken branch in the block
  uint8_t ctr = 0;        // 2 b: taken hint (conditional branches)
  uint8_t nd_t = 0;       // 4 b: conditional branches up to and including the taken branch
  uint8_t nd_nt = 0;      // 4 b: conditional branches in the block if it falls through
  uint32_t target = 0;    // 30 b: target word address
  uint8_t trip = 0;       // 4 b (BPO_LOOP): executions of this single-block loop per visit, 0 = unknown
  bool tconf = false;     // 1 b: the same trip count was seen twice in a row
};
enum UbEvKind : uint8_t { UE_TAKEN = 0, UE_FALL = 1, UE_NTSLOT = 2, UE_TRIP = 3 };
struct UbEv { uint8_t kind = 0, src = 0; pc_t start = 0; uint8_t slot = 0, type = 0, nd = 0; uint16_t hh = 0; pc_t target = 0; };

// One set-associative table of UbEnt (the S0 uBTB, and the optional S1 level BPO_L1_*).
struct UbTab {
  int N = 0, W = 1, S = 1, tagb = 10, hb = 0;   // hb: bits of recent history in index and tag
  std::vector<UbEnt> e;
  void init(int n, int w, int tb, int h) { N = n; W = w; S = n > 0 ? n / w : 1; tagb = tb; hb = h; e.assign(n, UbEnt{}); }
  uint32_t index(pc_t base, uint32_t hh = 0) const {
    hh &= (1u << hb) - 1;
    const uint32_t w = base >> 2, k = ceil_log2_u32(S);
    return (w ^ (w >> k) ^ (hh * 0x9e5u)) & (S - 1);
  }
  uint16_t tag(pc_t base, uint32_t hh = 0) const {
    hh &= (1u << hb) - 1;
    return (((base >> 2) >> ceil_log2_u32(S)) ^ (hh << 2)) & mask_of(tagb);
  }
};

// ---------------------------------------------------------------------------
// Per-block pipeline data
// ---------------------------------------------------------------------------
struct S0Guess {
  bool hit = false, taken = false;
  uint8_t slot = 0, type = 0;
  pc_t next = 0;
  uint8_t nbits = 0, bits = 0;   // history bits the block pushes: count 0..8 (4 b), first push = bit 0
  uint8_t ras_op = 0;            // 0 none, 1 push, 2 pop
  target_addr_t ras_val = 0;
};

struct MainOut {                 // result of the compare / select stage
  pc_t next = 0;
  uint8_t slot = 0, type = 0;
  bool taken = false;
  uint8_t nbits = 0, bits = 0, ras_op = 0;
  target_addr_t ras_val = 0;
  bool dir[FW] = {};
  PredMeta meta;
};

struct MainRaw {                 // synchronous-read array outputs, registered between S1 and S_D
  TypePred::Raw ty[FW];
  Tage::Raw tg[FW];
  Btb::Raw bt[FW];
};

struct BlkB {                    // second block of a fetch group (FE_PAIR)
  bool valid = false;
  uint16_t hh = 0;
  pc_t base = 0, bnd = 0;
  S0Guess g;
  Ckpt ck;
  Tage::Idx tix[FW];
  MainRaw raw;
};

struct Blk {
  bool pair = false;             // FE_PAIR: b2 is the second block of the group
  BlkB b2;
  bool valid = false;
  uint16_t hh = 0;               // recent history bits at S0 (low 16), for the uBTB / S1 table index (BPO_UB_HBITS, BPO_L1_HBITS)
  pc_t base = 0, bnd = 0;
  S0Guess g;
  Ckpt ck;
  RasC rf;                       // BPA_REPAIR=2: full RAS copy (ideal checkpoint)
  Tage::Idx tix[FW];
  MainRaw raw;
  MainOut shadow;                // BPU_PIPE=1 only
};

struct Decision {                // outcome of the last stage, computed from registers only
  bool valid = false;            // a block leaves the pipeline: PTAB write
  bool ovr = false;
  uint8_t squash_n = 0;          // younger blocks in flight (already pushed into the fetch-address FIFO)
  Blk blk;
  MainOut out;
  bool pair_unfit = false;
  bool pair = false, ovr_b = false;   // FE_PAIR: the group has a second block that survived the first block's check; the override (if any) is the second block's
  MainOut outB;
  Ckpt ock; MainOut oout; pc_t obase = 0; S0Guess og;   // the overriding block (A, or B of a pair)
};

struct UpdEnt {
  pc_t pc = 0;
  bool dir = false, pred = false, alt = false, loop_used = false;
  br_type_t type = BR_NONCTL;
  target_addr_t target = 0;
  pcpn_t pcpn = 0;
  tage_tag_t tag[TN] = {};
  tage_idx_t idx[TN] = {};
};

struct PortUse { uint8_t rd = 0, wr = 0; };

struct RecP {   // BPA_REPAIR: checkpoint record of one fetch block, written when the block leaves S_D with its PTAB entry
  Ckpt ck; RasC rf; pc_t base = 0; uint8_t ptype[FW] = {}; uint16_t id = 0; uint8_t epoch = 0; bool valid = false;
};

class BpuPipe {
 public:
  struct TrkCnt { uint8_t ev = 0, drop = 0; };
  struct Trk {                         // commit-side block tracker + uBTB train queue
    bool tr_v = false;
    pc_t tr_start = 0, tr_expect = 0;
    uint8_t tr_nd = 0;
    uint16_t tr_hh = 0;
    bool lt_v = false;                 // BPO_LOOP trip tracker: the committed stream is inside a single-block loop
    pc_t lt_start = 0;
    uint8_t lt_slot = 0, lt_cnt = 0;
    uint32_t ah = 0;                   // architectural history, newest bit in bit 0 (only the low BPO_UB_HBITS are used)
    UbEv tq[TQ];
    uint8_t tq_n = 0;
  };
  struct State : Trk {
    pc_t pc = RESET_PC;
    uint8_t lp_cnt = 0;                // S0 loop iteration counter (speculative)
    Ckpt pq[PQ];                       // checkpoints of the blocks in the PTAB, oldest first (BPO_PDCKPT)
    uint8_t pq_n = 0;
    uint16_t blk_seq = 0;              // BPA_REPAIR: next block sequence number
    uint8_t epoch = 0;                 // BPA_REPAIR: bumped whenever spec is overwritten by arch (the history ring is re-based: older checkpoints no longer index it)
    Blk R[MAXD];                       // R[0] = S1 ... R[D-1] = S_D
    HistRing spec, arch;
    RasC sras, aras;
    UpdEnt q_old[Q_DEPTH][NB];         // modes 1, 2: one queue per bank
    uint8_t wr[NB] = {}, rd[NB] = {}, cnt[NB] = {};
    UpdEnt uq[MAXUQ];                  // mode 3
    uint8_t uq_n = 0;
  };

  struct Stats {
    uint64_t cyc = 0, blocks = 0, s0_hit = 0, s0_hit_taken = 0, ovr = 0, ovr_next = 0, ovr_hist = 0, ovr_ras = 0, ovr_miss = 0, ov_t2n = 0, ov_n2t = 0, ov_ind = 0, ov_ret = 0, ov_oth = 0, squashed = 0,
             refetch = 0, s0_stall_ptab = 0, tq_ev = 0, tq_drop = 0, ub_wr = 0, upd_ctl = 0, upd_enq = 0, upd_drop = 0,
             upd_applied = 0, upd_delayed_cycles = 0, uq_occ_sum = 0, uq_max = 0, old_ctl = 0, old_drop_sb = 0, old_drop_full = 0,
             old_drop_nonctl = 0, old_nonctl_enq = 0, max_type_wr_bank = 0, commits = 0, cond = 0, cond_mispred = 0,
             l1_hit = 0, l1_ovr = 0, sd_ev = 0, merged = 0, blocked = 0, ub_wr1 = 0, refetch_pd = 0, pd_restore = 0;
  };

  struct Txn {
    State nx;                          // next value of every pipeline register
    uint32_t ub_wmask[2] = {};         // ways written in the set below: [0] uBTB (S0), [1] S1 level
    uint32_t ub_set[2] = {};
    UbEnt ub_v[2][MAXW];
    uint8_t bb_rd[MAXBB] = {}, bb_wr[MAXBB] = {};  // block-bank port use (BPO_BANKBLK)
    bool rec_v = false, rec2_v = false;                // BPA_REPAIR (rec2: second block of a pair)
    RecP rec, rec2;
    uint8_t s_pair = 0, s_pair_ovr_b = 0, s_pair_unfit = 0;
    TageWr tage[NB];
    BtbWr btb[NB];
    TypeWr type[CW + MAXP];
    uint8_t n_type = 0;
    PortUse p_ty[NB], p_tg[NB], p_bt[NB];
    // statistics increments (no architectural effect)
    uint8_t s_ov_t2n = 0, s_ov_n2t = 0, s_ov_ind = 0, s_ov_ret = 0, s_ov_oth = 0;
    uint8_t s_ovr_next = 0, s_ovr_hist = 0, s_ovr_ras = 0, s_ovr_miss = 0, s_hit = 0, s_hit_taken = 0, s_block = 0, s_stall_ptab = 0;
    TrkCnt tc;
    uint8_t s_upd_ctl = 0, s_upd_enq = 0, s_upd_drop = 0, s_upd_applied = 0, s_delayed = 0;
    uint8_t s_squashed = 0, s_refetch = 0, s_commit = 0, s_cond = 0, s_cond_mp = 0;
    uint8_t s_ctl_drop_sb = 0, s_ctl_drop_full = 0, s_ctl_drop_nonctl = 0, s_nonctl_enq = 0;
    uint8_t s_rep_ok = 0, s_rep_miss = 0, s_pd_restore = 0, s_refetch_pd = 0, s_l1_hit = 0, s_l1_ovr = 0, s_sd_ev = 0, s_merged = 0, s_blocked = 0;
  };

 private:
  int mode_ = 0, D_ = 2, N_ = 128, W_ = 2, S_ = 64, UQ_ = 8, P_ = 2;
  int loop_ = 0, bb_ = 0, merge_ = 1, ubmain_ = 0, tqd_ = 4, hist_ = 0, hsym_ = 2, hb_ = 0, pdck_ = 0, hb1_ = 0;
  State st_;
  UbTab ub0_, ub1_;
  Tage tage_[NB];
  Btb btb_[NB];
  TypePred type_;
  Stats stats_;
  RecP rec_[256];
  int repair_ = 0;
  uint64_t rep_ok_ = 0, rep_miss_ = 0, pair_blocks_ = 0, pair_ovr_b_ = 0, pair_unfit_ = 0;

  static pc_t block_bnd(pc_t base) {
    const uint32_t mask = ~(ICACHE_LINE_SIZE - 1);
    const uint32_t plus = base + FW * 4;
    return (base & mask) != (plus & mask) ? (plus & mask) : plus;
  }
  // 32 B window (one block-bank row, BPO_BANKBLK): the unit of banking by fetch block
  static uint32_t win_of(pc_t pc) { return pc >> 5; }
  static uint32_t bb_of(pc_t pc, int n) { return win_of(pc) % n; }

  static Ckpt take_ckpt(const HistRing &h, const RasC &r) {
    Ckpt c;
    c.head = h.head;
    std::memcpy(c.fh, h.fh, sizeof(c.fh));
    c.ras_ptr = r.ptr; c.ras_cnt = r.cnt; c.ras_top = r.top();
    return c;
  }
  static void restore(const Ckpt &c, HistRing &h, RasC &r) {
    h.head = c.head;
    std::memcpy(h.fh, c.fh, sizeof(h.fh));
    r.ptr = c.ras_ptr; r.cnt = c.ras_cnt; r.st[r.ptr] = c.ras_top;
  }
  uint8_t lp_next(uint8_t c, pc_t base, bool taken, pc_t next, uint8_t type) const {
    if (!loop_) return 0;
    return (taken && type == BR_DIRECT && next == base) ? (c < 15 ? c + 1 : 15) : 0;
  }
  // BPO_HIST=1: a taken branch pushes hsym_ bits folded from its pc and target; not-taken branches push nothing
  uint8_t hsym(pc_t bpc, pc_t tgt) const { return hist_sym(bpc, tgt); }
  uint16_t spec_hh(const HistRing &h) const {
    uint16_t v = 0;
    for (int i = 0; i < hb_; i++) v |= uint16_t(h.back(i)) << i;
    return v;
  }
  void push_arch(HistRing &h, pc_t bpc, bool taken, pc_t tgt, br_type_t ty) const {
    if (!hist_) { if (ty == BR_DIRECT) h.push(taken); return; }
    if (!taken) return;
    const uint8_t b = hsym(bpc, tgt);
    for (int i = 0; i < hsym_; i++) h.push((b >> i) & 1);
  }
  static void push_bits(uint8_t n, uint8_t bits, uint8_t ras_op, target_addr_t ras_val, HistRing &h, RasC &r) {
    for (int i = 0; i < n; i++) h.push((bits >> i) & 1);
    if (ras_op == 1) r.push(ras_val); else if (ras_op == 2) r.pop();
  }

  // ---- S0 -------------------------------------------------------------------------------------------------------------
  // One set read; among the ways that match the block start, the lowest-slot branch that is predicted taken wins.
  S0Guess s0_lookup(const UbTab &tb, pc_t base, uint16_t hh, pc_t bnd, uint8_t rcnt, target_addr_t rtop, uint8_t lpc) const {
    S0Guess g;
    g.next = bnd;
    const uint32_t set = tb.index(base, hh);
    const uint16_t tag = tb.tag(base, hh);
    int best = -1;
    for (int w = 0; w < tb.W; w++) {
      const UbEnt &e = tb.e[set * tb.W + w];
      if (!(e.v && e.tag == tag)) continue;
      if (!g.hit) { g.hit = true; g.nbits = hist_ ? 0 : e.nd_nt; }
      bool tk = e.taken && base + 4u * e.slot < bnd && (e.type != BR_DIRECT || e.ctr >= 2);
      if (tk && loop_ && e.type == BR_DIRECT && e.tconf && e.trip && (e.target << 2) == base && uint32_t(lpc) + 1 == e.trip) tk = false;   // predicted loop exit
      if (tk && (best < 0 || e.slot < tb.e[set * tb.W + best].slot)) best = w;
    }
    if (best < 0) return g;
    const UbEnt &e = tb.e[set * tb.W + best];
    const pc_t bpc = base + 4u * e.slot;
    g.taken = true; g.slot = e.slot; g.type = e.type;
    g.next = e.target << 2;
    g.nbits = e.nd_t;
    g.bits = (e.type == BR_DIRECT && e.nd_t > 0) ? (1u << (e.nd_t - 1)) : 0;
    if (e.type == BR_RET && rcnt > 0) g.next = rtop;
    if (hist_) { g.nbits = hsym_; g.bits = hsym(bpc, g.next); }
    if (br_call(e.type)) { g.ras_op = 1; g.ras_val = bpc + 4; }
    else if (e.type == BR_RET) g.ras_op = 2;
    return g;
  }

  // ---- main predictor: index (S0), read (S1), select (S_D) -----------------------------------------------------------------
  template <class B> void main_index(B &b, const HistRing &h) const {
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * 4;
      if (pc >= b.bnd) continue;
      Tage::index(bank_pc_of(pc), h.fh, b.tix[i]);
    }
  }
  template <class B> void main_read(const B &b, MainRaw &r) const {
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * 4;
      if (pc >= b.bnd) continue;
      const uint32_t bk = bank_of(pc);
      type_.read(pc, r.ty[i]); tage_[bk].read(b.tix[i], r.tg[i]); btb_[bk].read(bank_pc_of(pc), r.bt[i], b.tix[i].idx);
    }
  }
  static void count_reads(const Blk &b, Txn &t) {
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * 4;
      if (pc >= b.bnd) continue;
      const uint32_t bk = bank_of(pc);
      t.p_ty[bk].rd++; t.p_tg[bk].rd++; t.p_bt[bk].rd++;
    }
  }
  static uint32_t read_mask(const Blk &b) {
    uint32_t m = 0;
    if (!b.valid) return 0;
    for (int i = 0; i < FW; i++) if (b.base + i * 4 < b.bnd) m |= 1u << bank_of(b.base + i * 4);
    return m;
  }
  // block-bank rows read by the block's S1 access (BPO_BANKBLK): the 32 B windows it covers (1, or 2 if it straddles a window boundary)
  void bb_reads(const Blk &b, Txn &t) const {
    if (!b.valid || !bb_) return;
    const uint32_t w0 = win_of(b.base), w1 = win_of(b.bnd - 4);
    t.bb_rd[w0 % bb_]++;
    if (w1 != w0) t.bb_rd[w1 % bb_]++;
  }
  template <class B> void main_select(const B &b, const MainRaw &r, MainOut &o) const {
    o = MainOut{};
    o.next = b.bnd;
    bool taken_found = false, spec_done = false;
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * 4;
      if (pc >= b.bnd) continue;
      const uint32_t bk = bank_of(pc);
      const pc_t bpc = bank_pc_of(pc);
      const br_type_t ty = TypePred::select(pc, r.ty[i]);
      TagePred tp;
      tage_[bk].select(b.tix[i], r.tg[i], tp);
      const bool taken = ty == BR_NONCTL ? false
                         : (ty == BR_RET || ty == BR_CALL || br_ind(ty) || ty == BR_JAL) ? true : tp.pred;
      o.dir[i] = taken;
      o.meta.alt_pred[i] = tp.alt;
      o.meta.pcpn[i] = tp.pcpn;
      for (int k = 0; k < TN; k++) { o.meta.tage_tag[i][k] = tp.tag[k]; o.meta.tage_idx[i][k] = tp.idx[k]; }
      o.meta.ps_info[i] = ps_pack(ty, 0, tp.used_alt);
      if (taken && !taken_found) {
        int src = 0; bool itt = false;
        target_addr_t tgt = Btb::select(bpc, ty, r.bt[i], &src, tp.tag, &itt);
        if (ty == BR_RET && b.ck.ras_cnt > 0) { tgt = b.ck.ras_top; src = 7; }
        o.meta.ps_info[i] = ps_pack(ty, src, tp.used_alt) | (itt ? 0x80 : 0);
        o.next = tgt; o.slot = i; o.type = ty; o.taken = true;
        taken_found = true;
      }
      if (!spec_done) {
        if (hist_) { if (taken) { o.nbits = hsym_; o.bits = hsym(pc, o.next); } }
        else if (ty == BR_DIRECT) { o.bits |= uint8_t(taken) << o.nbits; o.nbits++; }
        if (taken) {
          spec_done = true;
          if (br_call(ty)) { o.ras_op = 1; o.ras_val = pc + 4; }
          else if (ty == BR_RET) o.ras_op = 2;
        }
      }
    }
  }

  // why: 1 next address, 2 history bits, 3 RAS operation
  bool disagree(const MainOut &m, const S0Guess &g, int &why) const {
    why = 0;
    if (m.next != g.next) { why = 1; return true; }
    if (mode_ == 1) return false;  // exact history / RAS were already applied at S0
    if (m.nbits != g.nbits || m.bits != g.bits) { why = 2; return true; }
    if (m.ras_op != g.ras_op || (m.ras_op == 1 && m.ras_val != g.ras_val)) { why = 3; return true; }
    return false;
  }

  // ---- uBTB training --------------------------------------------------------------------------------------------------------
  void ub_event(Trk &n, TrkCnt &t, const UbEv &e) const {
    t.ev++;
    if (n.tq_n >= tqd_) { t.drop++; return; }
    n.tq[n.tq_n++] = e;
  }
  // Commit-order block tracker: reconstructs the fetch block each committed instruction belonged to.
  // the block that just ended was not a self-loop iteration: if it closes a run of self-loop iterations, report the trip count
  void trip_end(Trk &n, TrkCnt &t) const {
    if (n.lt_v && n.lt_start == n.tr_start && n.lt_cnt >= 1) {
      UbEv e; e.kind = UE_TRIP; e.start = n.lt_start; e.slot = n.lt_slot; e.nd = n.lt_cnt + 1; e.hh = n.tr_hh;
      ub_event(n, t, e);
    }
    n.lt_v = false;
  }
  uint16_t ah_low(const Trk &n) const { return uint16_t(n.ah & ((1u << hb_) - 1)); }
  void ah_push(Trk &n, pc_t pc, bool taken, pc_t tgt, br_type_t ty) const {
    if (!hb_) return;
    if (!hist_) { if (ty == BR_DIRECT) n.ah = (n.ah << 1) | uint32_t(taken); return; }
    if (!taken) return;
    const uint8_t b = hsym(pc, tgt);
    for (int i = 0; i < hsym_; i++) n.ah = (n.ah << 1) | ((b >> i) & 1);
  }
  void ub_track(const InputPayload &in, Trk &n, TrkCnt &t) const {
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const pc_t pc = in.in_update_base_pc[i];
      const br_type_t ty = in.in_actual_br_type[i];
      if (!n.tr_v || pc != n.tr_expect || pc >= block_bnd(n.tr_start)) { n.tr_v = true; n.tr_start = pc; n.tr_nd = 0; n.tr_hh = ah_low(n); }
      const pc_t bnd = block_bnd(n.tr_start);
      const uint8_t slot = (pc - n.tr_start) >> 2;
      if (ty == BR_DIRECT && n.tr_nd < 8) n.tr_nd++;
      const bool taken = ty != BR_NONCTL && in.in_actual_dir[i];
      UbEv e; e.start = n.tr_start; e.slot = slot; e.type = ty; e.nd = n.tr_nd; e.hh = n.tr_hh;
      ah_push(n, pc, taken, in.in_actual_targets[i], ty);   // history after this instruction = history at the start of the next block
      if (taken) {
        e.kind = UE_TAKEN; e.target = in.in_actual_targets[i];
        ub_event(n, t, e);
        if (loop_) {   // single-block loop: the block ends with a taken conditional branch back to its own start
          if (ty == BR_DIRECT && e.target == n.tr_start) {
            if (n.lt_v && n.lt_start == n.tr_start && n.lt_slot == slot) { if (n.lt_cnt < 15) n.lt_cnt++; else n.lt_v = false; }
            else { n.lt_v = true; n.lt_start = n.tr_start; n.lt_slot = slot; n.lt_cnt = 1; }
          } else trip_end(n, t);
        }
        n.tr_start = e.target; n.tr_nd = 0; n.tr_expect = e.target; n.tr_hh = ah_low(n);
      } else {
        const pc_t nxt = pc + 4;
        if (ty == BR_DIRECT && in.in_pred_dir[i]) {   // predicted taken, was not: the front end restarts at pc + 4
          e.kind = UE_NTSLOT; ub_event(n, t, e);
          if (loop_) trip_end(n, t);
          n.tr_start = nxt; n.tr_nd = 0; n.tr_hh = ah_low(n);
        } else if (nxt >= bnd) {                      // sequential block end
          if (n.tr_nd > 0) { e.kind = UE_FALL; ub_event(n, t, e); }
          if (loop_) trip_end(n, t);
          n.tr_start = nxt; n.tr_nd = 0; n.tr_hh = ah_low(n);
        }
        n.tr_expect = nxt;
      }
    }
  }
  static void ub_trunc(UbEnt &e, int tagb) {  // declared widths
    e.tag &= mask_of(tagb); e.type &= mask_of(UB_TYPE_BITS); e.slot &= mask_of(UB_SLOT_BITS); e.ctr &= mask_of(UB_CTR_BITS);
    e.trip &= 15; e.nd_t &= mask_of(UB_ND_BITS); e.nd_nt &= mask_of(UB_ND_BITS); e.target &= mask_of(UB_TGT_BITS);
  }
  static bool ub_same(const UbEnt &a, const UbEnt &b) {
    return a.v == b.v && a.tag == b.tag && a.taken == b.taken && a.type == b.type && a.slot == b.slot && a.ctr == b.ctr &&
           a.nd_t == b.nd_t && a.nd_nt == b.nd_nt && a.target == b.target && a.trip == b.trip && a.tconf == b.tconf;
  }
  // Read-modify-write of one set by the head of the train queue (W ways read, written ways in wmask).
  // Every DIRECT way of the block below the taken slot (all of them for a fall-through block) was not taken: its counter steps down.
  void ub_rmw(const UbTab &tb, const UbEv &ev, UbEnt nw[MAXW], uint32_t &wmask) const {
    const int W_ = tb.W;
    const uint32_t set = tb.index(ev.start, ev.hh);
    const uint16_t tag = tb.tag(ev.start, ev.hh);
    wmask = 0;
    for (int w = 0; w < W_; w++) nw[w] = tb.e[set * W_ + w];
    const int limit = ev.kind == UE_TAKEN ? ev.slot : ev.kind == UE_FALL ? 8 : ev.kind == UE_TRIP ? 0 : ev.slot + 1;
    int same = -1, fall = -1;
    bool any = false;
    uint8_t nd_nt = ev.nd;
    for (int w = 0; w < W_; w++) {
      UbEnt &e = nw[w];
      if (!(e.v && e.tag == tag)) continue;
      any = true; nd_nt = e.nd_nt;
      if (!e.taken) fall = w;
      if (e.taken && e.type == BR_DIRECT && e.slot < limit && !(ev.kind == UE_TAKEN && e.slot == ev.slot)) e.ctr = e.ctr > 0 ? e.ctr - 1 : 0;
      if (ev.kind == UE_FALL) e.nd_nt = ev.nd;
      if (ev.kind == UE_TAKEN && e.taken && e.slot == ev.slot) same = w;
    }
    if (ev.kind == UE_TRIP) {
      for (int w = 0; w < W_; w++) {
        UbEnt &e = nw[w];
        if (e.v && e.tag == tag && e.taken && e.type == BR_DIRECT && e.slot == ev.slot && (e.target << 2) == ev.start) {
          if (e.trip == ev.nd) e.tconf = true; else { e.trip = ev.nd; e.tconf = false; }
        }
      }
    } else if (ev.kind == UE_TAKEN) {
      int w = same;
      if (w < 0) {  // allocate: the fall-through placeholder of this block, an invalid way, else the lowest counter
        w = fall;
        for (int i = 0; w < 0 && i < W_; i++) if (!nw[i].v) w = i;
        if (w < 0) { int m = 99; for (int i = 0; i < W_; i++) if (nw[i].ctr < m) { m = nw[i].ctr; w = i; } }
        nw[w] = UbEnt{};
        nw[w].v = true; nw[w].tag = tag; nw[w].taken = true; nw[w].slot = ev.slot;
        nw[w].ctr = ev.type == BR_DIRECT ? 2 : 3; nw[w].nd_nt = nd_nt;
      } else {
        nw[w].ctr = ev.type == BR_DIRECT ? (nw[w].ctr < 3 ? nw[w].ctr + 1 : 3) : 3;
      }
      if (nw[w].target != (ev.target >> 2)) { nw[w].trip = 0; nw[w].tconf = false; }
      nw[w].type = ev.type; nw[w].nd_t = ev.nd; nw[w].target = ev.target >> 2;
    } else if (ev.kind == UE_FALL && !any && !hist_) {
      int w = -1;
      for (int i = 0; w < 0 && i < W_; i++) if (!nw[i].v) w = i;
      if (w < 0) { int m = 99; for (int i = 0; i < W_; i++) if (nw[i].ctr < m) { m = nw[i].ctr; w = i; } }
      nw[w] = UbEnt{};
      nw[w].v = true; nw[w].tag = tag; nw[w].taken = false; nw[w].nd_nt = ev.nd;
    }
    for (int w = 0; w < W_; w++) { ub_trunc(nw[w], tb.tagb); if (!ub_same(nw[w], tb.e[set * W_ + w])) wmask |= 1u << w; }
  }

  // BPA_REPAIR: rebuild spec history / RAS = checkpoint of the mispredicted branch's block + earlier slots (not taken) + the branch's actual outcome
  bool repair_blk(const InputPayload &in, State &n) const {
    const RecP &rc = rec_[in.rf_blk & 255];
    const uint32_t rslot = (in.rf_pc - rc.base) >> 2;   // slot within the block (the back end's slot is the offset in the fetch group, which differs for the second block of a pair)
    if (!(rc.valid && rc.id == in.rf_blk && rc.epoch == st_.epoch && rslot < FW && rc.base + 4u * rslot == in.rf_pc)) return false;
    restore(rc.ck, n.spec, n.sras);
    if (repair_ == 2) n.sras = rc.rf;
    if (repair_ == 3) n.sras = n.aras;   // history repair only, RAS from the committed state
    for (uint32_t i = 0; i < rslot; i++) if (!hist_ && rc.ptype[i] == BR_DIRECT) n.spec.push(false);
    const bool tk = in.rf_type == BR_DIRECT ? (in.refetch_address != in.rf_pc + 4) : true;
    if (hist_) { if (tk) { const uint8_t sy = hsym(in.rf_pc, in.refetch_address); for (int k = 0; k < hsym_; k++) n.spec.push((sy >> k) & 1); } }
    else if (in.rf_type == BR_DIRECT) n.spec.push(tk);
    if (br_call(in.rf_type)) n.sras.push(in.rf_pc + 4); else if (in.rf_type == BR_RET) n.sras.pop();
    return true;
  }

  // ---- table updates ----------------------------------------------------------------------------------------------------------
  static UpdEnt make_ent(const InputPayload &in, int i) {
    UpdEnt e;
    e.pc = in.in_update_base_pc[i];
    e.dir = in.in_actual_dir[i]; e.pred = in.in_pred_dir[i]; e.alt = in.in_alt_pred[i];
    e.loop_used = in.in_loop_used[i];
    e.type = in.in_actual_br_type[i]; e.target = in.in_actual_targets[i];
    e.pcpn = in.in_pcpn[i];
    for (int k = 0; k < TN; k++) { e.tag[k] = in.in_tage_tags[i][k]; e.idx[k] = in.in_tage_idxs[i][k]; }
    return e;
  }
  void apply_table_update(const UpdEnt &e, Txn &t) const {
    const uint32_t b = bank_of(e.pc);
    const pc_t bpc = bank_pc_of(e.pc);
    if (e.type == BR_DIRECT) {
      TageUpd u;
      u.bank_pc = bpc; u.real_dir = e.dir; u.pred = e.pred; u.alt = e.alt; u.pcpn = e.pcpn;
      for (int k = 0; k < TN; k++) { u.tag[k] = e.tag[k]; u.idx[k] = e.idx[k]; }
      u.loop_used = e.loop_used;
      tage_[b].update(u, t.tage[b]);
      t.p_tg[b].wr++;
    }
    if (e.type != BR_NONCTL) { btb_[b].update(bpc, e.type, e.dir, e.target, t.btb[b], e.idx, e.tag); t.p_bt[b].wr++; }
  }
  void type_update(const UpdEnt &e, Txn &t) const {
    TypeWr &w = t.type[t.n_type];
    type_.update(e.pc, e.type, w);
    if (w.en) { t.p_ty[w.bank].wr++; t.n_type++; }
  }

  // Old rule: one entry per bank, second update to a bank in a cycle is dropped; type predictor updated straight from the commit stream.
  void update_old(const InputPayload &in, State &n, Txn &t) const {
    for (int b = 0; b < NB; b++) {
      if (st_.cnt[b] == 0) continue;
      apply_table_update(st_.q_old[st_.rd[b]][b], t);
      n.rd[b] = (st_.rd[b] + 1) % Q_DEPTH; n.cnt[b]--;
    }
    bool enq[NB] = {}, enq_nonctl[NB] = {};
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const UpdEnt e = make_ent(in, i);
      type_update(e, t);
      const uint32_t b = bank_of(e.pc);
      const bool ctl = e.type != BR_NONCTL;
      t.s_upd_ctl += ctl;
      if (enq[b] || n.cnt[b] >= Q_DEPTH) {
        if (ctl) {
          t.s_upd_drop++;
          if (enq[b]) { t.s_ctl_drop_sb++; t.s_ctl_drop_nonctl += enq_nonctl[b]; } else t.s_ctl_drop_full++;
        }
        continue;
      }
      enq[b] = true; enq_nonctl[b] = !ctl; t.s_nonctl_enq += !ctl;
      n.q_old[n.wr[b]][b] = e;
      n.wr[b] = (n.wr[b] + 1) % Q_DEPTH; n.cnt[b]++;
    }
  }
  // Update queue: control updates only; an entry is written when its bank has no S1 read this cycle (reads win), at most P_ writes
  // per cycle, one per bank. A full queue drops the new entry.
  void update_queue(const InputPayload &in, State &n, Txn &t) const {
    const uint32_t rdm = read_mask(st_.R[0]);
    uint8_t bbr[MAXBB] = {};
    if (bb_ && st_.R[0].valid) { const Blk &b = st_.R[0]; bbr[win_of(b.base) % bb_] = 1; bbr[win_of(b.bnd - 4) % bb_] = 1; }
    uint32_t wrm = 0;
    int nw = 0, kn = 0;
    UpdEnt keep[MAXUQ];
    bool done[MAXUQ] = {};
    for (int qi = 0; qi < st_.uq_n; qi++) {
      if (done[qi]) continue;
      const UpdEnt &e = st_.uq[qi];
      const uint32_t b = bank_of(e.pc);
      if (!bb_) {
        if (nw < P_ && !((rdm >> b) & 1) && !((wrm >> b) & 1)) {
          apply_table_update(e, t); type_update(e, t);
          wrm |= 1u << b; nw++; t.s_upd_applied++;
        } else {
          keep[kn++] = e;
          t.s_delayed++;
        }
        continue;
      }
      // block banks: one row write per (window bank) and cycle, reads win, entries of the same row (same window, same TAGE index set) share the write
      const uint32_t wb = bb_of(e.pc, bb_);
      if (nw < P_ && !bbr[wb] && !t.bb_wr[wb] && !((wrm >> b) & 1)) {
        apply_table_update(e, t); type_update(e, t);
        wrm |= 1u << b; nw++; t.bb_wr[wb] = 1; t.s_upd_applied++;
        if (merge_) {
          for (int qj = qi + 1; qj < st_.uq_n; qj++) {
            if (done[qj]) continue;
            const UpdEnt &f = st_.uq[qj];
            const uint32_t fb = bank_of(f.pc);
            if (win_of(f.pc) != win_of(e.pc) || ((wrm >> fb) & 1) || std::memcmp(f.idx, e.idx, sizeof(e.idx)) != 0) continue;
            apply_table_update(f, t); type_update(f, t);
            wrm |= 1u << fb; done[qj] = true; t.s_upd_applied++; t.s_merged++;
          }
        }
      } else {
        keep[kn++] = e;
        t.s_delayed++;
        if (bbr[wb]) t.s_blocked++;
      }
    }
    (void)rdm;
    for (int i = 0; i < kn; i++) n.uq[i] = keep[i];
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i] || in.in_actual_br_type[i] == BR_NONCTL) continue;
      t.s_upd_ctl++;
      if (kn < UQ_) { n.uq[kn++] = make_ent(in, i); t.s_upd_enq++; } else t.s_upd_drop++;
    }
    n.uq_n = kn;
  }

 public:
  // Fast-warmup replay (EXP-011): one committed instruction in program order, applied straight to the tables, the architectural
  // history / RAS and the uBTB (through the same block tracker the commit stream uses).
  void warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target) {
    State &n = st_;
    const br_type_t pty = type_.predict(pc);
    const uint32_t b = bank_of(pc);
    const pc_t bpc = bank_pc_of(pc);
    TagePred tp;
    tage_[b].predict(bpc, n.arch.fh, tp);
    if (type == BR_NONCTL) {
      if (pty != BR_NONCTL) { TypeWr tw; type_.update(pc, type, tw); type_.commit(tw); }
    } else {
      const bool pred = pty == BR_NONCTL ? false : (pty == BR_RET || pty == BR_CALL || br_ind(pty) || pty == BR_JAL) ? true : tp.pred;
      push_arch(n.arch, pc, dir, target, type);
      TypeWr tw; type_.update(pc, type, tw); type_.commit(tw);
      if (type == BR_DIRECT) {
        TageUpd u;
        std::memset(&u, 0, sizeof(u));
        u.bank_pc = bpc; u.real_dir = dir; u.pred = pred; u.alt = tp.alt; u.pcpn = tp.pcpn;
        for (int k = 0; k < TN; k++) { u.tag[k] = tp.tag[k]; u.idx[k] = tp.idx[k]; }
        TageWr w; tage_[b].update(u, w); tage_[b].commit(w);
      } else if (br_call(type)) n.aras.push(pc + 4);
      else if (type == BR_RET) n.aras.pop();
      BtbWr bw; btb_[b].update(bpc, type, dir, target, bw, tp.idx, tp.tag); btb_[b].commit(bw);
      static InputPayload in{};
      in.in_upd_valid[0] = true; in.in_update_base_pc[0] = pc; in.in_actual_br_type[0] = type; in.in_actual_dir[0] = dir;
      in.in_actual_targets[0] = target; in.in_pred_dir[0] = pred;
      for (int i = 1; i < CW; i++) in.in_upd_valid[i] = false;
      TrkCnt t;
      Trk tmp = n;
      tmp.tq_n = 0;
      if (ubmain_ != 2) ub_track(in, tmp, t);
      for (int i = 0; i < tmp.tq_n; i++) {
        UbEnt nw[MAXW]; uint32_t wm;
        ub_rmw(ub0_, tmp.tq[i], nw, wm);
        for (int w = 0; w < ub0_.W; w++) if ((wm >> w) & 1) ub0_.e[ub0_.index(tmp.tq[i].start, tmp.tq[i].hh) * ub0_.W + w] = nw[w];
        if (ub1_.N) {
          ub_rmw(ub1_, tmp.tq[i], nw, wm);
          for (int w = 0; w < ub1_.W; w++) if ((wm >> w) & 1) ub1_.e[ub1_.index(tmp.tq[i].start, tmp.tq[i].hh) * ub1_.W + w] = nw[w];
        }
      }
      n.tr_v = tmp.tr_v; n.tr_start = tmp.tr_start; n.tr_expect = tmp.tr_expect; n.tr_nd = tmp.tr_nd; n.tr_hh = tmp.tr_hh; n.ah = tmp.ah; n.lt_v = tmp.lt_v; n.lt_start = tmp.lt_start; n.lt_slot = tmp.lt_slot; n.lt_cnt = tmp.lt_cnt;
      return;
    }
    // non-control: advance the block tracker as well
    static InputPayload in2{};
    in2.in_upd_valid[0] = true; in2.in_update_base_pc[0] = pc; in2.in_actual_br_type[0] = BR_NONCTL; in2.in_actual_dir[0] = false;
    for (int i = 1; i < CW; i++) in2.in_upd_valid[i] = false;
    TrkCnt t;
    Trk tmp = n;
    tmp.tq_n = 0;
    if (ubmain_ != 2) ub_track(in2, tmp, t);
    for (int i = 0; i < tmp.tq_n; i++) {
      UbEnt nw[MAXW]; uint32_t wm;
      ub_rmw(ub0_, tmp.tq[i], nw, wm);
      for (int w = 0; w < ub0_.W; w++) if ((wm >> w) & 1) ub0_.e[ub0_.index(tmp.tq[i].start, tmp.tq[i].hh) * ub0_.W + w] = nw[w];
      if (ub1_.N) {
        ub_rmw(ub1_, tmp.tq[i], nw, wm);
        for (int w = 0; w < ub1_.W; w++) if ((wm >> w) & 1) ub1_.e[ub1_.index(tmp.tq[i].start, tmp.tq[i].hh) * ub1_.W + w] = nw[w];
      }
    }
    n.tr_v = tmp.tr_v; n.tr_start = tmp.tr_start; n.tr_expect = tmp.tr_expect; n.tr_nd = tmp.tr_nd; n.tr_hh = tmp.tr_hh; n.ah = tmp.ah; n.lt_v = tmp.lt_v; n.lt_start = tmp.lt_start; n.lt_slot = tmp.lt_slot; n.lt_cnt = tmp.lt_cnt;
  }
  void warm_finish() { st_.spec = st_.arch; st_.sras = st_.aras; }

  BpuPipe() { if (cfg_pipe() > 0) reset(); }
  const Stats &stats() const { return stats_; }
  int mode() const { return mode_; }
  int depth() const { return D_; }
  int s0_entries() const { return N_; }

  void reset() {
    mode_ = cfg_pipe(); D_ = cfg_main_lat(); N_ = cfg_s0_entries(); W_ = cfg_s0_ways(); S_ = N_ / W_; UQ_ = cfg_updq(); P_ = cfg_upd_ports();
    if (D_ < 1 || D_ > MAXD) { std::printf("[BPU_PIPE] BPU_MAIN_LAT must be 1..%d\n", MAXD); std::exit(1); }
    if (N_ < 2 || (N_ & (N_ - 1)) || W_ < 1 || W_ > MAXW || (W_ & (W_ - 1)) || N_ < W_) { std::printf("[BPU_PIPE] BPU_S0_ENTRIES / BPU_S0_WAYS must be powers of two (ways <= %d)\n", MAXW); std::exit(1); }
    if (UQ_ < 1 || UQ_ > MAXUQ || P_ < 1 || P_ > MAXP) { std::printf("[BPU_PIPE] bad BPU_UPDQ / BPU_UPD_PORTS\n"); std::exit(1); }
    loop_ = cfg_loop();
    if (loop_ && !cfg_hist()) { std::printf("[BPU_PIPE] BPO_LOOP needs BPO_HIST=1\n"); std::exit(1); }
    repair_ = cfg_repair();
    if (repair_ && mode_ < 2) { std::printf("[BPU_PIPE] BPA_REPAIR needs BPU_PIPE>=2\n"); std::exit(1); }
    for (auto &r : rec_) r.valid = false;
    pdck_ = cfg_pdckpt();
    if (pdck_ && mode_ < 2) { std::printf("[BPU_PIPE] BPO_PDCKPT needs BPU_PIPE>=2\n"); std::exit(1); }
    hist_ = cfg_hist(); hsym_ = cfg_hsym(); hb_ = cfg_ub_hbits(); hb1_ = cfg_l1_hbits();
    if (hb_ < 0 || hb_ > 16 || hb1_ < 0 || hb1_ > 16) { std::printf("[BPU_PIPE] BPO_UB_HBITS / BPO_L1_HBITS 0..16\n"); std::exit(1); }
    const int hmax = hb_ > hb1_ ? hb_ : hb1_;
    hb_ = hmax;   // history bits tracked: the larger of the two
    if (hist_ && (hsym_ < 1 || hsym_ > 4)) { std::printf("[BPU_PIPE] BPO_HSYM 1..4\n"); std::exit(1); }
    bb_ = cfg_bankblk(); merge_ = cfg_upd_merge(); ubmain_ = cfg_ub_main(); tqd_ = cfg_tq();
    if (bb_ < 0 || bb_ > MAXBB || (bb_ == 1) || (bb_ & (bb_ - 1)) || (bb_ && mode_ < 3)) { std::printf("[BPU_PIPE] BPO_BANKBLK must be 0 or a power of two in 2..%d and needs BPU_PIPE=3\n", MAXBB); std::exit(1); }
    if (tqd_ < 1 || tqd_ > TQ) { std::printf("[BPU_PIPE] BPO_TQ 1..%d\n", TQ); std::exit(1); }
    ub0_.init(N_, W_, cfg_ub_tag(), cfg_ub_hbits());
    {
      const int l1n = cfg_l1_entries(), l1w = cfg_l1_ways();
      if (l1n && (l1n & (l1n - 1) || l1w < 1 || l1w > MAXW || (l1w & (l1w - 1)) || l1n < l1w || mode_ < 2 || D_ < 2)) { std::printf("[BPU_PIPE] bad BPO_L1_ENTRIES / BPO_L1_WAYS (needs BPU_PIPE>=2, BPU_MAIN_LAT>=2)\n"); std::exit(1); }
      ub1_.init(l1n, l1w, cfg_ub_tag(), hb1_);
    }
    st_ = State{};
    type_.reset();
    static bool reg = false;
    if (!reg) { reg = true; self() = this; std::atexit([] { if (self()) self()->print_stats(); }); }
  }
  static BpuPipe *&self() { static BpuPipe *p = nullptr; return p; }

  // Outcome of the last stage (registers only). Called first in the cycle: the front end uses squash_n to gate the
  // fetch-address FIFO pop and to drop its tail entries.
  void decide(bool refetch, Decision &d) const {
    d = Decision{};
    const Blk &b = st_.R[D_ - 1];
    if (refetch || !b.valid) return;
    d.valid = true;
    d.blk = b;
    if (mode_ == 1) d.out = b.shadow;
    else if (D_ == 1) { MainRaw raw; main_read(b, raw); main_select(b, raw, d.out); }  // read and compare in one cycle
    else main_select(b, b.raw, d.out);
    int why;
    d.ovr = disagree(d.out, b.g, why);
    d.ock = b.ck; d.oout = d.out; d.obase = b.base; d.og = b.g;
    if (!d.ovr && b.pair) {   // FE_PAIR: the first block is confirmed, so the second one is checked as well
      d.pair = true;
      main_select(b.b2, b.b2.raw, d.outB);
      S0Guess g2 = b.b2.g;
      const int lenA = d.out.slot + 1, lenB = d.outB.taken ? d.outB.slot + 1 : int((b.b2.bnd - b.b2.base) >> 2);
      if (!d.out.taken || lenA != b.g.slot + 1 || lenA + lenB > FW) {   // the final prediction does not fit the fetched group: back to one block, the second one is fetched again as a normal block
        d.pair = false; d.ovr = true; d.pair_unfit = true;
      } else {
        d.ovr = d.ovr_b = disagree(d.outB, g2, why);
        if (d.ovr_b) { d.ock = b.b2.ck; d.oout = d.outB; d.obase = b.b2.base; d.og = g2; }
      }
    }
    if (d.ovr) for (int k = 0; k < D_ - 1; k++) d.squash_n += st_.R[k].valid;
  }

  // S1 level (BPO_L1_ENTRIES): a larger table read at S1 with the registered block address; on a hit that disagrees with the S0 guess
  // the guess of the block is replaced, the spec history / RAS are repaired from its checkpoint and S0 does not issue this cycle
  // (1 bubble instead of the 2 of an S_D override).
  bool l1_override(const Blk &b, State &n, Txn &t) const {
    if (!ub1_.N || !b.valid) return false;
    const S0Guess g1 = s0_lookup(ub1_, b.base, b.hh, b.bnd, b.ck.ras_cnt, b.ck.ras_top, b.ck.lp_cnt);
    if (!g1.hit) return false;
    t.s_l1_hit = 1;
    MainOut m; m.next = g1.next; m.nbits = g1.nbits; m.bits = g1.bits; m.ras_op = g1.ras_op; m.ras_val = g1.ras_val;
    int why;
    if (!disagree(m, b.g, why)) return false;
    t.s_l1_ovr = 1;
    n.R[1].g = g1;
    n.R[1].pair = false;
    restore(b.ck, n.spec, n.sras);
    push_bits(g1.nbits, g1.bits, g1.ras_op, g1.ras_val, n.spec, n.sras);
    n.pc = g1.next;
    n.lp_cnt = lp_next(b.ck.lp_cnt, b.base, g1.taken, g1.next, g1.type);
    return true;
  }

  // Training event from the result of the last stage (BPO_UB_MAIN): only where the final prediction differs from what S0 guessed
  void sd_event(const Decision &d, State &n, Txn &t) const {
    if (!d.valid || !ubmain_) return;
    const S0Guess &g = d.blk.g;
    UbEv e; e.start = d.blk.base; e.hh = d.blk.hh; e.src = 1;
    if (d.out.taken) {
      if (g.taken && g.slot == d.out.slot && g.next == d.out.next && g.nbits == d.out.nbits && g.type == d.out.type) return;
      e.kind = UE_TAKEN; e.slot = d.out.slot; e.type = d.out.type; e.nd = d.out.nbits; e.target = d.out.next;
    } else if (g.taken) {
      e.kind = UE_NTSLOT; e.slot = g.slot; e.type = g.type;
    } else if (!hist_ && d.out.nbits > 0 && (!g.hit || g.nbits != d.out.nbits)) {
      e.kind = UE_FALL; e.nd = d.out.nbits;
    } else return;
    t.s_sd_ev++;
    ub_event(n, t.tc, e);
  }

  void comb(const InputPayload &in, const Decision &d, OutputPayload &out, Txn &t) const {
    std::memset(&out, 0, sizeof(out));
    t = Txn{};
    State &n = t.nx;
    n = st_;
    const bool refetch = in.refetch;
    // BPO_PDCKPT: the checker consumed the PTAB head in the previous cycle -> its checkpoint leaves the record queue
    Ckpt pd_rec; bool pd_have = false;
    if (pdck_ && in.pd.valid && st_.pq_n > 0) {
      pd_rec = st_.pq[0]; pd_have = true;
      for (int i = 1; i < st_.pq_n; i++) n.pq[i - 1] = st_.pq[i];
      n.pq_n = st_.pq_n - 1;
    }

    // S0: look up the block starting at pc with the given speculative history / RAS, enter it into S1 and push the fetch address
    auto s0_issue = [&](pc_t pc, const HistRing &sp, const RasC &sr, uint8_t lpc) {
      Blk b;
      b.valid = true;
      b.base = pc;
      b.bnd = block_bnd(b.base);
      b.hh = spec_hh(sp);
      b.g = s0_lookup(ub0_, b.base, b.hh, b.bnd, sr.cnt, sr.top(), lpc);
      b.ck = take_ckpt(sp, sr);
      b.ck.lp_cnt = lpc;
      if (repair_ == 2) b.rf = sr;
      main_index(b, sp);
      if (mode_ == 1) {
        MainRaw raw;
        main_read(b, raw);
        count_reads(b, t);
        main_select(b, raw, b.shadow);
        push_bits(b.shadow.nbits, b.shadow.bits, b.shadow.ras_op, b.shadow.ras_val, n.spec, n.sras);
      } else {
        push_bits(b.g.nbits, b.g.bits, b.g.ras_op, b.g.ras_val, n.spec, n.sras);
      }
      n.R[0] = b;
      n.pc = b.g.next;
      n.lp_cnt = lp_next(lpc, b.base, b.g.taken, b.g.next, b.g.type);
      out.icache_read_valid = true;
      out.fetch_address = b.base;
      t.s_block = 1; t.s_hit = b.g.hit; t.s_hit_taken = b.g.taken;
      // FE_PAIR: the guess is a taken branch -> also look up the block at its target (spec history / RAS as left by the first block)
      const int pm = cfg_pair();   // FE_PAIR bits: 1 on; 2 = not after a return; 4 = not after a call; 8 = not after an indirect jump; 16 = not after a conditional branch
      const bool ty_ok = !((pm & 2) && b.g.type == BR_RET) && !((pm & 4) && br_call(b.g.type)) && !((pm & 8) && br_ind(b.g.type)) && !((pm & 16) && b.g.type == BR_DIRECT) && !((pm & 32) && b.g.type == BR_JAL);
      if (pm && ty_ok && mode_ >= 2 && D_ >= 2 && b.g.hit && b.g.taken && int(b.g.slot) + 1 < FW) {
        const int lenA = b.g.slot + 1;
        const pc_t T = b.g.next;
        if ((T & 3) == 0 && T != b.base + 4u * lenA && (T >> 12) == (b.base >> 12)) {   // same 4 KiB page: one translation serves both blocks
          HistRing sp2 = sp; RasC sr2 = sr;
          push_bits(b.g.nbits, b.g.bits, b.g.ras_op, b.g.ras_val, sp2, sr2);
          const uint8_t lp2 = n.lp_cnt;
          BlkB c;   // the second block is a normal fetch block (natural end), it must fit into the slots the first one leaves
          c.valid = true; c.base = T; c.bnd = block_bnd(T); c.hh = spec_hh(sp2);
          c.g = s0_lookup(ub0_, T, c.hh, c.bnd, sr2.cnt, sr2.top(), lp2);
          const int room = FW - lenA, natlen = int((c.bnd - T) >> 2);
          const int lenB = c.g.taken ? c.g.slot + 1 : natlen;
          if (lenB <= room) {
            c.ck = take_ckpt(sp2, sr2); c.ck.lp_cnt = lp2;
            main_index(c, sp2);
            push_bits(c.g.nbits, c.g.bits, c.g.ras_op, c.g.ras_val, n.spec, n.sras);
            n.R[0].b2 = c;
            n.R[0].pair = true;
            n.pc = c.g.next;
            n.lp_cnt = lp_next(lp2, T, c.g.taken, c.g.next, c.g.type);
            out.pair_valid = true; out.pair_addr2 = T; out.pair_a_len = lenA; out.pair_b_len = natlen < room ? natlen : room;
          }
        }
      }
    };

    // PTAB write: the block leaving S_D with the final prediction
    if (d.valid) {
      if (pdck_ && n.pq_n < PQ) n.pq[n.pq_n++] = d.blk.ck;
      out.PTAB_write_enable = true;
      out.out_pred_base_pc = d.blk.base;
      for (int i = 0; i < FW; i++) out.out_pred_dir[i] = d.out.dir[i];
      out.meta = d.out.meta;
      out.predict_next_fetch_address = d.out.next;
      if (d.pair) {   // FE_PAIR: the group is the first block (up to its taken branch) followed by the second block
        const int lenA = d.out.slot + 1;
        const BlkB &c = d.blk.b2;
        out.ptab_pcs_valid = true;
        for (int i = 0; i < FW; i++) { out.out_pred_pc[i] = i < lenA ? d.blk.base + 4u * i : c.base + 4u * (i - lenA); if (i >= lenA) out.out_pred_dir[i] = false; }
        for (int j = 0; lenA + j < FW; j++) {
          const int di = lenA + j;
          out.out_pred_dir[di] = d.outB.dir[j];
          out.meta.alt_pred[di] = d.outB.meta.alt_pred[j]; out.meta.pcpn[di] = d.outB.meta.pcpn[j];
          for (int k = 0; k < TN; k++) { out.meta.tage_tag[di][k] = d.outB.meta.tage_tag[j][k]; out.meta.tage_idx[di][k] = d.outB.meta.tage_idx[j][k]; }
          out.meta.ps_info[di] = d.outB.meta.ps_info[j];
        }
        out.predict_next_fetch_address = d.outB.next;
        t.s_pair = 1;
      }
      if (repair_) {
        t.rec_v = true;
        t.rec.ck = d.blk.ck; if (repair_ == 2) t.rec.rf = d.blk.rf; t.rec.base = d.blk.base; t.rec.id = st_.blk_seq; t.rec.epoch = st_.epoch; t.rec.valid = true;
        for (int i = 0; i < FW; i++) { out.meta.blk_id[i] = st_.blk_seq; t.rec.ptype[i] = d.out.meta.ps_info[i] & 7; }
        n.blk_seq = st_.blk_seq + 1;
        if (d.pair) {
          const int lenA = d.out.slot + 1;
          t.rec2_v = true;
          t.rec2.ck = d.blk.b2.ck; t.rec2.base = d.blk.b2.base; t.rec2.id = st_.blk_seq + 1; t.rec2.epoch = st_.epoch; t.rec2.valid = true;
          for (int i = 0; i < FW; i++) t.rec2.ptype[i] = d.outB.meta.ps_info[i] & 7;
          for (int i = lenA; i < FW; i++) out.meta.blk_id[i] = st_.blk_seq + 1;
          n.blk_seq = st_.blk_seq + 2;
        }
      }
    }

    // shift; the block in S1 is read this cycle and its raw array data rides along into the next stage register
    for (int k = D_ - 1; k >= 1; k--) n.R[k] = st_.R[k - 1];
    n.R[0] = Blk{};
    if (mode_ >= 2 && st_.R[0].valid) {
      count_reads(st_.R[0], t);
      bb_reads(st_.R[0], t);
      if (D_ >= 2) main_read(st_.R[0], n.R[1].raw);
      if (D_ >= 2 && st_.R[0].pair) main_read(st_.R[0].b2, n.R[1].b2.raw);
    }

    if (refetch) {
      t.s_refetch = 1; t.s_refetch_pd = in.refetch_pd;
      for (int k = 0; k < MAXD; k++) n.R[k] = Blk{};
      n.pc = in.refetch_address;
    } else if (d.ovr) {
      t.s_squashed = d.squash_n;
      for (int k = 0; k < MAXD; k++) n.R[k] = Blk{};
      restore(d.ock, n.spec, n.sras);
      push_bits(d.oout.nbits, d.oout.bits, d.oout.ras_op, d.oout.ras_val, n.spec, n.sras);
      n.pc = d.oout.next;
      n.lp_cnt = lp_next(d.ock.lp_cnt, d.obase, d.oout.taken, d.oout.next, d.oout.type);
      if ((cfg_bfast() & 2) && in.icache_read_ready && int(in.ptab_size) + 1 < PTAB_SIZE - 1) { const HistRing sp = n.spec; const RasC sr = n.sras; s0_issue(n.pc, sp, sr, n.lp_cnt); }   // FE_BFAST bit1
      int why; disagree(d.oout, d.og, why);
      t.s_pair_ovr_b = d.ovr_b; t.s_pair_unfit = d.pair_unfit; t.s_ovr_next = why == 1; t.s_ovr_hist = why == 2; t.s_ovr_ras = why == 3; t.s_ovr_miss = !d.og.hit;
      if (why == 1) {
        const S0Guess &g = d.og;
        if (g.taken && !d.oout.taken) t.s_ov_t2n = 1;
        else if (!g.taken && d.oout.taken) t.s_ov_n2t = 1;
        else if (br_ind(d.oout.type)) t.s_ov_ind = 1;
        else if (d.oout.type == BR_RET) { t.s_ov_ret = 1; }
        else {
          t.s_ov_oth = 1;
        }
      }
    } else if (l1_override(st_.R[0], n, t)) {
      // S1-level predictor disagrees with the S0 guess of the block in S1: that block's successor is not issued this cycle
      if ((cfg_bfast() & 1) && in.icache_read_ready && int(in.ptab_size) + 1 < PTAB_SIZE - 1) { const HistRing sp = n.spec; const RasC sr = n.sras; s0_issue(n.pc, sp, sr, n.lp_cnt); }   // FE_BFAST bit0
    } else {
      // ---- S0 ---------------------------------------------------------------------------------------------------------
      int inflight = 0;
      for (int k = 0; k < D_; k++) inflight += st_.R[k].valid;
      const bool ptab_stall = in.icache_read_ready && (int(in.ptab_size) + inflight >= PTAB_SIZE - 1);
      t.s_stall_ptab = ptab_stall;
      if (in.icache_read_ready && !ptab_stall) s0_issue(st_.pc, st_.spec, st_.sras, st_.lp_cnt);
    }

    // ---- commit side ---------------------------------------------------------------------------------------------------------
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const br_type_t ty = in.in_actual_br_type[i];
      t.s_commit++; t.s_cond += ty == BR_DIRECT; t.s_cond_mp += ty == BR_DIRECT && in.in_pred_dir[i] != in.in_actual_dir[i];
      push_arch(n.arch, in.in_update_base_pc[i], ty != BR_NONCTL && in.in_actual_dir[i], in.in_actual_targets[i], ty);
      if (br_call(ty)) n.aras.push(in.in_update_base_pc[i] + 4);
      else if (ty == BR_RET) n.aras.pop();
    }
    if (refetch) {
      if (pdck_ && in.refetch_pd && in.pd.valid && in.pd.flush && pd_have) {   // predecode flush: exact repair from the checked block's checkpoint
        restore(pd_rec, n.spec, n.sras);
        push_bits(in.pd.nbits, in.pd.bits, in.pd.ras_op, in.pd.ras_val, n.spec, n.sras);
        t.s_pd_restore = 1;
      } else if (repair_ && in.rf_valid && repair_blk(in, n)) {
        t.s_rep_ok = 1;
      } else { n.spec = n.arch; n.sras = n.aras; t.s_rep_miss = repair_ && in.rf_valid; n.epoch = st_.epoch + 1; }
      n.pq_n = 0;
      n.lp_cnt = 0;
      if ((cfg_rfast() & 1) && in.icache_read_ready) { const HistRing sp = n.spec; const RasC sr = n.sras; s0_issue(in.refetch_address, sp, sr, 0); }   // FE_RFAST bit0
    }
    if (mode_ >= 3) update_queue(in, n, t); else update_old(in, n, t);
    // uBTB training: head of the train queue is written, new events are appended behind it
    if (st_.tq_n > 0) {
      const UbEv &e = st_.tq[0];
      if (cfg_s0_train()) {
        if (!(ubmain_ == 4 && e.src == 1)) { ub_rmw(ub0_, e, t.ub_v[0], t.ub_wmask[0]); t.ub_set[0] = ub0_.index(e.start, e.hh); }   // BPO_UB_MAIN=4: S_D events train the S1 table only
        if (ub1_.N) { ub_rmw(ub1_, e, t.ub_v[1], t.ub_wmask[1]); t.ub_set[1] = ub1_.index(e.start, e.hh); }
      }
      for (int i = 1; i < st_.tq_n; i++) n.tq[i - 1] = st_.tq[i];
      n.tq_n = st_.tq_n - 1;
    }
    if (!refetch) sd_event(d, n, t);
    if (ubmain_ != 2) ub_track(in, n, t.tc);
    out.upd_free = mode_ >= 3 ? uint8_t(UQ_ - st_.uq_n) : 255;   // EXP-028 compact ROB commit credits (registered)
    out.tq_free = uint8_t(tqd_ - st_.tq_n);
  }

  void commit(const Txn &t) {
    // declared port counts: S1 reads one set per bank and array; updates write one per bank and array (the type table takes up to
    // CW writes in the old update path); with the update queue the arrays are single-ported (read + write <= 1).
    for (int b = 0; b < NB; b++) {
      const PortUse *p[3] = {&t.p_ty[b], &t.p_tg[b], &t.p_bt[b]};
      static const char *nm[3] = {"type", "tage", "btb"};
      for (int a = 0; a < 3; a++) {
        const int wr_max = (a == 0 && mode_ < 3) ? CW : 1;
        if (p[a]->rd > 1 || p[a]->wr > wr_max || (mode_ >= 3 && !bb_ && p[a]->rd + p[a]->wr > 1)) {
          std::printf("[BPU_PIPE] ERROR: port limit exceeded on %s bank %d: rd=%d wr=%d (mode %d)\n", nm[a], b, p[a]->rd, p[a]->wr, mode_);
          std::exit(1);
        }
      }
      if (t.p_ty[b].wr > stats_.max_type_wr_bank) stats_.max_type_wr_bank = t.p_ty[b].wr;
    }
    for (int b = 0; b < bb_; b++)
      if (t.bb_rd[b] > 1 || t.bb_wr[b] > 1 || t.bb_rd[b] + t.bb_wr[b] > 1) {
        std::printf("[BPU_PIPE] ERROR: block bank %d port limit exceeded: rd=%d wr=%d\n", b, t.bb_rd[b], t.bb_wr[b]);
        std::exit(1);
      }
    Stats &s = stats_;
    s.cyc++; s.blocks += t.s_block; s.s0_hit += t.s_hit; s.s0_hit_taken += t.s_hit_taken;
    s.ovr += t.s_ovr_next + t.s_ovr_hist + t.s_ovr_ras; s.ovr_next += t.s_ovr_next; s.ovr_hist += t.s_ovr_hist; s.ovr_ras += t.s_ovr_ras;
    s.commits += t.s_commit; s.cond += t.s_cond; s.cond_mispred += t.s_cond_mp; s.ovr_miss += t.s_ovr_miss; s.ov_t2n += t.s_ov_t2n; s.ov_n2t += t.s_ov_n2t; s.ov_ind += t.s_ov_ind; s.ov_ret += t.s_ov_ret; s.ov_oth += t.s_ov_oth; s.squashed += t.s_squashed; s.refetch += t.s_refetch; s.s0_stall_ptab += t.s_stall_ptab;
    s.tq_ev += t.tc.ev; s.tq_drop += t.tc.drop; s.ub_wr += t.ub_wmask[0] != 0; s.ub_wr1 += t.ub_wmask[1] != 0;
    pd_flush_count() += t.s_refetch_pd;
    rep_ok_ += t.s_rep_ok; rep_miss_ += t.s_rep_miss;
    if (t.rec_v) rec_[t.rec.id & 255] = t.rec;
    if (t.rec2_v) rec_[t.rec2.id & 255] = t.rec2;
    pair_blocks_ += t.s_pair; pair_ovr_b_ += t.s_pair_ovr_b; pair_unfit_ += t.s_pair_unfit;
    s.refetch_pd += t.s_refetch_pd; s.pd_restore += t.s_pd_restore; s.l1_hit += t.s_l1_hit; s.l1_ovr += t.s_l1_ovr; s.sd_ev += t.s_sd_ev; s.merged += t.s_merged; s.blocked += t.s_blocked;
    s.upd_ctl += t.s_upd_ctl; s.upd_enq += t.s_upd_enq; s.upd_drop += t.s_upd_drop; s.upd_applied += t.s_upd_applied;
    s.upd_delayed_cycles += t.s_delayed; s.uq_occ_sum += t.nx.uq_n; if (t.nx.uq_n > s.uq_max) s.uq_max = t.nx.uq_n;
    s.old_drop_sb += t.s_ctl_drop_sb; s.old_drop_full += t.s_ctl_drop_full; s.old_drop_nonctl += t.s_ctl_drop_nonctl;
    s.old_nonctl_enq += t.s_nonctl_enq;
    for (int b = 0; b < NB; b++) { tage_[b].commit(t.tage[b]); btb_[b].commit(t.btb[b]); }
    for (int i = 0; i < t.n_type; i++) type_.commit(t.type[i]);
    for (int w = 0; w < ub0_.W; w++) if ((t.ub_wmask[0] >> w) & 1) ub0_.e[t.ub_set[0] * ub0_.W + w] = t.ub_v[0][w];
    for (int w = 0; w < ub1_.W && ub1_.N; w++) if ((t.ub_wmask[1] >> w) & 1) ub1_.e[t.ub_set[1] * ub1_.W + w] = t.ub_v[1][w];
    st_ = t.nx;
  }

  void print_stats() const {
    const Stats &s = stats_;
    std::printf("[BPU_PIPE] mode=%d main_lat=%d s0_entries=%d x%d-way updq=%d upd_ports=%d cycles=%llu blocks=%llu s0_hit=%llu s0_hit_taken=%llu "
                "overrides=%llu (next=%llu hist=%llu ras=%llu; s0 miss=%llu) squashed=%llu refetch=%llu s0_stall_ptab=%llu\n",
                mode_, D_, N_, W_, UQ_, P_, (unsigned long long)s.cyc, (unsigned long long)s.blocks, (unsigned long long)s.s0_hit,
                (unsigned long long)s.s0_hit_taken, (unsigned long long)s.ovr, (unsigned long long)s.ovr_next, (unsigned long long)s.ovr_hist,
                (unsigned long long)s.ovr_ras, (unsigned long long)s.ovr_miss, (unsigned long long)s.squashed, (unsigned long long)s.refetch, (unsigned long long)s.s0_stall_ptab);
    std::printf("[BPA_REPAIR] repair=%d repaired=%llu record_missing=%llu\n", repair_, (unsigned long long)rep_ok_, (unsigned long long)rep_miss_);
    std::printf("[FE_PAIR] groups written to the PTAB with a second block=%llu, second-block overrides=%llu, pairs dropped at S_D (does not fit / first block changed)=%llu\n", (unsigned long long)pair_blocks_, (unsigned long long)pair_ovr_b_, (unsigned long long)pair_unfit_);
    std::printf("[BPU_OPT] refetch_predecode=%llu pdckpt=%d pd_restored=%llu\n", (unsigned long long)s.refetch_pd, pdck_, (unsigned long long)s.pd_restore);
    std::printf("[BPU_OPT] bankblk=%d merge=%d ub_main=%d l1=%dx%d tq=%d | l1_hit=%llu l1_ovr=%llu l1_writes=%llu sd_events=%llu merged_writes=%llu blocked_by_read=%llu\n", bb_, merge_, ubmain_, ub1_.N, ub1_.W, tqd_,
                (unsigned long long)s.l1_hit, (unsigned long long)s.l1_ovr, (unsigned long long)s.ub_wr1, (unsigned long long)s.sd_ev, (unsigned long long)s.merged, (unsigned long long)s.blocked);
    std::printf("[BPU_PIPE_OVR] s0_taken_main_not=%llu s0_not_main_taken=%llu both_taken_target_differs: indirect=%llu ret=%llu other=%llu\n",
                (unsigned long long)s.ov_t2n, (unsigned long long)s.ov_n2t, (unsigned long long)s.ov_ind, (unsigned long long)s.ov_ret,
                (unsigned long long)s.ov_oth);
    std::printf("[BPU_MPKI] commits=%llu cond_branches=%llu cond_mispredicts=%llu\n", (unsigned long long)s.commits, (unsigned long long)s.cond,
                (unsigned long long)s.cond_mispred);
    std::printf("[BPU_PIPE_UPD] ub_events=%llu ub_events_dropped=%llu ub_writes=%llu | control updates=%llu enq=%llu dropped=%llu applied=%llu "
                "delayed_entry_cycles=%llu uq_avg_occ=%.2f uq_max=%llu | old-rule drops: same_bank=%llu full=%llu (blocked by non-control=%llu) "
                "non-control enq=%llu max_type_wr_per_bank=%llu\n",
                (unsigned long long)s.tq_ev, (unsigned long long)s.tq_drop, (unsigned long long)s.ub_wr, (unsigned long long)s.upd_ctl,
                (unsigned long long)s.upd_enq, (unsigned long long)s.upd_drop, (unsigned long long)s.upd_applied,
                (unsigned long long)s.upd_delayed_cycles, s.cyc ? double(s.uq_occ_sum) / s.cyc : 0.0, (unsigned long long)s.uq_max,
                (unsigned long long)s.old_drop_sb, (unsigned long long)s.old_drop_full, (unsigned long long)s.old_drop_nonctl,
                (unsigned long long)s.old_nonctl_enq, (unsigned long long)s.max_type_wr_bank);
  }
};

}  // namespace bpuf

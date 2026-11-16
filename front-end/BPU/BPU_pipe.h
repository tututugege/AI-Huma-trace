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
// (Earlier variants BPU_PIPE=0..2 -- the zero-bubble predictor, the S0-exact history variant and the one-queue-per-bank update path --
// were removed; git history has them.  The update queue's writes are arbitrated against the S1 reads: the arrays are single-ported.)
// comb() reads registers and arrays only and returns the next value of every register in a Txn; commit() is the clock edge.
#pragma once
#include "BPU.h"
#include "fpr.h"
#include <type_traits>
#include <vector>
#include <utility>

namespace bpuf {

// Fixed design (the run-time sweeps BPU_* / BPO_* / FE_BFAST were removed; git history has them):
constexpr int MAIN_LAT = 2;            // main predictor S1..S2: S1 reads the arrays, S2 selects and compares with the S0 guess
constexpr int S0_N = 128, S0_W = 4;    // uBTB entries / ways
constexpr int L1_N = 512, L1_W = 2;    // S1-level uBTB entries / ways (override at S1, 1 bubble)
constexpr int UB_TAGB = 10;            // uBTB tag bits
constexpr int UQ_N = 8, UPD_PORTS = 2; // update queue entries, table writes per cycle
constexpr int BANKBLK = 4;             // block banks (32 B windows), one row access per bank and cycle
constexpr int TQ_N = 4;                // uBTB train queue entries
// FE_PAIR=1: dual-block fetch. When the S0 guess of a block is a taken branch, the block at its target is looked up in the same cycle
// (second uBTB read, second set of main-predictor reads in S1) and both are fetched as one group (<= FETCH_WIDTH slots in total).
inline int cfg_pair() { static const int v = std::getenv("FE_PAIR") ? std::atoi(std::getenv("FE_PAIR")) : 0; return v; }

constexpr int MAXW = 4;       // uBTB ways
constexpr int PQ = 64;        // PTAB checkpoint records (BPO_PDCKPT)
constexpr int HR_BITS = 512;  // circular global-history buffer storage; the ring is HR_BITS_USED long (256, or 512 with FE_PAIR: two blocks per cycle keep twice as many speculative history bits in flight)
inline uint32_t hr_mask() { static const uint32_t m = (cfg_pair() ? 512u : 256u) - 1u; return m; }

constexpr uint32_t mask_of(int bits) { return bits >= 32 ? 0xffffffffu : ((1u << bits) - 1u); }
// uBTB entry field widths: every field is truncated to its declared width on write.
constexpr int UB_TAG_BITS = 10, UB_SLOT_BITS = clog2(FETCH_WIDTH), UB_TYPE_BITS = 3, UB_CTR_BITS = 2, UB_ND_BITS = 4, UB_TGT_BITS = 32 - SLOT_SHIFT;   // slot offset and target slot address

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
struct RasC {   // the committed RAS: a circular stack
  target_addr_t st[RAS_DEPTH] = {};
  uint8_t ptr = 0, cnt = 0;  // 5 b, 6 b
  target_addr_t top() const { return st[ptr]; }
  void push(target_addr_t v) { ptr = (ptr + 1) % RAS_DEPTH; st[ptr] = v; if (cnt < RAS_DEPTH) cnt++; }
  void pop() { if (cnt > 0) { ptr = (ptr + RAS_DEPTH - 1) % RAS_DEPTH; cnt--; } }
};

// The speculative RAS (as XiangShan's): an append-only queue.  A push writes a new slot at tosw (its nos link points to the entry below), a pop only moves
// tosr back along the nos links, so no entry is overwritten before it is no longer reachable and a checkpoint of the pointers is exact (a circular stack
// overwrites the slot below the top when a wrong-path push follows a pop).  cnt = depth (saturates at RAS_DEPTH: the oldest entry is forgotten).
constexpr int RASQ = 64;
struct RasQ {
  target_addr_t st[RASQ] = {};
  uint8_t nos[RASQ] = {};
  uint8_t tosr = 0, tosw = 0, cnt = 0;
  target_addr_t top() const { return st[tosr]; }
  void push(target_addr_t v) { st[tosw] = v; nos[tosw] = tosr; tosr = tosw; tosw = (tosw + 1) % RASQ; if (cnt < RAS_DEPTH) cnt++; }
  void pop() { if (cnt > 0) { tosr = nos[tosr]; cnt--; } }
  static RasQ from_commit(const RasC &a) {   // the speculative RAS rebuilt from the committed stack (bottom first)
    RasQ q;
    for (int k = int(a.cnt) - 1; k >= 0; k--) q.push(a.st[(a.ptr + RAS_DEPTH - k) % RAS_DEPTH]);
    return q;
  }
};

// Per fetch block checkpoint, stored next to the block in the stage registers (state before the block's S0 push).
struct Ckpt {
  uint32_t head = 0;                    // 8 b
  uint32_t fh[FH_N_MAX][TN] = {};       // 12 folded histories (<= 11 b each)
  uint8_t ras_tosr = 0, ras_tosw = 0, ras_cnt = 0;   // the speculative RAS: read / write pointers (6 b each), depth
  target_addr_t ras_top = 0;            // 32 b
};

// ---------------------------------------------------------------------------
// uBTB (S0): direct mapped, indexed by the fetch-block start; 1 read port (S0), 1 RMW read + 1 write port (training)
// ---------------------------------------------------------------------------
struct UbEnt {
  bool v = false;
  uint16_t tag = 0;       // 10 b
  bool taken = false;     // 1 b: the block ends with a taken branch (else: fall-through block that has conditional branches)
  uint8_t type = 0;       // 3 b
  uint8_t slot = 0;       // 3 b (4 b with RVC): slot offset of the taken branch in the block
  uint8_t ctr = 0;        // 2 b: taken hint (conditional branches)
  uint8_t nd_t = 0;       // 4 b: conditional branches up to and including the taken branch
  uint8_t nd_nt = 0;      // 4 b: conditional branches in the block if it falls through
  bool rvc = false;       // 1 b: the taken branch is a 16-bit instruction (RAS push value pc + 2)
  uint32_t target = 0;    // 30 b: target word address
};
enum UbEvKind : uint8_t { UE_TAKEN = 0, UE_FALL = 1, UE_NTSLOT = 2 };
// uBTB targets hold the low 32 bits of the target slot address (UB_TGT_BITS); the upper bits come from the block's own address (a jump out of the 4 GiB
// region is then mispredicted and corrected at S_D).  For rv32 this is the identity.
inline pc_t ub_expand(uint32_t t, pc_t ref) { return (ref & ~static_cast<pc_t>(0xffffffffu)) | static_cast<pc_t>(static_cast<uint32_t>(t << SLOT_SHIFT)); }
struct UbEv { uint8_t kind = 0, src = 0; pc_t start = 0; uint8_t slot = 0, type = 0, nd = 0; pc_t target = 0; bool rvc = false; };

// One set-associative table of UbEnt (the S0 uBTB, and the optional S1 level BPO_L1_*).
struct UbTab {
  int N = 0, W = 1, S = 1, tagb = 10;
  std::vector<UbEnt> e;
  void init(int n, int w, int tb) { N = n; W = w; S = n > 0 ? n / w : 1; tagb = tb; e.assign(n, UbEnt{}); }
  uint32_t index(pc_t base) const {
    const uint32_t w = base >> SLOT_SHIFT, k = ceil_log2_u32(S);
    return (w ^ (w >> k)) & (S - 1);
  }
  uint16_t tag(pc_t base) const { return ((base >> SLOT_SHIFT) >> ceil_log2_u32(S)) & mask_of(tagb); }
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
  bool rvc = false;              // the taken branch is a 16-bit instruction
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
  pc_t base = 0, bnd = 0;
  S0Guess g;
  Ckpt ck;
  Tage::Idx tix[FW];
};

struct Blk {
  bool pair = false;             // FE_PAIR: b2 is the second block of the group
  BlkB b2;
  bool valid = false;
  pc_t base = 0, bnd = 0;
  S0Guess g;
  Ckpt ck;
  Tage::Idx tix[FW];
};

struct Decision {                // outcome of the last stage, computed from registers only
  bool valid = false;            // a block leaves the pipeline: PTAB write
  bool ovr = false;
  uint8_t squash_n = 0;          // younger blocks in flight (already pushed into the fetch-address FIFO)
  const Blk *blk = nullptr;      // the block in S_D of the current registers (valid until the next commit)
  MainOut out;
  bool pair = false, ovr_b = false;   // FE_PAIR: the group has a second block that survived the first block's check; the override (if any) is the second block's
  MainOut outB;
  Ckpt ock; MainOut oout; pc_t obase = 0; S0Guess og;   // the overriding block (A, or B of a pair)
};

struct UpdEnt {
  pc_t pc = 0;
  bool rvc = false;
  bool dir = false, pred = false, alt = false;
  br_type_t type = BR_NONCTL;
  target_addr_t target = 0;
  pcpn_t pcpn = 0;
  tage_tag_t tag[TN] = {};
  tage_idx_t idx[TN] = {};
};

struct PortUse { uint8_t rd = 0, wr = 0; };

struct RecP {   // BPA_REPAIR: checkpoint record of one fetch block, written when the block leaves S_D with its PTAB entry
  Ckpt ck; pc_t base = 0; uint8_t ptype[FW] = {}; uint16_t id = 0; uint8_t epoch = 0; bool valid = false;
};

class BpuPipe {
 public:
  struct Trk {                         // commit-side block tracker + uBTB train queue
    bool tr_v = false;
    pc_t tr_start = 0, tr_expect = 0;
    uint8_t tr_nd = 0;
    UbEv tq[TQ_N];
    uint8_t tq_n = 0;
  };
  struct State : Trk {
    pc_t pc = RESET_PC;
    Ckpt pq[PQ];                       // checkpoints of the blocks in the PTAB, oldest first (BPO_PDCKPT)
    uint8_t pq_n = 0;
    uint16_t blk_seq = 0;              // BPA_REPAIR: next block sequence number
    uint8_t epoch = 0;                 // BPA_REPAIR: bumped whenever spec is overwritten by arch (the history ring is re-based: older checkpoints no longer index it)
    Blk R[MAIN_LAT];                   // R[0] = S1, R[1] = S2
    MainRaw raw, raw_b;                // S1 array read data of the block in S2 (and of its second block); valid with R[1]
    HistRing spec, arch;
    RasQ sras;
    RasC aras;
    UpdEnt uq[UQ_N];
    uint8_t uq_n = 0;
  };

  struct Txn {
    uint32_t ub_wmask[2] = {};         // ways written in the set below: [0] uBTB (S0), [1] S1 level
    uint32_t ub_set[2] = {};
    UbEnt ub_v[2][MAXW];
    uint8_t bb_rd[BANKBLK] = {}, bb_wr[BANKBLK] = {};  // block-bank port use
    bool rec_v = false, rec2_v = false;                // BPA_REPAIR (rec2: second block of a pair)
    RecP rec, rec2;
    TageWr tage[NB];
    BtbWr btb[NB];
    TypeWr type[CW + UPD_PORTS];
    uint8_t n_type = 0;
    uint32_t ty_lane = 0;              // banks whose type lane an update of the queue used this cycle
    PortUse p_ty[NB], p_tg[NB], p_bt[NB];
    bool pmu_l1_ovr = false, pmu_ptab_stall = false;   // PMU (front-end loss cause) only
  };

 private:
  static constexpr int hsym_ = HSYM_BITS;
  // The pipeline registers are double-buffered: comb() builds the next value in *nxp_ (from *stp_), commit() flips the two.
  State sb_[2];
  State *stp_ = &sb_[0], *nxp_ = &sb_[1];
  UbTab ub0_, ub1_;
  Tage tage_[NB];
  Btb btb_[NB];
  TypePred type_;
  RecP rec_[256];

  static pc_t block_bnd(pc_t base) {
    const pc_t mask = ~static_cast<pc_t>(ICACHE_LINE_SIZE - 1);   // pc_t wide (rv64 kernel addresses are above 2^32)
    const pc_t plus = base + FW * SLOT_BYTES;
    return (base & mask) != (plus & mask) ? (plus & mask) : plus;
  }
  // 32 B window (one block-bank row, BPO_BANKBLK): the unit of banking by fetch block
  static uint32_t win_of(pc_t pc) { return pc >> 5; }
  static uint32_t bb_of(pc_t pc, int n) { return win_of(pc) % n; }

  static Ckpt take_ckpt(const HistRing &h, const RasQ &r) {
    Ckpt c;
    c.head = h.head;
    std::memcpy(c.fh, h.fh, sizeof(c.fh));
    c.ras_tosr = r.tosr; c.ras_tosw = r.tosw; c.ras_cnt = r.cnt; c.ras_top = r.top();
    return c;
  }
  static void restore(const Ckpt &c, HistRing &h, RasQ &r) {
    h.head = c.head;
    std::memcpy(h.fh, c.fh, sizeof(h.fh));
    r.tosr = c.ras_tosr; r.tosw = c.ras_tosw; r.cnt = c.ras_cnt;   // pointers only: the queue keeps what they reach
  }
  // BPO_HIST=1: a taken branch pushes hsym_ bits folded from its pc and target; not-taken branches push nothing
  uint8_t hsym(pc_t bpc, pc_t tgt) const { return hist_sym(bpc, tgt); }
  void push_arch(HistRing &h, pc_t bpc, bool taken, pc_t tgt, br_type_t ty) const {
    (void)ty;
    if (!taken) return;
    const uint8_t b = hsym(bpc, tgt);
    for (int i = 0; i < hsym_; i++) h.push((b >> i) & 1);
  }
  static void push_bits(uint8_t n, uint8_t bits, uint8_t ras_op, target_addr_t ras_val, HistRing &h, RasQ &r) {
    for (int i = 0; i < n; i++) h.push((bits >> i) & 1);
    if (ras_op == 1) r.push(ras_val); else if (ras_op == 2) r.pop();
  }

  // ---- S0 -------------------------------------------------------------------------------------------------------------
  // One set read; among the ways that match the block start, the lowest-slot branch that is predicted taken wins.
  S0Guess s0_lookup(const UbTab &tb, pc_t base, pc_t bnd, uint8_t rcnt, target_addr_t rtop) const {
    S0Guess g;
    g.next = bnd;
    const uint32_t set = tb.index(base);
    const uint16_t tag = tb.tag(base);
    int best = -1;
    for (int w = 0; w < tb.W; w++) {
      const UbEnt &e = tb.e[set * tb.W + w];
      if (!(e.v && e.tag == tag)) continue;
      if (!g.hit) { g.hit = true; g.nbits = 0; }
      const bool tk = e.taken && base + SLOT_BYTES * e.slot < bnd && (e.type != BR_DIRECT || e.ctr >= 2);
      if (tk && (best < 0 || e.slot < tb.e[set * tb.W + best].slot)) best = w;
    }
    if (best < 0) return g;
    const UbEnt &e = tb.e[set * tb.W + best];
    const pc_t bpc = base + SLOT_BYTES * e.slot;
    g.taken = true; g.slot = e.slot; g.type = e.type;
    g.next = ub_expand(e.target, base);
    if (e.type == BR_RET && rcnt > 0) g.next = rtop;
    g.nbits = hsym_; g.bits = hsym(bpc, g.next);
    if (br_call(e.type)) { g.ras_op = 1; g.ras_val = bpc + ilen_of(e.rvc); }
    else if (e.type == BR_RET) g.ras_op = 2;
    return g;
  }

  // ---- main predictor: index (S0), read (S1), select (S_D) -----------------------------------------------------------------
  template <class B> void main_index(B &b, const HistRing &h) const {
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * SLOT_BYTES;
      if (pc >= b.bnd) continue;
      Tage::index(bank_pc_of(pc), h.fh, b.tix[i]);
    }
  }
  template <class B> void main_read(const B &b, MainRaw &r) const {
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * SLOT_BYTES;
      if (pc >= b.bnd) continue;
      const uint32_t bk = bank_of(pc);
      type_.read(pc, r.ty[i]); tage_[bk].read(b.tix[i], r.tg[i]); btb_[bk].read(bank_pc_of(pc), r.bt[i], b.tix[i].idx);
    }
  }
  static void count_reads(const Blk &b, Txn &t) {
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * SLOT_BYTES;
      if (pc >= b.bnd) continue;
      const uint32_t bk = bank_of(pc);
      t.p_ty[bk].rd++; t.p_tg[bk].rd++; t.p_bt[bk].rd++;
    }
  }
  // block-bank rows read by the block's S1 access (BPO_BANKBLK): the 32 B windows it covers (1, or 2 if it straddles a window boundary)
  void bb_reads(const Blk &b, Txn &t) const {
    if (!b.valid) return;
    const uint32_t w0 = win_of(b.base), w1 = win_of(b.bnd - SLOT_BYTES);
    t.bb_rd[w0 % BANKBLK]++;
    if (w1 != w0) t.bb_rd[w1 % BANKBLK]++;
  }
  template <class B> void main_select(const B &b, const MainRaw &r, MainOut &o) const {
    o = MainOut{};
    o.next = b.bnd;
    bool taken_found = false, spec_done = false;
    for (int i = 0; i < FW; i++) {
      const pc_t pc = b.base + i * SLOT_BYTES;
      if (pc >= b.bnd) continue;
      const uint32_t bk = bank_of(pc);
      const pc_t bpc = bank_pc_of(pc);
      const br_type_t ty = TypePred::select(pc, r.ty[i]);
      const bool rvc = TypePred::select_rvc(pc, r.ty[i]);
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
        target_addr_t tgt = Btb::select(bpc, ty, r.bt[i], &src, tp.tag, &itt, pc + ilen_of(rvc));   // a BTB miss falls through (the real pc, not the bank-local one)
        if (ty == BR_RET && b.ck.ras_cnt > 0) { tgt = b.ck.ras_top; src = 7; }
        o.meta.ps_info[i] = ps_pack(ty, src, tp.used_alt) | (itt ? 0x80 : 0);
        o.next = tgt; o.slot = i; o.type = ty; o.taken = true; o.rvc = rvc;
        taken_found = true;
      }
      if (!spec_done) {
        if (taken) {
          o.nbits = hsym_; o.bits = hsym(pc, o.next);
          spec_done = true;
          if (br_call(ty)) { o.ras_op = 1; o.ras_val = pc + ilen_of(rvc); }
          else if (ty == BR_RET) o.ras_op = 2;
        }
      }
    }
  }

  // why: 1 next address, 2 history bits, 3 RAS operation
  bool disagree(const MainOut &m, const S0Guess &g, int &why) const {
    why = 0;
    if (m.next != g.next) { why = 1; return true; }
    if (m.nbits != g.nbits || m.bits != g.bits) { why = 2; return true; }
    if (m.ras_op != g.ras_op || (m.ras_op == 1 && m.ras_val != g.ras_val)) { why = 3; return true; }
    return false;
  }

  // ---- uBTB training --------------------------------------------------------------------------------------------------------
  static void ub_event(Trk &n, const UbEv &e) {
    if (n.tq_n < TQ_N) n.tq[n.tq_n++] = e;   // a full train queue drops the event
  }
  // Commit-order block tracker: reconstructs the fetch block each committed instruction belonged to.
  static void ub_track(const InputPayload &in, Trk &n) {
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const pc_t pc = in.in_update_base_pc[i];
      const br_type_t ty = in.in_actual_br_type[i];
      if (!n.tr_v || pc != n.tr_expect || pc >= block_bnd(n.tr_start)) { n.tr_v = true; n.tr_start = pc; n.tr_nd = 0; }
      const pc_t bnd = block_bnd(n.tr_start);
      const uint8_t slot = (pc - n.tr_start) >> SLOT_SHIFT;
      if (ty == BR_DIRECT && n.tr_nd < 8) n.tr_nd++;
      const bool taken = ty != BR_NONCTL && in.in_actual_dir[i];
      UbEv e; e.start = n.tr_start; e.slot = slot; e.type = ty; e.nd = n.tr_nd;
      if (taken) {
        e.kind = UE_TAKEN; e.target = in.in_actual_targets[i]; e.rvc = in.in_update_is_rvc[i];
        ub_event(n, e);
        n.tr_start = e.target; n.tr_nd = 0; n.tr_expect = e.target;
      } else {
        const pc_t nxt = pc + ilen_of(in.in_update_is_rvc[i]);
        if (ty == BR_DIRECT && in.in_pred_dir[i]) {   // predicted taken, was not: the front end restarts at pc + 4
          e.kind = UE_NTSLOT; ub_event(n, e);
          n.tr_start = nxt; n.tr_nd = 0;
        } else if (nxt >= bnd) {                      // sequential block end
          if (n.tr_nd > 0) { e.kind = UE_FALL; ub_event(n, e); }
          n.tr_start = bnd; n.tr_nd = 0;   // the next fetch block starts at the block end (RVC=1: an instruction that straddles it starts before nxt = its end; tr_expect stays nxt)
        }
        n.tr_expect = nxt;
      }
    }
  }
  static void ub_trunc(UbEnt &e, int tagb) {  // declared widths
    e.tag &= mask_of(tagb); e.type &= mask_of(UB_TYPE_BITS); e.slot &= mask_of(UB_SLOT_BITS); e.ctr &= mask_of(UB_CTR_BITS);
    e.nd_t &= mask_of(UB_ND_BITS); e.nd_nt &= mask_of(UB_ND_BITS); e.target &= mask_of(UB_TGT_BITS);
  }
  static bool ub_same(const UbEnt &a, const UbEnt &b) {
    return a.v == b.v && a.tag == b.tag && a.taken == b.taken && a.type == b.type && a.slot == b.slot && a.ctr == b.ctr &&
           a.nd_t == b.nd_t && a.nd_nt == b.nd_nt && a.target == b.target;
  }
  // Read-modify-write of one set by the head of the train queue (W ways read, written ways in wmask).
  // Every DIRECT way of the block below the taken slot (all of them for a fall-through block) was not taken: its counter steps down.
  void ub_rmw(const UbTab &tb, const UbEv &ev, UbEnt nw[MAXW], uint32_t &wmask) const {
    const int W = tb.W;
    const uint32_t set = tb.index(ev.start);
    const uint16_t tag = tb.tag(ev.start);
    wmask = 0;
    for (int w = 0; w < W; w++) nw[w] = tb.e[set * W + w];
    const int limit = ev.kind == UE_TAKEN ? ev.slot : ev.kind == UE_FALL ? FW : ev.slot + 1;
    int same = -1, fall = -1;
    uint8_t nd_nt = ev.nd;
    for (int w = 0; w < W; w++) {
      UbEnt &e = nw[w];
      if (!(e.v && e.tag == tag)) continue;
      nd_nt = e.nd_nt;
      if (!e.taken) fall = w;
      if (e.taken && e.type == BR_DIRECT && e.slot < limit && !(ev.kind == UE_TAKEN && e.slot == ev.slot)) e.ctr = e.ctr > 0 ? e.ctr - 1 : 0;
      if (ev.kind == UE_FALL) e.nd_nt = ev.nd;
      if (ev.kind == UE_TAKEN && e.taken && e.slot == ev.slot) same = w;
    }
    if (ev.kind == UE_TAKEN) {
      int w = same;
      if (w < 0) {  // allocate: the fall-through placeholder of this block, an invalid way, else the lowest counter
        w = fall;
        for (int i = 0; w < 0 && i < W; i++) if (!nw[i].v) w = i;
        if (w < 0) { int m = 99; for (int i = 0; i < W; i++) if (nw[i].ctr < m) { m = nw[i].ctr; w = i; } }
        nw[w] = UbEnt{};
        nw[w].v = true; nw[w].tag = tag; nw[w].taken = true; nw[w].slot = ev.slot;
        nw[w].ctr = ev.type == BR_DIRECT ? 2 : 3; nw[w].nd_nt = nd_nt;
      } else {
        nw[w].ctr = ev.type == BR_DIRECT ? (nw[w].ctr < 3 ? nw[w].ctr + 1 : 3) : 3;
      }
      nw[w].type = ev.type; nw[w].nd_t = ev.nd; nw[w].target = ev.target >> SLOT_SHIFT; nw[w].rvc = ev.rvc;
    }
    for (int w = 0; w < W; w++) { ub_trunc(nw[w], tb.tagb); if (!ub_same(nw[w], tb.e[set * W + w])) wmask |= 1u << w; }
  }

  // BPA_REPAIR: rebuild spec history / RAS = checkpoint of the mispredicted branch's block + earlier slots (not taken) + the branch's actual outcome
  bool repair_blk(const InputPayload &in, State &n) const {
    const RecP &rc = rec_[in.rf_blk & 255];
    // a carried instruction (RVC=1: its first halfword is the last slot of the previous block) is completed in slot 0 of this block, its pc is the block's base - SLOT_BYTES
    const bool carried = rc.valid && in.rf_pc + SLOT_BYTES == rc.base;
    const uint32_t rslot = carried ? 0 : (in.rf_pc - rc.base) >> SLOT_SHIFT;   // slot within the block (the back end's slot is the offset in the fetch group, which differs for the second block of a pair)
    if (!(rc.valid && rc.id == in.rf_blk && rc.epoch == (*stp_).epoch && rslot < FW && (carried || rc.base + SLOT_BYTES * rslot == in.rf_pc))) return false;
    restore(rc.ck, n.spec, n.sras);
    const bool tk = in.rf_type == BR_DIRECT ? (in.refetch_address != in.rf_pc + ilen_of(in.rf_is_rvc)) : true;
    if (tk) { const uint8_t sy = hsym(in.rf_pc, in.refetch_address); for (int k = 0; k < hsym_; k++) n.spec.push((sy >> k) & 1); }
    if (br_call(in.rf_type)) n.sras.push(in.rf_pc + ilen_of(in.rf_is_rvc)); else if (in.rf_type == BR_RET) n.sras.pop();
    return true;
  }

  // ---- table updates ----------------------------------------------------------------------------------------------------------
  static UpdEnt make_ent(const InputPayload &in, int i) {
    UpdEnt e;
    e.pc = in.in_update_base_pc[i]; e.rvc = in.in_update_is_rvc[i];
    e.dir = in.in_actual_dir[i]; e.pred = in.in_pred_dir[i]; e.alt = in.in_alt_pred[i];
    e.type = in.in_actual_br_type[i]; e.target = in.in_actual_targets[i];
    e.pcpn = in.in_pcpn[i];
    for (int k = 0; k < TN; k++) { e.tag[k] = in.in_tage_tags[i][k]; e.idx[k] = in.in_tage_idxs[i][k]; }
    return e;
  }
  void apply_table_update(const UpdEnt &e, Txn &t) const {
    uint64_t idxp = 0; uint32_t tagp = 0;   // the TAGE indices (10 bit) and tags (8 bit) of the entry, table 0 in the low bits (the RTL packs the same way)
    for (int i = 0; i < TN; i++) { idxp |= static_cast<uint64_t>(e.idx[i] & 0x3ffu) << (10 * i); tagp |= static_cast<uint32_t>(e.tag[i] & 0xffu) << (8 * i); }
    fpr::up(static_cast<uint32_t>(e.pc), e.dir, e.type, static_cast<uint32_t>(e.target), e.pred, e.pcpn, e.rvc, idxp, tagp);
    const uint32_t b = bank_of(e.pc);
    const pc_t bpc = bank_pc_of(e.pc);
    if (e.type == BR_DIRECT) {
      TageUpd u;
      u.bank_pc = bpc; u.real_dir = e.dir; u.pred = e.pred; u.alt = e.alt; u.pcpn = e.pcpn;
      for (int k = 0; k < TN; k++) { u.tag[k] = e.tag[k]; u.idx[k] = e.idx[k]; }
      tage_[b].update(u, t.tage[b]);
      t.p_tg[b].wr++;
    }
    if (e.type != BR_NONCTL) { btb_[b].update(bpc, e.type, e.dir, e.target, t.btb[b], e.idx, e.tag); t.p_bt[b].wr++; }
  }
  void type_update(const UpdEnt &e, Txn &t) const {
    TypeWr &w = t.type[t.n_type];
    type_.update(e.pc, e.type, w, e.rvc);
    t.ty_lane |= 1u << bank_of(e.pc);
    if (w.en) { t.p_ty[w.bank].wr++; t.n_type++; }
  }

  // Update queue: control updates only; an entry is written when its bank has no S1 read this cycle (reads win), at most P_ writes
  // per cycle, one per bank. A full queue drops the new entry.
  void update_queue(const InputPayload &in, State &n, Txn &t) const {
    uint8_t bbr[BANKBLK] = {};
    if ((*stp_).R[0].valid) { const Blk &b = (*stp_).R[0]; bbr[win_of(b.base) % BANKBLK] = 1; bbr[win_of(b.bnd - SLOT_BYTES) % BANKBLK] = 1; }
    uint32_t wrm = 0;
    int nw = 0, kn = 0;
    UpdEnt keep[UQ_N];
    bool done[UQ_N] = {};
    for (int qi = 0; qi < (*stp_).uq_n; qi++) {
      if (done[qi]) continue;
      const UpdEnt &e = (*stp_).uq[qi];
      const uint32_t b = bank_of(e.pc);
      // block banks: one row write per (window bank) and cycle, reads win, entries of the same row (same window, same TAGE index set) share the write
      const uint32_t wb = bb_of(e.pc, BANKBLK);
      if (nw < UPD_PORTS && !bbr[wb] && !t.bb_wr[wb] && !((wrm >> b) & 1)) {
        apply_table_update(e, t); type_update(e, t);
        wrm |= 1u << b; nw++; t.bb_wr[wb] = 1;
        for (int qj = qi + 1; qj < (*stp_).uq_n; qj++) {   // entries of the same row (same window, same TAGE index set) share the write
          if (done[qj]) continue;
          const UpdEnt &f = (*stp_).uq[qj];
          const uint32_t fb = bank_of(f.pc);
          if (win_of(f.pc) != win_of(e.pc) || ((wrm >> fb) & 1) || std::memcmp(f.idx, e.idx, sizeof(e.idx)) != 0) continue;
          apply_table_update(f, t); type_update(f, t);
          wrm |= 1u << fb; done[qj] = true;
        }
      } else {
        keep[kn++] = e;
      }
    }
    for (int i = 0; i < kn; i++) n.uq[i] = keep[i];
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i] || in.in_actual_br_type[i] == BR_NONCTL) continue;
      if (kn < UQ_N) n.uq[kn++] = make_ent(in, i);   // a full queue drops the update
    }
    n.uq_n = kn;
  }

 public:
  // Fast-warmup replay (EXP-011): one committed instruction in program order, applied straight to the tables, the architectural
  // history / RAS and the uBTB (through the same block tracker the commit stream uses).
  void warm_commit(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool rvc = false) {
    State &n = (*stp_);
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
      TypeWr tw; type_.update(pc, type, tw, rvc); type_.commit(tw);
      if (type == BR_DIRECT) {
        TageUpd u;
        std::memset(&u, 0, sizeof(u));
        u.bank_pc = bpc; u.real_dir = dir; u.pred = pred; u.alt = tp.alt; u.pcpn = tp.pcpn;
        for (int k = 0; k < TN; k++) { u.tag[k] = tp.tag[k]; u.idx[k] = tp.idx[k]; }
        TageWr w; tage_[b].update(u, w); tage_[b].commit(w);
      } else if (br_call(type)) n.aras.push(pc + ilen_of(rvc));
      else if (type == BR_RET) n.aras.pop();
      BtbWr bw; btb_[b].update(bpc, type, dir, target, bw, tp.idx, tp.tag); btb_[b].commit(bw);
      warm_track(pc, type, dir, target, pred, rvc);
      return;
    }
    warm_track(pc, BR_NONCTL, false, 0, false, rvc);   // non-control: advance the block tracker as well
  }
  // warm-up: the block tracker sees one committed instruction, its uBTB events are written at once
  void warm_track(pc_t pc, br_type_t type, bool dir, target_addr_t target, bool pred, bool rvc) {
    static InputPayload in{};
    in.in_upd_valid[0] = true; in.in_update_base_pc[0] = pc; in.in_actual_br_type[0] = type; in.in_actual_dir[0] = dir;
    in.in_actual_targets[0] = target; in.in_pred_dir[0] = pred; in.in_update_is_rvc[0] = rvc;
    for (int i = 1; i < CW; i++) in.in_upd_valid[i] = false;
    State &n = (*stp_);
    Trk tmp = n;
    tmp.tq_n = 0;
    ub_track(in, tmp);
    for (int i = 0; i < tmp.tq_n; i++)
      for (UbTab *tb : {&ub0_, &ub1_}) {
        UbEnt nw[MAXW]; uint32_t wm;
        ub_rmw(*tb, tmp.tq[i], nw, wm);
        for (int w = 0; w < tb->W; w++) if ((wm >> w) & 1) tb->e[tb->index(tmp.tq[i].start) * tb->W + w] = nw[w];
      }
    n.tr_v = tmp.tr_v; n.tr_start = tmp.tr_start; n.tr_expect = tmp.tr_expect; n.tr_nd = tmp.tr_nd;
  }
  void warm_finish() { (*stp_).spec = (*stp_).arch; (*stp_).sras = RasQ::from_commit((*stp_).aras); }

  BpuPipe() { reset(); }
  static constexpr int depth() { return MAIN_LAT; }

  void reset() {
    for (auto &r : rec_) r.valid = false;
    ub0_.init(S0_N, S0_W, UB_TAGB);
    ub1_.init(L1_N, L1_W, UB_TAGB);
    sb_[0] = State{}; sb_[1] = State{}; stp_ = &sb_[0]; nxp_ = &sb_[1];
    type_.reset();
  }

  // Outcome of the last stage (registers only). Called first in the cycle: the front end uses squash_n to gate the
  // fetch-address FIFO pop and to drop its tail entries.
  void decide(bool refetch, Decision &d) const {
    // Only the scalar fields are cleared when no block leaves the pipeline: blk / out / ... are read only under d.valid (and d.ovr, d.pair, which imply it).
    d.valid = d.ovr = d.pair = d.ovr_b = false;
    d.squash_n = 0;
    const Blk &b = (*stp_).R[MAIN_LAT - 1];
    if (refetch || !b.valid) return;
    d.out = MainOut{}; d.outB = MainOut{}; d.oout = MainOut{}; d.ock = Ckpt{}; d.obase = 0; d.og = S0Guess{};
    d.valid = true;
    d.blk = &b;
    main_select(b, (*stp_).raw, d.out);
    int why;
    d.ovr = disagree(d.out, b.g, why);
    d.ock = b.ck; d.oout = d.out; d.obase = b.base; d.og = b.g;
    if (!d.ovr && b.pair) {   // FE_PAIR: the first block is confirmed, so the second one is checked as well
      d.pair = true;
      main_select(b.b2, (*stp_).raw_b, d.outB);
      S0Guess g2 = b.b2.g;
      const int lenA = d.out.slot + 1, lenB = d.outB.taken ? d.outB.slot + 1 : int((b.b2.bnd - b.b2.base) >> SLOT_SHIFT);
      if (!d.out.taken || lenA != b.g.slot + 1 || lenA + lenB > FW) {   // the final prediction does not fit the fetched group: back to one block, the second one is fetched again as a normal block
        d.pair = false; d.ovr = true;
      } else {
        d.ovr = d.ovr_b = disagree(d.outB, g2, why);
        if (d.ovr_b) { d.ock = b.b2.ck; d.oout = d.outB; d.obase = b.b2.base; d.og = g2; }
      }
    }
    if (d.ovr) for (int k = 0; k < MAIN_LAT - 1; k++) d.squash_n += (*stp_).R[k].valid;
  }

  // S1 level: a larger table read at S1 with the registered block address; on a hit that disagrees with the S0 guess
  // the guess of the block is replaced, the spec history / RAS are repaired from its checkpoint and S0 does not issue this cycle
  // (1 bubble instead of the 2 of an S_D override).
  bool l1_override(const Blk &b, State &n, Txn &t) const {
    if (!b.valid) return false;
    const S0Guess g1 = s0_lookup(ub1_, b.base, b.bnd, b.ck.ras_cnt, b.ck.ras_top);
    if (!g1.hit) return false;
    MainOut m; m.next = g1.next; m.nbits = g1.nbits; m.bits = g1.bits; m.ras_op = g1.ras_op; m.ras_val = g1.ras_val;
    int why;
    if (!disagree(m, b.g, why)) return false;
    t.pmu_l1_ovr = true;
    n.R[1].g = g1;
    n.R[1].pair = false;
    restore(b.ck, n.spec, n.sras);
    push_bits(g1.nbits, g1.bits, g1.ras_op, g1.ras_val, n.spec, n.sras);
    n.pc = g1.next;
    return true;
  }

  // n = s except R, raw and pq (the caller rebuilds R and raw; only pq[0..pq_n) are live)
  static void copy_state_but_R(State &n, const State &s) {
    static_assert(std::is_trivially_copyable_v<State>);
    char *dp = reinterpret_cast<char *>(&n);
    const char *sp = reinterpret_cast<const char *>(&s);
    const size_t a1 = reinterpret_cast<char *>(&n.pq) - dp, b1 = a1 + sizeof(n.pq);
    const size_t a2 = reinterpret_cast<char *>(&n.R) - dp, b2 = reinterpret_cast<char *>(&n.raw_b) - dp + sizeof(n.raw_b);   // R, raw, raw_b are adjacent
    std::memcpy(dp, sp, a1);
    std::memcpy(dp + a1, sp + a1, size_t(s.pq_n) * sizeof(Ckpt));
    std::memcpy(dp + b1, sp + b1, a2 - b1);
    std::memcpy(dp + b2, sp + b2, sizeof(State) - b2);
  }

  void comb(const InputPayload &in, const Decision &d, OutputPayload &out, Txn &t) const {
    std::memset(&out, 0, sizeof(out));
    t = Txn{};
    State &n = *nxp_;
    copy_state_but_R(n, *stp_);
    const bool refetch = in.refetch;
    // BPO_PDCKPT: the checker consumed the PTAB head in the previous cycle -> its checkpoint leaves the record queue
    Ckpt pd_rec; bool pd_have = false;
    if (in.pd.valid && (*stp_).pq_n > 0) {
      pd_rec = (*stp_).pq[0]; pd_have = true;
      for (int i = 1; i < (*stp_).pq_n; i++) n.pq[i - 1] = (*stp_).pq[i];
      n.pq_n = (*stp_).pq_n - 1;
    }

    // S0: look up the block starting at pc with the given speculative history / RAS, enter it into S1 and push the fetch address
    auto s0_issue = [&](pc_t pc, const HistRing &sp, const RasQ &sr) {
      Blk &b = n.R[0];   // n.R[0] is Blk{} here (cleared above; at most one block is issued per cycle)
      b.valid = true;
      b.base = pc;
      b.bnd = block_bnd(b.base);
      b.g = s0_lookup(ub0_, b.base, b.bnd, sr.cnt, sr.top());
      b.ck = take_ckpt(sp, sr);
      main_index(b, sp);
      push_bits(b.g.nbits, b.g.bits, b.g.ras_op, b.g.ras_val, n.spec, n.sras);
      n.pc = b.g.next;
      out.icache_read_valid = true;
      out.fetch_address = b.base;
      // FE_PAIR: the guess is a taken branch -> also look up the block at its target (spec history / RAS as left by the first block)
      const int pm = cfg_pair();   // FE_PAIR bits: 1 on; 2 = not after a return; 4 = not after a call; 8 = not after an indirect jump; 16 = not after a conditional branch
      const bool ty_ok = !((pm & 2) && b.g.type == BR_RET) && !((pm & 4) && br_call(b.g.type)) && !((pm & 8) && br_ind(b.g.type)) && !((pm & 16) && b.g.type == BR_DIRECT) && !((pm & 32) && b.g.type == BR_JAL);
      if (pm && ty_ok && b.g.hit && b.g.taken && int(b.g.slot) + 1 < FW) {
        const int lenA = b.g.slot + 1;
        const pc_t T = b.g.next;
        if ((T & (SLOT_BYTES - 1)) == 0 && T != b.base + SLOT_BYTES * lenA && (T >> 12) == (b.base >> 12)) {   // same 4 KiB page: one translation serves both blocks
          HistRing sp2 = sp; RasQ sr2 = sr;
          push_bits(b.g.nbits, b.g.bits, b.g.ras_op, b.g.ras_val, sp2, sr2);
          BlkB c;   // the second block is a normal fetch block (natural end), it must fit into the slots the first one leaves
          c.valid = true; c.base = T; c.bnd = block_bnd(T);
          c.g = s0_lookup(ub0_, T, c.bnd, sr2.cnt, sr2.top());
          const int room = FW - lenA, natlen = int((c.bnd - T) >> SLOT_SHIFT);
          const int lenB = c.g.taken ? c.g.slot + 1 : natlen;
          if (lenB <= room) {
            c.ck = take_ckpt(sp2, sr2);
            main_index(c, sp2);
            push_bits(c.g.nbits, c.g.bits, c.g.ras_op, c.g.ras_val, n.spec, n.sras);
            n.R[0].b2 = c;
            n.R[0].pair = true;
            n.pc = c.g.next;
            out.pair_valid = true; out.pair_addr2 = T; out.pair_a_len = lenA; out.pair_b_len = natlen < room ? natlen : room;
          }
        }
      }
    };

    // PTAB write: the block leaving S_D with the final prediction
    if (d.valid) {
      if (n.pq_n < PQ) n.pq[n.pq_n++] = d.blk->ck;
      out.PTAB_write_enable = true;
      out.out_pred_base_pc = d.blk->base;
      for (int i = 0; i < FW; i++) out.out_pred_dir[i] = d.out.dir[i];
      out.meta = d.out.meta;
      out.predict_next_fetch_address = d.out.next;
      if (d.pair) {   // FE_PAIR: the group is the first block (up to its taken branch) followed by the second block
        const int lenA = d.out.slot + 1;
        const BlkB &c = d.blk->b2;
        out.ptab_pcs_valid = true;
        for (int i = 0; i < FW; i++) { out.out_pred_pc[i] = i < lenA ? d.blk->base + SLOT_BYTES * i : c.base + SLOT_BYTES * (i - lenA); if (i >= lenA) out.out_pred_dir[i] = false; }
        for (int j = 0; lenA + j < FW; j++) {
          const int di = lenA + j;
          out.out_pred_dir[di] = d.outB.dir[j];
          out.meta.alt_pred[di] = d.outB.meta.alt_pred[j]; out.meta.pcpn[di] = d.outB.meta.pcpn[j];
          for (int k = 0; k < TN; k++) { out.meta.tage_tag[di][k] = d.outB.meta.tage_tag[j][k]; out.meta.tage_idx[di][k] = d.outB.meta.tage_idx[j][k]; }
          out.meta.ps_info[di] = d.outB.meta.ps_info[j];
        }
        out.predict_next_fetch_address = d.outB.next;
      }
      {
        t.rec_v = true;
        t.rec.ck = d.blk->ck; t.rec.base = d.blk->base; t.rec.id = (*stp_).blk_seq; t.rec.epoch = (*stp_).epoch; t.rec.valid = true;
        for (int i = 0; i < FW; i++) { out.meta.blk_id[i] = (*stp_).blk_seq; t.rec.ptype[i] = d.out.meta.ps_info[i] & 7; }
        n.blk_seq = (*stp_).blk_seq + 1;
        if (d.pair) {
          const int lenA = d.out.slot + 1;
          t.rec2_v = true;
          t.rec2.ck = d.blk->b2.ck; t.rec2.base = d.blk->b2.base; t.rec2.id = (*stp_).blk_seq + 1; t.rec2.epoch = (*stp_).epoch; t.rec2.valid = true;
          for (int i = 0; i < FW; i++) t.rec2.ptype[i] = d.outB.meta.ps_info[i] & 7;
          for (int i = lenA; i < FW; i++) out.meta.blk_id[i] = (*stp_).blk_seq + 1;
          n.blk_seq = (*stp_).blk_seq + 2;
        }
      }
    }

    // shift; the block in S1 is read this cycle and its raw array data rides along into the next stage register
    for (int k = MAIN_LAT - 1; k >= 1; k--) n.R[k] = (*stp_).R[k - 1];
    n.R[0] = Blk{};
    if ((*stp_).R[0].valid) {
      count_reads((*stp_).R[0], t);
      bb_reads((*stp_).R[0], t);
      main_read((*stp_).R[0], n.raw);
      if ((*stp_).R[0].pair) main_read((*stp_).R[0].b2, n.raw_b);
    }

    if (refetch) {
      for (int k = 0; k < MAIN_LAT; k++) n.R[k] = Blk{};
      n.pc = in.refetch_address;
    } else if (d.ovr) {
      for (int k = 0; k < MAIN_LAT; k++) n.R[k] = Blk{};
      restore(d.ock, n.spec, n.sras);
      push_bits(d.oout.nbits, d.oout.bits, d.oout.ras_op, d.oout.ras_val, n.spec, n.sras);
      n.pc = d.oout.next;
    } else if (l1_override((*stp_).R[0], n, t)) {
      // S1-level predictor disagrees with the S0 guess of the block in S1: that block's successor is not issued this cycle
    } else {
      // ---- S0 ---------------------------------------------------------------------------------------------------------
      int inflight = 0;
      for (int k = 0; k < MAIN_LAT; k++) inflight += (*stp_).R[k].valid;
      const bool ptab_stall = in.icache_read_ready && (int(in.ptab_size) + inflight >= PTAB_SIZE - 1);
      t.pmu_ptab_stall = ptab_stall;
      if (in.icache_read_ready && !ptab_stall) s0_issue((*stp_).pc, (*stp_).spec, (*stp_).sras);
    }

    // ---- commit side ---------------------------------------------------------------------------------------------------------
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const br_type_t ty = in.in_actual_br_type[i];
      push_arch(n.arch, in.in_update_base_pc[i], ty != BR_NONCTL && in.in_actual_dir[i], in.in_actual_targets[i], ty);
      if (br_call(ty)) n.aras.push(in.in_update_base_pc[i] + ilen_of(in.in_update_is_rvc[i]));
      else if (ty == BR_RET) n.aras.pop();
    }
    if (refetch) {
      if (in.refetch_pd && in.pd.valid && in.pd.flush && pd_have) {   // predecode flush: exact repair from the checked block's checkpoint
        restore(pd_rec, n.spec, n.sras);
        push_bits(in.pd.nbits, in.pd.bits, in.pd.ras_op, in.pd.ras_val, n.spec, n.sras);
      } else if (!(in.rf_valid && repair_blk(in, n))) { n.spec = n.arch; n.sras = RasQ::from_commit(n.aras); n.epoch = (*stp_).epoch + 1; }
      n.pq_n = 0;
      if (in.icache_read_ready) { const HistRing sp = n.spec; const RasQ sr = n.sras; s0_issue(in.refetch_address, sp, sr); }   // S0 issues the redirect target in the refetch cycle
    }
    update_queue(in, n, t);
    // predecode found no branch where the type predictor claimed one: the entry is stale (a non-control commit never trains the table).  It is cleared when no
    // update of the queue uses the bank's type lane this cycle (otherwise the stale entry causes another predecode flush and the clear is retried).
    if (in.pd.valid && in.pd.nb_valid && !((t.ty_lane >> bank_of(in.pd.nb_pc)) & 1)) {
      TypeWr &w = t.type[t.n_type];
      type_.update(in.pd.nb_pc, BR_NONCTL, w);
      if (w.en) { t.p_ty[w.bank].wr++; t.n_type++; }
    }
    // uBTB training: head of the train queue is written, new events are appended behind it
    if ((*stp_).tq_n > 0) {
      const UbEv &e = (*stp_).tq[0];
      ub_rmw(ub0_, e, t.ub_v[0], t.ub_wmask[0]); t.ub_set[0] = ub0_.index(e.start);
      ub_rmw(ub1_, e, t.ub_v[1], t.ub_wmask[1]); t.ub_set[1] = ub1_.index(e.start);
      for (int i = 1; i < (*stp_).tq_n; i++) n.tq[i - 1] = (*stp_).tq[i];
      n.tq_n = (*stp_).tq_n - 1;
    }
    ub_track(in, n);
    out.upd_free = uint8_t(UQ_N - (*stp_).uq_n);   // EXP-028 compact ROB commit credits (registered)
    out.tq_free = uint8_t(TQ_N - (*stp_).tq_n);
  }

  void commit(const Txn &t) {
    // declared port counts: S1 reads one set per bank and array; updates write one per bank and array; the arrays are single-ported.
    for (int b = 0; b < NB; b++) {
      const PortUse *p[3] = {&t.p_ty[b], &t.p_tg[b], &t.p_bt[b]};
      static const char *nm[3] = {"type", "tage", "btb"};
      for (int a = 0; a < 3; a++) {
        if (p[a]->rd > 1 || p[a]->wr > 1) {
          std::printf("[BPU_PIPE] ERROR: port limit exceeded on %s bank %d: rd=%d wr=%d\n", nm[a], b, p[a]->rd, p[a]->wr);
          std::exit(1);
        }
      }
    }
    for (int b = 0; b < BANKBLK; b++)
      if (t.bb_rd[b] > 1 || t.bb_wr[b] > 1 || t.bb_rd[b] + t.bb_wr[b] > 1) {
        std::printf("[BPU_PIPE] ERROR: block bank %d port limit exceeded: rd=%d wr=%d\n", b, t.bb_rd[b], t.bb_wr[b]);
        std::exit(1);
      }
    if (t.rec_v) rec_[t.rec.id & 255] = t.rec;
    if (t.rec2_v) rec_[t.rec2.id & 255] = t.rec2;
    for (int b = 0; b < NB; b++) { tage_[b].commit(t.tage[b]); btb_[b].commit(t.btb[b]); }
    for (int i = 0; i < t.n_type; i++) type_.commit(t.type[i]);
    for (int w = 0; w < ub0_.W; w++) if ((t.ub_wmask[0] >> w) & 1) ub0_.e[t.ub_set[0] * ub0_.W + w] = t.ub_v[0][w];
    for (int w = 0; w < ub1_.W; w++) if ((t.ub_wmask[1] >> w) & 1) ub1_.e[t.ub_set[1] * ub1_.W + w] = t.ub_v[1][w];
    std::swap(stp_, nxp_);
  }
};

}  // namespace bpuf

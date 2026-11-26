// Pipelined block-based 1-Ahead A + 2-Ahead B FTB branch predictor (v10: Zero-T0-Read Meta + WRBypass + Deep-Macro 1RW).
// Matches rtl/frontend/bpu/ cycle-for-cycle (docs/modules/bpu.md).
//
//   S0 (0 bubble)   Registered 1-ahead A & 2-ahead B SRAM select (aftb 2048x8, aftb2 1024x8, atage/atage2, aitt/aitt2)
//                   with 64-entry FF uFTB cold-start fallback; starts next-block ahead 1RW read; pushes spec hist/RAS.
//   S1 (1 bubble)   Exact-PC 1RW SRAM read (Main FTB 4096x4 4-Bank + S2 8-table ExBlockTage 8-Bank + SC 2-Bank + ITT 2-Bank).
//   S2 (2 bubbles)  Pipeline register stage (breaks S1 SRAM CLK->Q from S3 compare/eval for >=1.5 GHz timing closure).
//   S3 (3 bubbles)  Final select on registered S2 raw data -> PTAB write + ftb_rec_[512] Meta write; 3-bubble override on mismatch.
//   Commit side     16-entry 1-branch/cycle queue (brq[16]) + ftb_rec_[512] Meta + 8-entry FF WRBypass -> ZERO T0 SRAM reads!
//                   Native 1.00 br/cyc single-cycle pure writes; Predict wins FTB via 1-entry write buf; Write wins not-hit on TAGE/SC/ITT.
#pragma once
#include "BPU.h"
#include "BpuFtb.h"
#include "env.h"
#include "fpr.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

struct FrontTopIn;
struct FrontReadData;
class SimContext;
class PreIduQueue;

namespace bpuf {

constexpr int MAIN_LAT = 3;     // S0 -> S1 (1RW SRAM read) -> S2 (pipe reg) -> S3 (final eval/override)
inline int cfg_pair() { return 2; }
constexpr int PQ = 64;          // PTAB checkpoint records (BPO_PDCKPT)
constexpr int HR_BITS = 1024;   // circular global-history ring (widened for 2-taken fetch rate)
constexpr uint32_t HR_MASK = HR_BITS - 1u;

struct HistRing {
  uint64_t bits[HR_BITS / 64] = {};
  uint32_t head = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  uint16_t sc_fh[SC_GEHL_N] = {};
  bool back(uint32_t k) const { const uint32_t p = (head - 1 - k) & HR_MASK; return (bits[p >> 6] >> (p & 63)) & 1; }
  void push(bool dir) {
    const TageCfg &K = tcfg();
    uint32_t out[TN];
    for (int i = 0; i < TN; i++) {
      const uint32_t p = (head - K.ghr[i]) & HR_MASK;
      out[i] = (bits[p >> 6] >> (p & 63)) & 1;
    }
    for (int k = 0; k < FH_N_MAX; k++)
      for (int i = 0; i < TN; i++) {
        const uint32_t len = K.fl[k][i];
        uint32_t v = fh[k][i];
        const uint32_t hi = (v >> (len - 1)) & 1;
        v = ((v << 1) & ((1u << len) - 1)) | (uint32_t(dir) ^ hi);
        v ^= out[i] << (K.ghr[i] % len);
        fh[k][i] = v;
      }
    for (int i = 0; i < EX_TN; i++) {
      const uint32_t p = (head - uint32_t(EX_GHR_LEN[i])) & HR_MASK;
      const uint32_t ob = (bits[p >> 6] >> (p & 63)) & 1u;
      for (int k = 0; k < EX_FH_K; k++) {
        const uint32_t len = uint32_t(EX_FH_LEN[k]);
        uint32_t v = ex_fh[k][i];
        const uint32_t hi = (v >> (len - 1)) & 1u;
        v = ((v << 1) & ((1u << len) - 1u)) | (uint32_t(dir) ^ hi);
        v ^= ob << (uint32_t(EX_GHR_LEN[i]) % len);
        ex_fh[k][i] = static_cast<uint16_t>(v);
      }
    }
    for (int g = 0; g < SC_GEHL_N; g++) {
      const uint32_t p = (head - uint32_t(SC_GHR_LEN[g])) & HR_MASK;
      const uint32_t ob = (bits[p >> 6] >> (p & 63)) & 1u;
      const uint32_t len = uint32_t(SC_FH_LEN);
      uint32_t v = sc_fh[g];
      const uint32_t hi = (v >> (len - 1)) & 1u;
      v = ((v << 1) & ((1u << len) - 1u)) | (uint32_t(dir) ^ hi);
      v ^= ob << (uint32_t(SC_GHR_LEN[g]) % len);
      sc_fh[g] = static_cast<uint16_t>(v);
    }
    const uint32_t p = head;
    bits[p >> 6] = (bits[p >> 6] & ~(1ull << (p & 63))) | (uint64_t(dir) << (p & 63));
    head = (head + 1) & HR_MASK;
  }
};

struct RasC {
  target_addr_t st[RAS_DEPTH] = {};
  uint8_t ptr = 0, cnt = 0;
  target_addr_t top() const { return st[ptr]; }
  void push(target_addr_t v) { ptr = (ptr + 1) % RAS_DEPTH; st[ptr] = v; if (cnt < RAS_DEPTH) cnt++; }
  void pop() { if (cnt > 0) { ptr = (ptr + RAS_DEPTH - 1) % RAS_DEPTH; cnt--; } }
};

constexpr int RASQ = 64;
struct RasQ {
  target_addr_t st[RASQ] = {};
  uint8_t nos[RASQ] = {};
  uint8_t tosr = 0, tosw = 0, cnt = 0;
  target_addr_t top() const { return st[tosr]; }
  void push(target_addr_t v) { st[tosw] = v; nos[tosw] = tosr; tosr = tosw; tosw = (tosw + 1) % RASQ; if (cnt < RAS_DEPTH) cnt++; }
  void pop() { if (cnt > 0) { tosr = nos[tosr]; cnt--; } }
  static RasQ from_commit(const RasC &a) {
    RasQ q;
    for (int k = int(a.cnt) - 1; k >= 0; k--) q.push(a.st[(a.ptr + RAS_DEPTH - k) % RAS_DEPTH]);
    return q;
  }
};

struct Ckpt {
  pc_t base = 0;
  uint32_t head = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  uint16_t sc_fh[SC_GEHL_N] = {};
  uint8_t imli = 0;
  uint8_t ras_tosr = 0, ras_tosw = 0, ras_cnt = 0;
  target_addr_t ras_top = 0;
};

struct S0Guess {
  bool hit = false, taken = false;
  uint8_t slot = 0, type = 0;
  bool rvc = false, carried = false;
  pc_t next = 0;
  uint8_t nbits = 0, bits = 0;
  uint8_t ras_op = 0;
  target_addr_t ras_val = 0;
};

struct MainOut {
  pc_t next = 0;
  uint8_t slot = 0, type = 0;
  bool taken = false;
  uint8_t nbits = 0, bits = 0, ras_op = 0;
  target_addr_t ras_val = 0;
  bool rvc = false, carried = false;
  bool dir[FW] = {};
  PredMeta meta;
};

struct Blk {
  bool valid = false;
  bool sq = false;              // true iff an older block was in R[1..2] when issued -> squashable by S3 override
  pc_t base = 0, bnd = 0;
  S0Guess g;
  Ckpt ck;
  FtbBlkIdx ftb_ix;
  ExTageIdx ex_ix;
  ScIdx sc_ix;
  bool ah_v = false;
  bool ah_hit = false;
  pc_t ah_key_base = 0;
  uint32_t ah_key_fh[FH_N_MAX][TN] = {};
  BpuFtb::AhMeta ah_meta{};
  // Optional paired second block B in the same fetch group (2-taken)
  bool has_b = false;
  pc_t base_b = 0, bnd_b = 0;
  uint8_t a_len = 0, b_len = 0;
  S0Guess g_b;
  Ckpt ck_b;
  FtbBlkIdx ftb_ix_b;
  ExTageIdx ex_ix_b;
  ScIdx sc_ix_b;
  bool ah_v_b = false;
  uint8_t ah_kind_b = 2;
  pc_t ah_key_base_b = 0;
  uint32_t ah_key_fh_b[FH_N_MAX][TN] = {};
  BpuFtb::AhMeta ah_meta_b{};
};

struct Decision {
  bool valid = false;
  bool ovr = false;
  uint8_t squash_n = 0;
  const Blk *blk = nullptr;
  MainOut out;
  bool pair = false, ovr_b = false;
  MainOut outB;
  BpuFtb::S2Meta s2_meta{};
  BpuFtb::S2Meta s2_meta_b{};
  Ckpt ock; MainOut oout; pc_t obase = 0; S0Guess og;
};

struct RecP {
  Ckpt ck; pc_t base = 0; uint16_t id = 0; uint8_t epoch = 0; bool valid = false;
};

// The front end's BPU phase (ports named after the front end's wires).
//   CONTRACT comb_front  in: in.global_refetch in.global_reset in.in.ctrl in.in.redirect in.in.train in.pipe_dec in.rd.q  out: out.bpu_can_run out.bpu_output out.bpu_seq_txn out.pmu_l1_ovr out.pmu_ptab_stall
class BpuPipe {
 public:
  struct In {
    const bool *global_reset = nullptr;
    const bool *global_refetch = nullptr;
    const FrontTopIn *in = nullptr;
    const FrontReadData *rd = nullptr;
    const Decision *pipe_dec = nullptr;
  } in;
  const ::PreIduQueue *ftq = nullptr;   // read-only registered Unified FTQ-64 state
  SimContext *ctx = nullptr;
  struct SeqTxn { bool valid = false, reset = false; };
  struct Out {
    OutputPayload bpu_output{};
    bool bpu_can_run = false;
    SeqTxn bpu_seq_txn{};
    bool pmu_l1_ovr = false;
    bool pmu_ptab_stall = false;
  } out;
  void comb_front();

  struct State {
    pc_t pc = RESET_PC;
    Ckpt pq[PQ];
    uint8_t pq_h = 0, pq_n = 0;
    uint16_t blk_seq = 0;
    uint8_t epoch = 0;
    Blk R[3];
    FtbRaw ftb_raw;
    FtbRaw ftb_raw_b;
    FtbRaw ftb_raw_s3;
    FtbRaw ftb_raw_b_s3;
    FtbAhPipe ah_pipe;       // 0 backup copies (saves ~45k FFs in RTL!)
    FtbAhRaw ah2_pipe;
    FtbTrkState ftb_trk;
    HistRing spec, arch;
    RasQ sras;
    RasC aras;
    uint8_t spec_imli = 0, arch_imli = 0;
    // 1-entry pending FTB write buffers (Predict ALWAYS wins FTB 1RW banks!)
    FtbWrBuf ftb1_wb{};
    FtbWrBuf aftb_wb{};
    FtbWrBuf aftb2_wb{};
    // 1-entry pending predecode false-branch clear
    bool clr_pend_v = false;
    pc_t clr_pend_base = 0, clr_pend_pc = 0;
  };

  struct Txn {
    bool rec_v = false, rec_b_v = false;
    RecP rec, rec_b;
    bool pmu_l1_ovr = false, pmu_ptab_stall = false;
    bool ftb_rec_v = false, ftb_rec_b_v = false;
    FtbBlkRec ftb_rec{}, ftb_rec_b{};
    // 1-cycle zero-T0-read commit plan
    BpuFtb::CommitPlan cp{};
    bool do_s2_base = false;
    // Single-way FTB writes firing this cycle & next write-buffer states
    bool do_ftb1_wr = false;
    FtbWrBuf ftb1_wr{};
    FtbWrBuf next_ftb1_wb{};
    bool do_aftb_wr = false;
    FtbWrBuf aftb_wr{};
    FtbWrBuf next_aftb_wb{};
    bool do_aftb2_wr = false;
    FtbWrBuf aftb2_wr{};
    FtbWrBuf next_aftb2_wb{};
    // Predecode false-branch clear
    bool do_clr = false;
    pc_t clr_base = 0, clr_pc = 0;
    bool next_clr_pend_v = false;
    pc_t next_clr_pend_base = 0, next_clr_pend_pc = 0;
    // Busy masks for deferred 1RW u-bit periodic reset
    uint8_t s2_tbl_busy[EX_TN] = {};
    uint8_t ah1_tbl_busy[TN] = {};
    uint8_t ah2_tbl_busy[TN] = {};
    void clear() {
      rec_v = rec_b_v = false;
      pmu_l1_ovr = pmu_ptab_stall = false;
      ftb_rec_v = ftb_rec_b_v = false;
      cp = BpuFtb::CommitPlan{};
      do_s2_base = false;
      do_ftb1_wr = do_aftb_wr = do_aftb2_wr = false;
      next_ftb1_wb = FtbWrBuf{};
      next_aftb_wb = FtbWrBuf{};
      next_aftb2_wb = FtbWrBuf{};
      do_clr = false;
      next_clr_pend_v = false;
      std::memset(s2_tbl_busy, 0, sizeof s2_tbl_busy);
      std::memset(ah1_tbl_busy, 0, sizeof ah1_tbl_busy);
      std::memset(ah2_tbl_busy, 0, sizeof ah2_tbl_busy);
    }
  };

 private:
  Txn txn_d_;
  bool txn_d_valid_ = false;
  static constexpr int hsym_ = HSYM_BITS;
  State st_;
  State *const stp_ = &st_;
  BpuFtb ftb_;
  FtbBlkRec ftb_rec_[FTB_REC_N];
  RecP rec_[256];

  static pc_t block_bnd(pc_t base) {
    const pc_t mask = ~static_cast<pc_t>(ICACHE_LINE_SIZE - 1);
    const pc_t plus = base + FW * SLOT_BYTES;
    return (base & mask) != (plus & mask) ? (plus & mask) : plus;
  }

  // 4-bank 1R1W ICache bank-conflict filter (2-port ITLB allows cross-page pairing)
  static bool ic_pair_ok(pc_t pcA, pc_t pcB) {
    const uint64_t lA = pcA >> 6;
    const uint64_t lB = pcB >> 6;
    return (lA == lB) || ((lA & 3u) != (lB & 3u));
  }

  static Ckpt take_ckpt(const HistRing &h, const RasQ &r, uint8_t imli) {
    Ckpt c;
    c.head = h.head;
    std::memcpy(c.fh, h.fh, sizeof(c.fh));
    std::memcpy(c.ex_fh, h.ex_fh, sizeof(c.ex_fh));
    std::memcpy(c.sc_fh, h.sc_fh, sizeof(c.sc_fh));
    c.imli = imli;
    c.ras_tosr = r.tosr; c.ras_tosw = r.tosw; c.ras_cnt = r.cnt; c.ras_top = r.top();
    return c;
  }
  static void restore(const Ckpt &c, HistRing &h, RasQ &r, uint8_t &imli) {
    h.head = c.head;
    std::memcpy(h.fh, c.fh, sizeof(c.fh));
    std::memcpy(h.ex_fh, c.ex_fh, sizeof(c.ex_fh));
    std::memcpy(h.sc_fh, c.sc_fh, sizeof(c.sc_fh));
    imli = c.imli;
    r.tosr = c.ras_tosr; r.tosw = c.ras_tosw; r.cnt = c.ras_cnt;
  }
  static void upd_imli(uint8_t &imli, uint8_t ty, bool taken, pc_t br_pc, pc_t tgt, bool reset_on_nt = false) {
    if (ty == BR_DIRECT) {
      if (taken) imli = (tgt < br_pc) ? ((imli < 255) ? uint8_t(imli + 1) : 255) : 0;
      else if (reset_on_nt) imli = 0;
    }
  }
  uint8_t hsym(pc_t bpc, pc_t tgt) const { return hist_sym(bpc, tgt); }
  void push_arch(HistRing &h, pc_t bpc, bool taken, pc_t tgt, br_type_t ty) const {
    (void)ty;
    if (!taken) return;
    const uint8_t b = hsym(bpc, tgt);
    for (int i = 0; i < hsym_; i++) h.push((b >> i) & 1);
  }
  static void hp(const char *site, pc_t pc, pc_t tgt, uint8_t n, uint8_t bits) {
    static const bool tr = std::getenv("HP_TRACE") != nullptr;
    if (tr && fpr::in_dump()) std::fprintf(stderr, "[HP] c=%llu %s pc=%llx tgt=%llx n=%u bits=%x\n", (unsigned long long)fpr::cycle(), site, (unsigned long long)pc, (unsigned long long)tgt, n, bits);
  }
  static void push_bits(uint8_t n, uint8_t bits, uint8_t ras_op, target_addr_t ras_val, HistRing &h, RasQ &r) {
    for (int i = 0; i < n; i++) h.push((bits >> i) & 1);
    if (ras_op == 1) r.push(ras_val); else if (ras_op == 2) r.pop();
  }

  static bool disagree(const MainOut &m, const S0Guess &g, bool check_slot = false) {
    if (m.next != g.next) return true;
    if (m.nbits != g.nbits || m.bits != g.bits) return true;
    if (m.ras_op != g.ras_op || (m.ras_op == 1 && m.ras_val != g.ras_val)) return true;
    if (check_slot && (m.taken != g.taken || m.slot != g.slot)) return true;
    return false;
  }

  bool repair_blk(const InputPayload &in, State &n) const {
    const RecP &rc = rec_[in.rf_blk & 255];
    const bool carried = rc.valid && in.rf_pc + SLOT_BYTES == rc.base;
    const uint32_t rslot = carried ? 0 : (in.rf_pc - rc.base) >> SLOT_SHIFT;
    if (!(rc.valid && rc.id == in.rf_blk && rc.epoch == (*stp_).epoch && rslot < FW && (carried || rc.base + SLOT_BYTES * rslot == in.rf_pc))) return false;
    restore(rc.ck, n.spec, n.sras, n.spec_imli);
    const bool tk = in.rf_type == BR_DIRECT ? (in.refetch_address != in.rf_pc + ilen_of(in.rf_is_rvc)) : true;
    if (tk) { const uint8_t sy = hsym(in.rf_pc, in.refetch_address); hp("RFIX", in.rf_pc, in.refetch_address, hsym_, sy); for (int k = 0; k < hsym_; k++) n.spec.push((sy >> k) & 1); }
    upd_imli(n.spec_imli, in.rf_type, tk, in.rf_pc, in.refetch_address, true);
    if (br_call(in.rf_type)) n.sras.push(in.rf_pc + ilen_of(in.rf_is_rvc)); else if (in.rf_type == BR_RET) n.sras.pop();
    return true;
  }

  static void ftb_emit_warm(FtbTrkState &ft, BpuFtb &ftb) {
    if (ft.nbr == 0) return;
    FtbTrainEv ev{};
    ev.start = ft.start;
    std::memcpy(ev.fh, ft.fh, sizeof ev.fh);
    std::memcpy(ev.ex_fh, ft.ex_fh, sizeof ev.ex_fh);
    std::memcpy(ev.sc_fh, ft.sc_fh, sizeof ev.sc_fh);
    ev.imli = ft.imli;
    ev.nbr = ft.nbr;
    for (int i = 0; i < ft.nbr; i++) ev.br[i] = ft.br[i];
    ftb.apply_train_ev(ev);
  }

  static void fill_ftb_rec(FtbBlkRec &dst, uint16_t id, pc_t base, const Ckpt &ck,
                           bool ah_v, uint8_t ah_kind, pc_t ah_a_base, pc_t ah_key_base,
                           const uint32_t ah_key_fh[FH_N_MAX][TN],
                           const BpuFtb::S2Meta &s2m, const BpuFtb::AhMeta &ahm) {
    dst = FtbBlkRec{};
    dst.valid = true;
    dst.id = id;
    dst.base = base;
    std::memcpy(dst.fh, ck.fh, sizeof dst.fh);
    std::memcpy(dst.ex_fh, ck.ex_fh, sizeof dst.ex_fh);
    std::memcpy(dst.sc_fh, ck.sc_fh, sizeof dst.sc_fh);
    dst.imli = ck.imli;
    dst.ah_v = ah_v;
    dst.ah_kind = ah_kind;
    dst.ah_a_base = ah_a_base;
    dst.ah_key_base = ah_key_base;
    std::memcpy(dst.ah_key_fh, ah_key_fh, sizeof dst.ah_key_fh);
    dst.ftb1_meta = s2m.ftb;
    dst.s2_tage_meta = s2m.tage;
    dst.s2_sc_meta = s2m.sc;
    dst.s2_itt_meta = s2m.itt;
    dst.ah_ftb_meta = ahm.ftb;
    dst.ah_tage_meta = ahm.tage;
    dst.ah_itt_meta = ahm.itt;
  }

 public:
  void warm_commit(pc_t pc, br_type_t ty, bool dir, target_addr_t target, bool rvc = false) {
    State &n = (*stp_);
    FtbTrkState &ft = n.ftb_trk;
    bool pred = false, alt = false; uint8_t pcpn = TN;
    if (ty == BR_DIRECT && ft.v && pc >= ft.start && pc < block_bnd(ft.start)) {
      FtbEnt ent;
      if (ftb_.ftb1_.lookup(ft.start, ent)) {
        const uint8_t off = static_cast<uint8_t>((pc - ft.start) >> SLOT_SHIFT);
        for (int k = 0; k < FTB_SLOTS; k++) {
          if (ent.s[k].valid && ent.s[k].offset == off) {
            const uint32_t w = static_cast<uint32_t>(ft.start >> SLOT_SHIFT);
            const ExTageIdx ex_ix = make_ex_tage_index(w, ft.ex_fh);
            ExBlockTage::Pred tp;
            ftb_.s2_tage_.predict_slot(ex_ix, k, tp);
            pred = tp.pred; alt = tp.alt;
            pcpn = (tp.pcpn < EX_TN) ? static_cast<uint8_t>(std::min<int>(tp.pcpn, TN - 1)) : static_cast<uint8_t>(TN);
            break;
          }
        }
      }
    }
    auto sync_ft_hist = [&] {
      std::memcpy(ft.fh, n.arch.fh, sizeof ft.fh);
      std::memcpy(ft.ex_fh, n.arch.ex_fh, sizeof ft.ex_fh);
      std::memcpy(ft.sc_fh, n.arch.sc_fh, sizeof ft.sc_fh);
      ft.imli = n.arch_imli;
    };
    if (!ft.v || pc != ft.expect || pc < ft.start || pc >= block_bnd(ft.start)) {
      if (ft.v && ft.nbr > 0) ftb_emit_warm(ft, ftb_);
      ft.v = true; ft.start = pc; ft.nbr = 0;
      sync_ft_hist();
    }
    const pc_t bnd = block_bnd(ft.start);
    const uint8_t off = static_cast<uint8_t>((pc - ft.start) >> SLOT_SHIFT);
    const bool taken = (ty != BR_NONCTL) && dir;
    if (ty != BR_NONCTL && ft.nbr < FW) {
      FtbTrkBr &b = ft.br[ft.nbr++];
      b.offset = off; b.carried = false; b.is_rvc = rvc; b.type = ty;
      b.actual_dir = taken; b.pred_dir = pred; b.alt_pred = alt; b.pcpn = pcpn; b.target = target;
    }
    push_arch(n.arch, pc, taken, target, ty);
    upd_imli(n.arch_imli, ty, taken, pc, target, false);
    if (br_call(ty)) n.aras.push(pc + ilen_of(rvc));
    else if (ty == BR_RET) n.aras.pop();
    if (taken) {
      ftb_emit_warm(ft, ftb_);
      ft.start = target; ft.expect = target; ft.nbr = 0;
      sync_ft_hist();
    } else {
      const pc_t nxt = pc + ilen_of(rvc);
      if (ty == BR_DIRECT && pred) {
        ftb_emit_warm(ft, ftb_);
        ft.start = nxt; ft.nbr = 0;
        sync_ft_hist();
      } else if (nxt >= bnd) {
        ftb_emit_warm(ft, ftb_);
        ft.start = bnd; ft.nbr = 0;
        sync_ft_hist();
      }
      ft.expect = nxt;
    }
  }
  void warm_finish() { (*stp_).spec = (*stp_).arch; (*stp_).sras = RasQ::from_commit((*stp_).aras); (*stp_).spec_imli = (*stp_).arch_imli; }

  BpuPipe() { reset(); }
  void comb_cycle(const InputPayload &in, const Decision &d, OutputPayload &out) { comb(in, d, out, txn_d_); txn_d_valid_ = true; }
  void seq() { if (txn_d_valid_) commit(txn_d_); txn_d_valid_ = false; }

  void dump_hist(uint32_t c) const {
    auto fhash = [](const HistRing &h) { uint64_t x = 1469598103934665603ull; for (int k = 0; k < FH_N_MAX; k++) for (int i = 0; i < TN; i++) x = (x ^ h.fh[k][i]) * 1099511628211ull; return x; };
    auto recent = [](const HistRing &h) { uint64_t x = 0; for (int k = 0; k < 64; k++) x |= uint64_t(h.back(k)) << k; return x; };
    std::fprintf(stderr, "H c=%u sr=%016llx sf=%016llx ar=%016llx af=%016llx\n", c, (unsigned long long)recent(st_.spec), (unsigned long long)fhash(st_.spec), (unsigned long long)recent(st_.arch),
                 (unsigned long long)fhash(st_.arch));
  }

  static constexpr int depth() { return MAIN_LAT; }
  uint8_t hold_n() const {
    return static_cast<uint8_t>(((*stp_).R[0].valid && (*stp_).R[0].sq) +
                                ((*stp_).R[1].valid && (*stp_).R[1].sq));
  }

  void reset() {
    for (auto &r : rec_) r.valid = false;
    for (auto &r : ftb_rec_) r.valid = false;
    ftb_.reset();
    st_ = State{};
  }

  void decide(bool refetch, Decision &d) const {
    d.valid = d.ovr = d.pair = d.ovr_b = false;
    d.squash_n = 0;
    const Blk &b = (*stp_).R[MAIN_LAT - 1];
    if (refetch || !b.valid) return;
    d.valid = true;
    d.blk = &b;
    const FtbRaw &raw_a = (*stp_).ftb_raw_s3;
    const FtbRaw &raw_b = (*stp_).ftb_raw_b_s3;
    ftb_.s2_select(b.base, b.bnd, b.ftb_ix, b.ex_ix, raw_a, b.ck.ras_cnt, b.ck.ras_top, d.out, &d.s2_meta);
    const bool ovr_a = disagree(d.out, b.g, b.has_b);
    if (!ovr_a && b.has_b) {
      ftb_.s2_select(b.base_b, b.bnd_b, b.ftb_ix_b, b.ex_ix_b, raw_b, b.ck_b.ras_cnt, b.ck_b.ras_top, d.outB, &d.s2_meta_b);
      d.pair = true;
      d.ovr_b = disagree(d.outB, b.g_b, false);
    }
    d.ovr = ovr_a || d.ovr_b;
    if (ovr_a || !d.pair) {
      d.ock = b.ck; d.oout = d.out; d.obase = b.base; d.og = b.g;
    } else {
      d.ock = b.ck_b; d.oout = d.outB; d.obase = b.base_b; d.og = b.g_b;
    }
    if (d.ovr) for (int k = 0; k < MAIN_LAT - 1; k++) d.squash_n += (*stp_).R[k].valid;
  }

  void comb(const InputPayload &in, const Decision &d, OutputPayload &out, Txn &t) {
    static_assert(MAIN_LAT >= 2, "the S1 block moves to R[1] before it is read");
    out = {};
    t.clear();
    State &n = st_;
    const uint16_t seq0 = n.blk_seq;
    const uint8_t epoch0 = n.epoch;

    Ckpt pd_rec; bool pd_have = false;
    if (in.pd.valid && n.pq_n > 0) {
      pd_rec = n.pq[n.pq_h]; pd_have = true;
      n.pq_h = uint8_t((n.pq_h + 1) % PQ); n.pq_n--;
    }

    bool s0_launched = false;
    pc_t s0_ah_base = 0;
    uint32_t s0_ah_fh[FH_N_MAX][TN] = {};

    auto s0_issue = [&](pc_t pc) {
      Blk &b = n.R[0];
      b = Blk{};
      b.valid = true;
      b.sq = n.R[1].valid || n.R[2].valid;
      b.base = pc;
      b.bnd = block_bnd(b.base);
      const FtbAhPipe ah_src = n.ah_pipe;
      const FtbAhRaw ah2_src = n.ah2_pipe;
      b.ah_v = ah_src.valid;
      b.ah_key_base = ah_src.key_base;
      std::memcpy(b.ah_key_fh, ah_src.key_fh, sizeof b.ah_key_fh);
      b.ah_hit = ftb_.ahead_select(b.base, b.bnd, ah_src, n.sras.cnt, n.sras.top(), b.g, &b.ah_meta);
      if (!b.ah_hit) b.g = ftb_.s0_lookup<S0Guess>(b.base, b.bnd, n.sras.cnt, n.sras.top());

      b.ck = take_ckpt(n.spec, n.sras, n.spec_imli);
      b.ck.base = b.base;
      const uint32_t wA = static_cast<uint32_t>(b.base >> SLOT_SHIFT);
      b.ftb_ix = BpuFtb::make_index(b.base, n.spec.fh);
      b.ex_ix = make_ex_tage_index(wA, n.spec.ex_fh);
      b.sc_ix = make_sc_index(wA, n.spec.sc_fh, n.spec_imli);
      hp("S0", b.base, b.g.next, b.g.nbits, b.g.bits);
      push_bits(b.g.nbits, b.g.bits, b.g.ras_op, b.g.ras_val, n.spec, n.sras);
      upd_imli(n.spec_imli, b.g.type, b.g.taken, b.base, b.g.next, false);

      // 2-taken: optional paired second block B predicted in parallel via 2-ahead B
      bool paired = false;
      if (b.g.taken) {
        const uint8_t a_len = static_cast<uint8_t>(b.g.slot + 1);
        const bool a_upper_ok = b.g.rvc || (b.g.slot == 0 && b.g.carried) ||
                                (b.base + static_cast<pc_t>(b.g.slot + 1) * SLOT_BYTES < b.bnd);
        const pc_t pcB = b.g.next;
        const pc_t a_br_pc = (b.g.slot == 0 && b.g.carried)
                                 ? (b.base - SLOT_BYTES)
                                 : (b.base + static_cast<pc_t>(b.g.slot) * SLOT_BYTES);
        const bool disc_ok = (pcB != b.base + static_cast<pc_t>(a_len) * SLOT_BYTES) &&
                             (pcB != a_br_pc + ilen_of(b.g.rvc));
        const uint32_t wB = static_cast<uint32_t>(pcB >> SLOT_SHIFT);
        const bool bank_ok = (s2_bank_of_w(wA) != s2_bank_of_w(wB));
        if (a_len >= 1 && a_len < FW && a_upper_ok && disc_ok && ic_pair_ok(b.base, pcB) && bank_ok) {
          const pc_t max_b_end = pcB + static_cast<pc_t>(FW - a_len) * SLOT_BYTES;
          const pc_t full_bnd_b = block_bnd(pcB);
          const pc_t bndB = full_bnd_b < max_b_end ? full_bnd_b : max_b_end;
          const bool b_truncated = (full_bnd_b > max_b_end);
          const uint8_t b_len = static_cast<uint8_t>((bndB - pcB) >> SLOT_SHIFT);
          if (b_len >= 2) {
            S0Guess gB{};
            BpuFtb::AhMeta ah_meta_b{};
            const bool ah_b_hit = ftb_.ahead2_select(pcB, bndB, ah_src.valid, ah_src.key_base, ah_src.key_fh,
                                                     ah2_src, n.sras.cnt, n.sras.top(), gB, b.base, &ah_meta_b);
            if (!ah_b_hit) gB.next = bndB;
            if (!b_truncated || gB.taken) {
              paired = true;
              b.has_b = true;
              b.base_b = pcB;
              b.bnd_b = bndB;
              b.a_len = a_len;
              b.b_len = b_len;
              b.g_b = gB;
              b.ah_meta_b = ah_meta_b;
              b.ck_b = take_ckpt(n.spec, n.sras, n.spec_imli);
              b.ck_b.base = pcB;
              b.ftb_ix_b = BpuFtb::make_index(pcB, n.spec.fh);
              b.ex_ix_b = make_ex_tage_index(wB, n.spec.ex_fh);
              b.sc_ix_b = make_sc_index(wB, n.spec.sc_fh, n.spec_imli);
              b.ah_v_b = ah_src.valid;
              b.ah_kind_b = 2;
              b.ah_key_base_b = ah_src.key_base;
              std::memcpy(b.ah_key_fh_b, ah_src.key_fh, sizeof b.ah_key_fh_b);
              push_bits(gB.nbits, gB.bits, gB.ras_op, gB.ras_val, n.spec, n.sras);
              upd_imli(n.spec_imli, gB.type, gB.taken, pcB, gB.next, false);
            }
          }
        }
      }

      // Launch next-block 1-ahead & 2-ahead reads from the LAST block of this cycle (B if paired, else A)
      const pc_t next_ah_base = paired ? b.base_b : b.base;
      const uint32_t (*next_ah_fh)[TN] = paired ? b.ck_b.fh : b.ck.fh;
      s0_launched = true;
      s0_ah_base = next_ah_base;
      std::memcpy(s0_ah_fh, next_ah_fh, sizeof s0_ah_fh);
      n.ah_pipe.valid = true;
      n.ah_pipe.key_base = next_ah_base;
      std::memcpy(n.ah_pipe.key_fh, next_ah_fh, sizeof n.ah_pipe.key_fh);

      n.pc = paired ? b.g_b.next : b.g.next;
      out.icache_read_valid = true;
      out.fetch_address = b.base;
      if (paired) {
        out.pair_valid = true;
        out.pair_addr2 = b.base_b;
        out.pair_a_len = b.a_len;
        out.pair_b_len = b.b_len;
      }
    };

    if (d.valid) {
      if (n.pq_n < PQ) { n.pq[(n.pq_h + n.pq_n) % PQ] = d.blk->ck; n.pq_n++; }
      out.PTAB_write_enable = true;
      out.out_pred_base_pc = d.blk->base;
      if (!d.pair) {
        for (int i = 0; i < FW; i++) out.out_pred_dir[i] = d.out.dir[i];
        out.meta = d.out.meta;
        out.predict_next_fetch_address = d.out.next;
        t.rec_v = true;
        t.rec.ck = d.blk->ck; t.rec.base = d.blk->base; t.rec.id = seq0; t.rec.epoch = epoch0; t.rec.valid = true;
        t.ftb_rec_v = true;
        fill_ftb_rec(t.ftb_rec, seq0, d.blk->base, d.blk->ck,
                     d.blk->ah_v, 1, 0, d.blk->ah_key_base, d.blk->ah_key_fh,
                     d.s2_meta, d.blk->ah_meta);
        for (int i = 0; i < FW; i++) out.meta.blk_id[i] = seq0;
        n.blk_seq = seq0 + 1;
      } else {
        const int al = d.blk->a_len;
        const uint16_t seq1 = static_cast<uint16_t>(seq0 + 1);
        out.ptab_pcs_valid = true;
        for (int i = 0; i < al; i++) {
          out.out_pred_pc[i] = d.blk->base + static_cast<pc_t>(i) * SLOT_BYTES;
          out.out_pred_dir[i] = d.out.dir[i];
          out.meta.alt_pred[i] = d.out.meta.alt_pred[i];
          out.meta.pcpn[i] = d.out.meta.pcpn[i];
          out.meta.ps_info[i] = d.out.meta.ps_info[i];
          out.meta.blk_id[i] = seq0;
        }
        for (int i = al; i < FW; i++) {
          const int j = i - al;
          out.out_pred_pc[i] = d.blk->base_b + static_cast<pc_t>(j) * SLOT_BYTES;
          out.out_pred_dir[i] = d.outB.dir[j];
          out.meta.alt_pred[i] = d.outB.meta.alt_pred[j];
          out.meta.pcpn[i] = d.outB.meta.pcpn[j];
          out.meta.ps_info[i] = d.outB.meta.ps_info[j];
          out.meta.blk_id[i] = seq1;
        }
        out.predict_next_fetch_address = d.outB.next;
        t.rec_v = true;
        t.rec.ck = d.blk->ck; t.rec.base = d.blk->base; t.rec.id = seq0; t.rec.epoch = epoch0; t.rec.valid = true;
        t.rec_b_v = true;
        t.rec_b.ck = d.blk->ck_b; t.rec_b.base = d.blk->base_b; t.rec_b.id = seq1; t.rec_b.epoch = epoch0; t.rec_b.valid = true;
        t.ftb_rec_v = true;
        fill_ftb_rec(t.ftb_rec, seq0, d.blk->base, d.blk->ck,
                     d.blk->ah_v, 1, 0, d.blk->ah_key_base, d.blk->ah_key_fh,
                     d.s2_meta, d.blk->ah_meta);
        t.ftb_rec_b_v = true;
        fill_ftb_rec(t.ftb_rec_b, seq1, d.blk->base_b, d.blk->ck_b,
                     d.blk->ah_v_b, d.blk->ah_kind_b, d.blk->base, d.blk->ah_key_base_b, d.blk->ah_key_fh_b,
                     d.s2_meta_b, d.blk->ah_meta_b);
        n.blk_seq = seq0 + 2;
      }
    }

    n.ftb_raw_s3 = n.ftb_raw;
    n.ftb_raw_b_s3 = n.ftb_raw_b;
    const Blk s1_blk = n.R[0];  // pre-shift R[0]: registered S1 block reading S2 SRAM this cycle
    for (int k = MAIN_LAT - 1; k >= 1; k--) n.R[k] = n.R[k - 1];
    n.R[0].valid = false;

    auto clear_R = [&] { for (int k = 0; k < MAIN_LAT; k++) n.R[k].valid = false; };
    if (in.refetch) {
      clear_R();
      n.ah_pipe.valid = false;
      n.pc = in.refetch_address;
      if (in.refetch_pd && in.pd.valid && in.pd.flush && pd_have) {
        restore(pd_rec, n.spec, n.sras, n.spec_imli);
        hp("PDFIX", 0, 0, in.pd.nbits, in.pd.bits);
        push_bits(in.pd.nbits, in.pd.bits, in.pd.ras_op, in.pd.ras_val, n.spec, n.sras);
        if (in.pd.ras_op2 == 1) n.sras.push(in.pd.ras_val2);
        else if (in.pd.ras_op2 == 2) n.sras.pop();
      } else if (!(in.rf_valid && repair_blk(in, n))) {
        hp("RARCH", in.rf_pc, in.refetch_address, 0, 0);
        n.spec = n.arch; n.sras = RasQ::from_commit(n.aras); n.spec_imli = n.arch_imli; n.epoch = epoch0 + 1;
      }
      n.pq_n = 0;
      if (in.icache_read_ready) s0_issue(in.refetch_address);
    } else if (d.ovr) {
      clear_R();
      // Zero-FF ahead recovery: S0 is idle on d.ovr cycles, so re-read Ahead 1RW SRAMs directly from (d.obase, d.ock.fh)!
      s0_launched = true;
      s0_ah_base = d.obase;
      std::memcpy(s0_ah_fh, d.ock.fh, sizeof s0_ah_fh);
      n.ah_pipe.valid = true;
      n.ah_pipe.key_base = d.obase;
      std::memcpy(n.ah_pipe.key_fh, d.ock.fh, sizeof n.ah_pipe.key_fh);
      restore(d.ock, n.spec, n.sras, n.spec_imli);
      hp("OVR", 0, d.oout.next, d.oout.nbits, d.oout.bits);
      push_bits(d.oout.nbits, d.oout.bits, d.oout.ras_op, d.oout.ras_val, n.spec, n.sras);
      upd_imli(n.spec_imli, d.oout.type, d.oout.taken, d.obase, d.oout.next, false);
      n.pc = d.oout.next;
    } else {
      t.pmu_ptab_stall = !in.icache_read_ready;
      if (in.icache_read_ready) s0_issue(n.pc);
    }

    // ---- Step 1: Zero-T0-Read Commit Pop & 1-Cycle Pure Write Plan (native 1.00 br/cyc!) ----
    FtbTrkState &ft = n.ftb_trk;
    if (ft.brq_n > 0) {
      const FtbBrqEnt &q = ft.brq[ft.brq_h];
      const FtbBlkRec &rc = ftb_rec_[q.blk_id & (FTB_REC_N - 1)];
      const bool carried = (q.pc + SLOT_BYTES == rc.base);
      const bool valid_rec = rc.valid && rc.id == q.blk_id && (carried || (q.pc >= rc.base && q.pc < block_bnd(rc.base)));
      ft.brq_h = static_cast<uint8_t>((ft.brq_h + 1) % FTB_BRQ_N);
      ft.brq_n--;
      if (valid_rec) {
        FtbTrkBr br{};
        br.offset = carried ? 0 : static_cast<uint8_t>((q.pc - rc.base) >> SLOT_SHIFT);
        br.carried = carried;
        br.is_rvc = q.is_rvc;
        br.type = q.type;
        br.actual_dir = q.actual_dir;
        br.pred_dir = q.pred_dir;
        br.alt_pred = q.alt_pred;
        br.pcpn = q.pcpn;
        br.target = q.target;
        ftb_.compute_commit_plan(rc, br, t.cp);
        fpr::up(static_cast<uint32_t>(q.pc), q.actual_dir, q.type, static_cast<uint32_t>(q.target), q.pred_dir,
                q.pcpn, q.is_rvc, static_cast<uint64_t>(static_cast<uint32_t>(rc.base)),
                (static_cast<uint32_t>(carried) << 4) | static_cast<uint32_t>(br.offset));
      }
    }

    // Build per-subtable 1RW write-bank masks for TAGE / SC / ITT (write-wins not-hit!)
    uint8_t s2_tage_busy[EX_TN] = {};
    uint8_t s2_sc_busy[7] = {};
    uint8_t s2_itt_busy[ITT_MAX] = {};
    uint8_t ah1_tage_busy[TN] = {}, ah2_tage_busy[TN] = {};
    uint8_t ah1_itt_busy[ITT_MAX] = {}, ah2_itt_busy[ITT_MAX] = {};

    if (t.cp.valid) {
      for (int k = 0; k < EX_TN; k++) {
        if ((t.cp.tage_wp.tbl_wr_mask >> k) & 1u) {
          s2_tage_busy[k] |= static_cast<uint8_t>(1u << (t.cp.tage_wp.tbl_idx[k] & uint32_t(BPU_S2_TAGE_BANKS - 1)));
        }
      }
      if (t.cp.sc_wp.do_wr) {
        // s2_sc_busy[0] stays 0 because Bias is a tiny FF array (0 SRAM bank conflicts!)
        for (int g = 0; g < SC_GEHL_N; g++) {
          s2_sc_busy[1 + g] |= static_cast<uint8_t>(1u << (t.cp.sc_wp.gehl_idx[g] & uint32_t(BPU_SC_BANKS - 1)));
        }
        s2_sc_busy[5] |= static_cast<uint8_t>(1u << (t.cp.sc_wp.imli_idx & uint32_t(BPU_SC_BANKS - 1)));
        s2_sc_busy[6] |= static_cast<uint8_t>(1u << (t.cp.sc_wp.loc_idx & uint32_t(BPU_SC_BANKS - 1)));
      }
      for (int k = 0; k < ITT_MAX; k++) {
        if ((t.cp.itt_wp.wr_mask >> k) & 1u) {
          s2_itt_busy[k] |= static_cast<uint8_t>(1u << (t.cp.itt_wp.idx[k] & uint32_t(BPU_ITT_BANKS - 1)));
        }
      }
      if (t.cp.atage_wp.valid) {
        const uint32_t am = uint32_t((t.cp.ah_use2 ? BPU_AH2_BANKS : BPU_AH1_BANKS) - 1);
        uint8_t *ab = t.cp.ah_use2 ? ah2_tage_busy : ah1_tage_busy;
        for (int k = 0; k < TN; k++) {
          if ((t.cp.atage_wp.tbl_wr_mask >> k) & 1u) {
            ab[k] |= static_cast<uint8_t>(1u << (t.cp.atage_wp.tbl_idx[k] & am));
          }
        }
      }
      if (t.cp.aitt_wp.wr_mask != 0) {
        const uint32_t am = uint32_t((t.cp.ah_use2 ? BPU_AH2_BANKS : BPU_AH1_BANKS) - 1);
        uint8_t *ab = t.cp.ah_use2 ? ah2_itt_busy : ah1_itt_busy;
        for (int k = 0; k < ITT_MAX; k++) {
          if ((t.cp.aitt_wp.wr_mask >> k) & 1u) {
            ab[k] |= static_cast<uint8_t>(1u << (t.cp.aitt_wp.idx[k] & am));
          }
        }
      }
    }

    // ---- Step 2: S1 Exact-PC & S0 Ahead 1RW Reads (FTB Predict-Wins; TAGE/SC/ITT Write-Wins not-hit) ----
    uint32_t ftb1_rd_mask = 0;
    if (s1_blk.valid) {
      ftb1_rd_mask |= 1u << s2_bank_of_w(static_cast<uint32_t>(s1_blk.base >> SLOT_SHIFT));
      if (s1_blk.has_b) {
        ftb1_rd_mask |= 1u << s2_bank_of_w(static_cast<uint32_t>(s1_blk.base_b >> SLOT_SHIFT));
      }
      uint8_t tage_sup_a = 0, sc_sup_a = 0, itt_sup_a = 0;
      const uint32_t loc_bk_a = ftb_.sc_.loc_index(s1_blk.sc_ix) & uint32_t(BPU_SC_BANKS - 1);
      for (int k = 0; k < EX_TN; k++) {
        const uint32_t bk = s1_blk.ex_ix.idx[k] & uint32_t(BPU_S2_TAGE_BANKS - 1);
        if ((s2_tage_busy[k] >> bk) & 1u) tage_sup_a |= static_cast<uint8_t>(1u << k);
        else s2_tage_busy[k] |= static_cast<uint8_t>(1u << bk);
      }
      for (int g = 0; g < SC_GEHL_N; g++) {
        const uint32_t bk = s1_blk.sc_ix.gehl_idx[g] & uint32_t(BPU_SC_BANKS - 1);
        if ((s2_sc_busy[1 + g] >> bk) & 1u) sc_sup_a |= static_cast<uint8_t>(1u << (1 + g));
        else s2_sc_busy[1 + g] |= static_cast<uint8_t>(1u << bk);
      }
      {
        const uint32_t bk = s1_blk.sc_ix.imli_idx & uint32_t(BPU_SC_BANKS - 1);
        if ((s2_sc_busy[5] >> bk) & 1u) sc_sup_a |= 1u << 5;
        else s2_sc_busy[5] |= static_cast<uint8_t>(1u << bk);
      }
      if ((s2_sc_busy[6] >> loc_bk_a) & 1u) sc_sup_a |= 1u << 6;
      else s2_sc_busy[6] |= static_cast<uint8_t>(1u << loc_bk_a);
      for (int k = 0; k < ITT_MAX; k++) {
        const int tk = IttCfg::h[k];
        const uint32_t bk = s1_blk.ftb_ix.idx[tk] & uint32_t(BPU_ITT_BANKS - 1);
        if ((s2_itt_busy[k] >> bk) & 1u) itt_sup_a |= static_cast<uint8_t>(1u << k);
        else s2_itt_busy[k] |= static_cast<uint8_t>(1u << bk);
      }
      if (n.R[1].valid) {
        ftb_.s1_read(s1_blk.base, s1_blk.ftb_ix, s1_blk.ex_ix, s1_blk.sc_ix, n.ftb_raw, tage_sup_a, sc_sup_a, itt_sup_a);
      }

      if (s1_blk.has_b) {
        uint8_t tage_sup_b = 0, sc_sup_b = 0, itt_sup_b = 0;
        const uint32_t loc_bk_b = ftb_.sc_.loc_index(s1_blk.sc_ix_b) & uint32_t(BPU_SC_BANKS - 1);
        for (int k = 0; k < EX_TN; k++) {
          const uint32_t bk = s1_blk.ex_ix_b.idx[k] & uint32_t(BPU_S2_TAGE_BANKS - 1);
          if ((s2_tage_busy[k] >> bk) & 1u) tage_sup_b |= static_cast<uint8_t>(1u << k);
          else s2_tage_busy[k] |= static_cast<uint8_t>(1u << bk);
        }
        for (int g = 0; g < SC_GEHL_N; g++) {
          const uint32_t bk = s1_blk.sc_ix_b.gehl_idx[g] & uint32_t(BPU_SC_BANKS - 1);
          if ((s2_sc_busy[1 + g] >> bk) & 1u) sc_sup_b |= static_cast<uint8_t>(1u << (1 + g));
          else s2_sc_busy[1 + g] |= static_cast<uint8_t>(1u << bk);
        }
        {
          const uint32_t bk = s1_blk.sc_ix_b.imli_idx & uint32_t(BPU_SC_BANKS - 1);
          if ((s2_sc_busy[5] >> bk) & 1u) sc_sup_b |= 1u << 5;
          else s2_sc_busy[5] |= static_cast<uint8_t>(1u << bk);
        }
        if ((s2_sc_busy[6] >> loc_bk_b) & 1u) sc_sup_b |= 1u << 6;
        else s2_sc_busy[6] |= static_cast<uint8_t>(1u << loc_bk_b);
        for (int k = 0; k < ITT_MAX; k++) {
          const int tk = IttCfg::h[k];
          const uint32_t bk = s1_blk.ftb_ix_b.idx[tk] & uint32_t(BPU_ITT_BANKS - 1);
          if ((s2_itt_busy[k] >> bk) & 1u) itt_sup_b |= static_cast<uint8_t>(1u << k);
          else s2_itt_busy[k] |= static_cast<uint8_t>(1u << bk);
        }
        if (n.R[1].valid) {
          ftb_.s1_read(s1_blk.base_b, s1_blk.ftb_ix_b, s1_blk.ex_ix_b, s1_blk.sc_ix_b, n.ftb_raw_b, tage_sup_b, sc_sup_b, itt_sup_b);
        }
      }
    }

    uint8_t ah1_ftb_rd_m = 0, ah2_ftb_rd_m = 0;
    if (s0_launched) {
      const uint32_t aw = static_cast<uint32_t>(s0_ah_base >> SLOT_SHIFT);
      ah1_ftb_rd_m = static_cast<uint8_t>(1u << ah1_bank_of_w(aw));
      ah2_ftb_rd_m = static_cast<uint8_t>(1u << ah2_bank_of_w(aw));
      const FtbBlkIdx ah_ix = BpuFtb::make_ahead_index(s0_ah_base, 0, s0_ah_fh);
      uint8_t ah1_t_sup = 0, ah2_t_sup = 0, ah1_i_sup = 0, ah2_i_sup = 0;
      for (int k = 0; k < TN; k++) {
        const uint32_t bk1 = ah_ix.idx[k] & uint32_t(BPU_AH1_BANKS - 1);
        const uint32_t bk2 = ah_ix.idx[k] & uint32_t(BPU_AH2_BANKS - 1);
        if ((ah1_tage_busy[k] >> bk1) & 1u) ah1_t_sup |= static_cast<uint8_t>(1u << k);
        else ah1_tage_busy[k] |= static_cast<uint8_t>(1u << bk1);
        if ((ah2_tage_busy[k] >> bk2) & 1u) ah2_t_sup |= static_cast<uint8_t>(1u << k);
        else ah2_tage_busy[k] |= static_cast<uint8_t>(1u << bk2);
      }
      for (int k = 0; k < ITT_MAX; k++) {
        const int tk = IttCfg::h[k];
        const uint32_t bk1 = ah_ix.idx[tk] & uint32_t(BPU_AH1_BANKS - 1);
        const uint32_t bk2 = ah_ix.idx[tk] & uint32_t(BPU_AH2_BANKS - 1);
        if ((ah1_itt_busy[k] >> bk1) & 1u) ah1_i_sup |= static_cast<uint8_t>(1u << k);
        else ah1_itt_busy[k] |= static_cast<uint8_t>(1u << bk1);
        if ((ah2_itt_busy[k] >> bk2) & 1u) ah2_i_sup |= static_cast<uint8_t>(1u << k);
        else ah2_itt_busy[k] |= static_cast<uint8_t>(1u << bk2);
      }
      ftb_.ahead_read(s0_ah_base, s0_ah_fh, n.ah_pipe.raw, ah1_t_sup, ah1_i_sup);
      ftb_.ahead2_read(s0_ah_base, s0_ah_fh, n.ah2_pipe, ah2_t_sup, ah2_i_sup);
    }

    for (int k = 0; k < EX_TN; k++) t.s2_tbl_busy[k] = s2_tage_busy[k];
    for (int k = 0; k < TN; k++) { t.ah1_tbl_busy[k] = ah1_tage_busy[k]; t.ah2_tbl_busy[k] = ah2_tage_busy[k]; }

    // ---- Step 3: Predict-Always-Wins FTB 1-Entry Write Buffers + s2_base + Predecode Clear ----
    const uint32_t s2_base_rd_mask = ftb1_rd_mask;
    if (t.cp.valid && t.cp.tage_wp.base_wr) {
      const uint32_t bbk = t.cp.tage_wp.base_idx & uint32_t(BPU_FTB1_BANKS - 1);
      t.do_s2_base = (((s2_base_rd_mask >> bbk) & 1u) == 0);
    }

    auto arbitrate_ftb_wb = [](const FtbWrBuf &old_wb, bool new_v, uint32_t new_s, uint8_t new_w, const FtbEnt &new_e,
                               uint32_t &rd_mask, uint32_t bank_mask,
                               bool &do_wr, FtbWrBuf &wr_out, FtbWrBuf &next_wb) {
      FtbWrBuf nw{};
      nw.valid = new_v; nw.set = new_s; nw.way = new_w; nw.ent = new_e;
      if (old_wb.valid) {
        const uint32_t obk = old_wb.set & bank_mask;
        if (((rd_mask >> obk) & 1u) == 0) {
          do_wr = true;
          wr_out = old_wb;
          rd_mask |= (1u << obk);
          if (new_v) next_wb = nw;
        } else {
          next_wb = new_v ? nw : old_wb;
        }
      } else if (new_v) {
        const uint32_t nbk = new_s & bank_mask;
        if (((rd_mask >> nbk) & 1u) == 0) {
          do_wr = true;
          wr_out = nw;
          rd_mask |= (1u << nbk);
        } else {
          next_wb = nw;
        }
      }
    };

    arbitrate_ftb_wb(n.ftb1_wb, t.cp.valid && t.cp.ftb1_wr, t.cp.ftb1_s, t.cp.ftb1_way, t.cp.ftb1_ent,
                     ftb1_rd_mask, uint32_t(BPU_FTB1_BANKS - 1),
                     t.do_ftb1_wr, t.ftb1_wr, t.next_ftb1_wb);

    uint32_t ah1_m = ah1_ftb_rd_m;
    arbitrate_ftb_wb(n.aftb_wb, t.cp.valid && t.cp.ah_wr && !t.cp.ah_use2, t.cp.ah_s, t.cp.ah_way, t.cp.ah_ent,
                     ah1_m, uint32_t(BPU_AH1_BANKS - 1),
                     t.do_aftb_wr, t.aftb_wr, t.next_aftb_wb);

    uint32_t ah2_m = ah2_ftb_rd_m;
    arbitrate_ftb_wb(n.aftb2_wb, t.cp.valid && t.cp.ah_wr && t.cp.ah_use2, t.cp.ah_s, t.cp.ah_way, t.cp.ah_ent,
                     ah2_m, uint32_t(BPU_AH2_BANKS - 1),
                     t.do_aftb2_wr, t.aftb2_wr, t.next_aftb2_wb);

    // Predecode false-branch clear on free FTB1 bank
    const bool pd_clr_en = in.pd.valid && in.pd.nb_valid && pd_have;
    const pc_t pd_clr_base = in.pd.nb_base ? in.pd.nb_base : pd_rec.base;
    const pc_t pd_clr_pc = in.pd.nb_pc;
    bool clr_consumed = false;
    if (n.clr_pend_v || pd_clr_en) {
      const pc_t cbase = n.clr_pend_v ? n.clr_pend_base : pd_clr_base;
      const pc_t cpc = n.clr_pend_v ? n.clr_pend_pc : pd_clr_pc;
      const uint32_t cbk = s2_bank_of_w(static_cast<uint32_t>(cbase >> SLOT_SHIFT));
      if (((ftb1_rd_mask >> cbk) & 1u) == 0) {
        t.do_clr = true;
        t.clr_base = cbase;
        t.clr_pc = cpc;
        clr_consumed = true;
      }
    }
    if (pd_clr_en && !(clr_consumed && !n.clr_pend_v)) {
      t.next_clr_pend_v = true;
      t.next_clr_pend_base = pd_clr_base;
      t.next_clr_pend_pc = pd_clr_pc;
    } else if (n.clr_pend_v && !clr_consumed) {
      t.next_clr_pend_v = true;
      t.next_clr_pend_base = n.clr_pend_base;
      t.next_clr_pend_pc = n.clr_pend_pc;
    }

    // ---- Step 4: ROB Commit Enqueue into 16-entry brq ----
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const pc_t pc = in.in_update_base_pc[i];
      const br_type_t ty = in.in_actual_br_type[i];
      const bool taken = (ty != BR_NONCTL) && in.in_actual_dir[i];
      const bool rvc = in.in_update_is_rvc[i];
      push_arch(n.arch, pc, taken, in.in_actual_targets[i], ty);
      upd_imli(n.arch_imli, ty, taken, pc, in.in_actual_targets[i], false);
      if (br_call(ty)) n.aras.push(pc + ilen_of(rvc));
      else if (ty == BR_RET) n.aras.pop();
      if (ty != BR_NONCTL && ft.brq_n < FTB_BRQ_N) {
        FtbBrqEnt &e = ft.brq[(ft.brq_h + ft.brq_n) % FTB_BRQ_N];
        e.blk_id = in.in_blk_id[i];
        e.pc = pc; e.is_rvc = rvc; e.type = ty;
        e.actual_dir = taken; e.pred_dir = in.in_pred_dir[i];
        e.alt_pred = in.in_alt_pred[i]; e.pcpn = in.in_pcpn[i];
        e.target = in.in_actual_targets[i];
        ft.brq_n++;
      }
    }
    out.upd_free = 15;
    out.tq_free = 15;
  }

  void commit(const Txn &t) {
    if (t.rec_v) rec_[t.rec.id & 255] = t.rec;
    if (t.rec_b_v) rec_[t.rec_b.id & 255] = t.rec_b;
    if (t.ftb_rec_v) ftb_rec_[t.ftb_rec.id & (FTB_REC_N - 1)] = t.ftb_rec;
    if (t.ftb_rec_b_v) ftb_rec_[t.ftb_rec_b.id & (FTB_REC_N - 1)] = t.ftb_rec_b;

    // 1-cycle pure writes (zero T0 reads!)
    if (t.cp.valid) ftb_.apply_commit_non_ftb(t.cp, t.do_s2_base);
    if (t.do_ftb1_wr) ftb_.apply_ftb1_write(t.ftb1_wr.set, t.ftb1_wr.way, t.ftb1_wr.ent);
    if (t.do_aftb_wr) ftb_.apply_aftb_write(false, t.aftb_wr.set, t.aftb_wr.way, t.aftb_wr.ent);
    if (t.do_aftb2_wr) ftb_.apply_aftb_write(true, t.aftb2_wr.set, t.aftb2_wr.way, t.aftb2_wr.ent);
    if (t.do_clr) ftb_.clear_false_branch(t.clr_base, t.clr_pc);

    // Deferred 1RW u-bit periodic reset on idle banks
    ftb_.s2_tage_.try_rst(t.s2_tbl_busy);
    ftb_.atage_.try_rst(t.ah1_tbl_busy);
    ftb_.atage2_.try_rst(t.ah2_tbl_busy);

    // Advance 1-entry pending FTB write buffers & clr_pend
    st_.ftb1_wb = t.next_ftb1_wb;
    st_.aftb_wb = t.next_aftb_wb;
    st_.aftb2_wb = t.next_aftb2_wb;
    st_.clr_pend_v = t.next_clr_pend_v;
    st_.clr_pend_base = t.next_clr_pend_base;
    st_.clr_pend_pc = t.next_clr_pend_pc;
  }
};

}  // namespace bpuf

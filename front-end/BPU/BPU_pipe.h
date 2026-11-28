#pragma once
// Clean Block-based 1-Ahead A FTB Branch Predictor (Clean BPU v11: Plan A).
// Matches rtl/frontend/bpu/ cycle-for-cycle (docs/modules/bpu.md).
//
//   S0 (0 bubble)   Registered 1-Ahead A SRAM select (aftb 2048x8 + atage 1024x4x4 + aitt 256x3) with tiny FF uFTB (16x4)
//                   cold-start fallback; launches next-block 1-Ahead A 1RW SRAM read from (s0_base, s0_ex_fh); pushes spec hist/RAS.
//                   No same-cycle S0 issue on refetch (2A).
//   S1 (1 bubble)   Exact-PC single-block 1RW SRAM read (Main FTB 4096x4 2B + 6-table ExBlockTage 4096x4x6 8B + ITT 256x3 2B).
//   S2 (2 bubbles)  Final evaluation & override; writes Unified FTQ-64, single-bank rec_[256] & ftb_rec_[512].
//                   On S2 override (2B): registers ovr_pend for 1-cycle decoupled spec/RAS restore + Zero-FF Ahead 1RW SRAM re-read!
//   Train           1-branch/cycle commit queue (native 1.00 br/cyc); Zero-T0-read pure writes via ftb_rec Meta + WRBypass + PLRU.
#include "BPU.h"
#include "BpuFtb.h"
#include "fpr.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

struct FrontTopIn;
struct FrontReadData;
class SimContext;
class PreIduQueue;

namespace bpuf {

constexpr int MAIN_LAT = 2;     // default S0 -> S1 (1RW SRAM read) -> S2 (final eval/override)
constexpr int MAIN_LAT_MAX = 3; // FE_BPU_S3=1: S0 -> S1 (SRAM read) -> S2 -> S3 (final eval/override)
inline int pipe_lat() { return cfg_fe_bpu_s3() ? 3 : 2; }
inline int cfg_pair() { return 1; }
constexpr int PQ = 64;          // PTAB checkpoint records
constexpr int HR_BITS = 256;    // circular global-history ring (covers max history EX_GHR_LEN[5] = 178)
constexpr uint32_t HR_MASK = HR_BITS - 1u;

struct HistRing {
  uint64_t bits[HR_BITS / 64] = {};
  uint32_t head = 0;
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  bool back(uint32_t k) const { const uint32_t p = (head - 1 - k) & HR_MASK; return (bits[p >> 6] >> (p & 63)) & 1; }
  void push(bool dir) {
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
  uint16_t ex_fh[EX_FH_K][EX_TN] = {};
  uint8_t ras_tosr = 0, ras_tosw = 0, ras_cnt = 0;
  target_addr_t ras_top = 0;
  bool ah_v = false;
  pc_t ah_key_base = 0;
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

struct FtbAhEval {
  bool valid = false;
  bool ah_v = false, ah_hit = false;
  pc_t eval_base = 0;
  pc_t ah_key_base = 0;
  uint16_t ah_key_ex_fh[EX_FH_K][EX_TN] = {};
  S0Guess g{};
  BpuFtb::AhMeta ah_meta{};
  bool slot_dir[FW] = {};
  PredMeta pred_meta{};
  pc_t ret_bpc = 0;
};

struct TrainPipeEnt {
  bool valid = false;
  FtbBlkRec rc{};
  FtbTrkBr br{};
  FtbBrqEnt q{};
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
  bool valid = false, sq = false;
  bool ah_v = false, ah_hit = false;
  pc_t ah_key_base = 0;
  pc_t prev_base = 0;
  uint16_t ah_key_ex_fh[EX_FH_K][EX_TN] = {};
  pc_t base = 0, bnd = 0;
  S0Guess g;
  Ckpt ck;
  FtbBlkIdx ftb_ix;
  ExTageIdx ex_ix;
  BpuFtb::AhDirIttOut ah_dir_itt{};
  BpuFtb::AhMeta ah_meta{};
};

struct RecP { Ckpt ck; pc_t base = 0; uint16_t id = 0; uint8_t epoch = 0; bool valid = false; };

struct Decision {
  bool valid = false, ovr = false, s2_ah_self0 = false;
  uint8_t squash_n = 0;
  const Blk *blk = nullptr;
  MainOut out;
  BpuFtb::S2Meta s2_meta{};
  BpuFtb::AhMeta s2_ah_meta{};
};

// MODULE front.core.bpu_pipe
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
  const ::PreIduQueue *ftq = nullptr;
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
    pc_t prev_base = 0;
    Ckpt pq[PQ];
    uint8_t pq_h = 0, pq_n = 0;
    uint16_t blk_seq = 0;
    uint8_t epoch = 0;
    Blk R[MAIN_LAT_MAX];
    FtbRaw ftb_raw;
    FtbRaw ftb_raw_s3;
    FtbAhPipe ah_pipe;
    FtbAhPipe ah_a1_raw;
    FtbAhEval ah_a1;
    FtbTrkState ftb_trk;
    HistRing spec, arch;
    RasQ sras;
    RasC aras;
    // 1-entry pending FTB write buffers (Predict ALWAYS wins FTB 1RW banks!)
    FtbWrBuf ftb1_wb{};
    FtbWrBuf aftb_wb{};
    // 2-cycle 1RW predecode false-branch clear FSM for u_ftb1 (matches bpu_ftb_tab.sv g_clr_sram)
    bool clr_pend_v = false;
    pc_t clr_pend_base = 0, clr_pend_pc = 0;
    bool clr_t1_v = false;
    pc_t clr_t1_base = 0, clr_t1_pc = 0;
    FtbEnt clr_t1_set[FTB_S1_W]{};
    // 1-cycle registered S2 override history/RAS + Ahead 1RW SRAM recovery (2B)
    bool ovr_pend = false;
    Ckpt ovr_ck{};
    MainOut ovr_out{};
    pc_t ovr_base = 0;
    // Configurable extra pipeline stages for commit training (FE_TRAIN_LAT = 1 or 2)
    TrainPipeEnt train_pipe[2]{};
  };

  struct Txn {
    bool rec_v = false;
    RecP rec;
    bool pmu_l1_ovr = false;
    bool pmu_ptab_stall = false;
    bool ftb_rec_v = false;
    FtbBlkRec ftb_rec{};
    BpuFtb::CommitPlan cp{};
    bool do_s2_base = false;
    bool do_ftb1_wr = false;
    FtbWrBuf ftb1_wr{};
    FtbWrBuf next_ftb1_wb{};
    bool do_aftb_wr = false;
    FtbWrBuf aftb_wr{};
    FtbWrBuf next_aftb_wb{};
    bool do_clr_ftb0 = false;
    pc_t clr_ftb0_base = 0, clr_ftb0_pc = 0;
    bool do_fast_ftb0 = false;
    pc_t fast_ftb0_base = 0;
    FtbEnt fast_ftb0_ent{};
    bool do_clr_aftb = false;
    pc_t clr_aftb_key = 0, clr_aftb_base = 0, clr_aftb_pc = 0;
    bool do_clr_wr = false;
    uint32_t clr_wr_set = 0;
    uint16_t clr_wr_tg = 0;
    uint8_t clr_wr_way = 0;
    FtbEnt clr_wr_ent{};
    bool do_clr_rd = false;
    uint32_t clr_rd_set = 0;
    pc_t clr_rd_base = 0, clr_rd_pc = 0;
    bool next_clr_t1_v = false;
    bool next_clr_pend_v = false;
    pc_t next_clr_pend_base = 0, next_clr_pend_pc = 0;
    TrainPipeEnt next_train_pipe[2]{};
    uint8_t s2_tbl_busy[EX_TN] = {};
    uint8_t ah1_tbl_busy[TN] = {};
    uint8_t ah1_ex_tbl_busy[EX_TN] = {};
    void clear() {
      rec_v = false;
      pmu_l1_ovr = false;
      pmu_ptab_stall = false;
      ftb_rec_v = false;
      cp = BpuFtb::CommitPlan{};
      do_s2_base = false;
      do_ftb1_wr = do_aftb_wr = false;
      next_ftb1_wb = FtbWrBuf{};
      next_aftb_wb = FtbWrBuf{};
      do_clr_ftb0 = do_fast_ftb0 = do_clr_aftb = do_clr_wr = do_clr_rd = false;
      next_clr_t1_v = false;
      next_clr_pend_v = false;
      next_train_pipe[0] = TrainPipeEnt{};
      next_train_pipe[1] = TrainPipeEnt{};
      std::memset(s2_tbl_busy, 0, sizeof s2_tbl_busy);
      std::memset(ah1_tbl_busy, 0, sizeof ah1_tbl_busy);
      std::memset(ah1_ex_tbl_busy, 0, sizeof ah1_ex_tbl_busy);
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

  // Same-Page Line-Straddle 2-Fetch: full 32B block (16 halfwords), truncating only at 4KB page boundary.
  static pc_t block_bnd(pc_t base) {
    const pc_t mask = ~static_cast<pc_t>(4096u - 1u);
    const pc_t plus = base + FW * SLOT_BYTES;
    return (base & mask) != (plus & mask) ? (plus & mask) : plus;
  }

  static Ckpt take_ckpt(const HistRing &h, const RasQ &r) {
    Ckpt c;
    c.head = h.head;
    std::memcpy(c.ex_fh, h.ex_fh, sizeof(c.ex_fh));
    c.ras_tosr = r.tosr; c.ras_tosw = r.tosw; c.ras_cnt = r.cnt; c.ras_top = r.top();
    return c;
  }

  static void restore(const Ckpt &c, HistRing &h, RasQ &r) {
    h.head = c.head;
    std::memcpy(h.ex_fh, c.ex_fh, sizeof(h.ex_fh));
    r.tosr = c.ras_tosr; r.tosw = c.ras_tosw; r.cnt = c.ras_cnt;
  }

  static void push_bits(uint8_t nbits, uint8_t bits, uint8_t rop, target_addr_t rval, HistRing &h, RasQ &r) {
    for (int i = 0; i < nbits; i++) h.push((bits >> i) & 1);
    if (rop == 1) r.push(rval); else if (rop == 2) r.pop();
  }

  static void push_arch(HistRing &a, pc_t pc, bool taken, pc_t tgt, uint8_t ty) {
    if (ty == BR_NONCTL || !taken) return;
    const uint8_t b = hist_sym(pc, tgt);
    for (int i = 0; i < hsym_; i++) a.push((b >> i) & 1);
  }

  static bool disagree(const MainOut &o, const S0Guess &g) {
    return o.next != g.next || o.taken != g.taken || o.ras_op != g.ras_op ||
           (o.ras_op == 1 && o.ras_val != g.ras_val) || o.nbits != g.nbits || (o.nbits && o.bits != g.bits);
  }

  static void hp(const char *tag, pc_t a, pc_t b, uint8_t nb, uint8_t bits) {
    if (fpr::on() && fpr::in_dump())
      std::fprintf(stderr, "HP c=%llu %s %08llx->%08llx n=%u b=%x\n",
                   (unsigned long long)fpr::cycle(), tag, (unsigned long long)a, (unsigned long long)b, nb, bits);
  }

  bool repair_blk(const InputPayload &in, State &n) const {
    const RecP &rc = rec_[in.rf_blk & 255];
    if (!rc.valid || rc.id != in.rf_blk || rc.epoch != n.epoch) return false;
    const bool carried = (in.rf_pc + SLOT_BYTES == rc.base);
    if (!carried) {
      if (in.rf_pc < rc.base) return false;
      const pc_t diff = in.rf_pc - rc.base;
      if ((diff & (SLOT_BYTES - 1)) || (diff >> SLOT_SHIFT) >= FW) return false;
    }
    restore(rc.ck, n.spec, n.sras);
    const bool tk = (in.rf_type == BR_DIRECT) ? (in.refetch_address != in.rf_pc + ilen_of(in.rf_is_rvc)) : true;
    const uint8_t nb = tk ? hsym_ : 0, bs = tk ? hist_sym(in.rf_pc, in.refetch_address) : 0;
    const uint8_t rop = br_call(in.rf_type) ? 1 : (in.rf_type == BR_RET ? 2 : 0);
    hp("REPAIR", in.rf_pc, in.refetch_address, nb, bs);
    push_bits(nb, bs, rop, in.rf_pc + ilen_of(in.rf_is_rvc), n.spec, n.sras);
    return true;
  }

  static void ftb_emit_warm(FtbTrkState &ft, BpuFtb &ftb) {
    if (!ft.v || ft.nbr == 0) return;
    FtbTrainEv ev{};
    ev.start = ft.start;
    std::memcpy(ev.ex_fh, ft.ex_fh, sizeof ev.ex_fh);
    ev.nbr = ft.nbr;
    for (int i = 0; i < ft.nbr; i++) ev.br[i] = ft.br[i];
    ftb.apply_train_ev(ev);
  }

  static void fill_ftb_rec(FtbBlkRec &dst, uint16_t id, pc_t base, const Ckpt &ck,
                           bool ah_v, pc_t ah_key_base, const uint16_t ah_key_ex_fh[EX_FH_K][EX_TN],
                           const BpuFtb::S2Meta &s2m, const BpuFtb::AhMeta &ahm,
                           bool ah_self0 = false, pc_t prev_base = 0) {
    dst.valid = true;
    dst.id = id;
    dst.base = base;
    std::memcpy(dst.ex_fh, ck.ex_fh, sizeof dst.ex_fh);
    dst.ah_v = ah_v;
    dst.ah_self0 = ah_self0;
    dst.ah_key_base = ah_key_base;
    dst.prev_base = prev_base;
    std::memcpy(dst.ah_key_ex_fh, ah_key_ex_fh, sizeof dst.ah_key_ex_fh);
    dst.ftb1_meta = s2m.ftb;
    dst.s2_tage_meta = s2m.tage;
    dst.s2_itt_meta = s2m.itt;
    dst.ah_ftb_meta = ahm.ftb;
    dst.ah_tage_meta = ahm.tage;
    dst.ah_ex_tage_meta = ahm.ex_tage;
    dst.ah_itt_meta = ahm.itt;
  }

 public:
  void warm_commit(pc_t pc, uint8_t ty, bool dir, pc_t target, bool rvc) {
    State &n = st_;
    FtbTrkState &ft = n.ftb_trk;
    auto sync_ft_hist = [&] {
      std::memcpy(ft.ex_fh, n.arch.ex_fh, sizeof ft.ex_fh);
    };
    if (!ft.v || pc != ft.expect) {
      ftb_emit_warm(ft, ftb_);
      ft.v = true; ft.start = pc; ft.expect = pc; ft.nbr = 0;
      sync_ft_hist();
    }
    pc_t bnd = block_bnd(ft.start);
    if (pc >= bnd) {
      ftb_emit_warm(ft, ftb_);
      ft.start = pc; ft.expect = pc; ft.nbr = 0;
      sync_ft_hist();
      bnd = block_bnd(ft.start);
    }
    const uint8_t off = static_cast<uint8_t>((pc - ft.start) >> SLOT_SHIFT);
    bool pred = false, alt = false;
    uint8_t pcpn = TN;
    if (ty == BR_DIRECT) {
      FtbEnt ent;
      if (ftb_.ftb1_.lookup(ft.start, ent)) {
        for (int k = 0; k < FTB_SLOTS; k++) {
          if (ent.s[k].valid && ent.s[k].offset == off) {
            const ExTageIdx ex_ix = make_ex_tage_index(static_cast<uint32_t>(ft.start >> SLOT_SHIFT), ft.ex_fh);
            ExBlockTage::Pred tp;
            ftb_.s2_tage_.predict_slot(ex_ix, k, tp);
            pred = tp.pred; alt = tp.alt;
            pcpn = (tp.pcpn < EX_TN) ? static_cast<uint8_t>(std::min<int>(tp.pcpn, TN - 1)) : static_cast<uint8_t>(TN);
            break;
          }
        }
      }
    }
    const bool taken = (ty != BR_NONCTL) && dir;
    if (ty != BR_NONCTL && ft.nbr < FW) {
      FtbTrkBr &b = ft.br[ft.nbr++];
      b.offset = off; b.carried = false; b.is_rvc = rvc; b.type = ty;
      b.actual_dir = taken; b.pred_dir = pred; b.alt_pred = alt; b.pcpn = pcpn; b.target = target;
    }
    push_arch(n.arch, pc, taken, target, ty);
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
  void warm_finish() { (*stp_).spec = (*stp_).arch; (*stp_).sras = RasQ::from_commit((*stp_).aras); }

  BpuPipe() { reset(); }
  void comb_cycle(const InputPayload &in, const Decision &d, OutputPayload &out) { comb(in, d, out, txn_d_); txn_d_valid_ = true; }
  void seq() { if (txn_d_valid_) commit(txn_d_); txn_d_valid_ = false; }

  void dump_hist(uint32_t c) const {
    auto fhash = [](const HistRing &h) { uint64_t x = 1469598103934665603ull; for (int k = 0; k < EX_FH_K; k++) for (int i = 0; i < EX_TN; i++) x = (x ^ h.ex_fh[k][i]) * 1099511628211ull; return x; };
    auto recent = [](const HistRing &h) { uint64_t x = 0; for (int k = 0; k < 64; k++) x |= uint64_t(h.back(k)) << k; return x; };
    std::fprintf(stderr, "H c=%u sr=%016llx sf=%016llx ar=%016llx af=%016llx\n", c, (unsigned long long)recent(st_.spec), (unsigned long long)fhash(st_.spec), (unsigned long long)recent(st_.arch),
                 (unsigned long long)fhash(st_.arch));
  }

  static constexpr int depth() { return MAIN_LAT; }
  uint8_t hold_n() const {
    if (cfg_fe_single_pred()) return 0;
    const int lat = pipe_lat();
    return static_cast<uint8_t>((((*stp_).R[0].valid && (*stp_).R[0].sq) ? 1 : 0) +
                                ((lat == 3 && (*stp_).R[1].valid && (*stp_).R[1].sq) ? 1 : 0));
  }

  void reset() {
    for (auto &r : rec_) r.valid = false;
    for (auto &r : ftb_rec_) r.valid = false;
    ftb_.reset();
    st_ = State{};
  }

  void decide(bool refetch, Decision &d) const {
    d.valid = d.ovr = d.s2_ah_self0 = false;
    d.squash_n = 0;
    if (cfg_fe_single_pred()) return;
    const int lat = pipe_lat();
    const Blk &b = (*stp_).R[lat - 1];
    if (refetch || !b.valid) return;
    d.valid = true;
    d.blk = &b;
    const FtbRaw &raw = (lat == 3) ? (*stp_).ftb_raw_s3 : (*stp_).ftb_raw;
    if (cfg_fe_pred_arch() == 2) {
      ftb_.s2_select_uni(b.base, b.bnd, b.ex_ix, raw, b.ah_v, b.ah_dir_itt, b.ah_meta, b.ck.ex_fh,
                         b.ck.ras_cnt, b.ck.ras_top, d.out, &d.s2_meta, &d.s2_ah_meta, &d.s2_ah_self0);
    } else {
      ftb_.s2_select(b.base, b.bnd, b.ftb_ix, b.ex_ix, raw, b.ck.ras_cnt, b.ck.ras_top, d.out, &d.s2_meta);
    }
    d.ovr = disagree(d.out, b.g);
    if (d.ovr) {
      d.squash_n = static_cast<uint8_t>(((*stp_).R[0].valid ? 1 : 0) +
                                        ((lat == 3 && (*stp_).R[1].valid) ? 1 : 0));
    }
  }

  void comb(const InputPayload &in, const Decision &d, OutputPayload &out, Txn &t) {
    out = {};
    t.clear();
    State &n = st_;
    const uint16_t seq0 = n.blk_seq;
    const uint8_t epoch0 = n.epoch;
    const int ah_mode = cfg_fe_ah_mode();
    const bool single = cfg_fe_single_pred();
    const int pred_arch = cfg_fe_pred_arch();
    const bool fast_ovr = cfg_fe_fast_ovr();
    const int sup_tage = cfg_fe_s2_sup_tage();
    const int lat = pipe_lat();
    const int itt_n = cfg_fe_itt_cfg() ? ITT_MAX_EXT : ITT_MAX;
    const bool ah_ex = cfg_fe_ah_ex_tage() || (pred_arch == 2);

    Ckpt pd_rec; bool pd_have = false;
    if (in.pd.valid && n.pq_n > 0) {
      pd_rec = n.pq[n.pq_h]; pd_have = true;
      n.pq_h = uint8_t((n.pq_h + 1) % PQ); n.pq_n--;
    }

    bool s0_launched = false;
    pc_t s0_ah_base = 0;
    uint16_t s0_ah_ex_fh[EX_FH_K][EX_TN] = {};
    const FtbAhPipe old_ah_a0 = n.ah_pipe;
    const FtbAhPipe old_ah_a1_raw = n.ah_a1_raw;
    const FtbAhEval old_ah_a1 = n.ah_a1;

    auto s0_issue = [&](pc_t pc) {
      Blk &b = n.R[0];
      b = Blk{};
      b.base = pc;
      b.bnd = block_bnd(b.base);
      b.prev_base = n.prev_base;
      bool slot_dir[FW] = {};
      PredMeta pred_meta{};
      if (pred_arch == 1) {
        b.g = ftb_.s0_lookup<S0Guess>(b.base, b.bnd, n.sras.cnt, n.sras.top());
      } else if (pred_arch == 2) {
        const FtbAhPipe &ah_src = (ah_mode == 0) ? old_ah_a0 : old_ah_a1_raw;
        b.ah_v = ah_src.valid;
        b.ah_key_base = ah_src.key_base;
        std::memcpy(b.ah_key_ex_fh, ah_src.key_ex_fh, sizeof b.ah_key_ex_fh);
        if (ah_src.valid) {
          ftb_.ahead_dir_itt_select(b.base, n.spec.ex_fh, ah_src, b.ah_dir_itt, &b.ah_meta, b.prev_base, false);
        }
        b.g = ftb_.s0_lookup_with_ah<S0Guess>(b.base, b.bnd, b.ah_dir_itt, n.sras.cnt, n.sras.top());
        b.ah_hit = b.g.hit && b.ah_v;
      } else if (ah_mode == 0) {
        const FtbAhPipe &ah_src = old_ah_a0;
        b.ah_v = ah_src.valid;
        b.ah_key_base = ah_src.key_base;
        std::memcpy(b.ah_key_ex_fh, ah_src.key_ex_fh, sizeof b.ah_key_ex_fh);
        b.ah_hit = ftb_.ahead_select(b.base, b.bnd, ah_src, n.sras.cnt, n.sras.top(), b.g, &b.ah_meta,
                                     single ? slot_dir : nullptr, single ? &pred_meta : nullptr);
        if (!b.ah_hit) {
          b.g = ftb_.s0_lookup<S0Guess>(b.base, b.bnd, n.sras.cnt, n.sras.top(),
                                        single ? slot_dir : nullptr, single ? &pred_meta : nullptr);
        }
      } else {
        if (old_ah_a1.valid && old_ah_a1.eval_base == b.base) {
          b.ah_v = old_ah_a1.ah_v;
          b.ah_key_base = old_ah_a1.ah_key_base;
          std::memcpy(b.ah_key_ex_fh, old_ah_a1.ah_key_ex_fh, sizeof b.ah_key_ex_fh);
          b.ah_meta = old_ah_a1.ah_meta;
          b.ah_hit = old_ah_a1.ah_hit;
          if (b.ah_hit) {
            b.g = old_ah_a1.g;
            if (single) {
              std::memcpy(slot_dir, old_ah_a1.slot_dir, sizeof slot_dir);
              pred_meta = old_ah_a1.pred_meta;
            }
            if (b.g.taken && b.g.type == BR_RET && n.sras.cnt > 0) {
              b.g.next = n.sras.top();
              b.g.bits = hist_sym(old_ah_a1.ret_bpc, b.g.next);
              if (single && b.g.slot < FW) {
                const uint8_t ualt = (pred_meta.ps_info[b.g.slot] >> 6) & 1u;
                pred_meta.ps_info[b.g.slot] = ps_pack(BR_RET, 7, ualt != 0);
              }
            }
          }
        }
        if (!b.ah_hit) {
          b.g = ftb_.s0_lookup<S0Guess>(b.base, b.bnd, n.sras.cnt, n.sras.top(),
                                        single ? slot_dir : nullptr, single ? &pred_meta : nullptr);
        }
      }

      b.ck = take_ckpt(n.spec, n.sras);
      b.ck.base = b.base;
      b.ck.ah_v = b.ah_v;
      b.ck.ah_key_base = b.ah_key_base;
      if (!single) {
        b.valid = true;
        b.sq = n.R[1].valid || (lat == 3 && n.R[2].valid);
        const uint32_t wA = static_cast<uint32_t>(b.base >> SLOT_SHIFT);
        b.ftb_ix = BpuFtb::make_index(b.base, n.spec.ex_fh);
        b.ex_ix = make_ex_tage_index(wA, n.spec.ex_fh);
      }
      hp("S0", b.base, b.g.next, b.g.nbits, b.g.bits);
      push_bits(b.g.nbits, b.g.bits, b.g.ras_op, b.g.ras_val, n.spec, n.sras);

      if (single) {
        if (n.pq_n < PQ) { n.pq[(n.pq_h + n.pq_n) % PQ] = b.ck; n.pq_n++; }
        out.PTAB_write_enable = true;
        out.out_pred_base_pc = b.base;
        for (int i = 0; i < FW; i++) out.out_pred_dir[i] = slot_dir[i];
        out.meta = pred_meta;
        for (int i = 0; i < FW; i++) out.meta.blk_id[i] = seq0;
        out.predict_next_fetch_address = b.g.next;
        t.rec_v = true;
        t.rec.ck = b.ck; t.rec.base = b.base; t.rec.id = seq0; t.rec.epoch = epoch0; t.rec.valid = true;
        t.ftb_rec_v = true;
        BpuFtb::S2Meta empty_s2{};
        fill_ftb_rec(t.ftb_rec, seq0, b.base, b.ck, b.ah_v, b.ah_key_base, b.ah_key_ex_fh, empty_s2, b.ah_meta);
        n.blk_seq = seq0 + 1;
      }

      if (pred_arch != 1) {
        if (ah_mode == 0) {
          // Launch next-block 1-Ahead A 1RW SRAM read from (b.base, b.ck.ex_fh)
          s0_launched = true;
          s0_ah_base = b.base;
          std::memcpy(s0_ah_ex_fh, b.ck.ex_fh, sizeof s0_ah_ex_fh);
          n.ah_pipe.valid = true;
          n.ah_pipe.key_base = b.base;
          std::memcpy(n.ah_pipe.key_ex_fh, b.ck.ex_fh, sizeof n.ah_pipe.key_ex_fh);
        } else {
          if (pred_arch == 2) {
            n.ah_a1_raw = old_ah_a0;
          } else {
            // A1: evaluate registered old_ah_a0 for T+1 block (b.g.next) and latch into n.ah_a1
            FtbAhEval a1{};
            a1.valid = true;
            a1.eval_base = b.g.next;
            a1.ah_v = old_ah_a0.valid;
            a1.ah_key_base = old_ah_a0.key_base;
            std::memcpy(a1.ah_key_ex_fh, old_ah_a0.key_ex_fh, sizeof a1.ah_key_ex_fh);
            a1.ah_hit = ftb_.ahead_select(b.g.next, block_bnd(b.g.next), old_ah_a0, 0, 0, a1.g, &a1.ah_meta,
                                          single ? a1.slot_dir : nullptr, single ? &a1.pred_meta : nullptr, &a1.ret_bpc);
            n.ah_a1 = a1;
          }
          // A0: launch Ahead SRAM read for T+2 block
          s0_launched = true;
          s0_ah_base = (ah_mode == 3) ? b.base : b.g.next;
          const uint16_t (*fh_ptr)[EX_TN] = (ah_mode == 1) ? n.spec.ex_fh : b.ck.ex_fh;
          std::memcpy(s0_ah_ex_fh, fh_ptr, sizeof s0_ah_ex_fh);
          n.ah_pipe.valid = true;
          n.ah_pipe.key_base = s0_ah_base;
          std::memcpy(n.ah_pipe.key_ex_fh, s0_ah_ex_fh, sizeof n.ah_pipe.key_ex_fh);
        }
      }

      n.pc = b.g.next;
      n.prev_base = b.base;
      out.icache_read_valid = true;
      out.fetch_address = b.base;
    };

    if (!single && d.valid) {
      if (n.pq_n < PQ) { n.pq[(n.pq_h + n.pq_n) % PQ] = d.blk->ck; n.pq_n++; }
      out.PTAB_write_enable = true;
      out.out_pred_base_pc = d.blk->base;
      for (int i = 0; i < FW; i++) out.out_pred_dir[i] = d.out.dir[i];
      out.meta = d.out.meta;
      out.predict_next_fetch_address = d.out.next;
      t.rec_v = true;
      t.rec.ck = d.blk->ck; t.rec.base = d.blk->base; t.rec.id = seq0; t.rec.epoch = epoch0; t.rec.valid = true;
      t.ftb_rec_v = true;
      fill_ftb_rec(t.ftb_rec, seq0, d.blk->base, d.blk->ck,
                   d.blk->ah_v, d.blk->ah_key_base, d.blk->ah_key_ex_fh,
                   d.s2_meta, (pred_arch == 2) ? d.s2_ah_meta : d.blk->ah_meta,
                   (pred_arch == 2) ? d.s2_ah_self0 : false, d.blk->prev_base);
      if (pred_arch == 2 && cfg_fe_uftb_fast_fill() && d.s2_meta.ftb.hit && !d.blk->g.hit && d.out.taken) {
        t.do_fast_ftb0 = true;
        t.fast_ftb0_base = d.blk->base;
        t.fast_ftb0_ent = d.s2_meta.ftb.ent;
      }
      for (int i = 0; i < FW; i++) out.meta.blk_id[i] = seq0;
      n.blk_seq = seq0 + 1;
    }

    const Blk s1_blk = n.R[0];  // registered S1 block reading Main S2 SRAM this cycle
    const Blk s2_blk = n.R[lat - 1];  // registered last-stage block evaluated by decide() before R shifts!
    if (!single) {
      if (lat == 3) {
        n.R[2] = n.R[1];
        n.ftb_raw_s3 = n.ftb_raw;
      }
      n.R[1] = n.R[0];
      n.R[0].valid = false;
    }

    auto clear_R = [&] { for (int i = 0; i < MAIN_LAT_MAX; i++) n.R[i].valid = false; };
    if (in.refetch) {
      clear_R();
      n.ovr_pend = false;
      n.ah_pipe.valid = false;
      n.ah_a1.valid = false;
      n.ah_a1_raw.valid = false;
      n.pc = in.refetch_address;
      n.prev_base = 0;
      if (in.refetch_pd && in.pd.valid && in.pd.flush && pd_have) {
        restore(pd_rec, n.spec, n.sras);
        hp("PDFIX", 0, 0, in.pd.nbits, in.pd.bits);
        push_bits(in.pd.nbits, in.pd.bits, in.pd.ras_op, in.pd.ras_val, n.spec, n.sras);
        if (in.pd.ras_op2 == 1) n.sras.push(in.pd.ras_val2);
        else if (in.pd.ras_op2 == 2) n.sras.pop();
      } else if (!(in.rf_valid && repair_blk(in, n))) {
        hp("RARCH", in.rf_pc, in.refetch_address, 0, 0);
        n.spec = n.arch; n.sras = RasQ::from_commit(n.aras); n.epoch = epoch0 + 1;
      }
      n.pq_n = 0;
    } else if (n.ovr_pend) {
      n.ovr_pend = false;
      n.ah_a1.valid = false;
      n.ah_a1_raw.valid = false;
      n.prev_base = n.ovr_base;
      restore(n.ovr_ck, n.spec, n.sras);
      hp("OVR", 0, n.ovr_out.next, n.ovr_out.nbits, n.ovr_out.bits);
      push_bits(n.ovr_out.nbits, n.ovr_out.bits, n.ovr_out.ras_op, n.ovr_out.ras_val, n.spec, n.sras);
      if (pred_arch != 1) {
        s0_launched = true;
        s0_ah_base = (ah_mode == 1 || ah_mode == 2) ? n.pc : n.ovr_base;
        const uint16_t (*fh_ptr)[EX_TN] = (ah_mode == 1) ? n.spec.ex_fh : n.ovr_ck.ex_fh;
        std::memcpy(s0_ah_ex_fh, fh_ptr, sizeof s0_ah_ex_fh);
        n.ah_pipe.valid = true;
        n.ah_pipe.key_base = s0_ah_base;
        std::memcpy(n.ah_pipe.key_ex_fh, s0_ah_ex_fh, sizeof n.ah_pipe.key_ex_fh);
      }
    } else if (d.ovr) {
      clear_R();
      if (fast_ovr) {
        n.ovr_pend = false;
        n.ah_a1.valid = false;
        n.ah_a1_raw.valid = false;
        restore(s2_blk.ck, n.spec, n.sras);
        hp("OVR", 0, d.out.next, d.out.nbits, d.out.bits);
        push_bits(d.out.nbits, d.out.bits, d.out.ras_op, d.out.ras_val, n.spec, n.sras);
        n.pc = d.out.next;
        n.prev_base = s2_blk.base;
        if (pred_arch == 1) {
          n.ah_pipe.valid = false;
        } else {
          s0_launched = true;
          s0_ah_base = (ah_mode == 1 || ah_mode == 2) ? d.out.next : s2_blk.base;
          const uint16_t (*fh_ptr)[EX_TN] = (ah_mode == 1) ? n.spec.ex_fh : s2_blk.ck.ex_fh;
          std::memcpy(s0_ah_ex_fh, fh_ptr, sizeof s0_ah_ex_fh);
          n.ah_pipe.valid = true;
          n.ah_pipe.key_base = s0_ah_base;
          std::memcpy(n.ah_pipe.key_ex_fh, s0_ah_ex_fh, sizeof n.ah_pipe.key_ex_fh);
        }
      } else {
        if (ah_mode == 0 && pred_arch != 1) {
          // Cut A1: decouple 1-Ahead SRAM read enable from combinational d.ovr (speculative read discarded on next cycle's ovr_pend re-read!)
          if (in.icache_read_ready) {
            s0_launched = true;
            s0_ah_base = n.pc;
            std::memcpy(s0_ah_ex_fh, n.spec.ex_fh, sizeof s0_ah_ex_fh);
          }
        } else {
          n.ah_pipe.valid = false;
          n.ah_a1.valid = false;
          n.ah_a1_raw.valid = false;
        }
        n.pc = d.out.next;
        n.ovr_pend = true;
        n.ovr_ck = s2_blk.ck;
        n.ovr_out = d.out;
        n.ovr_base = s2_blk.base;
      }
    } else {
      t.pmu_ptab_stall = !in.icache_read_ready;
      if (in.icache_read_ready) s0_issue(n.pc);
    }

    // ---- Step 1: Zero-T0-Read Commit Pop & Configurable Pipeline Write Plan ----
    FtbTrkState &ft = n.ftb_trk;
    const int tlat = cfg_fe_train_lat();
    TrainPipeEnt pop_ent{};
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
        if (tlat == 0) {
          ftb_.compute_commit_plan(rc, br, t.cp);
          fpr::up(static_cast<uint32_t>(q.pc), q.actual_dir, q.type, static_cast<uint32_t>(q.target), q.pred_dir,
                  q.pcpn, q.is_rvc, static_cast<uint64_t>(static_cast<uint32_t>(rc.base)),
                  (static_cast<uint32_t>(carried) << 4) | static_cast<uint32_t>(br.offset));
        } else {
          pop_ent.valid = true;
          pop_ent.rc = rc;
          pop_ent.br = br;
          pop_ent.q = q;
        }
      }
    }
    if (tlat == 1) {
      if (n.train_pipe[0].valid) {
        const auto &e = n.train_pipe[0];
        ftb_.compute_commit_plan(e.rc, e.br, t.cp);
        fpr::up(static_cast<uint32_t>(e.q.pc), e.q.actual_dir, e.q.type, static_cast<uint32_t>(e.q.target), e.q.pred_dir,
                e.q.pcpn, e.q.is_rvc, static_cast<uint64_t>(static_cast<uint32_t>(e.rc.base)),
                (static_cast<uint32_t>(e.br.carried) << 4) | static_cast<uint32_t>(e.br.offset));
      }
      t.next_train_pipe[0] = pop_ent;
    } else if (tlat >= 2) {
      if (n.train_pipe[1].valid) {
        const auto &e = n.train_pipe[1];
        ftb_.compute_commit_plan(e.rc, e.br, t.cp);
        fpr::up(static_cast<uint32_t>(e.q.pc), e.q.actual_dir, e.q.type, static_cast<uint32_t>(e.q.target), e.q.pred_dir,
                e.q.pcpn, e.q.is_rvc, static_cast<uint64_t>(static_cast<uint32_t>(e.rc.base)),
                (static_cast<uint32_t>(e.br.carried) << 4) | static_cast<uint32_t>(e.br.offset));
      }
      t.next_train_pipe[1] = n.train_pipe[0];
      t.next_train_pipe[0] = pop_ent;
    }

    // Build per-subtable 1RW write-bank masks for TAGE / ITT (write-wins not-hit!)
    const uint32_t s2_t_bmask = uint32_t(ftb_.s2_tage_.nbanks - 1);
    const uint32_t ah_t_bmask = uint32_t(ftb_.atage_.nbanks - 1);
    const uint32_t ah_ex_bmask = uint32_t(ftb_.ah_ex_tage_.nbanks - 1);
    uint8_t s2_tage_busy[EX_TN] = {};
    uint8_t s2_itt_busy[ITT_MAX_EXT] = {};
    uint8_t ah1_tage_busy[TN] = {};
    uint8_t ah1_ex_tage_busy[EX_TN] = {};
    uint8_t ah1_itt_busy[ITT_MAX_EXT] = {};

    if (t.cp.valid) {
      if (!single) {
        if (pred_arch != 2 || sup_tage > 0) {
          for (int k = 0; k < EX_TN; k++) {
            if ((t.cp.tage_wp.tbl_wr_mask >> k) & 1u) {
              s2_tage_busy[k] |= static_cast<uint8_t>(1u << (t.cp.tage_wp.tbl_idx[k] & s2_t_bmask));
            }
          }
        }
        if (pred_arch != 2) {
          for (int k = 0; k < itt_n; k++) {
            if ((t.cp.itt_wp.wr_mask >> k) & 1u) {
              s2_itt_busy[k] |= static_cast<uint8_t>(1u << (t.cp.itt_wp.idx[k] & uint32_t(BPU_ITT_BANKS - 1)));
            }
          }
        }
      }
      if (pred_arch != 1) {
        if (ah_ex) {
          if (t.cp.aex_tage_wp.valid) {
            for (int k = 0; k < EX_TN; k++) {
              if ((t.cp.aex_tage_wp.tbl_wr_mask >> k) & 1u) {
                ah1_ex_tage_busy[k] |= static_cast<uint8_t>(1u << (t.cp.aex_tage_wp.tbl_idx[k] & ah_ex_bmask));
              }
            }
          }
        } else if (t.cp.atage_wp.valid) {
          for (int k = 0; k < TN; k++) {
            if ((t.cp.atage_wp.tbl_wr_mask >> k) & 1u) {
              ah1_tage_busy[k] |= static_cast<uint8_t>(1u << (t.cp.atage_wp.tbl_idx[k] & ah_t_bmask));
            }
          }
        }
        for (int k = 0; k < itt_n; k++) {
          if ((t.cp.aitt_wp.wr_mask >> k) & 1u) {
            ah1_itt_busy[k] |= static_cast<uint8_t>(1u << (t.cp.aitt_wp.idx[k] & uint32_t(BPU_AH1_BANKS - 1)));
          }
        }
      }
    }

    // ---- Step 2: Single-block S1 Main & S0/A0 Ahead 1RW Reads ----
    uint32_t ftb1_rd_mask = 0;
    uint32_t s2_base_rd_mask = 0;
    if (!single && s1_blk.valid) {
      ftb1_rd_mask |= 1u << s2_bank_of_w(static_cast<uint32_t>(s1_blk.base >> SLOT_SHIFT));
      if (pred_arch != 2) {
        s2_base_rd_mask |= 1u << (s1_blk.ex_ix.base_idx & uint32_t(BPU_S2_BASE_BANKS - 1));
      }
      uint8_t tage_sup = 0, itt_sup = 0;
      if (pred_arch != 2 || sup_tage > 0) {
        for (int k = 0; k < EX_TN; k++) {
          if (!((ftb_.s2_tage_.active_mask >> k) & 1u)) continue;
          const uint32_t bk = s1_blk.ex_ix.idx[k] & s2_t_bmask;
          if ((s2_tage_busy[k] >> bk) & 1u) tage_sup |= static_cast<uint8_t>(1u << k);
          else s2_tage_busy[k] |= static_cast<uint8_t>(1u << bk);
        }
      }
      if (pred_arch != 2) {
        for (int k = 0; k < itt_n; k++) {
          const uint32_t bk = s1_blk.ftb_ix.itt_idx[k] & uint32_t(BPU_ITT_BANKS - 1);
          if ((s2_itt_busy[k] >> bk) & 1u) itt_sup |= static_cast<uint8_t>(1u << k);
          else s2_itt_busy[k] |= static_cast<uint8_t>(1u << bk);
        }
      }
      if (n.R[1].valid) {
        if (pred_arch == 2) {
          ftb_.s1_read_uni(s1_blk.base, s1_blk.ex_ix, n.ftb_raw, tage_sup);
          if (cfg_fe_ah_self0() && !s1_blk.ah_v && old_ah_a0.valid && old_ah_a0.key_base == s1_blk.base) {
            n.ftb_raw.self0_v = true;
            n.ftb_raw.self0_pipe = old_ah_a0;
          }
        } else {
          ftb_.s1_read(s1_blk.base, s1_blk.ftb_ix, s1_blk.ex_ix, n.ftb_raw, tage_sup, itt_sup);
        }
      }
    }

    uint8_t ah1_ftb_rd_m = 0;
    if (pred_arch != 1 && s0_launched) {
      if (pred_arch != 2) {
        ah1_ftb_rd_m = static_cast<uint8_t>(1u << (ftb_.aftb_.index(s0_ah_base) & uint32_t(BPU_AH1_BANKS - 1)));
      }
      const FtbBlkIdx ah_ix = BpuFtb::make_ahead_index(s0_ah_base, 0, s0_ah_ex_fh);
      uint8_t ah1_t_sup = 0, ah1_i_sup = 0;
      if (ah_ex) {
        const ExTageIdx ah_ex_ix = make_ahead_ex_tage_index(s0_ah_base, 0, s0_ah_ex_fh, ftb_.ah_ex_tage_.S);
        for (int k = 0; k < EX_TN; k++) {
          const uint32_t bk = ah_ex_ix.idx[k] & ah_ex_bmask;
          if ((ah1_ex_tage_busy[k] >> bk) & 1u) ah1_t_sup |= static_cast<uint8_t>(1u << k);
          else ah1_ex_tage_busy[k] |= static_cast<uint8_t>(1u << bk);
        }
      } else {
        for (int k = 0; k < TN; k++) {
          const uint32_t bk = ah_ix.idx[k] & ah_t_bmask;
          if ((ah1_tage_busy[k] >> bk) & 1u) ah1_t_sup |= static_cast<uint8_t>(1u << k);
          else ah1_tage_busy[k] |= static_cast<uint8_t>(1u << bk);
        }
      }
      for (int k = 0; k < itt_n; k++) {
        const uint32_t bk = ah_ix.itt_idx[k] & uint32_t(BPU_AH1_BANKS - 1);
        if ((ah1_itt_busy[k] >> bk) & 1u) ah1_i_sup |= static_cast<uint8_t>(1u << k);
        else ah1_itt_busy[k] |= static_cast<uint8_t>(1u << bk);
      }
      if (pred_arch == 2) ftb_.ahead_read_tage_itt(s0_ah_base, s0_ah_ex_fh, n.ah_pipe.raw, ah1_t_sup, ah1_i_sup);
      else ftb_.ahead_read(s0_ah_base, s0_ah_ex_fh, n.ah_pipe.raw, ah1_t_sup, ah1_i_sup);
    }

    for (int k = 0; k < EX_TN; k++) t.s2_tbl_busy[k] = s2_tage_busy[k];
    for (int k = 0; k < TN; k++) t.ah1_tbl_busy[k] = ah1_tage_busy[k];
    for (int k = 0; k < EX_TN; k++) t.ah1_ex_tbl_busy[k] = ah1_ex_tage_busy[k];

    // ---- Step 3: Predict-Always-Wins FTB 1-Entry Write Buffers + s2_base + Predecode Clear ----
    if (!single && pred_arch != 2 && t.cp.valid && t.cp.tage_wp.base_wr) {
      const uint32_t bbk = t.cp.tage_wp.base_idx & uint32_t(BPU_S2_BASE_BANKS - 1);
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

    uint32_t ah1_m = ah1_ftb_rd_m;
    if (!single) {
      arbitrate_ftb_wb(n.ftb1_wb, t.cp.valid && t.cp.ftb1_wr, t.cp.ftb1_s, t.cp.ftb1_way, t.cp.ftb1_ent,
                       ftb1_rd_mask, uint32_t(BPU_FTB1_BANKS - 1),
                       t.do_ftb1_wr, t.ftb1_wr, t.next_ftb1_wb);
    }
    if (pred_arch == 0) {
      arbitrate_ftb_wb(n.aftb_wb, t.cp.valid && t.cp.ah_wr, t.cp.ah_s, t.cp.ah_way, t.cp.ah_ent,
                       ah1_m, uint32_t(BPU_AH1_BANKS - 1),
                       t.do_aftb_wr, t.aftb_wr, t.next_aftb_wb);
    }

    const bool new_clr = in.pd.valid && in.pd.nb_valid && pd_have;
    const pc_t new_clr_base = in.pd.nb_base ? in.pd.nb_base : pd_rec.base;
    const pc_t new_clr_pc = in.pd.nb_pc;
    if (new_clr) {
      t.do_clr_ftb0 = true;
      t.clr_ftb0_base = new_clr_base;
      t.clr_ftb0_pc = new_clr_pc;
      if (single && pd_rec.ah_v && new_clr_base == pd_rec.base) {
        t.do_clr_aftb = true;
        t.clr_aftb_key = pd_rec.ah_key_base;
        t.clr_aftb_base = new_clr_base;
        t.clr_aftb_pc = new_clr_pc;
      }
    }
    // 2-cycle 1RW u_ftb1 false-branch clear FSM (matches bpu_ftb_tab.sv g_clr_sram bit-for-bit)
    if (!single) {
      const bool clr_car = (n.clr_t1_pc + SLOT_BYTES == n.clr_t1_base);
      const bool clr_vok = clr_car || (n.clr_t1_pc >= n.clr_t1_base);
      const uint32_t clr_off = clr_car ? 0u : static_cast<uint32_t>((n.clr_t1_pc - n.clr_t1_base) >> SLOT_SHIFT);
      const uint32_t t1_set = ftb_.ftb1_.index(n.clr_t1_base);
      const uint16_t t1_tg = ftb_.ftb1_.tag(n.clr_t1_base);
      const uint32_t t1bk = t1_set & uint32_t(BPU_FTB1_BANKS - 1);
      bool c_hit = false;
      uint8_t c_way = 0;
      FtbEnt c_ent{};
      if (n.clr_t1_v && clr_vok) {
        for (int w = 0; w < FTB_S1_W; w++) {
          if (!c_hit && n.clr_t1_set[w].valid && n.clr_t1_set[w].tag == t1_tg) {
            c_hit = true;
            c_way = static_cast<uint8_t>(w);
            c_ent = n.clr_t1_set[w];
            bool s_any = false;
            for (int k = 0; k < FTB_SLOTS; k++) {
              if (c_ent.s[k].valid && c_ent.s[k].offset == clr_off) c_ent.s[k] = FtbSlot{};
              else if (c_ent.s[k].valid) s_any = true;
            }
            if (!s_any) c_ent.valid = false;
          }
        }
      }
      const bool new_ftb_wr = t.cp.valid && t.cp.ftb1_wr;
      const bool clr_wr_fire = n.clr_t1_v && !t.do_ftb1_wr && !new_ftb_wr && (((ftb1_rd_mask >> t1bk) & 1u) == 0) && c_hit;
      if (clr_wr_fire) {
        t.do_clr_wr = true;
        t.clr_wr_set = t1_set;
        t.clr_wr_tg = t1_tg;
        t.clr_wr_way = c_way;
        t.clr_wr_ent = c_ent;
      }
      t.next_clr_t1_v = n.clr_t1_v && c_hit && !clr_wr_fire;
      bool clr_consumed = false;
      const pc_t cbase = n.clr_pend_v ? n.clr_pend_base : new_clr_base;
      const pc_t cpc = n.clr_pend_v ? n.clr_pend_pc : new_clr_pc;
      const uint32_t cset = ftb_.ftb1_.index(cbase);
      const uint32_t cbk = cset & uint32_t(BPU_FTB1_BANKS - 1);
      if (!t.next_clr_t1_v && (n.clr_pend_v || new_clr)) {
        if ((((ftb1_rd_mask >> cbk) & 1u) == 0) && !(clr_wr_fire && (cbk == t1bk))) {
          t.do_clr_rd = true;
          t.clr_rd_set = cset;
          t.clr_rd_base = cbase;
          t.clr_rd_pc = cpc;
          t.next_clr_t1_v = true;
          clr_consumed = true;
        }
      }
      if (new_clr && !(clr_consumed && !n.clr_pend_v)) {
        t.next_clr_pend_v = true;
        t.next_clr_pend_base = new_clr_base;
        t.next_clr_pend_pc = new_clr_pc;
      } else if (n.clr_pend_v && !clr_consumed) {
        t.next_clr_pend_v = true;
        t.next_clr_pend_base = n.clr_pend_base;
        t.next_clr_pend_pc = n.clr_pend_pc;
      }
    }

    // ---- Step 4: Architectural history/RAS advance & commit push into 16-entry BRQ ----
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      if (!in.in_upd_valid[i]) continue;
      const uint8_t ty = in.in_actual_br_type[i];
      const bool taken = (ty != BR_NONCTL) && in.in_actual_dir[i];
      const pc_t pc = in.in_update_base_pc[i];
      push_arch(n.arch, pc, taken, in.in_actual_targets[i], ty);
      if (br_call(ty)) n.aras.push(pc + ilen_of(in.in_update_is_rvc[i]));
      else if (ty == BR_RET) n.aras.pop();
      if (ty != BR_NONCTL && ft.brq_n < FTB_BRQ_N) {
        FtbBrqEnt &q = ft.brq[(ft.brq_h + ft.brq_n) % FTB_BRQ_N];
        q.blk_id = in.in_blk_id[i]; q.pc = pc; q.is_rvc = in.in_update_is_rvc[i];
        q.type = ty; q.actual_dir = taken; q.pred_dir = in.in_pred_dir[i];
        q.alt_pred = in.in_alt_pred[i]; q.pcpn = in.in_pcpn[i]; q.target = in.in_actual_targets[i];
        ft.brq_n++;
      }
    }

    out.upd_free = 15;
    out.tq_free = 15;
  }

  void commit(const Txn &t) {
    if (t.rec_v) rec_[t.rec.id & 255] = t.rec;
    if (t.ftb_rec_v) ftb_rec_[t.ftb_rec.id & (FTB_REC_N - 1)] = t.ftb_rec;

    // Snapshot T0 1RW SRAM read BEFORE T1/commit writes (read/write are bank-disjoint by construction)
    if (t.do_clr_rd) {
      for (int w = 0; w < FTB_S1_W; w++) st_.clr_t1_set[w] = ftb_.ftb1_.e[t.clr_rd_set * FTB_S1_W + w];
      st_.clr_t1_base = t.clr_rd_base;
      st_.clr_t1_pc = t.clr_rd_pc;
    }
    if (t.cp.valid) ftb_.apply_commit_non_ftb(t.cp, t.do_s2_base);
    else if (t.do_fast_ftb0) ftb_.mirror_ent_to_ftb0(t.fast_ftb0_base, t.fast_ftb0_ent, true);
    if (t.do_ftb1_wr) ftb_.apply_ftb1_write(t.ftb1_wr.set, t.ftb1_wr.way, t.ftb1_wr.ent);
    if (t.do_aftb_wr) ftb_.apply_aftb_write(t.aftb_wr.set, t.aftb_wr.way, t.aftb_wr.ent);
    if (t.do_clr_ftb0) ftb_.clear_false_branch_ftb0(t.clr_ftb0_base, t.clr_ftb0_pc);
    if (t.do_clr_aftb) ftb_.clear_false_branch_aftb(t.clr_aftb_key, t.clr_aftb_base, t.clr_aftb_pc);
    if (t.do_clr_wr) ftb_.apply_ftb1_clr_write(t.clr_wr_set, t.clr_wr_tg, t.clr_wr_way, t.clr_wr_ent);

    if (!cfg_fe_single_pred() && (cfg_fe_pred_arch() != 2 || cfg_fe_s2_sup_tage() > 0)) {
      ftb_.s2_tage_.try_rst(t.s2_tbl_busy);
    }
    if (cfg_fe_pred_arch() != 1) {
      if (cfg_fe_ah_ex_tage() || cfg_fe_pred_arch() == 2) ftb_.ah_ex_tage_.try_rst(t.ah1_ex_tbl_busy);
      else ftb_.atage_.try_rst(t.ah1_tbl_busy);
    }

    st_.ftb1_wb = t.next_ftb1_wb;
    st_.aftb_wb = t.next_aftb_wb;
    st_.clr_t1_v = t.next_clr_t1_v;
    st_.clr_pend_v = t.next_clr_pend_v;
    st_.clr_pend_base = t.next_clr_pend_base;
    st_.clr_pend_pc = t.next_clr_pend_pc;
    st_.train_pipe[0] = t.next_train_pipe[0];
    st_.train_pipe[1] = t.next_train_pipe[1];
  }
};

}  // namespace bpuf

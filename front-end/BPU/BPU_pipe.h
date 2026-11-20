// Pipelined block-based 1-ahead FTB branch predictor (EXP-050 Milestone A, ah1_4k_a2k1k + FTB_BRQ=2).
// Matches rtl/frontend/bpu/ cycle-for-cycle (docs/modules/bpu.md).
//
//   S0 (0 bubble)   Registered 1-ahead SRAM select (aftb 2048x8 + atage 1024x4x4 + aitt 256x3) with uFTB (256x4, 20b tgt)
//                   cold-start fallback; starts next-block 1-ahead SRAM read from (s0_base, s0_fh); pushes spec hist/RAS.
//   S1 (1 bubble)   Exact-PC SRAM read (Main FTB 4096x4 + S2 Block-TAGE 4096x4x4 + Single Block-ITT 256x3); triggers 1-bubble
//                   override only when S0 missed 1-ahead (!r0.ah_hit).
//   S2 (2 bubbles)  Final select on registered S1 raw data -> PTAB write + ftb_rec_[512] write; 2-bubble override on disagreement.
//   Commit side     16-entry 1-branch/cycle queue (brq[16], never backpressures ROB) + 1R1W ftb_rec_[512] -> single-branch RMW.
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

namespace bpuf {

constexpr int MAIN_LAT = 2;
inline int cfg_pair() { return 0; }
constexpr int PQ = 64;        // PTAB checkpoint records (BPO_PDCKPT)
constexpr int HR_BITS = 256;  // circular global-history ring
constexpr uint32_t HR_MASK = HR_BITS - 1u;

struct HistRing {
  uint64_t bits[HR_BITS / 64] = {};
  uint32_t head = 0;
  uint32_t fh[FH_N_MAX][TN] = {};
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
  uint8_t ras_tosr = 0, ras_tosw = 0, ras_cnt = 0;
  target_addr_t ras_top = 0;
};

struct S0Guess {
  bool hit = false, taken = false;
  uint8_t slot = 0, type = 0;
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
  bool rvc = false;
  bool dir[FW] = {};
  PredMeta meta;
};

struct Blk {
  bool valid = false;
  pc_t base = 0, bnd = 0;
  S0Guess g;
  Ckpt ck;
  FtbBlkIdx ftb_ix;
  bool ah_v = false;
  bool ah_hit = false;
  pc_t ah_key_base = 0;
  uint32_t ah_key_fh[FH_N_MAX][TN] = {};
};

struct Decision {
  bool valid = false;
  bool ovr = false;
  uint8_t squash_n = 0;
  const Blk *blk = nullptr;
  MainOut out;
  bool pair = false, ovr_b = false;
  MainOut outB;
  Ckpt ock; MainOut oout; pc_t obase = 0; S0Guess og;
};

struct RecP {
  Ckpt ck; pc_t base = 0; uint16_t id = 0; uint8_t epoch = 0; bool valid = false;
};

// The front end's BPU phase (ports named after the front end's wires).
//   CONTRACT comb_front  in: in.global_refetch in.global_reset in.in.ctrl in.in.redirect in.in.train in.pipe_dec in.rd.ptab_rd_snapshot in.rd.q  out: out.bpu_can_run out.bpu_output out.bpu_seq_txn out.pmu_l1_ovr out.pmu_ptab_stall
class BpuPipe {
 public:
  struct In {
    const bool *global_reset = nullptr;
    const bool *global_refetch = nullptr;
    const FrontTopIn *in = nullptr;
    const FrontReadData *rd = nullptr;
    const Decision *pipe_dec = nullptr;
  } in;
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
    Blk R[MAIN_LAT];
    FtbRaw ftb_raw;
    FtbAhPipe ah_pipe[2];
    FtbTrkState ftb_trk;
    HistRing spec, arch;
    RasQ sras;
    RasC aras;
  };

  struct Txn {
    bool rec_v = false;
    RecP rec;
    bool pmu_l1_ovr = false, pmu_ptab_stall = false;
    bool ftb_ev_v = false, ftb_clr_v = false, ftb_rec_v = false;
    FtbTrainEv ftb_ev{};
    FtbBlkRec ftb_rec{};
    pc_t ftb_clr_base = 0, ftb_clr_pc = 0;
    void clear() {
      rec_v = false;
      pmu_l1_ovr = pmu_ptab_stall = false;
      ftb_ev_v = ftb_clr_v = ftb_rec_v = false;
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
    r.tosr = c.ras_tosr; r.tosw = c.ras_tosw; r.cnt = c.ras_cnt;
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

  static bool disagree(const MainOut &m, const S0Guess &g) {
    if (m.next != g.next) return true;
    if (m.nbits != g.nbits || m.bits != g.bits) return true;
    if (m.ras_op != g.ras_op || (m.ras_op == 1 && m.ras_val != g.ras_val)) return true;
    return false;
  }

  bool repair_blk(const InputPayload &in, State &n) const {
    const RecP &rc = rec_[in.rf_blk & 255];
    const bool carried = rc.valid && in.rf_pc + SLOT_BYTES == rc.base;
    const uint32_t rslot = carried ? 0 : (in.rf_pc - rc.base) >> SLOT_SHIFT;
    if (!(rc.valid && rc.id == in.rf_blk && rc.epoch == (*stp_).epoch && rslot < FW && (carried || rc.base + SLOT_BYTES * rslot == in.rf_pc))) return false;
    restore(rc.ck, n.spec, n.sras);
    const bool tk = in.rf_type == BR_DIRECT ? (in.refetch_address != in.rf_pc + ilen_of(in.rf_is_rvc)) : true;
    if (tk) { const uint8_t sy = hsym(in.rf_pc, in.refetch_address); hp("RFIX", in.rf_pc, in.refetch_address, hsym_, sy); for (int k = 0; k < hsym_; k++) n.spec.push((sy >> k) & 1); }
    if (br_call(in.rf_type)) n.sras.push(in.rf_pc + ilen_of(in.rf_is_rvc)); else if (in.rf_type == BR_RET) n.sras.pop();
    return true;
  }

  static void ftb_emit_warm(FtbTrkState &ft, BpuFtb &ftb) {
    if (ft.nbr == 0) return;
    FtbTrainEv ev{};
    ev.start = ft.start;
    std::memcpy(ev.fh, ft.fh, sizeof ev.fh);
    ev.nbr = ft.nbr;
    for (int i = 0; i < ft.nbr; i++) ev.br[i] = ft.br[i];
    ftb.apply_train_ev(ev);
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
            const FtbBlkIdx ix = BpuFtb::make_index(ft.start, ft.fh);
            BlockTage::Pred tp;
            ftb_.s2_tage_.predict_slot(ix, k, tp);
            pred = tp.pred; alt = tp.alt; pcpn = tp.pcpn;
            break;
          }
        }
      }
    }
    if (!ft.v || pc != ft.expect || pc < ft.start || pc >= block_bnd(ft.start)) {
      if (ft.v && ft.nbr > 0) ftb_emit_warm(ft, ftb_);
      ft.v = true; ft.start = pc; ft.nbr = 0;
      std::memcpy(ft.fh, n.arch.fh, sizeof ft.fh);
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
    if (br_call(ty)) n.aras.push(pc + ilen_of(rvc));
    else if (ty == BR_RET) n.aras.pop();
    if (taken) {
      ftb_emit_warm(ft, ftb_);
      ft.start = target; ft.expect = target; ft.nbr = 0;
      std::memcpy(ft.fh, n.arch.fh, sizeof ft.fh);
    } else {
      const pc_t nxt = pc + ilen_of(rvc);
      if (ty == BR_DIRECT && pred) {
        ftb_emit_warm(ft, ftb_);
        ft.start = nxt; ft.nbr = 0;
        std::memcpy(ft.fh, n.arch.fh, sizeof ft.fh);
      } else if (nxt >= bnd) {
        ftb_emit_warm(ft, ftb_);
        ft.start = bnd; ft.nbr = 0;
        std::memcpy(ft.fh, n.arch.fh, sizeof ft.fh);
      }
      ft.expect = nxt;
    }
  }
  void warm_finish() { (*stp_).spec = (*stp_).arch; (*stp_).sras = RasQ::from_commit((*stp_).aras); }

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
    ftb_.s2_select(b.base, b.bnd, b.ftb_ix, (*stp_).ftb_raw, b.ck.ras_cnt, b.ck.ras_top, d.out);
    d.ovr = disagree(d.out, b.g);
    d.ock = b.ck; d.oout = d.out; d.obase = b.base; d.og = b.g;
    if (d.ovr) for (int k = 0; k < MAIN_LAT - 1; k++) d.squash_n += (*stp_).R[k].valid;
  }

  bool l1_override(const Blk &b, State &n, Txn &t) const {
    if (!b.valid || b.ah_hit) return false;
    const S0Guess g1 = ftb_.s1_predict<S0Guess>(b.base, b.bnd, n.ftb_raw, b.ck.ras_cnt, b.ck.ras_top);
    if (!g1.hit) return false;
    MainOut m; m.next = g1.next; m.nbits = g1.nbits; m.bits = g1.bits; m.ras_op = g1.ras_op; m.ras_val = g1.ras_val;
    if (!disagree(m, b.g)) return false;
    t.pmu_l1_ovr = true;
    n.R[1].g = g1;
    restore(b.ck, n.spec, n.sras);
    hp("L1OVR", b.base, g1.next, g1.nbits, g1.bits);
    push_bits(g1.nbits, g1.bits, g1.ras_op, g1.ras_val, n.spec, n.sras);
    n.pc = g1.next;
    return true;
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

    auto s0_issue = [&](pc_t pc) {
      Blk &b = n.R[0];
      b.valid = true;
      b.base = pc;
      b.bnd = block_bnd(b.base);
      const FtbAhPipe &ah_src = n.ah_pipe[0];
      b.ah_v = ah_src.valid;
      b.ah_key_base = ah_src.key_base;
      std::memcpy(b.ah_key_fh, ah_src.key_fh, sizeof b.ah_key_fh);
      b.ah_hit = ftb_.ahead_select(b.base, b.bnd, ah_src, n.sras.cnt, n.sras.top(), b.g);
      if (!b.ah_hit) b.g = ftb_.s0_lookup<S0Guess>(b.base, b.bnd, n.sras.cnt, n.sras.top());

      // Advance 1-ahead pipeline using registered (b.base, n.spec.fh) BEFORE push_bits
      n.ah_pipe[1] = n.ah_pipe[0];
      n.ah_pipe[0].valid = true;
      n.ah_pipe[0].key_base = b.base;
      std::memcpy(n.ah_pipe[0].key_fh, n.spec.fh, sizeof n.ah_pipe[0].key_fh);
      ftb_.ahead_read(b.base, n.spec.fh, n.ah_pipe[0].raw);

      b.ck = take_ckpt(n.spec, n.sras);
      b.ck.base = b.base;
      b.ftb_ix = BpuFtb::make_index(b.base, n.spec.fh);
      hp("S0", b.base, b.g.next, b.g.nbits, b.g.bits);
      push_bits(b.g.nbits, b.g.bits, b.g.ras_op, b.g.ras_val, n.spec, n.sras);
      n.pc = b.g.next;
      out.icache_read_valid = true;
      out.fetch_address = b.base;
    };

    if (d.valid) {
      if (n.pq_n < PQ) { n.pq[(n.pq_h + n.pq_n) % PQ] = d.blk->ck; n.pq_n++; }
      out.PTAB_write_enable = true;
      out.out_pred_base_pc = d.blk->base;
      for (int i = 0; i < FW; i++) out.out_pred_dir[i] = d.out.dir[i];
      out.meta = d.out.meta;
      out.predict_next_fetch_address = d.out.next;
      t.rec_v = true;
      t.rec.ck = d.blk->ck; t.rec.base = d.blk->base; t.rec.id = seq0; t.rec.epoch = epoch0; t.rec.valid = true;
      t.ftb_rec_v = true;
      t.ftb_rec.valid = true; t.ftb_rec.id = seq0; t.ftb_rec.base = d.blk->base;
      std::memcpy(t.ftb_rec.fh, d.blk->ck.fh, sizeof t.ftb_rec.fh);
      t.ftb_rec.ah_v = d.blk->ah_v;
      t.ftb_rec.ah_key_base = d.blk->ah_key_base;
      std::memcpy(t.ftb_rec.ah_key_fh, d.blk->ah_key_fh, sizeof t.ftb_rec.ah_key_fh);
      for (int i = 0; i < FW; i++) out.meta.blk_id[i] = seq0;
      n.blk_seq = seq0 + 1;
    }

    int inflight = 0;
    for (int k = 0; k < MAIN_LAT; k++) inflight += n.R[k].valid;
    for (int k = MAIN_LAT - 1; k >= 1; k--) n.R[k] = n.R[k - 1];
    n.R[0].valid = false;
    if (n.R[1].valid) ftb_.s1_read(n.R[1].base, n.R[1].ftb_ix, n.ftb_raw);

    auto clear_R = [&] { for (int k = 0; k < MAIN_LAT; k++) n.R[k].valid = false; };
    if (in.refetch) {
      clear_R();
      n.ah_pipe[0].valid = false; n.ah_pipe[1].valid = false;
      n.pc = in.refetch_address;
    } else if (d.ovr) {
      clear_R();
      if (n.ah_pipe[1].valid && n.ah_pipe[1].key_base == d.obase) {
        n.ah_pipe[0] = n.ah_pipe[1];
        n.ah_pipe[1].valid = false;
      } else if (!(n.ah_pipe[0].valid && n.ah_pipe[0].key_base == d.obase)) {
        n.ah_pipe[0].valid = false;
        n.ah_pipe[1].valid = false;
      }
      restore(d.ock, n.spec, n.sras);
      hp("OVR", 0, d.oout.next, d.oout.nbits, d.oout.bits);
      push_bits(d.oout.nbits, d.oout.bits, d.oout.ras_op, d.oout.ras_val, n.spec, n.sras);
      n.pc = d.oout.next;
    } else if (l1_override(n.R[1], n, t)) {
      // 1-bubble cold-start S1 override
    } else {
      const bool ptab_stall = in.icache_read_ready && (int(in.ptab_size) + inflight >= PTAB_SIZE - 1);
      t.pmu_ptab_stall = ptab_stall;
      if (in.icache_read_ready && !ptab_stall) s0_issue(n.pc);
    }

    // Commit side: 1-branch/cycle queue (brq[16]) + 1R1W ftb_rec_[512]
    FtbTrkState &ft = n.ftb_trk;
    if (ft.brq_n > 0) {
      const FtbBrqEnt &q = ft.brq[ft.brq_h];
      ft.brq_h = static_cast<uint8_t>((ft.brq_h + 1) % FTB_BRQ_N);
      ft.brq_n--;
      const FtbBlkRec &rc = ftb_rec_[q.blk_id & (FTB_REC_N - 1)];
      if (rc.valid && rc.id == q.blk_id) {
        const bool carried = (q.pc + SLOT_BYTES == rc.base);
        if (carried || (q.pc >= rc.base && q.pc < block_bnd(rc.base))) {
          FtbTrainEv ev{};
          ev.start = rc.base;
          std::memcpy(ev.fh, rc.fh, sizeof ev.fh);
          ev.ah_v = rc.ah_v;
          ev.ah_key_base = rc.ah_key_base;
          std::memcpy(ev.ah_key_fh, rc.ah_key_fh, sizeof ev.ah_key_fh);
          ev.nbr = 1;
          ev.br[0].offset = carried ? 0 : static_cast<uint8_t>((q.pc - rc.base) >> SLOT_SHIFT);
          ev.br[0].carried = carried;
          ev.br[0].is_rvc = q.is_rvc;
          ev.br[0].type = q.type;
          ev.br[0].actual_dir = q.actual_dir;
          ev.br[0].pred_dir = q.pred_dir;
          ev.br[0].alt_pred = q.alt_pred;
          ev.br[0].pcpn = q.pcpn;
          ev.br[0].target = q.target;
          t.ftb_ev_v = true;
          t.ftb_ev = ev;
          fpr::up(static_cast<uint32_t>(q.pc), q.actual_dir, q.type, static_cast<uint32_t>(q.target), q.pred_dir,
                  q.pcpn, q.is_rvc, static_cast<uint64_t>(static_cast<uint32_t>(rc.base)),
                  (static_cast<uint32_t>(carried) << 4) | static_cast<uint32_t>(ev.br[0].offset));
        }
      }
    }
    for (int i = 0; i < CW; i++) {
      if (!in.in_upd_valid[i]) continue;
      const pc_t pc = in.in_update_base_pc[i];
      const br_type_t ty = in.in_actual_br_type[i];
      const bool taken = (ty != BR_NONCTL) && in.in_actual_dir[i];
      const bool rvc = in.in_update_is_rvc[i];
      push_arch(n.arch, pc, taken, in.in_actual_targets[i], ty);
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
    if (in.refetch) {
      if (in.refetch_pd && in.pd.valid && in.pd.flush && pd_have) {
        restore(pd_rec, n.spec, n.sras);
        hp("PDFIX", 0, 0, in.pd.nbits, in.pd.bits);
        push_bits(in.pd.nbits, in.pd.bits, in.pd.ras_op, in.pd.ras_val, n.spec, n.sras);
      } else if (!(in.rf_valid && repair_blk(in, n))) {
        hp("RARCH", in.rf_pc, in.refetch_address, 0, 0);
        n.spec = n.arch; n.sras = RasQ::from_commit(n.aras); n.epoch = epoch0 + 1;
      }
      n.pq_n = 0;
      if (in.icache_read_ready) s0_issue(in.refetch_address);
    }
    if (in.pd.valid && in.pd.nb_valid && pd_have) {
      t.ftb_clr_v = true;
      t.ftb_clr_base = pd_rec.base;
      t.ftb_clr_pc = in.pd.nb_pc;
    }
    out.upd_free = 15;
    out.tq_free = 15;
  }

  void commit(const Txn &t) {
    if (t.rec_v) rec_[t.rec.id & 255] = t.rec;
    if (t.ftb_rec_v) ftb_rec_[t.ftb_rec.id & (FTB_REC_N - 1)] = t.ftb_rec;
    if (t.ftb_ev_v) ftb_.apply_train_ev(t.ftb_ev);
    if (t.ftb_clr_v) ftb_.clear_false_branch(t.ftb_clr_base, t.ftb_clr_pc);
  }
};

}  // namespace bpuf

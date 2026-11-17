#pragma once
#include <vector>

#include "IO.h"
#include "MemUtils.h"
#include "util.h"
#include "RealLsuPipe.h"
#include "TlbMmu.h"
#include "config.h"
#include <cstdint>

constexpr int MAX_IDX_WIDTH = clog2(std::max(LDQ_SIZE, STQ_SIZE));
class SimContext;

struct UncachedUnit {
  wire<1> valid;
  wire<1> is_load;
  wire<32> addr;
  wire<32> wdata;
  wire<3> func3;
  wire<MAX_IDX_WIDTH> idx;
};

struct LrScUnit {
  wire<1> reserve_valid;
  wire<32> reserve_addr;
  wire<BR_MASK_WIDTH> reserve_br_mask;
};

// Units the LSU shares with the checkpoint restore; the queues live in the staged LSU state (lp::LsuPipeState, RealLsuPipe.h).
struct LsuState {
  UncachedUnit uncached_unit;
  LrScUnit lrsc_unit;
};

struct BackIn;

// Load / store unit (staged pipeline, docs/LSU_PIPE.md). Contract (checked by scripts/comb/comb_graph.py against the
// code): the signals each phase reads (in) and writes (out), and the outputs only this module drives (in / out: parts
// of BackTop's shared input / output structs).
//   CONTRACT comb_outputs  in: -  out: out.lsu2dis out.lsu2rob out.lsu_awake
//   CONTRACT comb_writeback  in: in.dec_bcast in.rob_bcast in.rob_commit in.top_in.dcache2lsu  out: out.lsu2exe out.lsu_awake
//   CONTRACT comb_translate  in: in.dec_bcast in.exe2lsu in.rob_bcast in.dtlb_walk  out: out.dtlb_walk
//   CONTRACT comb_requests  in: in.dec_bcast in.dis2lsu in.exe2lsu in.rob_bcast in.rob_commit in.top_in.dcache2lsu out.lsu2dcache  out: out.lsu2dcache out.peripheral_req
//   CONTRACT comb_commit_store  in: in.rob_bcast in.rob_commit  out: out.commit_store
class RealLsu {
public:
  void comb_outputs();  // outputs from the registers: wakeups, free counts to dispatch, completion to the ROB
  void comb_writeback();  // committed stores, MMIO and DCache responses, load write-back to the EXU
  void comb_translate();  // AGU / STD results in, address translation, store-to-load forwarding
  void comb_requests();  // allocation from dispatch, flush, the DCache / MMIO requests, ordering checks
  void comb_commit_store();  // the committed stores to a modeled device (from the ROB's commit and the registered STQ)
  RealLsu() : nxt{}, ctx(nullptr), cur_{} {}
  explicit RealLsu(SimContext *ctx) : nxt{}, ctx(ctx), cur_{} {}

  // Registers: cur_ / pcur_ (q: read in comb through const references, written only by seq) and nxt / pnxt (d: written
  // by comb; d == q at the start of every cycle); pw holds the wires between the LSU's phases of one cycle.
  LsuState nxt;

  LsuState &cur_mut() { return cur_; }   // checkpoint restore only
  // Outputs: the signals this module drives (it alone).
  struct Out {
    LsuDisIO lsu2dis;
    LsuRobIO lsu2rob;
    LsuAwakeIO lsu_awake;
    LsuExeIO lsu2exe;
    LsuDcacheIO *lsu2dcache = nullptr;   // in BackTop (the back end's interface), bound by BackTop::init
    PeripheralReqIO *peripheral_req = nullptr;   // in BackTop (the back end's interface), bound by BackTop::init
    CommitStoreIO *commit_store = nullptr;       // in BackTop (the back end's interface), bound by BackTop::init
    PtwWalkPortCombIn *dtlb_walk = nullptr;   // the DTLB's requests to the page-table walker (its TLB, dtlb, writes them)
  } out;
  PeripheralRespIO mmio_resp_d{};   // the MMIO response (the memory subsystem drives it after the LSU's phases); q: mmio_resp_q_
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const PtwWalkPortCombOut *dtlb_walk = nullptr;   // the page-table walker's outputs to the DTLB (registered)
    const DecBroadcastIO *dec_bcast = nullptr;
    const DisLsuIO *dis2lsu = nullptr;
    const ExeLsuIO *exe2lsu = nullptr;
    const BackIn *top_in = nullptr;   // the back end's inputs (SimCpu)
    const RobBroadcastIO *rob_bcast = nullptr;
    const RobCommitIO *rob_commit = nullptr;
  } in;
  SimContext *ctx = nullptr;
  TlbMmu dtlb{DTLB_ENTRIES, TlbMmu::Kind::DTLB};   // the DTLB (looked up in comb_translate, its edge in seq)

  void init();
  void seq() {
    dtlb.seq();
    cur_ = nxt;
    seq_pipe();
    mmio_resp_q_ = mmio_resp_d;
  }

  // Staged LSU: the comb phases above are in RealLsuPipe.cpp, design in docs/LSU_PIPE.md.
  lp::LsuPipeState pnxt;

  lp::PCycle pw;

  StqEntry get_stq_entry(int idx, bool flag);
  void seq_pipe();  // pcur = pnxt, copying only the live LDQ / STQ entries (see RealLsuPipe.cpp)

private:
  PeripheralRespIO mmio_resp_q_{};   // the MMIO response of the previous cycle (loaded by seq)
  // phases of one cycle, run in the order of the methods above
  void cal();
  void lsu2dis();
  void lsu2rob();
  void stq_commit();
  void mmio_in();
  void dcache2lsu();
  void lsu2exe();
  void exe2lsu();
  void tlb();
  void stlf();
  void dis2lsu();
  void flush();
  void lsu2dcache();
  void mmio_out();
  void check();
  long long wd_next_ = 0x7fffffffffffffffLL;  // earliest watchdog deadline (stale entries only cause an extra scan)
  void wd_arm(long long &t) { t = sim_time; if (sim_time + 62 < wd_next_) wd_next_ = sim_time + 62; }
  void lp_emit_load_wb(int port, int ldq_idx, bool from_l3 = false);
  void tlb_out();
  void tlb_in_ldq();
  void tlb_in_stq();
  void lsu2dcache_ldq();
  void lsu2dcache_stq();
  void dcache2lsu_ldq();
  void dcache2lsu_stq();
  // this cycle's DTLB requests and responses, the load ports then the STA ports (also printed by LSU_PIPE_EVT)
  TlbMmu::Req tlb_req_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  TlbMmu::Resp tlb_resp_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
  // Wait table of the memory-dependence predictor (Alpha 21264 style, EXP-018/019): 2^CFG_LSU_MDP_BITS one-bit entries
  // indexed by a PC hash, set when a load of that PC violated, cleared every CFG_LSU_MDP_CLR cycles. A load whose bit is
  // set does not pass unknown-address stores. Training as in the RTL: the violators of a cycle are only flagged (trn, one
  // bit per LDQ entry); one flagged live entry per cycle (the lowest LDQ index) trains the table in the following cycle
  // (pend), and the bit is written at the start of the cycle after that. Updated in place (read before written).
  struct MemDepPred {
    static constexpr unsigned kBits = CFG_LSU_MDP_BITS, kClr = CFG_LSU_MDP_CLR;
    uint64_t trn = 0;           // flagged violators (LDQ bit mask)
    std::vector<int> pend;      // table indices trained in the previous cycle, written at the start of this one
    std::vector<bool> table;
    long long next_clear = 0;
    static uint32_t idx(uint32_t pc) { return ((pc >> SLOT_SHIFT) ^ (pc >> (SLOT_SHIFT + kBits))) & ((1u << kBits) - 1); }   // (RVC=1: 2-byte aligned)
    bool wait(uint32_t pc) {
      if (table.empty()) table.assign(1u << kBits, false);
      return table[idx(pc)];
    }
    void write_pending() {
      if (pend.empty()) return;
      if (table.empty()) table.assign(1u << kBits, false);
      for (int ix : pend) table[ix] = true;
      pend.clear();
    }
  };
  MemDepPred mdp_;
  LsuState cur_;
  lp::LsuPipeState pcur_;
};

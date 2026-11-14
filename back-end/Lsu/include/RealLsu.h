#pragma once

#include "IO.h"
#include "MemUtils.h"
#include "RealLsuBsdConfig.h"
#include "RealLsuPipe.h"
#include "TlbMmu.h"
#include "config.h"
#include <cstdint>

constexpr int LDQ_STQ_IDX_WIDTH = clog2(LDQ_SIZE + STQ_SIZE);
constexpr int MAX_IDX_WIDTH = clog2(std::max(LDQ_SIZE, STQ_SIZE));
class SimContext;

// Numeric values are kept stable (lockstep digest compares them).
enum class LoadState : uint8_t {
  Empty = 0,
  Allocated = 1,
  WaitTlb = 3,
  WaitTlbRetry = 4,
  WaitSTLF = 5,
  CheckStlf = 6,
  CheckStlfRetry = 7,
  ReadyToIssue = 9,
  WaitDcacheResp = 10,
  WaitMmioResp = 11,
  ReadyToWb = 12,
  PageFault = 14,
  WaitFinish = 15,
  WaitDcache = 16,
};

struct LdqEntry {
  wire<1> illegal_inst;   // F/D: the load is illegal (mstatus.FS = Off), reported through the write-back
  LoadState load_state;
  wire<1> v_addr_valid;
  wire<XLEN> v_addr;
  wire<1> p_addr_valid;
  wire<32> p_addr;
  ReplayType replay_type;
  wire<3> func3;
  wire<XLEN> result;
  wire<PRF_IDX_WIDTH> dest_preg;
  wire<ROB_IDX_WIDTH> rob_idx;
  wire<1> rob_flag;
  wire<BR_MASK_WIDTH> br_mask;
  wire<1> is_mmio;
  wire<XLEN> diag_val;  // faulting virtual address for the exception handler
  wire<1> page_fault;
  wire<1> is_lrsc;
  StoreTag stq_snapshot;
  wire<1> cache_miss;
};

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
  wire<ROB_IDX_WIDTH> reserve_rob_idx;
  wire<1> reserve_rob_flag;
  wire<BR_MASK_WIDTH> reserve_br_mask;
};

// Index queues hold entries in arrival order; a consumed entry in the middle only turns invalid and is
// popped once it reaches the head, so holes still occupy window slots (this is timing-relevant).
struct QIdx {
  wire<1> valid;
  wire<MAX_IDX_WIDTH> idx;
};
struct QWait : QIdx {
  wire<31 - LDQ_IDX_WIDTH> req_gen;
  wire<LSU_REPLAY_WAIT_CYCLES_WIDTH> replay_wait_cycles;
};

template <class E, int N>
struct Ring {
  E e[N];
  wire<clog2(N)> head;
  wire<clog2(N) + 1> count;
  uint32_t at(uint32_t i) const { return (head + i) % N; }
  E &push(uint32_t idx) {
    E &s = e[at(count)];
    s.valid = true;
    s.idx = idx;
    count++;
    return s;
  }
  void pop() {
    e[head].valid = false;
    head = (head + 1) % N;
    if (count > 0) count--;
  }
  // Pop invalid entries from the head, looking at most `window` slots deep.
  void trim(uint32_t window) {
    const uint32_t h = head;
    for (uint32_t i = 0; i < window && !e[(h + i) % N].valid; i++) pop();
  }
  void clear() {
    count = 0;
    for (auto &x : e) x.valid = false;
  }
  uint32_t window(uint32_t w) const { return count < w ? count : w; }
};

struct LsuState {
  LdqEntry ldq[LDQ_SIZE];

  StqEntry stq[STQ_SIZE];
  wire<STQ_IDX_WIDTH> stq_head;
  wire<STQ_IDX_WIDTH> stq_commit;
  wire<1> stq_head_flag;
  wire<STQ_IDX_WIDTH + 1> stq_commit_count;  // committed, not yet drained
  wire<STQ_IDX_WIDTH + 1> stq_count;         // including allocated-but-uncommitted

  Ring<QIdx, STQ_SIZE> wait_mmu_stq, mmu_done_stq;
  Ring<QIdx, LDQ_SIZE> wait_mmu_ldq, finish, stlf_queue;
  Ring<QWait, LDQ_SIZE> wait_dcache_ldq;

  UncachedUnit uncached_unit;
  LrScUnit lrsc_unit;
  wire<31 - LDQ_IDX_WIDTH> req_gen;  // tags DCache requests so stale replays are dropped
};

enum class STLFResult : wire<2> { Disjoint = 0, Overlap = 1, Retry = 2 };

class RealLsu {
public:
  RealLsu() : cur{}, nxt{}, ctx(nullptr) {}
  explicit RealLsu(SimContext *ctx) : cur{}, nxt{}, ctx(ctx) {}

  LsuState cur;
  LsuState nxt;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  SimContext *ctx = nullptr;

  void init();
  void comb_cal();
  void comb_spec_cancel();  // LSU_PIPE_L2: cancel of last cycle's speculative load wake-up (before the select)
  void comb_lsu2dis();
  void comb_lsu2rob();
  void comb_mmio_out();
  void comb_mmio_in();
  void comb_tlb_out();
  void comb_tlb_in_ldq();
  void comb_tlb_in_stq();
  void comb_exe2lsu();
  void comb_dis2lsu();
  void comb_lsu2dcache_ldq();
  void comb_dcache2lsu_ldq();
  void comb_lsu2dcache_stq();
  void comb_dcache2lsu_stq();
  void comb_lsu2exe();
  void comb_stq_commit();
  void comb_flush();
  void comb_stlf();
  void comb_check();
  void seq() {
    if (pipe) {   // the staged LSU keeps its queues in pcur / pnxt; of the legacy state only these two units are shared
      cur.uncached_unit = nxt.uncached_unit;
      cur.lrsc_unit = nxt.lrsc_unit;
      seq_pipe();
    } else {
      cur = nxt;
    }
  }

  // Staged LSU (switch LSU_PIPE / CFG_LSU_PIPE): same comb phases, see RealLsuPipe.cpp and docs/LSU_PIPE.md.
  bool pipe = false;
  lp::LsuPipeState pcur, pnxt;
  lp::PCycle pw;
  int lp_rp_ports = CFG_LSU_PIPE_RP;  // replay registers (LSU_PIPE_RP)
  int lp_bounce = 0;        // aged replay register bumps the last new load (LSU_PIPE_BOUNCE cycles, 0 = off)
  int lp_rblk = 1;          // do not launch on a port with a pending DCache hidden retry (LSU_PIPE_RBLK)
  int lp_gate = 1;          // DCache read enable qualified by the partial CAM (LSU_PIPE_GATE)

  StqEntry get_stq_entry(int idx, bool flag);
  void seq_pipe();  // pcur = pnxt, copying only the live LDQ / STQ entries (see RealLsuPipe.cpp)

private:
  uint32_t ldq_allocated_ = 0, ldq_empty_ = 0, stq_allocated_ = 0;  // start-of-cycle counts from comb_cal

  void handle_store_addr(const MicroOp &inst);
  void handle_store_data(const MicroOp &inst);
  void handle_load_req(const MicroOp &inst);

  // lp: the *_p phases replace the legacy ones when `pipe` is set
  void comb_cal_p();
  void comb_spec_cancel_p();
  void comb_lsu2dis_p();
  void comb_lsu2rob_p();
  void comb_mmio_in_p();
  void comb_tlb_out_p();
  void comb_tlb_in_ldq_p();
  void comb_tlb_in_stq_p();
  void comb_exe2lsu_p();
  void comb_dis2lsu_p();
  void comb_lsu2dcache_ldq_p();
  void comb_dcache2lsu_ldq_p();
  void comb_lsu2dcache_stq_p();
  void comb_dcache2lsu_stq_p();
  void comb_lsu2exe_p();
  void comb_stq_commit_p();
  void comb_flush_p();
  void comb_stlf_p();
  void comb_check_p();
  void comb_mmio_out_p();
  void lp_emit_load_wb(int port, int ldq_idx, bool from_l3 = false);
};

#pragma once

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

class RealLsu {
public:
  void comb_outputs();  // outputs from the registers: wakeups, free counts to dispatch, completion to the ROB
  void comb_writeback();  // committed stores, MMIO and DCache responses, load write-back to the EXU
  void comb_translate();  // AGU / STD results in, address translation, store-to-load forwarding
  void comb_requests();  // allocation from dispatch, flush, the DCache / MMIO requests, ordering checks
  RealLsu() : cur{}, nxt{}, ctx(nullptr) {}
  explicit RealLsu(SimContext *ctx) : cur{}, nxt{}, ctx(ctx) {}

  LsuState cur;
  LsuState nxt;
  struct Wires *wires = nullptr;  // signals shared with the other back-end modules
  SimContext *ctx = nullptr;
  TlbMmu *dtlb = nullptr;         // the DTLB (looked up in comb_tlb)

  void init();
  void seq() {
    cur = nxt;
    seq_pipe();
  }

  // Staged LSU: the comb phases above are in RealLsuPipe.cpp, design in docs/LSU_PIPE.md.
  lp::LsuPipeState pcur, pnxt;
  lp::PCycle pw;

  StqEntry get_stq_entry(int idx, bool flag);
  void seq_pipe();  // pcur = pnxt, copying only the live LDQ / STQ entries (see RealLsuPipe.cpp)

private:
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
  void lp_emit_load_wb(int port, int ldq_idx, bool from_l3 = false);
  void tlb_out();
  void tlb_in_ldq();
  void tlb_in_stq();
  void lsu2dcache_ldq();
  void lsu2dcache_stq();
  void dcache2lsu_ldq();
  void dcache2lsu_stq();
  LsuMMUIO tlb_req_{};    // this cycle's DTLB requests and responses (also printed by LSU_PIPE_EVT)
  MMULsuIO tlb_resp_{};
};

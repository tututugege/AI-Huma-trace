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
  wire<ROB_IDX_WIDTH> reserve_rob_idx;
  wire<1> reserve_rob_flag;
  wire<BR_MASK_WIDTH> reserve_br_mask;
};

// Units the LSU shares with the checkpoint restore; the queues live in the staged LSU state (lp::LsuPipeState, RealLsuPipe.h).
struct LsuState {
  UncachedUnit uncached_unit;
  LrScUnit lrsc_unit;
};

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
    cur = nxt;
    seq_pipe();
  }

  // Staged LSU: the comb phases above are in RealLsuPipe.cpp, design in docs/LSU_PIPE.md.
  lp::LsuPipeState pcur, pnxt;
  lp::PCycle pw;

  StqEntry get_stq_entry(int idx, bool flag);
  void seq_pipe();  // pcur = pnxt, copying only the live LDQ / STQ entries (see RealLsuPipe.cpp)

private:
  void lp_emit_load_wb(int port, int ldq_idx, bool from_l3 = false);
};

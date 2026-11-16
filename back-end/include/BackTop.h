#pragma once
#include "Csr.h"
#include "Dispatch.h"
#include "Exu.h"
#include "FTQ.h"
#include "IO.h"
#include "Wires.h"
#include "Idu.h"
#include "Isu.h"
#include "PreIduQueue.h"
#include "Ren.h"
#include "RealLsu.h"
#include "Rob.h"
#include "TlbMmu.h"
#include "config.h"
#include <cstdint>
#include <memory>

class SimContext;
class PtwWalkPort;

class Dispatch;
class Idu;
class Ren;
class Isu;
struct CPU_state;
class Exu;
class Rob;
class Csr;
class AbstractMMU;
class MemSubsystem;

// The back end's own phase: its outputs to the front end and the memory side (redirect, commit, stall) and the CSR's
// FS-dirty request. Contract (checked by scripts/comb/comb_graph.py against the code):
//   CONTRACT comb_outputs  in: csr2front dec_bcast ftq_commit_info pre2front rob_bcast rob_commit  out: top2csr out.commit_entry out.commit_extra out.commit_ftq out.fence_i out.flush out.itlb_flush out.mispred out.redirect_pc out.rf_blk out.rf_is_rvc out.rf_pc out.rf_slot out.rf_type out.rf_valid out.stall
//   CONTRACT owns: top2csr out.commit_entry out.commit_extra out.commit_ftq out.fence_i out.flush out.itlb_flush out.mispred out.redirect_pc out.rf_blk out.rf_is_rvc out.rf_pc out.rf_slot out.rf_type out.rf_valid out.stall
class BackTop : public Wires {
private:
  std::unique_ptr<TlbMmu> dtlb_mmu;
  void comb_outputs();
  void install_arch_state(const CPU_state &state, uint8_t privilege);

public:
  void warm_dtlb(const TlbMmu::TlbEntry *entries, int count, int repl_ptr) {  // fast-warmup replay (EXP-011)
    if (dtlb_mmu) dtlb_mmu->warm_install(entries, count, repl_ptr);
  }
private:

public:
  SimContext *ctx;

  PreIduQueue *pre;
  Idu *idu;
  Ren *rename;
  Dispatch *dis;
  Isu *isu;
  Exu *exu;
  Csr *csr;
  Rob *rob;
  RealLsu *lsu;

  void init();
  void clear_comb_inputs();
  void comb_csr_status();
  void comb();
  void pmu_sample();  // PMU (EXP-013): one cycle of the event bus, called at the end of comb()
  void seq();
  void set_lsu_ptw_walk_port(PtwWalkPort *port);

  BackTop(SimContext *ctx) {
    this->ctx = ctx;
    pre = nullptr;
    out.fire = nullptr;
  };
  ~BackTop() {
    delete pre;
    delete rename;
    delete dis;
    delete idu;
    delete isu;
    delete exu;
    delete rob;
    delete csr;
    delete lsu;
  }

  wire<XLEN> number_PC = 0;
  uint64_t ckpt_interval_inst_count = 0;

  void load_image(const std::string &filename);
  void restore_checkpoint(const std::string &filename);
  // Front-door restore: only the memory of the checkpoint is loaded and the restore program (stub) is put in the boot
  // window; the DUT and the reference core start from reset and execute it. ckpt_interval_inst_count is still read.
  void restore_checkpoint_front_door(const std::string &filename, const std::string &stub_file);
  void restore_from_ref();

  uint32_t get_reg(uint8_t arch_idx) {
    (void)arch_idx;
    return 0;  // no register file: architectural values live in the trace bridge
  }
};

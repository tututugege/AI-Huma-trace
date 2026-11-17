#pragma once
#include "Dispatch.h"
#include "Exu.h"
#include "FTQ.h"
#include "IO.h"
#include "BackPorts.h"
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
class AbstractMMU;
class MemSubsystem;

// SimCpu-level phases (include/SimCpu.h, scripts/comb/gen_top_order.py):
// The modules' comb phases (their CONTRACT lines and comb_outputs above) are SimCpu-level phases themselves:
// scripts/comb/gen_top_order.py schedules them with the other tops' phases (back.<module>.<phase>).
class BackTop : public BackPorts {
private:

public:
  void warm_dtlb(const TlbMmu::TlbEntry *entries, int count, int repl_ptr) {  // fast-warmup replay (EXP-011)
    lsu->dtlb.warm_install(entries, count, repl_ptr);
  }
private:

public:
  SimContext *ctx;

  std::unique_ptr<PreIduQueue> pre;
  std::unique_ptr<Idu> idu;
  std::unique_ptr<Ren> rename;
  std::unique_ptr<Dispatch> dis;
  std::unique_ptr<Isu> isu;
  std::unique_ptr<Exu> exu;
  std::unique_ptr<Rob> rob;
  std::unique_ptr<RealLsu> lsu;

  void init();
  void pmu_sample();     // PMU (EXP-013): one cycle of the event bus, after the comb phases of the cycle
  void seq();

  BackTop(SimContext *ctx) {
    this->ctx = ctx;
  };
  ~BackTop();

  wire<XLEN> restored_pc = 0;   // the pc of the last install_arch_state (the driver restarts the fetch there)

  // Architectural state the timing model itself keeps (privilege, CSRs, the LR/SC reservation), from a checkpoint or
  // the reference core; the pipeline starts with a refetch from state.pc.
  void install_arch_state(const CPU_state &state);
  // between cycles (a switch from fast mode): the redirect outputs as if the back end redirected to pc, the CSR
  // status refreshed (satp, privilege: the MMU mode)
  void restart_at(pc_t pc);
};

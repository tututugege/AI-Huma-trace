#pragma once
#include "IO.h"
#include "ExuTypes.h"
#include "Fu.h"
#include "config.h"
#include "FTQ.h"
#include <vector>
#include <utility>

// One functional unit: static description + its state. A unit holds at most one issued uop in its input slot (valid from the
// cycle after issue until the unit takes it); PIPE units a ring of in-flight uops, ITER units one busy-until uop.
struct FuUnit {
  FuKind kind = FuKind::ALU;
  FuMode mode = FuMode::SINGLE;
  int port = -1;
  uint64_t support_mask = 0;
  int lsu_agu_port = -1;
  int lsu_sdu_port = -1;
  int depth = 0;  // PIPE: latency == capacity

  bool slot_valid = false;
  ExuInst slot{};

  struct PipeEntry {
    ExuInst uop;
    int64_t done_cycle = 0;
  };
  std::vector<PipeEntry> ring;  // PIPE
  int head = 0, count = 0;
  bool busy = false;  // ITER
  int64_t done = 0;
  ExuInst cur{};
  ExuInst out{};  // SINGLE: result of this cycle

  bool idle() const { return mode == FuMode::SINGLE ? true : mode == FuMode::PIPE ? count == 0 : !busy; }
  PipeEntry &front() { return ring[head]; }
  void pop_front() { head = head + 1 == depth ? 0 : head + 1; count--; }
  void push_back(const PipeEntry &e) {
    int i = head + count;
    if (i >= depth) i -= depth;
    ring[i] = e;
    count++;
  }
};

// Execution units. Contract (checked by scripts/comb/comb_graph.py against the code): the signals each phase reads (in)
// and writes (out), and the outputs only this module drives.
//   CONTRACT comb_exec  in: in.dec_bcast in.lsu2exe in.rob_bcast  out: out.exe2iss out.exe2lsu out.exe2prf out.exu2id out.exu2rob out.prf_awake
class Exu {
public:
  void comb_exec();  // late load wakeup, the FUs, the port readiness for issue
  Exu(SimContext *ctx);

  SimContext *ctx;

  void init();
  void seq();           // the write-back register; the FU state is updated in place (each phase runs once per cycle, in order)

  // Outputs: the signals this module drives (it alone).
  struct Out {
    ExeIssIO exe2iss;
    ExeLsuIO exe2lsu;
    ExePrfIO exe2prf;
    ExuIdIO exu2id;
    ExuRobIO exu2rob;
    PrfAwakeIO prf_awake;
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const DecBroadcastIO *dec_bcast = nullptr;
    const IssPrfIO *iss2prf = nullptr;
    const LsuExeIO *lsu2exe = nullptr;
    const RobBroadcastIO *rob_bcast = nullptr;
  } in;

  const class PreIduQueue *ftq = nullptr;

private:
  // phases of one cycle, run in the order of the methods above
  void load_wake();
  void exec();
  void ready();
  void begin_cycle();   // called by load_wake(), the first phase of the cycle
  // The write-back register (one cycle after exe2prf.entry), kept only for what it is used for: a load completing on a
  // port that the LSU did not wake early wakes its consumers one cycle later (comb_load_wake -> prf_awake).
  struct LoadWake {
    wire<1> valid = false;
    wire<PRF_IDX_WIDTH> preg = 0;
    wire<BR_MASK_WIDTH> br_mask = 0;
  };
  // registers: q_ (read in comb through const Regs &q) and d (written by comb); seq: q_ = d. (The FU state in `units`
  // is still updated in place in comb: not yet under the contract.)
  struct Regs { LoadWake lwake_r[ISSUE_WIDTH] = {}; };
  Regs q_;
  std::vector<FuUnit> units;
  std::vector<int> port_fus[ISSUE_WIDTH];  // FU indices of each port, in write-back priority order
  uint64_t port_fu_mask[ISSUE_WIDTH] = {};
};

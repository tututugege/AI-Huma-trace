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

class Exu {
public:
  Exu(SimContext *ctx);

  SimContext *ctx;

  void init();
  void comb_begin();    // idle default outputs
  void comb_exec();     // FU controls, FU evaluation, result collection
  void comb_to_csr();
  void comb_pipeline(); // issued uops enter the FU input slots
  void comb_ready();    // which operation classes each port can accept
  void seq() {}         // all state is updated in place (each phase runs once per cycle, in order)

  struct Wires *wires = nullptr;  // signals shared with the other back-end modules

private:
  std::vector<FuUnit> units;
  std::vector<int> port_fus[ISSUE_WIDTH];  // FU indices of each port, in write-back priority order
  uint64_t port_fu_mask[ISSUE_WIDTH] = {};
};

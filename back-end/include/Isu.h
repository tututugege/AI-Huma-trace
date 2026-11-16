#pragma once
#include "IssueQueue.h"
#include "config.h"
#include "IO.h"
#include <cstdint>

// MUL fixed-latency wakeup entry (shift register)
struct MulWakeEntry {
  wire<1> valid = 0;
  wire<PRF_IDX_WIDTH> dest_preg = 0;
  wire<BR_MASK_WIDTH> br_mask;
};

// DIV / FP iterative wakeup entry (countdown slot)
struct IterWakeEntry {
  wire<1> valid = 0;
  int countdown = 0;
  wire<PRF_IDX_WIDTH> dest_preg = 0;
  wire<BR_MASK_WIDTH> br_mask;
};

constexpr int ISU_MUL_WAKE_UNIT_NUM = count_ports_with_mask(OP_MASK_MUL);
constexpr int ISU_DIV_WAKE_UNIT_NUM = count_ports_with_mask(OP_MASK_DIV);
constexpr int ISU_MUL_WAKE_DEPTH = (MUL_MAX_LATENCY > 1) ? (MUL_MAX_LATENCY - 1) : 1;
constexpr int ISU_MUL_WAKE_SLOT_NUM =
    (ISU_MUL_WAKE_UNIT_NUM > 0) ? ISU_MUL_WAKE_UNIT_NUM : 1;
constexpr int ISU_DIV_WAKE_SLOT_NUM =
    (ISU_DIV_WAKE_UNIT_NUM > 0) ? ISU_DIV_WAKE_UNIT_NUM : 1;
// F/D: the delayed wake-ups of every FP unit and the integer -> FP conversions share one slot array (a slot lives from the issue of
// the uop to its wake-up; at most FPW_CH of them fire in one cycle, each on its own channel)
constexpr int ISU_FP_WAKE_SLOT_NUM = 24;

// Issue unit (the issue queues and the wake-up network). Contract (checked by scripts/comb/comb_graph.py against the
// code): the signals each phase reads (in) and writes (out), and the outputs only this module drives.
//   CONTRACT comb_issue  in: dec_bcast exe2iss lsu_awake prf_awake rob_bcast  out: iss2dis iss2prf iss_awake
//   CONTRACT comb_enq    in: dec_bcast dis2iss rob_bcast                      out: -
//   CONTRACT owns: iss2dis iss2prf iss_awake
class Isu {
public:
  // Registers: q (read in comb through const Regs &q, written only by seq) and d (their next value, written by comb;
  // d == q at the start of every cycle). seq: q = d.
  struct Regs {
    MulWakeEntry mul_wake_pipe[ISU_MUL_WAKE_DEPTH][ISU_MUL_WAKE_SLOT_NUM];
    IterWakeEntry div_wake_slots[ISU_DIV_WAKE_SLOT_NUM];
    IterWakeEntry fp_wake_slots[ISU_FP_WAKE_SLOT_NUM];
  };
  Regs d;
  const Regs &q() const { return q_; }
  Regs &q_mut() { return q_; }   // checkpoint restore only
  void comb_issue();  // free slots, select and issue, the wakeup bus, next-cycle latencies
  void comb_enq();  // enqueue from dispatch, flush
  friend struct IsuDigestAccess;  // lockstep test hook
private:
  // phases of one cycle, run in the order of the methods above
  void ready();
  void issue();
  void awake();
  void calc_latency_next();
  void enq();
  void flush();
  void begin_cycle();   // called by ready(), the first phase of the cycle
  SimContext *ctx;

  // 1. issue queues (fixed order: INT, LD, STA, STD, BR, ...)
  // a vector; init() keeps the index equal to the IQ id
  std::vector<IssueQueue> iqs;
  std::vector<IssueQueueConfig> configs;

  // 2. delayed wakeups
  // MUL: fixed latency, a shift register (depth MUL_MAX_LATENCY - 1)
  // DIV / FP: iterative units, countdown slots (one per FU)

public:
  uint32_t port_attributes[ISSUE_WIDTH];
  // Outputs: the signals this module drives (it alone).
  struct Out {
    IssDisIO iss2dis;
    IssPrfIO iss2prf;
    IssAwakeIO iss_awake;
  } out;
  // Inputs: the signals this module reads (const: it cannot write them), bound by BackTop::init.
  struct In {
    const DecBroadcastIO *dec_bcast = nullptr;
    const DisIssIO *dis2iss = nullptr;
    const ExeIssIO *exe2iss = nullptr;
    const LsuAwakeIO *lsu_awake = nullptr;
    const PrfAwakeIO *prf_awake = nullptr;
    const RobBroadcastIO *rob_bcast = nullptr;
  } in;
  Isu(SimContext *ctx);
  void init();

  // comb

  // helpers

  // seq
  void seq();

private:
  void add_iq(const IssueQueueConfig &cfg);
  int get_latency(UopType uop_type, wire<7> func7, bool word = false);
  bool grant_gate(const Uop &u, int port);   // extra grant conditions (port completion cycle, FP write port)
  void apply_wakeup_to_uop(Uop &uop) const;
  void clear_wake_state();
  // Apply f to every delayed-wakeup slot of the working copy (MUL pipeline stages, DIV and FP slots).
  template <class F> void for_each_wake_slot_d(F f) {   // the wake slots of d
    for (auto &stage : d.mul_wake_pipe) for (auto &e : stage) f(e);
    for (auto &e : d.div_wake_slots) f(e);
    for (auto &e : d.fp_wake_slots) f(e);
  }

private:
  Regs q_;
};

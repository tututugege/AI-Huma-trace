#pragma once
#include "BPU/BPU_pipe.h"
#include "front_IO.h"
#include "front_fifo_ports.h"
#include "train_IO.h"

class SimContext;
class ICacheTop;
class PreIduQueue;

// The front end's control (the RTL's front_ctrl / front_ctrl_enables): Unified FTQ-64 enables (no ptab_bypass,
// no front2back_fifo, explicit inst_fifo backpressure on ifu_ptr) and end-of-cycle state / FTQ write requests.
//   CONTRACT control  in: in.in.ctrl in.in.redirect in.rd.q  out: out.front_state out.global_refetch out.global_reset out.pipe_dec out.pipe_head_ok
//   CONTRACT enables  in: in.bpu_output in.in.ctrl in.rd.fifo_rd_snapshot in.rd.q  out: out.f2b_read_enable out.fetch_addr_head out.fetch_addr_read_enable out.fifo_read_enable out.s2_wr_en
//   CONTRACT finish  in: in.bpu_can_run in.bpu_output in.checker_out in.f1_next_entry in.fifo_head in.fifo_rd in.flush_address in.grp_ptab in.ic_k in.ic_out in.in.ctrl in.in.redirect in.lh_new_data in.lh_new_pc in.lh_new_valid in.lh_updated in.pd_next in.pmu_l1_ovr in.pmu_ptab_stall in.predecode_flush in.rd.q out.out.FIFO_valid out.out.ftq_s2 out.out.inst_valid out.out.meta out.out.pc out.out.predict_dir  out: out.front_state out.out out.out.commit_stall out.out.commit_tq_free out.out.commit_upd_free
class FrontControl {
public:
  struct In {
    const FrontTopIn *in = nullptr;           // the front end's inputs
    const FrontReadData *rd = nullptr;        // its registers (q) and inst_fifo read data
    const bpuf::OutputPayload *bpu_output = nullptr;
    const bool *bpu_can_run = nullptr;
    const InstFifoReadData *fifo_rd = nullptr;
    const InstFifoOut *fifo_head = nullptr;
    const InstFifoEntry *f1_next_entry = nullptr;
    const PtabOut *grp_ptab = nullptr;
    const PredecodeCheckerOut *checker_out = nullptr;
    const ICacheOut *ic_out = nullptr, *ic_k = nullptr;
    const bool *predecode_flush = nullptr, *lh_new_valid = nullptr, *lh_updated = nullptr;
    const pc_t *flush_address = nullptr, *lh_new_pc = nullptr;
    const uint16_t *lh_new_data = nullptr;
    const bpuf::PdInfo *pd_next = nullptr;
    const bool *pmu_l1_ovr = nullptr, *pmu_ptab_stall = nullptr;
  } in;
  const bpuf::BpuPipe *bpu_pipe = nullptr;  // the BPU's registers (its last-stage decision)
  const ICacheTop *icache = nullptr;        // the I-cache's read readiness (from its registers)
  const PreIduQueue *ftq = nullptr;         // read-only Unified FTQ-64 registers (PreIduQueue::q)
  SimContext *ctx = nullptr;
  struct Out {
    bool global_reset = false;
    bool global_refetch = false;
    bpuf::Decision pipe_dec{};        // the BPU pipeline's last-stage decision of this cycle (from its registers)
    bool pipe_head_ok = true;         // ifu_ptr head is >= 1 cycle old (EXP-044)
    bool fetch_addr_read_enable = false;
    FetchAddrFifoOut fetch_addr_head{};  // ifu_ptr entry popped to ICache this cycle
    bool fifo_read_enable = false;    // predecode runs on inst_fifo head + Unified FTQ pd_ptr
    bool f2b_read_enable = false;     // backend can accept a group this cycle (can_read)
    bool s2_wr_en = false;            // BPU S2 writes prediction + ftq_meta into Unified FTQ s2_ptr
    PendingFrontState front_state{};  // the front end's registers of the next cycle (FrontCore::seq)
    FrontTopOut *out = nullptr;       // the front end's output (FrontTop::out), bound by FrontCore
  } out;
  void control();
  void enables();
  void finish();
};

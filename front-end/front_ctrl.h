#pragma once
#include "BPU/BPU_pipe.h"
#include "front_IO.h"
#include "front_fifo_ports.h"
#include "train_IO.h"

class SimContext;
class ICacheTop;

// The front end's control (the RTL's front_ctrl / front_ctrl_enables): the flush / refetch of the cycle and the BPU
// pipeline's last-stage decision, the FIFOs' read enables and the PTAB write, and at the end the front end's registers
// of the next cycle (front_state, loaded by FrontCore::seq), the output toward the back end and the PMU delivery split.
// Port names are the front end's wire names.
//   CONTRACT control  in: in.in.ctrl in.in.redirect in.rd.fetch_addr_fifo_rd_snapshot in.rd.q  out: out.front_state out.global_refetch out.global_reset out.pipe_dec out.pipe_head_ok
//   CONTRACT enables  in: in.bpu_output in.in.ctrl in.rd.q  out: out.f2b_read_enable out.fetch_addr_read_enable out.fifo_read_enable out.ptab_bypass out.ptab_can_write
//   CONTRACT finish  in: in.bpu_output in.f2b_bypass in.f2b_head in.f2b_rd in.fetch_addr_head in.fetch_addr_rd in.fifo_rd in.flush_address in.ic_k in.ic_out in.in.ctrl in.in.redirect in.lh_new_data in.lh_new_pc in.lh_new_valid in.lh_updated in.pd_next in.pmu_l1_ovr in.pmu_ptab_stall in.predecode_flush in.ptab_rd in.rd.q in.use_f2b_bypass out.out.FIFO_valid out.out.inst_valid out.out.is_rvc out.out.pc out.out.predict_dir  out: out.front_state out.out out.out.commit_stall out.out.commit_tq_free out.out.commit_upd_free
class FrontControl {
public:
  struct In {
    const FrontTopIn *in = nullptr;           // the front end's inputs
    const FrontReadData *rd = nullptr;        // its registers (q) and the FIFOs' read data
    const bpuf::OutputPayload *bpu_output = nullptr;
    const FetchAddrFifoOut *fetch_addr_head = nullptr;
    const FetchAddrFifoReadData *fetch_addr_rd = nullptr;
    const InstFifoReadData *fifo_rd = nullptr;
    const PtabReadData *ptab_rd = nullptr;
    const F2bFifoReadData *f2b_rd = nullptr;
    const F2bFifoOut *f2b_head = nullptr, *f2b_bypass = nullptr;
    const bool *use_f2b_bypass = nullptr;
    const ICacheOut *ic_out = nullptr, *ic_k = nullptr;
    const bool *predecode_flush = nullptr, *lh_new_valid = nullptr, *lh_updated = nullptr;
    const pc_t *flush_address = nullptr, *lh_new_pc = nullptr;
    const uint16_t *lh_new_data = nullptr;
    const bpuf::PdInfo *pd_next = nullptr;
    const bool *pmu_l1_ovr = nullptr, *pmu_ptab_stall = nullptr;
  } in;
  const bpuf::BpuPipe *bpu_pipe = nullptr;  // the BPU's registers (its last-stage decision)
  const ICacheTop *icache = nullptr;        // the I-cache's read readiness (from its registers)
  SimContext *ctx = nullptr;
  struct Out {
    bool global_reset = false;
    bool global_refetch = false;
    bpuf::Decision pipe_dec{};        // the BPU pipeline's last-stage decision of this cycle (from its registers)
    bool pipe_head_ok = true;         // the fetch-address FIFO head may be popped (not a block the last stage overrides)
    bool fetch_addr_read_enable = false;
    bool fifo_read_enable = false;    // the instruction FIFO and the PTAB are read together
    bool f2b_read_enable = false;
    bool ptab_can_write = false;      // the BPU's block enters the PTAB this cycle
    bool ptab_bypass = false;         // ... and the checker reads it in the same cycle
    PendingFrontState front_state{};  // the front end's registers of the next cycle (FrontCore::seq)
    FrontTopOut *out = nullptr;       // the front end's output (FrontTop::out), bound by FrontCore
  } out;
  void control();
  void enables();
  void finish();
};

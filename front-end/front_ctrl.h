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
class FrontControl {
public:
  struct In {
    const FrontTopIn *in = nullptr;           // the front end's inputs
    const FrontReadData *rd = nullptr;        // its registers (q) and the FIFOs' read data
    const bpuf::BpuPipe *bpu_pipe = nullptr;  // the BPU's registers (its last-stage decision) and its transaction (PMU)
    const ICacheTop *icache = nullptr;        // the I-cache's read readiness (from its registers)
    SimContext *ctx = nullptr;
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
  } in;
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

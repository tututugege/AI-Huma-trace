#pragma once
// All state of the front end, owned by FrontTop (one instance per FrontTop): its own registers, the BPU pipeline, the four
// FIFOs, the I-cache, the I-cache prefetcher. One cycle is read_q (the registers' q, read data of the FIFOs), the comb
// phases (each a phase of one module; the order generated from the contracts above class FrontCore) and seq (the clock edge).
#include "BPU/BPU_pipe.h"
#include "env.h"
#include "fifo/front_fifo.h"
#include "front_checker.h"
#include "front_fifo_ports.h"
#include "front_ctrl.h"
#include "icache/icache_top.h"
#include "train_IO.h"
#include <cstdlib>
#include <memory>
#include <unordered_set>
#include <vector>

class SimContext;

// FE_IPF: I-cache prefetch driven by the BPU run-ahead. Every fetch address the BPU pushes into the fetch-address FIFO is a
// candidate: if its line is not in the I-cache, has been filled before (so it is assumed to sit in the LLC) and no prefetch
// of it is in flight, a prefetch is started and the line is installed into the I-cache after FE_IPF_LAT cycles (tag + valid
// only; LLC / bus traffic of the prefetch is not modelled). Runs at the clock edge.
class FrontIpf {
public:
  void tick(ICacheTop &icache, uint64_t now, const uint32_t *cand, int n, CsrStatusIO *csr) {
    if (!on_) return;
    if (!hooked_) { hooked_ = true; icache.fill_hook = [this](uint32_t index, uint32_t tag) { seen_.insert(key(index, tag)); }; }
    for (size_t i = 0; i < fl_.size();) {
      if (fl_[i].ready <= now) { icache.prefetch_install(fl_[i].index, fl_[i].tag); fl_.erase(fl_.begin() + i); }
      else i++;
    }
    for (int c = 0; c < n; c++) {
      uint32_t pa;
      if (!icache.peek_translate(cand[c], csr, pa)) continue;
      const uint32_t index = (pa >> ICACHE_OFFSET_BITS) & (ICACHE_SET_NUM - 1u), tag = pa >> 12;
      if (icache.peek_hit(index, tag) || !seen_.count(key(index, tag))) continue;
      bool dup = false;
      for (const Fl &f : fl_) if (f.index == index && f.tag == tag) dup = true;
      if (dup || fl_.size() >= kMaxFl) continue;
      fl_.push_back({index, tag, now + uint64_t(lat_)});
    }
  }

private:
  static constexpr size_t kMaxFl = 4;   // in-flight instruction-prefetch fills
  struct Fl { uint32_t index, tag; uint64_t ready; };
  static uint32_t key(uint32_t index, uint32_t tag) { return (tag << 8) | index; }
  const int on_ = sim_env().fe_ipf;
  const int lat_ = sim_env().fe_ipf_lat;
  bool hooked_ = false;   // the I-cache reports its demand fills from the first tick on
  std::vector<Fl> fl_;
  std::unordered_set<uint32_t> seen_;   // lines filled before
};

// The phases' contracts (scripts/comb/gen_front_order.py checks them against the modules' code and generates
// FrontCore::comb() from them). Signals are the front end's wires: the modules' ports carry their names (In / Out
// fields), plus in (the front end's inputs), rd (the registers, q), out (its output) and bpu_pipe.txn (the BPU
// pipeline's transaction of the cycle). A FIFO's state is written by its read port and then its write port; every other
// signal has one writer.
//   CONTRACT fifo fetch_addr: fetch_addr_in fetch_addr_rd fetch_addr_fifo_req  ports: read_fetch_addr write_fetch_addr
//   CONTRACT fifo instruction: fifo_in fifo_rd fifo_req  ports: read_instruction write_instruction_fifo
//   CONTRACT fifo front2back: f2b_in f2b_rd front2back_fifo_req f2b_bypass use_f2b_bypass  ports: read_f2b write_front2back
//   CONTRACT control  in: global_refetch in pipe_dec rd  out: global_refetch global_reset pipe_dec pipe_head_ok
//   CONTRACT run_bpu  in: bpu_can_run bpu_output global_refetch global_reset in pipe_dec rd  out: bpu_can_run bpu_output bpu_pipe.txn
//   CONTRACT enables  in: bpu_output global_refetch global_reset in pipe_dec pipe_head_ok ptab_bypass ptab_can_write rd  out: f2b_read_enable fetch_addr_read_enable fifo_read_enable ptab_bypass ptab_can_write
//   CONTRACT read_fetch_addr  in: fetch_addr_fifo_req fetch_addr_in fetch_addr_rd fetch_addr_read_enable global_refetch global_reset pipe_dec rd  out: fetch_addr_fifo_req fetch_addr_head fetch_addr_in fetch_addr_rd
//   CONTRACT read_ptab  in: bpu_output fifo_read_enable global_refetch global_reset ptab_bypass ptab_can_write ptab_in ptab_rd ptab_req rd  out: ptab_head ptab_in ptab_rd ptab_req
//   CONTRACT read_instruction  in: fifo_head fifo_in fifo_rd fifo_read_enable global_refetch global_reset grp_ptab ptab_head rd  out: carry_low fifo_head fifo_in fifo_rd fifo_req grp_in_range grp_ptab grp_seg_first multi rvc_n rvc_straddle
//   CONTRACT read_f2b  in: f2b_in f2b_rd f2b_read_enable global_reset in rd  out: f2b_bypass f2b_head f2b_in f2b_rd front2back_fifo_req use_f2b_bypass
//   CONTRACT write_fetch_addr  in: bpu_can_run bpu_output fetch_addr_fifo_req fetch_addr_in fetch_addr_rd global_reset  out: fetch_addr_fifo_req fetch_addr_in fetch_addr_rd
//   CONTRACT run_icache  in: fetch_addr_head global_refetch global_reset ic_in ic_out in  out: ic_in ic_out
//   CONTRACT write_instruction_fifo  in: fetch_addr_head fifo_in fifo_rd fifo_req global_refetch global_reset ic_out rd  out: fifo_in fifo_rd fifo_req
//   CONTRACT check_group  in: carry_low checker_out fifo_head fifo_read_enable grp_in_range grp_ptab grp_seg_first lh_new_pc multi pd_next predecode_flush rvc_n rvc_straddle  out: checker_out flush_address lh_new_data lh_new_pc lh_new_valid lh_updated pd_next predecode_flush
//   CONTRACT write_front2back  in: checker_out f2b_bypass f2b_head f2b_in f2b_rd f2b_read_enable fifo_head fifo_read_enable front2back_fifo_req global_reset grp_ptab rd use_f2b_bypass  out: f2b_bypass f2b_in f2b_rd front2back_fifo_req use_f2b_bypass
//   CONTRACT invalidate_icache  in: ic_k predecode_flush  out: ic_k
//   CONTRACT finish  in: bpu_output bpu_pipe.txn f2b_bypass f2b_head f2b_rd fetch_addr_head fetch_addr_rd fifo_rd flush_address global_refetch global_reset ic_k ic_out in lh_new_data lh_new_pc lh_new_valid lh_updated out pd_next pipe_dec predecode_flush ptab_rd rd use_f2b_bypass  out: out
class FrontCore {
public:
  FrontCore();
  ~FrontCore();
  SimContext *ctx = nullptr;
  bpuf::BpuPipe bpu_pipe;              // the pipelined predictor (its registers / arrays and its transaction of the cycle)
  FetchAddrFifo fetch_addr_fifo;
  FrontFifo<InstFifo> inst_fifo;
  FrontFifo<Ptab> ptab;
  FrontFifo<F2bFifo> f2b_fifo;
  ICacheTop icache;
  FrontChecker checker;   // the predecode check
  FrontControl ctrl;      // the front end's control (front_ctrl.h)
  FetchAddrPort fa_port;  // the FIFOs' ports (front_fifo_ports.h)
  InstPort inst_port;
  PtabPort ptab_port;
  F2bPort f2b_port;
  FrontIpf ipf;

  void read_q(FrontReadData &rd) const;                                    // q and the FIFOs' read data
  void bind(const FrontTopIn *in, const FrontReadData *rd, FrontTopOut *out, SimContext *ctx);   // FrontTop::init
  void comb();   // all phases (init / step_bpu)
  // one cycle phase by phase (SimCpu::cycle() schedules them): each calls one module
  void control();
  void run_bpu();
  void enables();
  void read_fetch_addr();
  void read_ptab();
  void read_instruction();
  void read_f2b();
  void write_fetch_addr();
  void run_icache();
  void write_instruction_fifo();
  void check_group();
  void write_front2back();
  void invalidate_icache();
  void finish();

  void seq(const FrontTopIn &in);   // the clock edge

private:
  FrontRegs q_;   // the front end's own registers (q; d is ctrl.out.front_state.d)
};

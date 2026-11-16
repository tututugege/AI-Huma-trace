#pragma once
// All state of the front end, owned by FrontTop (one instance per FrontTop): its own registers, the BPU pipeline, the four
// FIFOs, the I-cache, the I-cache prefetcher. One cycle is read_q (the registers' q, read data of the FIFOs), comb (the
// phases of FrontComb, front_top.cpp; the write requests into a FrontUpdateRequest) and seq (the clock edge).
#include "BPU/BPU_pipe.h"
#include "env.h"
#include "fifo/front_fifo.h"
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

struct FrontCombSlot;   // the cycle's working state of the comb phases (FrontComb, front_top.cpp)

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
  FrontIpf ipf;

  void read_q(FrontReadData &rd) const;                                    // q and the FIFOs' read data
  void comb(const FrontTopIn &in, const FrontReadData &rd, FrontTopOut &out, FrontUpdateRequest &req);   // all phases (init)
  // one cycle phase by phase (SimCpu::cycle() schedules them): the working state, then the phases (their CONTRACT lines)
  void comb_begin(const FrontTopIn &in, const FrontReadData &rd, FrontTopOut &out, FrontUpdateRequest &req);
  void control();
  void run_bpu();
  void read_fifos();
  void write_fetch_addr();
  void run_icache();
  void write_instruction_fifo();
  void check_group();
  void write_front2back();
  void finish();

  void seq(const FrontTopIn &in, const FrontUpdateRequest &req);   // the clock edge

private:
  FrontRegs q_;   // the front end's own registers (q; d is req.front_state.d)
  std::unique_ptr<FrontCombSlot> comb_;   // the cycle's FrontComb (made anew by comb_begin)
};

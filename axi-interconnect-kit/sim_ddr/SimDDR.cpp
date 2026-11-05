#include "SimDDR.h"
#ifdef SIM_DDR_DATA
#include "PhysMemory.h"
#endif
#include <algorithm>

#ifdef DDR_SHADOW
#include "DdrShadow.h"
#define SHADOW_BEGIN() ddr_shadow_begin(this, &io)
#define SHADOW_END(phase) ddr_shadow_end(this, &io, phase)
#else
#define SHADOW_BEGIN() ((void)0)
#define SHADOW_END(phase) ((void)0)
#endif

#ifdef SIM_DDR_DS3
#include "SimDDR_DS3.h"
#endif

namespace sim_ddr {

#ifdef SIM_DDR_DATA
// functional data of the RTL closed-loop platform (same behaviour as the original SimDDR::do_memory_read / do_memory_write)
void SimDDR::do_memory_write(uint32_t addr, axi_data_t data, axi_strb_t wstrb) {
  if (pmem_ram_ptr() == nullptr) return;
  for (uint8_t byte = 0; byte < AXI_DATA_BYTES; ++byte) {
    if (!axi_compat::test_bit(wstrb, byte)) continue;
    const uint32_t byte_addr = addr + byte;
    const uint32_t word_addr = byte_addr & ~0x3u;
    const uint32_t shift = (byte_addr & 0x3u) * 8u;
    const uint32_t mask = 0xFFu << shift;
    const uint32_t nb = uint32_t(axi_compat::get_byte(data, byte));
    pmem_write(word_addr, (pmem_read(word_addr) & ~mask) | (nb << shift));
  }
}

axi_data_t SimDDR::do_memory_read(uint32_t addr) {
  axi_data_t data{};
  if (pmem_ram_ptr() == nullptr) { data = 0xDEADBEEF; return data; }
  for (uint8_t byte = 0; byte < AXI_DATA_BYTES; ++byte) {
    const uint32_t byte_addr = addr + byte;
    const uint32_t shift = (byte_addr & 0x3u) * 8u;
    axi_compat::set_byte(data, byte, static_cast<uint8_t>((pmem_read(byte_addr & ~0x3u) >> shift) & 0xFFu));
  }
  return data;
}
#endif

#ifdef SIM_DDR_DS3
struct SimDDR::Ds3Holder {
  Ds3 d;
  Ds3Holder() : d(CONFIG_CPU_FREQ_MHZ, CONFIG_AXI_SUBSYSTEM_FREQ_DIV, CONFIG_DDR_SOC_LATENCY_NS + CONFIG_DDR_CDC_LATENCY_NS) {}
};
SimDDR::SimDDR() : ds3_(new Ds3Holder) {}
SimDDR::~SimDDR() = default;
#endif

void SimDDR::init() {
  pend_.clear();
#ifdef SIM_DDR_DATA
  wbeats_.clear();
#endif
  resp_.clear();
  rd_.clear();
  fifo_ = accept_cd_ = drain_cd_ = turn_cd_ = 0;
  drain_mode_ = read_grant_ = write_grant_ = switch_pending_ = any_pending_ = false;
  rr_ = 0;
  sel_ = active_ = -1;
  last_ = switch_target_ = None;
  io.aw.awready = io.w.wready = io.b.bvalid = io.ar.arready = io.r.rvalid = io.r.rlast = false;
  io.b.bid = io.r.rid = 0;
  io.b.bresp = io.r.rresp = AXI_RESP_OKAY;
  io.r.rdata = 0;
}

// Which direction owns the backend this cycle. Switching costs a turnaround, during which nobody is granted.
void SimDDR::arbitrate() {
  read_grant_ = write_grant_ = switch_pending_ = any_pending_ = false;
  switch_target_ = None;
#ifdef SIM_DDR_DS3
  // DRAMsim3 owns the DRAM bus: AXI read and write data phases do not exclude each other here
  read_grant_ = sel_ >= 0;
  write_grant_ = drain_mode_ && fifo_ != 0 && drain_cd_ == 0 && drain_target() >= 0;
  any_pending_ = read_grant_ || write_grant_;
  return;
#endif
  const bool read_ready = sel_ >= 0;
  const bool write_ready = drain_mode_ && fifo_ != 0 && drain_cd_ == 0 && drain_target() >= 0;
  any_pending_ = read_ready || write_ready;
  if (!any_pending_ || turn_cd_ != 0) return;

  Mode want = None;
  if (last_ == Write && write_ready && keep_drain_mode()) want = Write;
  else if (read_ready) want = Read;  // an active burst, a continuing read window and a fresh read all pick Read
  else if (write_ready) want = Write;
  if (want == None) return;

  if (last_ != None && want != last_ && turnaround(last_, want) != 0) {
    switch_pending_ = true;
    switch_target_ = want;
    return;
  }
  read_grant_ = want == Read;
  write_grant_ = want == Write;
}

void SimDDR::comb_outputs() {
  SHADOW_BEGIN();
  // burst-drain read service: finish the active burst, else round-robin among transactions past their latency
  sel_ = -1;
  if (active_ >= 0 && active_ < int(rd_.size()) && rd_[active_].in_data && !rd_[active_].complete) sel_ = active_;
  for (size_t i = 0; sel_ < 0 && i < rd_.size(); i++) {
    const size_t idx = (rr_ + i) % rd_.size();
    if (rd_[idx].in_data && !rd_[idx].complete) sel_ = int(idx);
  }
  arbitrate();

  io.ar.arready = rd_.size() < SIM_DDR_MAX_OUTSTANDING;
  io.r.rvalid = io.r.rlast = false;
  io.r.rid = 0;
  io.r.rdata = axi_data_t{};
  io.r.rresp = AXI_RESP_OKAY;
  if (read_grant_ && sel_ >= 0) {
    const RdTxn &t = rd_[sel_];
    io.r.rvalid = true;
    io.r.rid = t.id;
#ifdef SIM_DDR_DATA
    io.r.rdata = do_memory_read(t.addr + uint32_t(t.beat) * AXI_DATA_BYTES);
#endif
    io.r.rlast = t.beat == t.len;
  }

  io.aw.awready = pend_.size() < SIM_DDR_WRITE_QUEUE_DEPTH;
  io.w.wready = data_target() >= 0 && fifo_ < SIM_DDR_WRITE_DATA_FIFO_DEPTH && accept_cd_ == 0 && !drain_mode_;
  io.b.bvalid = false;
  io.b.bid = 0;
  io.b.bresp = AXI_RESP_OKAY;
  if (!resp_.empty() && (SIM_DDR_WRITE_RESP_LATENCY == 0 || resp_.front().lat >= SIM_DDR_WRITE_RESP_LATENCY)) {
    io.b.bvalid = true;
    io.b.bid = resp_.front().id;
  }
  SHADOW_END(0);
}

void SimDDR::retire_writes() {
  while (!pend_.empty() && pend_.front().data_done && pend_.front().drained == pend_.front().beats) {
#ifdef SIM_DDR_DS3
    ds3_->d.add_write(pend_.front().addr, uint32_t(pend_.front().beats) * AXI_DATA_BYTES, pend_.front().id);
#else
    resp_.push_back({pend_.front().id, 0});
#endif
    pend_.pop_front();
  }
}

#ifdef SIM_DDR_DS3
// DRAM time follows SimDDR::seq() ticks; also called for the ticks the simulator skips while the AXI subsystem is quiescent
void SimDDR::ds3_advance(uint32_t n) {
  std::vector<uint8_t> wdone;
  for (uint32_t i = 0; i < n; i++) {
    ds3_->d.tick();
    wdone.clear();
    ds3_->d.pump(wdone);
    for (uint8_t id : wdone) resp_.push_back({id, 0});
  }
}
#endif

void SimDDR::seq() {
  SHADOW_BEGIN();
#ifdef SIM_DDR_DS3
  ds3_advance(1);
#endif
  if (turn_cd_ > 0) turn_cd_--;
  if (accept_cd_ > 0) accept_cd_--;
  if (drain_cd_ > 0) drain_cd_--;

  // responses age one cycle; the one just accepted leaves first, a response enqueued below starts at 0
  if (io.b.bvalid && io.b.bready) resp_.pop_front();
  for (Resp &r : resp_) r.lat++;

  if (io.aw.awvalid && io.aw.awready)
    pend_.push_back({uint16_t(uint16_t(io.aw.awlen) + 1u), 0, 0, uint8_t(io.aw.awid), false, uint32_t(io.aw.awaddr)});

  if (io.w.wvalid && io.w.wready) {
    const int t = data_target();
    if (t >= 0) {
      fifo_++;
#ifdef SIM_DDR_DATA
      wbeats_.push_back({pend_[t].addr + uint32_t(pend_[t].accepted) * AXI_DATA_BYTES, io.w.wdata, io.w.wstrb});
#endif
      pend_[t].accepted++;
      accept_cd_ = SIM_DDR_WRITE_ACCEPT_GAP;
      if (io.w.wlast) pend_[t].data_done = true;
    }
  }
  if (!drain_mode_ && enter_drain_mode()) drain_mode_ = true;
  // one buffered beat leaves per cycle while draining
  if (write_grant_ && drain_mode_ && fifo_ != 0 && drain_cd_ == 0) {
    const int t = drain_target();
    if (t >= 0) {
      fifo_--;
#ifdef SIM_DDR_DATA
      do_memory_write(wbeats_.front().addr, wbeats_.front().data, wbeats_.front().strb);
      wbeats_.pop_front();
#endif
      pend_[t].drained++;
      drain_cd_ = SIM_DDR_WRITE_DRAIN_GAP;
    }
  }
  retire_writes();
  if (drain_mode_ && !keep_drain_mode()) drain_mode_ = false;

  if (io.ar.arvalid && io.ar.arready) {
#ifdef SIM_DDR_DS3
    rd_.push_back({0, uint8_t(io.ar.arid), uint8_t(io.ar.arlen), 0, false, false,
                   ds3_->d.add_read(uint32_t(io.ar.araddr), (uint32_t(io.ar.arlen) + 1u) * AXI_DATA_BYTES)});
#else
    rd_.push_back({0, uint8_t(io.ar.arid), uint8_t(io.ar.arlen), 0, false, false});
#endif
    rd_.back().addr = uint32_t(io.ar.araddr);
  }
  if (read_grant_ && io.r.rvalid && io.r.rready && sel_ >= 0) {
    RdTxn &t = rd_[sel_];
    t.beat++;
    if (io.r.rlast) {
      t.complete = true;
      active_ = -1;
      rr_ = (size_t(sel_) + 1) % std::max<size_t>(1, rd_.size());
    } else {
      active_ = sel_;
    }
  }
#ifdef SIM_DDR_DS3
  for (RdTxn &t : rd_)
    if (!t.in_data && !t.complete && ds3_->d.read_ready(t.tag)) { t.in_data = true; ds3_->d.read_finish(t.tag); }
#else
  for (RdTxn &t : rd_)
    if (!t.in_data && !t.complete && ++t.lat >= SIM_DDR_LATENCY) t.in_data = true;
#endif

  // drop completed reads, keeping the active index pointing at the same transaction
  size_t dst = 0;
  for (size_t src = 0; src < rd_.size(); src++) {
    if (rd_[src].complete) {
#ifdef SIM_DDR_DS3
      ds3_->d.read_retire(rd_[src].tag);
#endif
      if (active_ >= 0 && int(src) < active_) active_--;
      continue;
    }
    if (dst != src) rd_[dst] = rd_[src];
    dst++;
  }
  rd_.resize(dst);
  if (!rd_.empty() && rr_ >= rd_.size()) rr_ = 0;
  if (active_ >= int(rd_.size())) active_ = -1;

  if (!any_pending_ && turn_cd_ == 0) {
    last_ = None;
  } else if (switch_pending_) {
    turn_cd_ = turnaround(last_, switch_target_);
    last_ = None;
  } else if (read_grant_) {
    last_ = Read;
  } else if (write_grant_) {
    last_ = Write;
  }
  SHADOW_END(1);
}

} // namespace sim_ddr

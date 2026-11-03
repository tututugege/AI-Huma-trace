// Canonical state digest of the compact SimDDR (the frozen reference exposes the same value through SimDDRRef::digest()).
#pragma once
#include "SimDDR.h"
namespace sim_ddr {
struct DdrShadowAccess {
  static uint64_t digest(const SimDDR &d) {
    uint64_t h = 1469598103934665603ull;
    auto m = [&](uint64_t v) { h = (h ^ (v + 0x9e3779b97f4a7c15ull)) * 1099511628211ull; };
    m(d.pend_.size());
    for (auto &t : d.pend_) { m(t.id); m(t.beats - 1); m(t.accepted); m(t.drained); m(t.data_done); }
    m(d.fifo_); m(d.accept_cd_); m(d.drain_cd_); m(d.drain_mode_);
    m(d.resp_.size());
    for (auto &r : d.resp_) { m(r.id); m(r.lat); }
    m(d.rd_.size());
    for (auto &t : d.rd_) { m(t.id); m(t.len); m(t.beat); m(t.lat); m(t.in_data); m(t.complete); }
    m(d.rr_); m(uint64_t(int64_t(d.active_)));
    m(uint64_t(d.last_)); m(d.turn_cd_);
    return h;
  }
};
}  // namespace sim_ddr

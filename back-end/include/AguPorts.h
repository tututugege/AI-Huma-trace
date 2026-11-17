#pragma once
// Shared AGUs (CFG_AGU_SHARED = NM, docs/TLB_PORTS.md): the load and the store-address pipes keep their lanes (LSU_LDU_COUNT,
// LSU_STA_COUNT) but the address generation and the DTLB have LSU_AGU_PHYS physical ports. Ports 0 .. NL-1 serve only loads,
// NL .. NL+NM-1 both (a load wins), NL+NM .. only store addresses. The load queue's issue port k is the physical port k; the STA
// queue's ports are the store-only ones first, then the shared ones. NM = 0: every pipe has its own port (the ports are the lanes).
#include "config.h"

namespace agu {
constexpr int NM = LSU_AGU_SHARED;
constexpr int NL = LSU_LDU_COUNT - NM;   // load-only ports
constexpr int NS = LSU_STA_COUNT - NM;   // store-only ports
constexpr int PORTS = LSU_AGU_PHYS;

constexpr int ld_port(int k) { return k; }
constexpr int st_port(int k) { return k < NS ? LSU_LDU_COUNT + k : NL + (k - NS); }
constexpr bool ld_can(int p) { return p < LSU_LDU_COUNT; }   // a load may use the port
constexpr bool st_can(int p) { return p >= NL; }             // a store address may use the port
}  // namespace agu

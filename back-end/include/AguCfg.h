#pragma once
// EXP-TLBPORTS: shape of the load / store address pipes. AGU_CFG="nL,nS,nM": nL AGUs only for loads, nS only for stores, nM shared
// (loads win). Per cycle: loads <= nL+nM, stores <= nS+nM, loads+stores <= nL+nS+nM. AGU_TOT=N is "0,0,N". Unset: every port
// has its own AGU (no limit). The same shape limits the issue (Isu::grant_gate) and the requests reaching the DTLB (new + replay).
#include <cstdio>
#include <cstdlib>
struct AguCfg {
  bool on = false;
  int nl = 0, ns = 0, nm = 0;
  int max_ld() const { return nl + nm; }
  int max_st() const { return ns + nm; }
  int max_tot() const { return nl + ns + nm; }
  // may one more load / store be granted given the counts so far
  bool ok_ld(int ld, int st) const { return !on || (ld < max_ld() && ld + st < max_tot()); }
  bool ok_st(int ld, int st) const { return !on || (st < max_st() && ld + st < max_tot()); }
  // AGU_PORTS="nL,nS,nM": the same shape as physical AGU ports (the port-accurate form): nL+nM+nS ports, 0..nL-1 load only,
  // nL..nL+nM-1 shared, nL+nM.. store only. The load queue's ports k = 0..nL+nM-1 are the physical ports k; the STA queue's
  // ports are the store-only ones first, then the shared ones (physical nL+nM+k for k < nS, nL+(k-nS) after that). A shared
  // port granted to a load is not available to a STA (the load queue is scheduled first); a replay (load / STA) that needs the
  // DTLB takes a free port it may use.
  bool phys = false;
  int ld_phys(int k) const { return k; }
  int st_phys(int k) const { return k < ns ? nl + nm + k : nl + (k - ns); }
  bool ld_can(int p) const { return p < nl + nm; }
  bool st_can(int p) const { return p >= nl; }
  static const AguCfg &get() {
    static const AguCfg c = [] {
      AguCfg r;
      if (const char *s = getenv("AGU_PORTS")) {
        if (std::sscanf(s, "%d,%d,%d", &r.nl, &r.ns, &r.nm) == 3) r.phys = true;
      } else if (const char *s = getenv("AGU_CFG")) {
        if (std::sscanf(s, "%d,%d,%d", &r.nl, &r.ns, &r.nm) == 3) r.on = true;
      } else if (const char *t = getenv("AGU_TOT")) {
        r.nm = std::atoi(t);
        r.on = true;
      }
      return r;
    }();
    return c;
  }
};

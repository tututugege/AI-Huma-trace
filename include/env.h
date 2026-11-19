#pragma once
// Every environment variable the simulator reads (docs/ENV_VARS.md), read once, at the first use of sim_env().
// (DS3_OUT is read by the AXI kit's DRAMsim3 wrapper, axi-interconnect-kit/sim_ddr/include/SimDDR_DS3.h.)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

struct SimEnv {
  // driver
  bool sim_max_cycles_set = false;   // SIM_MAX_CYCLES: stop after this many cycles (exit code 3)
  long long sim_max_cycles = 0;
  bool fast_warmup = false;          // FAST_WARMUP=1: fast warm-up of a checkpoint run
  // PMU / TMA
  bool pmu = true;                   // PMU (default on since 2026-11-19; "0" turns it off): statistics, pmu_*.csv at the end
  std::string pmu_out = "pmu";       // PMU_OUT: output file prefix
  unsigned long long pmu_interval = 0;   // PMU_INTERVAL: a time-series row every n cycles
  bool pmu_pcprof = true;            // PMU_PCPROF=0: no per-PC profile
  // front end (experimental)
  int fe_pair = 0;                   // FE_PAIR: two fetch blocks per cycle
  int fe_ovr_oracle = 0;             // FE_OVR_ORACLE=1: the S0 guess is the main predictor's result (no S1 / S_D override bubbles; upper-bound study)
  int fe_ipf = 0;                    // FE_IPF: I-cache prefetch
  int fe_ipf_lat = 12;               // FE_IPF_LAT: its latency (cycles)
  // L1D prefetcher
  const char *pf_enable = nullptr;   // PF_ENABLE (raw; the prefetcher has the default)
  // RTL comparison and alignment prints
  bool lsu_pipe_evt = false;         // LSU_PIPE_EVT: per-cycle LSU boundary events (E t=..)
  const char *lsu_pipe_stq = nullptr;   // LSU_PIPE_STQ=<idx>: one STQ entry's state with each event line
  bool fpr = false;                  // FPR=<bin>: BPU alignment fingerprint (FPR PT/UP)
  unsigned fpr_bin = 0;
  bool fpr_dump = false;             // FPR_DUMP=<from>:<to>: the events of these cycles (FPRRAW)
  unsigned fpr_from = 1, fpr_to = 0;
  bool tb_trace_commit = false;      // TB_TRACE_COMMIT: [COMMIT] per committed instruction

  SimEnv() {
    if (const char *v = std::getenv("SIM_MAX_CYCLES")) { sim_max_cycles_set = true; sim_max_cycles = std::atoll(v); }
    if (const char *v = std::getenv("FAST_WARMUP")) fast_warmup = v[0] == '1';
    if (const char *v = std::getenv("PMU")) pmu = std::strcmp(v, "0") != 0;
    if (const char *v = std::getenv("PMU_OUT")) pmu_out = v;
    if (const char *v = std::getenv("PMU_INTERVAL")) pmu_interval = std::strtoull(v, nullptr, 0);
    if (const char *v = std::getenv("PMU_PCPROF")) pmu_pcprof = std::strcmp(v, "0") != 0;
    if (const char *v = std::getenv("FE_PAIR")) fe_pair = std::atoi(v);
    if (const char *v = std::getenv("FE_OVR_ORACLE")) fe_ovr_oracle = std::atoi(v);
    if (const char *v = std::getenv("FE_IPF")) fe_ipf = std::atoi(v);
    if (const char *v = std::getenv("FE_IPF_LAT")) fe_ipf_lat = std::atoi(v);
    pf_enable = std::getenv("PF_ENABLE");
    lsu_pipe_evt = std::getenv("LSU_PIPE_EVT") != nullptr;
    lsu_pipe_stq = std::getenv("LSU_PIPE_STQ");
    if (const char *v = std::getenv("FPR")) { fpr = std::atoi(v) > 0; if (fpr) fpr_bin = static_cast<unsigned>(std::atoi(v)); }
    if (const char *v = std::getenv("FPR_DUMP")) { unsigned f = 0, t = 0; if (std::sscanf(v, "%u:%u", &f, &t) == 2) { fpr_from = f; fpr_to = t; fpr_dump = true; } }
    tb_trace_commit = std::getenv("TB_TRACE_COMMIT") != nullptr;
  }
};

inline const SimEnv &sim_env() { static const SimEnv e; return e; }

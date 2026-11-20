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
  int fe_ipf = 0;                    // FE_IPF: I-cache prefetch
  int fe_ipf_lat = 12;               // FE_IPF_LAT: its latency (cycles)
  // Block-based FTB BPU (exp/ftb-bpu, Phase 1; default off -> cycle-identical to main)
  int bpu_ftb = 0;                   // BPU_FTB=1: enable block-based FTB + S1 fast pred + S2 2-slot TAGE/ITT
  int ftb_s0_n = 256;                // FTB_S0_N: S0 uFTB entries (4-way)
  int ftb_s0_tgtb = 20;              // FTB_S0_TGTB: S0 uFTB target compression bits (0 = full 64b)
  int ftb_s1_n = 2048;               // FTB_S1_N: S1 Main FTB entries
  int ftb_s1_w = 4;                  // FTB_S1_W: S1 Main FTB associativity
  int ftb_s1_dir = 2;                // FTB_S1_DIR: 0=bimodal, 1=2-table mini-TAGE, 2=4-table mini-TAGE, 3=S2-TAGE oracle
  int ftb_s1_dir_n = 256;            // FTB_S1_DIR_N: S1 mini-TAGE entries per table (x2 slots)
  int ftb_tage_n = 2048;             // FTB_TAGE_N: S2 Block-TAGE entries per table (x2 slots)
  int ftb_itt_n = 256;               // FTB_ITT_N: S2 Single Block-ITT entries per table (x3 tables)
  int ftb_ovr_fast = 0;              // FTB_OVR_FAST: same-cycle re-issue on override (bit 0 = S2, bit 1 = S1)
  int ftb_slots = 4;                 // FTB_SLOTS: max branch slots per FTB entry (2..4, default 4 for non-truncating 32B RVC blocks)
  int ftb_ahead = 0;                 // FTB_AHEAD: 0=off, 1=1-ahead (cycle t-1 SRAM), 2=2-ahead (cycle t-2 SRAM)
  int ftb_ah_only = 0;               // FTB_AH_ONLY: 0=S1+S2 verify, 1=S2 verify only (skip S1 on ah_hit), 2=pure ahead (no S1/S2 ovr on ah_hit)
  int ftb_ah_n = 4096;               // FTB_AH_N: ahead FTB total entries
  int ftb_ah_w = 8;                  // FTB_AH_W: ahead FTB associativity (4 or 8 ways to hold multi-step successors)
  int ftb_ah_tage_n = 4096;          // FTB_AH_TAGE_N: ahead Block-TAGE entries per table
  int ftb_brq = 0;                   // FTB_BRQ=1: RTL-style 1-branch/cycle br_q[16] + 1R1W ftb_rec_[512] (no block tracker FSM)
  int ftb_no_aitt = 0;               // FTB_NO_AITT=1: omit ahead ITT (S2 ITT still predicts indirect targets)
  bool ftb_stats = false;            // FTB_STATS=1: print FTB hit/override/eviction stats on exit
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
    if (const char *v = std::getenv("FE_IPF")) fe_ipf = std::atoi(v);
    if (const char *v = std::getenv("FE_IPF_LAT")) fe_ipf_lat = std::atoi(v);
    if (const char *v = std::getenv("BPU_FTB")) bpu_ftb = std::atoi(v);
    if (const char *v = std::getenv("FTB_S0_N")) ftb_s0_n = std::atoi(v);
    if (const char *v = std::getenv("FTB_S0_TGTB")) ftb_s0_tgtb = std::atoi(v);
    if (const char *v = std::getenv("FTB_S1_N")) ftb_s1_n = std::atoi(v);
    if (const char *v = std::getenv("FTB_S1_W")) ftb_s1_w = std::atoi(v);
    if (const char *v = std::getenv("FTB_S1_DIR")) ftb_s1_dir = std::atoi(v);
    if (const char *v = std::getenv("FTB_S1_DIR_N")) ftb_s1_dir_n = std::atoi(v);
    if (const char *v = std::getenv("FTB_TAGE_N")) ftb_tage_n = std::atoi(v);
    if (const char *v = std::getenv("FTB_ITT_N")) ftb_itt_n = std::atoi(v);
    if (const char *v = std::getenv("FTB_OVR_FAST")) ftb_ovr_fast = std::atoi(v);
    if (const char *v = std::getenv("FTB_SLOTS")) ftb_slots = std::atoi(v);
    if (const char *v = std::getenv("FTB_AHEAD")) ftb_ahead = std::atoi(v);
    if (const char *v = std::getenv("FTB_AH_ONLY")) ftb_ah_only = std::atoi(v);
    if (const char *v = std::getenv("FTB_AH_N")) ftb_ah_n = std::atoi(v);
    if (const char *v = std::getenv("FTB_AH_W")) ftb_ah_w = std::atoi(v);
    if (const char *v = std::getenv("FTB_AH_TAGE_N")) ftb_ah_tage_n = std::atoi(v);
    if (const char *v = std::getenv("FTB_BRQ")) ftb_brq = std::atoi(v);
    if (const char *v = std::getenv("FTB_NO_AITT")) ftb_no_aitt = std::atoi(v);
    if (const char *v = std::getenv("FTB_STATS")) ftb_stats = v[0] == '1';
    pf_enable = std::getenv("PF_ENABLE");
    lsu_pipe_evt = std::getenv("LSU_PIPE_EVT") != nullptr;
    lsu_pipe_stq = std::getenv("LSU_PIPE_STQ");
    if (const char *v = std::getenv("FPR")) { fpr = std::atoi(v) > 0; if (fpr) fpr_bin = static_cast<unsigned>(std::atoi(v)); }
    if (const char *v = std::getenv("FPR_DUMP")) { unsigned f = 0, t = 0; if (std::sscanf(v, "%u:%u", &f, &t) == 2) { fpr_from = f; fpr_to = t; fpr_dump = true; } }
    tb_trace_commit = std::getenv("TB_TRACE_COMMIT") != nullptr;
  }
};

inline const SimEnv &sim_env() { static const SimEnv e; return e; }

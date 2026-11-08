#include "Pmu.h"
#include "config.h"
#include <algorithm>
#include <cstring>

namespace {
const char *const kName[] = {
#define PMU_EV(NAME, GROUP, WIDTH, DESC) #NAME,
#include "pmu_events.def"
#undef PMU_EV
};
const char *const kGroup[] = {
#define PMU_EV(NAME, GROUP, WIDTH, DESC) #GROUP,
#include "pmu_events.def"
#undef PMU_EV
};
const char *const kDesc[] = {
#define PMU_EV(NAME, GROUP, WIDTH, DESC) DESC,
#include "pmu_events.def"
#undef PMU_EV
};
constexpr int kMaxCommit = COMMIT_WIDTH;
constexpr int kOccBins = 17;  // histogram bins: occupancy in sixteenths of capacity (+ full)

std::string getenv_str(const char *n, const char *dflt) {
  const char *v = std::getenv(n);
  return v ? v : dflt;
}
}  // namespace

const char *Pmu::name(int e) { return kName[e]; }
const char *Pmu::group(int e) { return kGroup[e]; }
const char *Pmu::desc(int e) { return kDesc[e]; }

void Pmu::init_from_env() {
  const char *on = std::getenv("PMU");
  enabled = on && std::strcmp(on, "0") != 0;
  if (!enabled) return;
  out_prefix = getenv_str("PMU_OUT", "pmu");
  const char *iv = std::getenv("PMU_INTERVAL");
  interval = iv ? std::strtoull(iv, nullptr, 0) : 0;
  const char *pp = std::getenv("PMU_PCPROF");
  pcprof = !(pp && std::strcmp(pp, "0") == 0);
  reset();
}

void Pmu::reset() {
  std::memset(total, 0, sizeof(total));
  cur = PmuVec{};
  recovering = false;
  pc_stat.clear();
  h_commit.assign(kMaxCommit + 1, 0);
  h_rob.assign(kOccBins, 0);
  h_ldq.assign(kOccBins, 0);
  h_stq.assign(kOccBins, 0);
  h_mlp.assign(LDQ_SIZE + 1, 0);
  identity_violations = 0;
  ncycles = 0;
  std::memset(series_last, 0, sizeof(series_last));
  series_next = interval;
  if (series) { std::fclose(series); series = nullptr; }
}

void Pmu::accumulate() {
  for (int e = 0; e < PMU_NUM; e++) total[e] += cur.v[e];
  ncycles++;
  // slot identity: every slot of the cycle is in exactly one bucket
  const uint64_t slots = cur.v[PMU_SLOT_FIRE] + cur.v[PMU_SLOT_KILLED] + cur.v[PMU_SLOT_FE_EMPTY] +
                         cur.v[PMU_SLOT_BE_ROB_FULL] + cur.v[PMU_SLOT_BE_FLUSH_WAIT] + cur.v[PMU_SLOT_BE_SERIAL] +
                         cur.v[PMU_SLOT_BE_LDQ_FULL] + cur.v[PMU_SLOT_BE_STQ_FULL] + cur.v[PMU_SLOT_BE_IQ_FULL];
  if (slots != DECODE_WIDTH) identity_violations++;
  if (interval && ncycles >= series_next) {
    if (!series) {
      series = std::fopen((out_prefix + "_series.csv").c_str(), "w");
      if (series) {
        std::fprintf(series, "cycle");
        for (int e = 0; e < PMU_NUM; e++) std::fprintf(series, ",%s", kName[e]);
        std::fprintf(series, "\n");
      }
    }
    if (series) {
      std::fprintf(series, "%llu", (unsigned long long)ncycles);
      for (int e = 0; e < PMU_NUM; e++) {
        std::fprintf(series, ",%llu", (unsigned long long)(total[e] - series_last[e]));
        series_last[e] = total[e];
      }
      std::fprintf(series, "\n");
    }
    series_next += interval;
  }
}

void Pmu::close() {
  if (series) { std::fclose(series); series = nullptr; }
}

void Pmu::report(uint64_t instructions) {
  if (!enabled) return;
  const double cyc = total[PMU_CYCLES] ? (double)total[PMU_CYCLES] : 1.0;
  const double slots = cyc * DECODE_WIDTH;
  auto T = [&](PmuEvent e) { return (double)total[e]; };
  auto pct = [&](double x) { return 100.0 * x / slots; };
  const char *G = "\033[38;5;33m", *Z = "\033[0m";

  std::printf("%s========================= PMU (EXP-013) =========================%s\n", G, Z);
  std::printf("%-24s %16s %10s  %s\n", "event", "count", "per-cycle", "meaning");
  const char *last_group = "";
  for (int e = 0; e < PMU_NUM; e++) {
    if (std::strcmp(last_group, kGroup[e]) != 0) { std::printf("-- %s\n", kGroup[e]); last_group = kGroup[e]; }
    std::printf("%-24s %16llu %10.4f  %s\n", kName[e], (unsigned long long)total[e], total[e] / cyc, kDesc[e]);
  }
  std::printf("slot identity violations: %llu (must be 0)\n", (unsigned long long)identity_violations);
  std::printf("INSTRET %llu vs committed %llu (equal; a checkpoint run stops mid-cycle, up to %d more may be counted here)\n",
              (unsigned long long)total[PMU_INSTRET], (unsigned long long)instructions, COMMIT_WIDTH - 1);

  // ---- top-down tree ----
  const double fire = T(PMU_SLOT_FIRE), retired = T(PMU_INSTRET);
  const double wrong_path = std::max(0.0, fire - retired);  // dispatched, never committed: flushed later
  const double recov = T(PMU_SLOT_FE_RECOV_MISPRED) + T(PMU_SLOT_FE_RECOV_FLUSH);
  const double killed = T(PMU_SLOT_KILLED);
  const double bad_spec = wrong_path + recov + killed;
  const double fe = T(PMU_SLOT_FE_EMPTY) - recov;
  const double be = T(PMU_SLOT_BE_ROB_FULL) + T(PMU_SLOT_BE_FLUSH_WAIT) + T(PMU_SLOT_BE_SERIAL) + T(PMU_SLOT_BE_LDQ_FULL) +
                    T(PMU_SLOT_BE_STQ_FULL) + T(PMU_SLOT_BE_IQ_FULL);
  std::printf("%s*********Top-Down Analysis (PMU counters only; %d slots per cycle)*********%s\n", G, DECODE_WIDTH, Z);
  std::printf("Total Slots      : %.0f\n", slots);
  std::printf("Retiring         : %6.2f%%\n", pct(retired));
  std::printf("Bad Speculation  : %6.2f%%\n", pct(bad_spec));
  std::printf("  - Wrong-path work (dispatched, flushed): %6.2f%%\n", pct(wrong_path));
  std::printf("  - Mispredict recovery bubbles          : %6.2f%%\n", pct(T(PMU_SLOT_FE_RECOV_MISPRED)));
  std::printf("  - Flush recovery bubbles               : %6.2f%%\n", pct(T(PMU_SLOT_FE_RECOV_FLUSH)));
  std::printf("  - Killed in the dispatch stage         : %6.2f%%\n", pct(killed));
  std::printf("Frontend Bound   : %6.2f%%\n", pct(fe));
  std::printf("  - Fetch latency  (ICache %.2f%% + ITLB %.2f%%)\n", pct(T(PMU_SLOT_FE_ICACHE)), pct(T(PMU_SLOT_FE_ITLB)));
  std::printf("  - Fetch bandwidth: %6.2f%%\n", pct(T(PMU_SLOT_FE_OTHER)));
  std::printf("      of which rename no free preg: %6.2f%%  IDU no branch tag: %6.2f%%\n", pct(T(PMU_SLOT_FE_OTHER_NOPREG)), pct(T(PMU_SLOT_FE_OTHER_NOBRTAG)));
  std::printf("Backend Bound    : %6.2f%%\n", pct(be));
  const double mem = T(PMU_SLOT_BE_HEAD_MISS) + T(PMU_SLOT_BE_HEAD_MEM);
  std::printf("  by ROB head  : Memory %.2f%% (L1D miss %.2f%%, L1D hit/other %.2f%%)  Core %.2f%% (exec %.2f%%, head done %.2f%%)\n",
              pct(mem), pct(T(PMU_SLOT_BE_HEAD_MISS)), pct(T(PMU_SLOT_BE_HEAD_MEM)),
              pct(T(PMU_SLOT_BE_HEAD_CORE) + T(PMU_SLOT_BE_HEAD_DONE)), pct(T(PMU_SLOT_BE_HEAD_CORE)), pct(T(PMU_SLOT_BE_HEAD_DONE)));
  std::printf("  by resource  : ROB %.2f%%  IQ %.2f%%  LDQ %.2f%%  STQ %.2f%%  flush-wait %.2f%%  serial %.2f%%\n",
              pct(T(PMU_SLOT_BE_ROB_FULL)), pct(T(PMU_SLOT_BE_IQ_FULL)), pct(T(PMU_SLOT_BE_LDQ_FULL)), pct(T(PMU_SLOT_BE_STQ_FULL)),
              pct(T(PMU_SLOT_BE_FLUSH_WAIT)), pct(T(PMU_SLOT_BE_SERIAL)));

  // ---- derived metrics ----
  const double ki = std::max(1.0, T(PMU_INSTRET)) / 1000.0;
  std::printf("%s*********Derived*********%s\n", G, Z);
  std::printf("IPC %.4f | commit/cycle histogram:", T(PMU_INSTRET) / cyc);
  for (int i = 0; i <= kMaxCommit; i++) std::printf(" %d:%.1f%%", i, 100.0 * h_commit[i] / cyc);
  std::printf("\n");
  std::printf("MPKI  branch %.3f  jalr %.3f  ret %.3f | branch mispredict rate %.2f%% (of %llu)\n", T(PMU_COMMIT_MISPRED_BR) / ki,
              T(PMU_COMMIT_MISPRED_JALR) / ki, T(PMU_COMMIT_MISPRED_RET) / ki,
              T(PMU_COMMIT_BRANCH) ? 100.0 * T(PMU_COMMIT_MISPRED_BR) / T(PMU_COMMIT_BRANCH) : 0.0, (unsigned long long)total[PMU_COMMIT_BRANCH]);
  std::printf("MPKI  ICache %.3f  ITLB %.3f  L1D-load %.3f | L1D load miss rate %.2f%%\n", T(PMU_ICACHE_MISS) / ki, T(PMU_ITLB_MISS) / ki,
              T(PMU_DC_LD_MISS) / ki, (T(PMU_DC_LD_HIT) + T(PMU_DC_LD_MISS)) ? 100.0 * T(PMU_DC_LD_MISS) / (T(PMU_DC_LD_HIT) + T(PMU_DC_LD_MISS)) : 0.0);
  std::printf("MLP (avg outstanding L1D load misses while >= 1): %.2f | cycles with a miss outstanding %.1f%%\n",
              T(PMU_CYC_LD_MISS_OUT) ? T(PMU_DC_LD_MISS_OUT) / T(PMU_CYC_LD_MISS_OUT) : 0.0, 100.0 * T(PMU_CYC_LD_MISS_OUT) / cyc);
  std::printf("avg occupancy: ROB %.1f/%d  LDQ %.1f/%d  STQ %.1f/%d\n", T(PMU_OCC_ROB) / cyc, ROB_NUM, T(PMU_OCC_LDQ) / cyc, LDQ_SIZE,
              T(PMU_OCC_STQ) / cyc, STQ_SIZE);
  std::printf("prefetch: issued %llu useful %llu late %llu (accuracy %.1f%%)\n", (unsigned long long)total[PMU_PF_ISSUED],
              (unsigned long long)total[PMU_PF_USEFUL], (unsigned long long)total[PMU_PF_LATE],
              T(PMU_PF_ISSUED) ? 100.0 * T(PMU_PF_USEFUL) / T(PMU_PF_ISSUED) : 0.0);

  // ---- CSV ----
  if (FILE *f = std::fopen((out_prefix + "_events.csv").c_str(), "w")) {
    std::fprintf(f, "event,group,count\n");
    for (int e = 0; e < PMU_NUM; e++) std::fprintf(f, "%s,%s,%llu\n", kName[e], kGroup[e], (unsigned long long)total[e]);
    std::fclose(f);
  }
  if (FILE *f = std::fopen((out_prefix + "_hist.csv").c_str(), "w")) {
    std::fprintf(f, "hist,bin,cycles\n");
    for (size_t i = 0; i < h_commit.size(); i++) std::fprintf(f, "commit_per_cycle,%zu,%llu\n", i, (unsigned long long)h_commit[i]);
    for (size_t i = 0; i < h_rob.size(); i++) std::fprintf(f, "rob_occ_16ths,%zu,%llu\n", i, (unsigned long long)h_rob[i]);
    for (size_t i = 0; i < h_ldq.size(); i++) std::fprintf(f, "ldq_occ_16ths,%zu,%llu\n", i, (unsigned long long)h_ldq[i]);
    for (size_t i = 0; i < h_stq.size(); i++) std::fprintf(f, "stq_occ_16ths,%zu,%llu\n", i, (unsigned long long)h_stq[i]);
    for (size_t i = 0; i < h_mlp.size(); i++) std::fprintf(f, "outstanding_ld_miss,%zu,%llu\n", i, (unsigned long long)h_mlp[i]);
    std::fclose(f);
  }
  if (pcprof)
    if (FILE *f = std::fopen((out_prefix + "_pc.csv").c_str(), "w")) {
      std::vector<std::pair<uint32_t, PmuPcStat>> v(pc_stat.begin(), pc_stat.end());
      std::sort(v.begin(), v.end(), [](const auto &a, const auto &b) { return a.second.stall_slots > b.second.stall_slots; });
      std::fprintf(f, "pc,type,commits,head_cycles,head_miss_cycles,stall_slots,mispreds\n");
      for (const auto &kv : v)
        std::fprintf(f, "0x%08x,%d,%llu,%llu,%llu,%llu,%llu\n", kv.first, kv.second.type, (unsigned long long)kv.second.commits,
                     (unsigned long long)kv.second.head_cycles, (unsigned long long)kv.second.head_miss_cycles,
                     (unsigned long long)kv.second.stall_slots, (unsigned long long)kv.second.mispreds);
      std::fclose(f);
    }
  close();
  std::printf("PMU CSV: %s_{events,hist,pc%s}.csv\n", out_prefix.c_str(), interval ? ",series" : "");
}

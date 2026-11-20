// EXP-047 fast DSE (docs/DSE_FORK.md): one warm-up, then a fork per back-end configuration.
//   DSE_CONFIGS=<file>  one configuration per line: <name> key=value ... (the FIRST line is the baseline: the warm-up before
//                       the fork runs with its caps; keys: rob prf fprf br ldq stq mshr iq_int iq_mem
//                       iq_std iq_br iq_fp; a missing key keeps the compiled size; '#' starts a comment)
//   DSE_OUT=<dir>       each child runs in <dir>/<name>/ and writes its stdout / stderr to <dir>/<name>/sim.log (default ./dse)
//   DSE_SETTLE=<n>      the fork happens n committed instructions before the end of the detailed warm-up (default 200000):
//                       the children run those n instructions with their caps, then the measurement as usual
//   DSE_JOBS=<n>        children running at the same time (default: all)
//   DSE_FORK_AT=<n>     fork at this commit count instead (tests: zero-interval checkpoints have no warm-up)
// A child with the compiled sizes is cycle-identical to a plain run (the fork does not touch the simulation state).
#include "DseCap.h"
#include "DseFork.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {
struct DseCfg {
  std::string name;
  DseCaps caps;
};
std::vector<DseCfg> g_cfgs;
std::string g_out = "dse";
int g_jobs = 0;

[[noreturn]] void dse_die(const std::string &msg) {
  std::cerr << "[DSE] error: " << msg << std::endl;
  std::exit(2);
}

void set_cap(DseCaps &c, const std::string &key, int v, const std::string &line) {
  struct K { const char *name; int *p; int max, min; };
  const K keys[] = {
      {"rob", &c.rob, ROB_NUM, DECODE_WIDTH},          {"prf", &c.prf, PRF_NUM, ARF_NUM + 2},
      {"fprf", &c.fprf, FPRF_NUM, FP_ARF_NUM + 2},     {"br", &c.br, MAX_BR_NUM, MAX_BR_PER_CYCLE + 1},
      {"ldq", &c.ldq, LDQ_SIZE, 1},                    {"stq", &c.stq, STQ_SIZE, 1},
      {"mshr", &c.mshr, DCACHE_MSHR_ENTRIES, 1},       {"iq_int", &c.iq_int, CFG_INT_IQ_SIZE, 1},
      {"iq_mem", &c.iq_mem, CFG_MEM_IQ_SIZE, 1},       {"iq_std", &c.iq_std, CFG_STD_IQ_SIZE, 1},
      {"iq_br", &c.iq_br, CFG_BR_IQ_SIZE, 1},          {"iq_fp", &c.iq_fp, CFG_FP_IQ_SIZE, 1},
  };
  for (const K &k : keys)
    if (key == k.name) {
      if (v < k.min || v > k.max)
        dse_die(key + "=" + std::to_string(v) + " outside [" + std::to_string(k.min) + ", " + std::to_string(k.max) +
                "] (the maximum is the compiled size; build with a larger CFG_* to go higher): " + line);
      *k.p = v;
      return;
    }
  dse_die("unknown key '" + key + "': " + line);
}
}  // namespace

bool dse_init(uint64_t warmup_target) {
  const char *path = std::getenv("DSE_CONFIGS");
  if (!path || !*path) return false;
  std::ifstream f(path);
  if (!f) dse_die(std::string("cannot read DSE_CONFIGS=") + path);
  std::string line;
  while (std::getline(f, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line.resize(hash);
    std::istringstream is(line);
    DseCfg c;
    if (!(is >> c.name)) continue;
    std::string kv;
    while (is >> kv) {
      const size_t eq = kv.find('=');
      if (eq == std::string::npos) dse_die("expected key=value: " + line);
      set_cap(c.caps, kv.substr(0, eq), std::atoi(kv.c_str() + eq + 1), line);
    }
    for (const DseCfg &o : g_cfgs)
      if (o.name == c.name) dse_die("duplicate name " + c.name);
    g_cfgs.push_back(c);
  }
  if (g_cfgs.empty()) dse_die(std::string("no configuration in ") + path);
  if (const char *o = std::getenv("DSE_OUT")) g_out = o;
  const uint64_t settle = std::getenv("DSE_SETTLE") ? std::strtoull(std::getenv("DSE_SETTLE"), nullptr, 10) : 200000;
  if (settle > warmup_target)
    dse_die("DSE_SETTLE=" + std::to_string(settle) + " is larger than the warm-up (" + std::to_string(warmup_target) + ")");
  g_jobs = std::getenv("DSE_JOBS") ? std::atoi(std::getenv("DSE_JOBS")) : 0;
  if (g_jobs <= 0) g_jobs = static_cast<int>(g_cfgs.size());
  mkdir(g_out.c_str(), 0755);
  g_dse = g_cfgs[0].caps;  // the warm-up runs with the first configuration (the baseline): its child equals a plain run of it
  g_dse_active = true;
  g_dse_fork_at = warmup_target - settle;
  if (const char *fa = std::getenv("DSE_FORK_AT")) g_dse_fork_at = std::strtoull(fa, nullptr, 10);  // test override
  std::cout << "[DSE] " << g_cfgs.size() << " configurations, fork at commit " << g_dse_fork_at << " (settle " << settle
            << "), output " << g_out << "/<name>/sim.log, " << g_jobs << " at a time" << std::endl;
  return true;
}

int dse_fork() {
  g_dse_fork_pending = false;
  std::cout << "[DSE] forking" << std::endl;
  std::cout.flush();
  std::cerr.flush();
  std::fflush(nullptr);
  std::vector<pid_t> pids(g_cfgs.size(), -1);
  int running = 0, failed = 0;
  auto reap = [&]() {
    int st = 0;
    const pid_t p = wait(&st);
    if (p < 0) return;
    running--;
    for (size_t i = 0; i < pids.size(); i++)
      if (pids[i] == p) {
        const bool ok = WIFEXITED(st) && WEXITSTATUS(st) == 0;
        failed += !ok;
        std::cout << "[DSE] " << g_cfgs[i].name << (ok ? " done" : " FAILED") << " (status " << st << ")" << std::endl;
      }
  };
  for (size_t i = 0; i < g_cfgs.size(); i++) {
    while (running >= g_jobs) reap();
    const pid_t p = fork();
    if (p < 0) dse_die(std::string("fork: ") + std::strerror(errno));
    if (p == 0) {  // child: its caps, its own directory and log, then back to the simulation
      const std::string dir = g_out + "/" + g_cfgs[i].name;
      mkdir(dir.c_str(), 0755);
      if (chdir(dir.c_str()) != 0) dse_die("chdir " + dir);
      const int fd = open("sim.log", O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (fd < 0) dse_die("open " + dir + "/sim.log");
      dup2(fd, 1);
      dup2(fd, 2);
      close(fd);
      g_dse = g_cfgs[i].caps;
      g_dse_forked = true;
      const DseCaps &c = g_dse;
      std::cout << "[DSE] config " << g_cfgs[i].name << ": rob=" << c.rob << " prf=" << c.prf << " fprf=" << c.fprf
                << " br=" << c.br << " ldq=" << c.ldq << " stq=" << c.stq << " mshr=" << c.mshr << " iq_int=" << c.iq_int
                << " iq_mem=" << c.iq_mem << " iq_std=" << c.iq_std << " iq_br=" << c.iq_br << " iq_fp=" << c.iq_fp << std::endl;
      return -1;
    }
    pids[i] = p;
    running++;
  }
  while (running > 0) reap();
  std::cout << "[DSE] all " << g_cfgs.size() << " children finished, " << failed << " failed" << std::endl;
  return failed ? 1 : 0;
}

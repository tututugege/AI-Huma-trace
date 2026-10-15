#include "RealLsu.h"
#include <cstdio>
#include <cstdlib>

long long sim_time = 0; // Used by simulator assertion diagnostics.

static void require(bool ok, const char *message) {
  if (!ok) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

static void setup(RealLsu &lsu, unsigned stores) {
  auto &s = lsu.nxt;
  s.stq_count = stores;
  s.ldq[0].load_state = LoadState::CheckStlf;
  s.ldq[0].p_addr = 0x80001000;
  s.ldq[0].func3 = 2;
  s.ldq[0].stq_snapshot.idx = stores;
  s.stlf_queue_count = 1;
  s.stlf_queue[0].valid = true;
  for (unsigned i = 0; i < stores; ++i) {
    auto &st = s.stq[i];
    st.paddr = 0x80001000;
    st.paddr_valid = true;
    st.data_valid = true;
    st.func3 = 2;
    st.data = i == 0 ? 0x55 : 0x77;
    st.store_state = StoreState::WaitCommit;
  }
}

int main() {
  {
    RealLsu lsu;
    setup(lsu, 2);
    auto &sc = lsu.nxt.stq[1];
    sc.is_lrsc = true;
    sc.suppress_write = true;
    // A failed SC may still occupy STQ while waiting for retirement.
    sc.store_state = StoreState::Done;
    lsu.comb_stlf();
    require(lsu.nxt.ldq[0].load_state == LoadState::ReadyToWb &&
                lsu.nxt.ldq[0].result == 0x55,
            "failed SC must expose older pending store, not cache or SC data");
  }
  {
    RealLsu lsu;
    setup(lsu, 1);
    lsu.nxt.stq[0].is_lrsc = true;
    lsu.nxt.stq[0].suppress_write = true;
    lsu.comb_stlf();
    require(lsu.nxt.ldq[0].load_state == LoadState::ReadyToIssue &&
                lsu.nxt.wait_dcache_ldq_count == 1,
            "only failed SC remains: load must read cache");
  }
  for (bool succeeds : {false, true}) {
    RealLsu lsu;
    setup(lsu, 2);
    auto &sc = lsu.nxt.stq[1];
    sc.is_lrsc = true;
    sc.store_state = StoreState::Ready;
    lsu.comb_stlf();
    require(lsu.nxt.ldq[0].load_state == LoadState::CheckStlf &&
                lsu.nxt.stlf_queue_count == 1 &&
                lsu.nxt.finish_count == 0 &&
                lsu.nxt.wait_dcache_ldq_count == 0,
            "undecided SC must retry without forwarding or cache request");
    sc.sc_pass = succeeds;
    sc.suppress_write = !succeeds;
    sc.store_state = StoreState::WaitCommit;
    lsu.comb_stlf();
    require(lsu.nxt.ldq[0].load_state == LoadState::ReadyToWb &&
                lsu.nxt.ldq[0].result == (succeeds ? 0x77u : 0x55u),
            "retry must use resolved SC outcome");
  }
  {
    RealLsu lsu;
    setup(lsu, 2);
    lsu.comb_stlf();
    require(lsu.nxt.ldq[0].result == 0x77,
            "ordinary stores retain youngest-match priority");
  }
  std::puts("PASS: STLF failed/pending/successful SC and ordinary-store priority");
}

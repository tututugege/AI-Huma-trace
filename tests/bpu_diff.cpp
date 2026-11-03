// Differential test: the old RTL-style BPU_TOP (frozen copy in tests/bpu_ref) vs the simulator's compact bpuf::Bpu,
// identical stimulus, every output compared each cycle.
// Usage: bpu_diff [cycles] [seed] [pool_pcs]
#include "bpu_ref/BPU.h"
#include "BPU/BPU.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

BPU_TOP *g_bpu_top = nullptr;

struct Rng {
  uint64_t s;
  uint32_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return uint32_t(s >> 11); }
  uint32_t below(uint32_t n) { return next() % n; }
  bool chance(uint32_t pct) { return below(100) < pct; }
};

using InP = BPU_TOP::InputPayload;
using OutP = BPU_TOP::OutputPayload;

struct StaticBr { pc_t pc; br_type_t type; target_addr_t target; uint32_t bias; };

int main(int argc, char **argv) {
  const uint64_t cycles = argc > 1 ? strtoull(argv[1], 0, 10) : 1000000;
  Rng rng{argc > 2 ? strtoull(argv[2], 0, 10) * 0x9e3779b97f4a7c15ull + 1 : 12345};
  const int npool = argc > 3 ? atoi(argv[3]) : 600;
  const uint32_t span = argc > 4 ? atoi(argv[4]) : 8192;

  static BPU_TOP old_bpu;
  static bpuf::Bpu fast;
  static BPU_TOP::ReadData rd;
  static BPU_TOP::UpdateRequest req;
  static bpuf::Bpu::Txn txn;

  // synthetic program: a pool of static control-flow sites over a few KB of code
  std::vector<StaticBr> pool;
  const pc_t base = 0x80000000u + (rng.below(64) << 12);
  for (int i = 0; i < npool; i++) {
    StaticBr b;
    b.pc = base + (rng.below(span) << 2);
    const uint32_t r = rng.below(100);
    b.type = r < 50 ? BR_DIRECT : r < 62 ? BR_CALL : r < 74 ? BR_RET : r < 82 ? BR_IDIRECT : r < 92 ? BR_JAL : BR_NONCTL;
    b.target = base + (rng.below(span) << 2);
    b.bias = rng.below(101);
    pool.push_back(b);
  }
  auto pick_pc = [&]() -> pc_t { return rng.chance(85) ? pool[rng.below(pool.size())].pc : base + (rng.below(span) << 2); };

  pc_t last_fetch = RESET_PC;
  std::vector<OutP> hist_out(16);
  uint64_t mism = 0, fetches = 0, taken_preds = 0, upds = 0;
  bool first = true;

  for (uint64_t c = 0; c < cycles; c++) {
    InP in;
    std::memset(&in, 0, sizeof(in));
    in.refetch = !first && rng.chance(4);
    in.refetch_address = pick_pc() & ~3u;
    in.icache_read_ready = rng.chance(85);
    const int ncommit = rng.chance(60) ? 1 + rng.below(COMMIT_WIDTH) : 0;
    for (int i = 0; i < ncommit; i++) {
      const StaticBr &sb = pool[rng.below(pool.size())];
      in.in_upd_valid[i] = true;
      in.in_update_base_pc[i] = rng.chance(90) ? sb.pc : pick_pc();
      in.in_actual_br_type[i] = rng.chance(92) ? sb.type : rng.below(8);
      in.in_actual_dir[i] = (in.in_actual_br_type[i] == BR_DIRECT) ? rng.chance(sb.bias) : rng.chance(80);
      in.in_actual_targets[i] = rng.chance(85) ? sb.target : (base + (rng.below(span) << 2));
      const OutP &ho = hist_out[rng.below(16)];
      const int lane = rng.below(FETCH_WIDTH);
      if (rng.chance(70)) {  // realistic metadata taken from an earlier prediction
        in.in_pred_dir[i] = ho.out_pred_dir[lane];
        in.in_alt_pred[i] = ho.meta.alt_pred[lane];
        in.in_pcpn[i] = ho.meta.pcpn[lane];
        in.in_altpcpn[i] = ho.meta.altpcpn[lane];
        for (int k = 0; k < TN_MAX; k++) { in.in_tage_tags[i][k] = ho.meta.tage_tag[lane][k]; in.in_tage_idxs[i][k] = ho.meta.tage_idx[lane][k]; }
      } else {
        in.in_pred_dir[i] = rng.chance(50);
        in.in_alt_pred[i] = rng.chance(50);
        in.in_pcpn[i] = rng.chance(95) ? rng.below(TN_MAX + 1) : rng.below(8);
        in.in_altpcpn[i] = rng.below(TN_MAX + 1);
        for (int k = 0; k < TN_MAX; k++) { in.in_tage_tags[i][k] = rng.next(); in.in_tage_idxs[i][k] = rng.chance(95) ? rng.below(TN_ENTRY_NUM) : rng.next(); }
      }
      in.in_sc_used[i] = rng.chance(3);
      in.in_loop_used[i] = rng.chance(3);
      in.in_sc_pred[i] = rng.chance(50);
      in.in_sc_sum[i] = rng.next();
      in.in_loop_hit[i] = rng.chance(50);
      in.in_loop_pred[i] = rng.chance(50);
      in.in_loop_idx[i] = rng.next();
      in.in_loop_tag[i] = rng.next();
    }
    // follow the predicted stream most of the time so the pc walks through the pool
    if (rng.chance(1) && !first) {
      // rare reset of both
      old_bpu.bpu_seq_write(in, BPU_TOP::UpdateRequest{}, true);
      fast.reset();
      first = true;
      continue;
    }

    OutP o_old;
    bpuf::OutputPayload o_new;
    bpuf::InputPayload nin;
    std::memset(&nin, 0, sizeof(nin));
    nin.refetch = in.refetch; nin.refetch_address = in.refetch_address; nin.icache_read_ready = in.icache_read_ready;
    for (int i = 0; i < COMMIT_WIDTH; i++) {
      nin.in_update_base_pc[i] = in.in_update_base_pc[i]; nin.in_upd_valid[i] = in.in_upd_valid[i];
      nin.in_actual_dir[i] = in.in_actual_dir[i]; nin.in_actual_br_type[i] = in.in_actual_br_type[i];
      nin.in_actual_targets[i] = in.in_actual_targets[i]; nin.in_pred_dir[i] = in.in_pred_dir[i];
      nin.in_alt_pred[i] = in.in_alt_pred[i]; nin.in_pcpn[i] = in.in_pcpn[i]; nin.in_altpcpn[i] = in.in_altpcpn[i];
      for (int k = 0; k < TN_MAX; k++) { nin.in_tage_tags[i][k] = in.in_tage_tags[i][k]; nin.in_tage_idxs[i][k] = in.in_tage_idxs[i][k]; }
      nin.in_sc_used[i] = in.in_sc_used[i]; nin.in_loop_used[i] = in.in_loop_used[i];
    }
    old_bpu.bpu_seq_read(in, rd);
    old_bpu.bpu_comb_calc(in, rd, o_old, req);
    fast.comb(nin, o_new, txn);
    const bool same = o_old.fetch_address == o_new.fetch_address && o_old.icache_read_valid == o_new.icache_read_valid &&
                      o_old.predict_next_fetch_address == o_new.predict_next_fetch_address &&
                      o_old.PTAB_write_enable == o_new.PTAB_write_enable && o_old.out_pred_base_pc == o_new.out_pred_base_pc &&
                      o_old.update_queue_full == o_new.update_queue_full &&
                      !std::memcmp(o_old.out_pred_dir, o_new.out_pred_dir, sizeof(o_old.out_pred_dir)) &&
                      !std::memcmp(&o_old.meta, &o_new.meta, sizeof(o_old.meta));
    if (!same) {
      if (mism < 5) {
        printf("MISMATCH cycle %llu refetch=%d ready=%d\n", (unsigned long long)c, in.refetch, in.icache_read_ready);
        printf("  fetch %08x/%08x next %08x/%08x vld %d/%d ptab %d/%d base %08x/%08x\n", o_old.fetch_address, o_new.fetch_address,
               o_old.predict_next_fetch_address, o_new.predict_next_fetch_address, o_old.icache_read_valid, o_new.icache_read_valid,
               o_old.PTAB_write_enable, o_new.PTAB_write_enable, o_old.out_pred_base_pc, o_new.out_pred_base_pc);
        for (int i = 0; i < FETCH_WIDTH; i++)
          printf("  slot%d dir %d/%d alt %d/%d pcpn %d/%d altpcpn %d/%d\n", i, o_old.out_pred_dir[i], o_new.out_pred_dir[i],
                 o_old.meta.alt_pred[i], o_new.meta.alt_pred[i], o_old.meta.pcpn[i], o_new.meta.pcpn[i], o_old.meta.altpcpn[i], o_new.meta.altpcpn[i]);
      }
      mism++;
      if (mism >= 5) break;
    }
    if (o_old.PTAB_write_enable) { fetches++; for (int i = 0; i < FETCH_WIDTH; i++) taken_preds += o_old.out_pred_dir[i]; }
    for (int i = 0; i < COMMIT_WIDTH; i++) upds += in.in_upd_valid[i];
    hist_out[c & 15] = o_old;
    last_fetch = o_old.fetch_address;
    old_bpu.bpu_seq_write(in, req, false);
    fast.commit(nin, txn);
    first = false;
  }
  printf("bpu_diff: cycles=%llu fetch_groups=%llu taken_slots=%llu commits=%llu mismatches=%llu\n", (unsigned long long)cycles,
         (unsigned long long)fetches, (unsigned long long)taken_preds, (unsigned long long)upds, (unsigned long long)mism);
  (void)last_fetch;
  return mism ? 1 : 0;
}

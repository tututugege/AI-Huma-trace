// LLC differential: random upstream traffic, memory and table model through the live AXI_LLC and the frozen reference.
// Both run the same closed-loop simulation (own memory/table models, same seeds); every cycle's outputs, table commands
// and the canonical register state must agree.
#include "AXI_LLC.h"
#include "AXI_LLCRef.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <vector>

long long sim_time = 0;
using namespace axi_interconnect;

namespace {
struct Tables {
  AXI_LLCConfig cfg;
  std::vector<AXI_LLC_Bytes_t> data, meta, valid, repl;
  bool pd = false, pm = false, pv = false, pr = false;
  uint32_t id = 0, im = 0, iv = 0, ir = 0;
  void init(const AXI_LLCConfig &c) {
    cfg = c;
    const uint32_t sets = c.set_count();
    data.assign(sets, {}); meta.assign(sets, {}); valid.assign(sets, {}); repl.assign(sets, {});
    for (uint32_t s = 0; s < sets; s++) {
      data[s].resize((size_t)c.ways * c.line_bytes);
      meta[s].resize((size_t)c.ways * AXI_LLC_META_ENTRY_BYTES);
      valid[s].resize(c.ways);
      repl[s].resize(AXI_LLC_REPL_BYTES);
    }
  }
  AXI_LLC_LookupIn_t out() const {
    AXI_LLC_LookupIn_t l{};
    if (pd) { l.data_valid = true; l.data = data[id]; }
    if (pm) { l.meta_valid = true; l.meta = meta[im]; }
    if (pv) { l.valid_valid = true; l.valid = valid[iv]; }
    if (pr) { l.repl_valid = true; l.repl = repl[ir]; }
    return l;
  }
  static void put(AXI_LLC_Bytes_t &set, uint32_t way, uint32_t per_way, const AXI_LLC_Bytes_t &p, const std::vector<uint8_t> &be) {
    const size_t base = (size_t)way * per_way;
    for (uint32_t i = 0; i < per_way; i++) {
      if (base + i >= set.size() || i >= p.size()) break;
      if (!be.empty() && i < be.size() && be[i] == 0) continue;
      set.data()[base + i] = p.data()[i];
    }
  }
  void seq(const AXI_LLC_TableOut_t &t) {
    pd = pm = pv = pr = false;
    if (t.invalidate_all) for (auto &v : valid) std::fill(v.data(), v.data() + v.size(), 0);
    auto one = [&](const AXI_LLC_TableReq_t &r, std::vector<AXI_LLC_Bytes_t> &tab, uint32_t per_way, bool &pend, uint32_t &idx) {
      if (r.write && r.index < tab.size()) put(tab[r.index], r.way, per_way, r.payload, r.byte_enable);
      else if (r.enable && r.index < tab.size()) { pend = true; idx = r.index; }
    };
    one(t.data, data, cfg.line_bytes, pd, id);
    one(t.meta, meta, AXI_LLC_META_ENTRY_BYTES, pm, im);
    one(t.valid, valid, cfg.ways, pv, iv);   // per-way stride equals the row for valid: way index is a byte offset
    one(t.repl, repl, AXI_LLC_REPL_BYTES, pr, ir);
  }
};

struct Params { AXI_LLCConfig cfg; int region_lines; int p_bypass, p_direct, p_inval_all, p_inval_line, p_full_write, p_read, p_write, mem_lat, p_ready, p_resp_ready, p_stream, only_master, window; };

template <class LLC> struct Sim {
  LLC llc;
  Tables tab;
  std::mt19937_64 rng;
  Params p;
  struct Rd { bool v = false; AXI_LLC_ReadReqIn_t r; } rd[NUM_READ_MASTERS];
  struct Wr { bool v = false; AXI_LLC_WriteReqIn_t r; } wr[NUM_WRITE_MASTERS];
  struct Pend { int id; long due; };
  std::deque<Pend> rd_out, wr_out;
  Sim(const Params &pp, uint64_t seed) : rng(seed), p(pp) {
    llc.set_config(p.cfg);
    tab.init(p.cfg);
  }
  bool bit(int pct) { return (int)(rng() % 100) < pct; }
  uint32_t last_line = 0;
  uint32_t addr() {
    if (bit(p.p_stream)) { last_line = (last_line + p.cfg.line_bytes) % (p.region_lines * p.cfg.line_bytes * 4); return last_line; }
    const uint32_t line = rng() % p.region_lines;
    // lines spread over sets and tags: stride by the set count so conflicts happen
    const uint32_t sets = p.cfg.set_count();
    const uint32_t l = (line % 3 == 0) ? (line / 3) * sets : line;
    return l * p.cfg.line_bytes + (bit(50) ? 0 : (rng() % p.cfg.line_bytes) & ~3u);
  }
  // returns the observable output fingerprint of this cycle
  std::vector<uint64_t> step(long t) {
    sim_time = t;
    const bool quiesce = p.window && (t % 3000) < 150;  // no new requests: lets the cache drain so maintenance can be accepted
    for (int i = 0; i < NUM_READ_MASTERS; i++)
      if (!rd[i].v && !quiesce && (p.only_master < 0 || p.only_master == i) && bit(p.p_read)) {
        rd[i].v = true;
        auto &r = rd[i].r;
        r = {};
        r.valid = true; r.addr = addr(); r.id = rng() % 16; r.total_size = (rng() % 4 == 0) ? 3 : p.cfg.line_bytes - 1;
        r.bypass = bit(p.p_bypass); r.direct_mapped = bit(p.p_direct); r.mode2_ddr_aligned = bit(30);
      }
    for (int i = 0; i < NUM_WRITE_MASTERS; i++)
      if (!wr[i].v && !quiesce && p.only_master < 0 && bit(p.p_write)) {
        wr[i].v = true;
        auto &r = wr[i].r;
        r = {};
        r.valid = true; r.id = rng() % 16; r.bypass = bit(p.p_bypass); r.direct_mapped = bit(p.p_direct); r.mode2_ddr_aligned = bit(30);
        if (bit(p.p_full_write)) {
          r.addr = (addr() / p.cfg.line_bytes) * p.cfg.line_bytes; r.total_size = p.cfg.line_bytes - 1;
          for (uint32_t b = 0; b < p.cfg.line_bytes; b++) r.wstrb.set(b, true);
        } else {
          r.addr = addr() & ~3u; r.total_size = 3;
          for (int b = 0; b < 4; b++) r.wstrb.set(b, true);
        }
      }
    auto &in = llc.io.ext_in;
    in = {};
    for (int i = 0; i < NUM_READ_MASTERS; i++) { if (rd[i].v) in.upstream.read_req[i] = rd[i].r; in.upstream.read_resp[i].ready = bit(p.p_resp_ready); }
    for (int i = 0; i < NUM_WRITE_MASTERS; i++) { if (wr[i].v) in.upstream.write_req[i] = wr[i].r; in.upstream.write_resp[i].ready = bit(p.p_resp_ready); }
    in.mem.prefetch_allow = bit(90);
    in.mem.read_req_ready = bit(p.p_ready);
    in.mem.write_req_ready = bit(p.p_ready);
    if (!rd_out.empty() && rd_out.front().due <= t) { in.mem.read_resp_valid = true; in.mem.read_resp_id = rd_out.front().id; }
    if (!wr_out.empty() && wr_out.front().due <= t) { in.mem.write_resp_valid = true; in.mem.write_resp_id = wr_out.front().id; in.mem.write_resp = bit(10) ? 2 : 0; }
    if (bit(quiesce ? 10 : p.p_inval_all)) in.mem.invalidate_all = true;
    if (bit(p.p_inval_line)) { in.mem.invalidate_line_valid = true; in.mem.invalidate_line_addr = addr(); }
    llc.io.lookup_in = tab.out();

    llc.comb();

    const auto &o = llc.io.ext_out;
    for (int i = 0; i < NUM_READ_MASTERS; i++) if (rd[i].v && o.upstream.read_req[i].ready) rd[i].v = false;
    for (int i = 0; i < NUM_WRITE_MASTERS; i++) if (wr[i].v && o.upstream.write_req[i].ready) wr[i].v = false;
    if (in.mem.read_resp_valid && o.mem.read_resp_ready) rd_out.pop_front();
    if (in.mem.write_resp_valid && o.mem.write_resp_ready) wr_out.pop_front();
    if (o.mem.read_req_valid && in.mem.read_req_ready) rd_out.push_back({o.mem.read_req_id, t + 1 + (long)(rng() % p.mem_lat)});
    if (o.mem.write_req_valid && in.mem.write_req_ready) wr_out.push_back({o.mem.write_req_id, t + 1 + (long)(rng() % p.mem_lat)});

    std::vector<uint64_t> fp;
    auto m = [&](uint64_t v) { fp.push_back(v); };
    for (int i = 0; i < NUM_READ_MASTERS; i++) { m(o.upstream.read_req[i].ready); m(o.upstream.read_resp[i].valid); m(o.upstream.read_resp[i].id); m(o.upstream.read_resp[i].data[0]); }
    for (int i = 0; i < NUM_WRITE_MASTERS; i++) { m(o.upstream.write_req[i].ready); m(o.upstream.write_resp[i].valid); m(o.upstream.write_resp[i].id); m(o.upstream.write_resp[i].resp); }
    const auto &mo = o.mem;
    m(mo.invalidate_all_accepted); m(mo.invalidate_line_accepted); m(mo.read_req_valid); m(mo.read_req_addr); m(mo.read_req_size);
    m(mo.read_req_mode2_ddr_aligned); m(mo.read_req_id); m(mo.read_resp_ready); m(mo.write_req_valid); m(mo.write_req_addr);
    m(mo.write_req_size); m(mo.write_req_mode2_ddr_aligned); m(mo.write_req_id); m(mo.write_resp_ready); m(mo.write_req_data[0]);
    for (int b = 0; b < MAX_WRITE_TRANSACTION_BYTES; b++) m(mo.write_req_strobe.test(b));
    auto tr = [&](const AXI_LLC_TableReq_t &r) {
      m(r.enable); m(r.write); m(r.index); m(r.way);
      for (size_t i = 0; i < r.payload.size(); i++) m(r.payload.data()[i]);
      m(r.payload.size()); m(r.byte_enable.size());
      for (auto b : r.byte_enable) m(b);
    };
    const auto &to = llc.io.table_out;
    tr(to.data); tr(to.meta); tr(to.valid); tr(to.repl); m(to.invalidate_all);

    tab.seq(to);
    llc.seq();
    // canonical registers: live slots only (dead slots keep stale bytes by design)
    static thread_local AXI_LLC_Regs_t canon;
    std::memset(&canon, 0, sizeof(canon));
    if constexpr (std::is_same<LLC, AXI_LLC>::value) copy_regs_live(canon, llc.io.regs); else copy_regs_live_ref(canon, llc.io.regs);
    // flags of a lookup that is not valid are never read (every launcher rewrites them)
    if (!canon.lookup_valid_r) {
      canon.lookup_is_prefetch_r = canon.lookup_is_invalidate_r = canon.lookup_is_write_r = canon.lookup_is_bypass_r =
          canon.lookup_is_direct_mapped_r = canon.lookup_is_mode2_ddr_aligned_r = false;
      canon.lookup_addr_r = canon.lookup_size_r = canon.lookup_master_r = canon.lookup_id_r = 0;
    }
    const unsigned char *b = reinterpret_cast<const unsigned char *>(&canon);
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < sizeof(canon); i++) h = (h ^ b[i]) * 1099511628211ull;
    m(h);
    return fp;
  }
};
}  // namespace

int main(int argc, char **argv) {
  long cycles = argc > 1 ? atol(argv[1]) : 2000000;
  uint64_t seed = argc > 2 ? atol(argv[2]) : 1;
  std::mt19937_64 pick(seed);
  long bad = 0, total = 0;
  for (long done = 0; done < cycles && !bad;) {
    Params p{};
    p.cfg.enable = true;
    p.cfg.line_bytes = 64;
    const int shape = pick() % 4;
    p.cfg.ways = shape == 1 ? 4 : shape == 2 ? 1 : 2;
    p.cfg.size_bytes = (uint64_t)p.cfg.line_bytes * p.cfg.ways * (shape == 3 ? 8 : 2 + pick() % 3);
    p.cfg.mshr_num = 1 + pick() % 4;
    p.cfg.prefetch_enable = pick() % 2;
    p.cfg.prefetch_degree = 1 + pick() % 2;
    p.region_lines = 4 + pick() % 28;
    p.p_bypass = (int[]){0, 5, 20}[pick() % 3];
    p.p_direct = (int[]){0, 3, 10}[pick() % 3];
    p.p_inval_all = (int[]){0, 0, 1, 5}[pick() % 4];
    p.p_inval_line = (int[]){0, 1, 4}[pick() % 3];
    p.p_full_write = (int[]){10, 50, 90}[pick() % 3];
    p.p_read = (int[]){3, 10, 30, 70}[pick() % 4];
    p.p_write = 5 + pick() % 50;
    p.mem_lat = 1 + pick() % 12;
    p.p_ready = 30 + pick() % 71;
    p.p_resp_ready = (int[]){5, 30, 100}[pick() % 3];
    p.p_stream = (int[]){0, 30, 80}[pick() % 3];
    p.only_master = -1;
    if (pick() % 5 == 0) { p.cfg.mshr_num = 1; p.cfg.prefetch_enable = true; p.p_stream = 80; }
    p.window = pick() % 2;
    if (pick() % 4 == 0) {  // sequential miss stream on one master: trains and launches the prefetcher
      p.cfg.prefetch_enable = true; p.only_master = pick() % NUM_READ_MASTERS; p.p_stream = 100; p.p_read = 5 + pick() % 15;
      p.p_bypass = p.p_direct = p.p_inval_line = 0; p.region_lines = 8 + pick() % 24;
    }
    const long n = 20000 + pick() % 60000;
    const uint64_t s = pick();
    Sim<AXI_LLC> a(p, s);
    Sim<AXI_LLCRef> b(p, s);
    for (long t = 1; t <= n && !bad; t++, done++) {
      auto fa = a.step(t), fb = b.step(t);
      total++;
      if (fa != fb) {
        size_t i = 0;
        while (i < fa.size() && fa[i] == fb[i]) i++;
        printf("MISMATCH cycle %ld field %zu/%zu (ways=%u sets=%u mshr=%u pf=%d)\n", t, i, fa.size(), p.cfg.ways, p.cfg.set_count(), p.cfg.mshr_num, (int)p.cfg.prefetch_enable);
        bad++;
      }
    }
  }
  printf("llc_diff: %ld cycles, %ld mismatches\n", total, bad);
  return bad != 0;
}

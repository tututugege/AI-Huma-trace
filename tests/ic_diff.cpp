// Interconnect differential: random master traffic, a randomised AXI slave on each downstream port and a table model for
// the LLC, through the live AXI_Interconnect and the frozen original. Both run the same closed-loop simulation (same
// seeds); every cycle all upstream/downstream outputs and the LLC table commands must be identical.
#include "AXI_Interconnect.h"
#include "AXI_InterconnectRef.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <vector>

long long sim_time = 0;
using namespace axi_interconnect;

// The interconnect mirrors MMIO accesses into a backing memory (weak pmem_read/pmem_write): one private memory per
// simulation so the two instances cannot influence each other.
#include <map>
static std::map<uint32_t, uint32_t> *g_mem = nullptr;
uint32_t pmem_read(uint32_t a) { auto it = g_mem->find(a & ~3u); return it == g_mem->end() ? (a * 2654435761u) : it->second; }
void pmem_write(uint32_t a, uint32_t d) { (*g_mem)[a & ~3u] = d; }

namespace {
struct Tables {  // LLC table model (same as llc_diff)
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
    one(t.valid, valid, cfg.ways, pv, iv);
    one(t.repl, repl, AXI_LLC_REPL_BYTES, pr, ir);
  }
};

struct Params {
  AXI_LLCConfig cfg; int mode, region_lines, p_read, p_write, p_bypass, p_ready, p_resp_ready, lat, p_mmio, p_inval, p_modechg, p_stream, p_partial;
  uint32_t ddr_window_lines;
};

// One AXI slave (DDR or MMIO side): in-order read bursts, write bursts acknowledged after the last beat.
struct Slave {
  struct Rd { int id, beats; long due; };
  struct Wr { int id, len; };
  std::deque<Rd> rds;
  int r_beat = 0;
  std::deque<Wr> aws;
  int w_beats = 0;
  struct B { int id; long due; };
  std::deque<B> bs;
  bool arready = false, awready = false, wready = false, rvalid = false, bvalid = false;
  void pre(std::mt19937_64 &rng, int p_ready, long t, sim_ddr::SimDDR_IO_t &io) {
    auto bit = [&](int pct) { return (int)(rng() % 100) < pct; };
    arready = bit(p_ready); awready = bit(p_ready); wready = bit(p_ready);
    rvalid = !rds.empty() && rds.front().due <= t;
    bvalid = !bs.empty() && bs.front().due <= t;
    io.ar.arready = arready; io.aw.awready = awready; io.w.wready = wready;
    io.r.rvalid = rvalid; io.r.rid = rvalid ? rds.front().id : 0; io.r.rresp = 0;
    io.r.rlast = rvalid && r_beat + 1 == rds.front().beats;
    io.r.rdata = {};
    if (rvalid) axi_compat::set_u32(io.r.rdata, 0, uint32_t(rng()));
    io.b.bvalid = bvalid; io.b.bid = bvalid ? bs.front().id : 0; io.b.bresp = 0;
  }
  void post(std::mt19937_64 &rng, int lat, long t, const sim_ddr::SimDDR_IO_t &io) {
    if (io.ar.arvalid && arready) rds.push_back({int(io.ar.arid), int(io.ar.arlen) + 1, t + 1 + long(rng() % lat)});
    if (io.aw.awvalid && awready) aws.push_back({int(io.aw.awid), int(io.aw.awlen) + 1});
    if (io.w.wvalid && wready) {
      w_beats++;
      if (io.w.wlast) {
        if (!aws.empty()) { bs.push_back({aws.front().id, t + 1 + long(rng() % lat)}); aws.pop_front(); }
        w_beats = 0;
      }
    }
    if (rvalid && io.r.rready) { if (++r_beat == rds.front().beats) { rds.pop_front(); r_beat = 0; } }
    if (bvalid && io.b.bready) bs.pop_front();
  }
};

template <class IC> struct Sim {
  IC *ic;
  std::map<uint32_t, uint32_t> mem;
  Tables tab;
  Slave ddr, mmio;
  std::mt19937_64 rng;
  Params p;
  struct Rd { bool v = false; uint32_t addr; int size, id; bool bypass; } rd[NUM_READ_MASTERS];
  struct Wr { bool v = false; uint32_t addr; int size, id; bool bypass; bool full; } wr[NUM_WRITE_MASTERS];
  uint32_t last = 0;
  Sim(const Params &pp, uint64_t seed) : rng(seed), p(pp) {
    ic = new IC;
    ic->mode = p.mode;
    ic->llc_mapped_offset = 0x30000000u;
    ic->set_llc_config(p.cfg);
    ic->init();
    tab.init(p.cfg);
  }
  bool bit(int pct) { return (int)(rng() % 100) < pct; }
  uint32_t addr(int &size) {
    const uint32_t lb = p.cfg.line_bytes;
    uint32_t a;
    if (bit(p.p_mmio)) { size = bit(8) ? (int[]){0, 7, 63}[rng() % 3] : 3; return 0x10000000u + (rng() % 64) * 4; }  // a few unsupported MMIO sizes
    if (p.mode == 2 && bit(40)) {  // direct-mapped window
      size = bit(50) ? 3 : lb - 1;
      return 0x30000000u + (rng() % 64) * lb + ((size == 3) ? (rng() % 16) * 4 : 0);
    }
    const uint32_t line = bit(p.p_stream) ? (last = (last + 1) % p.region_lines) : rng() % p.region_lines;
    const uint32_t sets = p.cfg.set_count();
    const uint32_t l = (line % 3 == 0) ? (line / 3) * sets : line;
    a = 0x80000000u + l * lb;
    if (bit(50)) { size = lb - 1; } else { size = (int[]){0, 3, 7, 15}[rng() % 4]; a += (rng() % lb) & ~uint32_t(size); }
    return a;
  }
  std::vector<uint64_t> step(long t) {
    sim_time = t;
    g_mem = &mem;
    auto &c = *ic;
    for (int i = 0; i < NUM_READ_MASTERS; i++)
      if (!rd[i].v && bit(p.p_read)) { rd[i].v = true; rd[i].addr = addr(rd[i].size); rd[i].id = rng() % 16; rd[i].bypass = bit(p.p_bypass); }
    for (int i = 0; i < NUM_WRITE_MASTERS; i++)
      if (!wr[i].v && bit(p.p_write)) { wr[i].v = true; wr[i].addr = addr(wr[i].size); wr[i].id = rng() % 16; wr[i].bypass = bit(p.p_bypass); }
    // upstream inputs
    for (int i = 0; i < NUM_READ_MASTERS; i++) {
      auto &q = c.read_ports[i].req;
      q.valid = rd[i].v; q.addr = rd[i].v ? rd[i].addr : 0; q.total_size = rd[i].v ? rd[i].size : 0; q.id = rd[i].v ? rd[i].id : 0; q.bypass = rd[i].v && rd[i].bypass;
      c.read_ports[i].resp.ready = bit(p.p_resp_ready);
    }
    for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
      auto &q = c.write_ports[i].req;
      q.valid = wr[i].v; q.addr = wr[i].v ? wr[i].addr : 0; q.total_size = wr[i].v ? wr[i].size : 0; q.id = wr[i].v ? wr[i].id : 0; q.bypass = wr[i].v && wr[i].bypass;
      q.wdata.clear(); q.wstrb.clear();
      if (wr[i].v) {
        q.wdata[0] = uint32_t(rng());
        const int n = std::min<int>(wr[i].size + 1, MAX_WRITE_TRANSACTION_BYTES);
        const bool all = bit(80);
        for (int b = 0; b < n; b++) q.wstrb.set(b, all || bit(50));
      }
      c.write_ports[i].resp.ready = bit(p.p_resp_ready);
    }
    if (bit(p.p_inval)) c.set_llc_invalidate_all(true); else c.set_llc_invalidate_all(false);
    if (bit(p.p_inval)) c.set_llc_invalidate_line(true, 0x80000000u + (rng() % p.region_lines) * p.cfg.line_bytes); else c.set_llc_invalidate_line(false, 0);
    if (bit(p.p_modechg)) c.mode = (int[]){0, 1, 2}[rng() % 3];
    c.set_llc_lookup_in(tab.out());
    ddr.pre(rng, p.p_ready, t, c.axi_ddr_io);
    mmio.pre(rng, p.p_ready, t, c.axi_mmio_io);

    c.comb_outputs();
    c.comb_inputs();

    std::vector<uint64_t> fp;
    auto m = [&](uint64_t v) { fp.push_back(v); };
    for (int i = 0; i < NUM_READ_MASTERS; i++) {
      const auto &q = c.read_ports[i];
      m(q.req.ready); m(q.req.accepted); m(q.req.accepted_id); m(q.resp.valid); m(q.resp.id); m(q.resp.data[0]);
      m(c.read_req_accepted[i]); m(c.read_req_accepted_id[i]);
      if (q.req.accepted) rd[i].v = false;
    }
    for (int i = 0; i < NUM_WRITE_MASTERS; i++) {
      const auto &q = c.write_ports[i];
      m(q.req.ready); m(q.req.accepted); m(q.resp.valid); m(q.resp.id); m(q.resp.resp); m(c.write_req_accepted[i]);
      if (q.req.accepted) wr[i].v = false;
    }
    for (auto *io : {&c.axi_ddr_io, &c.axi_mmio_io}) {
      m(io->ar.arvalid); m(io->ar.arid); m(io->ar.araddr); m(io->ar.arlen); m(io->ar.arsize); m(io->ar.arburst); m(io->r.rready);
      m(io->aw.awvalid); m(io->aw.awid); m(io->aw.awaddr); m(io->aw.awlen); m(io->aw.awsize); m(io->aw.awburst);
      m(io->w.wvalid); m(uint32_t(io->w.wdata)); m(uint64_t(io->w.wstrb)); m(io->w.wlast); m(io->b.bready);
    }
    auto tr = [&](const AXI_LLC_TableReq_t &r) {
      m(r.enable); m(r.write); m(r.index); m(r.way);
      m(r.payload.size()); for (size_t i = 0; i < r.payload.size(); i++) m(r.payload.data()[i]);
      m(r.byte_enable.size()); for (auto b : r.byte_enable) m(b);
    };
    const auto &to = c.get_llc_table_out();
    tr(to.data); tr(to.meta); tr(to.valid); tr(to.repl); m(to.invalidate_all);
    m(c.llc_invalidate_all_accepted()); m(c.llc_invalidate_line_accepted()); m(c.active_mode()); m(c.idle_quiescent());

    ddr.post(rng, p.lat, t, c.axi_ddr_io);
    mmio.post(rng, p.lat, t, c.axi_mmio_io);
    tab.seq(to);
    c.seq();
    return fp;
  }
};
}  // namespace

int main(int argc, char **argv) {
  long cycles = argc > 1 ? atol(argv[1]) : 1000000;
  uint64_t seed = argc > 2 ? atol(argv[2]) : 1;
  std::mt19937_64 pick(seed);
  long bad = 0, total = 0;
  for (long done = 0; done < cycles && !bad;) {
    Params p{};
    p.cfg.enable = pick() % 8 != 0;
    p.cfg.line_bytes = 64;
    const int shape = pick() % 3;
    p.cfg.ways = shape == 1 ? 4 : 2;
    p.cfg.size_bytes = (uint64_t)64 * p.cfg.ways * (2 + pick() % 3);
    p.cfg.mshr_num = 1 + pick() % 4;
    p.cfg.prefetch_enable = pick() % 2;
    p.mode = (int[]){1, 1, 2, 0}[pick() % 4];
    p.region_lines = 4 + pick() % 28;
    p.p_read = (int[]){5, 20, 60}[pick() % 3];
    p.p_write = (int[]){2, 10, 40}[pick() % 3];
    p.p_bypass = (int[]){0, 5, 30}[pick() % 3];
    p.p_ready = 30 + pick() % 71;
    p.p_resp_ready = (int[]){10, 50, 100}[pick() % 3];
    p.lat = 1 + pick() % 10;
    p.p_mmio = (int[]){0, 5, 20}[pick() % 3];
    p.p_inval = (int[]){0, 0, 1}[pick() % 3];
    p.p_modechg = (int[]){0, 0, 0, 1}[pick() % 4] ? 1 : 0;
    p.p_stream = (int[]){0, 40, 90}[pick() % 3];
    const long n = 20000 + pick() % 50000;
    const uint64_t s = pick();
    Sim<AXI_Interconnect> a(p, s);
    Sim<AXI_InterconnectRef> b(p, s);
    for (long t = 1; t <= n && !bad; t++, done++) {
      auto fa = a.step(t), fb = b.step(t);
      total++;
      if (fa != fb) {
        size_t i = 0;
        while (i < fa.size() && fa[i] == fb[i]) i++;
        printf("MISMATCH cycle %ld field %zu/%zu (mode=%d llc=%d ways=%u mshr=%u)\n", t, i, fa.size(), p.mode, (int)p.cfg.enable, p.cfg.ways, p.cfg.mshr_num);
        bad++;
      }
    }
  }
  printf("ic_diff: %ld cycles, %ld mismatches\n", total, bad);
  return bad != 0;
}

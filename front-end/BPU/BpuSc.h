#pragma once
// Hardware-faithful Stage 2 8-table Long-History Block-TAGE + S2 Statistical Corrector (SC).
// Matches rtl/frontend/bpu/bpu_ftb_ex_tage.sv and bpu_ftb_sc.sv cycle-for-cycle:
//   - Incremental cyclic shift-XOR folded histories (ex_fh[3][8], sc_fh[4]) + 8-bit IMLI counter
//   - S1 synchronous SRAM read (read()) -> registered raw -> S2 pure combinational select (select_slot())
//   - Bias table organized as 64 sets x 4 ways ({tage_pred, low_conf}) so S1 reads all 4 ways and S2 muxes
//   - Single-branch commit update (BRQ_W=1) using live port-1 read
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace bpuf {

constexpr int EX_TN = 8;
constexpr int EX_FH_K = 3;
constexpr int EX_TAGE_N = 4096;
constexpr int EX_TAGE_BASE_N = 4096;
constexpr uint16_t EX_TAGE_TAG_MASK = 0x7ffu;  // 11-bit tags
constexpr int EX_TAGE_URST = 11;
constexpr int EX_GHR_LEN[EX_TN] = {5, 11, 22, 44, 89, 178, 356, 712};
constexpr int EX_FH_LEN[EX_FH_K] = {12, 11, 10};

constexpr int SC_GEHL_N = 4;
constexpr int SC_GHR_LEN[SC_GEHL_N] = {4, 10, 20, 40};
constexpr int SC_FH_LEN = 9;

struct ExTageIdx {
  uint32_t w = 0;
  uint32_t base_idx = 0;
  uint32_t idx[EX_TN] = {};
  uint16_t tag[EX_TN] = {};
};

struct ScIdx {
  uint32_t w = 0;
  uint32_t bias_set = 0;       // 6-bit (w & 63)
  uint32_t gehl_idx[SC_GEHL_N] = {};
  uint32_t imli_idx = 0;
  uint32_t lht_idx = 0;
};

inline ExTageIdx make_ex_tage_index(uint32_t w, const uint16_t ex_fh[EX_FH_K][EX_TN]) {
  ExTageIdx ix;
  ix.w = w;
  ix.base_idx = (w ^ (w >> 12)) & (EX_TAGE_BASE_N - 1);
  for (int t = 0; t < EX_TN; t++) {
    ix.idx[t] = (w ^ (w >> 11) ^ uint32_t(ex_fh[0][t])) & (EX_TAGE_N - 1);
    ix.tag[t] = static_cast<uint16_t>((w ^ (w >> 11) ^ uint32_t(ex_fh[1][t]) ^ (uint32_t(ex_fh[2][t]) << 1)) & EX_TAGE_TAG_MASK);
  }
  return ix;
}

inline ScIdx make_sc_index(uint32_t w, const uint16_t sc_fh[SC_GEHL_N], uint8_t imli) {
  ScIdx ix;
  ix.w = w;
  ix.bias_set = w & 63u;
  for (int g = 0; g < SC_GEHL_N; g++) {
    ix.gehl_idx[g] = (w ^ (w >> 9) ^ uint32_t(sc_fh[g])) & 511u;
  }
  ix.imli_idx = (w ^ (uint32_t(imli) * 13u)) & 255u;
  ix.lht_idx = w & 63u;
  return ix;
}

// 8-table long-history Block-TAGE for S2 (4096 entries x 8 tables x 4 slots)
struct ExBlockTage {
  static constexpr int SLOTS = 4;
  std::vector<uint8_t> base[SLOTS];
  std::vector<uint16_t> tag[EX_TN][SLOTS];
  std::vector<uint8_t> cnt[EX_TN][SLOTS];
  std::vector<uint8_t> useful[EX_TN][SLOTS];
  uint8_t use_alt[16] = {};
  uint8_t lsfr = 8;
  uint32_t reset_cnt = 0;

  struct RawSlot {
    uint8_t base_cnt = 1;
    uint16_t tag[EX_TN] = {};
    uint8_t cnt[EX_TN] = {};
    uint8_t useful[EX_TN] = {};
  };
  struct Raw {
    RawSlot s[SLOTS];
  };
  struct Pred {
    bool pred = false;
    bool alt = false;
    uint8_t pcpn = EX_TN;
    bool used_alt = false;
    uint8_t prov_cnt = 4;
    uint8_t base_cnt = 1;
  };

  void init() {
    for (int sl = 0; sl < SLOTS; sl++) {
      base[sl].assign(EX_TAGE_BASE_N, 1);
      for (int t = 0; t < EX_TN; t++) {
        tag[t][sl].assign(EX_TAGE_N, 0);
        cnt[t][sl].assign(EX_TAGE_N, 0);
        useful[t][sl].assign(EX_TAGE_N, 0);
      }
    }
    for (int i = 0; i < 16; i++) use_alt[i] = 7;
    lsfr = 8;
    reset_cnt = 0;
  }

  void read(const ExTageIdx &ix, Raw &r) const {
    for (int sl = 0; sl < SLOTS; sl++) {
      r.s[sl].base_cnt = base[sl][ix.base_idx];
      for (int t = 0; t < EX_TN; t++) {
        const uint32_t ti = ix.idx[t];
        r.s[sl].tag[t] = tag[t][sl][ti];
        r.s[sl].cnt[t] = cnt[t][sl][ti];
        r.s[sl].useful[t] = useful[t][sl][ti];
      }
    }
  }

  void select_slot(const ExTageIdx &ix, const RawSlot &r, Pred &o) const {
    int pcpn = EX_TN, altpcpn = EX_TN;
    for (int t = 0; t < EX_TN; t++) {
      if (r.tag[t] == ix.tag[t]) {
        altpcpn = pcpn;
        pcpn = t;
      }
    }
    o.base_cnt = r.base_cnt;
    const bool bp = o.base_cnt >= 2;
    o.pcpn = static_cast<uint8_t>(pcpn);
    o.alt = (altpcpn == EX_TN) ? bp : (r.cnt[altpcpn] >= 4);
    o.prov_cnt = (pcpn == EX_TN) ? o.base_cnt : r.cnt[pcpn];
    const bool pp = (pcpn == EX_TN) ? bp : (o.prov_cnt >= 4);
    const bool weak = (pcpn != EX_TN) && (o.prov_cnt == 3 || o.prov_cnt == 4);
    const bool ulow = (pcpn != EX_TN) && (r.useful[pcpn] == 0);
    const uint8_t ua = use_alt[(ix.w >> 2) & 15u];
    o.used_alt = (pcpn != EX_TN) && weak && ulow && (ua <= 8);
    o.pred = o.used_alt ? o.alt : pp;
  }

  void predict_slot(const ExTageIdx &ix, int sl, Pred &o) const {
    RawSlot r;
    r.base_cnt = base[sl][ix.base_idx];
    for (int t = 0; t < EX_TN; t++) {
      const uint32_t ti = ix.idx[t];
      r.tag[t] = tag[t][sl][ti];
      r.cnt[t] = cnt[t][sl][ti];
      r.useful[t] = useful[t][sl][ti];
    }
    select_slot(ix, r, o);
  }

  void update_slot(const ExTageIdx &ix, int sl, bool real_dir, Pred *pred_out = nullptr) {
    if (sl < 0 || sl >= SLOTS) return;
    const uint32_t next_rc = reset_cnt + 1;
    const bool do_rst = ((next_rc & ((1u << EX_TAGE_URST) - 1u)) == 0);
    auto set_u = [&](uint8_t &u, uint8_t nu) {
      // Single-write-port u_uhi SRAM: periodic reset takes priority on bit 1 on 1-in-2048 reset cycles
      u = do_rst ? static_cast<uint8_t>((u & 2u) | (nu & 1u)) : nu;
    };
    Pred p;
    predict_slot(ix, sl, p);
    if (pred_out) *pred_out = p;
    const int pcpn = p.pcpn;

    if (pcpn == EX_TN) {
      uint8_t &bc = base[sl][ix.base_idx];
      bc = real_dir ? (bc < 3 ? bc + 1 : 3) : (bc > 0 ? bc - 1 : 0);
    } else {
      uint8_t &c = cnt[pcpn][sl][ix.idx[pcpn]];
      const bool pdir = c >= 4;
      const bool weak = (c == 3 || c == 4);
      if (weak && pdir != p.alt) {
        uint8_t &ua = use_alt[(ix.w >> 2) & 15u];
        if (p.alt == real_dir) { if (ua > 0) ua--; }
        else { if (ua < 15) ua++; }
      }
      c = real_dir ? (c < 7 ? c + 1 : 7) : (c > 0 ? c - 1 : 0);
      if (pdir != p.alt) {
        uint8_t &u = useful[pcpn][sl][ix.idx[pcpn]];
        const uint8_t nu = (pdir == real_dir) ? (u < 3 ? uint8_t(u + 1) : 3) : (u > 0 ? uint8_t(u - 1) : 0);
        set_u(u, nu);
      }
    }

    if (p.pred != real_dir) {
      int free_t[EX_TN];
      int nf = 0;
      const int start_t = (pcpn == EX_TN) ? 0 : (pcpn + 1);
      for (int t = start_t; t < EX_TN; t++) {
        if (useful[t][sl][ix.idx[t]] == 0) free_t[nf++] = t;
      }
      if (nf > 0) {
        int chosen = free_t[0];
        if (nf > 1 && ((lsfr & 3u) == 0)) chosen = free_t[1];
        tag[chosen][sl][ix.idx[chosen]] = ix.tag[chosen];
        cnt[chosen][sl][ix.idx[chosen]] = real_dir ? 4 : 3;
        set_u(useful[chosen][sl][ix.idx[chosen]], 0);
      } else {
        for (int t = start_t; t < EX_TN; t++) {
          uint8_t &u = useful[t][sl][ix.idx[t]];
          if (u > 0) set_u(u, uint8_t(u - 1));
        }
      }
    }

    const uint8_t fb = ((lsfr >> 3) ^ (lsfr >> 2)) & 1u;
    lsfr = static_cast<uint8_t>(((lsfr << 1) | fb) & 0xfu);
    if (lsfr == 0) lsfr = 1;
    reset_cnt = next_rc;
    if (do_rst) {
      const uint32_t row = (reset_cnt >> EX_TAGE_URST) & (EX_TAGE_N - 1);
      for (int t = 0; t < EX_TN; t++) useful[t][sl][row] &= 1u;
    }
  }
};

// S2 Statistical Corrector (Bias 64x4 + 4 Global GEHL 512 + IMLI 256 + Local 256 + LHT 64)
struct StatCorrector {
  static constexpr int SLOTS = 4;
  static constexpr int BIAS_SETS = 64;
  static constexpr int GEHL_N = 512;
  static constexpr int IMLI_N = 256;
  static constexpr int LOC_N = 256;
  static constexpr int LHT_N = 64;

  std::vector<int8_t> bias[SLOTS][4];
  std::vector<int8_t> gehl[SC_GEHL_N][SLOTS];
  std::vector<int8_t> imli_tbl[SLOTS];
  std::vector<int8_t> loc_tbl[SLOTS];
  uint16_t lht[LHT_N] = {};
  int thres[16] = {};

  struct RawSlot {
    int8_t bias[4] = {};
    int8_t gehl[SC_GEHL_N] = {};
    int8_t imli = 0;
    int8_t loc = 0;
  };
  struct Raw {
    RawSlot s[SLOTS];
  };

  void init() {
    for (int sl = 0; sl < SLOTS; sl++) {
      for (int w = 0; w < 4; w++) bias[sl][w].assign(BIAS_SETS, 0);
      for (int g = 0; g < SC_GEHL_N; g++) gehl[g][sl].assign(GEHL_N, 0);
      imli_tbl[sl].assign(IMLI_N, 0);
      loc_tbl[sl].assign(LOC_N, 0);
    }
    std::memset(lht, 0, sizeof(lht));
    for (int i = 0; i < 16; i++) thres[i] = 35;
  }

  static int8_t sat_upd(int8_t v, bool taken) {
    int nv = int(v) + (taken ? 1 : -1);
    if (nv > 31) nv = 31;
    if (nv < -32) nv = -32;
    return static_cast<int8_t>(nv);
  }

  void read(const ScIdx &ix, Raw &r) const {
    const uint32_t lh = lht[ix.lht_idx];
    const uint32_t li = (ix.w ^ lh ^ (lh >> 6)) & (LOC_N - 1);
    for (int sl = 0; sl < SLOTS; sl++) {
      for (int way = 0; way < 4; way++) r.s[sl].bias[way] = bias[sl][way][ix.bias_set];
      for (int g = 0; g < SC_GEHL_N; g++) r.s[sl].gehl[g] = gehl[g][sl][ix.gehl_idx[g]];
      r.s[sl].imli = imli_tbl[sl][ix.imli_idx];
      r.s[sl].loc = loc_tbl[sl][li];
    }
  }

  static int eval_raw(const RawSlot &r, bool tage_pred, bool prov_hit, uint8_t prov_cnt, uint8_t base_cnt) {
    const int tage_w = 2 * (prov_hit ? (2 * int(prov_cnt) - 7) : (2 * int(base_cnt) - 3));
    const bool low_conf = prov_hit ? (prov_cnt == 3 || prov_cnt == 4) : (base_cnt == 1 || base_cnt == 2);
    const uint32_t bway = (uint32_t(tage_pred) << 1) | uint32_t(low_conf);
    int s = tage_w + (2 * int(r.bias[bway]) + 1);
    for (int g = 0; g < SC_GEHL_N; g++) s += (2 * int(r.gehl[g]) + 1);
    s += (2 * int(r.imli) + 1);
    s += (2 * int(r.loc) + 1);
    return s;
  }

  bool select_slot(uint32_t w, const RawSlot &r, bool tage_pred, bool prov_hit, uint8_t prov_cnt, uint8_t base_cnt) const {
    const int lsum = eval_raw(r, tage_pred, prov_hit, prov_cnt, base_cnt);
    const bool sc_pred = (lsum >= 0);
    const int th = thres[(w >> 2) & 15u];
    if (sc_pred != tage_pred && std::abs(lsum) >= th) return sc_pred;
    return tage_pred;
  }

  void update(const ScIdx &ix, int sl, bool real_dir, bool tage_pred, bool prov_hit, uint8_t prov_cnt, uint8_t base_cnt) {
    if (sl < 0 || sl >= SLOTS) return;
    const uint32_t lh = lht[ix.lht_idx];
    const uint32_t li = (ix.w ^ lh ^ (lh >> 6)) & (LOC_N - 1);
    RawSlot r;
    for (int way = 0; way < 4; way++) r.bias[way] = bias[sl][way][ix.bias_set];
    for (int g = 0; g < SC_GEHL_N; g++) r.gehl[g] = gehl[g][sl][ix.gehl_idx[g]];
    r.imli = imli_tbl[sl][ix.imli_idx];
    r.loc = loc_tbl[sl][li];

    const int lsum = eval_raw(r, tage_pred, prov_hit, prov_cnt, base_cnt);
    const bool sc_pred = (lsum >= 0);
    int &th = thres[(ix.w >> 2) & 15u];

    if (sc_pred != tage_pred && std::abs(lsum) >= th - 4 && std::abs(lsum) <= th + 4) {
      if (sc_pred == real_dir) { if (th > 5) th--; }
      else { if (th < 63) th++; }
    }

    if (sc_pred != real_dir || std::abs(lsum) < th + 18) {
      const bool low_conf = prov_hit ? (prov_cnt == 3 || prov_cnt == 4) : (base_cnt == 1 || base_cnt == 2);
      const uint32_t bway = (uint32_t(tage_pred) << 1) | uint32_t(low_conf);
      bias[sl][bway][ix.bias_set] = sat_upd(bias[sl][bway][ix.bias_set], real_dir);
      for (int g = 0; g < SC_GEHL_N; g++) {
        gehl[g][sl][ix.gehl_idx[g]] = sat_upd(gehl[g][sl][ix.gehl_idx[g]], real_dir);
      }
      imli_tbl[sl][ix.imli_idx] = sat_upd(imli_tbl[sl][ix.imli_idx], real_dir);
      loc_tbl[sl][li] = sat_upd(loc_tbl[sl][li], real_dir);
    }

    lht[ix.lht_idx] = static_cast<uint16_t>(((lh << 1) | uint16_t(real_dir)) & 0x3ffu);
  }
};

}  // namespace bpuf

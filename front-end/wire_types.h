#ifndef FRONTEND_WIRE_TYPES_H
#define FRONTEND_WIRE_TYPES_H

#include <cstdint>

#include "config.h"
#include "BPU/BPU_configs.h"

constexpr int ceil_log2_u32(uint32_t value) {
  int width = 0;
  uint32_t threshold = 1;
  while (threshold < value) {
    threshold <<= 1;
    ++width;
  }
  return width;
}

using wire1_t = bool;
using wire2_t = uint8_t;
using wire3_t = uint8_t;
using wire4_t = uint8_t;
using wire5_t = uint8_t;
using wire6_t = uint8_t;
using wire7_t = uint8_t;
using wire8_t = uint8_t;
using wire9_t = uint16_t;
using wire10_t = uint16_t;
using wire11_t = uint16_t;
using wire12_t = uint16_t;
using wire13_t = uint16_t;
using wire14_t = uint16_t;
using wire15_t = uint16_t;
using wire16_t = uint16_t;
using wire30_t = uint32_t;
using wire32_t = uint32_t;
using wire64_t = uint64_t;

template <int Bits>
struct wire_for_bits;

template <>
struct wire_for_bits<1> {
  using type = wire1_t;
};
template <>
struct wire_for_bits<2> {
  using type = wire2_t;
};
template <>
struct wire_for_bits<3> {
  using type = wire3_t;
};
template <>
struct wire_for_bits<4> {
  using type = wire4_t;
};
template <>
struct wire_for_bits<5> {
  using type = wire5_t;
};
template <>
struct wire_for_bits<6> {
  using type = wire6_t;
};
template <>
struct wire_for_bits<7> {
  using type = wire7_t;
};
template <>
struct wire_for_bits<8> {
  using type = wire8_t;
};
template <>
struct wire_for_bits<9> {
  using type = wire9_t;
};
template <>
struct wire_for_bits<10> {
  using type = wire10_t;
};
template <>
struct wire_for_bits<11> {
  using type = wire11_t;
};
template <>
struct wire_for_bits<12> {
  using type = wire12_t;
};
template <>
struct wire_for_bits<13> {
  using type = wire13_t;
};
template <>
struct wire_for_bits<14> {
  using type = wire14_t;
};
template <>
struct wire_for_bits<15> {
  using type = wire15_t;
};
template <>
struct wire_for_bits<16> {
  using type = wire16_t;
};
template <>
struct wire_for_bits<30> {
  using type = wire30_t;
};
template <>
struct wire_for_bits<32> {
  using type = wire32_t;
};
template <>
struct wire_for_bits<64> {
  using type = wire64_t;
};

template <int Bits>
using wire_for_bits_t = typename wire_for_bits<Bits>::type;

using br_type_t = wire3_t;
using pcpn_t = wire3_t;
static constexpr int pcpn_t_BITS = 3;
using predecode_type_t = wire2_t;

using tage_tag_t = wire_for_bits_t<TAGE_TAG_WIDTH>;
static constexpr int tage_tag_t_BITS = TAGE_TAG_WIDTH;
using tage_idx_t = wire_for_bits_t<TAGE_IDX_WIDTH>;
static constexpr int tage_idx_t_BITS = TAGE_IDX_WIDTH;

using btb_tag_t = wire_for_bits_t<BTB_TAG_LEN>;
using btb_type_idx_t = wire_for_bits_t<ceil_log2_u32(BTB_TYPE_ENTRY_NUM)>;
using bht_idx_t = wire_for_bits_t<ceil_log2_u32(BHT_ENTRY_NUM)>;
using tc_idx_t = wire_for_bits_t<ceil_log2_u32(TC_ENTRY_NUM)>;
using bht_hist_t = wire_for_bits_t<ceil_log2_u32(BHT_ENTRY_NUM)>;

using tage_use_alt_ctr_t = wire_for_bits_t<TAGE_USE_ALT_CTR_BITS>;



using type_pred_tag_t = wire_for_bits_t<TYPE_PRED_TAG_WIDTH>;
using type_pred_conf_t = wire_for_bits_t<TYPE_PRED_CONF_BITS>;
using type_pred_age_t = wire_for_bits_t<TYPE_PRED_AGE_BITS>;
// virtual addresses / PCs are register wide (Sv39 on rv64: kernel text is at 0xffffffff80000000)
using fetch_addr_t = wire_for_bits_t<XLEN>;
using pc_t = wire_for_bits_t<XLEN>;
using inst_word_t = wire32_t;
using target_addr_t = wire_for_bits_t<XLEN>;
using fetch_addr_fifo_size_t = wire_for_bits_t<ceil_log2_u32(FETCH_ADDR_FIFO_SIZE + 1)>;
using instruction_fifo_size_t =
    wire_for_bits_t<ceil_log2_u32(INSTRUCTION_FIFO_SIZE + 1)>;
static constexpr int instruction_fifo_size_t_BITS =
    ceil_log2_u32(INSTRUCTION_FIFO_SIZE + 1);
using ptab_size_t = wire_for_bits_t<ceil_log2_u32(PTAB_SIZE + 1)>;
using front2back_fifo_size_t =
    wire_for_bits_t<ceil_log2_u32(FRONT2BACK_FIFO_SIZE + 1)>;
static constexpr int front2back_fifo_size_t_BITS =
    ceil_log2_u32(FRONT2BACK_FIFO_SIZE + 1);

// Prediction/training metadata that travels with a fetch group from the BPU
// through PTAB and the front2back FIFO to the back end (all lanes of one group).
template <int N>
struct PredMetaT {
  wire1_t alt_pred[N];
  pcpn_t pcpn[N];
  tage_idx_t tage_idx[N][TN_MAX];
  tage_tag_t tage_tag[N][TN_MAX];
  wire16_t blk_id[N];  // BPA_REPAIR: sequence number of the fetch block (key of the BPU's checkpoint record)
  wire8_t ps_info[N];  // statistics only (BPA_STATS, docs/BPU_ACC.md): predicted type / target source / use-alt of the slot; no consumer in the pipeline

  // Copy the first M lanes of a narrower bundle; the other lanes keep their value.
  template <int M>
  void assign_prefix(const PredMetaT<M> &s) {
    static_assert(M <= N, "source bundle wider than destination");
    for (int i = 0; i < M; i++) {
      alt_pred[i] = s.alt_pred[i];
      pcpn[i] = s.pcpn[i];
      for (int j = 0; j < TN_MAX; j++) {
        tage_idx[i][j] = s.tage_idx[i][j];
        tage_tag[i][j] = s.tage_tag[i][j];
      }
      ps_info[i] = s.ps_info[i];
      blk_id[i] = s.blk_id[i];
    }
  }

  // Fields are stored at their declared hardware widths.
  void mask() {
    constexpr uint32_t kPcpnMask = (1u << pcpn_t_BITS) - 1u;
    constexpr uint32_t kTageIdxMask = (1u << tage_idx_t_BITS) - 1u;
    constexpr uint32_t kTageTagMask = (1u << tage_tag_t_BITS) - 1u;
    for (int i = 0; i < N; i++) {
      pcpn[i] = static_cast<pcpn_t>(pcpn[i] & kPcpnMask);
      for (int j = 0; j < TN_MAX; j++) {
        tage_idx[i][j] = static_cast<tage_idx_t>(tage_idx[i][j] & kTageIdxMask);
        tage_tag[i][j] = static_cast<tage_tag_t>(tage_tag[i][j] & kTageTagMask);
      }
    }
  }
};using PredMeta = PredMetaT<FETCH_WIDTH>;


#endif

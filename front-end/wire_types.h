#ifndef FRONTEND_WIRE_TYPES_H
#define FRONTEND_WIRE_TYPES_H

#include <cstdint>

#include "config.h"
#include "base_types.h"   // wire<N> / reg<N>: the narrowest integer type of N bits
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


using br_type_t = wire<3>;
using pcpn_t = wire<3>;
static constexpr int pcpn_t_BITS = 3;
using predecode_type_t = wire<2>;

using tage_tag_t = wire<TAGE_TAG_WIDTH>;
static constexpr int tage_tag_t_BITS = TAGE_TAG_WIDTH;
using tage_idx_t = wire<TAGE_IDX_WIDTH>;
static constexpr int tage_idx_t_BITS = TAGE_IDX_WIDTH;

using btb_tag_t = wire<BTB_TAG_LEN>;
using btb_type_idx_t = wire<ceil_log2_u32(BTB_TYPE_ENTRY_NUM)>;
using bht_idx_t = wire<ceil_log2_u32(BHT_ENTRY_NUM)>;
using tc_idx_t = wire<ceil_log2_u32(TC_ENTRY_NUM)>;
using bht_hist_t = wire<ceil_log2_u32(BHT_ENTRY_NUM)>;

using tage_use_alt_ctr_t = wire<TAGE_USE_ALT_CTR_BITS>;



using type_pred_tag_t = wire<TYPE_PRED_TAG_WIDTH>;
using type_pred_conf_t = wire<TYPE_PRED_CONF_BITS>;
using type_pred_age_t = wire<TYPE_PRED_AGE_BITS>;
// virtual addresses / PCs are register wide (Sv39 on rv64: kernel text is at 0xffffffff80000000)
using fetch_addr_t = wire<XLEN>;
using pc_t = wire<XLEN>;
using inst_word_t = wire<32>;
using target_addr_t = wire<XLEN>;
using fetch_addr_fifo_size_t = wire<ceil_log2_u32(FETCH_ADDR_FIFO_SIZE + 1)>;
using instruction_fifo_size_t =
    wire<ceil_log2_u32(INSTRUCTION_FIFO_SIZE + 1)>;
static constexpr int instruction_fifo_size_t_BITS =
    ceil_log2_u32(INSTRUCTION_FIFO_SIZE + 1);
using ptab_size_t = wire<ceil_log2_u32(PTAB_SIZE + 1)>;
using front2back_fifo_size_t =
    wire<ceil_log2_u32(FRONT2BACK_FIFO_SIZE + 1)>;
static constexpr int front2back_fifo_size_t_BITS =
    ceil_log2_u32(FRONT2BACK_FIFO_SIZE + 1);

// Prediction/training metadata that travels with a fetch group from the BPU
// through PTAB and the front2back FIFO to the back end (all lanes of one group).
template <int N>
struct PredMetaT {
  wire<1> alt_pred[N];
  pcpn_t pcpn[N];
  wire<16> blk_id[N];  // BPA_REPAIR: sequence number of the fetch block (key of the BPU's checkpoint record)
  wire<8> ps_info[N];  // statistics only (BPA_STATS, docs/BPU_ACC.md): predicted type / target source / use-alt of the slot; no consumer in the pipeline

  // Copy the first M lanes of a narrower bundle; the other lanes keep their value.
  template <int M>
  void assign_prefix(const PredMetaT<M> &s) {
    static_assert(M <= N, "source bundle wider than destination");
    for (int i = 0; i < M; i++) {
      alt_pred[i] = s.alt_pred[i];
      pcpn[i] = s.pcpn[i];
      ps_info[i] = s.ps_info[i];
      blk_id[i] = s.blk_id[i];
    }
  }

  // Fields are stored at their declared hardware widths.
  void mask() {
    constexpr uint32_t kPcpnMask = (1u << pcpn_t_BITS) - 1u;
    for (int i = 0; i < N; i++) {
      pcpn[i] = static_cast<pcpn_t>(pcpn[i] & kPcpnMask);
    }
  }
};using PredMeta = PredMetaT<FETCH_WIDTH>;


#endif

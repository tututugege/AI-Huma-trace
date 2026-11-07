#ifndef BPU_CONFIGS_H
#define BPU_CONFIGS_H

#include "../config/frontend_feature_config.h"

static_assert(BPU_BANK_NUM >= FETCH_WIDTH,
              "BPU_BANK_NUM must be >= FETCH_WIDTH");
static_assert(BPU_BANK_NUM >= COMMIT_WIDTH,
              "BPU_BANK_NUM must be >= COMMIT_WIDTH");
static_assert(TN_MAX == 4,
              "TN_MAX is fixed to 4 by design; do not change TAGE history level count");
static_assert(BPU_BANK_NUM > 0, "BPU_BANK_NUM must be > 0");
static_assert(BPU_TYPE_ENTRY_NUM > 0, "BPU_TYPE_ENTRY_NUM must be > 0");
static_assert(is_power_of_two_u64(BPU_TYPE_ENTRY_NUM),
              "BPU_TYPE_ENTRY_NUM must be power of two");
static_assert(BPU_TYPE_ENTRY_NUM <= (1u << 16),
              "BPU_TYPE_ENTRY_NUM too large for current wire_for_bits_t coverage");
static_assert(BASE_ENTRY_NUM > 0, "BASE_ENTRY_NUM must be > 0");
static_assert(is_power_of_two_u64(BASE_ENTRY_NUM),
              "BASE_ENTRY_NUM must be power of two");
static_assert(TN_ENTRY_NUM > 0, "TN_ENTRY_NUM must be > 0");
static_assert(is_power_of_two_u64(TN_ENTRY_NUM),
              "TN_ENTRY_NUM must be power of two");
static_assert(TAGE_BASE_IDX_WIDTH == clog2(BASE_ENTRY_NUM),
              "TAGE_BASE_IDX_WIDTH must match BASE_ENTRY_NUM");
static_assert(TAGE_IDX_WIDTH == clog2(TN_ENTRY_NUM),
              "TAGE_IDX_WIDTH must match TN_ENTRY_NUM");
static_assert(TAGE_BASE_IDX_WIDTH > 0 && TAGE_BASE_IDX_WIDTH <= 16,
              "TAGE_BASE_IDX_WIDTH must be in [1, 16]");
static_assert(TAGE_IDX_WIDTH > 0 && TAGE_IDX_WIDTH <= 16,
              "TAGE_IDX_WIDTH must be in [1, 16] with current tage_idx_t typing");




#define BPU_TYPE_IDX_MASK (BPU_TYPE_ENTRY_NUM - 1)




#if (TYPE_PRED_ENTRY_NUM % TYPE_PRED_WAY_NUM) != 0
#error "TYPE_PRED_ENTRY_NUM must be divisible by TYPE_PRED_WAY_NUM"
#endif

#if TYPE_PRED_ENTRY_NUM <= 0 || (TYPE_PRED_ENTRY_NUM & (TYPE_PRED_ENTRY_NUM - 1)) != 0
#error "TYPE_PRED_ENTRY_NUM must be power of two and > 0"
#endif

#define TYPE_PRED_SET_NUM (TYPE_PRED_ENTRY_NUM / TYPE_PRED_WAY_NUM)

#if (TYPE_PRED_SET_NUM & (TYPE_PRED_SET_NUM - 1)) != 0
#error "TYPE_PRED_SET_NUM must be power of two"
#endif

#if TYPE_PRED_SET_NUM < 2
#error "TYPE_PRED_SET_NUM must be >= 2 with current set-index typing assumptions"
#endif

#if TYPE_PRED_SET_NUM > (1 << 16)
#error "TYPE_PRED_SET_NUM too large for current wire_for_bits_t coverage"
#endif




#define TYPE_PRED_SET_MASK (TYPE_PRED_SET_NUM - 1)
#define TYPE_PRED_CONF_MAX ((1 << TYPE_PRED_CONF_BITS) - 1)
#define TYPE_PRED_AGE_MAX ((1 << TYPE_PRED_AGE_BITS) - 1)
#define TYPE_PRED_TAG_MASK ((1 << TYPE_PRED_TAG_WIDTH) - 1)



#define TAGE_SC_IDX_MASK (TAGE_SC_ENTRY_NUM - 1)





#define TAGE_SC_PATH_MASK ((1u << TAGE_SC_PATH_BITS) - 1u)












static_assert(BTB_TYPE_ENTRY_NUM > 0, "BTB_TYPE_ENTRY_NUM must be > 0");
static_assert(is_power_of_two_u64(BTB_TYPE_ENTRY_NUM),
              "BTB_TYPE_ENTRY_NUM must be power of two");
static_assert(BTB_TYPE_ENTRY_NUM <= (1u << 16),
              "BTB_TYPE_ENTRY_NUM too large for current btb_type_idx_t typing");
static_assert(BHT_ENTRY_NUM > 0, "BHT_ENTRY_NUM must be > 0");
static_assert(is_power_of_two_u64(BHT_ENTRY_NUM),
              "BHT_ENTRY_NUM must be power of two");
static_assert(BHT_ENTRY_NUM <= (1u << 16),
              "BHT_ENTRY_NUM too large for current bht_idx_t/bht_hist_t typing");
static_assert(TC_ENTRY_NUM > 0, "TC_ENTRY_NUM must be > 0");
static_assert(is_power_of_two_u64(TC_ENTRY_NUM),
              "TC_ENTRY_NUM must be power of two");
static_assert(TC_ENTRY_NUM <= (1u << 16),
              "TC_ENTRY_NUM too large for current tc_idx_t typing");

#define TAGE_TAG_MASK ((1 << TAGE_TAG_WIDTH) - 1)
#define TAGE_IDX_MASK ((1 << TAGE_IDX_WIDTH) - 1)
#define TAGE_BASE_IDX_MASK ((1 << TAGE_BASE_IDX_WIDTH) - 1)







#define BTB_IDX_MASK    (BTB_ENTRY_NUM - 1)
#define BTB_TAG_MASK    ((1 << BTB_TAG_LEN) - 1)
#define BTB_TYPE_IDX_MASK (BTB_TYPE_ENTRY_NUM - 1)
#define BHT_IDX_MASK    (BHT_ENTRY_NUM - 1)
#define TC_ENTRY_MASK   (TC_ENTRY_NUM - 1)
#define TC_TAG_MASK     ((1 << TC_TAG_LEN) - 1)

// Branch Types
#define BR_DIRECT 0 // only cond now
#define BR_CALL 1
#define BR_RET 2 // can merge...
#define BR_IDIRECT 3
#define BR_NONCTL 4
#define BR_JAL 5
#define BR_ICALL 6 // BPA_ICALL (docs/BPU_ACC.md): jalr with rd = ra, not a return: indirect target (TC) plus a RAS push

#define NLP_CONF_MAX ((1 << NLP_CONF_BITS) - 1)

#if NLP_CONF_THRESHOLD > NLP_CONF_MAX
#error "NLP_CONF_THRESHOLD must be <= NLP_CONF_MAX"
#endif

#define AHEAD_SLOT1_CONF_MAX ((1 << AHEAD_SLOT1_CONF_BITS) - 1)

#if AHEAD_SLOT1_CONF_THRESHOLD > AHEAD_SLOT1_CONF_MAX
#error "AHEAD_SLOT1_CONF_THRESHOLD must be <= AHEAD_SLOT1_CONF_MAX"
#endif





#endif

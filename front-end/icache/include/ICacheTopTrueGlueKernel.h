#pragma once

#include <stdint.h>

#ifndef ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH
#define ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH 16
#endif

#ifndef ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_WORDS
#define ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_WORDS 16
#endif

#ifndef ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_SIZE
#define ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_SIZE 64
#endif

#ifndef ICACHE_TOP_TRUE_GLUE_KERNEL_INST_NOP
#define ICACHE_TOP_TRUE_GLUE_KERNEL_INST_NOP 0x00000013u
#endif

#define ICACHE_TOP_TRUE_GLUE_KERNEL_PI_WIDTH                                  \
  (1 + 1 + 1 + 1 + 1 + 1 + 3 + 1 + 1 + 1 + 32 +                              \
   ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_WORDS * 32 + 1 + 1)
#define ICACHE_TOP_TRUE_GLUE_KERNEL_PO_WIDTH                                  \
  (4 + ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH * 32 +                         \
   ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH +                                  \
   ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH +                                  \
   ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH * 32 +                             \
   ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH +                                  \
   ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH + 32 + 32)

typedef struct {
  int pos;
} ICacheTopGlueCursor;

static inline uint32_t
icache_top_glue_read_uint(const bool *bits, ICacheTopGlueCursor *cur,
                          int width) {
  uint32_t value = 0;
  for (int i = 0; i < width; ++i) {
    if (bits[cur->pos++]) {
      value |= (1u << i);
    }
  }
  return value;
}

static inline uint32_t
icache_top_glue_read_bit(const bool *bits, ICacheTopGlueCursor *cur) {
  return bits[cur->pos++] ? 1u : 0u;
}

static inline void icache_top_glue_write_uint(bool *bits,
                                              ICacheTopGlueCursor *cur,
                                              uint32_t value, int width) {
  for (int i = 0; i < width; ++i) {
    bits[cur->pos++] = ((value >> i) & 1u) != 0;
  }
}

static inline void icache_top_glue_write_bit(bool *bits,
                                             ICacheTopGlueCursor *cur,
                                             uint32_t value) {
  bits[cur->pos++] = (value & 1u) != 0;
}

static inline void icache_top_true_glue_kernel(const bool *pi, bool *po) {
  ICacheTopGlueCursor r = {0};
  const uint32_t reset = icache_top_glue_read_bit(pi, &r);
  const uint32_t hold_for_recovery = icache_top_glue_read_bit(pi, &r);
  const uint32_t reg_ifu_req_ready = icache_top_glue_read_bit(pi, &r);
  const uint32_t reg_lookup_pending = icache_top_glue_read_bit(pi, &r);
  const uint32_t reg_req_valid = icache_top_glue_read_bit(pi, &r);
  const uint32_t reg_miss_txid_valid = icache_top_glue_read_bit(pi, &r);
  const uint32_t reg_state = icache_top_glue_read_uint(pi, &r, 3);
  const uint32_t reg_mem_axi_state = icache_top_glue_read_uint(pi, &r, 1);
  const uint32_t module_ifu_req_ready = icache_top_glue_read_bit(pi, &r);
  const uint32_t module_ifu_resp_valid = icache_top_glue_read_bit(pi, &r);
  const uint32_t module_ifu_resp_pc = icache_top_glue_read_uint(pi, &r, 32);
  uint32_t module_rd_data[ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_WORDS];
  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_WORDS; ++i) {
    module_rd_data[i] = icache_top_glue_read_uint(pi, &r, 32);
  }
  const uint32_t module_ifu_page_fault =
      icache_top_glue_read_bit(pi, &r);
  const uint32_t module_ifu_resp_single_word =
      icache_top_glue_read_bit(pi, &r);

  uint32_t icache_read_ready = 0;
  uint32_t icache_read_complete = 0;
  uint32_t icache_read_ready_2 = 0;
  uint32_t icache_read_complete_2 = 0;
  uint32_t fetch_group[ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH];
  uint32_t page_fault_inst[ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH];
  uint32_t inst_valid[ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH];
  uint32_t fetch_group_2[ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH];
  uint32_t page_fault_inst_2[ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH];
  uint32_t inst_valid_2[ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH];
  uint32_t fetch_pc = 0;
  uint32_t fetch_pc_2 = 0;

  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
    fetch_group[i] = ICACHE_TOP_TRUE_GLUE_KERNEL_INST_NOP;
    page_fault_inst[i] = 0;
    inst_valid[i] = 0;
    fetch_group_2[i] = ICACHE_TOP_TRUE_GLUE_KERNEL_INST_NOP;
    page_fault_inst_2[i] = 0;
    inst_valid_2[i] = 0;
  }

  if (reset) {
    icache_read_ready = 1;
  } else {
    const uint32_t local_busy =
        reg_lookup_pending || reg_req_valid || reg_miss_txid_valid ||
        (reg_state != 0u) || (reg_mem_axi_state != 0u);
    icache_read_ready =
        hold_for_recovery
            ? 0u
            : (reg_ifu_req_ready && !local_busy && module_ifu_req_ready);

    if (module_ifu_resp_valid) {
      icache_read_complete = 1;
      fetch_pc = module_ifu_resp_pc;
      const uint32_t mask = ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_SIZE - 1u;
      const int base_idx = (int)((module_ifu_resp_pc & mask) / 4u);
      if (module_ifu_resp_single_word) {
        fetch_group[0] = module_ifu_page_fault
                             ? ICACHE_TOP_TRUE_GLUE_KERNEL_INST_NOP
                             : module_rd_data[base_idx];
        page_fault_inst[0] = module_ifu_page_fault;
        inst_valid[0] = 1;
      } else {
        for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
          if (base_idx + i >= ICACHE_TOP_TRUE_GLUE_KERNEL_LINE_WORDS) {
            fetch_group[i] = ICACHE_TOP_TRUE_GLUE_KERNEL_INST_NOP;
            page_fault_inst[i] = 0;
            inst_valid[i] = 0;
          } else {
            fetch_group[i] = module_ifu_page_fault
                                 ? ICACHE_TOP_TRUE_GLUE_KERNEL_INST_NOP
                                 : module_rd_data[base_idx + i];
            page_fault_inst[i] = module_ifu_page_fault;
            inst_valid[i] = 1;
          }
        }
      }
    }
  }

  ICacheTopGlueCursor w = {0};
  icache_top_glue_write_bit(po, &w, icache_read_ready);
  icache_top_glue_write_bit(po, &w, icache_read_complete);
  icache_top_glue_write_bit(po, &w, icache_read_ready_2);
  icache_top_glue_write_bit(po, &w, icache_read_complete_2);
  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
    icache_top_glue_write_uint(po, &w, fetch_group[i], 32);
  }
  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
    icache_top_glue_write_bit(po, &w, page_fault_inst[i]);
  }
  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
    icache_top_glue_write_bit(po, &w, inst_valid[i]);
  }
  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
    icache_top_glue_write_uint(po, &w, fetch_group_2[i], 32);
  }
  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
    icache_top_glue_write_bit(po, &w, page_fault_inst_2[i]);
  }
  for (int i = 0; i < ICACHE_TOP_TRUE_GLUE_KERNEL_FETCH_WIDTH; ++i) {
    icache_top_glue_write_bit(po, &w, inst_valid_2[i]);
  }
  icache_top_glue_write_uint(po, &w, fetch_pc, 32);
  icache_top_glue_write_uint(po, &w, fetch_pc_2, 32);
}

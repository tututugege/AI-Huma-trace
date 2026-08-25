#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef ICACHE_COMPACT_KERNEL_LINE_SIZE
#ifdef ICACHE_LINE_SIZE
#define ICACHE_COMPACT_KERNEL_LINE_SIZE ICACHE_LINE_SIZE
#else
#define ICACHE_COMPACT_KERNEL_LINE_SIZE 64
#endif
#endif

#ifndef ICACHE_COMPACT_KERNEL_LOOKUP_LATENCY
#ifdef ICACHE_LOOKUP_LATENCY
#define ICACHE_COMPACT_KERNEL_LOOKUP_LATENCY ICACHE_LOOKUP_LATENCY
#else
#define ICACHE_COMPACT_KERNEL_LOOKUP_LATENCY 0
#endif
#endif

#define ICACHE_COMPACT_KERNEL_WAYS 8
#define ICACHE_COMPACT_KERNEL_WORD_BITS 32
#define ICACHE_COMPACT_KERNEL_WORD_BYTES 4
#define ICACHE_COMPACT_KERNEL_WORD_NUM                                      \
  (ICACHE_COMPACT_KERNEL_LINE_SIZE / ICACHE_COMPACT_KERNEL_WORD_BYTES)
#define ICACHE_COMPACT_KERNEL_OFFSET_BITS 6
#define ICACHE_COMPACT_KERNEL_INDEX_BITS                                    \
  (12 - ICACHE_COMPACT_KERNEL_OFFSET_BITS)
#define ICACHE_COMPACT_KERNEL_SET_NUM (1u << ICACHE_COMPACT_KERNEL_INDEX_BITS)
#define ICACHE_COMPACT_KERNEL_TAG_BITS 20
#define ICACHE_COMPACT_KERNEL_WAY_BITS 3
#define ICACHE_COMPACT_KERNEL_WORD_INDEX_BITS 4
#define ICACHE_COMPACT_KERNEL_LINE_DATA_WIDTH                               \
  (ICACHE_COMPACT_KERNEL_WORD_NUM * ICACHE_COMPACT_KERNEL_WORD_BITS)

#define ICACHE_COMPACT_KERNEL_REGS_WIDTH 1163
#define ICACHE_COMPACT_KERNEL_INPUT_WIDTH 616
#define ICACHE_COMPACT_KERNEL_LOOKUP_IN_WIDTH 525
#define ICACHE_COMPACT_KERNEL_PI_WIDTH 2304
#define ICACHE_COMPACT_KERNEL_OUT_WIDTH 661
#define ICACHE_COMPACT_KERNEL_TABLE_WRITE_WIDTH 544
#define ICACHE_COMPACT_KERNEL_PO_WIDTH 2368

#define ICACHE_COMPACT_MMIO_REQ_PI_WIDTH 52
#define ICACHE_COMPACT_MMIO_REQ_PO_WIDTH 110
#define ICACHE_COMPACT_DCACHE_PROBE_CTRL_PI_WIDTH 63
#define ICACHE_COMPACT_DCACHE_PROBE_CTRL_PO_WIDTH 53
#define ICACHE_COMPACT_DCACHE_PROBE_DATA_WORD_PI_WIDTH 73
#define ICACHE_COMPACT_DCACHE_PROBE_DATA_WORD_PO_WIDTH 64
#define ICACHE_COMPACT_REFILL_DATA_WORD_PI_WIDTH 73
#define ICACHE_COMPACT_REFILL_DATA_WORD_PO_WIDTH 32
#define ICACHE_COMPACT_REFILL_REPLACE_CTRL_PI_WIDTH 8
#define ICACHE_COMPACT_REFILL_REPLACE_CTRL_PO_WIDTH 7
#define ICACHE_COMPACT_FINISH_REFILL_CTRL_PI_WIDTH 38
#define ICACHE_COMPACT_FINISH_REFILL_CTRL_PO_WIDTH 50
#define ICACHE_COMPACT_LOOKUP_CTRL_PI_WIDTH 110
#define ICACHE_COMPACT_LOOKUP_CTRL_PO_WIDTH 89
#define ICACHE_COMPACT_MEM_RESP_CLASSIFY_PI_WIDTH 51
#define ICACHE_COMPACT_MEM_RESP_CLASSIFY_PO_WIDTH 14
#define ICACHE_COMPACT_FREE_TXID_ALLOC_PI_WIDTH 16
#define ICACHE_COMPACT_FREE_TXID_ALLOC_PO_WIDTH 5
#define ICACHE_COMPACT_LOOKUP_PHASE_CTRL_PI_WIDTH 16
#define ICACHE_COMPACT_LOOKUP_PHASE_CTRL_PO_WIDTH 9
#define ICACHE_COMPACT_IDLE_CTRL_PI_WIDTH 32
#define ICACHE_COMPACT_IDLE_CTRL_PO_WIDTH 32
#define ICACHE_COMPACT_SWAP_KILL_CTRL_PI_WIDTH 10
#define ICACHE_COMPACT_SWAP_KILL_CTRL_PO_WIDTH 26
#define ICACHE_COMPACT_SWAP_REQ_HANDSHAKE_CTRL_PI_WIDTH 13
#define ICACHE_COMPACT_SWAP_REQ_HANDSHAKE_CTRL_PO_WIDTH 17
#define ICACHE_COMPACT_CANCEL_WAIT_ACCEPT_CTRL_PI_WIDTH 10
#define ICACHE_COMPACT_CANCEL_WAIT_ACCEPT_CTRL_PO_WIDTH 27
#define ICACHE_COMPACT_DRAIN_CTRL_PI_WIDTH 7
#define ICACHE_COMPACT_DRAIN_CTRL_PO_WIDTH 28
#define ICACHE_COMPACT_WAIT_DCACHE_CTRL_PI_WIDTH 3
#define ICACHE_COMPACT_WAIT_DCACHE_CTRL_PO_WIDTH 15
#define ICACHE_COMPACT_SWAP_RESP_CTRL_PI_WIDTH 9
#define ICACHE_COMPACT_SWAP_RESP_CTRL_PO_WIDTH 23
#define ICACHE_COMPACT_MEM_REQ_ID_CTRL_PI_WIDTH 10
#define ICACHE_COMPACT_MEM_REQ_ID_CTRL_PO_WIDTH 4
#define ICACHE_COMPACT_MMU_VTAG_CTRL_PI_WIDTH 98
#define ICACHE_COMPACT_MMU_VTAG_CTRL_PO_WIDTH 20
#define ICACHE_COMPACT_MEM_RESP_READY_CTRL_PI_WIDTH 52
#define ICACHE_COMPACT_MEM_RESP_READY_CTRL_PO_WIDTH 1
#define ICACHE_COMPACT_MEM_REQ_VALID_CTRL_PI_WIDTH 14
#define ICACHE_COMPACT_MEM_REQ_VALID_CTRL_PO_WIDTH 1
#define ICACHE_COMPACT_PPN_READY_CTRL_PI_WIDTH 4
#define ICACHE_COMPACT_PPN_READY_CTRL_PO_WIDTH 1
#define ICACHE_COMPACT_STATE_IDLE_OR_INVALID_CTRL_PI_WIDTH 3
#define ICACHE_COMPACT_STATE_IDLE_OR_INVALID_CTRL_PO_WIDTH 1
#define ICACHE_COMPACT_AND2_CTRL_PI_WIDTH 2
#define ICACHE_COMPACT_AND2_CTRL_PO_WIDTH 1

#ifndef ICACHE_COMPACT_KERNEL_MMIO_UNCACHE_LIMIT
#ifdef CONFIG_ICACHE_MMIO_UNCACHE_LIMIT
#define ICACHE_COMPACT_KERNEL_MMIO_UNCACHE_LIMIT                              \
  CONFIG_ICACHE_MMIO_UNCACHE_LIMIT
#else
#define ICACHE_COMPACT_KERNEL_MMIO_UNCACHE_LIMIT 0x40000000u
#endif
#endif

#define ICACHE_COMPACT_KERNEL_MEM_REQ_CACHELINE_SIZE                          \
  (ICACHE_COMPACT_KERNEL_LINE_SIZE - 1u)
#define ICACHE_COMPACT_KERNEL_MEM_REQ_WORD_SIZE 3u

enum {
  ICACHE_COMPACT_STATE_IDLE = 0,
  ICACHE_COMPACT_STATE_SWAP_IN = 1,
  ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM = 2,
  ICACHE_COMPACT_STATE_DRAIN = 3,
  ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT = 4,
};

typedef struct {
  uint32_t req_valid_r;
  uint32_t req_pc_r;
  uint32_t req_index_r;
  uint32_t ifu_req_ready_r;
  uint32_t state;
  uint32_t mem_axi_state;
  uint32_t mem_resp_data_r[ICACHE_COMPACT_KERNEL_WORD_NUM];
  uint32_t replace_idx;
  uint32_t ppn_r;
  uint32_t miss_txid_valid_r;
  uint32_t miss_txid_r;
  uint32_t miss_ready_seen_r;
  uint32_t txid_inflight_r[16];
  uint32_t txid_canceled_r[16];
  uint32_t dcache_probe_inflight_r;
  uint32_t dcache_probe_done_r;
  uint32_t dcache_probe_hit_r;
  uint32_t dcache_probe_word_r;
  uint32_t dcache_probe_data_r[ICACHE_COMPACT_KERNEL_WORD_NUM];
  uint32_t lookup_pending_r;
  uint32_t lookup_index_r;
  uint32_t lookup_pc_r;
} ICacheCompactRegs;

typedef struct {
  uint32_t pc;
  uint32_t ifu_req_valid;
  uint32_t refetch;
  uint32_t flush;
  uint32_t ppn;
  uint32_t ppn_valid;
  uint32_t page_fault;
  uint32_t mem_req_ready;
  uint32_t mem_req_accepted;
  uint32_t mem_req_accepted_id;
  uint32_t mem_resp_valid;
  uint32_t mem_resp_id;
  uint32_t mem_resp_data[ICACHE_COMPACT_KERNEL_WORD_NUM];
  uint32_t dcache_resp_valid;
  uint32_t dcache_resp_retry;
  uint32_t dcache_resp_miss;
  uint32_t dcache_resp_data;
} ICacheCompactInput;

typedef struct {
  uint32_t meta_resp_valid;
  uint32_t hit_valid;
  uint32_t hit_way;
  uint32_t has_invalid_way;
  uint32_t first_invalid_way;
  uint32_t data_resp_valid;
  uint32_t data_resp_way;
  uint32_t data_resp_line[ICACHE_COMPACT_KERNEL_WORD_NUM];
} ICacheCompactLookup;

typedef struct {
  uint32_t miss;
  uint32_t ifu_resp_valid;
  uint32_t ifu_req_ready;
  uint32_t ifu_resp_pc;
  uint32_t rd_data[ICACHE_COMPACT_KERNEL_WORD_NUM];
  uint32_t ifu_page_fault;
  uint32_t ifu_resp_single_word;
  uint32_t ppn_ready;
  uint32_t mmu_req_valid;
  uint32_t mmu_req_vtag;
  uint32_t mem_req_valid;
  uint32_t mem_req_addr;
  uint32_t mem_req_total_size;
  uint32_t mem_req_bypass;
  uint32_t mem_req_id;
  uint32_t mem_resp_ready;
  uint32_t dcache_req_valid;
  uint32_t dcache_req_addr;
  uint32_t lookup_data_req_valid;
  uint32_t lookup_data_req_index;
  uint32_t lookup_data_req_way;
} ICacheCompactOutput;

typedef struct {
  uint32_t we;
  uint32_t invalidate_all;
  uint32_t index;
  uint32_t way;
  uint32_t data[ICACHE_COMPACT_KERNEL_WORD_NUM];
  uint32_t tag;
  uint32_t valid;
} ICacheCompactTableWrite;

typedef struct {
  uint32_t done;
  uint32_t hit;
  uint32_t line[ICACHE_COMPACT_KERNEL_WORD_NUM];
} ICacheCompactDcacheProbeStep;

typedef struct {
  ICacheCompactRegs regs;
  ICacheCompactInput in;
  ICacheCompactLookup lookup;
  ICacheCompactRegs reg_write;
  ICacheCompactOutput out;
  ICacheCompactTableWrite table_write;
  uint32_t allow_lookup_data_phase;
  uint32_t req_ready_w;
  uint32_t lookup_hit_valid_w;
  uint32_t lookup_data_ready_w;
  uint32_t lookup_hit_way_w;
  uint32_t lookup_has_invalid_way_w;
  uint32_t lookup_first_invalid_way_w;
  uint32_t lookup_hit_data_w[ICACHE_COMPACT_KERNEL_WORD_NUM];
  uint32_t mem_resp_data_w[ICACHE_COMPACT_KERNEL_WORD_NUM];
  uint32_t state_next;
  uint32_t mem_axi_state_next;
  uint32_t replace_idx_next;
  uint32_t fast_bypass_fire;
  uint32_t fast_bypass_from_pending;
  uint32_t mem_gnt;
} ICacheCompactContext;

typedef struct {
  int pos;
} ICacheCompactCursor;

static inline uint32_t icache_compact_mask(int width) {
  return width >= 32 ? 0xffffffffu : ((1u << width) - 1u);
}

static inline uint32_t icache_compact_read_uint(const bool *bits, int lsb,
                                                int width) {
  uint32_t value = 0;
  for (int i = 0; i < width && i < 32; ++i) {
    if (bits[lsb + i]) {
      value |= (1u << i);
    }
  }
  return value;
}

static inline uint32_t icache_compact_read_bit(const bool *bits, int bit) {
  return bits[bit] ? 1u : 0u;
}

static inline void icache_compact_write_uint(bool *bits, int lsb, int width,
                                             uint32_t value) {
  for (int i = 0; i < width; ++i) {
    bits[lsb + i] = ((value >> i) & 1u) != 0;
  }
}

static inline void icache_compact_write_bit(bool *bits, int bit,
                                            uint32_t value) {
  bits[bit] = (value & 1u) != 0;
}

static inline void icache_compact_clear(bool *bits, int width) {
  for (int i = 0; i < width; ++i) {
    bits[i] = false;
  }
}

static inline uint32_t icache_compact_index_from_pc(uint32_t pc) {
  return (pc >> ICACHE_COMPACT_KERNEL_OFFSET_BITS) &
         (ICACHE_COMPACT_KERNEL_SET_NUM - 1u);
}

static inline uint32_t icache_compact_line_base_addr(uint32_t ppn,
                                                     uint32_t index) {
  return (ppn << 12) | (index << ICACHE_COMPACT_KERNEL_OFFSET_BITS);
}

static inline uint32_t icache_compact_fetch_word_addr(uint32_t ppn,
                                                      uint32_t pc) {
  return (ppn << 12) | (pc & 0xfffu);
}

static inline uint32_t
icache_compact_req_word_addr(const ICacheCompactRegs *regs) {
  return icache_compact_fetch_word_addr(regs->ppn_r, regs->req_pc_r);
}

static inline uint32_t icache_compact_paddr_is_mmio(uint32_t paddr) {
  return paddr < ICACHE_COMPACT_KERNEL_MMIO_UNCACHE_LIMIT;
}

static inline uint32_t
icache_compact_req_is_mmio(const ICacheCompactRegs *regs) {
  return icache_compact_paddr_is_mmio(
      icache_compact_req_word_addr(regs));
}

static inline uint32_t
icache_compact_req_mem_addr(const ICacheCompactRegs *regs) {
  return icache_compact_req_is_mmio(regs)
             ? icache_compact_req_word_addr(regs)
             : icache_compact_line_base_addr(regs->ppn_r, regs->req_index_r);
}

static inline uint32_t
icache_compact_req_total_size(const ICacheCompactRegs *regs) {
  return icache_compact_req_is_mmio(regs)
             ? ICACHE_COMPACT_KERNEL_MEM_REQ_WORD_SIZE
             : ICACHE_COMPACT_KERNEL_MEM_REQ_CACHELINE_SIZE;
}

static inline void icache_compact_mmio_req_kernel(const bool *pi, bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_MMIO_REQ_PO_WIDTH);
  const uint32_t req_pc = icache_compact_read_uint(pi, 0, 32);
  const uint32_t ppn = icache_compact_read_uint(pi, 32, 20);
  const uint32_t index = icache_compact_index_from_pc(req_pc);
  const uint32_t word_addr = icache_compact_fetch_word_addr(ppn, req_pc);
  const uint32_t line_base = icache_compact_line_base_addr(ppn, index);
  const uint32_t is_mmio = icache_compact_paddr_is_mmio(word_addr);
  const uint32_t mem_addr = is_mmio ? word_addr : line_base;
  const uint32_t total_size = is_mmio
                                  ? ICACHE_COMPACT_KERNEL_MEM_REQ_WORD_SIZE
                                  : ICACHE_COMPACT_KERNEL_MEM_REQ_CACHELINE_SIZE;
  const uint32_t word_idx =
      ((req_pc & (ICACHE_COMPACT_KERNEL_LINE_SIZE - 1u)) / 4u) %
      ICACHE_COMPACT_KERNEL_WORD_NUM;

  int bit = 0;
  icache_compact_write_bit(po, bit++, is_mmio);
  icache_compact_write_uint(po, bit, 32, word_addr);
  bit += 32;
  icache_compact_write_uint(po, bit, 32, line_base);
  bit += 32;
  icache_compact_write_uint(po, bit, 32, mem_addr);
  bit += 32;
  icache_compact_write_uint(po, bit, 8, total_size);
  bit += 8;
  icache_compact_write_bit(po, bit++, is_mmio);
  icache_compact_write_uint(po, bit, 4, word_idx);
}

static inline void icache_compact_dcache_probe_ctrl_kernel(const bool *pi,
                                                          bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_DCACHE_PROBE_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t req_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t req_pc = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t ppn = icache_compact_read_uint(pi, bit, 20);
  bit += 20;
  const uint32_t probe_inflight = icache_compact_read_bit(pi, bit++);
  const uint32_t probe_done = icache_compact_read_bit(pi, bit++);
  const uint32_t probe_hit = icache_compact_read_bit(pi, bit++);
  const uint32_t probe_word = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t dcache_resp_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t dcache_resp_retry = icache_compact_read_bit(pi, bit++);
  const uint32_t dcache_resp_miss = icache_compact_read_bit(pi, bit++);

  const uint32_t index = icache_compact_index_from_pc(req_pc);
  const uint32_t word_addr = icache_compact_fetch_word_addr(ppn, req_pc);
  const uint32_t is_mmio = icache_compact_paddr_is_mmio(word_addr);
  const uint32_t line_base = icache_compact_line_base_addr(ppn, index);
  const uint32_t word_in_range = probe_word < ICACHE_COMPACT_KERNEL_WORD_NUM;

  uint32_t step_done = probe_done;
  uint32_t step_hit = probe_hit;
  uint32_t reg_inflight = probe_inflight;
  uint32_t reg_done = probe_done;
  uint32_t reg_hit = probe_hit;
  uint32_t reg_word = probe_word;
  uint32_t dcache_req_valid = 0;
  uint32_t dcache_req_addr = 0;
  uint32_t resp_data_write_valid = 0;
  uint32_t resp_data_write_word = 0;
  uint32_t mmio_skip = 0;
  uint32_t start_req = 0;
  uint32_t consume_resp = 0;
  uint32_t resp_miss = 0;
  uint32_t resp_hit_last_word = 0;
  uint32_t resp_hit_advance = 0;

  if (req_valid && is_mmio) {
    step_done = 1;
    step_hit = 0;
    reg_inflight = 0;
    reg_done = 1;
    reg_hit = 0;
    reg_word = 0;
    mmio_skip = 1;
  } else {
    if (!probe_done && !probe_inflight && word_in_range && req_valid) {
      dcache_req_valid = 1;
      dcache_req_addr = line_base + probe_word * 4u;
      reg_inflight = 1;
      start_req = 1;
    }

    if (probe_inflight && dcache_resp_valid) {
      reg_inflight = 0;
      consume_resp = 1;
      if (dcache_resp_retry) {
        /* Keep the current probe word and reissue after the transient conflict. */
      } else if (dcache_resp_miss) {
        step_done = 1;
        step_hit = 0;
        reg_done = 1;
        reg_hit = 0;
        resp_miss = 1;
      } else {
        step_hit = 1;
        reg_hit = 1;
        resp_data_write_valid = word_in_range;
        resp_data_write_word = probe_word;
        if (probe_word + 1u >= ICACHE_COMPACT_KERNEL_WORD_NUM) {
          step_done = 1;
          reg_done = 1;
          resp_hit_last_word = 1;
        } else {
          reg_word = probe_word + 1u;
          resp_hit_advance = 1;
        }
      }
    }
  }

  bit = 0;
  icache_compact_write_bit(po, bit++, step_done);
  icache_compact_write_bit(po, bit++, step_hit);
  icache_compact_write_bit(po, bit++, reg_inflight);
  icache_compact_write_bit(po, bit++, reg_done);
  icache_compact_write_bit(po, bit++, reg_hit);
  icache_compact_write_uint(po, bit, 4, reg_word);
  bit += 4;
  icache_compact_write_bit(po, bit++, dcache_req_valid);
  icache_compact_write_uint(po, bit, 32, dcache_req_addr);
  bit += 32;
  icache_compact_write_bit(po, bit++, resp_data_write_valid);
  icache_compact_write_uint(po, bit, 4, resp_data_write_word);
  bit += 4;
  icache_compact_write_bit(po, bit++, mmio_skip);
  icache_compact_write_bit(po, bit++, start_req);
  icache_compact_write_bit(po, bit++, consume_resp);
  icache_compact_write_bit(po, bit++, resp_miss);
  icache_compact_write_bit(po, bit++, resp_hit_last_word);
  icache_compact_write_bit(po, bit++, resp_hit_advance);
}

static inline void icache_compact_dcache_probe_data_word_kernel(const bool *pi,
                                                               bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_DCACHE_PROBE_DATA_WORD_PO_WIDTH);

  int bit = 0;
  const uint32_t old_word = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t resp_data = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t write_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t write_word = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t this_word = icache_compact_read_uint(pi, bit, 4);

  const uint32_t selected =
      write_valid && ((write_word & 0xfu) == (this_word & 0xfu));
  const uint32_t next_word = selected ? resp_data : old_word;

  bit = 0;
  icache_compact_write_uint(po, bit, 32, next_word);
  bit += 32;
  icache_compact_write_uint(po, bit, 32, next_word);
}

static inline void icache_compact_refill_data_word_kernel(const bool *pi,
                                                         bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_REFILL_DATA_WORD_PO_WIDTH);

  int bit = 0;
  const uint32_t is_mmio = icache_compact_read_bit(pi, bit++);
  const uint32_t mmio_word_idx = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t this_word = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t mem_resp_word0 = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t mem_resp_this_word = icache_compact_read_uint(pi, bit, 32);

  const uint32_t selected_word =
      is_mmio ? (((mmio_word_idx & 0xfu) == (this_word & 0xfu))
                     ? mem_resp_word0
                     : 0u)
              : mem_resp_this_word;
  icache_compact_write_uint(po, 0, 32, selected_word);
}

static inline void icache_compact_refill_replace_ctrl_kernel(const bool *pi,
                                                            bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_REFILL_REPLACE_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t is_mmio = icache_compact_read_bit(pi, bit++);
  const uint32_t replace_idx = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  const uint32_t has_invalid_way = icache_compact_read_bit(pi, bit++);
  const uint32_t first_invalid_way = icache_compact_read_uint(pi, bit, 3);

  uint32_t next = replace_idx & 0x7u;
  uint32_t used_invalid = 0;
  uint32_t used_round_robin = 0;
  if (!is_mmio) {
    if (has_invalid_way && first_invalid_way < ICACHE_COMPACT_KERNEL_WAYS) {
      next = first_invalid_way & 0x7u;
      used_invalid = 1;
    } else {
      next = ((replace_idx & 0x7u) + 1u) % ICACHE_COMPACT_KERNEL_WAYS;
      used_round_robin = 1;
    }
  }

  bit = 0;
  icache_compact_write_uint(po, bit, 3, next);
  bit += 3;
  icache_compact_write_bit(po, bit++, used_invalid);
  icache_compact_write_bit(po, bit++, used_round_robin);
  icache_compact_write_bit(po, bit++, is_mmio);
  icache_compact_write_bit(po, bit++, !is_mmio);
}

static inline void icache_compact_finish_refill_ctrl_kernel(const bool *pi,
                                                           bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_FINISH_REFILL_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t flush = icache_compact_read_bit(pi, bit++);
  const uint32_t refetch = icache_compact_read_bit(pi, bit++);
  const uint32_t is_mmio = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t req_index = icache_compact_read_uint(pi, bit, 6);
  bit += 6;
  const uint32_t ppn = icache_compact_read_uint(pi, bit, 20);
  bit += 20;
  const uint32_t replace_way = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  const uint32_t line_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t allow_ifu_resp = !refetch && !flush;
  const uint32_t table_we = (!flush && !is_mmio && line_valid) ? 1u : 0u;

  bit = 0;
  icache_compact_write_uint(po, bit, 3, ICACHE_COMPACT_STATE_IDLE);
  bit += 3;
  icache_compact_write_bit(po, bit++, 0); /* mem_axi_state_next */
  icache_compact_write_bit(po, bit++, miss_txid_valid);
  icache_compact_write_uint(po, bit, 4, miss_txid & 0xfu);
  bit += 4;
  icache_compact_write_bit(po, bit++, 0); /* miss_txid_valid_next */
  icache_compact_write_bit(po, bit++, 0); /* miss_ready_seen_next */
  icache_compact_write_bit(po, bit++, 1); /* reset_probe_regs */
  icache_compact_write_bit(po, bit++, table_we);
  icache_compact_write_uint(po, bit, 6, req_index);
  bit += 6;
  icache_compact_write_uint(po, bit, 3, replace_way & 0x7u);
  bit += 3;
  icache_compact_write_uint(po, bit, 20, ppn & 0xfffffu);
  bit += 20;
  icache_compact_write_bit(po, bit++, table_we); /* table_valid */
  icache_compact_write_bit(po, bit++, allow_ifu_resp);
  icache_compact_write_bit(po, bit++, is_mmio && allow_ifu_resp);
  icache_compact_write_bit(po, bit++, 1); /* req_ready */
  icache_compact_write_bit(po, bit++, !flush);
  icache_compact_write_bit(po, bit++, !refetch);
  icache_compact_write_bit(po, bit++, line_valid);
  icache_compact_write_bit(po, bit++, is_mmio);
}

static inline void icache_compact_lookup_ctrl_kernel(const bool *pi,
                                                     bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_LOOKUP_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t kill_pipe = icache_compact_read_bit(pi, bit++);
  const uint32_t fast_bypass_fire = icache_compact_read_bit(pi, bit++);
  const uint32_t fast_bypass_from_pending = icache_compact_read_bit(pi, bit++);
  const uint32_t req_ready_w_in = icache_compact_read_bit(pi, bit++);
  const uint32_t ifu_req_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t pc = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t lookup_meta_resp_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t ppn_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t req_valid_r = icache_compact_read_bit(pi, bit++);
  const uint32_t req_pc_r = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t state = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid_valid_r = icache_compact_read_bit(pi, bit++);
  const uint32_t lookup_pending_r = icache_compact_read_bit(pi, bit++);
  const uint32_t lookup_pc_r = icache_compact_read_uint(pi, bit, 32);

  const uint32_t use_external_lookup =
      ICACHE_COMPACT_KERNEL_LOOKUP_LATENCY > 0;
  const uint32_t slot_idle =
      !req_valid_r && state == ICACHE_COMPACT_STATE_IDLE &&
      mem_axi_state == 0 && !miss_txid_valid_r;
  const uint32_t index_pc = icache_compact_index_from_pc(pc);

  uint32_t req_ready_w = req_ready_w_in;
  uint32_t lookup_pending_next = lookup_pending_r;
  uint32_t lookup_pc_next = lookup_pc_r;
  uint32_t sram_load_fire = 0;
  uint32_t load_pc = lookup_pending_r ? lookup_pc_r : pc;
  uint32_t req_valid_next = req_valid_r;
  uint32_t req_pc_next = req_pc_r;
  uint32_t ifu_req_ready_next = req_ready_w_in;
  uint32_t mmu_req_valid = 0;
  uint32_t mmu_req_vtag = 0;

  if (kill_pipe) {
    req_ready_w = 0;
    req_valid_next = 0;
    lookup_pending_next = 0;
    lookup_pc_next = 0;
    req_pc_next = req_pc_r;
    ifu_req_ready_next = 0;
    mmu_req_valid = 0;
    mmu_req_vtag = 0;
  } else if (fast_bypass_fire) {
    req_valid_next = 0;
    lookup_pending_next = 0;
    lookup_pc_next = 0;
    req_pc_next = req_pc_r;
    ifu_req_ready_next = !fast_bypass_from_pending;
    mmu_req_valid = 0;
    mmu_req_vtag = 0;
  } else {
    const uint32_t can_accept =
        ifu_req_valid && req_ready_w && !lookup_pending_r && slot_idle;
    if (can_accept) {
      if (!use_external_lookup) {
        sram_load_fire = 1;
      } else {
        lookup_pending_next = 1;
        lookup_pc_next = pc;
        if (lookup_meta_resp_valid) {
          sram_load_fire = 1;
          lookup_pending_next = 0;
        }
      }
    }

    if (use_external_lookup && lookup_pending_r && lookup_meta_resp_valid &&
        slot_idle) {
      sram_load_fire = 1;
      lookup_pending_next = 0;
    }

    if (sram_load_fire) {
      req_valid_next = 1;
    } else if (req_ready_w) {
      req_valid_next = 0;
    }

    ifu_req_ready_next =
        req_ready_w && !lookup_pending_next && !req_valid_next;
    if (sram_load_fire) {
      req_pc_next = load_pc;
    }

    if (can_accept) {
      mmu_req_valid = 1;
      mmu_req_vtag = pc >> 12;
    } else if (lookup_pending_r && !ppn_valid) {
      mmu_req_valid = 1;
      mmu_req_vtag = lookup_pc_r >> 12;
    } else if (req_valid_r && !ppn_valid) {
      mmu_req_valid = 1;
      mmu_req_vtag = req_pc_r >> 12;
    } else {
      mmu_req_valid = 0;
      mmu_req_vtag = 0;
    }
    (void)index_pc;
  }

  bit = 0;
  icache_compact_write_bit(po, bit++, req_valid_next);
  icache_compact_write_bit(po, bit++, lookup_pending_next);
  icache_compact_write_uint(po, bit, 32, lookup_pc_next);
  bit += 32;
  icache_compact_write_bit(po, bit++, ifu_req_ready_next);
  icache_compact_write_uint(po, bit, 32, req_pc_next);
  bit += 32;
  icache_compact_write_bit(po, bit++, mmu_req_valid);
  icache_compact_write_uint(po, bit, 20, mmu_req_vtag);
  bit += 20;
  icache_compact_write_bit(po, bit++, req_ready_w);
}

static inline void icache_compact_mem_resp_classify_kernel(const bool *pi,
                                                          bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_MEM_RESP_CLASSIFY_PO_WIDTH);

  int bit = 0;
  const uint32_t mem_resp_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_resp_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t mem_req_accepted = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t state = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  uint32_t txid_inflight[16] = {0};
  uint32_t txid_canceled[16] = {0};
  for (int i = 0; i < 16; ++i) {
    txid_inflight[i] = icache_compact_read_bit(pi, bit++);
  }
  for (int i = 0; i < 16; ++i) {
    txid_canceled[i] = icache_compact_read_bit(pi, bit++);
  }

  const uint32_t resp_id = mem_resp_id & 0xfu;
  const uint32_t miss_id = miss_txid & 0xfu;
  const uint32_t canceled_resp = mem_resp_valid && txid_canceled[resp_id];
  const uint32_t same_cycle_accept_resp_candidate =
      mem_resp_valid && state == ICACHE_COMPACT_STATE_SWAP_IN &&
      mem_axi_state == 0 && miss_txid_valid && resp_id == miss_id &&
      mem_req_accepted && ((mem_req_accepted_id & 0xfu) == miss_id);
  const uint32_t active_resp =
      mem_resp_valid && miss_txid_valid && resp_id == miss_id &&
      (txid_inflight[resp_id] || same_cycle_accept_resp_candidate) &&
      !canceled_resp;
  const uint32_t orphan_resp =
      mem_resp_valid && !active_resp && !canceled_resp &&
      !txid_inflight[resp_id];
  const uint32_t passive_ready = canceled_resp || orphan_resp;
  const uint32_t clear_valid = passive_ready;
  const uint32_t clear_id = resp_id;

  bit = 0;
  icache_compact_write_uint(po, bit, 4, resp_id);
  bit += 4;
  icache_compact_write_bit(po, bit++, canceled_resp);
  icache_compact_write_bit(po, bit++, same_cycle_accept_resp_candidate);
  icache_compact_write_bit(po, bit++, active_resp);
  icache_compact_write_bit(po, bit++, orphan_resp);
  icache_compact_write_bit(po, bit++, passive_ready);
  icache_compact_write_bit(po, bit++, clear_valid);
  icache_compact_write_uint(po, bit, 4, clear_id);
}

static inline void icache_compact_free_txid_alloc_kernel(const bool *pi,
                                                        bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_FREE_TXID_ALLOC_PO_WIDTH);

  uint32_t valid = 0;
  uint32_t txid = 0;
  for (uint32_t id = 0; id < 16; ++id) {
    if (!valid && !icache_compact_read_bit(pi, (int)id)) {
      valid = 1;
      txid = id;
    }
  }

  icache_compact_write_bit(po, 0, valid);
  icache_compact_write_uint(po, 1, 4, txid);
}

static inline void icache_compact_lookup_phase_ctrl_kernel(const bool *pi,
                                                          bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_LOOKUP_PHASE_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t meta_resp_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t compare_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t hit_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t hit_way = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  const uint32_t has_invalid_way = icache_compact_read_bit(pi, bit++);
  const uint32_t first_invalid_way = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  const uint32_t allow_lookup_data_phase = icache_compact_read_bit(pi, bit++);
  const uint32_t data_resp_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t data_resp_way = icache_compact_read_uint(pi, bit, 3);

  uint32_t lookup_has_invalid_way = 0;
  uint32_t lookup_first_invalid_way = 0;
  uint32_t lookup_hit_valid = 0;
  uint32_t lookup_hit_way = 0;
  uint32_t lookup_data_ready = 0;

  if (meta_resp_valid) {
    lookup_has_invalid_way =
        has_invalid_way && first_invalid_way < ICACHE_COMPACT_KERNEL_WAYS;
    if (lookup_has_invalid_way) {
      lookup_first_invalid_way = first_invalid_way & 0x7u;
    }
    lookup_hit_valid =
        compare_valid && hit_valid && hit_way < ICACHE_COMPACT_KERNEL_WAYS;
    if (lookup_hit_valid) {
      lookup_hit_way = hit_way & 0x7u;
    }
  }

  if (allow_lookup_data_phase && lookup_hit_valid && data_resp_valid &&
      ((data_resp_way & 0x7u) == lookup_hit_way)) {
    lookup_data_ready = 1;
  }

  bit = 0;
  icache_compact_write_bit(po, bit++, lookup_has_invalid_way);
  icache_compact_write_uint(po, bit, 3, lookup_first_invalid_way);
  bit += 3;
  icache_compact_write_bit(po, bit++, lookup_hit_valid);
  icache_compact_write_uint(po, bit, 3, lookup_hit_way);
  bit += 3;
  icache_compact_write_bit(po, bit++, lookup_data_ready);
}

static inline void icache_compact_idle_ctrl_kernel(const bool *pi, bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_IDLE_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t kill_pipe = icache_compact_read_bit(pi, bit++);
  const uint32_t req_valid_r = icache_compact_read_bit(pi, bit++);
  const uint32_t ifu_req_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t ppn_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t page_fault = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t free_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t free_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t lookup_hit_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t lookup_data_ready = icache_compact_read_bit(pi, bit++);
  const uint32_t lookup_hit_way = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  const uint32_t fast_index = icache_compact_read_uint(
      pi, bit, ICACHE_COMPACT_KERNEL_INDEX_BITS);
  bit += ICACHE_COMPACT_KERNEL_INDEX_BITS;
  const uint32_t req_index = icache_compact_read_uint(
      pi, bit, ICACHE_COMPACT_KERNEL_INDEX_BITS);

  uint32_t state_next = ICACHE_COMPACT_STATE_IDLE;
  uint32_t req_ready_w = 0;
  uint32_t fast_bypass_fire = 0;
  uint32_t ifu_resp_valid = 0;
  uint32_t ifu_page_fault = 0;
  uint32_t ppn_ready = 0;
  uint32_t lookup_data_req_valid = 0;
  uint32_t lookup_data_req_index = 0;
  uint32_t lookup_data_req_way = 0;
  uint32_t ifu_resp_pc_use_input_pc = 0;
  uint32_t rd_data_zero = 0;
  uint32_t rd_data_use_lookup = 0;
  uint32_t reg_ppn_we = 0;
  uint32_t reset_probe_regs = 0;
  uint32_t miss_txid_valid_we = 0;
  uint32_t miss_txid_valid_next = miss_txid_valid;
  uint32_t miss_txid_next = miss_txid & 0xfu;
  uint32_t miss_ready_seen_we = 0;
  uint32_t miss_ready_seen_next = 0;
  uint32_t no_free_txid_stall = 0;

  if (kill_pipe) {
    req_ready_w = 1;
  } else {
    const uint32_t fast_lookup_path =
        ICACHE_COMPACT_KERNEL_LOOKUP_LATENCY == 0 && !req_valid_r &&
        ifu_req_valid && ppn_valid;
    uint32_t fast_path_consumed = 0;

    if (fast_lookup_path) {
      ifu_resp_pc_use_input_pc = 1;
      if (page_fault) {
        ifu_resp_valid = 1;
        ifu_page_fault = 1;
        req_ready_w = 1;
        fast_bypass_fire = 1;
        rd_data_zero = 1;
        fast_path_consumed = 1;
      } else if (lookup_hit_valid && lookup_data_ready) {
        ifu_resp_valid = 1;
        req_ready_w = 1;
        fast_bypass_fire = 1;
        rd_data_use_lookup = 1;
        fast_path_consumed = 1;
      } else if (lookup_hit_valid) {
        lookup_data_req_valid = 1;
        lookup_data_req_index = fast_index;
        lookup_data_req_way = lookup_hit_way & 0x7u;
        req_ready_w = 0;
        fast_path_consumed = 1;
      }
    }

    if (!fast_path_consumed) {
      if (req_valid_r && ppn_valid) {
        if (page_fault) {
          ifu_resp_valid = 1;
          ifu_page_fault = 1;
          req_ready_w = 1;
          rd_data_zero = 1;
        } else if (lookup_hit_valid && lookup_data_ready) {
          ifu_resp_valid = 1;
          req_ready_w = 1;
          rd_data_use_lookup = 1;
          ppn_ready = 1;
        } else if (lookup_hit_valid) {
          lookup_data_req_valid = 1;
          lookup_data_req_index = req_index;
          lookup_data_req_way = lookup_hit_way & 0x7u;
          req_ready_w = 0;
          ppn_ready = 1;
        } else {
          const uint32_t txid_valid = miss_txid_valid || free_txid_valid;
          const uint32_t txid = miss_txid_valid ? miss_txid : free_txid;
          ppn_ready = 1;
          if (!txid_valid) {
            req_ready_w = 0;
            no_free_txid_stall = 1;
          } else {
            state_next = ICACHE_COMPACT_STATE_SWAP_IN;
            req_ready_w = 0;
            reg_ppn_we = 1;
            reset_probe_regs = 1;
            miss_txid_valid_we = 1;
            miss_txid_valid_next = 1;
            miss_txid_next = txid & 0xfu;
            miss_ready_seen_we = 1;
            miss_ready_seen_next = 0;
          }
        }
      } else if (req_valid_r && !ppn_valid) {
        req_ready_w = 0;
        ppn_ready = 1;
      } else {
        req_ready_w = 1;
        ppn_ready = req_valid_r;
      }
    }
  }

  bit = 0;
  icache_compact_write_uint(po, bit, 3, state_next);
  bit += 3;
  icache_compact_write_bit(po, bit++, req_ready_w);
  icache_compact_write_bit(po, bit++, fast_bypass_fire);
  icache_compact_write_bit(po, bit++, ifu_resp_valid);
  icache_compact_write_bit(po, bit++, ifu_page_fault);
  icache_compact_write_bit(po, bit++, ppn_ready);
  icache_compact_write_bit(po, bit++, lookup_data_req_valid);
  icache_compact_write_uint(po, bit, ICACHE_COMPACT_KERNEL_INDEX_BITS,
                            lookup_data_req_index);
  bit += ICACHE_COMPACT_KERNEL_INDEX_BITS;
  icache_compact_write_uint(po, bit, 3, lookup_data_req_way);
  bit += 3;
  icache_compact_write_bit(po, bit++, ifu_resp_pc_use_input_pc);
  icache_compact_write_bit(po, bit++, rd_data_zero);
  icache_compact_write_bit(po, bit++, rd_data_use_lookup);
  icache_compact_write_bit(po, bit++, reg_ppn_we);
  icache_compact_write_bit(po, bit++, reset_probe_regs);
  icache_compact_write_bit(po, bit++, miss_txid_valid_we);
  icache_compact_write_bit(po, bit++, miss_txid_valid_next);
  icache_compact_write_uint(po, bit, 4, miss_txid_next);
  bit += 4;
  icache_compact_write_bit(po, bit++, miss_ready_seen_we);
  icache_compact_write_bit(po, bit++, miss_ready_seen_next);
  icache_compact_write_bit(po, bit++, no_free_txid_stall);
}

static inline void icache_compact_swap_kill_ctrl_kernel(const bool *pi,
                                                       bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_SWAP_KILL_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t kill_pipe = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t req_accepted_now = icache_compact_read_bit(pi, bit++);
  const uint32_t req_ready_seen_now = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_ready_seen = icache_compact_read_bit(pi, bit++);

  uint32_t handled = 0;
  uint32_t state_next = ICACHE_COMPACT_STATE_SWAP_IN;
  uint32_t mem_axi_state_next = mem_axi_state;
  uint32_t req_ready_w = 0;
  uint32_t reset_probe_regs = 0;
  uint32_t miss_txid_valid_we = 0;
  uint32_t miss_txid_valid_next = miss_txid_valid;
  uint32_t miss_ready_seen_we = 0;
  uint32_t miss_ready_seen_next = miss_ready_seen;
  uint32_t txid_cancel_we = 0;
  uint32_t txid_cancel_id = miss_txid & 0xfu;
  uint32_t txid_cancel_next = 0;
  uint32_t txid_inflight_we = 0;
  uint32_t txid_inflight_id = miss_txid & 0xfu;
  uint32_t txid_inflight_next = 0;
  uint32_t cancel_wait_accept = 0;
  uint32_t req_already_inflight = 0;
  uint32_t req_waiting_accept = 0;

  if (kill_pipe) {
    handled = 1;
    req_already_inflight =
        miss_txid_valid && (mem_axi_state != 0 || req_accepted_now);
    req_waiting_accept =
        miss_txid_valid && !req_already_inflight &&
        (miss_ready_seen || req_ready_seen_now);
    reset_probe_regs = 1;
    mem_axi_state_next = 0;

    if (req_already_inflight) {
      txid_cancel_we = 1;
      txid_cancel_next = 1;
      txid_inflight_we = 1;
      txid_inflight_next = 1;
      state_next = ICACHE_COMPACT_STATE_IDLE;
      miss_txid_valid_we = 1;
      miss_txid_valid_next = 0;
      miss_ready_seen_we = 1;
      miss_ready_seen_next = 0;
      req_ready_w = 1;
    } else if (req_waiting_accept) {
      state_next = ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT;
      cancel_wait_accept = 1;
      req_ready_w = 0;
    } else {
      state_next = ICACHE_COMPACT_STATE_IDLE;
      miss_txid_valid_we = 1;
      miss_txid_valid_next = 0;
      miss_ready_seen_we = 1;
      miss_ready_seen_next = 0;
      req_ready_w = 1;
    }
  }

  bit = 0;
  icache_compact_write_bit(po, bit++, handled);
  icache_compact_write_uint(po, bit, 3, state_next);
  bit += 3;
  icache_compact_write_bit(po, bit++, mem_axi_state_next);
  icache_compact_write_bit(po, bit++, req_ready_w);
  icache_compact_write_bit(po, bit++, reset_probe_regs);
  icache_compact_write_bit(po, bit++, miss_txid_valid_we);
  icache_compact_write_bit(po, bit++, miss_txid_valid_next);
  icache_compact_write_bit(po, bit++, miss_ready_seen_we);
  icache_compact_write_bit(po, bit++, miss_ready_seen_next);
  icache_compact_write_bit(po, bit++, txid_cancel_we);
  icache_compact_write_uint(po, bit, 4, txid_cancel_id);
  bit += 4;
  icache_compact_write_bit(po, bit++, txid_cancel_next);
  icache_compact_write_bit(po, bit++, txid_inflight_we);
  icache_compact_write_uint(po, bit, 4, txid_inflight_id);
  bit += 4;
  icache_compact_write_bit(po, bit++, txid_inflight_next);
  icache_compact_write_bit(po, bit++, cancel_wait_accept);
  icache_compact_write_bit(po, bit++, req_already_inflight);
  icache_compact_write_bit(po, bit++, req_waiting_accept);
}

static inline void icache_compact_swap_req_handshake_ctrl_kernel(
    const bool *pi, bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_SWAP_REQ_HANDSHAKE_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_ready = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t active_resp = icache_compact_read_bit(pi, bit++);

  const uint32_t miss_id = miss_txid & 0xfu;
  const uint32_t req_ready_seen_now =
      mem_axi_state == 0 && mem_req_ready && miss_txid_valid;
  const uint32_t req_accepted_now =
      mem_axi_state == 0 && mem_req_accepted &&
      ((mem_req_accepted_id & 0xfu) == miss_id) && miss_txid_valid;
  const uint32_t accepted_no_resp = req_accepted_now && !active_resp;
  const uint32_t mem_req_valid = (mem_axi_state == 0 && !req_accepted_now);
  const uint32_t mem_axi_state_next = accepted_no_resp ? 1u : mem_axi_state;
  const uint32_t txid_inflight_we = accepted_no_resp;
  const uint32_t txid_inflight_id = miss_id;
  const uint32_t txid_inflight_next = accepted_no_resp;
  const uint32_t miss_ready_seen_we = req_ready_seen_now;
  const uint32_t miss_ready_seen_next = req_ready_seen_now;
  const uint32_t mem_req_id = miss_id;

  bit = 0;
  icache_compact_write_bit(po, bit++, req_ready_seen_now);
  icache_compact_write_bit(po, bit++, req_accepted_now);
  icache_compact_write_bit(po, bit++, mem_req_valid);
  icache_compact_write_bit(po, bit++, accepted_no_resp);
  icache_compact_write_bit(po, bit++, mem_axi_state_next);
  icache_compact_write_bit(po, bit++, txid_inflight_we);
  icache_compact_write_uint(po, bit, 4, txid_inflight_id);
  bit += 4;
  icache_compact_write_bit(po, bit++, txid_inflight_next);
  icache_compact_write_bit(po, bit++, miss_ready_seen_we);
  icache_compact_write_bit(po, bit++, miss_ready_seen_next);
  icache_compact_write_uint(po, bit, 4, mem_req_id);
}

static inline void icache_compact_cancel_wait_accept_ctrl_kernel(
    const bool *pi, bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_CANCEL_WAIT_ACCEPT_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t mem_req_accepted = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);

  const uint32_t txid = miss_txid & 0xfu;
  const uint32_t accept_match =
      mem_req_accepted && miss_txid_valid &&
      ((mem_req_accepted_id & 0xfu) == txid);

  bit = 0;
  icache_compact_write_bit(po, bit++, 0); /* mem_req_valid */
  icache_compact_write_bit(po, bit++, 1); /* mem_resp_ready */
  icache_compact_write_bit(po, bit++, accept_match);
  icache_compact_write_bit(po, bit++, accept_match); /* txid_cancel_we */
  icache_compact_write_uint(po, bit, 4, txid);
  bit += 4;
  icache_compact_write_bit(po, bit++, 1); /* txid_cancel_next */
  icache_compact_write_bit(po, bit++, accept_match); /* txid_inflight_we */
  icache_compact_write_uint(po, bit, 4, txid);
  bit += 4;
  icache_compact_write_bit(po, bit++, 1); /* txid_inflight_next */
  icache_compact_write_uint(po, bit, 3, ICACHE_COMPACT_STATE_IDLE);
  bit += 3;
  icache_compact_write_bit(po, bit++, 0); /* mem_axi_state_next */
  icache_compact_write_bit(po, bit++, 1); /* miss_txid_valid_we */
  icache_compact_write_bit(po, bit++, 0); /* miss_txid_valid_next */
  icache_compact_write_bit(po, bit++, 1); /* miss_ready_seen_we */
  icache_compact_write_bit(po, bit++, 0); /* miss_ready_seen_next */
  icache_compact_write_bit(po, bit++, 1); /* reset_probe_regs */
  icache_compact_write_bit(po, bit++, 1); /* req_ready_w */
  icache_compact_write_bit(po, bit++, 1); /* handled */
}

static inline void icache_compact_drain_ctrl_kernel(const bool *pi,
                                                    bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_DRAIN_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t mem_resp_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);

  const uint32_t txid = miss_txid & 0xfu;
  const uint32_t done = mem_resp_valid;

  bit = 0;
  icache_compact_write_bit(po, bit++, 1); /* mem_resp_ready */
  icache_compact_write_bit(po, bit++, 0); /* mem_req_valid */
  icache_compact_write_uint(
      po, bit, 3, done ? ICACHE_COMPACT_STATE_IDLE : ICACHE_COMPACT_STATE_DRAIN);
  bit += 3;
  icache_compact_write_bit(po, bit++, done ? 0u : mem_axi_state);
  icache_compact_write_bit(po, bit++, done && miss_txid_valid);
  icache_compact_write_uint(po, bit, 4, txid);
  bit += 4;
  icache_compact_write_bit(po, bit++, 0); /* txid_inflight_next */
  icache_compact_write_bit(po, bit++, done && miss_txid_valid);
  icache_compact_write_uint(po, bit, 4, txid);
  bit += 4;
  icache_compact_write_bit(po, bit++, 0); /* txid_cancel_next */
  icache_compact_write_bit(po, bit++, done && miss_txid_valid);
  icache_compact_write_bit(po, bit++, 0); /* miss_txid_valid_next */
  icache_compact_write_bit(po, bit++, done);
  icache_compact_write_bit(po, bit++, 0); /* miss_ready_seen_next */
  icache_compact_write_bit(po, bit++, done); /* reset_probe_regs */
  icache_compact_write_bit(po, bit++, done ? 1u : 0u); /* req_ready_w */
  icache_compact_write_bit(po, bit++, done);
  icache_compact_write_bit(po, bit++, done && !miss_txid_valid);
  icache_compact_write_bit(po, bit++, !done);
  icache_compact_write_bit(po, bit++, 1); /* handled */
}

static inline void icache_compact_wait_dcache_ctrl_kernel(const bool *pi,
                                                         bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_WAIT_DCACHE_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t kill_pipe = icache_compact_read_bit(pi, bit++);
  const uint32_t dcache_step_done = icache_compact_read_bit(pi, bit++);
  const uint32_t dcache_step_hit = icache_compact_read_bit(pi, bit++);

  uint32_t state_next = ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM;
  uint32_t mem_axi_state_next = 0;
  uint32_t req_ready_w = 0;
  uint32_t reset_probe_regs = 0;
  uint32_t miss_txid_valid_we = 0;
  uint32_t miss_txid_valid_next = 0;
  uint32_t miss_ready_seen_we = 0;
  uint32_t miss_ready_seen_next = 0;
  uint32_t finish_refill_fire = 0;
  uint32_t finish_use_dcache_line = 0;
  uint32_t wait_stall = 0;
  uint32_t kill_path = 0;

  if (kill_pipe) {
    state_next = ICACHE_COMPACT_STATE_IDLE;
    req_ready_w = 1;
    reset_probe_regs = 1;
    miss_txid_valid_we = 1;
    miss_ready_seen_we = 1;
    kill_path = 1;
  } else if (!dcache_step_done) {
    state_next = ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM;
    req_ready_w = 0;
    wait_stall = 1;
  } else {
    finish_refill_fire = 1;
    finish_use_dcache_line = dcache_step_hit;
  }

  bit = 0;
  icache_compact_write_bit(po, bit++, 1); /* handled */
  icache_compact_write_uint(po, bit, 3, state_next);
  bit += 3;
  icache_compact_write_bit(po, bit++, mem_axi_state_next);
  icache_compact_write_bit(po, bit++, req_ready_w);
  icache_compact_write_bit(po, bit++, reset_probe_regs);
  icache_compact_write_bit(po, bit++, miss_txid_valid_we);
  icache_compact_write_bit(po, bit++, miss_txid_valid_next);
  icache_compact_write_bit(po, bit++, miss_ready_seen_we);
  icache_compact_write_bit(po, bit++, miss_ready_seen_next);
  icache_compact_write_bit(po, bit++, finish_refill_fire);
  icache_compact_write_bit(po, bit++, finish_use_dcache_line);
  icache_compact_write_bit(po, bit++, wait_stall);
  icache_compact_write_bit(po, bit++, kill_path);
}

static inline void icache_compact_swap_resp_ctrl_kernel(const bool *pi,
                                                       bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_SWAP_RESP_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t req_accepted_now = icache_compact_read_bit(pi, bit++);
  const uint32_t active_resp = icache_compact_read_bit(pi, bit++);
  const uint32_t dcache_step_done = icache_compact_read_bit(pi, bit++);
  const uint32_t dcache_step_hit = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);

  const uint32_t txid = miss_txid & 0xfu;
  const uint32_t same_cycle_fire = (mem_axi_state == 0) && req_accepted_now &&
                                   active_resp;
  const uint32_t stored_req_fire = (mem_axi_state != 0) && active_resp;
  const uint32_t fire = same_cycle_fire || stored_req_fire;
  const uint32_t wait_dcache = fire && !dcache_step_done;
  const uint32_t finish_refill_fire = fire && dcache_step_done;

  uint32_t state_next = ICACHE_COMPACT_STATE_SWAP_IN;
  uint32_t mem_axi_state_next = mem_axi_state;
  uint32_t req_ready_w = 0;
  uint32_t txid_inflight_we = 0;
  uint32_t txid_inflight_next = 0;
  uint32_t miss_txid_valid_we = 0;
  uint32_t miss_txid_valid_next = 0;
  uint32_t miss_ready_seen_we = 0;
  uint32_t miss_ready_seen_next = 0;

  if (wait_dcache) {
    state_next = ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM;
    mem_axi_state_next = 0;
    txid_inflight_we = 1;
    txid_inflight_next = 0;
    miss_txid_valid_we = 1;
    miss_ready_seen_we = 1;
  } else if (finish_refill_fire) {
    state_next = ICACHE_COMPACT_STATE_IDLE;
    mem_axi_state_next = 0;
    req_ready_w = 1;
  }

  bit = 0;
  icache_compact_write_bit(po, bit++, fire);
  icache_compact_write_bit(po, bit++, fire); /* mem_resp_ready */
  icache_compact_write_bit(po, bit++, fire); /* latch_mem_refill */
  icache_compact_write_uint(po, bit, 3, state_next);
  bit += 3;
  icache_compact_write_bit(po, bit++, mem_axi_state_next);
  icache_compact_write_bit(po, bit++, req_ready_w);
  icache_compact_write_bit(po, bit++, txid_inflight_we);
  icache_compact_write_uint(po, bit, 4, txid);
  bit += 4;
  icache_compact_write_bit(po, bit++, txid_inflight_next);
  icache_compact_write_bit(po, bit++, miss_txid_valid_we);
  icache_compact_write_bit(po, bit++, miss_txid_valid_next);
  icache_compact_write_bit(po, bit++, miss_ready_seen_we);
  icache_compact_write_bit(po, bit++, miss_ready_seen_next);
  icache_compact_write_bit(po, bit++, finish_refill_fire);
  icache_compact_write_bit(po, bit++, dcache_step_hit && finish_refill_fire);
  icache_compact_write_bit(po, bit++, wait_dcache);
  icache_compact_write_bit(po, bit++, same_cycle_fire);
  icache_compact_write_bit(po, bit++, stored_req_fire);
}

static inline void icache_compact_mem_req_id_ctrl_kernel(const bool *pi,
                                                         bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_MEM_REQ_ID_CTRL_PO_WIDTH);

  int bit = 0;
  uint32_t state = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  if (state > ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT) {
    state = ICACHE_COMPACT_STATE_IDLE;
  }
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t refetch = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);

  const uint32_t swap_issue =
      state == ICACHE_COMPACT_STATE_SWAP_IN && !mem_axi_state && !refetch;
  const uint32_t mem_req_id =
      (swap_issue || miss_txid_valid) ? (miss_txid & 0xfu) : 0u;
  icache_compact_write_uint(po, 0, 4, mem_req_id);
}

static inline void icache_compact_mmu_vtag_ctrl_kernel(const bool *pi,
                                                       bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_MMU_VTAG_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t lookup_pending = icache_compact_read_bit(pi, bit++);
  const uint32_t req_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t lookup_pc = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t req_pc = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  const uint32_t pc = icache_compact_read_uint(pi, bit, 32);

  uint32_t vtag;
  if (lookup_pending) {
    vtag = lookup_pc >> 12;
  } else if (req_valid) {
    vtag = req_pc >> 12;
  } else {
    vtag = pc >> 12;
  }
  icache_compact_write_uint(po, 0, 20, vtag);
}

static inline void icache_compact_mem_resp_ready_ctrl_kernel(const bool *pi,
                                                             bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_MEM_RESP_READY_CTRL_PO_WIDTH);

  int bit = 0;
  uint32_t state = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  if (state > ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT) {
    state = ICACHE_COMPACT_STATE_IDLE;
  }
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t refetch = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t mem_resp_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_resp_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t miss_txid_valid = icache_compact_read_bit(pi, bit++);
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  uint32_t txid_inflight[16] = {0};
  uint32_t txid_canceled[16] = {0};
  for (int i = 0; i < 16; ++i) {
    txid_inflight[i] = icache_compact_read_bit(pi, bit++);
  }
  for (int i = 0; i < 16; ++i) {
    txid_canceled[i] = icache_compact_read_bit(pi, bit++);
  }

  const uint32_t resp_id = mem_resp_id & 0xfu;
  const uint32_t miss_id = miss_txid & 0xfu;
  const uint32_t accepted_match =
      mem_req_accepted && ((mem_req_accepted_id & 0xfu) == miss_id);
  const uint32_t canceled_resp = mem_resp_valid && txid_canceled[resp_id];
  const uint32_t same_cycle_accept_resp_candidate =
      mem_resp_valid && state == ICACHE_COMPACT_STATE_SWAP_IN &&
      mem_axi_state == 0 && miss_txid_valid && resp_id == miss_id &&
      accepted_match;
  const uint32_t active_resp =
      mem_resp_valid && miss_txid_valid && resp_id == miss_id &&
      (txid_inflight[resp_id] || same_cycle_accept_resp_candidate) &&
      !canceled_resp;
  const uint32_t orphan_resp =
      mem_resp_valid && !active_resp && !canceled_resp &&
      !txid_inflight[resp_id];
  const uint32_t passive_ready = canceled_resp || orphan_resp;
  const uint32_t swap_ready =
      state == ICACHE_COMPACT_STATE_SWAP_IN && !refetch && active_resp &&
      (mem_axi_state != 0 || accepted_match);
  const uint32_t forced_ready =
      state == ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT ||
      state == ICACHE_COMPACT_STATE_DRAIN;

  icache_compact_write_bit(po, 0,
                           passive_ready || swap_ready || forced_ready);
}

static inline void icache_compact_mem_req_valid_ctrl_kernel(const bool *pi,
                                                            bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_MEM_REQ_VALID_CTRL_PO_WIDTH);

  int bit = 0;
  uint32_t state = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  if (state > ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT) {
    state = ICACHE_COMPACT_STATE_IDLE;
  }
  const uint32_t mem_axi_state = icache_compact_read_bit(pi, bit++);
  const uint32_t refetch = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted = icache_compact_read_bit(pi, bit++);
  const uint32_t mem_req_accepted_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  const uint32_t miss_txid = icache_compact_read_uint(pi, bit, 4);

  const uint32_t accepted_match =
      mem_req_accepted && ((mem_req_accepted_id & 0xfu) == (miss_txid & 0xfu));
  const uint32_t mem_req_valid =
      state == ICACHE_COMPACT_STATE_SWAP_IN && !refetch &&
      mem_axi_state == 0 && !accepted_match;

  icache_compact_write_bit(po, 0, mem_req_valid);
}

static inline void icache_compact_ppn_ready_ctrl_kernel(const bool *pi,
                                                        bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_PPN_READY_CTRL_PO_WIDTH);

  int bit = 0;
  const uint32_t req_valid = icache_compact_read_bit(pi, bit++);
  uint32_t state = icache_compact_read_uint(pi, bit, 3);
  if (state > ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT) {
    state = ICACHE_COMPACT_STATE_IDLE;
  }

  icache_compact_write_bit(po, 0,
                           state == ICACHE_COMPACT_STATE_IDLE && req_valid);
}

static inline void icache_compact_state_idle_or_invalid_ctrl_kernel(
    const bool *pi, bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_STATE_IDLE_OR_INVALID_CTRL_PO_WIDTH);

  const uint32_t raw_state = icache_compact_read_uint(pi, 0, 3);
  const uint32_t state =
      raw_state > ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT
          ? ICACHE_COMPACT_STATE_IDLE
          : raw_state;

  icache_compact_write_bit(po, 0, state == ICACHE_COMPACT_STATE_IDLE);
}

static inline void icache_compact_and2_ctrl_kernel(const bool *pi, bool *po) {
  icache_compact_clear(po, ICACHE_COMPACT_AND2_CTRL_PO_WIDTH);

  icache_compact_write_bit(po, 0,
                           icache_compact_read_bit(pi, 0) &&
                               icache_compact_read_bit(pi, 1));
}

static inline void icache_compact_decode_regs(const bool *pi,
                                              ICacheCompactRegs *regs) {
  int bit = 0;
  *regs = (ICacheCompactRegs){0};
  regs->req_valid_r = icache_compact_read_bit(pi, bit++);
  regs->req_pc_r = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  regs->req_index_r = icache_compact_index_from_pc(regs->req_pc_r);
  regs->ifu_req_ready_r = icache_compact_read_bit(pi, bit++);
  regs->state = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  if (regs->state > ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT) {
    regs->state = ICACHE_COMPACT_STATE_IDLE;
  }
  regs->mem_axi_state = icache_compact_read_bit(pi, bit++);
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    regs->mem_resp_data_r[word] = icache_compact_read_uint(pi, bit, 32);
    bit += 32;
  }
  regs->replace_idx = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  regs->ppn_r = icache_compact_read_uint(pi, bit, 20);
  bit += 20;
  regs->miss_txid_valid_r = icache_compact_read_bit(pi, bit++);
  regs->miss_txid_r = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  regs->miss_ready_seen_r = icache_compact_read_bit(pi, bit++);
  for (int i = 0; i < 16; ++i) {
    regs->txid_inflight_r[i] = icache_compact_read_bit(pi, bit++);
  }
  for (int i = 0; i < 16; ++i) {
    regs->txid_canceled_r[i] = icache_compact_read_bit(pi, bit++);
  }
  regs->dcache_probe_inflight_r = icache_compact_read_bit(pi, bit++);
  regs->dcache_probe_done_r = icache_compact_read_bit(pi, bit++);
  regs->dcache_probe_hit_r = icache_compact_read_bit(pi, bit++);
  regs->dcache_probe_word_r = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    regs->dcache_probe_data_r[word] = icache_compact_read_uint(pi, bit, 32);
    bit += 32;
  }
  regs->lookup_pending_r = icache_compact_read_bit(pi, bit++);
  regs->lookup_pc_r = icache_compact_read_uint(pi, bit, 32);
  regs->lookup_index_r = icache_compact_index_from_pc(regs->lookup_pc_r);
}

static inline void icache_compact_encode_regs(bool *po,
                                              const ICacheCompactRegs *regs) {
  int bit = 0;
  icache_compact_write_bit(po, bit++, regs->req_valid_r);
  icache_compact_write_uint(po, bit, 32, regs->req_pc_r);
  bit += 32;
  icache_compact_write_bit(po, bit++, regs->ifu_req_ready_r);
  icache_compact_write_uint(po, bit, 3, regs->state);
  bit += 3;
  icache_compact_write_bit(po, bit++, regs->mem_axi_state);
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    icache_compact_write_uint(po, bit, 32, regs->mem_resp_data_r[word]);
    bit += 32;
  }
  icache_compact_write_uint(po, bit, 3, regs->replace_idx);
  bit += 3;
  icache_compact_write_uint(po, bit, 20, regs->ppn_r);
  bit += 20;
  icache_compact_write_bit(po, bit++, regs->miss_txid_valid_r);
  icache_compact_write_uint(po, bit, 4, regs->miss_txid_r);
  bit += 4;
  icache_compact_write_bit(po, bit++, regs->miss_ready_seen_r);
  for (int i = 0; i < 16; ++i) {
    icache_compact_write_bit(po, bit++, regs->txid_inflight_r[i]);
  }
  for (int i = 0; i < 16; ++i) {
    icache_compact_write_bit(po, bit++, regs->txid_canceled_r[i]);
  }
  icache_compact_write_bit(po, bit++, regs->dcache_probe_inflight_r);
  icache_compact_write_bit(po, bit++, regs->dcache_probe_done_r);
  icache_compact_write_bit(po, bit++, regs->dcache_probe_hit_r);
  icache_compact_write_uint(po, bit, 4, regs->dcache_probe_word_r);
  bit += 4;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    icache_compact_write_uint(po, bit, 32, regs->dcache_probe_data_r[word]);
    bit += 32;
  }
  icache_compact_write_bit(po, bit++, regs->lookup_pending_r);
  icache_compact_write_uint(po, bit, 32, regs->lookup_pc_r);
}

static inline uint32_t
icache_compact_decode_input(const bool *pi, ICacheCompactInput *in) {
  int bit = 0;
  *in = (ICacheCompactInput){0};
  const uint32_t allow_lookup_data_phase = icache_compact_read_bit(pi, bit++);
  in->pc = icache_compact_read_uint(pi, bit, 32);
  bit += 32;
  in->ifu_req_valid = icache_compact_read_bit(pi, bit++);
  in->refetch = icache_compact_read_bit(pi, bit++);
  in->flush = icache_compact_read_bit(pi, bit++);
  in->ppn = icache_compact_read_uint(pi, bit, 20);
  bit += 20;
  in->ppn_valid = icache_compact_read_bit(pi, bit++);
  in->page_fault = icache_compact_read_bit(pi, bit++);
  in->mem_req_ready = icache_compact_read_bit(pi, bit++);
  in->mem_req_accepted = icache_compact_read_bit(pi, bit++);
  in->mem_req_accepted_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  in->mem_resp_valid = icache_compact_read_bit(pi, bit++);
  in->mem_resp_id = icache_compact_read_uint(pi, bit, 4);
  bit += 4;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    in->mem_resp_data[word] = icache_compact_read_uint(pi, bit, 32);
    bit += 32;
  }
  in->dcache_resp_valid = icache_compact_read_bit(pi, bit++);
  in->dcache_resp_retry = icache_compact_read_bit(pi, bit++);
  in->dcache_resp_miss = icache_compact_read_bit(pi, bit++);
  in->dcache_resp_data = icache_compact_read_uint(pi, bit, 32);
  return allow_lookup_data_phase;
}

static inline void icache_compact_decode_lookup(const bool *pi,
                                                ICacheCompactLookup *lookup) {
  int bit = 0;
  *lookup = (ICacheCompactLookup){0};
  lookup->meta_resp_valid = icache_compact_read_bit(pi, bit++);
  lookup->hit_valid = icache_compact_read_bit(pi, bit++);
  lookup->hit_way = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  lookup->has_invalid_way = icache_compact_read_bit(pi, bit++);
  lookup->first_invalid_way = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  lookup->data_resp_valid = icache_compact_read_bit(pi, bit++);
  lookup->data_resp_way = icache_compact_read_uint(pi, bit, 3);
  bit += 3;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    lookup->data_resp_line[word] = icache_compact_read_uint(pi, bit, 32);
    bit += 32;
  }
}

static inline void icache_compact_encode_out(bool *po,
                                             const ICacheCompactOutput *out) {
  int bit = 0;
  icache_compact_write_bit(po, bit++, out->miss);
  icache_compact_write_bit(po, bit++, out->ifu_resp_valid);
  icache_compact_write_bit(po, bit++, out->ifu_req_ready);
  icache_compact_write_uint(po, bit, 32, out->ifu_resp_pc);
  bit += 32;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    icache_compact_write_uint(po, bit, 32, out->rd_data[word]);
    bit += 32;
  }
  icache_compact_write_bit(po, bit++, out->ifu_page_fault);
  icache_compact_write_bit(po, bit++, out->ifu_resp_single_word);
  icache_compact_write_bit(po, bit++, out->ppn_ready);
  icache_compact_write_bit(po, bit++, out->mmu_req_valid);
  icache_compact_write_uint(po, bit, 20, out->mmu_req_vtag);
  bit += 20;
  icache_compact_write_bit(po, bit++, out->mem_req_valid);
  icache_compact_write_uint(po, bit, 32, out->mem_req_addr);
  bit += 32;
  icache_compact_write_uint(po, bit, 8, out->mem_req_total_size);
  bit += 8;
  icache_compact_write_bit(po, bit++, out->mem_req_bypass);
  icache_compact_write_uint(po, bit, 4, out->mem_req_id);
  bit += 4;
  icache_compact_write_bit(po, bit++, out->mem_resp_ready);
  icache_compact_write_bit(po, bit++, out->dcache_req_valid);
  icache_compact_write_uint(po, bit, 32, out->dcache_req_addr);
  bit += 32;
  icache_compact_write_bit(po, bit++, out->lookup_data_req_valid);
  icache_compact_write_uint(po, bit, ICACHE_COMPACT_KERNEL_INDEX_BITS,
                            out->lookup_data_req_index);
  bit += ICACHE_COMPACT_KERNEL_INDEX_BITS;
  icache_compact_write_uint(po, bit, 3, out->lookup_data_req_way);
}

static inline void
icache_compact_encode_table_write(bool *po,
                                  const ICacheCompactTableWrite *write) {
  int bit = 0;
  icache_compact_write_bit(po, bit++, write->we);
  icache_compact_write_bit(po, bit++, write->invalidate_all);
  icache_compact_write_uint(po, bit, ICACHE_COMPACT_KERNEL_INDEX_BITS,
                            write->index);
  bit += ICACHE_COMPACT_KERNEL_INDEX_BITS;
  icache_compact_write_uint(po, bit, 3, write->way);
  bit += 3;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    icache_compact_write_uint(po, bit, 32, write->data[word]);
    bit += 32;
  }
  icache_compact_write_uint(po, bit, 20, write->tag);
  bit += 20;
  icache_compact_write_bit(po, bit++, write->valid);
}

static inline uint32_t
icache_compact_req_slot_idle(const ICacheCompactRegs *regs) {
  return !regs->req_valid_r && regs->state == ICACHE_COMPACT_STATE_IDLE &&
         regs->mem_axi_state == 0 && !regs->miss_txid_valid_r;
}

static inline int icache_compact_alloc_free_txid(const ICacheCompactRegs *regs) {
  for (int id = 0; id < 16; ++id) {
    if (!regs->txid_inflight_r[id]) {
      return id;
    }
  }
  return -1;
}

static inline void icache_compact_capture_lookup_meta(ICacheCompactContext *ctx,
                                                      uint32_t compare_tag,
                                                      uint32_t compare_valid) {
  ctx->lookup_hit_valid_w = 0;
  ctx->lookup_data_ready_w = 0;
  ctx->lookup_hit_way_w = 0;
  ctx->lookup_has_invalid_way_w = 0;
  ctx->lookup_first_invalid_way_w = 0;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    ctx->lookup_hit_data_w[word] = 0;
  }

  if (!ctx->lookup.meta_resp_valid) {
    return;
  }
  ctx->lookup_has_invalid_way_w =
      ctx->lookup.has_invalid_way &&
      ctx->lookup.first_invalid_way < ICACHE_COMPACT_KERNEL_WAYS;
  if (ctx->lookup_has_invalid_way_w) {
    ctx->lookup_first_invalid_way_w = ctx->lookup.first_invalid_way;
  }
  ctx->lookup_hit_valid_w =
      compare_valid && ctx->lookup.hit_valid &&
      ctx->lookup.hit_way < ICACHE_COMPACT_KERNEL_WAYS;
  if (ctx->lookup_hit_valid_w) {
    ctx->lookup_hit_way_w = ctx->lookup.hit_way;
  }
  (void)compare_tag;
}

static inline void
icache_compact_capture_lookup_data(ICacheCompactContext *ctx) {
  ctx->lookup_data_ready_w = 0;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    ctx->lookup_hit_data_w[word] = 0;
  }
  if (!ctx->allow_lookup_data_phase || !ctx->lookup_hit_valid_w ||
      !ctx->lookup.data_resp_valid ||
      ctx->lookup.data_resp_way != ctx->lookup_hit_way_w) {
    return;
  }
  ctx->lookup_data_ready_w = 1;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    ctx->lookup_hit_data_w[word] = ctx->lookup.data_resp_line[word];
  }
}

static inline void icache_compact_reset_dcache_probe_regs(
    ICacheCompactContext *ctx) {
  ctx->reg_write.dcache_probe_inflight_r = 0;
  ctx->reg_write.dcache_probe_done_r = 0;
  ctx->reg_write.dcache_probe_hit_r = 0;
  ctx->reg_write.dcache_probe_word_r = 0;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    ctx->reg_write.dcache_probe_data_r[word] = 0;
  }
}

static inline ICacheCompactDcacheProbeStep
icache_compact_step_dcache_probe(ICacheCompactContext *ctx) {
  ICacheCompactDcacheProbeStep step;
  step.done = ctx->regs.dcache_probe_done_r;
  step.hit = ctx->regs.dcache_probe_hit_r;
  for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
    step.line[word] = ctx->regs.dcache_probe_data_r[word];
  }

  if (ctx->regs.req_valid_r && icache_compact_req_is_mmio(&ctx->regs)) {
    step.done = 1;
    step.hit = 0;
    ctx->reg_write.dcache_probe_inflight_r = 0;
    ctx->reg_write.dcache_probe_done_r = 1;
    ctx->reg_write.dcache_probe_hit_r = 0;
    ctx->reg_write.dcache_probe_word_r = 0;
    return step;
  }

  const uint32_t word_idx = ctx->regs.dcache_probe_word_r;
  if (!ctx->regs.dcache_probe_done_r && !ctx->regs.dcache_probe_inflight_r &&
      word_idx < ICACHE_COMPACT_KERNEL_WORD_NUM && ctx->regs.req_valid_r) {
    ctx->out.dcache_req_valid = 1;
    ctx->out.dcache_req_addr =
        icache_compact_line_base_addr(ctx->regs.ppn_r, ctx->regs.req_index_r) +
        word_idx * 4u;
    ctx->reg_write.dcache_probe_inflight_r = 1;
  }

  if (ctx->regs.dcache_probe_inflight_r && ctx->in.dcache_resp_valid) {
    ctx->reg_write.dcache_probe_inflight_r = 0;
    if (ctx->in.dcache_resp_retry) {
      /* Keep the current probe word and reissue after the transient conflict. */
    } else if (ctx->in.dcache_resp_miss) {
      step.done = 1;
      step.hit = 0;
      ctx->reg_write.dcache_probe_done_r = 1;
      ctx->reg_write.dcache_probe_hit_r = 0;
    } else {
      step.hit = 1;
      step.line[word_idx] = ctx->in.dcache_resp_data;
      ctx->reg_write.dcache_probe_hit_r = 1;
      ctx->reg_write.dcache_probe_data_r[word_idx] =
          ctx->in.dcache_resp_data;
      if (word_idx + 1u >= ICACHE_COMPACT_KERNEL_WORD_NUM) {
        step.done = 1;
        ctx->reg_write.dcache_probe_done_r = 1;
      } else {
        ctx->reg_write.dcache_probe_word_r = word_idx + 1u;
      }
    }
  }
  return step;
}

static inline void icache_compact_latch_mem_refill(ICacheCompactContext *ctx) {
  ctx->mem_gnt = 1;
  if (icache_compact_req_is_mmio(&ctx->regs)) {
    for (uint32_t offset = 0; offset < ICACHE_COMPACT_KERNEL_WORD_NUM;
         ++offset) {
      ctx->mem_resp_data_w[offset] = 0;
    }
    const uint32_t word_idx =
        ((ctx->regs.req_pc_r & (ICACHE_COMPACT_KERNEL_LINE_SIZE - 1u)) / 4u) %
        ICACHE_COMPACT_KERNEL_WORD_NUM;
    ctx->mem_resp_data_w[word_idx] = ctx->in.mem_resp_data[0];
    ctx->replace_idx_next = ctx->regs.replace_idx;
  } else {
    for (uint32_t offset = 0; offset < ICACHE_COMPACT_KERNEL_WORD_NUM;
         ++offset) {
      ctx->mem_resp_data_w[offset] = ctx->in.mem_resp_data[offset];
    }
    icache_compact_capture_lookup_meta(ctx, ctx->regs.ppn_r & 0xfffffu, 0);
    if (ctx->lookup_has_invalid_way_w) {
      ctx->replace_idx_next = ctx->lookup_first_invalid_way_w;
    } else {
      ctx->replace_idx_next =
          (ctx->regs.replace_idx + 1u) % ICACHE_COMPACT_KERNEL_WAYS;
    }
  }
}

static inline void icache_compact_finish_refill(ICacheCompactContext *ctx,
                                                const uint32_t *line_words,
                                                uint32_t replace_way) {
  ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
  ctx->mem_axi_state_next = 0;
  if (ctx->regs.miss_txid_valid_r) {
    ctx->reg_write.txid_inflight_r[ctx->regs.miss_txid_r & 0xfu] = 0;
  }
  ctx->reg_write.miss_txid_valid_r = 0;
  ctx->reg_write.miss_ready_seen_r = 0;
  icache_compact_reset_dcache_probe_regs(ctx);

  const uint32_t mmio_refill = icache_compact_req_is_mmio(&ctx->regs);
  if (!ctx->in.flush && !mmio_refill) {
    ctx->table_write.we = 1;
    ctx->table_write.index = ctx->regs.req_index_r;
    ctx->table_write.way = replace_way;
    for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
      ctx->table_write.data[word] = line_words[word];
    }
    ctx->table_write.tag = ctx->regs.ppn_r;
    ctx->table_write.valid = 1;
  }

  if (!ctx->in.refetch && !ctx->in.flush) {
    ctx->out.ifu_resp_valid = 1;
    for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
      ctx->out.rd_data[word] = line_words[word];
    }
    ctx->out.ifu_resp_single_word = mmio_refill;
  }
  ctx->req_ready_w = 1;
}

static inline void icache_compact_eval_state_machine(ICacheCompactContext *ctx) {
  ctx->table_write = (ICacheCompactTableWrite){0};
  ctx->table_write.invalidate_all = ctx->in.flush;
  ctx->mem_gnt = 0;
  ctx->out.ppn_ready = 0;
  ctx->out.mem_resp_ready = 0;
  ctx->out.mem_req_valid = 0;
  ctx->out.mem_req_addr = icache_compact_req_mem_addr(&ctx->regs);
  ctx->out.mem_req_total_size = icache_compact_req_total_size(&ctx->regs);
  ctx->out.mem_req_bypass = icache_compact_req_is_mmio(&ctx->regs);
  ctx->out.dcache_req_valid = 0;
  ctx->out.dcache_req_addr = 0;
  ctx->out.ifu_resp_valid = 0;
  ctx->out.ifu_resp_single_word = 0;
  ctx->out.lookup_data_req_valid = 0;
  ctx->out.lookup_data_req_index = 0;
  ctx->out.lookup_data_req_way = 0;
  ctx->out.ifu_resp_pc = ctx->regs.req_pc_r;
  ctx->out.ifu_page_fault = 0;
  ctx->out.mem_req_id =
      ctx->regs.miss_txid_valid_r ? (ctx->regs.miss_txid_r & 0xfu) : 0;
  ctx->state_next = ctx->regs.state;
  ctx->mem_axi_state_next = ctx->regs.mem_axi_state;
  ctx->req_ready_w = 0;

  const uint32_t kill_pipe = ctx->in.refetch;
  const uint32_t resp_id = ctx->in.mem_resp_id & 0xfu;
  const uint32_t canceled_resp =
      ctx->in.mem_resp_valid && ctx->regs.txid_canceled_r[resp_id];
  const uint32_t same_cycle_accept_resp_candidate =
      ctx->in.mem_resp_valid &&
      ctx->regs.state == ICACHE_COMPACT_STATE_SWAP_IN &&
      ctx->regs.mem_axi_state == 0 &&
      ctx->regs.miss_txid_valid_r &&
      resp_id == (ctx->regs.miss_txid_r & 0xfu) &&
      ctx->in.mem_req_accepted &&
      ((ctx->in.mem_req_accepted_id & 0xfu) ==
       (ctx->regs.miss_txid_r & 0xfu));
  const uint32_t active_resp =
      ctx->in.mem_resp_valid && ctx->regs.miss_txid_valid_r &&
      resp_id == (ctx->regs.miss_txid_r & 0xfu) &&
      (ctx->regs.txid_inflight_r[resp_id] ||
       same_cycle_accept_resp_candidate) &&
      !canceled_resp;
  const uint32_t orphan_resp =
      ctx->in.mem_resp_valid && !active_resp && !canceled_resp &&
      !ctx->regs.txid_inflight_r[resp_id];

  if (canceled_resp) {
    ctx->out.mem_resp_ready = 1;
    ctx->reg_write.txid_canceled_r[resp_id] = 0;
    ctx->reg_write.txid_inflight_r[resp_id] = 0;
  }
  if (orphan_resp) {
    ctx->out.mem_resp_ready = 1;
    ctx->reg_write.txid_canceled_r[resp_id] = 0;
    ctx->reg_write.txid_inflight_r[resp_id] = 0;
  }

  switch (ctx->regs.state) {
  case ICACHE_COMPACT_STATE_IDLE:
    if (kill_pipe) {
      ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
      ctx->req_ready_w = 1;
      break;
    }

    if (ICACHE_COMPACT_KERNEL_LOOKUP_LATENCY == 0 &&
        !ctx->regs.req_valid_r && ctx->in.ifu_req_valid &&
        ctx->in.ppn_valid) {
      ctx->out.ifu_resp_pc = ctx->in.pc;
      if (ctx->in.page_fault) {
        for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
          ctx->out.rd_data[word] = 0;
        }
        ctx->out.ifu_resp_valid = 1;
        ctx->out.ifu_page_fault = 1;
        ctx->req_ready_w = 1;
        ctx->fast_bypass_fire = 1;
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
        break;
      }
      uint32_t index =
          icache_compact_index_from_pc(ctx->in.pc);
      icache_compact_capture_lookup_meta(ctx, ctx->in.ppn & 0xfffffu, 1);
      icache_compact_capture_lookup_data(ctx);
      if (ctx->lookup_hit_valid_w && ctx->lookup_data_ready_w) {
        for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
          ctx->out.rd_data[word] = ctx->lookup_hit_data_w[word];
        }
        ctx->out.ifu_resp_valid = 1;
        ctx->out.ifu_page_fault = 0;
        ctx->req_ready_w = 1;
        ctx->fast_bypass_fire = 1;
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
        break;
      } else if (ctx->lookup_hit_valid_w) {
        ctx->out.lookup_data_req_valid = 1;
        ctx->out.lookup_data_req_index = index;
        ctx->out.lookup_data_req_way = ctx->lookup_hit_way_w;
        ctx->req_ready_w = 0;
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
        break;
      }
    }

    if (ctx->regs.req_valid_r && ctx->in.ppn_valid) {
      if (ctx->in.page_fault) {
        for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
          ctx->out.rd_data[word] = 0;
        }
        ctx->out.ifu_resp_valid = 1;
        ctx->out.ifu_page_fault = 1;
        ctx->req_ready_w = 1;
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
        break;
      }

      icache_compact_capture_lookup_meta(ctx, ctx->in.ppn & 0xfffffu, 1);
      icache_compact_capture_lookup_data(ctx);
      ctx->out.ifu_resp_valid =
          ctx->lookup_hit_valid_w && ctx->lookup_data_ready_w;
      ctx->req_ready_w = ctx->lookup_hit_valid_w && ctx->lookup_data_ready_w;
      if (ctx->lookup_hit_valid_w && ctx->lookup_data_ready_w) {
        for (uint32_t word = 0; word < ICACHE_COMPACT_KERNEL_WORD_NUM; ++word) {
          ctx->out.rd_data[word] = ctx->lookup_hit_data_w[word];
        }
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
      } else if (ctx->lookup_hit_valid_w) {
        ctx->out.lookup_data_req_valid = 1;
        ctx->out.lookup_data_req_index = ctx->regs.req_index_r;
        ctx->out.lookup_data_req_way = ctx->lookup_hit_way_w;
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
      } else {
        int txid = ctx->regs.miss_txid_valid_r
                       ? (int)(ctx->regs.miss_txid_r & 0xfu)
                       : icache_compact_alloc_free_txid(&ctx->regs);
        if (txid < 0) {
          ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
          ctx->req_ready_w = 0;
        } else {
          ctx->reg_write.ppn_r = ctx->in.ppn;
          icache_compact_reset_dcache_probe_regs(ctx);
          ctx->reg_write.miss_txid_valid_r = 1;
          ctx->reg_write.miss_txid_r = (uint32_t)txid & 0xfu;
          ctx->reg_write.miss_ready_seen_r = 0;
          ctx->state_next = ICACHE_COMPACT_STATE_SWAP_IN;
        }
      }
    } else if (ctx->regs.req_valid_r && !ctx->in.ppn_valid) {
      ctx->req_ready_w = 0;
      ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
    } else {
      ctx->req_ready_w = 1;
      ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
    }
    ctx->out.ppn_ready = ctx->regs.req_valid_r;
    break;

  case ICACHE_COMPACT_STATE_SWAP_IN: {
    ICacheCompactDcacheProbeStep dcache_step =
        icache_compact_step_dcache_probe(ctx);
    const uint32_t req_ready_seen_now =
        ctx->regs.mem_axi_state == 0 && ctx->in.mem_req_ready &&
        ctx->regs.miss_txid_valid_r;
    if (req_ready_seen_now) {
      ctx->reg_write.miss_ready_seen_r = 1;
    }
    const uint32_t req_accepted_now =
        ctx->regs.mem_axi_state == 0 && ctx->in.mem_req_accepted &&
        ((ctx->in.mem_req_accepted_id & 0xfu) ==
         (ctx->regs.miss_txid_r & 0xfu)) &&
        ctx->regs.miss_txid_valid_r;
    if (kill_pipe) {
      const uint32_t req_already_inflight =
          ctx->regs.miss_txid_valid_r &&
          (ctx->regs.mem_axi_state != 0 || req_accepted_now);
      const uint32_t req_waiting_accept =
          ctx->regs.miss_txid_valid_r && !req_already_inflight &&
          (ctx->regs.miss_ready_seen_r || req_ready_seen_now);
      if (req_already_inflight) {
        uint32_t txid = ctx->regs.miss_txid_r & 0xfu;
        ctx->reg_write.txid_canceled_r[txid] = 1;
        ctx->reg_write.txid_inflight_r[txid] = 1;
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
        ctx->mem_axi_state_next = 0;
        ctx->reg_write.miss_txid_valid_r = 0;
        ctx->reg_write.miss_ready_seen_r = 0;
        icache_compact_reset_dcache_probe_regs(ctx);
        ctx->req_ready_w = 1;
      } else if (req_waiting_accept) {
        ctx->state_next = ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT;
        ctx->mem_axi_state_next = 0;
        icache_compact_reset_dcache_probe_regs(ctx);
        ctx->req_ready_w = 0;
      } else {
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
        ctx->mem_axi_state_next = 0;
        ctx->reg_write.miss_txid_valid_r = 0;
        ctx->reg_write.miss_ready_seen_r = 0;
        icache_compact_reset_dcache_probe_regs(ctx);
        ctx->req_ready_w = 1;
      }
      break;
    }

    if (ctx->regs.mem_axi_state == 0) {
      ctx->out.mem_req_addr = icache_compact_req_mem_addr(&ctx->regs);
      ctx->out.mem_req_total_size = icache_compact_req_total_size(&ctx->regs);
      ctx->out.mem_req_bypass = icache_compact_req_is_mmio(&ctx->regs);
      ctx->out.mem_req_id = ctx->regs.miss_txid_r & 0xfu;
      ctx->state_next = ICACHE_COMPACT_STATE_SWAP_IN;
      if (ctx->in.mem_req_accepted &&
          ((ctx->in.mem_req_accepted_id & 0xfu) ==
           (ctx->regs.miss_txid_r & 0xfu))) {
        ctx->out.mem_req_valid = 0;
        if (active_resp) {
          ctx->out.mem_resp_ready = 1;
          icache_compact_latch_mem_refill(ctx);
          ICacheCompactDcacheProbeStep same_cycle_dcache_step =
              icache_compact_step_dcache_probe(ctx);
          if (!same_cycle_dcache_step.done) {
            ctx->state_next = ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM;
            ctx->mem_axi_state_next = 0;
            ctx->reg_write.txid_inflight_r[ctx->regs.miss_txid_r & 0xfu] = 0;
            ctx->reg_write.miss_txid_valid_r = 0;
            ctx->reg_write.miss_ready_seen_r = 0;
            ctx->req_ready_w = 0;
          } else {
            icache_compact_finish_refill(
                ctx,
                same_cycle_dcache_step.hit ? same_cycle_dcache_step.line
                                           : ctx->mem_resp_data_w,
                ctx->replace_idx_next);
          }
        } else {
          ctx->mem_axi_state_next = 1;
          ctx->reg_write.txid_inflight_r[ctx->regs.miss_txid_r & 0xfu] = 1;
        }
      } else {
        ctx->out.mem_req_valid = 1;
        ctx->mem_axi_state_next = 0;
      }
    } else {
      ctx->out.mem_req_valid = 0;
      ctx->out.mem_resp_ready = ctx->out.mem_resp_ready || active_resp;
      ctx->mem_gnt = active_resp && ctx->out.mem_resp_ready;
      ctx->state_next = ICACHE_COMPACT_STATE_SWAP_IN;
      if (ctx->mem_gnt) {
        icache_compact_latch_mem_refill(ctx);
        ctx->mem_axi_state_next = 0;
        ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
        ctx->reg_write.txid_inflight_r[ctx->regs.miss_txid_r & 0xfu] = 0;
        ctx->reg_write.miss_txid_valid_r = 0;
        ctx->reg_write.miss_ready_seen_r = 0;
        if (!dcache_step.done) {
          ctx->state_next = ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM;
          ctx->req_ready_w = 0;
          break;
        }
        icache_compact_finish_refill(
            ctx, dcache_step.hit ? dcache_step.line : ctx->mem_resp_data_w,
            ctx->replace_idx_next);
      }
    }
    break;
  }

  case ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM:
    if (kill_pipe) {
      ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
      ctx->mem_axi_state_next = 0;
      ctx->reg_write.miss_txid_valid_r = 0;
      ctx->reg_write.miss_ready_seen_r = 0;
      icache_compact_reset_dcache_probe_regs(ctx);
      ctx->req_ready_w = 1;
      break;
    } else {
      ICacheCompactDcacheProbeStep dcache_step =
          icache_compact_step_dcache_probe(ctx);
      if (!dcache_step.done) {
        ctx->state_next = ICACHE_COMPACT_STATE_WAIT_DCACHE_AFTER_MEM;
        ctx->req_ready_w = 0;
        break;
      }
      icache_compact_finish_refill(
          ctx, dcache_step.hit ? dcache_step.line : ctx->regs.mem_resp_data_r,
          ctx->regs.replace_idx);
    }
    break;

  case ICACHE_COMPACT_STATE_CANCEL_WAIT_ACCEPT:
    ctx->out.mem_req_valid = 0;
    ctx->out.mem_resp_ready = 1;
    if (ctx->in.mem_req_accepted && ctx->regs.miss_txid_valid_r &&
        ((ctx->in.mem_req_accepted_id & 0xfu) ==
         (ctx->regs.miss_txid_r & 0xfu))) {
      uint32_t txid = ctx->regs.miss_txid_r & 0xfu;
      ctx->reg_write.txid_canceled_r[txid] = 1;
      ctx->reg_write.txid_inflight_r[txid] = 1;
    }
    ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
    ctx->mem_axi_state_next = 0;
    ctx->reg_write.miss_txid_valid_r = 0;
    ctx->reg_write.miss_ready_seen_r = 0;
    icache_compact_reset_dcache_probe_regs(ctx);
    ctx->req_ready_w = 1;
    break;

  case ICACHE_COMPACT_STATE_DRAIN:
    ctx->out.mem_resp_ready = 1;
    ctx->out.mem_req_valid = 0;
    if (ctx->in.mem_resp_valid) {
      ctx->mem_axi_state_next = 0;
      ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
      if (ctx->regs.miss_txid_valid_r) {
        uint32_t txid = ctx->regs.miss_txid_r & 0xfu;
        ctx->reg_write.txid_inflight_r[txid] = 0;
        ctx->reg_write.txid_canceled_r[txid] = 0;
        ctx->reg_write.miss_txid_valid_r = 0;
      }
      ctx->reg_write.miss_ready_seen_r = 0;
      icache_compact_reset_dcache_probe_regs(ctx);
      ctx->req_ready_w = 1;
    } else {
      ctx->state_next = ICACHE_COMPACT_STATE_DRAIN;
      ctx->req_ready_w = 0;
    }
    break;

  default:
    ctx->state_next = ICACHE_COMPACT_STATE_IDLE;
    ctx->req_ready_w = 1;
    break;
  }

  ctx->reg_write.state = ctx->state_next;
  ctx->reg_write.mem_axi_state = ctx->mem_axi_state_next;

  if (ctx->in.ppn_valid && ctx->out.ppn_ready) {
    ctx->reg_write.ppn_r = ctx->in.ppn;
  }
  ctx->reg_write.ifu_req_ready_r = ctx->req_ready_w;

  if (ctx->regs.state == ICACHE_COMPACT_STATE_SWAP_IN && ctx->mem_gnt) {
    for (uint32_t offset = 0; offset < ICACHE_COMPACT_KERNEL_WORD_NUM;
         ++offset) {
      ctx->reg_write.mem_resp_data_r[offset] = ctx->mem_resp_data_w[offset];
    }
    ctx->reg_write.replace_idx = ctx->replace_idx_next;
  }
}

static inline void icache_compact_lookup(ICacheCompactContext *ctx,
                                         uint32_t index) {
  const uint32_t use_external_lookup = ICACHE_COMPACT_KERNEL_LOOKUP_LATENCY > 0;
  const uint32_t kill_pipe = ctx->in.refetch;
  uint32_t lookup_pending_next = ctx->regs.lookup_pending_r;
  uint32_t lookup_index_next = ctx->regs.lookup_index_r;
  uint32_t lookup_pc_next = ctx->regs.lookup_pc_r;
  uint32_t sram_load_fire = 0;
  uint32_t load_pc =
      ctx->regs.lookup_pending_r ? ctx->regs.lookup_pc_r : ctx->in.pc;
  uint32_t load_index = index;
  uint32_t req_valid_next = ctx->regs.req_valid_r;

  if (kill_pipe) {
    ctx->req_ready_w = 0;
    req_valid_next = 0;
    ctx->out.mmu_req_valid = 0;
    ctx->reg_write.req_valid_r = 0;
    ctx->reg_write.lookup_pending_r = 0;
    ctx->reg_write.lookup_index_r = 0;
    ctx->reg_write.lookup_pc_r = 0;
    ctx->reg_write.ifu_req_ready_r = 0;
    return;
  }

  if (ctx->fast_bypass_fire) {
    req_valid_next = 0;
    ctx->reg_write.req_valid_r = 0;
    ctx->reg_write.lookup_pending_r = 0;
    ctx->reg_write.lookup_index_r = 0;
    ctx->reg_write.lookup_pc_r = 0;
    ctx->out.mmu_req_valid = 0;
    ctx->reg_write.ifu_req_ready_r = !ctx->fast_bypass_from_pending;
    return;
  }

  const uint32_t slot_idle = icache_compact_req_slot_idle(&ctx->regs);
  const uint32_t can_accept =
      ctx->in.ifu_req_valid && ctx->req_ready_w &&
      !ctx->regs.lookup_pending_r && slot_idle;
  if (can_accept) {
    if (!use_external_lookup) {
      sram_load_fire = 1;
    } else {
      lookup_pending_next = 1;
      lookup_index_next = index;
      lookup_pc_next = ctx->in.pc;
      if (ctx->lookup.meta_resp_valid) {
        sram_load_fire = 1;
        lookup_pending_next = 0;
      }
    }
  }

  if (use_external_lookup && ctx->regs.lookup_pending_r &&
      ctx->lookup.meta_resp_valid && slot_idle) {
    sram_load_fire = 1;
    lookup_pending_next = 0;
  }

  if (sram_load_fire) {
    req_valid_next = 1;
  } else if (ctx->req_ready_w) {
    req_valid_next = 0;
  }

  ctx->reg_write.req_valid_r = req_valid_next;
  ctx->reg_write.lookup_pending_r =
      use_external_lookup ? lookup_pending_next : 0;
  ctx->reg_write.lookup_index_r =
      use_external_lookup ? lookup_index_next : 0;
  ctx->reg_write.lookup_pc_r = use_external_lookup ? lookup_pc_next : 0;
  ctx->reg_write.ifu_req_ready_r =
      ctx->req_ready_w && !lookup_pending_next && !req_valid_next;

  if (sram_load_fire && !kill_pipe) {
    ctx->reg_write.req_pc_r = load_pc;
    ctx->reg_write.req_index_r = load_index;
  }

  ctx->out.mmu_req_valid = 0;
  if (can_accept) {
    ctx->out.mmu_req_valid = 1;
    ctx->out.mmu_req_vtag = ctx->in.pc >> 12;
  } else if (ctx->regs.lookup_pending_r && !ctx->in.ppn_valid) {
    ctx->out.mmu_req_valid = 1;
    ctx->out.mmu_req_vtag = ctx->regs.lookup_pc_r >> 12;
  } else if (ctx->regs.req_valid_r && !ctx->in.ppn_valid) {
    ctx->out.mmu_req_valid = 1;
    ctx->out.mmu_req_vtag = ctx->regs.req_pc_r >> 12;
  }
}

static inline void icache_compact_comb(ICacheCompactContext *ctx) {
  ctx->out = (ICacheCompactOutput){0};
  ctx->table_write = (ICacheCompactTableWrite){0};
  ctx->fast_bypass_fire = 0;
  ctx->fast_bypass_from_pending = 0;
  ctx->out.ifu_req_ready = ctx->regs.ifu_req_ready_r;
  ctx->out.ifu_resp_pc = ctx->regs.req_pc_r;
  ctx->out.mem_req_id =
      ctx->regs.miss_txid_valid_r ? (ctx->regs.miss_txid_r & 0xfu) : 0;
  ctx->out.mem_req_addr = icache_compact_req_mem_addr(&ctx->regs);
  ctx->out.mem_req_total_size = icache_compact_req_total_size(&ctx->regs);
  ctx->out.mem_req_bypass = icache_compact_req_is_mmio(&ctx->regs);
  if (ctx->regs.lookup_pending_r) {
    ctx->out.mmu_req_vtag = ctx->regs.lookup_pc_r >> 12;
  } else if (ctx->regs.req_valid_r) {
    ctx->out.mmu_req_vtag = ctx->regs.req_pc_r >> 12;
  } else {
    ctx->out.mmu_req_vtag = ctx->in.pc >> 12;
  }

  ctx->reg_write = ctx->regs;
  ctx->req_ready_w = ctx->regs.ifu_req_ready_r;
  ctx->replace_idx_next = ctx->regs.replace_idx;

  icache_compact_eval_state_machine(ctx);
  const uint32_t index = icache_compact_index_from_pc(ctx->in.pc);
  icache_compact_lookup(ctx, index);
  ctx->out.ifu_req_ready = ctx->req_ready_w;
}

static inline void icache_compact_kernel(const bool *pi, bool *po) {
  ICacheCompactContext ctx;
  ctx = (ICacheCompactContext){0};
  icache_compact_clear(po, ICACHE_COMPACT_KERNEL_PO_WIDTH);
  icache_compact_decode_regs(pi, &ctx.regs);
  ctx.allow_lookup_data_phase =
      icache_compact_decode_input(pi + ICACHE_COMPACT_KERNEL_REGS_WIDTH,
                                  &ctx.in);
  icache_compact_decode_lookup(
      pi + ICACHE_COMPACT_KERNEL_REGS_WIDTH +
          ICACHE_COMPACT_KERNEL_INPUT_WIDTH,
      &ctx.lookup);
  icache_compact_comb(&ctx);

  int bit = 0;
  icache_compact_encode_regs(po + bit, &ctx.reg_write);
  bit += ICACHE_COMPACT_KERNEL_REGS_WIDTH;
  icache_compact_encode_out(po + bit, &ctx.out);
  bit += ICACHE_COMPACT_KERNEL_OUT_WIDTH;
  icache_compact_encode_table_write(po + bit, &ctx.table_write);
}

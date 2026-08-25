#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ICACHE_LOOKUP_META_PACK_WAYS 8
#define ICACHE_LOOKUP_META_PACK_TAG_BITS 20
#define ICACHE_LOOKUP_META_PACK_WAY_BITS 3
#define ICACHE_LOOKUP_META_PACK_PI_WIDTH \
  (1 + ICACHE_LOOKUP_META_PACK_TAG_BITS + 1 + \
   ICACHE_LOOKUP_META_PACK_WAYS * (ICACHE_LOOKUP_META_PACK_TAG_BITS + 1))
#define ICACHE_LOOKUP_META_PACK_PO_WIDTH \
  (1 + 1 + ICACHE_LOOKUP_META_PACK_WAY_BITS + 1 + \
   ICACHE_LOOKUP_META_PACK_WAY_BITS)
#define ICACHE_LOOKUP_WAY_MATCH_PI_WIDTH \
  (1 + 1 + ICACHE_LOOKUP_META_PACK_TAG_BITS + \
   ICACHE_LOOKUP_META_PACK_TAG_BITS + 1)
#define ICACHE_LOOKUP_WAY_MATCH_PO_WIDTH 1
#define ICACHE_LOOKUP_HIT_SELECT_PI_WIDTH ICACHE_LOOKUP_META_PACK_WAYS
#define ICACHE_LOOKUP_HIT_SELECT_PO_WIDTH \
  (1 + ICACHE_LOOKUP_META_PACK_WAY_BITS)
#define ICACHE_LOOKUP_INVALID_SELECT_PI_WIDTH \
  (1 + ICACHE_LOOKUP_META_PACK_WAYS)
#define ICACHE_LOOKUP_INVALID_SELECT_PO_WIDTH \
  (1 + ICACHE_LOOKUP_META_PACK_WAY_BITS)
#define ICACHE_LOOKUP_TAG5_EQ_PI_WIDTH 10
#define ICACHE_LOOKUP_TAG5_EQ_PO_WIDTH 1
#define ICACHE_LOOKUP_WAY_MATCH_SELECT_PI_WIDTH 7
#define ICACHE_LOOKUP_WAY_MATCH_SELECT_PO_WIDTH 1

static inline bool icache_lookup_meta_pack_read_bit(const bool *bits, int bit) {
  return bits[bit];
}

static inline uint32_t icache_lookup_meta_pack_read_uint(const bool *bits,
                                                        int lsb, int width) {
  uint32_t value = 0;
  for (int i = 0; i < width; ++i) {
    if (bits[lsb + i]) {
      value |= (1u << i);
    }
  }
  return value;
}

static inline void icache_lookup_meta_pack_write_bit(bool *bits, int bit,
                                                     bool value) {
  bits[bit] = value;
}

static inline void icache_lookup_meta_pack_write_uint(bool *bits, int lsb,
                                                     int width,
                                                     uint32_t value) {
  for (int i = 0; i < width; ++i) {
    bits[lsb + i] = ((value >> i) & 1u) != 0;
  }
}

static inline void icache_lookup_way_match_kernel(const bool *pi, bool *po) {
  int bit = 0;
  const bool resp_valid = icache_lookup_meta_pack_read_bit(pi, bit++);
  const bool compare_valid = icache_lookup_meta_pack_read_bit(pi, bit++);
  const uint32_t compare_tag = icache_lookup_meta_pack_read_uint(
      pi, bit, ICACHE_LOOKUP_META_PACK_TAG_BITS);
  bit += ICACHE_LOOKUP_META_PACK_TAG_BITS;
  const uint32_t way_tag = icache_lookup_meta_pack_read_uint(
      pi, bit, ICACHE_LOOKUP_META_PACK_TAG_BITS);
  bit += ICACHE_LOOKUP_META_PACK_TAG_BITS;
  const bool way_valid = icache_lookup_meta_pack_read_bit(pi, bit++);
  po[0] = resp_valid && compare_valid && way_valid &&
          (way_tag == compare_tag);
}

static inline void icache_lookup_tag5_eq_kernel(const bool *pi, bool *po) {
  const uint32_t lhs = icache_lookup_meta_pack_read_uint(pi, 0, 5);
  const uint32_t rhs = icache_lookup_meta_pack_read_uint(pi, 5, 5);
  po[0] = lhs == rhs;
}

static inline void icache_lookup_way_match_select_kernel(const bool *pi,
                                                         bool *po) {
  const bool resp_valid = icache_lookup_meta_pack_read_bit(pi, 0);
  const bool compare_valid = icache_lookup_meta_pack_read_bit(pi, 1);
  const bool way_valid = icache_lookup_meta_pack_read_bit(pi, 2);
  const bool eq0 = icache_lookup_meta_pack_read_bit(pi, 3);
  const bool eq1 = icache_lookup_meta_pack_read_bit(pi, 4);
  const bool eq2 = icache_lookup_meta_pack_read_bit(pi, 5);
  const bool eq3 = icache_lookup_meta_pack_read_bit(pi, 6);
  po[0] = resp_valid && compare_valid && way_valid && eq0 && eq1 && eq2 &&
          eq3;
}

static inline void icache_lookup_hit_select_kernel(const bool *pi, bool *po) {
  bool hit_valid = false;
  uint32_t hit_way = 0;
  for (int way = 0; way < ICACHE_LOOKUP_META_PACK_WAYS; ++way) {
    if (!hit_valid && icache_lookup_meta_pack_read_bit(pi, way)) {
      hit_valid = true;
      hit_way = (uint32_t)way;
    }
  }

  int bit = 0;
  icache_lookup_meta_pack_write_bit(po, bit++, hit_valid);
  icache_lookup_meta_pack_write_uint(
      po, bit, ICACHE_LOOKUP_META_PACK_WAY_BITS, hit_way);
}

static inline void icache_lookup_invalid_select_kernel(const bool *pi,
                                                       bool *po) {
  int bit = 0;
  const bool resp_valid = icache_lookup_meta_pack_read_bit(pi, bit++);
  bool has_invalid_way = false;
  uint32_t first_invalid_way = 0;
  if (resp_valid) {
    for (int way = 0; way < ICACHE_LOOKUP_META_PACK_WAYS; ++way) {
      const bool valid = icache_lookup_meta_pack_read_bit(pi, bit + way);
      if (!valid && !has_invalid_way) {
        has_invalid_way = true;
        first_invalid_way = (uint32_t)way;
      }
    }
  }

  bit = 0;
  icache_lookup_meta_pack_write_bit(po, bit++, has_invalid_way);
  icache_lookup_meta_pack_write_uint(
      po, bit, ICACHE_LOOKUP_META_PACK_WAY_BITS, first_invalid_way);
}

static inline void icache_lookup_meta_pack_kernel(const bool *pi, bool *po) {
  bool compare_valid = false;
  uint32_t compare_tag = 0;
  bool resp_valid = false;
  uint32_t tag[ICACHE_LOOKUP_META_PACK_WAYS] = {0};
  bool valid[ICACHE_LOOKUP_META_PACK_WAYS] = {false};

  bool hit_valid = false;
  uint32_t hit_way = 0;
  bool has_invalid_way = false;
  uint32_t first_invalid_way = 0;

  int bit = 0;
  compare_valid = icache_lookup_meta_pack_read_bit(pi, bit++);
  compare_tag = icache_lookup_meta_pack_read_uint(
      pi, bit, ICACHE_LOOKUP_META_PACK_TAG_BITS);
  bit += ICACHE_LOOKUP_META_PACK_TAG_BITS;
  resp_valid = icache_lookup_meta_pack_read_bit(pi, bit++);
  for (int way = 0; way < ICACHE_LOOKUP_META_PACK_WAYS; ++way) {
    tag[way] = icache_lookup_meta_pack_read_uint(
        pi, bit, ICACHE_LOOKUP_META_PACK_TAG_BITS);
    bit += ICACHE_LOOKUP_META_PACK_TAG_BITS;
    valid[way] = icache_lookup_meta_pack_read_bit(pi, bit++);
  }

  if (resp_valid) {
    for (int way = 0; way < ICACHE_LOOKUP_META_PACK_WAYS; ++way) {
      if (!valid[way] && !has_invalid_way) {
        has_invalid_way = true;
        first_invalid_way = (uint32_t)way;
      }
      if (!compare_valid || hit_valid || !valid[way]) {
        continue;
      }
      if (tag[way] != compare_tag) {
        continue;
      }
      hit_valid = true;
      hit_way = (uint32_t)way;
    }
  }

  bit = 0;
  icache_lookup_meta_pack_write_bit(po, bit++, resp_valid);
  icache_lookup_meta_pack_write_bit(po, bit++, hit_valid);
  icache_lookup_meta_pack_write_uint(
      po, bit, ICACHE_LOOKUP_META_PACK_WAY_BITS, hit_way);
  bit += ICACHE_LOOKUP_META_PACK_WAY_BITS;
  icache_lookup_meta_pack_write_bit(po, bit++, has_invalid_way);
  icache_lookup_meta_pack_write_uint(
      po, bit, ICACHE_LOOKUP_META_PACK_WAY_BITS, first_invalid_way);
}

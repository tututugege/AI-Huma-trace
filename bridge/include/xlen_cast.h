#pragma once
// Register width of the function model this simulator is built against (-DXLEN=64 together with a rv64 librefcpu; the
// default is rv32).  The timing model itself is still 32 bit (pc_t, addresses, wire<32> results): values that come from
// the function model are carried as xlen_t up to the model's boundary and narrowed there with narrow_pc() / narrow_va(),
// which stop the simulation instead of silently truncating.
#include "refcpu_api.h"
#include <cstdio>
#include <cstdlib>

using xlen_t = refcpu_xlen_t;
using sxlen_t = int64_t;

// the low 32 bits of a value that must fit the model's 32-bit pc / address fields
inline uint32_t narrow_pc(xlen_t v, const char *what = "pc") {
  if ((v >> 32) != 0) {
    std::fprintf(stderr, "[xlen] %s 0x%llx does not fit 32 bits: the timing model is not widened yet\n", what,
                 static_cast<unsigned long long>(v));
    std::exit(2);
  }
  return static_cast<uint32_t>(v);
}
// translation enabled in satp: Sv32 (bit 31) on rv32, Sv39 (MODE = 8) on rv64
inline bool satp_on(xlen_t satp) {
  return (satp >> 60) == 8;
}

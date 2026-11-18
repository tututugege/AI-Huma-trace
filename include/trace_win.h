// trace_win.h -- the cycle window of the diagnostic traces (MSD_TRACE / BRIDGE_TRACE / TB_TRACE_COMMIT): the FPR_DUMP window when one is given (cycle = the
// fingerprint cycle, the RTL's PMU cycle counter), everywhere otherwise.  fpr::set_cycle keeps `trace_win::cycle` current.
#pragma once
#include <cstdint>
namespace trace_win {
inline bool active = false;           // an FPR_DUMP window is set
inline uint64_t from = 1, to = 0, cycle = 0;
inline bool ok() { return !active || (cycle >= from && cycle <= to); }
}  // namespace trace_win

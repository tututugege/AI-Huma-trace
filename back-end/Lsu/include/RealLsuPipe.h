// RealLsuPipe.h -- state of the staged LSU (switch LSU_PIPE / CFG_LSU_PIPE). Design: docs/LSU_PIPE.md.
// Every field has a declared width (wire<N>); registers live in LsuPipeState (pcur = registered value, pnxt = next value).
#pragma once

#include "IO.h"
#include "MemUtils.h"
#include "config.h"
#include <cstdint>

namespace lp {
typedef unsigned __int128 stq_mask_t;  // one bit per STQ entry (STQ_SIZE may exceed 64 in the EXP-024 write-buffer upper-bound runs)

constexpr int LD = LSU_LDU_COUNT;      // load pipelines (L1/L2)
constexpr int STA = LSU_STA_COUNT;     // STA ports (AGU + DTLB)
constexpr int STD = LSU_SDU_COUNT;     // store-data write ports
constexpr int RP_MAX = 4;              // replay registers (runtime value 1..4, CFG_LSU_PIPE_RP / LSU_PIPE_RP)
constexpr int DRAIN_MAX = STA;         // store drain ports at most (runtime value CFG_LSU_PIPE_DRAIN / LSU_PIPE_DRAIN, <= STA)
constexpr int RETIRE_PORTS = 4;        // STQ head entries freed per cycle
constexpr int EPOCH_W = 4;             // per-LDQ-entry request epoch (stale-response filter), never reset
constexpr int RWAIT_W = 3;             // replay countdown (0..LSU_PEPLAY_WAIT_CYCLES)
static_assert(LSU_PEPLAY_WAIT_CYCLES < (1 << RWAIT_W), "replay countdown width");
static_assert(LDQ_SIZE <= 64 && STQ_SIZE <= 128, "LDQ 64-bit masks, STQ 128-bit masks");

// LDQ entry state (4 bits)
enum PLd : uint8_t {
  L_EMPTY = 0,
  L_ALLOC = 1,    // allocated at dispatch, waiting for the AGU request
  L_PIPE = 2,     // inside L1 / L2 (or in a replay register)
  L_DCW = 3,      // DCache request sent, waiting for the response (bank-conflict retry is hidden in the DCache)
  L_RPW = 4,      // replay wait: cause in rcause, wake condition in rwait/blk_idx
  L_RDY = 5,      // replay ready: eligible for a replay port
  L_WBP = 6,      // result ready, waiting for a load write-back port
  L_MMW = 7,      // MMIO load waiting to be the oldest instruction
  L_MMR = 8,      // MMIO load on the bus
  L_WB3 = 9,      // LSU_PIPE_L2 bit 1: decided at L2, in the L3 register, writes back next cycle
  L_SPD = 10,     // memory speculation: written back (consumers woken) but passed an unknown-address older store; ROB completion pending
};
// replay cause (3 bits)
enum PRc : uint8_t {
  RC_TLB = 0,     // DTLB miss / no response: retry after rwait
  RC_ADDR = 1,    // youngest conflicting older store has no address yet: wake on its STA (blk_idx)
  RC_HOLD = 2,    // partial overlap / undecided SC: wake on the store retiring (blk_idx)
  RC_DATA = 3,    // store covers the load but its data is missing: wake = STD write -> forward directly
  RC_DC = 4,      // DCache replay (CONFLICT / MSHR_HIT / MSHR_FULL): rwait or MSHR fill
};

struct PLdq {
  wire<4> st = L_EMPTY;
  wire<ROB_IDX_WIDTH> rob_idx = 0;
  wire<1> rob_flag = 0;
  wire<BR_MASK_WIDTH> br_mask = 0;
  wire<PRF_IDX_WIDTH> dest_preg = 0;
  wire<3> func3 = 0;
  wire<XLEN> vaddr = 0;
  wire<1> pa_v = 0;           // paddr valid (translated once; replays skip the DTLB)
  wire<32> paddr = 0;
  wire<1> mmio = 0;
  wire<1> pf = 0;
  wire<1> illegal = 0;        // F/D: the load is illegal (mstatus.FS = Off), reported through the write-back
  wire<1> lrsc = 0;           // LR (reservation is set at write-back)
  wire<1> miss = 0;           // saw a DCache miss (TMA statistics only)
  wire<XLEN> result = 0;      // MMIO read data / faulting vaddr
  wire<STQ_IDX_WIDTH> snap_idx = 0;  // youngest older store + 1 (allocation tail at dispatch)
  wire<1> snap_flag = 0;
  wire<3> rcause = 0;
  wire<STQ_IDX_WIDTH> blk_idx = 0;   // STQ entry the replay waits for
  wire<RWAIT_W> rwait = 0;
  wire<2> rtype = 0;                 // ReplayType of the last DCache response
  wire<EPOCH_W> epoch = 0;
  wire<6> wd = 0;             // DCache response watchdog (cycles in DCW)
  wire<1> sw = 0;             // LSU_PIPE_L2: consumers already woken at L2 (no second wake-up at write-back)
  wire<1> spec = 0;           // memory speculation: passed an older store whose address was unknown at issue
  wire<1> vio = 0;            // a store address that arrived later overlaps this load: result is stale, re-execute
  wire<32> pc = 0;            // load PC (memory-dependence predictor)
  wire<16> sset = 0;          // store-set id of the load PC (predictor mode 1)
  wire<1> wait = 0;           // predictor: this load waited for unknown-address stores (no speculation)
  wire<1> second = 0;         // second pass (completion write-back or re-execution): no further speculation, no wake-up
  bool atnt = false;          // LSU_MS_DATA: the address came from a stale register
  bool stale = false;         // LSU_MS_DATA: the first write-back carried a value different from the true one
  stq_mask_t skip = 0;        // LSU_MS_DATA: STQ entries the last L1 CAM passed (address unknown): their data is not part of the loaded value
};

struct PStq {
  wire<32> pc = 0;            // store PC (predictor mode 1)
  wire<16> sset = 0;          // store-set id of the store PC
  wire<1> va_v = 0;           // STA arrived (vaddr[11:0] + size known: partial CAM can compare)
  wire<XLEN> vaddr = 0;
  wire<1> pa_v = 0;           // DTLB hit: paddr known
  wire<32> paddr = 0;
  wire<1> data_v = 0;
  wire<XLEN> data = 0;
  wire<3> func3 = 0;
  wire<1> pf = 0, mmio = 0, lrsc = 0, sc_pass = 0, suppress = 0;
  wire<1> trp = 0;            // STA DTLB retry pending
  wire<1> wbp = 0;            // STA completion (to the ROB) pending
  wire<1> wbd = 0;            // STA completion sent: store may commit
  wire<1> cmt = 0;            // committed by the ROB
  wire<1> iss = 0;            // sent to the DCache, response pending
  wire<1> mmw = 0;            // on the MMIO bus
  wire<2> wbi = 0;            // EXP-024 write buffer: 0 not processed, 1 in a write-buffer entry, 2 bypasses it (MMIO / suppressed)
  wire<5> wbe = 0;            // its write-buffer entry
  wire<1> mf = 0;             // EXP-024 LSU_PIPE_MERGE: sent as a follower of the leader store `ml` (shares its port and response)
  wire<STQ_IDX_WIDTH> ml = 0;
  wire<1> done = 0;           // written (or cancelled): may retire
  wire<PRF_IDX_WIDTH> dest_preg = 0;
  wire<ROB_IDX_WIDTH> rob_idx = 0;
  wire<1> rob_flag = 0;
  wire<BR_MASK_WIDTH> br_mask = 0;
  wire<2> rtype = 0;
  wire<RWAIT_W> rwait = 0;
  wire<6> wd = 0;             // DCache response watchdog (cycles in iss)
  // LSU_MS_DATA: what the trace says this store writes (captured the cycle after dispatch, the fact leaves the bridge at commit)
  bool m_init = false, m_v = false, m_amo = false, m_atnt = false;
  uint32_t m_w = 0, m_old = 0, m_new = 0, m_rs2 = 0;  // physical word, word before / after, true rs2
  uint8_t m_off = 0, m_sz = 0;                         // byte lane inside the word
};

// L1 -> L2 pipeline register (one per load port)
struct PL12 {
  wire<1> v = 0;
  wire<LDQ_IDX_WIDTH> ldq = 0;
  wire<EPOCH_W> epoch = 0;
  wire<2> tlb = 0;            // 0 hit, 1 miss / no response, 2 page fault
  wire<1> mmio = 0;
  wire<1> cand_v = 0;         // partial CAM: youngest older store that may conflict
  wire<STQ_IDX_WIDTH> cand = 0;
  wire<1> rpl = 0;            // the load came from a replay register (LSU_PIPE_L2 bit 128)
  wire<1> dcblk = 0;          // DCache request not launched: the port has a pending hidden retry -> short replay
};
// L2/L3 register (LSU_PIPE_L2 bit 1): the direct completion decided at L2 of load port p
struct PL3 {
  wire<1> v = 0;
  wire<LDQ_IDX_WIDTH> ldq = 0;
};
// cancel of a speculative wake-up (registered at the end of L2, applied before the next select)
struct PSf {
  wire<1> v = 0;
  wire<PRF_IDX_WIDTH> preg = 0;
};
// replay register: entry picked by a replay port, re-enters at L1
struct PRp {
  wire<1> v = 0;
  wire<LDQ_IDX_WIDTH> ldq = 0;
  wire<3> age = 0;            // cycles the register waited for a free L1 slot (saturating)
};

constexpr int WB_MAX = 32;      // write-buffer entries at most (runtime value LSU_PIPE_WBUF)
constexpr int WB_MEM = 32;      // stores per entry at most
// EXP-024 write buffer: committed stores of one cache line merged in commit order into one entry; the entry drains as one DCache request.
struct PWbE {
  bool v = false;
  uint8_t st = 0;               // 0 waiting, 1 sent to the DCache
  bool mergeable = true;
  uint32_t line = 0;
  uint64_t seq = 0;             // age
  uint8_t cnt = 0;
  uint8_t mem[WB_MEM] = {};     // STQ indices, oldest first
};

struct LsuPipeState {
  PLdq ldq[LDQ_SIZE];
  PStq stq[STQ_SIZE];
  PWbE wb[WB_MAX];
  uint64_t wb_seq = 0;
  wire<STQ_IDX_WIDTH> stq_head = 0;
  wire<1> stq_head_flag = 0;
  wire<STQ_IDX_WIDTH + 1> stq_count = 0;   // allocated
  wire<STQ_IDX_WIDTH + 1> stq_ccount = 0;  // committed, not yet retired
  wire<STQ_IDX_WIDTH> stq_commit = 0;      // head + ccount
  PL12 l12[LD];
  PL3 l3[LD];
  PSf sf[LD];
  PRp rp[RP_MAX];
  PRp srp;                                  // STA replay register (stq index)
  wire<STQ_IDX_WIDTH> srp_ptr = 0;          // round-robin start of the STA replay register pick (fairness: no starvation of walk owners)
  wire<LDQ_IDX_WIDTH> rp_ptr = 0;           // round-robin start of the RDY find-first (fairness: no starvation of walk owners)
};

// per-cycle combinational values that travel between the LSU comb phases (not registers)
struct PL1 {
  bool v = false;
  bool replay = false;
  bool need_tlb = false;
  uint8_t ldq = 0;
  uint8_t tlb = 0;
  bool hit = false, mmio = false;
  uint32_t paddr = 0;
  bool cand_v = false;
  uint8_t cand = 0;
  bool launch = false;
};
struct PSta {
  bool v = false;
  bool replay = false;
  uint8_t stq = 0;
};
struct PCycle {
  PL1 l1[LD];
  PSta s1[STA];
  bool wbd_v[LD] = {};           // L2 direct completion on load port p
  uint8_t wbd_ldq[LD] = {};
  stq_mask_t st_fullwake = 0;      // stores woken by an MSHR-full fill this cycle (bit = STQ index); LSU_PIPE_CUT bit 16
  uint8_t st_rwpre[STQ_SIZE] = {};     // their replay wait before the wake
  stq_mask_t ev_sta = 0, ev_data = 0, ev_ret = 0, ev_sc = 0;  // STQ events of this cycle (bit = STQ index)
  bool fill = false;
  uint32_t fill_addr = 0;
  int mmio_ld = -1;              // LDQ entry whose MMIO response arrived this cycle (LSU_PIPE_CUT bit 64: exempt from the F cut)
};

}  // namespace lp

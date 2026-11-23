#pragma once

#include "InstructionBuffer.h"
#include "config.h"
#include "wire_types.h"
#include "util.h"
#include <bitset>
#include <cstdint>
#include <cstring>
#include <sys/types.h>

struct DecRenIO {

  Uop uop[DECODE_WIDTH] = {};
  wire<1> valid[DECODE_WIDTH] = {};
};

struct RenDecIO {

  wire<1> ready = {};
};

struct RenRobIO {

  wire<1> walking = {};
};

// IDU -> PreIduQueue consume handshake (only what PreIduQueue needs).
struct IduConsumeIO {
  wire<1> fire[DECODE_WIDTH] = {};
};

struct PreFrontIO {

  wire<1> fire[BACKEND_FETCH_WIDTH] = {};
  wire<1> ready = {};
};

struct FrontPreIO {

  wire<32> inst[BACKEND_FETCH_WIDTH] = {};
  wire<XLEN> pc[BACKEND_FETCH_WIDTH] = {};
  wire<1> is_rvc[BACKEND_FETCH_WIDTH] = {};   // the instruction is 16 bits (RVC=1)
  wire<1> valid[BACKEND_FETCH_WIDTH] = {};
  wire<1> front_stall = {};
  int front_upd_free = 255, front_tq_free = 255;  // free entries at the start of the cycle (registered)  // BPU update / uBTB train queue free entries (compact ROB credits, EXP-028)
  wire<1> predict_dir[BACKEND_FETCH_WIDTH] = {};

  PredMetaT<BACKEND_FETCH_WIDTH> meta = {};
  wire<XLEN> predict_next_fetch_address[BACKEND_FETCH_WIDTH] = {};
  wire<1> page_fault_inst[BACKEND_FETCH_WIDTH] = {};
  wire<1> pf_inst_hi[BACKEND_FETCH_WIDTH] = {};
};

struct DecBroadcastIO {

  wire<1> mispred;
  wire<BR_MASK_WIDTH> br_mask;
  wire<BR_TAG_WIDTH> br_id;
  wire<ROB_IDX_WIDTH> redirect_rob_idx;
  wire<BR_MASK_WIDTH> clear_mask; // Bits to clear from all in-flight br_masks
                                  // (resolved branches)

  DecBroadcastIO() {
    mispred = {};
    br_mask = {};
    br_id = {};
    redirect_rob_idx = {};
    clear_mask = 0;
  }
};

// What an FTQ PC read returns (PreIduQueue::ftq_pc: a combinational read of the registered FTQ array)
struct FtqPc {
  wire<XLEN> pc = {};
  wire<1> is_rvc = {};
  wire<1> pred_taken = {};
  wire<XLEN> next_pc = {};
};

// what the BPU training needs of a committing instruction besides its uop (the return / call classification)
struct RobCommitExtra {
  wire<AREG_IDX_WIDTH> src1_areg{};
  wire<32> imm{};
};

struct FtqCommitInfoResp {
  wire<XLEN> pc{};
  wire<1> is_rvc{};
  wire<1> pred_taken{};
  wire<1> alt_pred{};
  pcpn_t pcpn{};
  wire<8> ps_info{};  // statistics only (BPA_STATS)
  uint16_t blk_id{};  // FTQ block sequence id (used by BPU_FTB=1 for exact block-based training)
  wire<1> ftq_is_last{};
};

struct FtqCommitInfoIO {
  FtqCommitInfoResp resp[COMMIT_WIDTH] = {};
};

struct PreIssueIO {
  InstructionBufferEntry entries[DECODE_WIDTH];

  PreIssueIO() {
    for (auto &e : entries) {
      e = {};
    }
  }
};

// Backward-compatible alias for existing modules.

struct RobCommitIO {

  struct RobCommitEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  RobCommitEntry commit_entry[COMMIT_WIDTH] = {};
  RobCommitExtra commit_extra[COMMIT_WIDTH] = {};   // per committing slot: what the BPU training needs besides the uop
};

struct RobDisIO {
  struct TmaMeta {
    wire<1> head_is_memory = {};
    wire<1> head_is_miss = {};
  } tma;

  wire<1> ready = {};
  wire<1> empty = {};
  wire<1> stall = {};
  wire<ROB_IDX_WIDTH> enq_idx = {};
  wire<1> rob_flag = {};
  int free_cnt = 0;  // free entries; enq_idx is the tail entry
};

struct DisRobIO {

  Uop uop[DECODE_WIDTH] = {};
  wire<1> valid[DECODE_WIDTH] = {};
  wire<1> dis_fire[DECODE_WIDTH] = {};
};

struct RenDisIO {

  Uop uop[DECODE_WIDTH] = {};
  wire<1> valid[DECODE_WIDTH] = {};
};

struct DisRenIO {

  wire<1> ready = {};
};

struct PrfAwakeIO {

  WakeInfo wake[LSU_LOAD_WB_WIDTH] = {};
};

// wake-up channels of the issue-queue bus: the ones of config.h (MAX_WAKEUP_PORTS) + the LSU channels (2 per load port: [p] unused, [LD+p] L3)
constexpr int LSU_EARLY_WAKE_PORTS = 2 * LSU_LOAD_WB_WIDTH;
constexpr int WAKE_CH_ALL = MAX_WAKEUP_PORTS + LSU_EARLY_WAKE_PORTS;

// Wake-ups driven by the LSU itself: [LD + p] from the L3 register (the [p] channels belonged to the removed speculative L2 wake-up
// and stay empty; the wake-up bus keeps its channel layout)
struct LsuAwakeIO {
  WakeInfo wake[2 * LSU_LOAD_WB_WIDTH] = {};
  WakeInfo cancel[LSU_LOAD_WB_WIDTH] = {};
};

struct DisIssIO {

  struct DisIssReq {
    wire<1> valid = {};
    Uop uop = {};
  };

  DisIssReq req[IQ_NUM][MAX_IQ_DISPATCH_WIDTH] = {};
};

struct IssDisIO {

  wire<IQ_READY_NUM_WIDTH> ready_num[IQ_NUM];

  IssDisIO() {
    for (auto &v : ready_num)
      v = 0;
  }
};

struct IssAwakeIO {

  WakeInfo wake[WAKE_CH_ALL] = {};
  // the same bus as a lookup: preg p is on it iff woke_at[p] == epoch (each drive of the bus starts a new epoch)
  uint64_t epoch = 1;
  uint64_t woke_at[TAG_NUM] = {};
  bool woken(uint32_t preg) const { return preg < TAG_NUM && woke_at[preg] == epoch; }
};

struct RobBroadcastIO {

  wire<1> flush = {};
  wire<1> flush_now = {};   // the flush decided in this cycle, also when `flush` is delayed (CFG_FLUSH_REG): the qualifier of this cycle's commit
  wire<1> mret = {};
  wire<1> sret = {};
  wire<1> ecall = {};
  wire<1> breakpoint = {};  // ebreak of the semihosting marker sequence: a breakpoint exception (cause 3)
  wire<1> exception = {};
  wire<1> fence = {};
  wire<1> fence_i = {};

  wire<1> page_fault_inst = {};
  wire<1> page_fault_load = {};
  wire<1> page_fault_store = {};
  wire<1> illegal_inst = {};
  wire<1> interrupt = {};
  wire<XLEN> trap_val = {};
  wire<XLEN> pc = {};
  wire<1> is_rvc = {};   // the instruction at pc is 16 bits: the refetch after it starts at pc + 2

  wire<ROB_IDX_WIDTH> head_rob_idx = {};
  wire<1> head_valid = {};
  wire<ROB_IDX_WIDTH> head_incomplete_rob_idx = {};
  wire<1> head_incomplete_valid = {};
};

struct IssPrfIO {

  struct IssPrfEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  IssPrfEntry iss_entry[ISSUE_WIDTH] = {};
};

struct ExeIssIO {

  wire<MAX_UOP_TYPE> fu_ready_mask[ISSUE_WIDTH] = {};
  wire<1> div_wake_valid = 0;
  wire<PRF_IDX_WIDTH> div_wake_preg = 0;
};

struct ExuRobIO {

  struct ExuRobEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  ExuRobEntry entry[ISSUE_WIDTH] = {};
};

struct ExuIdIO {

  wire<1> mispred;
  wire<XLEN> redirect_pc;
  wire<ROB_IDX_WIDTH> redirect_rob_idx;
  wire<BR_TAG_WIDTH> br_id;
  wire<FTQ_IDX_WIDTH>
      ftq_idx; // FTQ index of mispredicting branch, for tail recovery
  wire<BR_MASK_WIDTH>
      clear_mask; // OR of all resolved branches' (1 << br_id) this cycle
  wire<4> ftq_off;  // BPA_REPAIR: slot of the mispredicting branch in its fetch block
  wire<3> br_type;  // BPA_REPAIR: its type (BR_* encoding of the BPU)

  ExuIdIO() {
    ftq_off = 0;
    br_type = 0;
    mispred = {};
    redirect_pc = {};
    redirect_rob_idx = {};
    br_id = {};
    ftq_idx = 0;
    clear_mask = 0;
  }
};


struct CsrInterruptInjectIO {
  wire<1> external_irq_pending_valid = {};
  wire<1> external_irq_pending = {};
  wire<1> supervisor_external_irq_pending_valid = {};
  wire<1> supervisor_external_irq_pending = {};
  wire<1> timer_irq_pending_valid = {};
  wire<1> timer_irq_pending = {};
};

struct CsrStatusIO {

  wire<XLEN> sstatus = {};
  wire<XLEN> mstatus = {};
  wire<XLEN> satp = {};
  wire<2> privilege = {};
};

// What the commit does to the CSR file at the clock edge (Csr::seq): the interrupt is taken, a CSR instruction writes
// (from its uop and the value of its fact), an instruction set mstatus.FS to Dirty (a fact of the trace).
struct RobCsrIO {

  wire<1> interrupt_resp = {};
  wire<1> commit = {};       // a CSR instruction commits; it writes when `we`
  wire<1> we = {};
  wire<12> idx = {};
  wire<32> wcmd = {};
  wire<XLEN> wdata = {};
  wire<1> fs_dirty = {};
};

struct PtwWalkReq {
  wire<XLEN> vaddr = {};
  wire<XLEN> satp = {};
};

struct PtwWalkResp {
  wire<1> fault = {};
  wire<XLEN> vaddr = {};
  wire<XLEN> leaf_pte = {};   // the PTE (rv64: 8 bytes)
  wire<8> leaf_level = {}; // Sv32: 1 = L1 (megapage) leaf, 0 = L0 leaf; Sv39: 2 = 1 GiB, 1 = 2 MiB, 0 = 4 KiB
};

struct MemReqMeta {
  wire<ROB_IDX_WIDTH> rob_idx{};
};

struct MemRespMeta {
  wire<ROB_IDX_WIDTH> rob_idx{};
};

// The stores committed this cycle to a modeled device (CLINT timer / timecmp, interrupt controller, ...): the
// environment applies them at commit (the RTL testbench does the same, rtl/docs/modules/soc_top.md). One slot per
// commit slot; LSU drives it from the ROB's commit and its registered STQ.
struct CommitStore {
  bool valid = false;
  uint32_t paddr = 0;
  uint64_t data = 0;
  uint8_t func3 = 0;
};
struct CommitStoreIO {
  CommitStore slot[COMMIT_WIDTH] = {};
};

struct PeripheralReqIO {
  wire<1> is_mmio = {};
  wire<1> wen = {};
  wire<32> mmio_addr = {};
  wire<32> mmio_wdata = {};
  wire<3> mmio_fun3 = {};
};
struct PeripheralRespIO {
  wire<1> is_mmio = {};
  wire<1> ready = {};
  wire<XLEN> mmio_rdata = {};
};
enum class ReplayType : wire<2> {
  HIT = 0,
  CONFLICT = 1,//mshr conflict, fill confilct
  MSHR_HIT = 2,
  MSHR_FULL = 3,
};

enum class StoreState : uint8_t {
  Empty,
  Allocated,
  WaitAddr,
  WaitData,
  WaitTlb,
  WaitTlbRetry,
  WaitDone,
  Ready,
  WaitCommit,
  Committed,
  WaitDcacheResp,
  WaitMmioResp,
  Replaying,
  PageFault,
  WaitFinish,
  Done
};
// STQ entry (here for StoreReq)
struct StqEntry {

  wire<1> data_valid = false;
  wire<XLEN> data = 0;
  wire<3>  func3 = 0;



  wire<XLEN> vaddr = 0;

  wire<1> paddr_valid = false;
  wire<32> paddr = 0;
  wire<1> page_fault = false;
  
  wire<1> suppress_write = 0; // For MMIO: bits to suppress in the write (e.g., for LR/SC)

  wire<1> is_mmio = false;
  wire<1> sc_pass = false; // For SC: whether the store-conditional succeeded
  wire<PRF_IDX_WIDTH> dest_preg = 0; // SC returns 0/1 through STA writeback




  wire<BR_MASK_WIDTH> br_mask = {};
  wire<ROB_IDX_WIDTH> rob_idx = 0;
  wire<1> rob_flag = 0;
  wire<1> stq_flag = 0;
};

struct LoadReq {
  wire<1> valid;
  wire<32> addr;
  wire<32> req_id;
  wire<1> replay;
  wire<32> pc = 0;   // EXP-048: the load's PC (L1D prefetcher training, Berti)
  wire<XLEN> vaddr = 0;   // EXP-048: the load's virtual address (Berti in virtual mode)
};

struct StoreReq {
  wire<1> valid;
  wire<32> addr;
  wire<8> strb;
  wire<32> req_id;
  wire<1> replay;
};


// load response
struct LoadResp {
  wire<1> valid;
  wire<32> req_id;
  wire<1> retry;
  wire<1> bank_conflict;
  ReplayType replay;
};

// store response
struct StoreResp {
  wire<1> valid;
  wire<1> bank_conflict;
  ReplayType replay;
  wire<32>  req_id;
};

// request ports (loads and stores)
struct DCacheReqPorts {

  LoadReq load_ports[LSU_LDU_COUNT];
  StoreReq store_ports[LSU_STA_COUNT];

  void clear() {
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
      load_ports[i].valid = false;
      load_ports[i].replay = false;
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
      store_ports[i].valid = false;
      store_ports[i].replay = false;
    }
  }
};

// response ports
struct DCacheRespPorts {
  LoadResp load_resps[LSU_LDU_COUNT];
  StoreResp store_resps[LSU_STA_COUNT];

  void clear() {
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
      load_resps[i].valid = false;
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
      store_resps[i].valid = false;
    }
  }
};
struct LsuDcacheIO {
  DCacheReqPorts req_ports;
  wire<LSU_LDU_WIDTH+1> icache_req;

  LsuDcacheIO() { icache_req = LSU_LDU_COUNT; }
};

struct DcacheLsuIO {
  DCacheRespPorts resp_ports;
  wire<1> mshr_fill;
  wire<32> mshr_fill_addr;
  // per load port: a hidden bank-conflict retry is latched in the DCache (registered bit). The LSU must leave the port idle,
  // otherwise a new request wins the port every cycle and the retry starves (LSU_PIPE_RBLK).
  wire<1> load_retry_busy[LSU_LDU_COUNT] = {};
  wire<1> store_retry_busy[LSU_STA_COUNT] = {};  // same for the store ports

  // no response this cycle: the response valids and the stand-alone signals (mshr_fill, the retry-busy bits)
  void clear_valid() {
    resp_ports.clear();
    mshr_fill = false;
    for (auto &b : load_retry_busy) b = false;
    for (auto &b : store_retry_busy) b = false;
  }
};

struct LsuDisIO {

  wire<STQ_IDX_WIDTH> stq_tail;                     // allocation pointer
  wire<1> stq_tail_flag;                            // wrap bit of stq_tail
  wire<bit_width_for_count(STQ_SIZE + 1)> stq_free; // free entries
  wire<bit_width_for_count(LDQ_SIZE + 1)> ldq_free; // free LDQ entries
  wire<LDQ_IDX_WIDTH> ldq_alloc_idx[MAX_LDQ_DISPATCH_WIDTH];
  wire<1> ldq_alloc_valid[MAX_LDQ_DISPATCH_WIDTH];

  LsuDisIO() {
    stq_tail = 0;
    stq_tail_flag = 0;
    stq_free = 0;
    ldq_free = 0;
    for (auto &v : ldq_alloc_idx)
      v = 0;
    for (auto &v : ldq_alloc_valid)
      v = 0;
  }
};

struct LsuRobIO {
  struct TmaMeta {
    std::bitset<ROB_NUM> miss_mask;
  } tma;
  wire<1> committed_store_pending;

  LsuRobIO() {
    tma.miss_mask.reset();
    committed_store_pending = 0;
  }
};

struct LsuExeIO {

  struct LsuExeRespEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  LsuExeRespEntry wb_req[LSU_LOAD_WB_WIDTH] = {};
  LsuExeRespEntry sta_wb_req[LSU_STA_COUNT] = {};
};

// Dispatch -> LSU: the LSQ allocations of the fired loads and stores; the LSU reads the instruction's fields (ROB entry,
// STQ / LDQ entry, funct3, pc) from its pool entry `id`.
struct DisLsuIO {
  wire<1> alloc_req[MAX_STQ_DISPATCH_WIDTH] = {};
  uint16_t id[MAX_STQ_DISPATCH_WIDTH] = {};
  wire<BR_MASK_WIDTH> br_mask[MAX_STQ_DISPATCH_WIDTH] = {};
  wire<1> ldq_alloc_req[MAX_LDQ_DISPATCH_WIDTH] = {};
  uint16_t ldq_id[MAX_LDQ_DISPATCH_WIDTH] = {};
  wire<BR_MASK_WIDTH> ldq_br_mask[MAX_LDQ_DISPATCH_WIDTH] = {};
};

struct ExeLsuIO {

  struct ExeLsuReqEntry {
    wire<1> valid = {};
    wire<1> raw = {};   // the AGU holds a uop this cycle, whether a flush / mispredict kills it or not (the replay port allocation looks at this)
    Uop uop = {};
  };

  ExeLsuReqEntry agu_req[LSU_AGU_COUNT] = {};
  ExeLsuReqEntry sdu_req[LSU_SDU_COUNT] = {};
};

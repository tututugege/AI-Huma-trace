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
  wire<32> pc[BACKEND_FETCH_WIDTH] = {};
  wire<1> valid[BACKEND_FETCH_WIDTH] = {};
  wire<1> front_stall = {};
  int front_upd_free = 255, front_tq_free = 255;  // free entries at the start of the cycle (registered)  // BPU update / uBTB train queue free entries (compact ROB credits, EXP-028)
  wire<1> predict_dir[BACKEND_FETCH_WIDTH] = {};

  PredMetaT<BACKEND_FETCH_WIDTH> meta = {};
  wire<32> predict_next_fetch_address[BACKEND_FETCH_WIDTH] = {};
  wire<1> page_fault_inst[BACKEND_FETCH_WIDTH] = {};
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

struct FtqPcReadReq {
  wire<1> valid = {};
  wire<FTQ_IDX_WIDTH> ftq_idx = {};
  wire<FTQ_OFFSET_WIDTH> ftq_offset = {};
};

struct FtqPcReadResp {
  wire<1> valid = {};
  wire<1> entry_valid = {};
  wire<32> pc = {};
  wire<1> pred_taken = {};
  wire<32> next_pc = {};
};

struct FtqPrfPcReqIO {
  FtqPcReadReq req[FTQ_PRF_PC_PORT_NUM] = {};
};

struct FtqPrfPcRespIO {
  FtqPcReadResp resp[FTQ_PRF_PC_PORT_NUM] = {};
};

struct FtqRobPcReqIO {
  FtqPcReadReq req[FTQ_ROB_PC_PORT_NUM] = {};
};

struct FtqRobPcRespIO {
  FtqPcReadResp resp[FTQ_ROB_PC_PORT_NUM] = {};
};

struct FtqCommitInfoResp {
  wire<32> pc;
  wire<1> pred_taken;
  wire<1> alt_pred;
  pcpn_t pcpn;
  tage_idx_t tage_idx[4];
  tage_tag_t tage_tag[4];
  wire<1> sc_pred;
  tage_scl_meta_sum_t sc_sum;
  tage_scl_meta_idx_t sc_idx[BPU_SCL_META_NTABLE];
  wire<1> loop_used;
  wire<1> loop_hit;
  wire<1> loop_pred;
  tage_loop_meta_idx_t loop_idx;
  tage_loop_meta_tag_t loop_tag;
  wire<8> ps_info;  // statistics only (BPA_STATS)

  FtqCommitInfoResp() { std::memset(this, 0, sizeof(FtqCommitInfoResp)); }
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
using PreIduIssueIO = PreIssueIO;

struct RobCommitIO {

  struct RobCommitEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  RobCommitEntry commit_entry[COMMIT_WIDTH] = {};
};

struct RobDisIO {
  struct TmaMeta {
    wire<1> head_is_memory = {};
    wire<1> head_is_miss = {};
    wire<1> head_not_ready = {};
  } tma;

  wire<1> ready = {};
  wire<1> empty = {};
  wire<1> stall = {};
  wire<ROB_IDX_WIDTH> enq_idx = {};
  wire<1> rob_flag = {};
  int free_cnt = 0;  // compact ROB (ROB_COMPACT=1): free entries; enq_idx is then the tail entry, not a line
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

// wake-up channels of the issue-queue bus: the ones of config.h (MAX_WAKEUP_PORTS) + the LSU_PIPE_L2 channels (2 per load port: [p] L2, [LD+p] L3)
constexpr int LSU_EARLY_WAKE_PORTS = 2 * LSU_LOAD_WB_WIDTH;
constexpr int WAKE_CH_ALL = MAX_WAKEUP_PORTS + LSU_EARLY_WAKE_PORTS;

// LSU_PIPE_L2: wake-ups driven by the LSU itself (speculative at L2, or from the L3 register) and their cancel
struct LsuAwakeIO {
  WakeInfo wake[2 * LSU_LOAD_WB_WIDTH] = {};    // [p] L2 (speculative / oracle), [LD + p] L3 register
  WakeInfo cancel[LSU_LOAD_WB_WIDTH] = {};      // speculative wake-up of the previous cycle was wrong: consumers busy again
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
};

struct RobBroadcastIO {

  wire<1> flush = {};
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
  wire<32> trap_val = {};
  wire<32> pc = {};

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

struct PrfExeIO {

  struct PrfExeEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  PrfExeEntry iss_entry[ISSUE_WIDTH] = {};
};

struct ExePrfIO {

  struct ExePrfEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  ExePrfEntry entry[ISSUE_WIDTH] = {};
  ExePrfEntry bypass[TOTAL_FU_COUNT] = {};
};

struct ExeIssIO {

  wire<MAX_UOP_TYPE> fu_ready_mask[ISSUE_WIDTH] = {};
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
  wire<32> redirect_pc;
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

struct ExeCsrIO {

  wire<1> we = {};
  wire<1> re = {};
  wire<12> idx = {};
  wire<32> wdata = {};
  wire<32> wcmd = {};
};

struct CsrExeIO {

  wire<32> rdata = {};
};

struct CsrRobIO {

  wire<1> interrupt_req = {};
};

struct CsrFrontIO {

  wire<32> epc = {};
  wire<32> trap_pc = {};
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

  wire<32> sstatus = {};
  wire<32> mstatus = {};
  wire<32> satp = {};
  wire<2> privilege = {};
};

struct RobCsrIO {

  wire<1> interrupt_resp = {};
  wire<1> commit = {};
};

struct PtwWalkReq {
  wire<32> vaddr = {};
  wire<32> satp = {};
  wire<32> access_type; // 0=Fetch, 1=Load, 2=Store
};

struct PtwWalkResp {
  wire<1> fault = {};
  wire<32> vaddr = {};
  wire<32> leaf_pte = {};
  wire<8> leaf_level = {}; // 1: L1 leaf, 0: L0 leaf
};

struct PtwMemRespIO {
  wire<1> valid = {};
  wire<32> data = {};
};

struct PtwWalkRespIO {
  wire<1> valid = {};
  PtwWalkResp resp = {};
};

struct MemReqMeta {
  wire<ROB_IDX_WIDTH> rob_idx;

  MemReqMeta() { std::memset(this, 0, sizeof(MemReqMeta)); }
};

struct MemRespMeta {
  wire<ROB_IDX_WIDTH> rob_idx;
  bool is_cache_miss;

  MemRespMeta() { std::memset(this, 0, sizeof(MemRespMeta)); }
};

struct MemReqIO {

  wire<1> en = {};
  wire<1> wen = {};
  wire<32> addr = {};
  wire<32> wdata = {};
  wire<8> wstrb = {};

  MemReqMeta meta = {};
};

struct MemReadyIO {

  wire<1> ready = {};
};

struct MemRespIO {

  wire<1> wen = {};
  wire<1> valid = {};
  wire<32> data = {};

  wire<32> addr = {};
  MemRespMeta meta = {};
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
  wire<32> mmio_rdata = {};
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
// STQ 条目结构（定义在此以供 StoreReq 使用）
struct StqEntry {

  wire<1> data_valid = false;
  wire<32> data = 0;
  wire<3>  func3 = 0;



  wire<1> vaddr_valid = false;
  wire<32> vaddr = 0;

  wire<1> paddr_valid = false;
  wire<32> paddr = 0;
  wire<1> page_fault = false;
  
  wire<1> suppress_write = 0; // For MMIO: bits to suppress in the write (e.g., for LR/SC)

  wire<1> is_mmio = false;
  wire<1> is_lrsc = false;
  wire<1> sc_pass = false; // For SC: whether the store-conditional succeeded
  wire<PRF_IDX_WIDTH> dest_preg = 0; // SC returns 0/1 through STA writeback

  StoreState store_state = StoreState::Empty;

  ReplayType replay_type = ReplayType::HIT;
  uint32_t replay_wait_cycles = 0;


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
};

struct StoreReq {
  wire<1> valid;
  wire<32> addr;
  wire<8> strb;
  wire<32> req_id;
  wire<1> replay;
};


// Load响应结构
struct LoadResp {
  wire<1> valid;
  wire<32> req_id;
  wire<1> retry;
  wire<1> bank_conflict;
  ReplayType replay;
};

// Store响应结构
struct StoreResp {
  wire<1> valid;
  wire<1> bank_conflict;
  ReplayType replay;
  wire<32>  req_id;
};

// 请求端口集合（支持4个Load + 4个Store）
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

// 响应端口集合
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
enum class MMUResultType : wire<2> {
  MISS = 0,
  HIT = 1,
  PAGE_FAULT = 2,
};

static constexpr int LSU_MMU_IDX_WIDTH =
    (LDQ_IDX_WIDTH > STQ_IDX_WIDTH) ? LDQ_IDX_WIDTH : STQ_IDX_WIDTH;

struct MMUReq{
  wire<1> valid;
  wire<32> vaddr;
  wire<LSU_MMU_IDX_WIDTH> entry_idx;
  wire<LSU_MMU_IDX_WIDTH> wait_idx;
};
struct MMUResp{
  wire<1> valid;
  wire<32> paddr;
  MMUResultType result;
  wire<LSU_MMU_IDX_WIDTH> entry_idx;
  wire<LSU_MMU_IDX_WIDTH> wait_idx;
};
struct MMULsuIO{
  MMUResp ldq_resp[LSU_LDU_COUNT];
  MMUResp stq_resp[LSU_STA_COUNT];
};

struct LsuMMUIO{
  MMUReq ldq_req[LSU_LDU_COUNT];
  MMUReq stq_req[LSU_STA_COUNT];
  CsrStatusIO csr_status;
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
};

struct LsuDisIO {

  wire<STQ_IDX_WIDTH> stq_tail;                     // 当前分配指针
  wire<1> stq_tail_flag;                            // stq_tail 对应 ring 代际位
  wire<bit_width_for_count(STQ_SIZE + 1)> stq_free; // 剩余空闲条目数
  wire<bit_width_for_count(LDQ_SIZE + 1)> ldq_free; // 剩余 Load 队列空闲数
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

struct DisLsuIO {

  wire<1> alloc_req[MAX_STQ_DISPATCH_WIDTH];
  wire<BR_MASK_WIDTH> br_mask[MAX_STQ_DISPATCH_WIDTH];
  wire<3> func3[MAX_STQ_DISPATCH_WIDTH];
  wire<ROB_IDX_WIDTH> rob_idx[MAX_STQ_DISPATCH_WIDTH];
  wire<1> rob_flag[MAX_STQ_DISPATCH_WIDTH];
  wire<1> stq_flag[MAX_STQ_DISPATCH_WIDTH];

  wire<1> ldq_alloc_req[MAX_LDQ_DISPATCH_WIDTH];
  wire<LDQ_IDX_WIDTH> ldq_idx[MAX_LDQ_DISPATCH_WIDTH];
  wire<BR_MASK_WIDTH> ldq_br_mask[MAX_LDQ_DISPATCH_WIDTH];
  wire<ROB_IDX_WIDTH> ldq_rob_idx[MAX_LDQ_DISPATCH_WIDTH];
  wire<1> ldq_rob_flag[MAX_LDQ_DISPATCH_WIDTH];
  wire<32> st_pc[MAX_STQ_DISPATCH_WIDTH];   // store PC (store-set predictor)
  wire<32> ldq_pc[MAX_LDQ_DISPATCH_WIDTH];  // load PC (memory-dependence predictor index / training)

  DisLsuIO() {
    for (auto &v : alloc_req)
      v = 0;
    for (auto &v : br_mask)
      v = 0;
    for (auto &v : func3)
      v = 0;
    for (auto &v : rob_idx)
      v = 0;
    for (auto &v : rob_flag)
      v = 0;
    for (auto &v : stq_flag)
      v = 0;
    for (auto &v : ldq_alloc_req)
      v = 0;
    for (auto &v : ldq_idx)
      v = 0;
    for (auto &v : ldq_br_mask)
      v = 0;
    for (auto &v : ldq_rob_idx)
      v = 0;
    for (auto &v : ldq_rob_flag)
      v = 0;
    for (auto &v : ldq_pc)
      v = 0;
    for (auto &v : st_pc)
      v = 0;
  }
};

struct ExeLsuIO {

  struct ExeLsuReqEntry {
    wire<1> valid = {};
    Uop uop = {};
  };

  ExeLsuReqEntry agu_req[LSU_AGU_COUNT] = {};
  ExeLsuReqEntry sdu_req[LSU_SDU_COUNT] = {};
};

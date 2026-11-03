#pragma once
// Frozen copy of the original RTL-style LSU, used only as the lockstep reference (LSU_SHADOW builds).

#include "RealLsu.h"
typedef struct {
  RobCommitIO *rob_commit; RobBroadcastIO *rob_bcast; DecBroadcastIO *dec_bcast; CsrStatusIO *csr_status; DisLsuIO *dis2lsu;
  ExeLsuIO *exe2lsu; PeripheralRespIO *peripheral_resp; DcacheLsuIO *dcache2lsu; MMULsuIO *mmu2lsu;
} LsuInRef;
typedef struct {
  LsuDisIO *lsu2dis; LsuRobIO *lsu2rob; LsuExeIO *lsu2exe; PeripheralReqIO *peripheral_req; LsuDcacheIO *lsu2dcache; LsuMMUIO *lsu2mmu;
} LsuOutRef;
enum class LoadStateRef : uint8_t {
  Empty,
  Allocated,
  WaitAddr,
  WaitTlb,
  WaitTlbRetry,
  WaitSTLF,
  CheckStlf,
  CheckStlfRetry,
  WaitOlderStore,
  ReadyToIssue,
  WaitDcacheResp,
  WaitMmioResp,
  ReadyToWb,
  Replaying,
  PageFault,
  WaitFinish,
  WaitDcache,
  Done
};

struct LoadTagRef {
  reg<LDQ_IDX_WIDTH> idx = 0;
  reg<1> flag = false;
};

struct LdqEntryRef {

  LoadStateRef load_state;
  
  wire<1> v_addr_valid;
  wire<32> v_addr;

  wire<1> p_addr_valid;
  wire<32> p_addr;

  ReplayType replay_type;

  wire<3> func3;
  
  wire<32> result;
  wire<PRF_IDX_WIDTH> dest_preg;
  wire<4> byte_mask;

  wire<ROB_IDX_WIDTH> rob_idx;
  wire<1> rob_flag;
  wire<BR_MASK_WIDTH> br_mask;
  
  wire<1> is_mmio;
  wire<32> diag_val; // 用于记录发生异常时的相关信息（如访问的虚拟地址），供后续异常处理使用
  wire<1> page_fault;
  wire<1> is_lrsc;

  StoreTag stq_snapshot;


  wire<1> cache_miss;
};

struct UncachedUnitRef {
  wire<1> valid;

  wire<1> is_load;

  wire<32> addr;
  wire<32> wdata;
  wire<3> func3;

  wire<MAX_IDX_WIDTH> idx;
};
struct FenceUnitRef {
  wire<1> valid;

  wire<1> fence;
  wire<1> fence_i;
  wire<1> sfence_vma;

  wire<ROB_IDX_WIDTH> rob_idx;
  wire<1> rob_flag;
};
struct LrScUnitRef {
  wire<1> reserve_valid;
  wire<32> reserve_addr;
  wire<ROB_IDX_WIDTH> reserve_rob_idx;
  wire<1> reserve_rob_flag;
  wire<BR_MASK_WIDTH> reserve_br_mask;
};
struct WaitMmuSTQEntryRef{
  wire<1> valid;
  wire<STQ_IDX_WIDTH> stq_idx;
};
struct WaitMmuLDQEntryRef{
  wire<1> valid;
  wire<LDQ_IDX_WIDTH> ldq_idx;
};
struct WaitDcacheLDQEntryRef{
  wire<1> valid;
  wire<31-LDQ_IDX_WIDTH> req_gen;
  wire<LDQ_IDX_WIDTH> ldq_idx;
  wire<LSU_REPLAY_WAIT_CYCLES_WIDTH> replay_wait_cycles;
};
struct MMUDoneEntryRef{
  wire<1> valid;
  wire<STQ_IDX_WIDTH> stq_idx;
};
struct FinishEntryRef{
  wire<1> valid;
  wire<LDQ_IDX_WIDTH> ldq_idx;
};
struct STLFEntryRef{
  wire<1> valid;
  wire<LDQ_IDX_WIDTH> ldq_idx;
};
struct LsuStateRef{
  LdqEntryRef ldq[LDQ_SIZE];

  StqEntry stq[STQ_SIZE];
  wire<STQ_IDX_WIDTH> stq_head;
  wire<STQ_IDX_WIDTH> stq_commit;
  wire<1> stq_head_flag;
  wire<STQ_IDX_WIDTH+1> stq_commit_count; // 已提交但尚未排空的store条目数量
  wire<STQ_IDX_WIDTH+1> stq_count; // 包括分配但未提交的条目

  WaitMmuSTQEntryRef wait_mmu_stq[STQ_SIZE]; 
  wire<STQ_IDX_WIDTH> wait_mmu_stq_head;
  wire<STQ_IDX_WIDTH+1> wait_mmu_stq_count;

  WaitMmuLDQEntryRef wait_mmu_ldq[LDQ_SIZE];
  wire<LDQ_IDX_WIDTH> wait_mmu_ldq_head;
  wire<LDQ_IDX_WIDTH+1> wait_mmu_ldq_count;

  MMUDoneEntryRef mmu_done_stq[STQ_SIZE];
  wire<STQ_IDX_WIDTH> mmu_done_stq_head;
  wire<STQ_IDX_WIDTH+1> mmu_done_stq_count;

  FinishEntryRef finish[LDQ_SIZE];
  wire<LDQ_IDX_WIDTH> finish_head;
  wire<LDQ_IDX_WIDTH+1> finish_count;

  STLFEntryRef stlf_queue[LDQ_SIZE];
  wire<LDQ_IDX_WIDTH> stlf_queue_head;
  wire<LDQ_IDX_WIDTH+1> stlf_queue_count;


  WaitDcacheLDQEntryRef wait_dcache_ldq[LDQ_SIZE];
  wire<LDQ_IDX_WIDTH> wait_dcache_ldq_head;
  wire<LDQ_IDX_WIDTH+1> wait_dcache_ldq_count;

  UncachedUnitRef uncached_unit;
  LrScUnitRef lrsc_unit;

  wire<31-LDQ_IDX_WIDTH> req_gen; // 用于区分不同轮次的重放，防止过期重放条目被误用

  wire<1> stq_wait;
};

enum class STLFResultRef : wire<2> {
  Disjoint = 0,
  Overlap = 1,
  Retry = 2,
};
class RealLsuRef {
public:
  RealLsuRef():cur{}, nxt{}, in{}, out{}, ctx(nullptr) {};
  RealLsuRef(SimContext *ctx);

  LsuStateRef cur;
  LsuStateRef nxt;

  LsuInRef in{};
  LsuOutRef out{};
  SimContext *ctx = nullptr;

  void init();
  void comb_cal();
  void comb_lsu2dis();
  void comb_lsu2rob();
  void comb_mmio_out();
  void comb_mmio_in();
  void comb_tlb_out();
  void comb_tlb_in_ldq();
  void comb_tlb_in_stq();
  void comb_exe2lsu();
  void comb_dis2lsu();
  void comb_lsu2dcache_ldq();
  void comb_dcache2lsu_ldq();
  void comb_lsu2dcache_stq();
  void comb_dcache2lsu_stq();
  void comb_lsureplay();
  void comb_lsu2exe();
  void comb_stq_commit();
  void comb_flush();
  void comb_stlf();
  void comb_check();
  void seq();
  uint64_t digest() const;
  uint64_t digest_nxt() const;

  void handle_store_addr(const MicroOp &inst);
  void handle_store_data(const MicroOp &inst);
  void handle_load_req(const MicroOp &inst);

  bool alloc_stq_entry(mask_t br_mask,uint32_t rob_idx, uint32_t rob_flag,uint32_t func3, bool slot_flag);
  bool alloc_ldq_entry( mask_t br_mask,uint32_t rob_idx, uint32_t rob_flag,uint32_t ldq_idx);

  StqEntry get_stq_entry(int idx,bool flag);

  wire<STQ_IDX_WIDTH> enqueue_wait_mmu_stq(wire<STQ_IDX_WIDTH> stq_idx);
  wire<LDQ_IDX_WIDTH> enqueue_wait_mmu_ldq(wire<LDQ_IDX_WIDTH> ldq_idx);
  wire<STQ_IDX_WIDTH> enqueue_mmu_done_stq(wire<STQ_IDX_WIDTH> stq_idx);
  
  void enqueue_wait_dcache_ldq(wire<LDQ_IDX_WIDTH> ldq_idx);
  void enqueue_finish(wire<LDQ_IDX_WIDTH> ldq_idx);
  void enqueue_stlf(wire<LDQ_IDX_WIDTH> ldq_idx);

  void dequeue_wait_mmu_stq();
  void dequeue_wait_mmu_ldq();
  void dequeue_mmu_done_stq();
  void dequeue_wait_dcache_ldq();
  void dequeue_finish();
  void dequeue_stlf();

  void activate_wait_mmu_stq(wire<STQ_IDX_WIDTH> stq_idx);
  void activate_wait_mmu_ldq(wire<LDQ_IDX_WIDTH> ldq_idx);
  void activate_mmu_done_stq(wire<STQ_IDX_WIDTH> stq_idx);
};

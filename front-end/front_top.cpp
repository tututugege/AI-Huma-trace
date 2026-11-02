#include "BPU/BPU.h"
#include "front_IO.h"
#include "front_module.h"
#include "predecode.h"
#include "predecode_checker.h"
#include "train_IO.h"
#include <RISCV.h>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

#ifndef CONFIG_FRONT_FOCUS_PC_BEGIN
#define CONFIG_FRONT_FOCUS_PC_BEGIN 0u
#endif

#ifndef CONFIG_FRONT_FOCUS_PC_END
#define CONFIG_FRONT_FOCUS_PC_END 0u
#endif

// ============================================================================
// 全局状态（寄存器锁存值）
// ============================================================================
static bool predecode_refetch = false;
static uint32_t predecode_refetch_address = 0;
static uint32_t front_sim_time = 0;

// FIFO 状态锁存
static bool fetch_addr_fifo_full_latch = false;
static bool fetch_addr_fifo_empty_latch = true;
static bool fifo_full_latch = false;
static bool fifo_empty_latch = true;
static bool ptab_full_latch = false;
static bool ptab_empty_latch = true;
static bool front2back_fifo_full_latch = false;
static bool front2back_fifo_empty_latch = true;
static SimContext *front_ctx = nullptr;

void front_set_context(SimContext *ctx) { front_ctx = ctx; }


struct FrontDeadlockSnapshot {
  bool valid = false;
  bool global_reset = false;
  bool global_refetch = false;
  bool icache_ready = false;
  bool icache_ready_2 = false;
  bool fetch_addr_read_enable_slot0 = false;
  bool fetch_addr_read_enable_slot1 = false;
  bool inst_fifo_read_enable = false;
  bool ptab_read_enable = false;
  bool front2back_read_enable = false;
  icache_out icache = {};
  instruction_FIFO_out fifo = {};
  PTAB_out ptab = {};
  front2back_FIFO_out front2back = {};
  front_top_out out = {};
};

static FrontDeadlockSnapshot front_deadlock_snapshot;


static uint32_t sequential_next_pc(uint32_t fetch_pc,
                                   const wire1_t *inst_valid) {
  uint32_t valid_prefix = 0;
  while (valid_prefix < FETCH_WIDTH && inst_valid[valid_prefix]) {
    ++valid_prefix;
  }
  uint32_t next_pc = fetch_pc + valid_prefix * 4u;
  const uint32_t line_mask = ~(ICACHE_LINE_SIZE - 1u);
  if ((next_pc & line_mask) != (fetch_pc & line_mask)) {
    next_pc &= line_mask;
  }
  return next_pc;
}

static inline bool front_focus_pc(uint32_t pc) {
  const uint32_t begin = static_cast<uint32_t>(CONFIG_FRONT_FOCUS_PC_BEGIN);
  const uint32_t end = static_cast<uint32_t>(CONFIG_FRONT_FOCUS_PC_END);
  return end > begin && (pc - begin) < (end - begin);
}

static bool front_focus_any_fifo_pc(const instruction_FIFO_out &fifo_out) {
  for (int i = 0; i < FETCH_WIDTH; ++i) {
    if (fifo_out.inst_valid[i] && front_focus_pc(fifo_out.pc[i])) {
      return true;
    }
  }
  return false;
}

static bool front_focus_any_out_pc(const front_top_out &out) {
  for (int i = 0; i < FETCH_WIDTH; ++i) {
    if (out.inst_valid[i] && front_focus_pc(out.pc[i])) {
      return true;
    }
  }
  return false;
}





void front_dump_debug_state() {
  std::printf(
      "[DEADLOCK][FRONT] sim_time=%u fetch_addr_fifo{full=%d empty=%d} "
      "inst_fifo{full=%d empty=%d} ptab{full=%d empty=%d} front2back{full=%d "
      "empty=%d}\n",
      front_sim_time, static_cast<int>(fetch_addr_fifo_full_latch),
      static_cast<int>(fetch_addr_fifo_empty_latch),
      static_cast<int>(fifo_full_latch), static_cast<int>(fifo_empty_latch),
      static_cast<int>(ptab_full_latch), static_cast<int>(ptab_empty_latch),
      static_cast<int>(front2back_fifo_full_latch),
      static_cast<int>(front2back_fifo_empty_latch));
  if (!front_deadlock_snapshot.valid) {
    std::printf("[DEADLOCK][FRONT] no deadlock snapshot captured yet\n");
    return;
  }

  const auto &s = front_deadlock_snapshot;
  std::printf(
      "[DEADLOCK][FRONT][SNAP] reset=%d refetch=%d ic_ready={%d,%d} "
      "read_en={fa0=%d fa1=%d fifo=%d ptab=%d f2b=%d} fifo_valid=%d "
      "predict_next=0x%08x\n",
      static_cast<int>(s.global_reset), static_cast<int>(s.global_refetch),
      static_cast<int>(s.icache_ready), static_cast<int>(s.icache_ready_2),
      static_cast<int>(s.fetch_addr_read_enable_slot0),
      static_cast<int>(s.fetch_addr_read_enable_slot1),
      static_cast<int>(s.inst_fifo_read_enable),
      static_cast<int>(s.ptab_read_enable),
      static_cast<int>(s.front2back_read_enable),
      static_cast<int>(s.out.FIFO_valid), s.out.predict_next_fetch_address);
  std::printf(
      "[DEADLOCK][FRONT][ICACHE] fetch_pc={0x%08x,0x%08x} req_fire=%d "
      "req_blocked=%d resp_fire=%d miss=%d miss_busy=%d outstanding=%d "
      "itlb{hit=%d miss=%d fault=%d retry=%d other_walk=%d req_blocked=%d "
      "wait_walk=%d local_walker=%d}\n",
      s.icache.fetch_pc, s.icache.fetch_pc_2,
      static_cast<int>(s.icache.perf_req_fire),
      static_cast<int>(s.icache.perf_req_blocked),
      static_cast<int>(s.icache.perf_resp_fire),
      static_cast<int>(s.icache.perf_miss_event),
      static_cast<int>(s.icache.perf_miss_busy),
      static_cast<int>(s.icache.perf_outstanding_req),
      static_cast<int>(s.icache.perf_itlb_hit),
      static_cast<int>(s.icache.perf_itlb_miss),
      static_cast<int>(s.icache.perf_itlb_fault),
      static_cast<int>(s.icache.perf_itlb_retry),
      static_cast<int>(s.icache.perf_itlb_retry_other_walk),
      static_cast<int>(s.icache.perf_itlb_retry_walk_req_blocked),
      static_cast<int>(s.icache.perf_itlb_retry_wait_walk_resp),
      static_cast<int>(s.icache.perf_itlb_retry_local_walker_busy));
  icache_dump_debug_state();
  for (int i = 0; i < FETCH_WIDTH; ++i) {
    std::printf(
        "[DEADLOCK][FRONT][OUT %d] valid=%d pc=0x%08x inst=0x%08x pf=%d "
        "fifo_pc=0x%08x fifo_inst=0x%08x fifo_valid=%d\n",
        i, static_cast<int>(s.out.inst_valid[i]), s.out.pc[i],
        s.out.instructions[i], static_cast<int>(s.out.page_fault_inst[i]),
        s.fifo.pc[i], s.fifo.instructions[i],
        static_cast<int>(s.fifo.inst_valid[i]));
  }
}


static BPU_TOP bpu_instance;

// 定义全局指针，供TAGE访问BPU的GHR/FH
BPU_TOP *g_bpu_top = &bpu_instance;
// ============================================================================
// 辅助函数
// ============================================================================





static void front_ptab_write_comb(const FrontPtabWriteCombIn &input,
                                  FrontPtabWriteCombOut &output) {
    const BPU_TOP::OutputPayload &bpu_output = input.bpu_output;
    const wire1_t global_reset = input.global_reset;
    const wire1_t global_refetch = input.global_refetch;
    const wire1_t ptab_can_write = input.ptab_can_write;
    std::memset(&output, 0, sizeof(output));
    output.ptab_in.reset = global_reset;
    output.ptab_in.refetch = global_refetch;
    output.ptab_in.read_enable = false;
    output.ptab_in.write_enable = ptab_can_write;
    if (!ptab_can_write) {
        return;
    }
    for (int i = 0; i < FETCH_WIDTH; i++) {
        output.ptab_in.predict_dir[i] = bpu_output.out_pred_dir[i];
        output.ptab_in.predict_base_pc[i] = bpu_output.out_pred_base_pc + (i * 4);
    }
    output.ptab_in.meta = bpu_output.meta;
    output.ptab_in.predict_next_fetch_address = bpu_output.predict_next_fetch_address;
}

static void front_checker_input_comb(const FrontCheckerInputCombIn &input,
                                     FrontCheckerInputCombOut &output) {
    const instruction_FIFO_out &fifo_out = input.fifo_out;
    const PTAB_out &ptab_out = input.ptab_out;
    std::memset(&output, 0, sizeof(output));
    for (int i = 0; i < FETCH_WIDTH; i++) {
        output.checker_in.predict_dir[i] = ptab_out.predict_dir[i];
        output.checker_in.predecode_type[i] = fifo_out.predecode_type[i];
        output.checker_in.predecode_target_address[i] =
            fifo_out.predecode_target_address[i];
    }
    output.checker_in.seq_next_pc = fifo_out.seq_next_pc;
    output.checker_in.predict_next_fetch_address = ptab_out.predict_next_fetch_address;
}

static void front_front2back_write_comb(const FrontFront2backWriteCombIn &input,
                                        FrontFront2backWriteCombOut &output) {
    const instruction_FIFO_out &fifo_out = input.fifo_out;
    const PTAB_out &ptab_out = input.ptab_out;
    const predecode_checker_out &checker_out = input.checker_out;
    const wire1_t use_front2back_output_bypass =
        input.use_front2back_output_bypass;
    std::memset(&output, 0, sizeof(output));
    for (int i = 0; i < FETCH_WIDTH; i++) {
        output.front2back_fifo_in.fetch_group[i] = fifo_out.instructions[i];
        output.front2back_fifo_in.page_fault_inst[i] = fifo_out.page_fault_inst[i];
        output.front2back_fifo_in.inst_valid[i] = fifo_out.inst_valid[i];
        output.front2back_fifo_in.predict_dir_corrected[i] =
            checker_out.predict_dir_corrected[i];
        output.front2back_fifo_in.predict_base_pc[i] = ptab_out.predict_base_pc[i];
        if (use_front2back_output_bypass) {
            output.bypass_front2back_fifo_out.fetch_group[i] = fifo_out.instructions[i];
            output.bypass_front2back_fifo_out.page_fault_inst[i] =
                fifo_out.page_fault_inst[i];
            output.bypass_front2back_fifo_out.inst_valid[i] = fifo_out.inst_valid[i];
            output.bypass_front2back_fifo_out.predict_dir_corrected[i] =
                checker_out.predict_dir_corrected[i];
            output.bypass_front2back_fifo_out.predict_base_pc[i] =
                ptab_out.predict_base_pc[i];
        }
    }
    output.front2back_fifo_in.meta = ptab_out.meta;
    output.front2back_fifo_in.meta.mask();
    if (use_front2back_output_bypass) {
        output.bypass_front2back_fifo_out.meta = ptab_out.meta;
    }
    output.front2back_fifo_in.predict_next_fetch_address_corrected =
        checker_out.predict_next_fetch_address_corrected;
    if (use_front2back_output_bypass) {
        output.bypass_front2back_fifo_out.front2back_FIFO_valid = true;
        output.bypass_front2back_fifo_out.predict_next_fetch_address_corrected =
            checker_out.predict_next_fetch_address_corrected;
    }
}

static void front_output_comb(const FrontOutputCombIn &input,
                              FrontOutputCombOut &output) {
    const front2back_FIFO_out &saved_front2back_fifo_out =
        input.saved_front2back_fifo_out;
    const front2back_FIFO_out &bypass_front2back_fifo_out =
        input.bypass_front2back_fifo_out;
    const wire1_t use_front2back_output_bypass =
        input.use_front2back_output_bypass;
    std::memset(&output, 0, sizeof(output));
    const struct front2back_FIFO_out *out_src = &saved_front2back_fifo_out;
    if (!saved_front2back_fifo_out.front2back_FIFO_valid &&
        use_front2back_output_bypass) {
        out_src = &bypass_front2back_fifo_out;
    }

    output.out.FIFO_valid = out_src->front2back_FIFO_valid;
    for (int i = 0; i < FETCH_WIDTH; i++) {
        output.out.instructions[i] = out_src->fetch_group[i];
        output.out.page_fault_inst[i] = out_src->page_fault_inst[i];
        output.out.predict_dir[i] = out_src->predict_dir_corrected[i];
        output.out.pc[i] = out_src->predict_base_pc[i];
        output.out.alt_pred[i] = out_src->meta.alt_pred[i];
        output.out.altpcpn[i] = out_src->meta.altpcpn[i];
        output.out.pcpn[i] = out_src->meta.pcpn[i];
        for (int j = 0; j < 4; j++) {
            output.out.tage_idx[i][j] = out_src->meta.tage_idx[i][j];
            output.out.tage_tag[i][j] = out_src->meta.tage_tag[i][j];
        }
        output.out.sc_used[i] = out_src->meta.sc_used[i];
        output.out.sc_pred[i] = out_src->meta.sc_pred[i];
        output.out.sc_sum[i] = out_src->meta.sc_sum[i];
        for (int t = 0; t < BPU_SCL_META_NTABLE; ++t) {
            output.out.sc_idx[i][t] = out_src->meta.sc_idx[i][t];
        }
        output.out.loop_used[i] = out_src->meta.loop_used[i];
        output.out.loop_hit[i] = out_src->meta.loop_hit[i];
        output.out.loop_pred[i] = out_src->meta.loop_pred[i];
        output.out.loop_idx[i] = out_src->meta.loop_idx[i];
        output.out.loop_tag[i] = out_src->meta.loop_tag[i];
        output.out.inst_valid[i] = out_src->inst_valid[i];
    }
    for (int i = FETCH_WIDTH; i < BACKEND_FETCH_WIDTH; ++i) {
        output.out.instructions[i] = INST_NOP;
        output.out.page_fault_inst[i] = false;
        output.out.inst_valid[i] = false;
    }
    output.out.predict_next_fetch_address = out_src->predict_next_fetch_address_corrected;
}

// 准备 BPU 输入

static void accumulate_fetch_addr_req(FetchAddrCombOut &acc,
                                      const FetchAddrCombOut &step) {
    acc.clear_fifo |= step.clear_fifo;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: fetch_address_FIFO pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_en) {
        if (acc.push_en) {
            std::printf("[front_top] ERROR: fetch_address_FIFO push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_en = true;
        acc.push_data = step.push_data;
    }
}

static void accumulate_instruction_fifo_req(InstructionCombOut &acc,
                                            const InstructionCombOut &step) {
    acc.clear_fifo |= step.clear_fifo;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: instruction_FIFO pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_en) {
        if (acc.push_en) {
            std::printf("[front_top] ERROR: instruction_FIFO push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_en = true;
        acc.push_entry = step.push_entry;
    }
}

static void accumulate_ptab_req(PtabCombOut &acc, const PtabCombOut &step) {
    acc.clear_ptab |= step.clear_ptab;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: PTAB pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_write_en) {
        if (acc.push_write_en) {
            std::printf("[front_top] ERROR: PTAB write push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_write_en = true;
        acc.push_write_entry = step.push_write_entry;
    }
}

static void accumulate_front2back_req(Front2BackCombOut &acc,
                                      const Front2BackCombOut &step) {
    acc.clear_fifo |= step.clear_fifo;
    if (step.pop_en) {
        if (acc.pop_en) {
            std::printf("[front_top] ERROR: front2back_FIFO pop requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.pop_en = true;
    }
    if (step.push_en) {
        if (acc.push_en) {
            std::printf("[front_top] ERROR: front2back_FIFO push requested multiple times in one cycle\n");
            std::exit(1);
        }
        acc.push_en = true;
        acc.push_entry = step.push_entry;
    }
}

// ============================================================================
// 主函数
// ============================================================================
void front_comb_calc(const struct front_top_in &inp, const FrontReadData &rd,
                     struct front_top_out &out, FrontUpdateRequest &req) {
    struct front_top_in *in = const_cast<struct front_top_in *>(&inp);
    struct front_top_out *out_ptr = &out;
    PendingBpuSeqTxn &bpu_seq_txn_req = req.bpu_seq_txn;
    PendingFrontState &front_state_req = req.front_state;
    FetchAddrCombOut &fetch_addr_fifo_final_req = req.fetch_addr_fifo_req;
    InstructionCombOut &fifo_final_req = req.fifo_req;
    PtabCombOut &ptab_final_req = req.ptab_req;
    Front2BackCombOut &front2back_fifo_final_req = req.front2back_fifo_req;
    std::memset(&out, 0, sizeof(struct front_top_out));
    uint32_t front_sim_time = rd.front_sim_time_snapshot + 1;

    front_state_req.valid = false;
    
    // ========================================================================
    // 阶段 0: 初始化所有模块的输入输出结构
    // ========================================================================
    struct fetch_address_FIFO_in fetch_addr_fifo_in;
    struct fetch_address_FIFO_out fetch_addr_fifo_out;
    fetch_address_FIFO_read_data fetch_addr_fifo_rd = rd.fetch_addr_fifo_rd_snapshot;
    fetch_address_FIFO_read_data fetch_addr_fifo_next_rd = fetch_addr_fifo_rd;
    FetchAddrCombOut fetch_addr_fifo_req{};
    FetchAddrCombOut fetch_addr_fifo_step_req{};
    struct icache_in icache_in;
    struct icache_out icache_out;
    struct instruction_FIFO_in fifo_in;
    struct instruction_FIFO_out fifo_out;
    instruction_FIFO_read_data fifo_rd = rd.fifo_rd_snapshot;
    instruction_FIFO_read_data fifo_next_rd = fifo_rd;
    InstructionCombOut fifo_req{};
    InstructionCombOut fifo_step_req{};
    struct PTAB_in ptab_in;
    struct PTAB_out ptab_out;
    PTAB_read_data ptab_rd = rd.ptab_rd_snapshot;
    PTAB_read_data ptab_next_rd = ptab_rd;
    PtabCombOut ptab_req{};
    PtabCombOut ptab_step_req{};
    struct front2back_FIFO_in front2back_fifo_in;
    struct front2back_FIFO_out front2back_fifo_out;
    front2back_FIFO_read_data front2back_fifo_rd = rd.front2back_fifo_rd_snapshot;
    front2back_FIFO_read_data front2back_fifo_next_rd = front2back_fifo_rd;
    Front2BackCombOut front2back_fifo_req{};
    Front2BackCombOut front2back_fifo_step_req{};
    
    memset(&fetch_addr_fifo_in, 0, sizeof(fetch_addr_fifo_in));
    memset(&fetch_addr_fifo_out, 0, sizeof(fetch_addr_fifo_out));
    memset(&icache_in, 0, sizeof(icache_in));
    memset(&icache_out, 0, sizeof(icache_out));
    memset(&fifo_in, 0, sizeof(fifo_in));
    memset(&fifo_out, 0, sizeof(fifo_out));
    memset(&ptab_in, 0, sizeof(ptab_in));
    memset(&ptab_out, 0, sizeof(ptab_out));
    memset(&front2back_fifo_in, 0, sizeof(front2back_fifo_in));
    memset(&front2back_fifo_out, 0, sizeof(front2back_fifo_out));
    
    bool global_reset = false;
    bool global_refetch = false;
    uint32_t refetch_address = 0;
    bool icache_ready = false;
    bool icache_ready_2 = false;
    bool fetch_addr_fifo_read_enable_slot0 = false;
    bool fetch_addr_fifo_read_enable_slot1_candidate = false;
    bool inst_fifo_read_enable = false;
    bool ptab_read_enable = false;
    bool front2back_read_enable = false;
    struct fetch_address_FIFO_out saved_fetch_addr_fifo_out_0{};
    struct fetch_address_FIFO_out saved_fetch_addr_fifo_out_1{};

    {
        // ========================================================================
        // 阶段 1: 计算全局 flush/refetch 信号
        // ========================================================================
        global_reset = in->reset;
        global_refetch = in->refetch || rd.predecode_refetch_snapshot;
        refetch_address = in->refetch ? in->refetch_address
                                      : rd.predecode_refetch_address_snapshot;
        
        // ========================================================================
        // 阶段 2: 确定各 FIFO 的读使能（在实际读取前先决策）
        // ========================================================================
        
        // fetch_address_FIFO 读使能：icache 准备好接收 且 FIFO 非空
        // 需要先获取 icache 的 ready 状态
        icache_in.reset = global_reset;
        icache_in.refetch = global_refetch;
        icache_in.itlb_flush = in->itlb_flush;
        icache_in.fence_i = in->fence_i;
        icache_in.invalidate_req = false;
        icache_in.csr_status = in->csr_status;
        icache_peek_ready(&icache_in, &icache_out);
        icache_ready = icache_out.icache_read_ready;
        icache_ready_2 = icache_out.icache_read_ready_2;

        fetch_addr_fifo_read_enable_slot0 =
            icache_ready && !rd.fetch_addr_fifo_empty_latch_snapshot && !global_reset &&
            !global_refetch;
        // instruction_FIFO 和 PTAB 读使能：predecode checker 可以工作
        inst_fifo_read_enable =
            !rd.fifo_empty_latch_snapshot && !rd.ptab_empty_latch_snapshot &&
            !rd.front2back_fifo_full_latch_snapshot && !global_reset && !global_refetch;
        ptab_read_enable = inst_fifo_read_enable;
        // front2back_FIFO 读使能：后端请求读取
        front2back_read_enable = in->FIFO_read_enable;
        
        // ========================================================================
        // 阶段 3: 执行所有 FIFO 的读操作（获取输出数据）
        // ========================================================================
        fetch_addr_fifo_in.reset = global_reset;
        fetch_addr_fifo_in.refetch = global_refetch;
        fetch_addr_fifo_in.read_enable = fetch_addr_fifo_read_enable_slot0;
        fetch_addr_fifo_in.write_enable = false;
        fetch_addr_fifo_in.fetch_address = 0;

        fifo_in.reset = global_reset;
        fifo_in.refetch = global_refetch;
        fifo_in.read_enable = inst_fifo_read_enable;
        fifo_in.write_enable = false;

        ptab_in.reset = global_reset;
        ptab_in.refetch = global_refetch;
        ptab_in.read_enable = ptab_read_enable;
        ptab_in.write_enable = false;

        front2back_fifo_in.reset = global_reset;
        front2back_fifo_in.refetch = in->refetch;
        front2back_fifo_in.read_enable = front2back_read_enable;
        front2back_fifo_in.write_enable = false;

        fetch_address_FIFO_comb_calc(&fetch_addr_fifo_in, &fetch_addr_fifo_rd,
                                     &fetch_addr_fifo_out, &fetch_addr_fifo_next_rd,
                                     &fetch_addr_fifo_step_req);
        accumulate_fetch_addr_req(fetch_addr_fifo_req, fetch_addr_fifo_step_req);
        fetch_addr_fifo_rd = fetch_addr_fifo_next_rd;

        saved_fetch_addr_fifo_out_0 = fetch_addr_fifo_out;

        bool fetch_addr_fifo_read_enable_slot1 = false;
        if (fetch_addr_fifo_read_enable_slot1) {
            fetch_addr_fifo_in.read_enable = true;
            fetch_addr_fifo_in.write_enable = false;
            fetch_addr_fifo_in.fetch_address = 0;
            fetch_address_FIFO_comb_calc(&fetch_addr_fifo_in, &fetch_addr_fifo_rd,
                                         &fetch_addr_fifo_out, &fetch_addr_fifo_next_rd,
                                         &fetch_addr_fifo_step_req);
            accumulate_fetch_addr_req(fetch_addr_fifo_req, fetch_addr_fifo_step_req);
            fetch_addr_fifo_rd = fetch_addr_fifo_next_rd;
            saved_fetch_addr_fifo_out_1 = fetch_addr_fifo_out;
        }
        
        instruction_FIFO_comb_calc(&fifo_in, &fifo_rd, &fifo_out, &fifo_next_rd,
                                   &fifo_step_req);
        accumulate_instruction_fifo_req(fifo_req, fifo_step_req);
        fifo_rd = fifo_next_rd;
        
        PTAB_comb_calc(&ptab_in, &ptab_rd, &ptab_out, &ptab_next_rd, &ptab_step_req);
        accumulate_ptab_req(ptab_req, ptab_step_req);
        ptab_rd = ptab_next_rd;
        
        front2back_FIFO_comb_calc(&front2back_fifo_in, &front2back_fifo_rd,
                                  &front2back_fifo_out, &front2back_fifo_next_rd,
                                  &front2back_fifo_step_req);
        accumulate_front2back_req(front2back_fifo_req, front2back_fifo_step_req);
        front2back_fifo_rd = front2back_fifo_next_rd;
    }
    
    // 保存读出的数据用于后续处理
    struct instruction_FIFO_out saved_fifo_out = fifo_out;
    struct PTAB_out saved_ptab_out = ptab_out;
    struct front2back_FIFO_out saved_front2back_fifo_out = front2back_fifo_out;
    struct instruction_FIFO_out bypass_fifo_out;
    memset(&bypass_fifo_out, 0, sizeof(bypass_fifo_out));
    struct front2back_FIFO_out bypass_front2back_fifo_out;
    memset(&bypass_front2back_fifo_out, 0, sizeof(bypass_front2back_fifo_out));
    bool use_icache_to_predecode_bypass = false;
    bool use_front2back_output_bypass = false;
    bool predecode_can_run = false;
    bool predecode_source_valid = inst_fifo_read_enable;
    
    bool bpu_stall = false;
    bool bpu_can_run = false;
    bool can_bypass_fetch_to_icache = false;
    BPU_TOP::OutputPayload bpu_output{};
    {
        // ========================================================================
        // 阶段 4: BPU 控制逻辑
        // ========================================================================
        bpu_stall = rd.fetch_addr_fifo_full_latch_snapshot || rd.ptab_full_latch_snapshot;
        bpu_can_run = !bpu_stall || global_reset || global_refetch;
        BPU_TOP::InputPayload bpu_input{};
        bpu_input.refetch = global_refetch;
        bpu_input.refetch_address = refetch_address;
        bpu_input.icache_read_ready = bpu_can_run && !rd.fetch_addr_fifo_full_latch_snapshot;
        for (int i = 0; i < COMMIT_WIDTH; i++) {
            bpu_input.in_update_base_pc[i] = in->predict_base_pc[i];
            bpu_input.in_upd_valid[i] = in->back2front_valid[i];
            bpu_input.in_actual_dir[i] = in->actual_dir[i];
            bpu_input.in_actual_br_type[i] = in->actual_br_type[i];
            bpu_input.in_actual_targets[i] = in->actual_target[i];
            bpu_input.in_pred_dir[i] = in->predict_dir[i];
            bpu_input.in_alt_pred[i] = in->alt_pred[i];
            bpu_input.in_pcpn[i] = in->pcpn[i];
            bpu_input.in_altpcpn[i] = in->altpcpn[i];
            for (int j = 0; j < 4; j++) {
                bpu_input.in_tage_tags[i][j] = in->tage_tag[i][j];
                bpu_input.in_tage_idxs[i][j] = in->tage_idx[i][j];
            }
            bpu_input.in_sc_used[i] = in->sc_used[i];
            bpu_input.in_sc_pred[i] = in->sc_pred[i];
            bpu_input.in_sc_sum[i] = in->sc_sum[i];
            for (int t = 0; t < BPU_SCL_META_NTABLE; ++t) {
                bpu_input.in_sc_idx[i][t] = in->sc_idx[i][t];
            }
            bpu_input.in_loop_used[i] = in->loop_used[i];
            bpu_input.in_loop_hit[i] = in->loop_hit[i];
            bpu_input.in_loop_pred[i] = in->loop_pred[i];
            bpu_input.in_loop_idx[i] = in->loop_idx[i];
            bpu_input.in_loop_tag[i] = in->loop_tag[i];
        }
        bpu_seq_txn_req.valid = false;
        bpu_seq_txn_req.reset = in->reset;
        if (in->reset) {
            bpu_seq_txn_req.valid = true;
            bpu_seq_txn_req.inp = bpu_input;
            bpu_seq_txn_req.req = BPU_TOP::UpdateRequest{};
            bpu_seq_txn_req.reset = true;
            bpu_output.fetch_address = RESET_PC;
        } else {
            BPU_TOP::ReadData bpu_rd;
            bpu_instance.bpu_seq_read(bpu_input, bpu_rd);
            bpu_instance.bpu_comb_calc(bpu_input, bpu_rd, bpu_output,
                                       bpu_seq_txn_req.req);
            bpu_seq_txn_req.valid = true;
            bpu_seq_txn_req.inp = bpu_input;
            bpu_seq_txn_req.reset = false;
        }
        
        if (bpu_output.icache_read_valid && bpu_can_run) {

        }
        
        // ========================================================================
        // 阶段 5: fetch_address_FIFO 写控制（支持双写和Mini Flush）
        // ========================================================================
        fetch_addr_fifo_in.reset = false;
        fetch_addr_fifo_in.refetch = false;
        fetch_addr_fifo_in.read_enable = false;
        bool normal_write_enable =
            bpu_output.icache_read_valid && bpu_can_run && !global_reset;
        bool refetch_write_enable = normal_write_enable && global_refetch; 
        (void)refetch_write_enable;

        can_bypass_fetch_to_icache = false;
        if (normal_write_enable) {
            fetch_addr_fifo_in.write_enable = true;
            fetch_addr_fifo_in.fetch_address = bpu_output.fetch_address;

            fetch_address_FIFO_comb_calc(&fetch_addr_fifo_in, &fetch_addr_fifo_rd,
                                         &fetch_addr_fifo_out, &fetch_addr_fifo_next_rd,
                                         &fetch_addr_fifo_step_req);
            accumulate_fetch_addr_req(fetch_addr_fifo_req, fetch_addr_fifo_step_req);
            fetch_addr_fifo_rd = fetch_addr_fifo_next_rd;
        }
    }

    bool icache_slot0_data_valid = false;
    bool icache_slot1_data_valid = false;
    {
        // ========================================================================
        // 阶段 6: icache 控制逻辑
        // ========================================================================
        icache_in.reset = global_reset;
        icache_in.refetch = global_refetch;
        icache_in.itlb_flush = in->itlb_flush;
        icache_in.fence_i = in->fence_i;
        icache_in.invalidate_req = false;
        icache_in.csr_status = in->csr_status;
        icache_in.run_comb_only = false;
        
        if (saved_fetch_addr_fifo_out_0.read_valid) {
            icache_in.icache_read_valid = true;
            icache_in.fetch_address = saved_fetch_addr_fifo_out_0.fetch_address;

        } else if (can_bypass_fetch_to_icache) {
            icache_in.icache_read_valid = true;
            icache_in.fetch_address = bpu_output.fetch_address;

        } else {
            icache_in.icache_read_valid = false;
            icache_in.fetch_address = 0;
        }
        icache_in.icache_read_valid_2 = false;
        icache_in.fetch_address_2 = 0;
        assert(!icache_in.icache_read_valid_2);
        
        icache_comb_calc(&icache_in, &icache_out);
        
        // ========================================================================
        // 阶段 7: instruction_FIFO 写控制（写入 icache 返回的数据）
        // ========================================================================
        fifo_in.reset = global_reset;
        fifo_in.refetch = global_refetch;
        fifo_in.read_enable = false;
        fifo_in.write_enable = false;

        icache_slot0_data_valid = icache_out.icache_read_complete;
        assert(!icache_slot1_data_valid);
    }

    {
        bool inst_fifo_full_for_write = rd.fifo_full_latch_snapshot;
        bool can_bypass_icache_to_predecode =
            rd.fifo_empty_latch_snapshot && !rd.ptab_empty_latch_snapshot &&
            !rd.front2back_fifo_full_latch_snapshot && !global_reset &&
            !global_refetch && icache_slot0_data_valid;
        can_bypass_icache_to_predecode = false;
        if (can_bypass_icache_to_predecode) {
            ptab_in.reset = global_reset;
            ptab_in.refetch = global_refetch;
            ptab_in.read_enable = true;
            ptab_in.write_enable = false;
            PTAB_comb_calc(&ptab_in, &ptab_rd, &ptab_out, &ptab_next_rd,
                           &ptab_step_req);
            accumulate_ptab_req(ptab_req, ptab_step_req);
            ptab_rd = ptab_next_rd;
            saved_ptab_out = ptab_out;
            predecode_source_valid = true;

            {
                use_icache_to_predecode_bypass = true;
                for (int i = 0; i < FETCH_WIDTH; i++) {
                    bypass_fifo_out.instructions[i] = icache_out.fetch_group[i];
                    bypass_fifo_out.pc[i] = icache_out.fetch_pc + (i * 4);
                    bypass_fifo_out.page_fault_inst[i] = icache_out.page_fault_inst[i];
                    bypass_fifo_out.inst_valid[i] = icache_out.inst_valid[i];

                    if (icache_out.inst_valid[i]) {
                        uint32_t current_pc = icache_out.fetch_pc + (i * 4);
                        predecode_in predecode_inp{icache_out.fetch_group[i], current_pc};
                        predecode_read_data predecode_rd{};
                        PredecodeResult result{};
                        predecode_seq_read(&predecode_inp, &predecode_rd);
                        predecode_comb(predecode_rd, result);
                        bypass_fifo_out.predecode_type[i] = result.type;
                        bypass_fifo_out.predecode_target_address[i] = result.target_address;
                    } else {
                        bypass_fifo_out.predecode_type[i] = PREDECODE_NON_BRANCH;
                        bypass_fifo_out.predecode_target_address[i] = 0;
                    }
                }

                bypass_fifo_out.seq_next_pc =
                    sequential_next_pc(icache_out.fetch_pc,
                                       icache_out.inst_valid);
                saved_fifo_out = bypass_fifo_out;
            }
        }
        predecode_can_run =
            (predecode_source_valid && use_icache_to_predecode_bypass) ||
            inst_fifo_read_enable;
        if (!inst_fifo_full_for_write && icache_slot0_data_valid && !global_reset &&
            !global_refetch && !use_icache_to_predecode_bypass) {
            fifo_in.write_enable = true;
            for (int i = 0; i < FETCH_WIDTH; i++) {
                fifo_in.fetch_group[i] = icache_out.fetch_group[i];
                fifo_in.pc[i] = icache_out.fetch_pc + (i * 4);
                fifo_in.page_fault_inst[i] = icache_out.page_fault_inst[i];
                fifo_in.inst_valid[i] = icache_out.inst_valid[i];

                if (icache_out.inst_valid[i]) {
                    uint32_t current_pc = icache_out.fetch_pc + (i * 4);
                    predecode_in predecode_inp{icache_out.fetch_group[i], current_pc};
                    predecode_read_data predecode_rd{};
                    PredecodeResult result{};
                    predecode_seq_read(&predecode_inp, &predecode_rd);
                    predecode_comb(predecode_rd, result);
                    fifo_in.predecode_type[i] = result.type;
                    fifo_in.predecode_target_address[i] = result.target_address;

                } else {
                    fifo_in.predecode_type[i] = PREDECODE_NON_BRANCH;
                    fifo_in.predecode_target_address[i] = 0;
                }
            }

            fifo_in.seq_next_pc = sequential_next_pc(
                icache_out.fetch_pc, icache_out.inst_valid);

            instruction_FIFO_comb_calc(&fifo_in, &fifo_rd, &fifo_out, &fifo_next_rd,
                                       &fifo_step_req);
            accumulate_instruction_fifo_req(fifo_req, fifo_step_req);
            fifo_rd = fifo_next_rd;
            inst_fifo_full_for_write = fifo_out.full;
        }

        bool can_write_slot1 = !inst_fifo_full_for_write && icache_slot1_data_valid
                               && !global_reset && !global_refetch;
        if (can_write_slot1) {
            fifo_in.write_enable = true;
            for (int i = 0; i < FETCH_WIDTH; i++) {
                fifo_in.fetch_group[i] = icache_out.fetch_group_2[i];
                fifo_in.pc[i] = icache_out.fetch_pc_2 + (i * 4);
                fifo_in.page_fault_inst[i] = icache_out.page_fault_inst_2[i];
                fifo_in.inst_valid[i] = icache_out.inst_valid_2[i];

                if (icache_out.inst_valid_2[i]) {
                    uint32_t current_pc = icache_out.fetch_pc_2 + (i * 4);
                    predecode_in predecode_inp{icache_out.fetch_group_2[i], current_pc};
                    predecode_read_data predecode_rd{};
                    PredecodeResult result{};
                    predecode_seq_read(&predecode_inp, &predecode_rd);
                    predecode_comb(predecode_rd, result);
                    fifo_in.predecode_type[i] = result.type;
                    fifo_in.predecode_target_address[i] = result.target_address;

                } else {
                    fifo_in.predecode_type[i] = PREDECODE_NON_BRANCH;
                    fifo_in.predecode_target_address[i] = 0;
                }
            }

            fifo_in.seq_next_pc = sequential_next_pc(
                icache_out.fetch_pc_2, icache_out.inst_valid_2);

            instruction_FIFO_comb_calc(&fifo_in, &fifo_rd, &fifo_out, &fifo_next_rd,
                                       &fifo_step_req);
            accumulate_instruction_fifo_req(fifo_req, fifo_step_req);
            fifo_rd = fifo_next_rd;
        }
    }
    
    bool do_predecode_flush = false;
    uint32_t predecode_flush_address = 0;
    {
        bool ptab_can_write = bpu_output.PTAB_write_enable &&
                              !rd.ptab_full_latch_snapshot && !global_reset &&
                              !global_refetch;
        
        FrontPtabWriteCombOut ptab_write_out{};
        FrontPtabWriteCombIn ptab_write_in{};
        ptab_write_in.bpu_output = bpu_output;
        ptab_write_in.global_reset = global_reset;
        ptab_write_in.global_refetch = global_refetch;
        ptab_write_in.ptab_can_write = ptab_can_write;
        front_ptab_write_comb(ptab_write_in, ptab_write_out);
        ptab_in = ptab_write_out.ptab_in;

        if (ptab_can_write) {

            
            PTAB_comb_calc(&ptab_in, &ptab_rd, &ptab_out, &ptab_next_rd,
                           &ptab_step_req);
            accumulate_ptab_req(ptab_req, ptab_step_req);
            ptab_rd = ptab_next_rd;
        }
        
        struct predecode_checker_out checker_out;
        memset(&checker_out, 0, sizeof(checker_out));
        
        if (predecode_can_run) {
            for (int i = 0; i < FETCH_WIDTH; i++) {
                if (saved_fifo_out.pc[i] != saved_ptab_out.predict_base_pc[i]) {
                    printf("ERROR: fifo pc[%d]: %x != ptab pc[%d]: %x\n",
                           i, saved_fifo_out.pc[i], i, saved_ptab_out.predict_base_pc[i]);
                    exit(1);
                }
            }
            
            FrontCheckerInputCombOut checker_input_out{};
            FrontCheckerInputCombIn checker_input_in{};
            checker_input_in.fifo_out = saved_fifo_out;
            checker_input_in.ptab_out = saved_ptab_out;
            front_checker_input_comb(checker_input_in, checker_input_out);
            predecode_checker_read_data checker_rd{};
            predecode_checker_seq_read(&checker_input_out.checker_in, &checker_rd);
            predecode_checker_comb(checker_rd, checker_out);
            

            
            if (checker_out.predecode_flush_enable) {
                do_predecode_flush = true;
                predecode_flush_address =
                    checker_out.predict_next_fetch_address_corrected;
            }
        }
        
        bool front2back_can_write = predecode_can_run &&
                                    !rd.front2back_fifo_full_latch_snapshot &&
                                    !global_reset;
        bool can_bypass_front2back_to_output =
            front2back_read_enable && rd.front2back_fifo_empty_latch_snapshot &&
            !saved_front2back_fifo_out.front2back_FIFO_valid && front2back_can_write;
        if (can_bypass_front2back_to_output) {
            use_front2back_output_bypass = true;
        }


        
        front2back_fifo_in.reset = global_reset;
        front2back_fifo_in.refetch = in->refetch;
        front2back_fifo_in.read_enable = false;
        front2back_fifo_in.write_enable =
            front2back_can_write && !can_bypass_front2back_to_output;
        
        if (front2back_can_write) {
            FrontFront2backWriteCombOut front2back_write_out{};
            FrontFront2backWriteCombIn front2back_write_in{};
            front2back_write_in.fifo_out = saved_fifo_out;
            front2back_write_in.ptab_out = saved_ptab_out;
            front2back_write_in.checker_out = checker_out;
            front2back_write_in.use_front2back_output_bypass =
                use_front2back_output_bypass;
            front_front2back_write_comb(front2back_write_in, front2back_write_out);
            front2back_fifo_in.fetch_group[0] =
                front2back_write_out.front2back_fifo_in.fetch_group[0];
            front2back_fifo_in = front2back_write_out.front2back_fifo_in;
            front2back_fifo_in.reset = global_reset;
            front2back_fifo_in.refetch = in->refetch;
            front2back_fifo_in.read_enable = false;
            front2back_fifo_in.write_enable =
                front2back_can_write && !can_bypass_front2back_to_output;
            bypass_front2back_fifo_out = front2back_write_out.bypass_front2back_fifo_out;
            
            if (front2back_fifo_in.write_enable) {
                front2back_FIFO_comb_calc(&front2back_fifo_in, &front2back_fifo_rd,
                                          &front2back_fifo_out,
                                          &front2back_fifo_next_rd,
                                          &front2back_fifo_step_req);
                accumulate_front2back_req(front2back_fifo_req,
                                          front2back_fifo_step_req);
                front2back_fifo_rd = front2back_fifo_next_rd;
            }
        }
        
        front_state_req.valid = true;
        if (do_predecode_flush) {
            struct icache_in invalidate_req_in;
            memset(&invalidate_req_in, 0, sizeof(invalidate_req_in));
            invalidate_req_in.invalidate_req = true;
            invalidate_req_in.csr_status = in->csr_status;
            icache_comb_calc(&invalidate_req_in, &icache_out);

            front_state_req.next_predecode_refetch = true;
            front_state_req.next_predecode_refetch_address = predecode_flush_address;
            

        } else {
            front_state_req.next_predecode_refetch = false;
            front_state_req.next_predecode_refetch_address = 0;
        }
    }
    
    {
        front_state_req.next_fetch_addr_fifo_full =
            (fetch_addr_fifo_rd.size >= (FETCH_ADDR_FIFO_SIZE - 1));
        front_state_req.next_fetch_addr_fifo_empty = (fetch_addr_fifo_rd.size == 0);
        front_state_req.next_fifo_full = (fifo_rd.size == INSTRUCTION_FIFO_SIZE);
        front_state_req.next_fifo_empty = (fifo_rd.size == 0);
        front_state_req.next_ptab_full = (ptab_rd.size >= (PTAB_SIZE - 1));
        front_state_req.next_ptab_empty = (ptab_rd.size == 0);
        front_state_req.next_front2back_fifo_full =
            (front2back_fifo_rd.size == FRONT2BACK_FIFO_SIZE);
        front_state_req.next_front2back_fifo_empty = (front2back_fifo_rd.size == 0);
        
        FrontOutputCombOut final_output_out{};
        FrontOutputCombIn final_output_in{};
        final_output_in.saved_front2back_fifo_out = saved_front2back_fifo_out;
        final_output_in.bypass_front2back_fifo_out = bypass_front2back_fifo_out;
        final_output_in.use_front2back_output_bypass = use_front2back_output_bypass;
        front_output_comb(final_output_in, final_output_out);
        *out_ptr = final_output_out.out;
        out_ptr->commit_stall = bpu_output.update_queue_full;


        for (int i = 0; i < FETCH_WIDTH; i++) {
            if (out_ptr->inst_valid[i]) {

            }
        }
        
        front_state_req.next_front_sim_time = front_sim_time;
        fetch_addr_fifo_final_req = fetch_addr_fifo_req;
        fifo_final_req = fifo_req;
        ptab_final_req = ptab_req;
        front2back_fifo_final_req = front2back_fifo_req;
        front_deadlock_snapshot.valid = true;
        front_deadlock_snapshot.global_reset = global_reset;
        front_deadlock_snapshot.global_refetch = global_refetch;
        front_deadlock_snapshot.icache_ready = icache_ready;
        front_deadlock_snapshot.icache_ready_2 = icache_ready_2;
        front_deadlock_snapshot.fetch_addr_read_enable_slot0 =
            fetch_addr_fifo_read_enable_slot0;
        front_deadlock_snapshot.fetch_addr_read_enable_slot1 =
            fetch_addr_fifo_read_enable_slot1_candidate;
        front_deadlock_snapshot.inst_fifo_read_enable = inst_fifo_read_enable;
        front_deadlock_snapshot.ptab_read_enable = ptab_read_enable;
        front_deadlock_snapshot.front2back_read_enable = front2back_read_enable;
        front_deadlock_snapshot.icache = icache_out;
        front_deadlock_snapshot.fifo = fifo_out;
        front_deadlock_snapshot.ptab = ptab_out;
        front_deadlock_snapshot.front2back = front2back_fifo_out;
        front_deadlock_snapshot.out = *out_ptr;
        req.out_regs = out;
    }
}

namespace {

void front_seq_read(const struct front_top_in &inp, FrontReadData &rd) {
  rd.predecode_refetch_snapshot = predecode_refetch;
  rd.predecode_refetch_address_snapshot = predecode_refetch_address;
  rd.front_sim_time_snapshot = front_sim_time;
  rd.fetch_addr_fifo_full_latch_snapshot = fetch_addr_fifo_full_latch;
  rd.fetch_addr_fifo_empty_latch_snapshot = fetch_addr_fifo_empty_latch;
  rd.fifo_full_latch_snapshot = fifo_full_latch;
  rd.fifo_empty_latch_snapshot = fifo_empty_latch;
  rd.ptab_full_latch_snapshot = ptab_full_latch;
  rd.ptab_empty_latch_snapshot = ptab_empty_latch;
  rd.front2back_fifo_full_latch_snapshot = front2back_fifo_full_latch;
  rd.front2back_fifo_empty_latch_snapshot = front2back_fifo_empty_latch;

  fetch_address_FIFO_in fetch_addr_in;
  instruction_FIFO_in fifo_in;
  PTAB_in ptab_in;
  front2back_FIFO_in front2back_in;
  icache_in icache_inp;
  icache_out icache_outp;
  std::memset(&fetch_addr_in, 0, sizeof(fetch_addr_in));
  std::memset(&fifo_in, 0, sizeof(fifo_in));
  std::memset(&ptab_in, 0, sizeof(ptab_in));
  std::memset(&front2back_in, 0, sizeof(front2back_in));
  std::memset(&icache_inp, 0, sizeof(icache_inp));
  icache_inp.csr_status = inp.csr_status;
  std::memset(&icache_outp, 0, sizeof(icache_outp));

  fetch_address_FIFO_seq_read(&fetch_addr_in, &rd.fetch_addr_fifo_rd_snapshot);
  instruction_FIFO_seq_read(&fifo_in, &rd.fifo_rd_snapshot);
  PTAB_seq_read(&ptab_in, &rd.ptab_rd_snapshot);
  front2back_FIFO_seq_read(&front2back_in, &rd.front2back_fifo_rd_snapshot);
  icache_seq_read(&icache_inp, &icache_outp);
  (void)inp;
}

void front_seq_write(const struct front_top_in &inp, const FrontUpdateRequest &req,
                     bool reset) {
  (void)inp;
  (void)reset;
  if (req.bpu_seq_txn.valid) {
    bpu_instance.bpu_seq_write(req.bpu_seq_txn.inp, req.bpu_seq_txn.req,
                               req.bpu_seq_txn.reset);
  }
  fetch_address_FIFO_seq_write(&req.fetch_addr_fifo_req);
  instruction_FIFO_seq_write(&req.fifo_req);
  PTAB_seq_write(&req.ptab_req);
  front2back_FIFO_seq_write(&req.front2back_fifo_req);
  predecode_checker_seq_write();
  icache_seq_write();

  if (req.front_state.valid) {
    front_sim_time = req.front_state.next_front_sim_time;
    predecode_refetch = req.front_state.next_predecode_refetch;
    predecode_refetch_address = req.front_state.next_predecode_refetch_address;
    fetch_addr_fifo_full_latch = req.front_state.next_fetch_addr_fifo_full;
    fetch_addr_fifo_empty_latch = req.front_state.next_fetch_addr_fifo_empty;
    fifo_full_latch = req.front_state.next_fifo_full;
    fifo_empty_latch = req.front_state.next_fifo_empty;
    ptab_full_latch = req.front_state.next_ptab_full;
    ptab_empty_latch = req.front_state.next_ptab_empty;
    front2back_fifo_full_latch = req.front_state.next_front2back_fifo_full;
    front2back_fifo_empty_latch = req.front_state.next_front2back_fifo_empty;
  }
}

} // namespace

void front_top_seq_read(const struct front_top_in &inp, FrontReadData &rd) {
  front_seq_read(inp, rd);
}

void front_top_seq_write(const struct front_top_in &inp,
                         const FrontUpdateRequest &req, bool reset) {
  front_seq_write(inp, req, reset);
}

void front_top(struct front_top_in *in, struct front_top_out *out) {
  assert(in);
  assert(out);

  FrontReadData rd;
  FrontUpdateRequest req;
  front_seq_read(*in, rd);
  front_comb_calc(*in, rd, *out, req);
  front_seq_write(*in, req, in->reset);
}

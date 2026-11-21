#ifndef FRONT_IO_H
#define FRONT_IO_H

#include "frontend.h"
#include "wire_types.h"
#include <cstdint>
#include <type_traits>

struct CsrStatusIO;

// FE_PAIR: a fetch-address entry may carry a second fetch block that is read in the same cycle (fetch across a predicted-taken branch).
// The first block holds a_len instructions (up to and including its taken branch), the second b_len instructions starting at addr2.
struct FetchPair {
  bool valid = false;
  pc_t addr2 = 0;
  uint8_t a_len = 0, b_len = 0;
};

// The front end's inputs: control, the back end's redirect, the BPU training of the committing instructions.
struct FrontCtrl {
  wire<1> reset;
  wire<1> itlb_flush;         // SFENCE.VMA
  wire<1> fence_i;
  wire<1> fifo_read_enable;   // the back end takes a fetch group this cycle
};
struct FrontRedirect {        // a back-end mispredict / flush: refetch from address
  wire<1> valid;
  fetch_addr_t address;
  wire<1> rf_valid;           // BPA_REPAIR: the redirect is a branch mispredict; the block / slot / type of the branch
  wire<16> rf_blk;
  wire<8> rf_slot, rf_type;
  pc_t rf_pc;
  wire<1> rf_is_rvc;          // the mispredicted branch is a 16-bit instruction
};
struct BpuTrain {             // one committing instruction
  wire<1> valid;
  pc_t base_pc;
  wire<1> is_rvc;             // a 16-bit instruction
  wire<1> pred_dir;
  wire<1> actual_dir;
  br_type_t br_type;
  target_addr_t target;
  wire<1> alt_pred;
  pcpn_t pcpn;
  uint16_t blk_id;
  wire<1> ftq_is_last;
};
struct FrontTopIn {
  FrontCtrl ctrl;
  FrontRedirect redirect;
  BpuTrain train[COMMIT_WIDTH];
};

struct FrontTopOut {
  // Fixed-width interface to the backend. Lanes at or above
  // FETCH_WIDTH are kept invalid by front_output_comb().
  wire<1> FIFO_valid;
  wire<1> commit_stall;
  uint8_t commit_upd_free = 255, commit_tq_free = 255;  // BPU update / uBTB train queue free entries (EXP-028)
  pc_t pc[BACKEND_FETCH_WIDTH];
  wire<1> is_rvc[BACKEND_FETCH_WIDTH] = {};   // 16-bit instruction (RVC=1); the instruction word is then the expanded 32-bit equivalent
  inst_word_t instructions[BACKEND_FETCH_WIDTH];
  wire<1> predict_dir[BACKEND_FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  PredMetaT<BACKEND_FETCH_WIDTH> meta;
  wire<1> page_fault_inst[BACKEND_FETCH_WIDTH];
  wire<1> pf_inst_hi[BACKEND_FETCH_WIDTH] = {};
  wire<1> inst_valid[BACKEND_FETCH_WIDTH];
};



struct ICacheIn {
  wire<1> reset;
  wire<1> refetch;
  wire<1> itlb_flush;
  wire<1> fence_i;
  // cancel in-flight/front-end-visible request state only; keep cache contents
  wire<1> invalidate_req;
  // from BPU
  wire<1> icache_read_valid;
  fetch_addr_t fetch_address;
};

struct ICacheOut {
  // to BPU & instruction FIFO
  wire<1> icache_read_ready;
  wire<1> icache_read_complete; // fetched current fetch group
  // perf / debug observability for front-end attribution
  wire<1> perf_req_fire;
  wire<1> perf_miss_event;
  wire<1> perf_miss_busy;
  wire<1> perf_itlb_hit;
  wire<1> perf_itlb_miss;
  wire<1> perf_itlb_fault;
  // to instruction FIFO
  inst_word_t fetch_group[FETCH_WIDTH];
  wire<1> page_fault_inst[FETCH_WIDTH];
  wire<1> inst_valid[FETCH_WIDTH];
  pc_t fetch_pc; // PC address of the cache line (the address used to fetch instructions from icache)
  pc_t fetch_paddr;           // FE_PAIR: physical address of fetch_pc
  wire<1> fetch_single_word;
};


struct InstFifoOut {
  wire<1> full;
  wire<1> empty;
  // to back-end
  wire<1> FIFO_valid;
  inst_word_t instructions[FETCH_WIDTH];
  pc_t pc[FETCH_WIDTH]; // THIS IS ONLY FOR DEBUG!!! 
  wire<1> page_fault_inst[FETCH_WIDTH];
  wire<1> pf_inst_hi[FETCH_WIDTH] = {};
  wire<1> inst_valid[FETCH_WIDTH];
  predecode_type_t predecode_type[FETCH_WIDTH];
  target_addr_t predecode_target_address[FETCH_WIDTH];
  pc_t seq_next_pc;
  wire<1> is_rvc[FETCH_WIDTH];
  wire<1> start_a[FETCH_WIDTH];
  wire<1> start_b[FETCH_WIDTH];
  uint16_t raw16[FETCH_WIDTH] = {};   // RVC=1: the first halfword of the window at each slot, as fetched (before expansion)
  uint16_t up16 = 0;                  // RVC=1: the upper halfword of the window of the group's last slot (before the carry mask) ...
  wire<1> up_ok = 0;                  // ... and whether it is in the same cache line as the slot (the fetch can deliver it with the group)
  // S2-confirmed A/B mux fallback (intact block A preserved if S2 overrides A so B is cancelled)
  wire<1> has_alt_a = 0;
  inst_word_t alt_instructions[FETCH_WIDTH] = {};
  pc_t alt_pc[FETCH_WIDTH] = {};
  wire<1> alt_page_fault_inst[FETCH_WIDTH] = {};
  wire<1> alt_inst_valid[FETCH_WIDTH] = {};
  predecode_type_t alt_predecode_type[FETCH_WIDTH] = {};
  target_addr_t alt_predecode_target_address[FETCH_WIDTH] = {};
  pc_t alt_seq_next_pc = 0;
  wire<1> alt_is_rvc[FETCH_WIDTH] = {};
  wire<1> alt_start_a[FETCH_WIDTH] = {};
  wire<1> alt_start_b[FETCH_WIDTH] = {};
  uint16_t alt_raw16[FETCH_WIDTH] = {};
  uint16_t alt_up16 = 0;
  wire<1> alt_up_ok = 0;
};


struct PtabOut {
  wire<1> full;
  wire<1> empty;
  // to back-end
  wire<1> predict_dir[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  pc_t predict_base_pc[FETCH_WIDTH];
  wire<1> is_pair = 0;
  // for TAGE update
  PredMeta meta;
};


struct F2bFifoOut {
  wire<1> full;
  wire<1> empty;
  // to back-end
  wire<1> front2back_FIFO_valid; // same as the previous FIFO_valid
  inst_word_t fetch_group[FETCH_WIDTH];
  wire<1> page_fault_inst[FETCH_WIDTH];
  wire<1> pf_inst_hi[FETCH_WIDTH] = {};
  wire<1> inst_valid[FETCH_WIDTH];
  wire<1> is_rvc[FETCH_WIDTH];
  wire<1> predict_dir_corrected[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address_corrected;
  pc_t predict_base_pc[FETCH_WIDTH];
  PredMeta meta;
};

struct InstFifoEntry {
  inst_word_t instructions[FETCH_WIDTH];
  pc_t pc[FETCH_WIDTH];
  wire<1> page_fault_inst[FETCH_WIDTH];
  wire<1> inst_valid[FETCH_WIDTH];
  predecode_type_t predecode_type[FETCH_WIDTH];
  target_addr_t predecode_target_address[FETCH_WIDTH];
  pc_t seq_next_pc;
  wire<1> is_rvc[FETCH_WIDTH];
  wire<1> start_a[FETCH_WIDTH];
  wire<1> start_b[FETCH_WIDTH];
  uint16_t raw16[FETCH_WIDTH] = {};   // RVC=1: the first halfword of the window at each slot, as fetched (before expansion)
  uint16_t up16 = 0;                  // RVC=1: the upper halfword of the window of the group's last slot (before the carry mask) ...
  wire<1> up_ok = 0;                  // ... and whether it is in the same cache line as the slot (the fetch can deliver it with the group)
  wire<1> has_alt_a = 0;
  inst_word_t alt_instructions[FETCH_WIDTH] = {};
  pc_t alt_pc[FETCH_WIDTH] = {};
  wire<1> alt_page_fault_inst[FETCH_WIDTH] = {};
  wire<1> alt_inst_valid[FETCH_WIDTH] = {};
  predecode_type_t alt_predecode_type[FETCH_WIDTH] = {};
  target_addr_t alt_predecode_target_address[FETCH_WIDTH] = {};
  pc_t alt_seq_next_pc = 0;
  wire<1> alt_is_rvc[FETCH_WIDTH] = {};
  wire<1> alt_start_a[FETCH_WIDTH] = {};
  wire<1> alt_start_b[FETCH_WIDTH] = {};
  uint16_t alt_raw16[FETCH_WIDTH] = {};
  uint16_t alt_up16 = 0;
  wire<1> alt_up_ok = 0;
};

struct PtabEntry {
  wire<1> predict_dir[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  pc_t predict_base_pc[FETCH_WIDTH];
  wire<1> is_pair = 0;
  PredMeta meta;
};

struct F2bFifoEntry {
  inst_word_t fetch_group[FETCH_WIDTH];
  wire<1> page_fault_inst[FETCH_WIDTH];
  wire<1> pf_inst_hi[FETCH_WIDTH] = {};
  wire<1> inst_valid[FETCH_WIDTH];
  wire<1> is_rvc[FETCH_WIDTH];
  wire<1> predict_dir_corrected[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address_corrected;
  pc_t predict_base_pc[FETCH_WIDTH];
  PredMeta meta;
};

// The three entry FIFOs' inputs: the control of the cycle and the entry written (read in the same cycle when the
// FIFO is empty: bypass).
template <class Entry> struct FifoIn {
  wire<1> reset;          // clear the FIFO
  wire<1> refetch;        // clear the FIFO (a refetch)
  wire<1> read_enable;    // the read port pops the head
  wire<1> write_enable;   // the write port pushes wdata
  Entry wdata;
};
using InstFifoIn = FifoIn<InstFifoEntry>;
using PtabIn = FifoIn<PtabEntry>;
using F2bFifoIn = FifoIn<F2bFifoEntry>;

struct FetchAddrFifoIn {
  wire<1> reset;
  wire<1> refetch;
  wire<1> read_enable;
  wire<1> write_enable;
  fetch_addr_t fetch_address;
  uint8_t drop_tail_n;  // BPU_PIPE: squash the n youngest entries (override of an older block)
  FetchPair pair;       // FE_PAIR: second block of the entry
};

struct FetchAddrFifoOut {
  wire<1> full;
  wire<1> empty;
  wire<1> read_valid;      // an address was popped this cycle
  fetch_addr_t fetch_address;
  FetchPair pair;
};

#endif

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
  uint32_t addr2 = 0;
  uint8_t a_len = 0, b_len = 0;
};

struct FrontTopIn {
  wire<1> reset;
  // from back-end
  wire<1> back2front_valid[COMMIT_WIDTH];
  wire<1> refetch;
  wire<1> itlb_flush;
  wire<1> fence_i;
  fetch_addr_t refetch_address;
  wire<1> rf_valid;       // BPA_REPAIR: branch mispredict with block / slot / type of the branch
  wire<16> rf_blk;
  wire<8> rf_slot, rf_type;
  pc_t rf_pc;
  wire<1> rf_is_rvc;       // the mispredicted branch is a 16-bit instruction
  pc_t predict_base_pc[COMMIT_WIDTH];
  wire<1> update_is_rvc[COMMIT_WIDTH];   // the committed instruction is a 16-bit instruction
  wire<1> predict_dir[COMMIT_WIDTH];
  wire<1> actual_dir[COMMIT_WIDTH];
  br_type_t actual_br_type[COMMIT_WIDTH];
  target_addr_t actual_target[COMMIT_WIDTH];
  wire<1> alt_pred[COMMIT_WIDTH];
  pcpn_t pcpn[COMMIT_WIDTH];
  tage_idx_t tage_idx[COMMIT_WIDTH][4]; // TN_MAX = 4
  tage_tag_t tage_tag[COMMIT_WIDTH][4]; // TN_MAX = 4
  wire<1> FIFO_read_enable;
  CsrStatusIO *csr_status;
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
  CsrStatusIO *csr_status;
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

struct InstFifoIn {
  wire<1> reset;
  wire<1> refetch;
  // from icache
  wire<1> write_enable;
  inst_word_t fetch_group[FETCH_WIDTH];
  pc_t pc[FETCH_WIDTH]; // THIS IS ONLY FOR DEBUG!!! 
  wire<1> page_fault_inst[FETCH_WIDTH];
  wire<1> inst_valid[FETCH_WIDTH];
  // from back-end
  wire<1> read_enable;
  // from predecode
  predecode_type_t predecode_type[FETCH_WIDTH];
  target_addr_t predecode_target_address[FETCH_WIDTH];
  pc_t seq_next_pc;
  // RVC=1: per slot, the window at that slot is a 16-bit instruction (the word in fetch_group is then the expanded one), and which slots start an instruction
  // if the group begins with an instruction (start_a) or with the second half of the previous block's last instruction (start_b)
  wire<1> is_rvc[FETCH_WIDTH];
  wire<1> start_a[FETCH_WIDTH];
  wire<1> start_b[FETCH_WIDTH];
  uint16_t raw16[FETCH_WIDTH] = {};   // RVC=1: the first halfword of the window at each slot, as fetched (before expansion)
  uint16_t up16 = 0;                  // RVC=1: the upper halfword of the window of the group's last slot (before the carry mask) ...
  wire<1> up_ok = 0;                  // ... and whether it is in the same cache line as the slot (the fetch can deliver it with the group)
};

struct InstFifoOut {
  wire<1> full;
  wire<1> empty;
  // to back-end
  wire<1> FIFO_valid;
  inst_word_t instructions[FETCH_WIDTH];
  pc_t pc[FETCH_WIDTH]; // THIS IS ONLY FOR DEBUG!!! 
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
};

struct PtabIn {
  wire<1> reset;
  wire<1> refetch;
  // from BPU
  wire<1> write_enable;
  wire<1> predict_dir[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  pc_t predict_base_pc[FETCH_WIDTH];
  // for TAGE update
  PredMeta meta;
  // from back-end
  wire<1> read_enable;
};

struct PtabOut {
  wire<1> full;
  wire<1> empty;
  // to back-end
  wire<1> predict_dir[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address;
  pc_t predict_base_pc[FETCH_WIDTH];
  // for TAGE update
  PredMeta meta;
};

struct F2bFifoIn {
  wire<1> reset;
  wire<1> refetch;
  wire<1> write_enable;
  wire<1> read_enable;
  inst_word_t fetch_group[FETCH_WIDTH];
  wire<1> page_fault_inst[FETCH_WIDTH];
  wire<1> inst_valid[FETCH_WIDTH];
  wire<1> is_rvc[FETCH_WIDTH];
  // to back-end
  wire<1> predict_dir_corrected[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address_corrected;
  pc_t predict_base_pc[FETCH_WIDTH];
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
  wire<1> inst_valid[FETCH_WIDTH];
  wire<1> is_rvc[FETCH_WIDTH];
  wire<1> predict_dir_corrected[FETCH_WIDTH];
  fetch_addr_t predict_next_fetch_address_corrected;
  pc_t predict_base_pc[FETCH_WIDTH];
  PredMeta meta;
};

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

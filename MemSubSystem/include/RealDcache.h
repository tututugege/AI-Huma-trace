#pragma once

#include "MSHR.h"
#include "WriteBuffer.h"
#include "config.h"
#include "types.h"
#include "DcacheConfig.h"   
#include "IO.h"
#include "DataPrefetcher.h"
#include <cstdint>

class SimContext;

// ─────────────────────────────────────────────────────────────────────────────
// S1S2Reg — pipeline register between Stage 1 (SRAM read) and Stage 2 (hit check)
// ─────────────────────────────────────────────────────────────────────────────
struct S1S2Reg {
    reg<LSU_LDU_WIDTH+1> icache_req; // latched from in.lsu2dcache->icache_req at comb_stage1(); used for timing the critical path of hit check and MSHR lookup
    
    FillWrite fill_write; // latched from in.mshr2dcache->fill_req at comb_stage1(); used for timing the critical path of hit check and MSHR lookup
    // Load slots
    struct LoadSlot {
        reg<1>     valid    = false;
        reg<32>    addr     = 0;
        reg<32>     req_id   = 0;
        reg<1>    replayed  = false; // whether this load has been replayed due to MSHR full or conflict, used to avoid accepting new requests for the same load and causing starvation when there are multiple back-to-back misses
        reg<1>    bank_conflict = false;
        reg<1>    perf_replay = false;
        reg<1>    pf = false;        // prefetch tag probe riding an idle load port (no response to the LSU)
        reg<1>    pf_llc = false;    // ... whose line only goes to the LLC
    } loads[LSU_LDU_COUNT];

    // Store slots
    struct StoreSlot {
        reg<1>     valid    = false;
        reg<32>    addr     = 0;
        reg<8>     strb     = 0; // byte-enable
        reg<32>    req_id   = 0;
        reg<1>     replayed  = false; // whether this store has been replayed due to MSHR full or conflict, used to avoid accepting new requests for the same store and causing starvation when there are multiple back-to-back misses
        reg<1>     bank_conflict = false;
        reg<1>     perf_replay = false;
    } stores[LSU_STA_COUNT];
};
// Comb phases in this block's own ports (MemSubsystem::init binds them; scripts/comb/modules.py checks them):
//   CONTRACT comb_stage2  in: in.mshr2dcache in.wb2dcache out.dcache2mshr out.dcache2route  out: out.dcache2mshr out.dcache2route out.dcache2wb
//   CONTRACT comb_stage1  in: in.lsu2dcache in.mshr2dcache out.dcache2mshr out.dcache2route  out: out.dcache2mshr out.dcache2wb
class RealDcache {
public:
    void init();
    void seq();

    DcacheINIO in;
    DcacheOUTIO out;
    DcacheArrays arrays;   // the tag / valid / dirty / prefetched / PLRU arrays

    void comb_stage2();   // the array read, stage 2, the array writes, the response to the route block
    void comb_stage1();
    const DcacheS1Reg &s1_q() const { return s1_q_; }   // register: the stage-1 requests of the last cycle (MSHR / WB read it)
    void set_hidden_bank_retry_blocked(
        const bool load_blocked[LSU_LDU_COUNT],
        const bool store_blocked[LSU_STA_COUNT]) {
        for (int i = 0; i < LSU_LDU_COUNT; i++) {
            bank_retry_load_blocked[i] = load_blocked[i];
        }
        for (int i = 0; i < LSU_STA_COUNT; i++) {
            bank_retry_store_blocked[i] = store_blocked[i];
        }
    }

    bool load_retry_pending(int i) const { return q_.bank_retry.loads[i].valid; }
    bool store_retry_pending(int i) const { return q_.bank_retry.stores[i].valid; }

private:
    struct FillReqMerge {
        wire<1> valid = false;
    };
    struct BankRetryLoad {
        reg<1> valid = false;
        reg<32> addr = 0;
        reg<32> req_id = 0;
    };
    struct BankRetryStore {
        reg<1> valid = false;
        reg<32> addr = 0;
        reg<8> strb = 0;
        reg<32> req_id = 0;
    };
    struct BankRetryState {
        BankRetryLoad loads[LSU_LDU_COUNT];
        BankRetryStore stores[LSU_STA_COUNT];
    };

    // registers: q (read in comb through a const reference, written only by seq / init) and d (comb writes it; d == q at the start of every cycle)
    struct Regs {
        S1S2Reg s1s2;               // the stage-1 -> stage-2 pipeline register
        BankRetryState bank_retry;  // per-port hidden retry latches for LSU bank conflicts
    };
    Regs q_, d;
    // Combinational scratch from comb_stage2() to comb_stage1(); not sequenced state.
    FillReqMerge fill_req_merge_wires[LSU_STA_COUNT] = {};
    BankRetryState bank_retry_issue_wires = {};
    bool bank_retry_load_blocked[LSU_LDU_COUNT] = {};
    bool bank_retry_store_blocked[LSU_STA_COUNT] = {};
    // the D-cache's own wires of the cycle (stage 1 -> the array read, stage 2 -> the array write and the response)
    // and its registers (the stage-1 requests, the response metadata), loaded by seq
    struct RespMeta {
        bool valid = false;
        bool is_store = false;
        uint32_t addr = 0;
    };
    DcacheLsuIO resp_raw_{};                                        // stage 2's response, before drive_response
    DcacheLineReadReq line_read_req_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
    DcacheLineReadResp line_read_resp_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
    PendingWrite pending_writes_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
    LruUpdate lru_updates_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
    FillWrite fill_write_{};
    FillIn fill_in_{};
    FillOut fill_out_{};
    bool fill_store_dirty_ = false;   // drive_response -> stage 1: a store merged into the arriving fill
    RespMeta resp_meta_cur_[LSU_LDU_COUNT + LSU_STA_COUNT]{};   // register
    RespMeta resp_meta_nxt_[LSU_LDU_COUNT + LSU_STA_COUNT]{};
    DcacheS1Reg s1_q_{};                                        // register
    void stage1();
    void stage2();
    void drive_response();
    void record_metadata();
public:
    DataPrefetcher pf; // L1D data prefetcher (EXP-009)
};

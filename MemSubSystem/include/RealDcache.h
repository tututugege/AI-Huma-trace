#pragma once

#include "MSHR.h"
#include "WriteBuffer.h"
#include "config.h"
#include "types.h"
#include "DcacheConfig.h"   
#include "IO.h"
#include <cstdint>

class SimContext;

// ─────────────────────────────────────────────────────────────────────────────
// S1S2Reg — pipeline register between Stage 1 (SRAM read) and Stage 2 (hit check)
// ─────────────────────────────────────────────────────────────────────────────
struct S1S2Reg {
    reg<LSU_LDU_WIDTH+1> icache_req; // latched from in.lsu2dcache->icache_req at stage1_comb(); used for timing the critical path of hit check and MSHR lookup
    
    FILLWrite fill_write; // latched from in.mshr2dcache->fill_req at stage1_comb(); used for timing the critical path of hit check and MSHR lookup
    // Load slots
    struct LoadSlot {
        reg<1>     valid    = false;
        reg<32>    addr     = 0;
        reg<32>     req_id   = 0;
        reg<1>    replayed  = false; // whether this load has been replayed due to MSHR full or conflict, used to avoid accepting new requests for the same load and causing starvation when there are multiple back-to-back misses
        reg<1>    bank_conflict = false;
        reg<1>    perf_replay = false;
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
class RealDcache {
public:
    void init();
    void comb();
    void seq();
    void bind_context(SimContext *c) { ctx = c; }

    DcacheINIO in;
    DcacheOUTIO out;

    void stage1_comb();
    void stage2_comb();
    const S1S2Reg &stage1_next_state() const { return s1s2_nxt; }
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

    bool idle_quiescent() const {
        if (s1s2_cur.fill_write.valid) return false;
        for (const auto &l : s1s2_cur.loads) if (l.valid) return false;
        for (const auto &s : s1s2_cur.stores) if (s.valid) return false;
        for (const auto &l : bank_retry_cur.loads) if (l.valid) return false;
        for (const auto &s : bank_retry_cur.stores) if (s.valid) return false;
        return true;
    }
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

    S1S2Reg s1s2_cur; // latched at start of cycle
    S1S2Reg s1s2_nxt; // computed by comb(); committed by seq()
    BankRetryState bank_retry_cur; // per-port hidden retry latches for LSU bank conflicts
    BankRetryState bank_retry_nxt;
    // Combinational scratch from stage2_comb() to stage1_comb(); not sequenced state.
    FillReqMerge fill_req_merge_wires[LSU_STA_COUNT] = {};
    BankRetryState bank_retry_issue_wires = {};
    bool bank_retry_load_blocked[LSU_LDU_COUNT] = {};
    bool bank_retry_store_blocked[LSU_STA_COUNT] = {};
    SimContext *ctx = nullptr;
};

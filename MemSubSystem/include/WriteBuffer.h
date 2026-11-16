#pragma once
#include "AxiMasterPort.h"

#include "DcacheConfig.h"
#include "IO.h"
#include <cstdint>

class SimContext;

struct WriteBufferEntry {
    reg<32> addr;
    bool lsu_origin;
};

struct WBState {
    reg<DCACHE_WB_COUNT_BITS> count; // number of valid entries in the buffer
    reg<DCACHE_WB_BITS> head;  // index of the oldest entry (next to evict)
    reg<1> send;  // flag to indicate if a request is currently being sent

    // reg<1> bypassvalid[LSU_LDU_COUNT];
    // reg<32> bypassdata[LSU_LDU_COUNT];

    // reg<1> mergevalid[LSU_STA_COUNT];
    // reg<1> mergebusy[LSU_STA_COUNT];

    WriteBufferEntry write_buffer[DCACHE_WB_ENTRIES];
};

struct WBIn {
    DcacheWBIO *dcache2wb;              // this cycle's DCache requests (dirty_info)
    const DcacheS1Reg *s1_q = nullptr;  // the stage-1 lookups of the last cycle (bypass_req, merge_req)
    const axi_interconnect::WriteMasterPort_t *axi = nullptr;  // the DCACHE_W port: req.accepted / resp (interconnect side)
};
struct WBOut {
    WBDcacheIO *wb2dcache;
    axi_interconnect::WriteMasterPort_t *axi = nullptr;        // the same port: the request and resp.ready (WB side)

    void clear() {
        wb2dcache->clear();
        clear_master_fields(*axi);
    }
};
class WriteBuffer {
public:
    void comb_axi_to_dcache();  // AXI responses in, this cycle's lookups / free count out to the DCache (before the DCache stage 2)
    void comb_dcache_to_axi();  // the DCache requests in, the AXI request out (after the DCache stage 1)
    WriteBuffer() = default;
    
    void init();

    // Phase 1: compute out.full and out.free_count from current nxt.count.
    // Phase 2: process push requests from in.push_ports[], accept B channel
    // responses, and fill axi_out with the next AW+W request.
    // Reads axi_in (set by caller before invoking); writes axi_out.
    void seq();

    // Input / output signal ports (public for direct access by RealDcache).
    WBIn  in;
    WBOut out;
    WBState cur, nxt;


private:
    void comb_inputs_axi();
    void comb_outputs_dcache();
    void comb_inputs_dcache();
    void comb_outputs_axi();
};

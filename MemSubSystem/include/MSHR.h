#pragma once
#include "AxiMasterPort.h"

#include "config.h"
#include "types.h"
#include "WriteBuffer.h"
#include "DcacheConfig.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "IO.h"

class SimContext;

struct MSHREntry {
    reg<1> valid;
    reg<1> issued;
    reg<32> addr;
    reg<1> merged_store_dirty;
    bool lsu_origin;
    bool is_prefetch;
    bool no_fill;
};


struct MSHRState{
    reg<DCACHE_MSHR_COUNT_BITS> mshr_count; 

    // Hold one blocked AXI read response when WB backpressure prevents
    // immediate consumption. This register is intentionally bounded to one slot.
    reg<1> axi_resp_hold_valid;
    reg<8> axi_resp_hold_id;

    // reg<1> find_hit[LSU_LDU_COUNT + LSU_STA_COUNT]; // combinational hit result for each load/store slot
    MSHREntry mshr_entries[DCACHE_MSHR_ENTRIES];
};

struct MSHRIn{
    DcacheMSHRIO* dcache2mshr;          // this cycle's DCache requests (mshr_req, fill_resp)
    const DcacheS1Reg* s1_q = nullptr;  // the stage-1 lookups of the last cycle (find_req)
    const axi_interconnect::ReadMasterPort_t *axi = nullptr;  // the DCACHE_R port: req.accepted / resp (interconnect side)
};
struct MSHROut{
    MSHRDcacheIO* mshr2dcache;
    axi_interconnect::ReadMasterPort_t *axi = nullptr;        // the same port: the request and resp.ready (MSHR side)
};
// Comb phases in this block's own ports (MemSubsystem::init binds them; scripts/comb/gen_mem_order.py checks these
// against the code and orders MemSubsystem::comb_outputs / comb_inputs from them):
//   CONTRACT comb_axi_to_dcache  in: in.axi in.s1_q  out: out.mshr2dcache
//   CONTRACT comb_dcache_to_axi  in: in.axi in.dcache2mshr out.axi  out: out.axi
class MSHR {
public:
    void comb_axi_to_dcache();  // AXI responses in, this cycle's lookups / free count out to the DCache (before the DCache stage 2)
    void comb_dcache_to_axi();  // the DCache requests in, the AXI request out (after the DCache stage 1)
    MSHR() = default;
    void init();

    void seq(bool sample_perf = true);

    MSHRIn in;
    MSHROut out;
    using Regs = MSHRState;   // registers: q (read in comb through a const reference, written only by seq / init) and d (comb writes it; d == q at the start of every cycle)
    Regs d;
    const Regs &q() const { return q_; }
    bool axi_resp_accepted_this_cycle = false;
    bool nofill_released = false;

private:
    Regs q_;
    void comb_inputs_axi();
    void comb_outputs_dcache();
    void comb_inputs_dcache();
    void comb_outputs_axi();
};

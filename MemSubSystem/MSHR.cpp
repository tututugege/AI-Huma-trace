#include "MSHR.h"
#include "PhysMemory.h"
#include "types.h"
#include "DataPrefetcher.h"

#include <cassert>
#include <cstdio>
#include <cstring>

namespace {
int find_entry_idx(const MSHRState &state, uint32_t line_addr) {
    for (int i = 0; i < DCACHE_MSHR_ENTRIES; i++) {
        const MSHREntry &e = state.mshr_entries[i];
        if (e.valid && e.addr == line_addr) {
            return i;
        }
    }
    return -1;
}

void clear_store_merge(MSHREntry &entry) {
    entry.merged_store_dirty = false;
}

void merge_store_into_entry(MSHREntry &entry, const MSHRReq &req) {
    if (!req.is_store) {
        return;
    }
    entry.merged_store_dirty = true;
}
} // namespace


// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────

static MSHR *g_dbg_mshr = nullptr;

void MSHR::init()
{
    g_dbg_mshr = this;
    q_ = {};
    d = {};
    axi_resp_accepted_this_cycle = false;
}
void MSHR::comb_outputs_dcache()
{   
    out.mshr2dcache->free = DCACHE_MSHR_ENTRIES - d.mshr_count;
    out.mshr2dcache->release_event = nofill_released;

    for(int i=0;i<LSU_LDU_COUNT + LSU_STA_COUNT;i++){
        bool hit = false;
        if (in.s1_q->find_req[i].valid) {
            const uint32_t find_addr =
                get_addr(in.s1_q->find_req[i].set_idx,
                         in.s1_q->find_req[i].tag, 0);
            for (int j = 0; j < DCACHE_MSHR_ENTRIES; j++) {
                MSHREntry &e = d.mshr_entries[j];
                if (e.valid && e.addr == find_addr) {
                    hit = true;
                    if (e.is_prefetch && !in.s1_q->find_req[i].is_prefetch) {
                        // a demand request (load/store) merges into an in-flight prefetch: it must now fill the L1D,
                        // otherwise an LLC-only prefetch would be dropped and the demand request would wait forever
                        e.is_prefetch = false;
                        e.no_fill = false;
                        g_pf_stats.late++;
                    }
                    break;
                }
            }
        }
        out.mshr2dcache->find_resp[i].valid = hit;
        out.mshr2dcache->find_resp[i].hit = hit;
    }

    if(d.axi_resp_hold_valid){
        out.mshr2dcache->fill_req.valid = true;
        const MSHREntry &e = d.mshr_entries[d.axi_resp_hold_id];
        out.mshr2dcache->fill_req.id = d.axi_resp_hold_id;
        out.mshr2dcache->fill_req.addr = e.addr;
        out.mshr2dcache->fill_req.dirty = e.merged_store_dirty;
        out.mshr2dcache->fill_req.lsu_origin = e.lsu_origin;
        out.mshr2dcache->fill_req.is_prefetch = e.is_prefetch;
    }
    else{
        out.mshr2dcache->fill_req.valid = false;
        out.mshr2dcache->fill_req.id = 0;
        out.mshr2dcache->fill_req.dirty = false;
        out.mshr2dcache->fill_req.lsu_origin = false;
        out.mshr2dcache->fill_req.is_prefetch = false;
        out.mshr2dcache->fill_req.way_idx = 0;
        out.mshr2dcache->fill_req.addr = 0;
    }
}
void MSHR::comb_outputs_axi()
{  
    clear_master_fields(*out.axi);

    // demand misses go out before prefetches (two fixed-priority passes over the entries)
    for (int pass = 0; pass < 2 && !out.axi->req.valid; pass++) {
        for(uint32_t i=0; i<DCACHE_MSHR_ENTRIES; i++){
            const MSHREntry &ce = d.mshr_entries[i];
            if (ce.valid && !ce.issued && ce.is_prefetch == (pass == 1))
            {
                out.axi->req.valid = true;
                out.axi->req.addr = ce.addr;
                out.axi->req.total_size = DCACHE_LINE_SIZE - 1;
                out.axi->req.id = i;
                break;
            }
        }
    }
    out.axi->resp.ready =
        in.axi->resp.valid ? axi_resp_accepted_this_cycle
                              : !d.axi_resp_hold_valid;
}

void MSHR::comb_inputs_dcache()
{
    if(in.dcache2mshr->fill_resp.done){
        const uint8_t hold_id =
            static_cast<uint8_t>(in.dcache2mshr->fill_resp.id);
        const bool valid_id = hold_id < DCACHE_MSHR_ENTRIES;
        const bool matches_hold =
            valid_id && d.axi_resp_hold_valid &&
            d.axi_resp_hold_id == hold_id;
        const bool matches_entry =
            valid_id && d.mshr_entries[hold_id].valid &&
            d.mshr_entries[hold_id].issued;
        if (matches_hold && matches_entry) {
            const bool entry_was_valid = d.mshr_entries[hold_id].valid;
            d.mshr_entries[hold_id].valid = false;
            d.mshr_entries[hold_id].issued = false;
            clear_store_merge(d.mshr_entries[hold_id]);
            d.mshr_entries[hold_id].lsu_origin = false;
            d.mshr_entries[hold_id].is_prefetch = false;
            d.mshr_entries[hold_id].no_fill = false;
            if (entry_was_valid && d.mshr_count > 0) {
                d.mshr_count = d.mshr_count - 1;
            }
            if (d.axi_resp_hold_valid && d.axi_resp_hold_id == hold_id) {
                d.axi_resp_hold_valid = false;
                d.axi_resp_hold_id = 0;
            }
        }
    }

    for(int i=0;i<LSU_LDU_COUNT + LSU_STA_COUNT;i++){
        if(in.dcache2mshr->mshr_req[i].valid){
            const AddrFields f = decode(in.dcache2mshr->mshr_req[i].addr);
            const uint32_t line_addr = get_addr(f.set_idx, f.tag, 0);

            int entry_idx = find_entry_idx(d, line_addr);

            if (entry_idx < 0) {
                for(int j=0;j<DCACHE_MSHR_ENTRIES;j++){
                    if(!d.mshr_entries[j].valid){
                        entry_idx = j;
                        d.mshr_entries[j].valid = true;
                        d.mshr_entries[j].issued = false;
                        d.mshr_entries[j].addr = line_addr;
                        clear_store_merge(d.mshr_entries[j]);
                        d.mshr_entries[j].lsu_origin =
                            in.dcache2mshr->mshr_req[i].lsu_origin;
                        d.mshr_entries[j].is_prefetch =
                            in.dcache2mshr->mshr_req[i].is_prefetch;
                        d.mshr_entries[j].no_fill = in.dcache2mshr->mshr_req[i].no_fill;
                        if (in.dcache2mshr->mshr_req[i].is_prefetch) g_pf_stats.issued++;
                        d.mshr_count = d.mshr_count + 1;
                        break;
                    }
                }
            }

            if (entry_idx >= 0) {
                if (d.mshr_entries[entry_idx].is_prefetch && !in.dcache2mshr->mshr_req[i].is_prefetch) {
                    d.mshr_entries[entry_idx].is_prefetch = false;  // a demand request caught up with the prefetch
                    d.mshr_entries[entry_idx].no_fill = false;      // ... and now it fills the L1D like a demand miss
                    g_pf_stats.late++;
                }
                merge_store_into_entry(d.mshr_entries[entry_idx],
                                       in.dcache2mshr->mshr_req[i]);
            }
        }
    }

}
void MSHR::comb_inputs_axi()
{
    axi_resp_accepted_this_cycle = false;
    nofill_released = false;
    const bool req_accepted = in.axi->req.accepted;
    const uint8_t accepted_id = in.axi->req.accepted_id;

    // The AXI/LLC fabric can accept a DCache read and produce the response
    // before the next CPU cycle observes the issued bit. Make the accepted
    // state visible to response handling in this same comb pass.
    if (req_accepted) {
        if (accepted_id < DCACHE_MSHR_ENTRIES) {
            MSHREntry &ne = d.mshr_entries[accepted_id];
            if (ne.valid && !ne.issued) {
                d.mshr_entries[accepted_id].issued = true;
            }
        }
        else
        {
            Assert(false && "Invalid MSHR accepted request ID");
        }
    }

    // ── Accept R channel response ─────────────────────────────────────────────

    if(!d.axi_resp_hold_valid){
        if(in.axi->resp.valid){
            const uint8_t resp_id = in.axi->resp.id;
            bool resp_matches_entry = false;
            if (resp_id < DCACHE_MSHR_ENTRIES) {
                const MSHREntry &e = d.mshr_entries[resp_id];
                resp_matches_entry = e.valid && e.issued;
            }
            if (resp_matches_entry && d.mshr_entries[resp_id].no_fill) {
                // LLC-only prefetch: the response is consumed and dropped, the entry is released without an L1D fill
                MSHREntry &fe = d.mshr_entries[resp_id];
                fe.valid = false; fe.issued = false; clear_store_merge(fe);
                
                fe.lsu_origin = false; fe.is_prefetch = false; fe.no_fill = false;
                if (d.mshr_count > 0) d.mshr_count = d.mshr_count - 1;
                axi_resp_accepted_this_cycle = true;
                nofill_released = true;
            }
            else if (resp_matches_entry) {
                d.axi_resp_hold_valid = true;
                d.axi_resp_hold_id = resp_id;
                axi_resp_accepted_this_cycle = true;
            }
            else {
            }
        }
    }

}
void MSHR::seq(bool sample_perf)
{
    q_ = d;
}

void MSHR::comb_axi_to_dcache() {
    comb_inputs_axi();
    comb_outputs_dcache();
}

void MSHR::comb_dcache_to_axi() {
    comb_inputs_dcache();
    comb_outputs_axi();
}

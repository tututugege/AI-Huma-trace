#include "WriteBuffer.h"
#include "PhysMemory.h"
#include "config.h"
#include "types.h"
#include <cassert>
#include <cstring>

namespace {

static constexpr uint64_t full_line_wstrb_mask() {
    uint64_t mask = 0;
    for (uint32_t i = 0; i < DCACHE_LINE_SIZE && i < 64; i++) {
        mask |= (1ull << i);
    }
    return mask;
}

static uint32_t find_wb_entry(const WriteBufferEntry *entries, uint32_t head,
                                 uint32_t count, uint32_t addr) {
    if(count == 0){
        return DCACHE_WB_ENTRIES; // no valid entry
    }
    uint32_t best_match = DCACHE_WB_ENTRIES;
    const uint32_t line_addr = (addr & ~(DCACHE_LINE_SIZE - 1));
    for (uint32_t i = head, cnt = 0; cnt < count;
         i = (i + 1) % DCACHE_WB_ENTRIES, cnt++) {
        if (entries[i].addr == line_addr) {
            best_match = i;
        }
    }
    return best_match;
}

static bool valid_wb_entry_index(uint32_t idx) {
    return idx < DCACHE_WB_ENTRIES;
} 
}// namespace

// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────
void WriteBuffer::init() {
    std::memset(&q_, 0, sizeof(q_));
    std::memset(&d, 0, sizeof(d));
    out.clear();
}
void WriteBuffer::comb_outputs_dcache() {
    
    out.clear();
    out.wb2dcache->free = DCACHE_WB_ENTRIES - d.count;

    // reg<1> bypassvalid[LSU_LDU_COUNT];
    for(int i=0;i<LSU_LDU_COUNT;i++){
        bool bypassvalid = false;
        if(in.s1_q->bypass_req[i].valid){
            int wb_idx = find_wb_entry(d.write_buffer, d.head, d.count,
                                       in.s1_q->bypass_req[i].addr);
            if(valid_wb_entry_index(wb_idx)){
                bypassvalid = true;
            }
        }
        out.wb2dcache->bypass_resp[i].valid = bypassvalid;
    }

    // reg<1> mergevalid[LSU_STA_COUNT];
    // reg<1> mergebusy[LSU_STA_COUNT];
    for(int i=0;i<LSU_STA_COUNT;i++){
        bool mergevalid = false;
        bool mergebusy = false;
        if(in.s1_q->merge_req[i].valid){
            int wb_idx = find_wb_entry(d.write_buffer, d.head, d.count,
                                       in.s1_q->merge_req[i].addr);
            if(valid_wb_entry_index(wb_idx)){
                if(wb_idx == d.head && d.count > 0 && d.send != 0){
                    mergebusy = true;
                }
                else{
                    mergevalid = true;
                }
            }
        }
        out.wb2dcache->merge_resp[i].valid = mergevalid;
        out.wb2dcache->merge_resp[i].busy = mergebusy;
    }
}
void WriteBuffer::comb_outputs_axi() {
    clear_master_fields(*out.axi);
    out.axi->resp.ready = true;
    if (d.send != 0) {
        return;
    }
    if (d.count == 0) {
        return;
    }
    const WriteBufferEntry &head_e = d.write_buffer[d.head];
    out.axi->req.valid = true;
    out.axi->req.addr = head_e.addr;
    out.axi->req.total_size = DCACHE_LINE_SIZE - 1;
    out.axi->req.id = 0;
    out.axi->req.wstrb = full_line_wstrb_mask();

}

void WriteBuffer::comb_inputs_axi(){
    const bool req_accepted = in.axi->req.accepted;
    const bool resp_valid = in.axi->resp.valid;

    if(req_accepted){
        d.send = 1; // request has been accepted, wait for response before sending next
    }
    if (resp_valid) {
        if (d.count > 0) {
            d.count = d.count - 1;
            d.head = (d.head + 1) % DCACHE_WB_ENTRIES;
        }
        else{
            assert(false && "WriteBuffer underflow: received AXI response when buffer is empty");
        }
        d.send = 0; // allow sending (or retrying) the head entry
    }
}

void WriteBuffer::comb_inputs_dcache() {
    
    if(in.dcache2wb->dirty_info.valid){
        if(d.count < DCACHE_WB_ENTRIES){
            WriteBufferEntry &e = d.write_buffer[(d.head + d.count) % DCACHE_WB_ENTRIES];
            e.addr     = in.dcache2wb->dirty_info.addr;
            e.lsu_origin = in.dcache2wb->dirty_info.lsu_origin;
        }
        else{
                assert(false && "WriteBuffer overflow: MSHR is producing evictions faster than WriteBuffer can drain them");
        }
    }
    if(in.dcache2wb->dirty_info.valid && d.count < DCACHE_WB_ENTRIES){
        d.count = d.count + 1;
    }

}

void WriteBuffer::seq() {
    q_ = d;
}

void WriteBuffer::comb_axi_to_dcache() {
    comb_inputs_axi();
    comb_outputs_dcache();
}

void WriteBuffer::comb_dcache_to_axi() {
    comb_inputs_dcache();
    comb_outputs_axi();
}

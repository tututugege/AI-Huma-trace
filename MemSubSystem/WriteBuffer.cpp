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
    std::memset(&cur, 0, sizeof(cur));
    std::memset(&nxt, 0, sizeof(nxt));
    out.clear();
}
void WriteBuffer::comb_outputs_dcache() {
    
    out.clear();
    out.wb2dcache->free = DCACHE_WB_ENTRIES - nxt.count;

    // reg<1> bypassvalid[LSU_LDU_COUNT];
    for(int i=0;i<LSU_LDU_COUNT;i++){
        bool bypassvalid = false;
        if(in.s1_q->bypass_req[i].valid){
            int wb_idx = find_wb_entry(nxt.write_buffer, nxt.head, nxt.count,
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
            int wb_idx = find_wb_entry(nxt.write_buffer, nxt.head, nxt.count,
                                       in.s1_q->merge_req[i].addr);
            if(valid_wb_entry_index(wb_idx)){
                if(wb_idx == nxt.head && nxt.count > 0 && nxt.send != 0){
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
    if (nxt.send != 0) {
        return;
    }
    if (nxt.count == 0) {
        return;
    }
    const WriteBufferEntry &head_e = nxt.write_buffer[nxt.head];
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
        nxt.send = 1; // request has been accepted, wait for response before sending next
    }
    if (resp_valid) {
        if (nxt.count > 0) {
            nxt.count = nxt.count - 1;
            nxt.head = (nxt.head + 1) % DCACHE_WB_ENTRIES;
        }
        else{
            assert(false && "WriteBuffer underflow: received AXI response when buffer is empty");
        }
        nxt.send = 0; // allow sending (or retrying) the head entry
    }
}

void WriteBuffer::comb_inputs_dcache() {
    
    if(in.dcache2wb->dirty_info.valid){
        if(nxt.count < DCACHE_WB_ENTRIES){
            WriteBufferEntry &e = nxt.write_buffer[(nxt.head + nxt.count) % DCACHE_WB_ENTRIES];
            e.addr     = in.dcache2wb->dirty_info.addr;
            e.lsu_origin = in.dcache2wb->dirty_info.lsu_origin;
        }
        else{
                assert(false && "WriteBuffer overflow: MSHR is producing evictions faster than WriteBuffer can drain them");
        }
    }
    if(in.dcache2wb->dirty_info.valid && nxt.count < DCACHE_WB_ENTRIES){
        nxt.count = nxt.count + 1;
    }

}

void WriteBuffer::seq() {
    cur = nxt;
}

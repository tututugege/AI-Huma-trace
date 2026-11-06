#include "RealDcache.h"
#include "types.h"
#include <cassert>
#include <cstring>
#include <MemUtils.h>

namespace {
static bool dcache_req_id_is_lsu(uint32_t req_id) {
    return (req_id & (1u << 31)) == 0;
}
} // namespace

namespace {
enum class L1DReplayReason {
    Conflict,
    BankConflict,
    MshrFull,
    WaitMshrHit,
    WaitMshrFirstAlloc,
    WaitMshrFillReq,
    WaitMshrFillWrite,
};

static bool l1d_perf_is_lsu_req(uint32_t req_id) {
    return dcache_req_id_is_lsu(req_id);
}







static void count_l1d_fillout_bank_grant(SimContext *ctx) {
    if (ctx != nullptr) {

    }
}

static void count_l1d_fillout_bank_conflict(SimContext *ctx, uint64_t count) {
    if (ctx != nullptr) {

    }
}
} // namespace

void RealDcache::init() {
    init_dcache();
    s1s2_cur = {};
    s1s2_nxt = {};
    bank_retry_cur = {};
    bank_retry_nxt = {};
    bank_retry_issue_wires = {};
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        bank_retry_load_blocked[i] = false;
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        bank_retry_store_blocked[i] = false;
    }
    for (auto &entry : fill_req_merge_wires) {
        entry = {};
    }
}

void RealDcache::stage1_comb() {
    bank_retry_nxt = bank_retry_cur;

    auto line_id = [](uint32_t addr) -> uint32_t {
        return addr >> DCACHE_OFFSET_BITS;
    };
    auto bank_index = [](const AddrFields &f) -> uint32_t {
        return static_cast<uint32_t>(f.bank);
    };

    AddrFields fill_f = decode(in.mshr2dcache->fill_req.addr);
    const uint32_t fill_line_id = line_id(in.mshr2dcache->fill_req.addr);
    
    bool same_fill_already_in_s2 =
    s1s2_cur.fill_write.valid &&
    in.mshr2dcache->fill_req.valid &&
    s1s2_cur.fill_write.set_idx == fill_f.set_idx &&
    s1s2_cur.fill_write.tag == fill_f.tag;

    const bool hold_fill_in_s2 =
        same_fill_already_in_s2 && !out.fill_write->valid;
    const bool fillout_needs_snapshot =
        in.mshr2dcache->fill_req.valid &&
        !(same_fill_already_in_s2 && out.fill_write->valid);

    bool bank_valid[DCACHE_BANK_NUM] = {};
    uint32_t bank_line_id[DCACHE_BANK_NUM] = {};
    bool bank_from_fillout[DCACHE_BANK_NUM] = {};
    bool fillout_granted = false;
    uint64_t same_line_merge_count = 0;
    uint64_t fillout_bank_conflict_count = 0;

    if (fillout_needs_snapshot) {
        const uint32_t bank = bank_index(fill_f);
        bank_valid[bank] = true;
        bank_line_id[bank] = fill_line_id;
        bank_from_fillout[bank] = true;
        fillout_granted = true;

    }

    auto try_grant_bank = [&](const AddrFields &f, uint32_t req_line_id) -> bool {
        const uint32_t bank = bank_index(f);
        if (!bank_valid[bank]) {
            bank_valid[bank] = true;
            bank_line_id[bank] = req_line_id;
            bank_from_fillout[bank] = false;
            return true;
        }
        if (bank_line_id[bank] == req_line_id) {
            same_line_merge_count++;
            // return true;
        }
        if (bank_from_fillout[bank]) {
            fillout_bank_conflict_count++;
        }
        return false;
    };

    if (hold_fill_in_s2) {
        s1s2_nxt.fill_write = s1s2_cur.fill_write;
    } else {
        s1s2_nxt.fill_write = {};
        if (fillout_granted && !same_fill_already_in_s2) {
            s1s2_nxt.fill_write.valid = true;
            s1s2_nxt.fill_write.id = in.mshr2dcache->fill_req.id;
            s1s2_nxt.fill_write.dirty = in.mshr2dcache->fill_req.dirty;
            s1s2_nxt.fill_write.lsu_origin = in.mshr2dcache->fill_req.lsu_origin;
            s1s2_nxt.fill_write.is_prefetch = in.mshr2dcache->fill_req.is_prefetch;
            s1s2_nxt.fill_write.set_idx = fill_f.set_idx;
            s1s2_nxt.fill_write.tag = fill_f.tag;
        }
    }

    if (s1s2_nxt.fill_write.valid) {
        for (int i = 0; i < LSU_STA_COUNT; i++) {
            if (!fill_req_merge_wires[i].valid) continue;
            s1s2_nxt.fill_write.dirty = true;
        }
    }


    memset(out.dcache2wb->bypass_req, 0, sizeof(out.dcache2wb->bypass_req));
    memset(out.dcache2wb->merge_req, 0, sizeof(out.dcache2wb->merge_req));
    memset(out.dcache2mshr->find_req, 0, sizeof(out.dcache2mshr->find_req));
    memset(out.fillout, 0, sizeof(*out.fillout));

    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        *out.dcachereadreq[i] = {};
    }


    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const LoadReq &lsu_req = in.lsu2dcache->req_ports.load_ports[i];
        const BankRetryLoad &pending_retry = bank_retry_cur.loads[i];
        const BankRetryLoad &new_retry = bank_retry_issue_wires.loads[i];
        const bool retry_blocked = bank_retry_load_blocked[i];
        const bool use_pending_retry =
            !lsu_req.valid && !retry_blocked && pending_retry.valid;
        const bool use_new_retry =
            !lsu_req.valid && !retry_blocked && !use_pending_retry &&
            new_retry.valid;
        const bool use_hidden_retry = use_pending_retry || use_new_retry;
        const bool req_valid =
            lsu_req.valid || use_pending_retry || use_new_retry;
        const uint32_t req_addr =
            lsu_req.valid ? lsu_req.addr
                          : (use_pending_retry ? pending_retry.addr
                                               : new_retry.addr);
        const uint32_t req_id =
            lsu_req.valid ? lsu_req.req_id
                          : (use_pending_retry ? pending_retry.req_id
                                               : new_retry.req_id);
        S1S2Reg::LoadSlot &slot = s1s2_nxt.loads[i];
        slot.pf = false;
        if (use_pending_retry) {
            bank_retry_nxt.loads[i] = {};
        }
        if (new_retry.valid && !use_new_retry) {
            Assert(!bank_retry_nxt.loads[i].valid &&
                   "DCache load bank retry latch overflow");
            bank_retry_nxt.loads[i] = new_retry;
        }
        if (!req_valid){
            slot.valid = false;
            slot.replayed = false;
            slot.bank_conflict = false;
            slot.perf_replay = false;
            out.dcache2wb->bypass_req[i].valid = false;
        }
        else{
            AddrFields f  = decode(req_addr);
            const bool bank_granted = try_grant_bank(f, line_id(req_addr));
            // for(int j=0;j<LSU_STA_COUNT;j++){
            //     if(s1s2_cur.loads[j].valid&&CheckAddr(req.addr,(uint8_t)0xffff, s1s2_cur.stores[j].addr,s1s2_cur.stores[j].strb)){
            //         replay = true; // 如果有older load地址重叠且还在等待dcache响应或者等待重放中（即需要重放），则这个load也需要重放，以避免饥饿。注意我们只检查older load的状态，因为如果older load是hit，说明它可以在同一周期完成并更新cache，从而这个load不需要重放。另一方面，如果我们检查这个load自己的状态，可能会导致当有多条连续miss时出现不必要的重放，因为这个load可能还没有被标记为replay，但它会在同一周期内因为MSHR miss而被重放。
            //         break;
            //     }
            // }
            // for(int j=0;j<LSU_STA_COUNT;j++){
            //     if(in.lsu2dcache->req_ports.store_ports[j].valid&&CheckAddr(req.addr,(uint8_t)0xffff, in.lsu2dcache->req_ports.store_ports[j].addr,in.lsu2dcache->req_ports.store_ports[j].strb)){
            //         replay = true; // 如果有older store地址重叠且还在等待dcache响应或者等待重放中（即需要重放），则这个load也需要重放，以避免饥饿。注意我们只检查older store的状态，因为如果older store是hit，说明它可以在同一周期完成并更新cache，从而这个load不需要重放。另一方面，如果我们检查这个load自己的状态，可能会导致当有多条连续miss时出现不必要的重放，因为这个load可能还没有被标记为replay，但它会在同一周期内因为MSHR miss而被重放。
            //         break;
            //     }
            // }
            slot.valid    = true;
            slot.addr     = req_addr;
            slot.req_id   = req_id;
            slot.replayed  = !bank_granted;
            slot.bank_conflict = !bank_granted;
            slot.perf_replay = use_hidden_retry ? true : lsu_req.replay;
            if (use_hidden_retry) {

            }
            if (!bank_granted) {
                continue;
            }

            out.dcachereadreq[i]->valid = true;
            out.dcachereadreq[i]->set_idx = f.set_idx;
            out.dcachereadreq[i]->word_off = f.word_off;
            out.dcache2wb->bypass_req[i].valid = true;
            out.dcache2wb->bypass_req[i].addr = req_addr;

            out.dcache2mshr->find_req[i].valid = true;
            out.dcache2mshr->find_req[i].set_idx = f.set_idx;
            out.dcache2mshr->find_req[i].tag = f.tag;
        }        
    }

    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const StoreReq &lsu_req = in.lsu2dcache->req_ports.store_ports[i];
        const BankRetryStore &pending_retry = bank_retry_cur.stores[i];
        const BankRetryStore &new_retry = bank_retry_issue_wires.stores[i];
        const bool retry_blocked = bank_retry_store_blocked[i];
        const bool use_pending_retry =
            !lsu_req.valid && !retry_blocked && pending_retry.valid;
        const bool use_new_retry =
            !lsu_req.valid && !retry_blocked && !use_pending_retry &&
            new_retry.valid;
        const bool use_hidden_retry = use_pending_retry || use_new_retry;
        const bool req_valid =
            lsu_req.valid || use_pending_retry || use_new_retry;
        const uint32_t req_addr =
            lsu_req.valid ? lsu_req.addr
                          : (use_pending_retry ? pending_retry.addr
                                               : new_retry.addr);
        const uint8_t req_strb =
            lsu_req.valid ? lsu_req.strb
                          : (use_pending_retry ? pending_retry.strb
                                               : new_retry.strb);
        const uint32_t req_id =
            lsu_req.valid ? lsu_req.req_id
                          : (use_pending_retry ? pending_retry.req_id
                                               : new_retry.req_id);
        S1S2Reg::StoreSlot &slot = s1s2_nxt.stores[i];
        int idx       = LSU_LDU_COUNT + i;
        if (use_pending_retry) {
            bank_retry_nxt.stores[i] = {};
        }
        if (new_retry.valid && !use_new_retry) {
            Assert(!bank_retry_nxt.stores[i].valid &&
                   "DCache store bank retry latch overflow");
            bank_retry_nxt.stores[i] = new_retry;
        }
        if(!req_valid){
            slot.valid = false;
            slot.replayed = false;
            slot.bank_conflict = false;
            slot.perf_replay = false;
            out.dcache2wb->merge_req[i].valid = false;
        }
        else{
            AddrFields f  = decode(req_addr);
            const bool bank_granted = try_grant_bank(f, line_id(req_addr));
            // for(int j=0;j<LSU_STA_COUNT;j++){
            //     if(s1s2_cur.stores[j].valid&&CheckAddr(req.addr,req.strb, s1s2_cur.stores[j].addr,s1s2_cur.stores[j].strb)){
            //         replayed = out.dcache2lsu->resp_ports.store_resps[j].replay != ReplayType::HIT; // If there is an older store with overlapping address that is waiting for MSHR allocation or has been allocated an MSHR but not yet completed (i.e., needs to be replayed), then this store should also be replayed to avoid starvation. Note that we only check the replay status of the older store, because if the older store is a hit, it means it can complete in the same cycle and update the cache before this store's hit check, so this store doesn't need to be replayed. On the other hand, if we check the replay status of this store itself, it may cause unnecessary replays when there are multiple back-to-back misses, because this store may not have been marked as replayed yet when we check, even though it will be replayed in the same cycle due to MSHR miss.
            //         break;
            //     }
            // }
            slot.valid    = true;
            slot.addr     = req_addr;
            slot.strb     = req_strb;
            slot.req_id   = req_id;
            slot.replayed = !bank_granted;
            slot.bank_conflict = !bank_granted;
            slot.perf_replay = use_hidden_retry ? true : lsu_req.replay;
            if (use_hidden_retry) {

            }
            if (!bank_granted) {
                continue;
            }

            out.dcachereadreq[idx]->valid = true;
            out.dcachereadreq[idx]->set_idx = f.set_idx;
            out.dcachereadreq[idx]->word_off = f.word_off;
            out.dcache2wb->merge_req[i].valid = true;
            out.dcache2wb->merge_req[i].addr = req_addr;

            out.dcache2mshr->find_req[idx].valid = true;
            out.dcache2mshr->find_req[idx].set_idx = f.set_idx;
            out.dcache2mshr->find_req[idx].tag = f.tag;
        }
    }

    // Prefetch tag probe: lowest priority, rides an idle load port (it competes for the bank like any load).
    if (pf.enabled() && in.lsu2dcache != nullptr && pf.head_valid()) {
        for (int i = 0; i < LSU_LDU_COUNT; i++) {
            S1S2Reg::LoadSlot &slot = s1s2_nxt.loads[i];
            if (slot.valid) continue;                      // port busy with a demand request or a retry
            const uint32_t pf_addr = pf.head_addr();
            const AddrFields f = decode(pf_addr);
            if (!try_grant_bank(f, line_id(pf_addr))) { g_pf_stats.probe_conflict++; break; }
            slot.valid = true;
            slot.pf = true;
            slot.pf_llc = pf.head_llc_only();
            slot.addr = pf_addr;
            slot.req_id = 1u << 31;                        // not an LSU request
            slot.replayed = false;
            slot.bank_conflict = false;
            slot.perf_replay = true;
            out.dcachereadreq[i]->valid = true;
            out.dcachereadreq[i]->set_idx = f.set_idx;
            out.dcachereadreq[i]->word_off = f.word_off;
            out.dcache2wb->bypass_req[i].valid = true;     // a line still in the write buffer must not be re-read
            out.dcache2wb->bypass_req[i].addr = pf_addr;
            out.dcache2mshr->find_req[i].valid = true;
            out.dcache2mshr->find_req[i].set_idx = f.set_idx;
            out.dcache2mshr->find_req[i].tag = f.tag;
            out.dcache2mshr->find_req[i].is_prefetch = true;
            pf.head_taken = true;
            break;
        }
    }

    out.fillout->valid = fillout_granted;
    out.fillout->set_idx = fillout_granted ? fill_f.set_idx : 0;




    s1s2_nxt.icache_req = in.lsu2dcache->icache_req;
    
}

void RealDcache::stage2_comb() {

    memset(out.dcache2lsu->resp_ports.load_resps, 0, sizeof(out.dcache2lsu->resp_ports.load_resps));
    memset(out.dcache2lsu->resp_ports.store_resps, 0, sizeof(out.dcache2lsu->resp_ports.store_resps));
    memset(out.dcache2mshr->mshr_req, 0, sizeof(out.dcache2mshr->mshr_req));
    memset(&out.dcache2wb->dirty_info, 0, sizeof(out.dcache2wb->dirty_info));
    memset(&out.dcache2mshr->fill_resp, 0, sizeof(out.dcache2mshr->fill_resp));
    
    *out.fill_write = {};
    for(int i=0;i<LSU_LDU_COUNT + LSU_STA_COUNT;i++){
        *out.lru_updates[i] = {};
        *out.pendingwrite[i] = {};
    }
    for (auto &entry : fill_req_merge_wires) {
        entry = {};
    }
    bank_retry_issue_wires = {};

    if(s1s2_cur.fill_write.valid){
        int select_way = choose_plru_tree_victim(in.fillin->plru_tree_state, in.fillin->valid_snap);
        if(in.fillin->valid_snap[select_way] && in.fillin->dirty_snap[select_way]){
            if(in.wb2dcache->free>0){
                out.dcache2wb->dirty_info.valid = true;
                out.dcache2wb->dirty_info.lsu_origin =
                    s1s2_cur.fill_write.lsu_origin;
                out.dcache2wb->dirty_info.addr = get_addr(s1s2_cur.fill_write.set_idx, in.fillin->tag_snap[select_way], 0);

                out.fill_write->valid = true;
                out.fill_write->dirty = s1s2_cur.fill_write.dirty;
                out.fill_write->lsu_origin = s1s2_cur.fill_write.lsu_origin;
                out.fill_write->is_prefetch = s1s2_cur.fill_write.is_prefetch;
                out.fill_write->set_idx = s1s2_cur.fill_write.set_idx;
                out.fill_write->tag = s1s2_cur.fill_write.tag;
                out.fill_write->way_idx = select_way;
                out.dcache2mshr->fill_resp.done = true; // Mark the MSHR fill as done to unblock the waiting load/store, even though the line is not yet filled into the cache. This allows the writeback to be issued in parallel with the fill, improving performance when the evicted line is dirty.
                out.dcache2mshr->fill_resp.id = s1s2_cur.fill_write.id;
            }
            else{
                out.fill_write->valid = false; // Can't accept new fill, so stall the MSHR fill response and the waiting load/store. This may cause deadlock when MSHR and WB are both full, but it's a rare case and can be resolved by replaying the load/store.
                out.dcache2wb->dirty_info.valid = false; // Can't accept new writeback, so treat the line as clean and let it be overwritten without writeback. This may cause data loss but prevents deadlock when MSHR and WB are both full.
            }
        }
        else{
            out.fill_write->valid = true;
            out.fill_write->dirty = s1s2_cur.fill_write.dirty;
            out.fill_write->lsu_origin = s1s2_cur.fill_write.lsu_origin;
            out.fill_write->is_prefetch = s1s2_cur.fill_write.is_prefetch;
            out.fill_write->set_idx = s1s2_cur.fill_write.set_idx;
            out.fill_write->tag = s1s2_cur.fill_write.tag;
            out.fill_write->way_idx = select_way;
            out.dcache2mshr->fill_resp.done = true; // Mark the MSHR fill as done to unblock the waiting load/store, even though the line is not yet filled into the cache. This allows the fill to complete and the line to be allocated in cache without waiting for an unnecessary writeback to drain when the evicted line is clean.
            out.dcache2mshr->fill_resp.id = s1s2_cur.fill_write.id;
        }
    }

    int mshr_free_entries = in.mshr2dcache->free;
    int pf_slot = -1;
    // ── Load ports ────────────────────────────────────────────────────────────
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const S1S2Reg::LoadSlot &slot = s1s2_cur.loads[i];
        LoadResp &resp = out.dcache2lsu->resp_ports.load_resps[i];
        DcacheLineReadResp &line_resp = *in.dcachelinereadresp[i];
        resp.valid = false; 
        if (!slot.valid) continue;
        if (slot.pf) { pf_slot = i; continue; }            // handled after the demand requests

        AddrFields f          = decode(slot.addr);
        uint32_t tag_expected = f.tag;

        if(slot.replayed){
            if (slot.bank_conflict && dcache_req_id_is_lsu(slot.req_id) &&
                !bank_retry_cur.loads[i].valid) {
                bank_retry_issue_wires.loads[i].valid = true;
                bank_retry_issue_wires.loads[i].addr = slot.addr;
                bank_retry_issue_wires.loads[i].req_id = slot.req_id;

                continue;
            }
            resp.valid  = true;
            resp.retry = true;
            resp.bank_conflict = slot.bank_conflict;
            resp.replay = ReplayType::CONFLICT; // This load has been replayed due to MSHR full or conflict, so replay again to give chance for the MSHR state to be updated and avoid starvation when there are multiple back-to-back misses.
            resp.req_id = slot.req_id;

            continue;
        }
        int hit_way = -1;
        for (int w = 0; w < DCACHE_WAYS_NUM; w++) {
            if (line_resp.valid[w] && line_resp.tag[w] == tag_expected) {
                hit_way = w;
                break;
            }
        }

        
         if(in.wb2dcache->bypass_resp[i].valid){
            resp.valid = true;
            resp.req_id = slot.req_id;
            resp.retry = false;
            resp.replay = ReplayType::HIT;
        }else if (hit_way >= 0 ) {
            if (!slot.perf_replay && dcache_req_id_is_lsu(slot.req_id)) {
                g_pf_stats.demand_hit++;
                if (pf_array[f.set_idx][hit_way]) {        // first use of a prefetched line
                    pf_array[f.set_idx][hit_way] = false;
                    g_pf_stats.useful++;
                    pf.acc[i] = PfAccess{true, slot.addr, false, true};
                }
            }
            // ── Cache Hit ────────────────────────────────────────────────────
            resp.valid  = true;
            resp.replay = ReplayType::HIT;
            resp.req_id = slot.req_id;
            resp.retry = false;
            out.lru_updates[i]->valid = true;
            out.lru_updates[i]->set_idx = f.set_idx;
            out.lru_updates[i]->way = hit_way;
        }
        else {
            if(s1s2_cur.icache_req==i){
                // ICache coherent probes are lookup-only: report miss to the
                // front-end route block without allocating an MSHR.
                resp.valid = true;
                resp.retry = false;
                resp.replay = ReplayType::CONFLICT;
                resp.req_id = slot.req_id;
                continue;
            }
            if (!slot.perf_replay && dcache_req_id_is_lsu(slot.req_id)) {
                g_pf_stats.demand_miss++;
                pf.acc[i] = PfAccess{true, slot.addr, true, false};
            }
            if(in.mshr2dcache->find_resp[i].valid){
                resp.valid = true;
                resp.replay = ReplayType::MSHR_HIT;
                resp.req_id = slot.req_id;

            }
            else {
                // Miss with no existing MSHR entry: allocate a new MSHR and replay later when it is ready.
                if(mshr_free_entries == 0){
                    resp.valid = true;
                    resp.replay = ReplayType::MSHR_FULL;
                    resp.req_id = slot.req_id;

                }
                else { // The only free MSHR entry is being allocated by the other load port in the same cycle, so can't accept this new miss
                    out.dcache2mshr->mshr_req[i].valid = true;
                    out.dcache2mshr->mshr_req[i].addr = slot.addr;
                    out.dcache2mshr->mshr_req[i].lsu_origin =
                        l1d_perf_is_lsu_req(slot.req_id);

                    resp.valid = true;
                    resp.replay = ReplayType::MSHR_HIT;
                    resp.req_id = slot.req_id;
                    mshr_free_entries--;

                }
            }
        } 
    }

    // ── Store ports ───────────────────────────────────────────────────────────
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const S1S2Reg::StoreSlot &slot = s1s2_cur.stores[i];
        StoreResp &resp = out.dcache2lsu->resp_ports.store_resps[i];
        const int iidx = LSU_LDU_COUNT + i;

        if (!slot.valid) continue;

        AddrFields f          = decode(slot.addr);
        uint32_t tag_expected = f.tag;


        if(slot.replayed){
            if (slot.bank_conflict && dcache_req_id_is_lsu(slot.req_id) &&
                !bank_retry_cur.stores[i].valid) {
                bank_retry_issue_wires.stores[i].valid = true;
                bank_retry_issue_wires.stores[i].addr = slot.addr;
                bank_retry_issue_wires.stores[i].strb = slot.strb;
                bank_retry_issue_wires.stores[i].req_id = slot.req_id;

                continue;
            }
            resp.valid  = true;
            resp.bank_conflict = slot.bank_conflict;
            resp.replay = ReplayType::CONFLICT; // This store has been replayed due to MSHR full or conflict, so replay again to give chance for the MSHR state to be updated and avoid starvation when there are multiple back-to-back misses.
            resp.req_id = slot.req_id;

            continue;
        }        
        if(cache_line_match(slot.addr,in.mshr2dcache->fill_req.addr)&&in.mshr2dcache->fill_req.valid){ // merge the store into the incoming fill so the store can complete without replaying
            const AddrFields fill_req_f = decode(in.mshr2dcache->fill_req.addr);
            const bool fill_req_already_in_s2 =
                s1s2_cur.fill_write.valid &&
                s1s2_cur.fill_write.set_idx == fill_req_f.set_idx &&
                s1s2_cur.fill_write.tag == fill_req_f.tag;

            if (fill_req_already_in_s2 && out.fill_write->valid) {
                out.fill_write->dirty = true;
            } else {
                fill_req_merge_wires[i].valid = true;
            }

            resp.valid  = true;
            resp.replay = ReplayType::HIT;
            resp.req_id = slot.req_id;
            continue;
        }
        wire<DCACHE_WAY_BITS> hit_way = DCACHE_WAYS_NUM;
        for (int w = 0; w < DCACHE_WAYS_NUM; w++) {
            if (in.dcachelinereadresp[iidx]->valid[w] && in.dcachelinereadresp[iidx]->tag[w] == tag_expected) {
                hit_way = w;
                break;
            }
        }

        if (hit_way < DCACHE_WAYS_NUM) {
            if(s1s2_cur.fill_write.set_idx == f.set_idx && s1s2_cur.fill_write.valid&&out.fill_write->way_idx == hit_way){ // hit the line being filled in the same cycle, treat it as a hit and update the pending fill write to avoid a deadlock when MSHR is full
                resp.valid  = true;
                resp.replay = ReplayType::CONFLICT;
                resp.req_id = slot.req_id;

            }
            else{
                resp.valid  = true;
                resp.replay = ReplayType::HIT;
                resp.req_id = slot.req_id;
                *out.pendingwrite[i] = {
                    true,
                    f.set_idx,
                    hit_way,
                    f.word_off
                };
                *out.lru_updates[LSU_LDU_COUNT + i] = {true, f.set_idx, hit_way};
            }
        }
        else {
            if(in.wb2dcache->merge_resp[i].valid){
                resp.valid = true;
                resp.replay = ReplayType::HIT;
                resp.req_id = slot.req_id;
            }
            else if(in.wb2dcache->merge_resp[i].busy){
                resp.valid = true;
                resp.replay = ReplayType::CONFLICT; // The store can proceed but the merge logic is busy, so replay later to give chance for the merge logic to catch up and avoid long latency on the following stores that hit the same line.
                resp.req_id = slot.req_id;

            }
            else if(in.mshr2dcache->find_resp[iidx].valid){
                out.dcache2mshr->mshr_req[iidx].valid = true;
                out.dcache2mshr->mshr_req[iidx].addr = slot.addr;
                out.dcache2mshr->mshr_req[iidx].is_store = true;
                out.dcache2mshr->mshr_req[iidx].lsu_origin =
                    l1d_perf_is_lsu_req(slot.req_id);
                resp.valid = true;
                resp.replay = ReplayType::HIT;
                resp.req_id = slot.req_id;
            }
            else{
                // Miss with no existing MSHR entry: allocate a new MSHR and replay later when it is ready.
                if(mshr_free_entries == 0){ // No free MSHR entry, or the only free entry is being allocated by the other store port in the same cycle, so can't accept this new miss
                    resp.valid = true;
                    resp.replay = ReplayType::MSHR_FULL; // MSHR full, replay later
                    resp.req_id = slot.req_id;

                }
                else {
                    out.dcache2mshr->mshr_req[iidx].valid = true;
                    out.dcache2mshr->mshr_req[iidx].addr = slot.addr;
                    out.dcache2mshr->mshr_req[iidx].is_store = true;
                    out.dcache2mshr->mshr_req[iidx].lsu_origin =
                        l1d_perf_is_lsu_req(slot.req_id);

                    resp.valid = true;
                    resp.replay = ReplayType::HIT;
                    resp.req_id = slot.req_id;
                    mshr_free_entries--;
                }
            }
        }
    }

    // ── Prefetch probe (lowest priority: after every demand request has taken its MSHR) ──────────
    if (pf_slot >= 0) {
        const S1S2Reg::LoadSlot &slot = s1s2_cur.loads[pf_slot];
        const AddrFields f = decode(slot.addr);
        const bool pf_llc_only = slot.pf_llc;
        const DcacheLineReadResp &line_resp = *in.dcachelinereadresp[pf_slot];
        int hit_way = -1;
        for (int w = 0; w < DCACHE_WAYS_NUM; w++)
            if (line_resp.valid[w] && line_resp.tag[w] == f.tag) { hit_way = w; break; }
        const bool fill_same =
            (s1s2_cur.fill_write.valid && s1s2_cur.fill_write.set_idx == f.set_idx && s1s2_cur.fill_write.tag == f.tag) ||
            (in.mshr2dcache->fill_req.valid && cache_line_match(slot.addr, in.mshr2dcache->fill_req.addr));
        g_pf_stats.probe++;
        if (in.wb2dcache->bypass_resp[pf_slot].valid) g_pf_stats.probe_wb++;
        else if (hit_way >= 0 || fill_same) g_pf_stats.probe_hit++;
        else if (in.mshr2dcache->find_resp[pf_slot].valid) g_pf_stats.probe_inflight++;
        else if (mshr_free_entries <= PF_MSHR_RESERVE) g_pf_stats.probe_mshr_full++;
        else {
            out.dcache2mshr->mshr_req[pf_slot].valid = true;
            out.dcache2mshr->mshr_req[pf_slot].addr = slot.addr;
            out.dcache2mshr->mshr_req[pf_slot].is_store = false;
            out.dcache2mshr->mshr_req[pf_slot].lsu_origin = false;
            out.dcache2mshr->mshr_req[pf_slot].is_prefetch = true;
            out.dcache2mshr->mshr_req[pf_slot].no_fill = pf_llc_only;
            mshr_free_entries--;
        }
    }

    out.dcache2lsu->mshr_fill = out.fill_write->valid || in.mshr2dcache->release_event; // Inform LSU about MSHR fill in the same cycle, so that it can prioritize the waiting load/store to consume the fill data and free up the MSHR entry as soon as possible, improving performance when there are back-to-back misses.
    out.dcache2lsu->mshr_fill_addr =
        out.fill_write->valid
            ? get_addr(out.fill_write->set_idx, out.fill_write->tag, 0)
            : 0;
}

void RealDcache::comb() {

    stage2_comb();

    stage1_comb();
}

// ─────────────────────────────────────────────────────────────────────────────
// seq — advance all state on the simulated clock edge.
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::seq() {
    // 1. Advance pipeline register.
    s1s2_cur = s1s2_nxt;
    bank_retry_cur = bank_retry_nxt;
    pf.seq();
}

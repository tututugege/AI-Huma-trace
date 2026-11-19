#include <cstdlib>
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

static bool l1d_perf_is_lsu_req(uint32_t req_id) {
    return dcache_req_id_is_lsu(req_id);
}
} // namespace

void RealDcache::init() {
    arrays.init();
    resp_raw_ = {};
    for (auto &meta : resp_meta_cur_) meta = {};
    for (auto &meta : resp_meta_nxt_) meta = {};
    for (auto &r : line_read_req_) r = {};
    for (auto &r : line_read_resp_) r = {};
    for (auto &entry : lru_updates_) entry = {};
    for (auto &entry : pending_writes_) entry = {};
    fill_write_ = {};
    fill_in_ = {};
    fill_out_ = {};
    fill_store_dirty_ = false;
    s1_q_ = {};
    q_.s1s2 = {};
    d.s1s2 = {};
    q_.bank_retry = {};
    d.bank_retry = {};
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

void RealDcache::stage1() {
    const Regs &q = q_;
    d.bank_retry = q.bank_retry;

    auto line_id = [](uint32_t addr) -> uint32_t {
        return addr >> DCACHE_OFFSET_BITS;
    };
    auto bank_index = [](const AddrFields &f) -> uint32_t {
        return static_cast<uint32_t>(f.bank);
    };

    AddrFields fill_f = decode(in.mshr2dcache->fill_req.addr);
    const uint32_t fill_line_id = line_id(in.mshr2dcache->fill_req.addr);
    
    bool same_fill_already_in_s2 =
    q.s1s2.fill_write.valid &&
    in.mshr2dcache->fill_req.valid &&
    q.s1s2.fill_write.set_idx == fill_f.set_idx &&
    q.s1s2.fill_write.tag == fill_f.tag;

    const bool hold_fill_in_s2 =
        same_fill_already_in_s2 && !fill_write_.valid;
    const bool fillout_needs_snapshot =
        in.mshr2dcache->fill_req.valid &&
        !(same_fill_already_in_s2 && fill_write_.valid);

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
        d.s1s2.fill_write = q.s1s2.fill_write;
    } else {
        d.s1s2.fill_write = {};
        if (fillout_granted && !same_fill_already_in_s2) {
            d.s1s2.fill_write.valid = true;
            d.s1s2.fill_write.id = in.mshr2dcache->fill_req.id;
            d.s1s2.fill_write.dirty = in.mshr2dcache->fill_req.dirty || fill_store_dirty_;
            d.s1s2.fill_write.lsu_origin = in.mshr2dcache->fill_req.lsu_origin;
            d.s1s2.fill_write.is_prefetch = in.mshr2dcache->fill_req.is_prefetch;
            d.s1s2.fill_write.set_idx = fill_f.set_idx;
            d.s1s2.fill_write.tag = fill_f.tag;
        }
    }

    if (d.s1s2.fill_write.valid) {
        for (int i = 0; i < LSU_STA_COUNT; i++) {
            if (!fill_req_merge_wires[i].valid) continue;
            d.s1s2.fill_write.dirty = true;
        }
    }


    for (auto &r : out.dcache2wb->bypass_req) r.valid = false;
    for (auto &r : out.dcache2wb->merge_req) r.valid = false;
    for (auto &r : out.dcache2mshr->find_req) r.valid = false;
    fill_out_.valid = false;

    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        line_read_req_[i].valid = false;
    }


    // Bank arbitration order: a port that issues a hidden retry this cycle is granted first, so a retry cannot be starved by an
    // endless stream on a lower-numbered port.
    int ld_order[LSU_LDU_COUNT];
    {
        int n = 0;
        for (int i = 0; i < LSU_LDU_COUNT; i++) {
            const bool lv = in.lsu2dcache->req_ports.load_ports[i].valid;
            if (!lv && !bank_retry_load_blocked[i] && (q.bank_retry.loads[i].valid || bank_retry_issue_wires.loads[i].valid)) ld_order[n++] = i;
        }
        for (int i = 0; i < LSU_LDU_COUNT; i++) {
            bool in_first = false;
            for (int k = 0; k < n; k++) in_first |= (ld_order[k] == i);
            if (!in_first) ld_order[n++] = i;
        }
    }
    for (int oi = 0; oi < LSU_LDU_COUNT; oi++) {
        const int i = ld_order[oi];
        const LoadReq &lsu_req = in.lsu2dcache->req_ports.load_ports[i];
        const BankRetryLoad &pending_retry = q.bank_retry.loads[i];
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
        const uint32_t req_pc =
            lsu_req.valid ? uint32_t(lsu_req.pc)
                          : (use_pending_retry ? uint32_t(pending_retry.pc)
                                               : uint32_t(new_retry.pc));
        const uint64_t req_va =
            lsu_req.valid ? uint64_t(lsu_req.vaddr)
                          : (use_pending_retry ? uint64_t(pending_retry.va)
                                               : uint64_t(new_retry.va));
        S1S2Reg::LoadSlot &slot = d.s1s2.loads[i];
        slot.pf = false;
        if (use_pending_retry) {
            d.bank_retry.loads[i] = {};
        }
        if (new_retry.valid && !use_new_retry) {
            Assert(!d.bank_retry.loads[i].valid &&
                   "DCache load bank retry latch overflow");
            d.bank_retry.loads[i] = new_retry;
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
            slot.valid    = true;
            slot.addr     = req_addr;
            slot.req_id   = req_id;
            slot.pc       = req_pc;
            slot.va       = req_va;
            slot.replayed  = !bank_granted;
            slot.bank_conflict = !bank_granted;
            slot.perf_replay = use_hidden_retry ? true : lsu_req.replay;
            if (!bank_granted) {
                continue;
            }

            line_read_req_[i].valid = true;
            line_read_req_[i].set_idx = f.set_idx;
            line_read_req_[i].word_off = f.word_off;
            out.dcache2wb->bypass_req[i].valid = true;
            out.dcache2wb->bypass_req[i].addr = req_addr;

            out.dcache2mshr->find_req[i] = MSHRFindReq{.valid = true, .is_prefetch = false, .set_idx = f.set_idx, .tag = f.tag};
        }        
    }

    int st_order[LSU_STA_COUNT];
    {
        int n = 0;
        for (int i = 0; i < LSU_STA_COUNT; i++) {
            if (!in.lsu2dcache->req_ports.store_ports[i].valid && !bank_retry_store_blocked[i] &&
                (q.bank_retry.stores[i].valid || bank_retry_issue_wires.stores[i].valid)) st_order[n++] = i;
        }
        for (int i = 0; i < LSU_STA_COUNT; i++) {
            bool in_first = false;
            for (int k = 0; k < n; k++) in_first |= (st_order[k] == i);
            if (!in_first) st_order[n++] = i;
        }
    }
    for (int oi = 0; oi < LSU_STA_COUNT; oi++) {
        const int i = st_order[oi];
        const StoreReq &lsu_req = in.lsu2dcache->req_ports.store_ports[i];
        const BankRetryStore &pending_retry = q.bank_retry.stores[i];
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
        S1S2Reg::StoreSlot &slot = d.s1s2.stores[i];
        int idx       = LSU_LDU_COUNT + i;
        if (use_pending_retry) {
            d.bank_retry.stores[i] = {};
        }
        if (new_retry.valid && !use_new_retry) {
            Assert(!d.bank_retry.stores[i].valid &&
                   "DCache store bank retry latch overflow");
            d.bank_retry.stores[i] = new_retry;
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
            slot.valid    = true;
            slot.addr     = req_addr;
            slot.strb     = req_strb;
            slot.req_id   = req_id;
            slot.replayed = !bank_granted;
            slot.bank_conflict = !bank_granted;
            slot.perf_replay = use_hidden_retry ? true : lsu_req.replay;
            if (!bank_granted) {
                continue;
            }

            line_read_req_[idx].valid = true;
            line_read_req_[idx].set_idx = f.set_idx;
            line_read_req_[idx].word_off = f.word_off;
            out.dcache2wb->merge_req[i].valid = true;
            out.dcache2wb->merge_req[i].addr = req_addr;

            out.dcache2mshr->find_req[idx] = MSHRFindReq{.valid = true, .is_prefetch = false, .set_idx = f.set_idx, .tag = f.tag};
        }
    }

    // Prefetch tag probe: lowest priority, rides an idle load port (it competes for the bank like any load).
    if (pf.enabled() && in.lsu2dcache != nullptr && pf.head_valid()) {
        for (int i = 0; i < LSU_LDU_COUNT; i++) {
            S1S2Reg::LoadSlot &slot = d.s1s2.loads[i];
            if (slot.valid) continue;                      // port busy with a demand request or a retry
            const uint32_t pf_addr = pf.head_addr();
            const AddrFields f = decode(pf_addr);
            if (!try_grant_bank(f, line_id(pf_addr))) break;
            slot.valid = true;
            slot.pf = true;
            slot.pf_llc = pf.head_llc_only();
            slot.addr = pf_addr;
            slot.req_id = 1u << 31;                        // not an LSU request
            slot.replayed = false;
            slot.bank_conflict = false;
            slot.perf_replay = true;
            line_read_req_[i].valid = true;
            line_read_req_[i].set_idx = f.set_idx;
            line_read_req_[i].word_off = f.word_off;
            out.dcache2wb->bypass_req[i].valid = true;     // a line still in the write buffer must not be re-read
            out.dcache2wb->bypass_req[i].addr = pf_addr;
            out.dcache2mshr->find_req[i] = MSHRFindReq{.valid = true, .is_prefetch = true, .set_idx = f.set_idx, .tag = f.tag};
            pf.head_taken = true;
            break;
        }
    }

    fill_out_.valid = fillout_granted;
    fill_out_.set_idx = fillout_granted ? fill_f.set_idx : 0;




    d.s1s2.icache_req = in.lsu2dcache->icache_req;
    
}

void RealDcache::stage2() {
    const Regs &q = q_;

    for (auto &r : resp_raw_.resp_ports.load_resps) r.valid = false;
    for (auto &r : resp_raw_.resp_ports.store_resps) r.valid = false;
    for (auto &r : out.dcache2mshr->mshr_req) r.valid = false;
    out.dcache2wb->dirty_info.valid = false;
    out.dcache2mshr->fill_resp.done = false;
    
    fill_write_.valid = false;
    for(int i=0;i<LSU_LDU_COUNT + LSU_STA_COUNT;i++){
        lru_updates_[i].valid = false;
        pending_writes_[i].valid = false;
    }
    for (auto &entry : fill_req_merge_wires) {
        entry = {};
    }
    bank_retry_issue_wires = {};

    if(q.s1s2.fill_write.valid){
        int select_way = choose_plru_tree_victim(fill_in_.plru_tree_state, fill_in_.valid_snap);
        if(fill_in_.valid_snap[select_way] && fill_in_.dirty_snap[select_way]){
            if(in.wb2dcache->free>0){
                out.dcache2wb->dirty_info.valid = true;
                out.dcache2wb->dirty_info.lsu_origin =
                    q.s1s2.fill_write.lsu_origin;
                out.dcache2wb->dirty_info.addr = get_addr(q.s1s2.fill_write.set_idx, fill_in_.tag_snap[select_way], 0);

                fill_write_.valid = true;
                fill_write_.dirty = q.s1s2.fill_write.dirty;
                fill_write_.lsu_origin = q.s1s2.fill_write.lsu_origin;
                fill_write_.is_prefetch = q.s1s2.fill_write.is_prefetch;
                fill_write_.set_idx = q.s1s2.fill_write.set_idx;
                fill_write_.tag = q.s1s2.fill_write.tag;
                fill_write_.way_idx = select_way;
                pf.fill_in = PfFill{true, get_addr(q.s1s2.fill_write.set_idx, q.s1s2.fill_write.tag, 0), q.s1s2.fill_write.is_prefetch};
                out.dcache2mshr->fill_resp.done = true; // Mark the MSHR fill as done to unblock the waiting load/store, even though the line is not yet filled into the cache. This allows the writeback to be issued in parallel with the fill, improving performance when the evicted line is dirty.
                out.dcache2mshr->fill_resp.id = q.s1s2.fill_write.id;
            }
            else{
                fill_write_.valid = false; // Can't accept new fill, so stall the MSHR fill response and the waiting load/store. This may cause deadlock when MSHR and WB are both full, but it's a rare case and can be resolved by replaying the load/store.
                out.dcache2wb->dirty_info.valid = false; // Can't accept new writeback, so treat the line as clean and let it be overwritten without writeback. This may cause data loss but prevents deadlock when MSHR and WB are both full.
            }
        }
        else{
            fill_write_.valid = true;
            fill_write_.dirty = q.s1s2.fill_write.dirty;
            fill_write_.lsu_origin = q.s1s2.fill_write.lsu_origin;
            fill_write_.is_prefetch = q.s1s2.fill_write.is_prefetch;
            fill_write_.set_idx = q.s1s2.fill_write.set_idx;
            fill_write_.tag = q.s1s2.fill_write.tag;
            fill_write_.way_idx = select_way;
            pf.fill_in = PfFill{true, get_addr(q.s1s2.fill_write.set_idx, q.s1s2.fill_write.tag, 0), q.s1s2.fill_write.is_prefetch};
            out.dcache2mshr->fill_resp.done = true; // Mark the MSHR fill as done to unblock the waiting load/store, even though the line is not yet filled into the cache. This allows the fill to complete and the line to be allocated in cache without waiting for an unnecessary writeback to drain when the evicted line is clean.
            out.dcache2mshr->fill_resp.id = q.s1s2.fill_write.id;
        }
    }

    int mshr_free_entries = in.mshr2dcache->free;
    int pf_slot = -1;
    // ── Load ports ────────────────────────────────────────────────────────────
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const S1S2Reg::LoadSlot &slot = q.s1s2.loads[i];
        LoadResp &resp = resp_raw_.resp_ports.load_resps[i];
        DcacheLineReadResp &line_resp = line_read_resp_[i];
        resp.valid = false; 
        if (!slot.valid) continue;
        if (slot.pf) { pf_slot = i; continue; }            // handled after the demand requests

        AddrFields f          = decode(slot.addr);
        uint32_t tag_expected = f.tag;

        if(slot.replayed){
            if (slot.bank_conflict && dcache_req_id_is_lsu(slot.req_id) &&
                !q.bank_retry.loads[i].valid) {
                bank_retry_issue_wires.loads[i].valid = true;
                bank_retry_issue_wires.loads[i].addr = slot.addr;
                bank_retry_issue_wires.loads[i].req_id = slot.req_id;
                bank_retry_issue_wires.loads[i].pc = slot.pc;
                bank_retry_issue_wires.loads[i].va = slot.va;

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
                if (arrays.pf_array[f.set_idx][hit_way]) {        // first use of a prefetched line
                    arrays.pf_array[f.set_idx][hit_way] = false;
                    g_pf_stats.useful++;
                    pf.acc[i] = PfAccess{true, slot.addr, false, true, slot.pc, slot.va};
                }
            }
            // ── Cache Hit ────────────────────────────────────────────────────
            resp.valid  = true;
            resp.replay = ReplayType::HIT;
            resp.req_id = slot.req_id;
            resp.retry = false;
            lru_updates_[i].valid = true;
            lru_updates_[i].set_idx = f.set_idx;
            lru_updates_[i].way = hit_way;
        }
        else {
            if(q.s1s2.icache_req==i){
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
                pf.acc[i] = PfAccess{true, slot.addr, true, false, slot.pc, slot.va};
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
                    out.dcache2mshr->mshr_req[i] = MSHRReq{.valid = true, .addr = slot.addr, .lsu_origin = l1d_perf_is_lsu_req(slot.req_id)};

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
        const S1S2Reg::StoreSlot &slot = q.s1s2.stores[i];
        StoreResp &resp = resp_raw_.resp_ports.store_resps[i];
        const int iidx = LSU_LDU_COUNT + i;

        if (!slot.valid) continue;

        AddrFields f          = decode(slot.addr);
        uint32_t tag_expected = f.tag;


        if(slot.replayed){
            if (slot.bank_conflict && dcache_req_id_is_lsu(slot.req_id) &&
                !q.bank_retry.stores[i].valid) {
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
                q.s1s2.fill_write.valid &&
                q.s1s2.fill_write.set_idx == fill_req_f.set_idx &&
                q.s1s2.fill_write.tag == fill_req_f.tag;

            if (fill_req_already_in_s2 && fill_write_.valid) {
                fill_write_.dirty = true;
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
            if (line_read_resp_[iidx].valid[w] && line_read_resp_[iidx].tag[w] == tag_expected) {
                hit_way = w;
                break;
            }
        }

        if (hit_way < DCACHE_WAYS_NUM) {
            if(q.s1s2.fill_write.set_idx == f.set_idx && q.s1s2.fill_write.valid&&fill_write_.way_idx == hit_way){ // hit the line being filled in the same cycle, treat it as a hit and update the pending fill write to avoid a deadlock when MSHR is full
                resp.valid  = true;
                resp.replay = ReplayType::CONFLICT;
                resp.req_id = slot.req_id;

            }
            else{
                resp.valid  = true;
                resp.replay = ReplayType::HIT;
                resp.req_id = slot.req_id;
                pending_writes_[i] = {
                    true,
                    f.set_idx,
                    hit_way,
                    f.word_off
                };
                lru_updates_[LSU_LDU_COUNT + i] = {true, f.set_idx, hit_way};
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
                out.dcache2mshr->mshr_req[iidx] = MSHRReq{.valid = true, .addr = slot.addr, .is_store = true, .lsu_origin = l1d_perf_is_lsu_req(slot.req_id)};
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
                    out.dcache2mshr->mshr_req[iidx] = MSHRReq{.valid = true, .addr = slot.addr, .is_store = true, .lsu_origin = l1d_perf_is_lsu_req(slot.req_id)};

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
        const S1S2Reg::LoadSlot &slot = q.s1s2.loads[pf_slot];
        const AddrFields f = decode(slot.addr);
        const bool pf_llc_only = slot.pf_llc;
        const DcacheLineReadResp &line_resp = line_read_resp_[pf_slot];
        int hit_way = -1;
        for (int w = 0; w < DCACHE_WAYS_NUM; w++)
            if (line_resp.valid[w] && line_resp.tag[w] == f.tag) { hit_way = w; break; }
        const bool fill_same =
            (q.s1s2.fill_write.valid && q.s1s2.fill_write.set_idx == f.set_idx && q.s1s2.fill_write.tag == f.tag) ||
            (in.mshr2dcache->fill_req.valid && cache_line_match(slot.addr, in.mshr2dcache->fill_req.addr));
        // EXP-048 diagnostics: the outcome of this cycle's probe (counted once in seq)
        pf_probe_outcome_ = (in.wb2dcache->bypass_resp[pf_slot].valid || hit_way >= 0 || fill_same) ? 2
                            : in.mshr2dcache->find_resp[pf_slot].valid ? 3 : mshr_free_entries <= PF_MSHR_RESERVE ? 4 : 1;
        if (in.wb2dcache->bypass_resp[pf_slot].valid || hit_way >= 0 || fill_same || in.mshr2dcache->find_resp[pf_slot].valid ||
            mshr_free_entries <= PF_MSHR_RESERVE) {
            // the line is in the write buffer / the cache / an MSHR, or the MSHRs are reserved for demand misses: the prefetch is dropped
        } else {
            out.dcache2mshr->mshr_req[pf_slot] = MSHRReq{.valid = true, .addr = slot.addr, .is_prefetch = true, .no_fill = pf_llc_only};
            mshr_free_entries--;
        }
    }

    resp_raw_.mshr_fill = fill_write_.valid || in.mshr2dcache->release_event; // Inform LSU about MSHR fill in the same cycle, so that it can prioritize the waiting load/store to consume the fill data and free up the MSHR entry as soon as possible, improving performance when there are back-to-back misses.
    resp_raw_.mshr_fill_addr =
        fill_write_.valid
            ? get_addr(fill_write_.set_idx, fill_write_.tag, 0)
            : 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// seq — advance all state on the simulated clock edge.
// ─────────────────────────────────────────────────────────────────────────────
void RealDcache::seq() {
    // 1. Advance pipeline register.
    q_ = d;
    switch (pf_probe_outcome_) {   // EXP-048 diagnostics
    case 1: g_pf_stats.pr_alloc++; break;
    case 2: g_pf_stats.pr_present++; break;
    case 3: g_pf_stats.pr_inflight++; break;
    case 4: g_pf_stats.pr_nomshr++; break;
    }
    pf_probe_outcome_ = 0;
    pf.seq();
    // 2. the stage-1 requests (find / bypass / merge, line reads, fill read) and the response metadata of this cycle
    std::memcpy(s1_q_.find_req, out.dcache2mshr->find_req, sizeof(s1_q_.find_req));
    std::memcpy(s1_q_.bypass_req, out.dcache2wb->bypass_req, sizeof(s1_q_.bypass_req));
    std::memcpy(s1_q_.merge_req, out.dcache2wb->merge_req, sizeof(s1_q_.merge_req));
    std::memcpy(s1_q_.line_read_req, line_read_req_, sizeof(s1_q_.line_read_req));
    s1_q_.fill_out = fill_out_;
    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        resp_meta_cur_[i] = resp_meta_nxt_[i];
    }
}

static bool replay_waits_for_mshr_fill(ReplayType replay) {
    return replay == ReplayType::MSHR_HIT || replay == ReplayType::MSHR_FULL;
}

// Stage 2 of the request that stage 1 issued last cycle: the tag / fill array read, the stage-2 logic, the array
// writes, the response to the route block.
void RealDcache::comb_stage2() {
    Dcache_Read(arrays, s1_q_.line_read_req, line_read_resp_, s1_q_.fill_out, fill_in_);
    stage2();
    Dcache_Write(arrays, pending_writes_, lru_updates_, fill_write_);
    drive_response();
}

// This cycle's response as the route block sees it (and through it the LSU, the I-cache probe, the PTW): the valid
// stage-2 responses, a response that waits for an MSHR fill arriving now turned into a hit (matched through the
// stage-1 metadata register), and the hidden bank-retry busy bits.
void RealDcache::drive_response() {
    DcacheLsuIO &route = *out.dcache2route;
    route = {};
    fill_store_dirty_ = false;
    for (int i = 0; i < LSU_LDU_COUNT; i++)
        if (resp_raw_.resp_ports.load_resps[i].valid) route.resp_ports.load_resps[i] = resp_raw_.resp_ports.load_resps[i];
    for (int i = 0; i < LSU_STA_COUNT; i++)
        if (resp_raw_.resp_ports.store_resps[i].valid) route.resp_ports.store_resps[i] = resp_raw_.resp_ports.store_resps[i];
    if (resp_raw_.mshr_fill) {
        route.mshr_fill = true;
        route.mshr_fill_addr = resp_raw_.mshr_fill_addr;
    }

    const MSHRFillReq &fill = in.mshr2dcache->fill_req;
    if (fill.valid) {
        for (int i = 0; i < LSU_LDU_COUNT; i++) {
            const LoadResp &resp = route.resp_ports.load_resps[i];
            const RespMeta &meta = resp_meta_cur_[i];
            if (!resp.valid || !meta.valid || meta.is_store || !replay_waits_for_mshr_fill(resp.replay) ||
                !cache_line_match(meta.addr, fill.addr))
                continue;
            route.resp_ports.load_resps[i].replay = ReplayType::HIT;
        }
        for (int i = 0; i < LSU_STA_COUNT; i++) {
            const StoreResp &resp = route.resp_ports.store_resps[i];
            const RespMeta &meta = resp_meta_cur_[LSU_LDU_COUNT + i];
            if (!resp.valid || !meta.valid || !meta.is_store || !replay_waits_for_mshr_fill(resp.replay) ||
                !cache_line_match(meta.addr, fill.addr))
                continue;
            fill_store_dirty_ = true;
            route.resp_ports.store_resps[i].replay = ReplayType::HIT;
        }
    }

    for (int i = 0; i < LSU_LDU_COUNT; i++) route.load_retry_busy[i] = load_retry_pending(i);
    for (int i = 0; i < LSU_STA_COUNT; i++) route.store_retry_busy[i] = store_retry_pending(i);
}

// Stage 1 of this cycle's requests (a port whose response is still held is blocked), and its metadata for the
// response next cycle.
void RealDcache::comb_stage1() {
    bool load_resp_blocked[LSU_LDU_COUNT] = {};
    bool store_resp_blocked[LSU_STA_COUNT] = {};
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        load_resp_blocked[i] = out.dcache2route->resp_ports.load_resps[i].valid;
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        store_resp_blocked[i] = out.dcache2route->resp_ports.store_resps[i].valid;
    }
    set_hidden_bank_retry_blocked(load_resp_blocked, store_resp_blocked);

    stage1();
    record_metadata();
}

void RealDcache::record_metadata() {
    for (auto &meta : resp_meta_nxt_) {
        meta = {};
    }
    const S1S2Reg &stage1 = d.s1s2;
    for (int i = 0; i < LSU_LDU_COUNT; i++) {
        const auto &load = stage1.loads[i];
        if (!load.valid) {
            continue;
        }
        resp_meta_nxt_[i].valid = true;
        resp_meta_nxt_[i].is_store = false;
        resp_meta_nxt_[i].addr = load.addr;
    }
    for (int i = 0; i < LSU_STA_COUNT; i++) {
        const auto &store = stage1.stores[i];
        if (!store.valid) {
            continue;
        }
        const int idx = LSU_LDU_COUNT + i;
        resp_meta_nxt_[idx].valid = true;
        resp_meta_nxt_[idx].is_store = true;
        resp_meta_nxt_[idx].addr = store.addr;
    }
}

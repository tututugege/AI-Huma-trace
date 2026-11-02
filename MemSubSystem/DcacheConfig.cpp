#include "PhysMemory.h"
#include "DcacheConfig.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

uint32_t tag_array[DCACHE_SETS_NUM][DCACHE_WAYS_NUM] = {};
// No L1D line-data storage (P3): load data is a fact of the trace; tags/valid/dirty/PLRU are kept.
bool valid_array[DCACHE_SETS_NUM][DCACHE_WAYS_NUM] = {};
bool dirty_array[DCACHE_SETS_NUM][DCACHE_WAYS_NUM] = {};
bool plru_tree_state[DCACHE_SETS_NUM][DCACHE_PLRU_TREE_BITS] = {};

namespace {
inline bool invalid_set_or_way(uint32_t set_idx, uint32_t way) {
    return set_idx >= DCACHE_SETS_NUM || way >= DCACHE_WAYS_NUM;
}

struct DataBankAccessRecord {
    bool valid = false;
    const char *op = "";
    const char *source = "";
    uint32_t set_idx = 0;
};

DataBankAccessRecord data_bank_access[DCACHE_BANK_NUM] = {};

inline uint32_t data_bank_from_set(uint32_t set_idx) {
    return set_idx & (DCACHE_BANK_NUM - 1);
}

inline void begin_data_bank_cycle() {
    for (auto &record : data_bank_access) {
        record = {};
    }
}

inline void check_data_bank_access(uint32_t set_idx, const char *op, const char *source) {
    const uint32_t bank = data_bank_from_set(set_idx);
    auto &record = data_bank_access[bank];
    if (record.valid) {
        std::fprintf(stderr,
                     "[DCacheConfig] data_array bank conflict: bank=%u new=%s/%s set=%u previous=%s/%s set=%u\n",
                     bank,
                     op,
                     source,
                     set_idx,
                     record.op,
                     record.source,
                     record.set_idx);
        std::fflush(stderr);
        std::abort();
    }

    record.valid = true;
    record.op = op;
    record.source = source;
    record.set_idx = set_idx;
}

inline void plru_touch_way(uint32_t set_idx, uint32_t way) {
    if (set_idx >= DCACHE_SETS_NUM || way >= DCACHE_WAYS_NUM) {
        return;
    }

    uint32_t node = 0;
    uint32_t left_way = 0;
    uint32_t span = DCACHE_WAYS_NUM;
    while (span > 1) {
        const uint32_t half = span / 2;
        const bool went_right = way >= left_way + half;
        plru_tree_state[set_idx][node] = went_right ? 0 : 1;
        node = node * 2 + (went_right ? 2 : 1);
        if (went_right) {
            left_way += half;
        }
        span = half;
    }
}
} // namespace

void init_dcache()
{
    std::memset(tag_array, 0, sizeof(tag_array));
    std::memset(valid_array, 0, sizeof(valid_array));
    std::memset(dirty_array, 0, sizeof(dirty_array));
    std::memset(plru_tree_state, 0, sizeof(plru_tree_state));
    begin_data_bank_cycle();
}

AddrFields decode(uint32_t addr)
{
    AddrFields f;
    f.tag = addr >> (DCACHE_SET_BITS + DCACHE_OFFSET_BITS);
    f.set_idx = (addr >> DCACHE_OFFSET_BITS) & (DCACHE_SETS_NUM - 1);
    f.word_off = (addr >> 2) & (DCACHE_WORD_NUM - 1);
    f.bank = f.set_idx & (DCACHE_BANK_NUM - 1);
    return f;
}

uint32_t get_addr(uint32_t set_idx, uint32_t tag, uint32_t word_off)
{
    return (tag << (DCACHE_SET_BITS + DCACHE_OFFSET_BITS)) | (set_idx << DCACHE_OFFSET_BITS) | (word_off << 2);
}

uint32_t choose_plru_tree_victim(const bool plru_tree[DCACHE_PLRU_TREE_BITS], const bool valid[DCACHE_WAYS_NUM])
{
    for (int w = 0; w < DCACHE_WAYS_NUM; w++) {
        if (!valid[w]) {
            return w;
        }
    }

    uint32_t node = 0;
    uint32_t victim_way = 0;
    uint32_t span = DCACHE_WAYS_NUM;
    while (span > 1) {
        const uint32_t half = span / 2;
        const bool go_right = (plru_tree[node] & 1) != 0;
        node = node * 2 + (go_right ? 2 : 1);
        if (go_right) {
            victim_way += half;
        }
        span = half;
    }
    return victim_way;
}



void write_dcache_line(uint32_t set_idx, uint32_t way, uint32_t tag)
{
    check_data_bank_access(set_idx, "write", "fill_line");

    uint32_t target_way = way;
    for (uint32_t w = 0; w < DCACHE_WAYS_NUM; w++)
    {
        if (valid_array[set_idx][w] && tag_array[set_idx][w] == tag)
        {
            target_way = w;
            break;
        }
    }

    valid_array[set_idx][target_way] = true;
    dirty_array[set_idx][target_way] = false;
    tag_array[set_idx][target_way] = tag;
    plru_tree_touch(set_idx, target_way);
}

bool cache_line_match(uint32_t addr1, uint32_t addr2){
    return (addr1 >> DCACHE_OFFSET_BITS) == (addr2 >> DCACHE_OFFSET_BITS);
}
void Dcache_Read(const DcacheLineReadReq read_req[LSU_LDU_COUNT+LSU_STA_COUNT],DcacheLineReadResp resp[LSU_LDU_COUNT+LSU_STA_COUNT], const FillOut &fillout,FillIn &fillin)
{
    begin_data_bank_cycle();

    for (int i = 0; i < LSU_LDU_COUNT + LSU_STA_COUNT; i++) {
        const auto &req = read_req[i];
        if (!req.valid) {
            memset(&resp[i], 0, sizeof(resp[i]));
            continue;
        }
        check_data_bank_access(req.set_idx, "read", "read_req");
        memcpy(resp[i].valid, valid_array[req.set_idx], sizeof(valid_array[req.set_idx]));
        memcpy(resp[i].tag, tag_array[req.set_idx], sizeof(tag_array[req.set_idx]));
        memcpy(resp[i].dirty, dirty_array[req.set_idx], sizeof(dirty_array[req.set_idx]));
    }

    if(fillout.valid){
        // check_data_bank_access(fillout.set_idx, "read", "fillout");
        memcpy(fillin.valid_snap,valid_array[fillout.set_idx], sizeof(valid_array[fillout.set_idx]));
        memcpy(fillin.tag_snap,tag_array[fillout.set_idx], sizeof(tag_array[fillout.set_idx]));
        memcpy(fillin.dirty_snap,dirty_array[fillout.set_idx], sizeof(dirty_array[fillout.set_idx]));
        memcpy(fillin.plru_tree_state,plru_tree_state[fillout.set_idx], sizeof(plru_tree_state[fillout.set_idx]));
    }
    else{
        memset(&fillin, 0, sizeof(fillin));
    }
}
void Dcache_Write(const PendingWrite pws[LSU_LDU_COUNT+LSU_STA_COUNT], const LruUpdate lru_updates[LSU_LDU_COUNT+LSU_STA_COUNT], const FILLWrite &fillwrite){
    for(int i=0;i<LSU_LDU_COUNT+LSU_STA_COUNT;i++){
        const PendingWrite &pw = pws[i];
        const LruUpdate &lru_update = lru_updates[i];
        if(pw.valid){
            // check_data_bank_access(pw.set_idx, "write", "pending_write");
            dirty_array[pw.set_idx][pw.way_idx] = true;
        }

        if(lru_update.valid){
            plru_touch_way(lru_update.set_idx, lru_update.way);
        }
    }
    if(fillwrite.valid){
        uint32_t target_way = fillwrite.way_idx;
        for (uint32_t w = 0; w < DCACHE_WAYS_NUM; w++) {
            if (valid_array[fillwrite.set_idx][w] &&
                tag_array[fillwrite.set_idx][w] == fillwrite.tag) {
                target_way = w;
                break;
            }
        }
        write_dcache_line(fillwrite.set_idx, fillwrite.way_idx, fillwrite.tag);
        if (fillwrite.dirty) {
            dirty_array[fillwrite.set_idx][target_way] = true;
        }
    }

}

void plru_tree_touch(uint32_t set_idx, uint32_t way) {
    plru_touch_way(set_idx, way);
}

bool CheckAddr(uint32_t addr1, uint8_t strb1, uint32_t addr2, uint8_t strb2) {
    uint32_t base1 = addr1 & ~0x3u;
    uint32_t base2 = addr2 & ~0x3u;
    uint32_t min_base = base1 < base2 ? base1 : base2;

    uint32_t diff1 = base1 - min_base;
    uint32_t diff2 = base2 - min_base;
    if (diff1 >= 8 || diff2 >= 8) return false;

    uint64_t mask1 = static_cast<uint64_t>(strb1) << diff1;
    uint64_t mask2 = static_cast<uint64_t>(strb2) << diff2;
    return (mask1 & mask2) != 0;
}

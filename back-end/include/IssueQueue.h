#pragma once

#include "config.h"
#include "IO.h"
#include "util.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// 定义每个issue port操作类型的位掩码

// ==========================================
// 1. 配置结构体
// ==========================================
//
struct PortBinding {
  int port_idx;             // 物理端口号 (对应 out.iss2prf 下标)
  uint64_t capability_mask; // 该端口支持的操作码掩码
};

struct IssueQueueConfig {
  int id;                         // IQ 枚举 (IQ_INT, IQ_LD...)
  int size;                       // 深度
  int dispatch_width;             // 每周期最大入队数 (Write Ports)
  uint64_t supported_ops;         // 支持的操作码掩码
  std::vector<PortBinding> ports; // 存储端口配置
};

using IqStoredUop = Uop;

struct IqStoredEntry {
  wire<1> valid;
  IqStoredUop uop;
};

struct IssueQueueIn {
  std::vector<IqStoredEntry> enq_reqs;
  uint32_t wake_pregs[MAX_WAKEUP_PORTS] = {};
  wire<1> wake_valid[MAX_WAKEUP_PORTS] = {};
  wire<1> issue_block = 0;
  wire<1> flush_all = 0;
  wire<1> flush_br = 0;
  wire<BR_MASK_WIDTH> flush_br_mask = 0;
  wire<BR_MASK_WIDTH> clear_mask = 0;
  wire<MAX_UOP_TYPE> port_fu_ready_mask[ISSUE_WIDTH] = {};
};

struct IssueQueueOut {
  wire<IQ_READY_NUM_WIDTH> free_slots = 0;
  IqStoredEntry issue_grants[ISSUE_WIDTH];
};

// ==========================================
// 2. IssueQueue (纯逻辑，不含IO)
// ==========================================
class IssueQueue {
public:
  IssueQueueIn in;
  IssueQueueOut out;

private:
  int size;
  int dispatch_width;
  std::vector<PortBinding> ports;
  int wake_words_per_row;

  std::vector<IqStoredEntry> entry;
  std::vector<IqStoredEntry> entry_1;
  int count, count_1;

  // Wakeup Matrix: [Physical Register] -> Bitmask of IQ slots
  std::vector<uint64_t> wake_matrix_src1;
  std::vector<uint64_t> wake_matrix_src2;
  std::vector<uint64_t> wake_matrix_src1_1;
  std::vector<uint64_t> wake_matrix_src2_1;

  // Dirty tracking: entry/wake_matrix (committed) always equal entry_1/
  // wake_matrix_*_1 (working) at comb_begin (seq copies back). Writes to the
  // working copy are recorded so seq() copies only what changed.
  bool entry_dirty = false;
  bool entry_all_dirty = false;
  std::vector<uint64_t> slot_dirty;
  void mark_slot(int i) {
    slot_dirty[i >> 6] |= 1ull << (i & 63);
    entry_dirty = true;
  }
  bool matrix_all_dirty = false;
  std::vector<uint8_t> row_dirty;
  std::vector<uint32_t> dirty_rows;
  void mark_row(uint32_t preg) {
    if (!row_dirty[preg]) {
      row_dirty[preg] = 1;
      dirty_rows.push_back(preg);
    }
  }
#ifdef IQ_SELFCHECK
  void selfcheck(const char *where) const {
    bool ok = entry.size() == entry_1.size() && count == count_1 &&
              wake_matrix_src1 == wake_matrix_src1_1 &&
              wake_matrix_src2 == wake_matrix_src2_1 &&
              std::memcmp(entry.data(), entry_1.data(),
                          entry.size() * sizeof(IqStoredEntry)) == 0;
    if (!ok) {
      std::fprintf(stderr, "[IQ_SELFCHECK] committed != working at %s\n", where);
      std::abort();
    }
  }
#endif

public:
  IssueQueue(const IssueQueueConfig &cfg)
      : size(cfg.size), dispatch_width(cfg.dispatch_width), // <--- 初始化
        ports(cfg.ports) {
    entry.resize(size);
    entry_1.resize(size);
    slot_dirty.assign((size + 63) / 64, 0);
    count = 0;
    count_1 = 0;
    wake_words_per_row = (size + 63) / 64;

    // Initialize segmented wakeup matrices:
    // [preg][word_idx], where each word tracks 64 IQ slots.
    wake_matrix_src1.resize(PRF_NUM * wake_words_per_row, 0);
    wake_matrix_src2.resize(PRF_NUM * wake_words_per_row, 0);
    wake_matrix_src1_1.resize(PRF_NUM * wake_words_per_row, 0);
    wake_matrix_src2_1.resize(PRF_NUM * wake_words_per_row, 0);
    row_dirty.assign(PRF_NUM, 0);

    in.enq_reqs.resize(dispatch_width);
  }

  void comb_begin() {
#ifdef IQ_SELFCHECK
    selfcheck("comb_begin");
#endif
    count_1 = count;
    out.free_slots = size - count;
    for (auto &grant : out.issue_grants) {
      grant.valid = false; // comb_issue() clears the payload before filling it
    }
    for (int i = 0; i < MAX_WAKEUP_PORTS; i++) {
      in.wake_pregs[i] = 0;
      in.wake_valid[i] = 0;
    }
    in.issue_block = 0;
    in.flush_all = 0;
    in.flush_br = 0;
    in.flush_br_mask = 0;
    in.clear_mask = 0;
    for (auto &req : in.enq_reqs) {
      req.valid = false; // the uop payload is only read when valid
    }
    for (int i = 0; i < ISSUE_WIDTH; i++) {
      in.port_fu_ready_mask[i] = 0;
    }
  }

  void comb_enq() {
    for (const auto &req : in.enq_reqs) {
      if (!req.valid) {
        continue;
      }
      int success = enqueue(req);
      Assert(success && "发射队列溢出！Dispatch 逻辑故障！");
    }
    out.free_slots = size - count_1;
  }

  void comb_wakeup() {
    for (int i = 0; i < MAX_WAKEUP_PORTS; i++) {
      if (!in.wake_valid[i]) {
        continue;
      }
      uint32_t preg = in.wake_pregs[i];
      if (preg >= PRF_NUM) {
        continue;
      }
      size_t row_base = matrix_row_base(preg);
      mark_row(preg);

      for (int w = 0; w < wake_words_per_row; w++) {
        uint64_t mask1 = wake_matrix_src1_1[row_base + w];
        while (mask1) {
          int bit = __builtin_ctzll(mask1);
          int idx = (w << 6) + bit;
          if (idx < size && entry_1[idx].valid && entry_1[idx].uop->src1_en &&
              entry_1[idx].uop.src1_busy && entry_1[idx].uop->src1_preg == preg) {
            entry_1[idx].uop.src1_busy = false;
            mark_slot(idx);
          }
          mask1 &= (mask1 - 1);
        }
        wake_matrix_src1_1[row_base + w] = 0;

        uint64_t mask2 = wake_matrix_src2_1[row_base + w];
        while (mask2) {
          int bit = __builtin_ctzll(mask2);
          int idx = (w << 6) + bit;
          if (idx < size && entry_1[idx].valid && entry_1[idx].uop->src2_en &&
              entry_1[idx].uop.src2_busy && entry_1[idx].uop->src2_preg == preg) {
            entry_1[idx].uop.src2_busy = false;
            mark_slot(idx);
          }
          mask2 &= (mask2 - 1);
        }
        wake_matrix_src2_1[row_base + w] = 0;
      }
    }
  }

  void comb_issue() {
    for (auto &grant : out.issue_grants) {
      grant = {};
    }
    const std::vector<std::pair<int, int>> &scheduled_pairs = schedule();
    committed_indices.clear();

    for (const auto &pair : scheduled_pairs) {
      int entry_idx = pair.first;
      int phys_port = pair.second;
      uint64_t req_bit = (1ULL << static_cast<uint32_t>(entry[entry_idx].uop.op));
      if ((in.port_fu_ready_mask[phys_port] & req_bit) &&
          !in.issue_block) {
        out.issue_grants[phys_port].valid = true;
        out.issue_grants[phys_port].uop = entry[entry_idx].uop;
        committed_indices.push_back(entry_idx);
      }
    }

    commit_issue(committed_indices);
    out.free_slots = size - count_1;
  }

  void comb_flush() {
    if (in.flush_all) {
      flush_all();
    } else if (in.flush_br) {
      flush_br(in.flush_br_mask);
    }
    if (in.clear_mask) {
      clear_br(in.clear_mask);
    }
    out.free_slots = size - count_1;
  }

private:
  // 入队 (返回成功入队的个数)
  int enqueue(const IqStoredEntry &inst) {
    if (count_1 >= size)
      return 0;
    for (int i = 0; i < size; i++) {
      if (!entry_1[i].valid) {
#ifndef IQ_NEG_CONTROL
        mark_slot(i);
#endif
        entry_1[i] = inst;
        entry_1[i].valid = true;
        set_dep_bits_for_slot(entry_1[i], i);

        count_1++;
        return 1;
      }
    }
    return 0;
  }

  // Flush
  void flush_br(wire<BR_MASK_WIDTH> br_mask) {
    for (int i = 0; i < size; i++) {
      if (!entry_1[i].valid) {
        continue;
      }
      bool match_mask = (entry_1[i].uop.br_mask & br_mask) != 0;
      if (match_mask) {
        clear_dep_bits_for_slot(entry_1[i], i);
        entry_1[i].valid = false;
        mark_slot(i);
        count_1--;
      }
    }
  }

  // Clear resolved branch bits from surviving entries
  void clear_br(wire<BR_MASK_WIDTH> clear_mask) {
    if (clear_mask == 0) return;
    for (int i = 0; i < size; i++) {
      if (entry_1[i].valid) {
        entry_1[i].uop.br_mask &= ~clear_mask;
        mark_slot(i);
      }
    }
  }

  void flush_all() {
    for (auto &e : entry_1)
      e.valid = false;
    entry_dirty = true;
    entry_all_dirty = true;
    matrix_all_dirty = true;
    count_1 = 0;
    
    // Clear Wakeup Matrices
    std::fill(wake_matrix_src1_1.begin(), wake_matrix_src1_1.end(), 0);
    std::fill(wake_matrix_src2_1.begin(), wake_matrix_src2_1.end(), 0);
  }

  // 提交调度结果 (将选中的指令移出队列)
  void commit_issue(const std::vector<int> &indices) {
    for (int idx : indices) {
      if (entry_1[idx].valid) {
        clear_dep_bits_for_slot(entry_1[idx], idx);
        entry_1[idx].valid = false;
        mark_slot(idx);
        count_1--;
      }
    }
  }

public:
  void seq() {
    if (entry_dirty) {
      if (entry_all_dirty) {
        entry = entry_1;
        entry_all_dirty = false;
      } else {
        for (size_t k = 0; k < slot_dirty.size(); ++k) {
          for (uint64_t m = slot_dirty[k]; m; m &= m - 1) {
            const size_t i = k * 64 + __builtin_ctzll(m);
            entry[i] = entry_1[i];
          }
        }
      }
      std::fill(slot_dirty.begin(), slot_dirty.end(), 0);
      entry_dirty = false;
    }
    count = count_1;
    if (matrix_all_dirty) {
      wake_matrix_src1 = wake_matrix_src1_1;
      wake_matrix_src2 = wake_matrix_src2_1;
      matrix_all_dirty = false;
      for (uint32_t r : dirty_rows) row_dirty[r] = 0;
      dirty_rows.clear();
    } else {
      for (uint32_t r : dirty_rows) {
        const size_t base = matrix_row_base(r);
        for (int w = 0; w < wake_words_per_row; w++) {
          wake_matrix_src1[base + w] = wake_matrix_src1_1[base + w];
          wake_matrix_src2[base + w] = wake_matrix_src2_1[base + w];
        }
        row_dirty[r] = 0;
      }
      dirty_rows.clear();
    }
#ifdef IQ_SELFCHECK
    selfcheck("seq");
#endif
  }

private:
  // Scratch buffers reused every cycle (no per-cycle heap traffic).
  std::vector<std::pair<int, int>> sched_result;
  std::vector<int> ready_indices;
  std::vector<uint8_t> inst_issued;
  std::vector<int> committed_indices;

  const std::vector<std::pair<int, int>> &schedule() {
    std::vector<std::pair<int, int>> &result = sched_result;
    result.clear();
    int num_ports = ports.size();


    // 1. Gather all ready instructions
    ready_indices.clear();
    int seen_valid = 0;
    for (int i = 0; i < size; i++) {
      if (!entry[i].valid) {
        continue;
      }
      seen_valid++;
      if (is_ready(entry[i])) {
        ready_indices.push_back(i);
      }
      if (seen_valid >= count) {
        break;
      }
    }

    // 2. Sort ready instructions by priority
    if (ISSUE_SCHEDULE_POLICY == IssueSchedulePolicy::ROB_OLDEST_FIRST) {
      auto older_than = [&](int lhs, int rhs) {
        const auto &a = entry[lhs].uop;
        const auto &b = entry[rhs].uop;
        if (a.rob_flag == b.rob_flag) {
          if (a.rob_idx != b.rob_idx) {
            return a.rob_idx < b.rob_idx;
          }
        } else {
          if (a.rob_idx != b.rob_idx) {
            return a.rob_idx > b.rob_idx;
          }
        }
        return lhs < rhs; // Stable tie-breaker.
      };
      std::sort(ready_indices.begin(), ready_indices.end(), older_than);
    }
    // If IQ_SLOT_PRIORITY, ready_indices is already sorted by slot index (lowest first).

    // 3. Port-centric scheduling: for each port, find the best instruction
    inst_issued.assign(size, 0);
    
    for (int p = 0; p < num_ports; p++) {
      for (int idx : ready_indices) {
        if (inst_issued[idx]) {
          continue;
        }
        
        uint32_t op_type = entry[idx].uop.op;
        uint64_t op_bit = (1ULL << op_type);
        
        if (ports[p].capability_mask & op_bit) {
          result.push_back({idx, ports[p].port_idx});
          inst_issued[idx] = 1;
          break; // Move to the next port
        }
      }
    }

    return result;
  }
  size_t matrix_row_base(uint32_t preg) const {
    return static_cast<size_t>(preg) * static_cast<size_t>(wake_words_per_row);
  }

  uint64_t slot_bit(int idx) const {
    return 1ULL << (idx & 63);
  }

  int slot_word(int idx) const {
    return idx >> 6;
  }

  void set_dep_bits_for_slot(const IqStoredEntry &ent, int idx) {
    int word = slot_word(idx);
    uint64_t bit = slot_bit(idx);

    if (ent.uop->src1_en && ent.uop.src1_busy) {
      uint32_t preg = ent.uop->src1_preg;
      if (preg < PRF_NUM) {
        wake_matrix_src1_1[matrix_row_base(preg) + word] |= bit;
        mark_row(preg);
      }
    }
    if (ent.uop->src2_en && ent.uop.src2_busy) {
      uint32_t preg = ent.uop->src2_preg;
      if (preg < PRF_NUM) {
        wake_matrix_src2_1[matrix_row_base(preg) + word] |= bit;
        mark_row(preg);
      }
    }
  }

  void clear_dep_bits_for_slot(const IqStoredEntry &ent, int idx) {
    int word = slot_word(idx);
    uint64_t bit = slot_bit(idx);
    uint64_t clear_mask = ~bit;

    if (ent.uop->src1_en) {
      uint32_t preg = ent.uop->src1_preg;
      if (preg < PRF_NUM) {
        wake_matrix_src1_1[matrix_row_base(preg) + word] &= clear_mask;
        mark_row(preg);
      }
    }
    if (ent.uop->src2_en) {
      uint32_t preg = ent.uop->src2_preg;
      if (preg < PRF_NUM) {
        wake_matrix_src2_1[matrix_row_base(preg) + word] &= clear_mask;
        mark_row(preg);
      }
    }
  }

  bool is_ready(const IqStoredEntry &ent) {
    const auto &op = ent.uop;
    bool ops_ok =
        (!op->src1_en || !op.src1_busy) && (!op->src2_en || !op.src2_busy);
    return ops_ok ;
  }
};

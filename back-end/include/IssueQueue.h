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
#include <functional>
#include <vector>

// Why a selected grant was refused (0: granted). The order is the index + 1 of PmuBackSignals::iss_ref.
enum GateRefuse : int { GATE_OK = 0, GATE_REF_FU = 1, GATE_REF_WB = 2, GATE_REF_FPWB = 3, GATE_REF_AGU = 4 };

// operation-type bit masks of the issue ports

// ==========================================
// 1. configuration
// ==========================================
//
struct PortBinding {
  int port_idx;             // physical port number (index of out.iss2prf)
  uint64_t capability_mask; // operations the port supports
};

struct IssueQueueConfig {
  int id;                         // IQ id (IQ_INT, IQ_MEM, ...)
  int size;                       // depth
  int dispatch_width;             // max enqueues per cycle (write ports)
  uint64_t supported_ops;         // operations the IQ accepts
  std::vector<PortBinding> ports; // its issue ports
};

struct IqStoredEntry {
  wire<1> valid;
  Uop uop;
};

struct IssueQueueIn {
  std::vector<IqStoredEntry> enq_reqs;
  uint32_t wake_pregs[WAKE_CH_ALL] = {};
  wire<1> wake_valid[WAKE_CH_ALL] = {};
  wire<1> issue_block = 0;
  wire<1> flush_all = 0;
  wire<1> flush_br = 0;
  wire<BR_MASK_WIDTH> flush_br_mask = 0;
  wire<BR_MASK_WIDTH> clear_mask = 0;
  wire<MAX_UOP_TYPE> port_fu_ready_mask[ISSUE_WIDTH] = {};
  // extra grant condition (completion-cycle reservation of the port, FP write-port reservation, shared AGU): 0 = granted (the reservations
  // are made), otherwise the GateRefuse reason; called only for a grant about to be made
  std::function<int(const Uop &, int)> grant_gate;
  std::function<int(const Uop &, int)> gate_probe;     // the same check without reserving (statistics only)
  std::function<bool(const Uop &, int)> presel_ok;     // CFG_WB_PRESEL: the uop's completion cycle on the port is free (select mask)
  std::function<void(int why, bool lost)> on_refuse;   // statistics: a selected grant was refused (why: GateRefuse)
};

struct IssueQueueOut {
  wire<IQ_READY_NUM_WIDTH> free_slots = 0;
  IqStoredEntry issue_grants[ISSUE_WIDTH];
};

// ==========================================
// 2. IssueQueue (logic only, no IO)
// ==========================================
class IssueQueue {
public:
  friend struct IsuDigestAccess;  // lockstep test hook
  IssueQueueIn in;
  IssueQueueOut out;

private:
  int id_ = 0;   // IQ id (IQ_INT, IQ_MEM, ...)
  int size;
  int dispatch_width;
  std::vector<PortBinding> ports;
  int wake_words_per_row;
public:
  int port_lo = 0, port_hi = 0;   // the queue's physical issue ports are [port_lo, port_hi)
private:

  // Registers: read by comb_issue (the select), updated only by seq: the picks leave, this cycle's wake-ups clear busy
  // bits, the enqueue requests enter, then the flush / branch-clear.
  struct Regs {
    std::vector<IqStoredEntry> entry;
    int count = 0;
    // Wakeup Matrix: [Physical Register] -> Bitmask of IQ slots
    std::vector<uint64_t> wake_matrix_src1;
    std::vector<uint64_t> wake_matrix_src2;
    std::vector<uint64_t> wake_matrix_src3;   // F/D: the third source of the fused multiply-add (empty without FP)
  };
  Regs q_;

  // Slots changed at this edge: seq() refreshes only their ready bits.
  bool entry_dirty = false;
  bool entry_all_dirty = false;
  std::vector<uint64_t> slot_dirty;
  std::vector<uint64_t> d_vmask;     // bit i = entry[i].valid, so scans visit only the live slots
  std::vector<uint64_t> alloc_mask_; // CFG_IQ_SAME_CYCLE_REUSE = 0: the valid bits at the start of the edge (the slots an enqueue may take)
  std::vector<uint64_t> ready_bits;  // valid && is_ready(q_.entry[i]), refreshed in seq()
  void refresh_ready(int i) {
    const Regs &q = q_;
    const uint64_t bit = 1ull << (i & 63);
    if (q.entry[i].valid && is_ready(q.entry[i])) ready_bits[i >> 6] |= bit;
    else ready_bits[i >> 6] &= ~bit;
  }
  void mark_slot(int i) {
    slot_dirty[i >> 6] |= 1ull << (i & 63);
    entry_dirty = true;
  }
  std::vector<uint32_t> wake_pregs_;   // this cycle's wake-ups (the valid channels of the bus), applied at seq

public:
  IssueQueue(const IssueQueueConfig &cfg)
      : id_(cfg.id), size(cfg.size), dispatch_width(cfg.dispatch_width),
        ports(cfg.ports) {
    if (!ports.empty()) {
      port_lo = port_hi = ports[0].port_idx;
      for (const auto &pb : ports) { port_lo = std::min(port_lo, pb.port_idx); port_hi = std::max(port_hi, pb.port_idx + 1); }
    }
    q_.entry.resize(size);
    slot_dirty.assign((size + 63) / 64, 0);
    ready_bits.assign((size + 63) / 64, 0);
    d_vmask.assign((size + 63) / 64, 0);
    q_.count = 0;
    wake_words_per_row = (size + 63) / 64;

    // Initialize segmented wakeup matrices:
    // [preg][word_idx], where each word tracks 64 IQ slots.
    q_.wake_matrix_src1.resize(TAG_NUM * wake_words_per_row, 0);
    q_.wake_matrix_src2.resize(TAG_NUM * wake_words_per_row, 0);
    {
      q_.wake_matrix_src3.resize(TAG_NUM * wake_words_per_row, 0);
    }

    in.enq_reqs.resize(dispatch_width);
  }

  void comb_begin() {
    const Regs &q = q_;
    out.free_slots = size - q.count;
    // (the wake-up inputs, the enqueue requests and the FU ready masks are written by the Isu phases every cycle before they are read)
    in.issue_block = 0;
    in.flush_all = 0;
    in.flush_br = 0;
    in.flush_br_mask = 0;
    in.clear_mask = 0;
  }

  // The registers woken this cycle (the valid channels of the wake-up bus, in channel order); applied at seq.
  void set_wakeup(const uint32_t *pregs, int n) { wake_pregs_.assign(pregs, pregs + n); }

private:
  void wakeup(const uint32_t *pregs, int n) {
    for (int i = 0; i < n; i++) {
      uint32_t preg = pregs[i];
      if (preg >= TAG_NUM) {
        continue;
      }
      size_t row_base = matrix_row_base(preg);

      for (int w = 0; w < wake_words_per_row; w++) {
        uint64_t mask1 = q_.wake_matrix_src1[row_base + w];
        while (mask1) {
          int bit = __builtin_ctzll(mask1);
          int idx = (w << 6) + bit;
          if (idx < size && q_.entry[idx].valid && q_.entry[idx].uop->src1_en &&
              q_.entry[idx].uop.src1_busy && q_.entry[idx].uop->src1_preg == preg) {
            q_.entry[idx].uop.src1_busy = false;
            mark_slot(idx);
          }
          mask1 &= (mask1 - 1);
        }
        q_.wake_matrix_src1[row_base + w] = 0;

        uint64_t mask2 = q_.wake_matrix_src2[row_base + w];
        while (mask2) {
          int bit = __builtin_ctzll(mask2);
          int idx = (w << 6) + bit;
          if (idx < size && q_.entry[idx].valid && q_.entry[idx].uop->src2_en &&
              q_.entry[idx].uop.src2_busy && q_.entry[idx].uop->src2_preg == preg) {
            q_.entry[idx].uop.src2_busy = false;
            mark_slot(idx);
          }
          mask2 &= (mask2 - 1);
        }
        q_.wake_matrix_src2[row_base + w] = 0;

        {
          uint64_t mask3 = q_.wake_matrix_src3[row_base + w];
          while (mask3) {
            int bit = __builtin_ctzll(mask3);
            int idx = (w << 6) + bit;
            if (idx < size && q_.entry[idx].valid && q_.entry[idx].uop->src3_en &&
                q_.entry[idx].uop.src3_busy && q_.entry[idx].uop->src3_preg == preg) {
              q_.entry[idx].uop.src3_busy = false;
              mark_slot(idx);
            }
            mask3 &= (mask3 - 1);
          }
          q_.wake_matrix_src3[row_base + w] = 0;
        }
      }
    }
  }

public:
  void comb_issue() {
    const Regs &q = q_;
    // Grants are all-zero unless the previous call filled one, and most cycles have nothing ready.
    if (grants_live) {   // only the valid bits: a payload is read only under its valid bit, and only the queue's own ports
      for (int p = port_lo; p < port_hi; p++) out.issue_grants[p].valid = false;
      grants_live = false;
    }
    bool any_ready = false;
    for (uint64_t w : ready_bits) any_ready |= (w != 0);
    if (!any_ready) return;
    const std::vector<std::pair<int, int>> &scheduled_pairs = schedule();

    for (const auto &pair : scheduled_pairs) {
      int entry_idx = pair.first;
      int phys_port = pair.second;
      uint64_t req_bit = (1ULL << static_cast<uint32_t>(q.entry[entry_idx].uop.op));
      if (in.issue_block) continue;
      const int why = !(in.port_fu_ready_mask[phys_port] & req_bit) ? GATE_REF_FU
                      : in.grant_gate ? in.grant_gate(q.entry[entry_idx].uop, phys_port) : 0;
      if (!why) {
        out.issue_grants[phys_port].valid = true;
        out.issue_grants[phys_port].uop = q.entry[entry_idx].uop;
        grants_live = true;
        committed_indices.push_back(entry_idx);
      } else if (in.on_refuse) {
        in.on_refuse(why, alt_exists(phys_port));
      }
    }

  }

private:
  // enqueue (returns the number enqueued)
  int enqueue(const IqStoredEntry &inst) {
    if (q_.count >= size)
      return 0;
    for (size_t w = 0; w < d_vmask.size(); w++) {   // lowest free slot
      const uint64_t freebits = CFG_IQ_SAME_CYCLE_REUSE ? ~d_vmask[w] : ~(alloc_mask_[w] | d_vmask[w]);
      if (!freebits) continue;
      const int i = int(w * 64) + __builtin_ctzll(freebits);
      if (i >= size) break;
#ifndef IQ_NEG_CONTROL
      mark_slot(i);
#endif
      q_.entry[i] = inst;
      q_.entry[i].valid = true;
      d_vmask[w] |= 1ull << (i & 63);
      set_dep_bits_for_slot(q_.entry[i], i);

      q_.count++;
      return 1;
    }
    return 0;
  }

  // Flush
  void flush_br(wire<BR_MASK_WIDTH> br_mask) {
    for (size_t w = 0; w < d_vmask.size(); w++)
      for (uint64_t m = d_vmask[w]; m; m &= m - 1) {
        const int i = int(w * 64) + __builtin_ctzll(m);
        if ((q_.entry[i].uop.br_mask & br_mask) == 0) continue;
        clear_dep_bits_for_slot(q_.entry[i], i);
        q_.entry[i].valid = false;
        d_vmask[w] &= ~(1ull << (i & 63));
        mark_slot(i);
        q_.count--;
      }
  }

  // Clear resolved branch bits from surviving entries
  void clear_br(wire<BR_MASK_WIDTH> clear_mask) {
    if (clear_mask == 0) return;
    for (size_t w = 0; w < d_vmask.size(); w++)
      for (uint64_t m = d_vmask[w]; m; m &= m - 1) {
        const int i = int(w * 64) + __builtin_ctzll(m);
        if ((q_.entry[i].uop.br_mask & clear_mask) == 0) continue;   // unchanged entries need no copy at seq()
        q_.entry[i].uop.br_mask &= ~clear_mask;
        mark_slot(i);
      }
  }

  void flush_all() {
    for (auto &e : q_.entry)
      e.valid = false;
    std::fill(d_vmask.begin(), d_vmask.end(), 0);
    entry_dirty = true;
    entry_all_dirty = true;
    q_.count = 0;
    
    // Clear Wakeup Matrices
    std::fill(q_.wake_matrix_src1.begin(), q_.wake_matrix_src1.end(), 0);
    std::fill(q_.wake_matrix_src2.begin(), q_.wake_matrix_src2.end(), 0);
    std::fill(q_.wake_matrix_src3.begin(), q_.wake_matrix_src3.end(), 0);
  }

  // apply the issue selection (remove the selected entries)
  void commit_issue(const std::vector<int> &indices) {
    for (int idx : indices) {
      if (q_.entry[idx].valid) {
        clear_dep_bits_for_slot(q_.entry[idx], idx);
        q_.entry[idx].valid = false;
        d_vmask[idx >> 6] &= ~(1ull << (idx & 63));
        mark_slot(idx);
        q_.count--;
      }
    }
  }

public:
  // The clock edge: the picks of this cycle leave, the wake-ups clear busy bits, the enqueue requests enter (their
  // wake-ups already applied by the Isu), then the flush / resolved-branch clear; the ready bits of the changed slots.
  void seq() {
    if (!CFG_IQ_SAME_CYCLE_REUSE) alloc_mask_ = d_vmask;
    commit_issue(committed_indices);
    committed_indices.clear();
    if (!wake_pregs_.empty()) wakeup(wake_pregs_.data(), int(wake_pregs_.size()));
    for (const auto &req : in.enq_reqs) {
      if (!req.valid) continue;
      const int success = enqueue(req);
      Assert(success && "issue queue overflow: dispatch logic error");
    }
    if (in.flush_all) flush_all();
    else if (in.flush_br) flush_br(in.flush_br_mask);
    if (in.clear_mask) clear_br(in.clear_mask);
    if (entry_dirty) {
      if (entry_all_dirty) {
        for (int i = 0; i < size; i++) refresh_ready(i);
        entry_all_dirty = false;
      } else {
        for (size_t k = 0; k < slot_dirty.size(); ++k)
          for (uint64_t m = slot_dirty[k]; m; m &= m - 1) refresh_ready(int(k * 64 + __builtin_ctzll(m)));
      }
      std::fill(slot_dirty.begin(), slot_dirty.end(), 0);
      entry_dirty = false;
    }
  }

private:
  // Scratch buffers reused every cycle (no per-cycle heap traffic).
  std::vector<std::pair<int, int>> sched_result;
  std::vector<int> ready_indices;
  std::vector<uint8_t> inst_issued;
  std::vector<int> committed_indices;
  bool grants_live = true;  // out.issue_grants may hold a payload (conservative until the first comb_issue)

  // Statistics (EXP-045): a ready uop that the select did not pick could have been granted on this port.
  bool alt_exists(int phys_port) const {
    int local = -1;
    for (size_t k = 0; k < ports.size(); k++) if (ports[k].port_idx == phys_port) local = static_cast<int>(k);
    if (local < 0) return false;
    for (int idx : ready_indices) {
      if (inst_issued[idx]) continue;
      const uint64_t bit = 1ULL << static_cast<uint32_t>(q_.entry[idx].uop.op);
      if (!(ports[local].capability_mask & bit) || !(in.port_fu_ready_mask[phys_port] & bit)) continue;
      if (!in.gate_probe || in.gate_probe(q_.entry[idx].uop, phys_port) == 0) return true;
    }
    return false;
  }

  const std::vector<std::pair<int, int>> &schedule() {
    std::vector<std::pair<int, int>> &result = sched_result;
    result.clear();
    int num_ports = ports.size();

    // 1. Gather all ready instructions
    ready_indices.clear();
    for (size_t w = 0; w < ready_bits.size(); w++)
      for (uint64_t m = ready_bits[w]; m; m &= m - 1) ready_indices.push_back(int(w * 64) + __builtin_ctzll(m));

    // 2. Sort ready instructions by priority
    if (ISSUE_SCHEDULE_POLICY == IssueSchedulePolicy::ROB_OLDEST_FIRST) {
      auto older_than = [&](int lhs, int rhs) {
        const auto &a = q_.entry[lhs].uop;
        const auto &b = q_.entry[rhs].uop;
        if (a->rob_flag == b->rob_flag) {
          if (a->rob_idx != b->rob_idx) {
            return a->rob_idx < b->rob_idx;
          }
        } else {
          if (a->rob_idx != b->rob_idx) {
            return a->rob_idx > b->rob_idx;
          }
        }
        return lhs < rhs; // Stable tie-breaker.
      };
      std::sort(ready_indices.begin(), ready_indices.end(), older_than);
    }
    // If IQ_SLOT_PRIORITY, ready_indices is already sorted by slot index (lowest first).

    // 3. Port-centric scheduling: for each port, find the best instruction
    inst_issued.assign(size, 0);
    
    // OLDEST_ON_FIRST_PORT: the first port of each capability (no earlier port shares a capability bit with it) takes the oldest ready
    // uop it can serve; the other ports take the lowest slot.
    std::vector<int> by_age;
    const bool age_policy = ISSUE_SCHEDULE_POLICY == IssueSchedulePolicy::OLDEST_ON_FIRST_PORT && ((CFG_ISSUE_OLDEST_QUEUES >> id_) & 1);
    if (age_policy) {
      by_age = ready_indices;
      std::sort(by_age.begin(), by_age.end(), [&](int lhs, int rhs) {
        const auto &a = q_.entry[lhs].uop;
        const auto &b = q_.entry[rhs].uop;
        if (a->rob_flag == b->rob_flag) {
          if (a->rob_idx != b->rob_idx) return a->rob_idx < b->rob_idx;
        } else {
          if (a->rob_idx != b->rob_idx) return a->rob_idx > b->rob_idx;
        }
        return lhs < rhs;
      });
    }
    for (int p = 0; p < num_ports; p++) {
      bool first_of_cap = true;
      if (age_policy)
        for (int q = 0; q < p; q++)
          if (ports[q].capability_mask & ports[p].capability_mask) first_of_cap = false;
      const std::vector<int> &order = (first_of_cap && age_policy) ? by_age : ready_indices;
      for (int idx : order) {
        if (inst_issued[idx]) {
          continue;
        }
        
        uint32_t op_type = q_.entry[idx].uop.op;
        uint64_t op_bit = (1ULL << op_type);
        
        if ((ports[p].capability_mask & op_bit) &&
            (!CFG_WB_PRESEL || ((in.port_fu_ready_mask[ports[p].port_idx] & op_bit) &&
                                (!in.presel_ok || in.presel_ok(q_.entry[idx].uop, ports[p].port_idx))))) {
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
      if (preg < TAG_NUM) {
        q_.wake_matrix_src1[matrix_row_base(preg) + word] |= bit;
      }
    }
    if (ent.uop->src2_en && ent.uop.src2_busy) {
      uint32_t preg = ent.uop->src2_preg;
      if (preg < TAG_NUM) {
        q_.wake_matrix_src2[matrix_row_base(preg) + word] |= bit;
      }
    }
    if (ent.uop->src3_en && ent.uop.src3_busy) {
      uint32_t preg = ent.uop->src3_preg;
      if (preg < TAG_NUM) {
        q_.wake_matrix_src3[matrix_row_base(preg) + word] |= bit;
      }
    }
  }

  void clear_dep_bits_for_slot(const IqStoredEntry &ent, int idx) {
    int word = slot_word(idx);
    uint64_t bit = slot_bit(idx);
    uint64_t clear_mask = ~bit;

    if (ent.uop->src1_en) {
      uint32_t preg = ent.uop->src1_preg;
      if (preg < TAG_NUM) {
        q_.wake_matrix_src1[matrix_row_base(preg) + word] &= clear_mask;
      }
    }
    if (ent.uop->src2_en) {
      uint32_t preg = ent.uop->src2_preg;
      if (preg < TAG_NUM) {
        q_.wake_matrix_src2[matrix_row_base(preg) + word] &= clear_mask;
      }
    }
    if (ent.uop->src3_en) {
      uint32_t preg = ent.uop->src3_preg;
      if (preg < TAG_NUM) {
        q_.wake_matrix_src3[matrix_row_base(preg) + word] &= clear_mask;
      }
    }
  }

  bool is_ready(const IqStoredEntry &ent) {
    const auto &op = ent.uop;
    bool ops_ok =
        (!op->src1_en || !op.src1_busy) && (!op->src2_en || !op.src2_busy) && (!op->src3_en || !op.src3_busy);
    return ops_ok ;
  }
};

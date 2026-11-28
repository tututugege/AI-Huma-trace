#pragma once

#include "config.h"
#include "AguPorts.h"
#include "DseCap.h"
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
  uint8_t spec_t1 = 0;  // 4-bit L1-spec load port mask for uops woken this cycle / eligible to enter RR next cycle
};

struct IssueQueueIn {
  std::vector<IqStoredEntry> enq_reqs;
  uint32_t wake_pregs[WAKE_CH_ALL] = {};
  wire<1> wake_valid[WAKE_CH_ALL] = {};
  wire<1> issue_block = 0;
  wire<1> flush_all = 0;
  wire<1> flush_br = 0;
  wire<ROB_IDX_WIDTH> flush_rob_idx = 0;
  wire<1> flush_rob_flag = 0;
  // extra grant condition (completion-cycle reservation of the port, FP write-port reservation): 0 = granted (the reservations
  // are made), otherwise the GateRefuse reason; called only for a grant about to be made
  std::function<int(const Uop &, int)> grant_gate;
  std::function<int(const Uop &, int)> gate_probe;     // the same check without reserving (statistics only)
  // EXP-046: the select skips a ready uop whose completion cycle on the port is already reserved (EXP-035), so the next ready uop
  // gets the port instead of the port idling (the check reads only registered per-port state; grant_gate keeps its own check)
  std::function<bool(const Uop &, int)> presel_ok;
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
    std::vector<uint8_t> spec_t1;  // valid for 1 cycle after L1-spec wakeup until grant or end-of-cycle clear
    std::vector<uint8_t> spec_rr;  // non-zero while an issued L1-spec uop awaits cancel verdict in RR
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
  bool has_spec_t1_ = false;
  bool has_spec_rr_ = false;
  std::vector<uint64_t> slot_dirty;
  std::vector<uint64_t> d_vmask;     // bit i = entry[i].valid, so scans visit only the live slots
  std::vector<uint64_t> alloc_mask_; // CFG_IQ_SAME_CYCLE_REUSE = 0: the valid bits at the start of the edge (the slots an enqueue may take)
  std::vector<uint64_t> ready_bits;  // valid && is_ready(q_.entry[i]) && spec_rr[i] == 0, refreshed in seq()
  void refresh_ready(int i) {
    const Regs &q = q_;
    const uint64_t bit = 1ull << (i & 63);
    if (q.entry[i].valid && is_ready(q.entry[i]) && q.spec_rr[i] == 0) ready_bits[i >> 6] |= bit;
    else ready_bits[i >> 6] &= ~bit;
  }
  void mark_slot(int i) {
    slot_dirty[i >> 6] |= 1ull << (i & 63);
    entry_dirty = true;
  }
  std::vector<uint32_t> wake_pregs_;   // this cycle's wake-ups (the valid channels of the bus), applied at seq
  std::vector<uint8_t> wake_spec_masks_;

public:
  IssueQueue(const IssueQueueConfig &cfg)
      : id_(cfg.id), size(cfg.size), dispatch_width(cfg.dispatch_width),
        ports(cfg.ports) {
    if (!ports.empty()) {
      port_lo = port_hi = ports[0].port_idx;
      for (const auto &pb : ports) { port_lo = std::min(port_lo, pb.port_idx); port_hi = std::max(port_hi, pb.port_idx + 1); }
    }
    q_.entry.resize(size);
    q_.spec_t1.assign(size, 0);
    q_.spec_rr.assign(size, 0);
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
    out.free_slots = dse_free(dse_iq_cap(id_, size), q.count);
    // (the wake-up inputs, the enqueue requests and the FU ready masks are written by the Isu phases every cycle before they are read)
    in.issue_block = 0;
    in.flush_all = 0;
    in.flush_br = 0;
    in.flush_rob_idx = 0;
    in.flush_rob_flag = 0;
  }

  // The registers woken this cycle (the valid channels of the wake-up bus, in channel order); applied at seq.
  void set_wakeup(const uint32_t *pregs, int n, const uint8_t *spec_masks = nullptr) {
    wake_pregs_.assign(pregs, pregs + n);
    if (spec_masks) wake_spec_masks_.assign(spec_masks, spec_masks + n);
    else wake_spec_masks_.assign(n, 0);
  }
  void set_cancel(const uint32_t *pregs, int n, uint8_t port_mask = 0) {
    cancel_pregs_.assign(pregs, pregs + n);
    cancel_port_mask_ = port_mask;
  }

private:
  std::vector<uint32_t> cancel_pregs_;
  uint8_t cancel_port_mask_ = 0;
  bool in_cancel(uint32_t preg) const {
    for (uint32_t p : cancel_pregs_) if (p == preg) return true;
    return false;
  }
  bool is_cancelled(const Uop &u) const {
    if (cancel_pregs_.empty()) return false;
    return (u->src1_en && in_cancel(u->src1_preg)) ||
           (u->src2_en && in_cancel(u->src2_preg)) ||
           (u->src3_en && in_cancel(u->src3_preg));
  }
  void apply_cancel(const std::vector<uint64_t> *must_restore = nullptr) {
    for (size_t w = 0; w < d_vmask.size(); w++)
      for (uint64_t m = d_vmask[w]; m; m &= m - 1) {
        const int idx = int(w * 64) + __builtin_ctzll(m);
        if (q_.spec_rr[idx] != 0) continue;
        auto &ent = q_.entry[idx];
        bool chg = false;
        if (ent.uop->src1_en && !ent.uop.src1_busy && in_cancel(ent.uop->src1_preg)) { ent.uop.src1_busy = true; chg = true; }
        if (ent.uop->src2_en && !ent.uop.src2_busy && in_cancel(ent.uop->src2_preg)) { ent.uop.src2_busy = true; chg = true; }
        if (ent.uop->src3_en && !ent.uop.src3_busy && in_cancel(ent.uop->src3_preg)) { ent.uop.src3_busy = true; chg = true; }
        if (must_restore && ((*must_restore)[w] & (1ull << (idx & 63)))) {
          Assert(chg && "cancelled spec_rr IQ slot missing src_busy restore");
        }
        if (chg) {
          set_dep_bits_for_slot(ent, idx);
          mark_slot(idx);
        }
      }
  }
  void wakeup(const uint32_t *pregs, int n) {
    for (int i = 0; i < n; i++) {
      uint32_t preg = pregs[i];
      if (preg >= TAG_NUM) {
        continue;
      }
      const uint8_t spec_m = (i < int(wake_spec_masks_.size())) ? wake_spec_masks_[i] : 0;
      size_t row_base = matrix_row_base(preg);

      for (int w = 0; w < wake_words_per_row; w++) {
        uint64_t mask1 = q_.wake_matrix_src1[row_base + w];
        while (mask1) {
          int bit = __builtin_ctzll(mask1);
          int idx = (w << 6) + bit;
          if (idx < size && q_.entry[idx].valid && q_.spec_rr[idx] == 0 && q_.entry[idx].uop->src1_en &&
              q_.entry[idx].uop.src1_busy && q_.entry[idx].uop->src1_preg == preg) {
            q_.entry[idx].uop.src1_busy = false;
            if (spec_m) { q_.spec_t1[idx] |= spec_m; has_spec_t1_ = true; }
            mark_slot(idx);
          }
          mask1 &= (mask1 - 1);
        }
        q_.wake_matrix_src1[row_base + w] = 0;

        uint64_t mask2 = q_.wake_matrix_src2[row_base + w];
        while (mask2) {
          int bit = __builtin_ctzll(mask2);
          int idx = (w << 6) + bit;
          if (idx < size && q_.entry[idx].valid && q_.spec_rr[idx] == 0 && q_.entry[idx].uop->src2_en &&
              q_.entry[idx].uop.src2_busy && q_.entry[idx].uop->src2_preg == preg) {
            q_.entry[idx].uop.src2_busy = false;
            if (spec_m) { q_.spec_t1[idx] |= spec_m; has_spec_t1_ = true; }
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
            if (idx < size && q_.entry[idx].valid && q_.spec_rr[idx] == 0 && q_.entry[idx].uop->src3_en &&
                q_.entry[idx].uop.src3_busy && q_.entry[idx].uop->src3_preg == preg) {
              q_.entry[idx].uop.src3_busy = false;
              if (spec_m) { q_.spec_t1[idx] |= spec_m; has_spec_t1_ = true; }
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
      if (in.issue_block) continue;
      const int why = in.grant_gate ? in.grant_gate(q.entry[entry_idx].uop, phys_port) : 0;
      if (!why) {
        out.issue_grants[phys_port].valid = true;
        out.issue_grants[phys_port].uop = q.entry[entry_idx].uop;
        out.issue_grants[phys_port].spec_t1 = q.spec_t1[entry_idx];
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
    const int cap = dse_iq_cap(id_, size);
    if (q_.count >= cap)
      return 0;
    for (size_t w = 0; w < d_vmask.size(); w++) {   // lowest free slot (does not reuse slots freed this cycle)
      const uint64_t freebits = ~(alloc_mask_[w] | d_vmask[w]);
      if (!freebits) continue;
      const int i = int(w * 64) + __builtin_ctzll(freebits);
      if (i >= cap) break;
#ifndef IQ_NEG_CONTROL
      mark_slot(i);
#endif
      q_.entry[i] = inst;
      q_.entry[i].valid = true;
      q_.spec_t1[i] = inst.spec_t1;
      q_.spec_rr[i] = 0;
      if (inst.spec_t1) has_spec_t1_ = true;
      d_vmask[w] |= 1ull << (i & 63);
      set_dep_bits_for_slot(q_.entry[i], i);

      q_.count++;
      return 1;
    }
    return 0;
  }

  // Flush
  void flush_br() {
    for (size_t w = 0; w < d_vmask.size(); w++)
      for (uint64_t m = d_vmask[w]; m; m &= m - 1) {
        const int i = int(w * 64) + __builtin_ctzll(m);
        if (!is_younger_rob(q_.entry[i].uop->rob_idx, q_.entry[i].uop->rob_flag, in.flush_rob_idx, in.flush_rob_flag)) continue;
        clear_dep_bits_for_slot(q_.entry[i], i);
        q_.entry[i].valid = false;
        q_.spec_t1[i] = 0;
        q_.spec_rr[i] = 0;
        d_vmask[w] &= ~(1ull << (i & 63));
        mark_slot(i);
        q_.count--;
      }
  }

  void flush_all() {
    for (auto &e : q_.entry)
      e.valid = false;
    std::fill(q_.spec_t1.begin(), q_.spec_t1.end(), 0);
    std::fill(q_.spec_rr.begin(), q_.spec_rr.end(), 0);
    has_spec_t1_ = false;
    has_spec_rr_ = false;
    std::fill(d_vmask.begin(), d_vmask.end(), 0);
    entry_dirty = true;
    entry_all_dirty = true;
    q_.count = 0;
    
    // Clear Wakeup Matrices
    std::fill(q_.wake_matrix_src1.begin(), q_.wake_matrix_src1.end(), 0);
    std::fill(q_.wake_matrix_src2.begin(), q_.wake_matrix_src2.end(), 0);
    std::fill(q_.wake_matrix_src3.begin(), q_.wake_matrix_src3.end(), 0);
  }

  // apply the issue selection (remove non-speculative picks; park L1-spec picks in spec_rr for 1-cycle cancel confirmation)
  void commit_issue(const std::vector<int> &indices) {
    for (int idx : indices) {
      if (!q_.entry[idx].valid) continue;
      clear_dep_bits_for_slot(q_.entry[idx], idx);
      if (q_.spec_t1[idx] != 0) {
        q_.spec_rr[idx] = q_.spec_t1[idx];
        q_.spec_t1[idx] = 0;
        has_spec_rr_ = true;
        mark_slot(idx);
      } else {
        q_.entry[idx].valid = false;
        d_vmask[idx >> 6] &= ~(1ull << (idx & 63));
        mark_slot(idx);
        q_.count--;
      }
    }
  }

public:
  // The clock edge: resolve RR-parked spec picks, park/remove this cycle's picks, restore cancelled busy bits,
  // apply wake-ups, enqueue, flush/branch-clear, then refresh ready bits.
  void seq() {
    alloc_mask_ = d_vmask;
    std::vector<uint64_t> restored_rr_mask;
    bool has_restored_rr = false;
    if (has_spec_rr_) {
      restored_rr_mask.assign(d_vmask.size(), 0);
      for (size_t w = 0; w < d_vmask.size(); w++)
        for (uint64_t m = d_vmask[w]; m; m &= m - 1) {
          const int idx = int(w * 64) + __builtin_ctzll(m);
          const uint8_t sm = q_.spec_rr[idx];
          if (sm == 0) continue;
          if ((sm & cancel_port_mask_) == 0) {
            q_.entry[idx].valid = false;
            q_.spec_rr[idx] = 0;
            d_vmask[w] &= ~(1ull << (idx & 63));
            mark_slot(idx);
            q_.count--;
          } else {
            q_.spec_rr[idx] = 0;
            restored_rr_mask[w] |= (1ull << (idx & 63));
            has_restored_rr = true;
            mark_slot(idx);
          }
        }
      has_spec_rr_ = false;
    }
    commit_issue(committed_indices);
    committed_indices.clear();
    if (has_spec_t1_) {
      std::fill(q_.spec_t1.begin(), q_.spec_t1.end(), 0);
      has_spec_t1_ = false;
    }
    if (!cancel_pregs_.empty()) apply_cancel(has_restored_rr ? &restored_rr_mask : nullptr);
    else Assert(!has_restored_rr && "restored spec_rr without cancel_pregs");
    if (!wake_pregs_.empty()) wakeup(wake_pregs_.data(), int(wake_pregs_.size()));
    for (const auto &req : in.enq_reqs) {
      if (!req.valid) continue;
      const int success = enqueue(req);
      Assert(success && "issue queue overflow: dispatch logic error");
    }
    if (in.flush_all) flush_all();
    else if (in.flush_br) flush_br();
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

  // Statistics (EXP-046): a ready uop that the select did not pick could have been granted on this port.
  bool alt_exists(int phys_port) const {
    int local = -1;
    for (size_t k = 0; k < ports.size(); k++) if (ports[k].port_idx == phys_port) local = static_cast<int>(k);
    if (local < 0) return false;
    for (int idx : ready_indices) {
      if (inst_issued[idx]) continue;
      const uint64_t bit = 1ULL << static_cast<uint32_t>(q_.entry[idx].uop.op);
      if (!(ports[local].capability_mask & bit)) continue;
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
      for (uint64_t m = ready_bits[w]; m; m &= m - 1) {
        const int idx = int(w * 64) + __builtin_ctzll(m);
        if (!is_cancelled(q_.entry[idx].uop)) ready_indices.push_back(idx);
      }

    // 2. Port-centric scheduling: ready_indices is already sorted by slot index (lowest first).
    inst_issued.assign(size, 0);
    if (id_ == IQ_MEM) {
      // IQ_MEM (5 physical AGUs: NL load-only, NM shared load>STA, NS STA-only): pick oldest + remaining lowest-slot Loads and STAs.
      std::vector<int> by_age = ready_indices;
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
      auto pick_kind = [&](UopType target, int cap) {
        std::vector<int> picks;
        int oldest = -1;
        for (int idx : by_age)
          if (q_.entry[idx].uop.op == target) { oldest = idx; picks.push_back(idx); break; }
        for (int idx : ready_indices) {
          if ((int)picks.size() >= cap) break;
          if (idx != oldest && q_.entry[idx].uop.op == target) picks.push_back(idx);
        }
        return picks;
      };
      const std::vector<int> ld_picks = pick_kind(UOP_LOAD, LSU_LDU_COUNT);
      const std::vector<int> sta_picks = pick_kind(UOP_STA, LSU_STA_COUNT);
      uint32_t agu_used = 0;
      for (size_t k = 0; k < ld_picks.size(); k++) {
        const int ap = agu::ld_port(static_cast<int>(k));
        const int phys = IQ_AGU_PORT_BASE + ap;
        if (!in.presel_ok || in.presel_ok(q_.entry[ld_picks[k]].uop, phys)) {
          result.push_back({ld_picks[k], phys});
          inst_issued[ld_picks[k]] = 1;
          agu_used |= 1u << ap;
        }
      }
      for (size_t k = 0; k < sta_picks.size(); k++) {
        const int ap = agu::st_port(static_cast<int>(k));
        if (agu_used >> ap & 1) continue;
        const int phys = IQ_AGU_PORT_BASE + ap;
        if (!in.presel_ok || in.presel_ok(q_.entry[sta_picks[k]].uop, phys)) {
          result.push_back({sta_picks[k], phys});
          inst_issued[sta_picks[k]] = 1;
          agu_used |= 1u << ap;
        }
      }
      return result;
    }

    for (int p = 0; p < num_ports; p++) {
      for (int idx : ready_indices) {
        if (inst_issued[idx]) {
          continue;
        }
        
        uint32_t op_type = q_.entry[idx].uop.op;
        uint64_t op_bit = (1ULL << op_type);
        const bool port_no_pc =
            (ports[p].port_idx >= IQ_ALU_PORT_BASE &&
             ports[p].port_idx < IQ_ALU_PORT_BASE + ALU_NUM &&
             ports[p].port_idx != IQ_ALU_PORT_BASE &&
             ports[p].port_idx != IQ_ALU_PORT_BASE + 2);

        if ((ports[p].capability_mask & op_bit) &&
            !(port_no_pc && q_.entry[idx].uop->src1_is_pc) &&
            (!in.presel_ok || in.presel_ok(q_.entry[idx].uop, ports[p].port_idx))) {
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

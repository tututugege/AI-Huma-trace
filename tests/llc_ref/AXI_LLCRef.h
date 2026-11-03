#pragma once
// Frozen copy of the original AXI_LLC (differential reference, tests/llc_diff.cpp). Shares the register/IO structs with the live class.
#include "AXI_LLC.h"

namespace axi_interconnect {

void copy_regs_live_ref(AXI_LLC_Regs_t &dst, const AXI_LLC_Regs_t &src);

class AXI_LLCRef {
public:
  AXI_LLCRef();

  void set_config(const AXI_LLCConfig &config);
  const AXI_LLCConfig &config() const { return config_; }

  void reset();
  void comb();
  void seq();
  bool can_accept_read_now(uint8_t master, bool bypass, uint32_t addr) const;
  const AXI_LLCPerfCounters_t &perf_counters() const { return io.regs.perf; }
  void debug_print() const;

  static uint32_t line_words(const AXI_LLCConfig &config);
  static uint32_t line_addr(const AXI_LLCConfig &config, uint32_t addr);
  static uint32_t set_index(const AXI_LLCConfig &config, uint32_t addr);
  static uint32_t tag_of(const AXI_LLCConfig &config, uint32_t addr);
  static uint32_t valid_row_bytes(const AXI_LLCConfig &config);
  static AXI_LLCMetaEntry_t decode_meta(const AXI_LLC_Bytes_t &payload,
                                        uint32_t way);
  static void encode_meta(const AXI_LLCMetaEntry_t &entry,
                          AXI_LLC_Bytes_t &payload);

  AXI_LLC_IO_t io;

private:
  int find_free_mshr(const AXI_LLC_Regs_t &regs) const;
  int find_mshr_by_line_addr(const AXI_LLC_Regs_t &regs, uint32_t line_addr) const;
  int find_mshr_by_victim_addr(const AXI_LLC_Regs_t &regs,
                               uint32_t victim_addr) const;
  bool has_mshr_for_master(const AXI_LLC_Regs_t &regs, uint8_t master) const;
  uint32_t count_free_mshrs(const AXI_LLC_Regs_t &regs) const;
  bool has_demand_mshr(const AXI_LLC_Regs_t &regs) const;
  int pick_mem_issue_slot(const AXI_LLC_Regs_t &regs) const;
  int find_mshr_by_mem_id(const AXI_LLC_Regs_t &regs, uint8_t mem_id) const;
  int pick_refill_commit_slot(const AXI_LLC_Regs_t &regs) const;
  int pick_new_read_master(const AXI_LLC_Regs_t &regs) const;
  int pick_new_write_master(const AXI_LLC_Regs_t &regs) const;
  int pick_write_lookup_master(const AXI_LLC_Regs_t &regs) const;
  int pick_bypass_write_issue_master(const AXI_LLC_Regs_t &regs) const;
  int find_bypass_write_mem_owner(const AXI_LLC_Regs_t &regs) const;
  bool write_queue_full(const AXI_LLC_Regs_t &regs, uint8_t master) const;
  bool write_queue_empty(const AXI_LLC_Regs_t &regs, uint8_t master) const;
  const AXI_LLCWritePendingReq_t *write_queue_front(
      const AXI_LLC_Regs_t &regs, uint8_t master) const;
  int find_prefetch_queue_slot(const AXI_LLC_Regs_t &regs, uint32_t line_addr) const;
  int find_free_prefetch_queue_slot(const AXI_LLC_Regs_t &regs) const;
  int pick_prefetch_queue_slot(const AXI_LLC_Regs_t &regs) const;
  bool prefetch_candidate_exists(const AXI_LLC_Regs_t &regs, uint32_t line_addr) const;
  bool has_pending_upstream_write_line(uint32_t line_addr) const;
  bool can_allocate_prefetch_mshr(const AXI_LLC_Regs_t &regs) const;
  bool write_line_pending(const AXI_LLC_Regs_t &regs, uint32_t line_addr) const;
  bool read_victim_snapshot_present(const AXI_LLC_Regs_t &regs,
                                    uint32_t victim_addr) const;
  bool victim_snapshot_waits_for_write_resolution(const AXI_LLC_Regs_t &regs,
                                                  uint32_t victim_addr) const;
  bool victim_line_pending(const AXI_LLC_Regs_t &regs, uint32_t line_addr) const;
  bool way_reserved_by_pending_write(const AXI_LLC_Regs_t &regs, uint32_t set,
                                     uint8_t way,
                                     uint32_t line_addr_value) const;
  bool way_reserved_by_pending_refill(const AXI_LLC_Regs_t &regs, uint32_t set,
                                      uint8_t way,
                                      uint32_t line_addr_value) const;
  int pick_unreserved_way(const AXI_LLC_Regs_t &regs, uint32_t set,
                          int first_invalid_way, uint32_t repl_way_raw,
                          uint32_t line_addr_value) const;
  bool can_accept_invalidate_line_now(uint32_t line_addr) const;
  bool has_dirty_or_write_hazard(const AXI_LLC_Regs_t &regs) const;
  bool can_accept_invalidate_all_now(const AXI_LLC_Regs_t &regs) const;
  bool direct_mapped_coords(uint32_t addr, uint32_t *set,
                            uint8_t *way) const;
  bool line_has_valid_meta(const AXI_LLC_Bytes_t &valid_payload,
                           const AXI_LLC_Bytes_t &meta_payload, uint32_t tag,
                           int *hit_way, int *first_invalid_way,
                           AXI_LLCMetaEntry_t *hit_meta) const;
  bool enqueue_read_response(uint8_t master, uint8_t id,
                             const WideReadData_t &data);
  bool read_victim_wb_queue_full(const AXI_LLC_Regs_t &regs) const;
  bool read_victim_snapshot_queued(const AXI_LLC_Regs_t &regs, uint8_t owner_slot,
                                   uint32_t victim_addr) const;
  bool enqueue_read_victim_snapshot(uint8_t owner_slot, uint32_t victim_addr,
                                    const WideWriteStrb_t &victim_strobe);
  void try_schedule_prefetch(const AXI_LLCMissEntry_t &entry);
  void try_launch_prefetch_lookup();
  bool try_launch_pending_write_lookup();

  void drive_read_responses();
  void drive_write_path();
  void drive_lookup_request();
  bool try_complete_lookup();
  void drive_mem_read_path();
  void accept_new_requests();
  void accept_maintenance_request();
  void comb_disabled();

  AXI_LLCConfig config_{};
};

} // namespace axi_interconnect

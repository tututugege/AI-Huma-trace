#pragma once

#include <stdbool.h>
#include <stdint.h>

#define AXI_DUAL_PORT_MMIO_TOTAL_SIZE 3u
#define AXI_DUAL_PORT_AXI_SIZE_32B 2u
#define AXI_DUAL_PORT_AXI_SIZE_256B 5u
#define AXI_DUAL_PORT_DDR_BEAT_BYTES 32u
#define AXI_DUAL_PORT_AXI_ID_MASK_FOR_WIDTH(width) \
  ((uint32_t)((width) >= 8u ? 0xFFu : ((1u << (width)) - 1u)))

typedef struct AxiDualPortRouteShape {
  bool ddr_port;
  bool mmio_port;
  bool supported;
  uint8_t axi_len;
  uint8_t axi_size;
} AxiDualPortRouteShape;

typedef struct AxiDualPortRouteSupport {
  bool ddr_port;
  bool mmio_port;
  bool supported;
} AxiDualPortRouteSupport;

typedef struct AxiDualPortAxiBeatShape {
  uint8_t total_beats;
  uint8_t axi_len;
  uint8_t axi_size;
} AxiDualPortAxiBeatShape;

typedef struct AxiDualPortIssueGateResult {
  bool bridge_arready;
  bool axi_arvalid;
  bool bridge_awready;
  bool axi_awvalid;
  bool ar_hazard;
  bool ar_would_issue;
  bool aw_same_cycle_read_hazard;
  bool aw_hazard;
  bool ar_fire;
  bool aw_fire;
} AxiDualPortIssueGateResult;

typedef struct AxiDualPortReqSteerResult {
  bool ddr_req_valid;
  bool mmio_req_valid;
  bool req_ready;
} AxiDualPortReqSteerResult;

typedef struct AxiDualPortRespMuxControl {
  bool ddr_resp_ready;
  bool mmio_resp_ready;
  bool resp_valid;
  bool select_mmio;
} AxiDualPortRespMuxControl;

typedef struct AxiBridgeRespAcceptControl {
  bool axi_rready;
  bool rd_resp_accept;
  bool axi_bready;
  bool wr_resp_accept;
} AxiBridgeRespAcceptControl;

typedef struct AxiBridgeReqAcceptControl {
  bool accept_cache;
  bool accept_bypass;
  bool accept_write;
  uint8_t accept_slot;
  uint8_t accept_axi_id;
  uint8_t accept_total_beats;
} AxiBridgeReqAcceptControl;

typedef struct AxiBridgePendingScanResult {
  bool free_found;
  uint8_t free_slot;
  bool axi_id_found;
  uint8_t axi_id;
  bool match_found;
  uint8_t match_slot;
  bool complete_found;
  uint8_t complete_slot;
} AxiBridgePendingScanResult;

typedef struct AxiBridgeIssueSelectControl {
  bool issue_valid;
  bool issue_mode2_ddr_aligned;
  uint32_t issue_addr;
  uint8_t issue_size;
  uint8_t issue_axi_id;
  uint8_t issue_beat_idx;
  uint8_t issue_total_beats;
} AxiBridgeIssueSelectControl;

typedef struct AxiBridgeMode2Shape {
  bool single_axi_beat;
  uint32_t issue_addr;
  uint8_t issue_size;
} AxiBridgeMode2Shape;

typedef struct AxiBridgeDownstreamIssueShape {
  uint32_t issue_addr;
  uint8_t issue_size;
  bool extract_from_aligned_beat;
  uint8_t total_beats;
  uint8_t axi_len;
  uint8_t axi_size;
} AxiBridgeDownstreamIssueShape;

typedef struct AxiBridgeSourceRespMuxControl {
  bool resp_valid;
  bool select_read;
  bool rd_pop;
  bool wr_pop;
} AxiBridgeSourceRespMuxControl;

typedef struct AxiBridgeRespRouteControl {
  bool rd_complete_rsp_space;
  bool rd_complete_push;
  bool cache_rd_rsp_push;
  bool bypass_rd_rsp_push;
  bool wr_match_rsp_space;
  bool cache_wr_rsp_push;
  bool bypass_wr_rsp_push;
} AxiBridgeRespRouteControl;

typedef struct AxiBridgeFifoPtrControl {
  uint8_t next_head;
  uint8_t next_tail;
  uint8_t next_count;
} AxiBridgeFifoPtrControl;

typedef struct AxiBridgeWritePack64 {
  uint64_t axi_wdata;
  uint32_t axi_wstrb;
} AxiBridgeWritePack64;

typedef struct AxiBridgeReadPack64 {
  uint64_t merged_data;
  uint64_t final_data;
} AxiBridgeReadPack64;

typedef struct AxiBridgeReadRespControl {
  bool rd_last_beat;
  uint8_t next_resp_code;
} AxiBridgeReadRespControl;

typedef struct AxiBridgeQueueControl {
  bool rd_issue_space;
  bool wr_aw_space;
  bool wr_w_space;
  bool cache_rd_rsp_valid;
  bool bypass_rd_rsp_valid;
  bool cache_wr_rsp_valid;
  bool bypass_wr_rsp_valid;
  bool cache_rd_rsp_space;
  bool bypass_rd_rsp_space;
  bool cache_wr_rsp_space;
  bool bypass_wr_rsp_space;
  bool rd_issue_handshake;
  bool wr_aw_handshake;
  bool wr_w_handshake;
  bool rd_issue_push;
  bool rd_issue_pop;
  bool wr_aw_push;
  bool wr_aw_pop;
  bool wr_w_push;
  bool wr_w_pop;
} AxiBridgeQueueControl;

typedef struct AxiDualPortSlotHazardResult {
  bool primary_slot_hazard;
  bool secondary_slot_hazard;
} AxiDualPortSlotHazardResult;

typedef struct AxiDualPortHazardMatchResult {
  bool ddr_line_match;
  bool mmio_line_match;
  bool ddr_id_match;
  bool mmio_id_match;
} AxiDualPortHazardMatchResult;

static inline bool axi_dual_port_is_ddr_addr(uint32_t addr,
                                             uint32_t ddr_base) {
  return addr >= ddr_base;
}

static inline bool axi_dual_port_mmio_request_supported(uint8_t total_size) {
  return total_size == AXI_DUAL_PORT_MMIO_TOTAL_SIZE;
}

static inline uint8_t axi_dual_port_axi_size_for_beat_bytes(uint8_t beat_bytes) {
  if (beat_bytes == 32u) {
    return 5u;
  }
  if (beat_bytes == 16u) {
    return 4u;
  }
  if (beat_bytes == 8u) {
    return 3u;
  }
  return 2u;
}

static inline uint8_t
axi_dual_port_axi_len_for_beat_bytes(uint8_t total_size, uint8_t beat_bytes) {
  if (beat_bytes == 0u) {
    return 0u;
  }
  const uint32_t bytes = (uint32_t)total_size + 1u;
  const uint32_t beats = (bytes + (uint32_t)beat_bytes - 1u) /
                         (uint32_t)beat_bytes;
  return beats > 0u ? (uint8_t)(beats - 1u) : 0u;
}

static inline AxiDualPortAxiBeatShape
axi_dual_port_axi_beat_shape(uint8_t total_size, uint8_t beat_bytes) {
  AxiDualPortAxiBeatShape out;
  out.axi_len = axi_dual_port_axi_len_for_beat_bytes(total_size, beat_bytes);
  out.total_beats = (uint8_t)(out.axi_len + 1u);
  out.axi_size = axi_dual_port_axi_size_for_beat_bytes(beat_bytes);
  return out;
}

static inline AxiDualPortRouteSupport
axi_dual_port_route_support(uint32_t addr, uint8_t total_size,
                            uint32_t ddr_base) {
  AxiDualPortRouteSupport out;
  out.ddr_port = axi_dual_port_is_ddr_addr(addr, ddr_base);
  out.mmio_port = !out.ddr_port;
  out.supported =
      out.ddr_port || axi_dual_port_mmio_request_supported(total_size);
  return out;
}





static inline AxiDualPortIssueGateResult axi_dual_port_issue_gate(
    bool bridge_arvalid, bool axi_arready, uint32_t ar_line,
    bool ar_slot_hazard, bool ar_pending_write_hazard, bool bridge_awvalid,
    bool axi_awready, uint32_t aw_line, bool aw_slot_hazard,
    bool aw_pending_read_hazard) {
  AxiDualPortIssueGateResult out;
  out.ar_hazard = ar_slot_hazard || ar_pending_write_hazard;
  out.ar_would_issue = bridge_arvalid && !out.ar_hazard;
  out.aw_same_cycle_read_hazard = out.ar_would_issue && (ar_line == aw_line);
  out.aw_hazard =
      aw_slot_hazard || aw_pending_read_hazard ||
      out.aw_same_cycle_read_hazard;
  out.bridge_arready = axi_arready && !out.ar_hazard;
  out.axi_arvalid = bridge_arvalid && !out.ar_hazard;
  out.bridge_awready = axi_awready && !out.aw_hazard;
  out.axi_awvalid = bridge_awvalid && !out.aw_hazard;
  out.ar_fire = out.axi_arvalid && axi_arready;
  out.aw_fire = out.axi_awvalid && axi_awready;
  return out;
}







static inline uint32_t axi_bridge_align_down_addr(uint32_t addr,
                                                  uint8_t align_bytes) {
  if (align_bytes <= 1u) {
    return addr;
  }
  return (addr / (uint32_t)align_bytes) * (uint32_t)align_bytes;
}

static inline bool axi_bridge_mode2_single_axi_beat(uint32_t addr,
                                                    uint8_t total_size,
                                                    uint8_t axi_data_bytes) {
  const uint16_t req_bytes = (uint16_t)total_size + 1u;
  const uint32_t beat_addr =
      axi_bridge_align_down_addr(addr, axi_data_bytes);
  const uint32_t end_byte = (addr - beat_addr) + (uint32_t)req_bytes;
  return (req_bytes <= (uint16_t)axi_data_bytes) &&
         (end_byte <= (uint32_t)axi_data_bytes);
}



static inline AxiBridgeMode2Shape axi_bridge_mode2_shape(
    uint32_t addr, uint8_t total_size, uint8_t line_bytes,
    uint8_t axi_data_bytes) {
  AxiBridgeMode2Shape out;
  out.single_axi_beat =
      axi_bridge_mode2_single_axi_beat(addr, total_size, axi_data_bytes);
  out.issue_addr = out.single_axi_beat
                       ? axi_bridge_align_down_addr(addr, axi_data_bytes)
                       : axi_bridge_align_down_addr(addr, line_bytes);
  out.issue_size =
      out.single_axi_beat ? (uint8_t)(axi_data_bytes - 1u)
                          : (uint8_t)(line_bytes - 1u);
  return out;
}

static inline AxiBridgeDownstreamIssueShape
axi_bridge_downstream_read_issue_shape(bool mmio_port, uint32_t addr,
                                       uint8_t total_size,
                                       uint8_t line_bytes,
                                       uint8_t axi_data_bytes,
                                       bool force_ddr_aligned) {
  AxiBridgeDownstreamIssueShape out;
  out.issue_addr = addr;
  out.issue_size = total_size;
  out.extract_from_aligned_beat = false;

  if (mmio_port) {
    out.issue_size = AXI_DUAL_PORT_MMIO_TOTAL_SIZE;
  } else if (force_ddr_aligned) {
    const AxiBridgeMode2Shape mode2 =
        axi_bridge_mode2_shape(addr, total_size, line_bytes, axi_data_bytes);
    out.issue_addr = mode2.issue_addr;
    out.issue_size = mode2.issue_size;
    out.extract_from_aligned_beat =
        mode2.single_axi_beat || (out.issue_addr != addr);
  } else if (((uint16_t)total_size + 1u) <= (uint16_t)axi_data_bytes) {
    out.issue_addr = axi_bridge_align_down_addr(addr, axi_data_bytes);
    out.issue_size = (uint8_t)(axi_data_bytes - 1u);
    out.extract_from_aligned_beat = true;
  }

  const AxiDualPortAxiBeatShape beat =
      axi_dual_port_axi_beat_shape(out.issue_size,
                                   mmio_port ? 4u : axi_data_bytes);
  out.total_beats = beat.total_beats;
  out.axi_len = beat.axi_len;
  out.axi_size = mmio_port ? AXI_DUAL_PORT_AXI_SIZE_32B : beat.axi_size;
  return out;
}

static inline AxiBridgeDownstreamIssueShape
axi_bridge_downstream_write_issue_shape(bool mmio_port, uint32_t addr,
                                        uint8_t total_size,
                                        uint8_t line_bytes,
                                        uint8_t axi_data_bytes,
                                        bool force_ddr_aligned) {
  AxiBridgeDownstreamIssueShape out =
      axi_bridge_downstream_read_issue_shape(mmio_port, addr, total_size,
                                             line_bytes, axi_data_bytes,
                                             force_ddr_aligned);
  out.extract_from_aligned_beat = false;
  return out;
}














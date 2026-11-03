#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sc_axi4_in_t {
  // AR channel (Slave -> Master): read-address accept
  uint8_t arready;

  // AW channel (Slave -> Master): write-address accept
  uint8_t awready;

  // W channel (Slave -> Master): write-data accept
  uint8_t wready;

  // R channel (Slave -> Master): read-data response
  uint8_t rvalid;
  uint8_t rid;
  uint32_t rdata;
  uint8_t rresp;
  uint8_t rlast;

  // B channel (Slave -> Master): write response
  uint8_t bvalid;
  uint8_t bid;
  uint8_t bresp;
} sc_axi4_in_t;

typedef struct sc_axi4_out_t {
  // AR channel (Master -> Slave): read-address request
  uint8_t arvalid;
  uint8_t arid;
  uint32_t araddr;
  uint8_t arlen;
  uint8_t arsize;
  uint8_t arburst;

  // AW channel (Master -> Slave): write-address request
  uint8_t awvalid;
  uint8_t awid;
  uint32_t awaddr;
  uint8_t awlen;
  uint8_t awsize;
  uint8_t awburst;

  // W channel (Master -> Slave): write-data request
  uint8_t wvalid;
  uint32_t wdata;
  uint8_t wstrb;
  uint8_t wlast;

  // R channel (Master -> Slave): read-data handshake
  uint8_t rready;

  // B channel (Master -> Slave): write-response handshake
  uint8_t bready;
} sc_axi4_out_t;

typedef struct sc_sim_status_t {
  uint64_t sim_time;
  uint64_t inst_count;
  uint8_t halted;
  uint8_t success;
  uint8_t wait_axi;
  uint8_t uart_valid;
  uint8_t uart_ch;
} sc_sim_status_t;

typedef struct sc_sim_handle sc_sim_handle;

sc_sim_handle *sc_sim_create(void);
void sc_sim_destroy(sc_sim_handle *handle);

int sc_sim_load_image(sc_sim_handle *handle, const char *image_path,
                      uint64_t *image_size_out);
void sc_sim_set_limits(sc_sim_handle *handle, uint64_t max_inst,
                       uint64_t max_cycles);

int sc_sim_step(sc_sim_handle *handle, const sc_axi4_in_t *axi_in,
                sc_axi4_out_t *axi_out, sc_sim_status_t *status_out);
void sc_sim_get_status(const sc_sim_handle *handle, sc_sim_status_t *status_out);

const char *sc_sim_last_error(const sc_sim_handle *handle);

#ifdef __cplusplus
}
#endif

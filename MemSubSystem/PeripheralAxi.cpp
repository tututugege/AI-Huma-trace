#include "PeripheralAxi.h"
#include "PeripheralModel.h"
#include "PhysMemory.h"
#include "config.h"
#include "mmio_address_map.h"

void PeripheralAxi::init() {   // (in / out are the port bindings, set by MemSubsystem::init)
  cur_ = {};
  nxt = {};
  if (out.resp != nullptr) {
    *out.resp = {};
  }
}

uint8_t PeripheralAxi::access_size_bytes(uint8_t func3) {
  switch (func3 & 0x3u) {
  case 0:
    return 1;
  case 1:
    return 2;
  default:
    return 4;
  }
}

uint32_t PeripheralAxi::pack_write_data(uint32_t data, uint8_t func3) {
  switch (func3 & 0x3u) {
  case 0:
    return data & 0xFFu;
  case 1:
    return data & 0xFFFFu;
  default:
    return data;
  }
}

uint64_t PeripheralAxi::pack_write_strb(uint8_t func3) {
  switch (func3 & 0x3u) {
  case 0:
    return 0x1ull;
  case 1:
    return 0x3ull;
  default:
    return 0xFull;
  }
}

uint32_t PeripheralAxi::extract_read_data(uint32_t raw, uint8_t func3) {
  switch (func3) {
  case 0b000: {
    uint32_t val = raw & 0xFFu;
    return (val & 0x80u) ? (val | 0xFFFFFF00u) : val;
  }
  case 0b001: {
    uint32_t val = raw & 0xFFFFu;
    return (val & 0x8000u) ? (val | 0xFFFF0000u) : val;
  }
  case 0b100:
    return raw & 0xFFu;
  case 0b101:
    return raw & 0xFFFFu;
  default:
    return raw;
  }
}

void PeripheralAxi::comb_outputs() {
  clear_master_fields(*out.read);
  clear_master_fields(*out.write);
  out.write->req.wdata.clear();
  out.read->req.bypass = out.write->req.bypass = true;   // MMIO: never through the LLC

  if (out.resp != nullptr) {
    *out.resp = {};
    out.resp->ready = !cur.busy && !cur.resp_valid;
    if (cur.resp_valid) {
      out.resp->is_mmio = 1;
      out.resp->mmio_rdata = cur.rdata;
    }
  }

  if (!cur.busy) {
    return;
  }

  if (!cur.req_accepted) {
    const uint8_t total_size = static_cast<uint8_t>(access_size_bytes(cur.func3) - 1u);
    if (cur.write) {
      out.write->req.valid = true;
      out.write->req.addr = cur.addr;
      out.write->req.total_size = total_size;
      out.write->req.id = cur.req_id;
      out.write->req.wstrb = pack_write_strb(cur.func3);
      out.write->req.wdata[0] = pack_write_data(cur.wdata, cur.func3);
    } else {
      out.read->req.valid = true;
      out.read->req.addr = cur.addr;
      out.read->req.total_size = total_size;
      out.read->req.id = cur.req_id;
    }
    return;
  }

  if (cur.write) {
    out.write->resp.ready = true;
  } else {
    out.read->resp.ready = true;
  }
}

void PeripheralAxi::comb_inputs() {
  nxt = cur;

  if (cur.resp_valid) {
    nxt.resp_valid = false;
    if (out.resp != nullptr) {
      out.resp->ready = false;
    }
    return;
  }

  if (!cur.busy) {
    if (in.req != nullptr && in.req->is_mmio) {
      if (peripheral_model != nullptr &&
          PeripheralModel::is_modeled_mmio(in.req->mmio_addr)) {
        nxt.busy = false;
        nxt.write = in.req->wen;
        nxt.req_accepted = false;
        nxt.resp_valid = true;
        nxt.addr = in.req->mmio_addr;
        nxt.wdata = in.req->mmio_wdata;
        nxt.func3 = in.req->mmio_fun3;
        nxt.rdata = in.req->wen
                        ? 0
                        : peripheral_model->read_load64(in.req->mmio_addr,
                                                        nxt.func3);
        nxt.req_id = 0;
        return;
      }
      nxt.busy = true;
      nxt.write = in.req->wen;
      nxt.req_accepted = false;
      nxt.resp_valid = false;
      nxt.addr = in.req->mmio_addr;
      nxt.wdata = in.req->mmio_wdata;
      nxt.func3 = in.req->mmio_fun3;
      nxt.rdata = 0;
      nxt.req_id = 0;
    }
    return;
  }

  if (!cur.req_accepted) {
    if ((cur.write && in.write->req.accepted) ||
        (!cur.write && in.read->req.accepted)) {
      nxt.req_accepted = true;
    }
    return;
  }

  if (cur.write) {
    if (in.write->resp.valid) {
      nxt.busy = false;
      nxt.write = false;
      nxt.req_accepted = false;
      nxt.resp_valid = true;
      nxt.rdata = 0;
    }
    return;
  }

  if (in.read->resp.valid) {
    nxt.busy = false;
    nxt.write = false;
    nxt.req_accepted = false;
    nxt.resp_valid = true;
    nxt.rdata = extract_read_data(in.read->resp.data[0], cur.func3);
  }

  if (out.resp != nullptr) {
    out.resp->ready = !nxt.busy && !nxt.resp_valid;
  }
}

void PeripheralAxi::seq() {
  cur_ = nxt;
  nxt = cur;
}

void PeripheralAxi::comb() {
  comb_outputs();
  comb_inputs();
}

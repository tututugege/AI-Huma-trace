#include "PeripheralAxi.h"
#include "PeripheralModel.h"
#include "PhysMemory.h"
#include "config.h"
#include "mmio_address_map.h"

void PeripheralAxi::init() {   // (in / out are the port bindings, set by MemSubsystem::init)
  q_ = {};
  d = {};
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
    const Regs &q = q_;
  clear_master_fields(*out.read);
  clear_master_fields(*out.write);
  out.write->req.wdata.clear();
  out.read->req.bypass = out.write->req.bypass = true;   // MMIO: never through the LLC

  if (out.resp != nullptr) {
    out.resp->is_mmio = false;
    out.resp->ready = !q.busy && !q.resp_valid;
    if (q.resp_valid) {
      out.resp->is_mmio = 1;
      out.resp->mmio_rdata = q.rdata;
    }
  }

  if (!q.busy) {
    return;
  }

  if (!q.req_accepted) {
    const uint8_t total_size = static_cast<uint8_t>(access_size_bytes(q.func3) - 1u);
    if (q.write) {
      out.write->req.valid = true;
      out.write->req.addr = q.addr;
      out.write->req.total_size = total_size;
      out.write->req.id = q.req_id;
      out.write->req.wstrb = pack_write_strb(q.func3);
      out.write->req.wdata[0] = pack_write_data(q.wdata, q.func3);
    } else {
      out.read->req.valid = true;
      out.read->req.addr = q.addr;
      out.read->req.total_size = total_size;
      out.read->req.id = q.req_id;
    }
    return;
  }

  if (q.write) {
    out.write->resp.ready = true;
  } else {
    out.read->resp.ready = true;
  }
}

void PeripheralAxi::comb_inputs() {
    const Regs &q = q_;
  d = q;

  if (q.resp_valid) {
    d.resp_valid = false;
    if (out.resp != nullptr) {
      out.resp->ready = false;
    }
    return;
  }

  if (!q.busy) {
    if (in.req != nullptr && in.req->is_mmio) {
      if (peripheral_model != nullptr &&
          PeripheralModel::is_modeled_mmio(in.req->mmio_addr)) {
        d.busy = false;
        d.write = in.req->wen;
        d.req_accepted = false;
        d.resp_valid = true;
        d.addr = in.req->mmio_addr;
        d.wdata = in.req->mmio_wdata;
        d.func3 = in.req->mmio_fun3;
        d.rdata = in.req->wen
                        ? 0
                        : peripheral_model->read_load64(in.req->mmio_addr,
                                                        d.func3);
        d.req_id = 0;
        return;
      }
      d.busy = true;
      d.write = in.req->wen;
      d.req_accepted = false;
      d.resp_valid = false;
      d.addr = in.req->mmio_addr;
      d.wdata = in.req->mmio_wdata;
      d.func3 = in.req->mmio_fun3;
      d.rdata = 0;
      d.req_id = 0;
    }
    return;
  }

  if (!q.req_accepted) {
    if ((q.write && in.write->req.accepted) ||
        (!q.write && in.read->req.accepted)) {
      d.req_accepted = true;
    }
    return;
  }

  if (q.write) {
    if (in.write->resp.valid) {
      d.busy = false;
      d.write = false;
      d.req_accepted = false;
      d.resp_valid = true;
      d.rdata = 0;
    }
    return;
  }

  if (in.read->resp.valid) {
    d.busy = false;
    d.write = false;
    d.req_accepted = false;
    d.resp_valid = true;
    d.rdata = extract_read_data(in.read->resp.data[0], q.func3);
  }

  if (out.resp != nullptr) {
    out.resp->ready = !d.busy && !d.resp_valid;
  }
}

void PeripheralAxi::seq() {
  q_ = d;
}

void PeripheralAxi::comb_commit_store() {
  const CommitStoreIO &cs = *in.commit_store;
  for (int i = 0; i < COMMIT_WIDTH; i++) {
    if (cs.slot[i].valid) peripheral_model->on_commit_store(cs.slot[i].paddr, cs.slot[i].data, cs.slot[i].func3);
  }
}

void PeripheralAxi::comb() {
  comb_outputs();
  comb_inputs();
}

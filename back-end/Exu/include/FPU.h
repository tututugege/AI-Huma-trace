#pragma once
#include "AbstractFU.h"  
#include "IO.h"
#include "config.h"
#include <cassert>
#include <climits>



// ==========================================
// FPUSoftfloat
// ==========================================
// ==========================================
// FPU timing model (P3: the softfloat datapath and the generated FADD/FMUL RTL-as-C++ are removed;
// the FP result is a fact of the trace, only the latency by operation type is modelled)
// ==========================================
class FPUSoftfloat : public IterativeFU {
    static constexpr int FADD = 0b00000;
    static constexpr int FSUB = 0b00001;
    static constexpr int FMUL = 0b00010;
    static constexpr int FDIV = 0b00011;
    static constexpr int FSQRT = 0x2C;
    static constexpr int FCVT_W_S = 0x60;
    static constexpr int FCVT_S_W = 0x68;
    static constexpr int LAT_FADD = 5;
    static constexpr int LAT_FMUL = 3;
    static constexpr int LAT_FDIV = 10;
    static constexpr int LAT_FCVT = 3;

public:
  FPUSoftfloat(std::string name = "FPUSoftfloat", int port_idx = 0, int max_lat = LAT_FDIV)
      : IterativeFU(name, port_idx, max_lat) {}

protected:
  void impl_compute(ExuInst &inst) override { (void)inst; }

  int calculate_latency(const ExuInst &inst) override {
    if (inst.op != UOP_FP) {
      return LAT_FADD;
    }
    switch (inst.func7 >> 2) {
    case FMUL:
      return LAT_FMUL;
    case FDIV:
      return LAT_FDIV;
    case FADD:
    case FSUB:
      return LAT_FADD;
    default:
      if (inst.func7 == FCVT_W_S || inst.func7 == FCVT_S_W) {
        return LAT_FCVT;
      }
      if (inst.func7 == FSQRT) {
        return LAT_FDIV;
      }
      return LAT_FADD;
    }
  }
};

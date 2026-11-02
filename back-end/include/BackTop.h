#pragma once
#include "Csr.h"
#include "Dispatch.h"
#include "Exu.h"
#include "FTQ.h"
#include "IO.h"
#include "Wires.h"
#include "Idu.h"
#include "Isu.h"
#include "PreIduQueue.h"
#include "Prf.h"
#include "Ren.h"
#include "RealLsu.h"
#include "Rob.h"
#include "TlbMmu.h"
#include "config.h"
#include <cstdint>
#include <memory>

class SimContext;
class PtwMemPort;
class PtwWalkPort;

class Dispatch;
class Idu;
class Ren;
class Isu;
class Prf;
class Exu;
class Rob;
class Csr;
class AbstractMMU;
class MemSubsystem;

class BackTop : public Wires {
private:
  std::unique_ptr<TlbMmu> dtlb_mmu;
  void comb_lsu_mmu();

public:
  SimContext *ctx;

  PreIduQueue *pre;
  Idu *idu;
  Ren *rename;
  Dispatch *dis;
  Isu *isu;
  Prf *prf;
  Exu *exu;
  Csr *csr;
  Rob *rob;
  RealLsu *lsu;

  void init();
  void clear_comb_inputs();
  void comb_csr_status();
  void comb();
  void comb_begin_all();
  void comb_stage_decode_and_resources();
  void comb_stage_commit();
  void comb_stage_memory_responses();
  void comb_stage_execute();
  void comb_stage_issue_and_translate();
  void comb_stage_operands_and_dispatch();
  void comb_stage_fire_and_recover();
  void seq();
  void set_lsu_ptw_mem_port(PtwMemPort *port);
  void set_lsu_ptw_walk_port(PtwWalkPort *port);

  BackTop(SimContext *ctx) {
    this->ctx = ctx;
    pre = nullptr;
    out.fire = nullptr;
  };
  ~BackTop() {
    delete pre;
    delete rename;
    delete dis;
    delete idu;
    delete isu;
    delete exu;
    delete prf;
    delete rob;
    delete csr;
    delete lsu;
  }

  uint32_t number_PC = 0;
  uint64_t ckpt_interval_inst_count = 0;

  void load_image(const std::string &filename);
  void load_flash_image(const std::string &filename);
  void load_sd_boot_blob(const std::string &sdcard_image,
                         uint32_t sector_offset = 64u);
  void restore_checkpoint(const std::string &filename);
  void restore_from_ref();

  uint32_t get_reg(uint8_t arch_idx) {
    (void)arch_idx;
    return 0;  // no register file: architectural values live in the trace bridge
  }
};

#pragma once
// Every signal that crosses a module boundary inside the back-end (and the top-level in/out), in one place.
// Modules read and write them through a Wires pointer; BackTop owns the Wires (it derives from it) and
// evaluates the modules in dependency order, so a wire is valid once the stage that drives it has run.
#include "IO.h"

struct Back_in : FrontPreIO {
  // from MemSubsystem
  PeripheralRespIO peripheral_resp;
  DcacheLsuIO dcache2lsu;

  Back_in() : FrontPreIO(), peripheral_resp{}, dcache2lsu{} {}
};

struct Back_out {
  // to front-end
  bool mispred;
  bool stall;
  bool flush;
  bool fence_i;
  bool itlb_flush;
  wire<1> *fire;
  uint32_t redirect_pc;
  bool rf_valid = false;               // BPA_REPAIR: the redirect is a branch mispredict; the block / slot / type of the branch
  uint16_t rf_blk = 0;
  uint8_t rf_slot = 0, rf_type = 0;
  uint32_t rf_pc = 0;
  InstEntry commit_entry[COMMIT_WIDTH];

  wire<32> sstatus;
  wire<32> mstatus;
  wire<32> satp;
  wire<2> privilege;

  // to MemSubsystem
  PeripheralReqIO peripheral_req;
  LsuDcacheIO lsu2dcache;

  Back_out()
      : mispred(false), stall(false), flush(false), fence_i(false),
        itlb_flush(false), fire(nullptr), redirect_pc(0), commit_entry{},
        sstatus{}, mstatus{}, satp{}, privilege{}, peripheral_req{},
        lsu2dcache{} {}
};

struct Wires {
  FrontPreIO front2pre;
  PreFrontIO pre2front;
  DecRenIO dec2ren;
  DecBroadcastIO dec_bcast;
  RenDecIO ren2dec;
  RenDisIO ren2dis;
  DisRenIO dis2ren;
  DisIssIO dis2iss;
  DisRobIO dis2rob;
  DisLsuIO dis2lsu;
  IssAwakeIO iss_awake;
  IssPrfIO iss2prf;
  IssDisIO iss2dis;
  PrfExeIO prf2exe;
  PrfAwakeIO prf_awake;
  ExuIdIO exu2id;
  ExuRobIO exu2rob;
  FtqPrfPcReqIO ftq_prf_pc_req;
  FtqPrfPcRespIO ftq_prf_pc_resp;
  ExePrfIO exe2prf;
  ExeIssIO exe2iss;
  ExeLsuIO exe2lsu;
  LsuExeIO lsu2exe;
  LsuDisIO lsu2dis;
  LsuRobIO lsu2rob;
  LsuMMUIO lsu2mmu_io;
  MMULsuIO mmu2lsu_io;
  RobDisIO rob2dis;
  RobCsrIO rob2csr;
  RobBroadcastIO rob_bcast;
  RobCommitIO rob_commit;
  FtqRobPcReqIO ftq_rob_pc_req;
  FtqRobPcRespIO ftq_rob_pc_resp;
  CsrExeIO csr2exe;
  CsrRobIO csr2rob;
  CsrFrontIO csr2front;
  CsrStatusIO csr_status;
  ExeCsrIO exe2csr;
  PreIssueIO pre_issue;
  IduConsumeIO idu_consume;
  FtqCommitInfoIO ftq_commit_info;
  CsrInterruptInjectIO csr_interrupt_inject_io;
  Back_in in;
  Back_out out;
  const ExuIdIO *idu_br_latch = nullptr;  // owned by Idu
};

#pragma once
// The back end's interface and BackTop's own signals. Every other signal between the back-end modules is an output of
// its module (struct Out, the module alone writes it); a module reads its inputs through const pointers (struct In),
// all bound in BackTop::init.
#include "IO.h"

struct BackIn : FrontPreIO {
  // from MemSubsystem
  DcacheLsuIO dcache2lsu;

  BackIn() : FrontPreIO(), dcache2lsu{} {}
};

struct BackOut {
  // to front-end
  bool mispred;
  bool stall;
  bool flush;
  bool fence_i;
  bool itlb_flush;
  wire<1> fire[BACKEND_FETCH_WIDTH] = {};   // the fetch-group slots the back end took this cycle
  wire<XLEN> redirect_pc;
  bool rf_valid = false;               // BPA_REPAIR: the redirect is a branch mispredict; the block / slot / type of the branch
  uint16_t rf_blk = 0;
  uint8_t rf_slot = 0, rf_type = 0;
  wire<XLEN> rf_pc = 0;
  wire<1> rf_is_rvc = 0;
  InstEntry commit_entry[COMMIT_WIDTH];
  RobCommitExtra commit_extra[COMMIT_WIDTH];   // BPU training of the committing instructions (from the ROB, the FTQ)
  FtqCommitInfoResp commit_ftq[COMMIT_WIDTH];

  wire<32> sstatus;
  wire<32> mstatus;
  wire<XLEN> satp;
  wire<2> privilege;

  // to MemSubsystem
  PeripheralReqIO peripheral_req;
  LsuDcacheIO lsu2dcache;
  CommitStoreIO commit_store;   // the device stores of this cycle's commit (the peripheral model applies them)

  BackOut()
      : mispred(false), stall(false), flush(false), fence_i(false),
        itlb_flush(false), redirect_pc(0), commit_entry{},
        sstatus{}, mstatus{}, satp{}, privilege{}, peripheral_req{},
        lsu2dcache{} {}
};

struct BackPorts {
  TopCsrIO top2csr;                          // BackTop -> CSR
  CsrStatusIO csr_status;                    // CSR -> the front end, the back end (the CSR drives it)
  CsrInterruptInjectIO csr_interrupt_inject_io;   // the interrupt sources (memory subsystem) -> CSR
  BackIn in;                                 // from SimCpu / the front end / the memory subsystem
  BackOut out;                               // to SimCpu / the front end / the memory subsystem
};

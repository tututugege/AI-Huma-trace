# One form for every module (exp/global-sched, u2..u4)

Every comb module of the CPU is written the same way, and one tool checks and schedules all of them.

## The form

```cpp
// <what the module is>
//   CONTRACT comb_x  in: in.a in.b.f  out: out.c
//   CONTRACT comb_y  in: in.d  out: out.c out.e
class M {
public:
  struct In  { const A *a = nullptr; const B *b = nullptr; const D *d = nullptr; } in;   // what it reads
  struct Out { C c{}; E *e = nullptr; } out;    // what it drives: its own fields, or pointers to a signal its parent owns
  void comb_x();
  void comb_y();
  void seq();
};
```

* Ports carry the names of the wires they connect to. A read of another module's registers goes through a plain
  pointer member (`Rob::ftq`, `FrontControl::bpu_pipe`), not a port.
* The parent binds every port, one line each: `rob->in.csr2rob = &csr->out.csr2rob;` (BackTop::init),
  `mshr_.in.s1_q = &dcache_.s1_q();` (MemSubsystem::init), `checker.in.multi = &inst_port.out.multi;` (FrontCore),
  `f2b.in.front_out = &front.out;` (SimCpu::init).
* CONTRACT lines name the module's own ports, `in.<port>` / `out.<port>`, or a field of a port that points at a top's
  whole interface struct (`in.top_in.front_stall`, `out.top_out.flush`). A phase's lines list the ports it reads and
  writes. A read of its own owned output field is internal and is not listed.
* A module's phases run in the order its header lists them.

## The modules (26: back end 9, front end 8, memory 6, SimCpu 3)

| top | modules |
| --- | --- |
| back end (BackTop::init) | pre, rename, idu, lsu, csr, rob, dis, exu, isu |
| front end (FrontCore) | ctrl, bpu_pipe, fa_port, ptab_port, inst_port, f2b_port, icache, checker |
| memory (MemSubsystem::init) | wb_, mshr_, dcache_, mem_route_block, ptw_block, peripheral_axi_ |
| SimCpu::init | b2f (Back2Front: the register and the front end's inputs), f2b (Front2Back), axi (AxiSubsystem) |

What moved into modules to get there:
* the D-cache owns its stage wires and registers (the array read / write and the routed response in comb_stage2, the
  stage-1 requests and the response metadata as its registers; the MSHR and the write buffer read `dcache_.s1_q()`);
* MemPtwBlock::comb_request / comb_feedback (MemSubsystem's ptw_request / ptw_feedback);
* PeripheralAxi::comb_commit_store (MemSubsystem's comb_commit_store);
* Back2Front::comb_front (FrontTop::comb_begin); the front end's register snapshot (read_data) is taken at the edge.
MemSubsystem, BackTop, FrontTop and FrontCore have no comb code of their own: they own their modules and bind them.

## The tools (scripts/comb)

* `modules.py`: the module table, each class's CONTRACT lines, the parents' bindings (a port resolves to a SimCpu path,
  through pointer members bound in SimCpu::init; an AXI master index is kept), and the scan of every phase (its own
  method and the methods of its class it calls: `in.X` / `out.X` reads and writes, aliases, calls with non-const
  reference parameters, memcpy direction). `--check` compares the scan with the contracts; without it, it prints the
  scanned lines.
* `gen_top_order.py`: the only generator. Every module phase is a node; a writer of a path runs before its readers;
  ties: stay with the previous phase's top, then the top priority. It writes SimCpu::cycle() and FrontCore::comb().
* `comb_graph.py --check`: modules.py, the cycle() hazard scan as the code runs (comb_top.py: no phase reads a signal a
  later phase writes), and the generator's check.
Removed: gen_comb_order.py, gen_front_order.py, gen_mem_order.py, comb_order.py, global_graph.py, the front / memory
domains of comb_domains.py.

## The last pieces (u5)

* The AXI subsystem is a module (`axi`): its ports are the masters' ports of the interconnect (bound by SimCpu::init);
  comb_begin is a `default:` writer of the requests' valid, comb_outputs a `pre:` reader of the memory masters'
  requests (it reads the I-cache's of this cycle: `in: out.read_ports[MASTER_ICACHE].req`). The interconnect's code
  (AXI_Interconnect) is scanned as its submodule.
* The I-cache's AXI read port, its D-cache probe and the response registers are its In / Out ports (`out.axi`,
  `out.probe_req`, `in.axi_q`, `in.probe_resp`, `in.csr_status`); ICacheTop::Ports and FrontTop's icache_* pointers are
  gone.
* The TLBs' walk ports are the LSU's `out.dtlb_walk` and the I-cache's `out.itlb_walk`; their TLB (TlbMmu) is a
  submodule: the scan follows the parent's calls into it. A walk port's method names the part it touches (requests:
  comb_input, flushes: seq_input, the walker's response: comb_out, a register).
* The peripheral model is a submodule of PeripheralAxi, whose comb_commit_store applies the commit's device stores.
* Contract kinds: `in:`, `out:`, `pre:` (read before the other writers), `default:` (written before every other user);
  a port array may be named with a master index. The include/SimCpu.h PORT table, the EXPORT lines and the
  generator's extras are gone: every comb phase of SimCpu::cycle() is a module phase.

## Verified

Each step: gates RVC=0 62/62, RVC=1 67/67; u2 and u4 also FPR x4, TB / LSU prints, the Linux checkpoint, FE_PAIR /
FE_IPF (vbin.sh). The generated order is the previous one except the commit's device stores, which now run just before
their only reader (peripheral_axi_).

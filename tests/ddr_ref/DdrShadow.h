// Lockstep hook: the frozen original SimDDR runs next to the compact one; outputs and a state digest must match.
#pragma once
#include "SimDDR.h"
void ddr_shadow_begin(sim_ddr::SimDDR *d, sim_ddr::SimDDR_IO_t *io);
void ddr_shadow_end(sim_ddr::SimDDR *d, sim_ddr::SimDDR_IO_t *io, int phase);

#pragma once

#include "IO.h"

using ExuInst = Uop;

struct ExuEntry {
  wire<1> valid;
  ExuInst uop;
};

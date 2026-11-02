#pragma once
// Lockstep verification hooks. A normal build calls each module phase directly; a build with <MOD>_SHADOW=1 also runs the
// phase on the frozen original of that module and compares outputs and state (see tests/<mod>_ref).
#ifdef LSU_SHADOW
#include "LsuShadow.h"
#else
#define LSU(f) lsu->f()
#endif
#ifdef ROB_SHADOW
#include "RobShadow.h"
#else
#define ROB(f) rob->f()
#endif
#ifdef IDU_SHADOW
#include "IduShadow.h"
#else
#define IDU(f) idu->f()
#endif
#ifdef PRE_SHADOW
#include "PreShadow.h"
#else
#define PRE(f) pre->f()
#endif
#ifdef CSR_SHADOW
#include "CsrShadow.h"
#else
#define CSR(f) csr->f()
#endif
#ifdef REN_SHADOW
#include "RenShadow.h"
#else
#define REN(f) rename->f()
#endif
#ifdef ISU_SHADOW
#include "IsuShadow.h"
#else
#define ISU(f) isu->f()
#endif
#ifdef EXU_SHADOW
#include "ExuShadow.h"
#else
#define EXU(f) exu->f()
#endif
#ifdef DIS_SHADOW
#include "DisShadow.h"
#else
#define DIS(f) dis->f()
#endif

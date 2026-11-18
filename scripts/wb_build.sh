#!/usr/bin/env bash
# EXP-045 build helper: RVC=1 simulator against the private function-model main lib.  usage: wb_build.sh <build_dir> [extra cxxflags]
FM=${FM:-/Users/ruyaoh/personal/AI-Huma/fm-wbport-rvc1}
cd "$(dirname "$0")/.." && gmake -j4 BUILD_DIR="$1" RVC=1 LDFLAGS=-lz REFCPU_LIB=$FM/librefcpu.a REFCPU_INC=$FM/include/api/ EXTRA_CXXFLAGS="$2" > "$1.log" 2>&1
rc=$?; grep " error" "$1.log" | head -5; echo "$1 rc=$rc"; exit $rc

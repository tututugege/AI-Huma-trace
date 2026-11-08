#!/usr/bin/env bash
# dse_build.sh (ON the server) <configs.txt> [only-name...]: build one simulator per config line "name|extra cxxflags" into /root/dse/AI-Huma/trace-model/build_dse_<name>
cd /root/dse/AI-Huma/trace-model || exit 1
CF=$1; shift
[ -f build_dramsim3/libdramsim3.a ] || make dramsim3 > build_dramsim3.log 2>&1 || { echo 'dramsim3 build failed'; exit 1; }
build() { n=${1%%|*}; fl=${1#*|}; d=build_dse_$n
  [ -x $d/simulator ] && [ "$(cat $d/flags 2>/dev/null)" = "$fl" ] && { echo "[$n] cached"; return; }
  rm -rf $d; make -j8 BUILD_DIR=$d EXTRA_CXXFLAGS="$fl" > $d.log 2>&1 && echo "$fl" > $d/flags && echo "[$n] built" || echo "[$n] BUILD FAILED (see $d.log)"; }
export -f build
grep -v '^#' "$CF" | grep -v '^$' | { if [ $# -gt 0 ]; then grep -E "^($(IFS='|'; echo "$*"))\|"; else cat; fi; } | xargs -P 8 -I{} bash -c 'build "{}"'

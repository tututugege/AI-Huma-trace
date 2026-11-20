#!/bin/bash
# jbuild_dse.sh <src-tag> <bin-tag> [EXTRA_CXXFLAGS]: static rv64gc trace-model from tm_<src-tag>.tgz (EXP-047 DSE fork), binary -> eda nfs tm/bin/sim64_<bin-tag>
set -e
B=~/rv64spec/llcplru1; T=$1; O=$2; F=${3:-}; D=$B/$T; mkdir -p $D
cd $D && [ -f Makefile ] || tar -xzf $B/tm_$T.tgz 2>/dev/null
mkdir -p third_party/DRAMsim3 libs && [ -f third_party/DRAMsim3/src/dramsim3.h ] || cp -r $B/tm/third_party/DRAMsim3/. third_party/DRAMsim3/
[ -d build_dramsim3 ] || cp -r $B/tm/build_dramsim3 . ; cp $B/tm/libs/softfloat.a libs/softfloat.a
true
sed -i "s/^  io = {};$/  io = AXI_LLC_IO_t();/" axi-interconnect-kit/axi_interconnect/AXI_LLC.cpp
nice make -j12 XLEN=64 RVC=1 BUILD_DIR=build_$O EXTRA_CXXFLAGS="$F" DS3_DIR=/tmp/llcplru1_ds3 DS3_LIB=build_dramsim3/libdramsim3.a LDFLAGS="-static -lz" REFCPU_LIB=$B/fm2/librefcpu.a REFCPU_INC=$B/fm2/include/api/ > $D/build_$O.log 2>&1 || { grep -E " error" $D/build_$O.log | head; exit 1; }
cp build_$O/simulator $B/sim64_$O
scp -q -o BatchMode=yes $B/sim64_$O houruyao@10.208.120.12:/nfs_global/S/houruyao/spec06_rv64gc/tm/bin/
echo built sim64_$O

#!/usr/bin/env bash
# setup_rtl_server.sh: install what the RTL testbench build needs on an Ubuntu 22.04 server (run on the server, as root):
# clang, Verilator v5.052 from source (apt has 4.x), then leave /root/verilator installed in /usr/local.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq clang flex bison help2man autoconf automake libfl-dev libgoogle-perftools-dev ccache zlib1g-dev perl python3 git make g++ numactl
if ! verilator --version 2>/dev/null | grep -q "5.052"; then
  [ -d /root/verilator ] || git clone --depth 1 --branch v5.052 https://github.com/verilator/verilator /root/verilator
  cd /root/verilator && autoconf && ./configure --prefix=/usr/local && make -j32 && make install
fi
verilator --version
echo SETUP-DONE

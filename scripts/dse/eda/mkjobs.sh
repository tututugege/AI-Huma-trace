#!/bin/bash
# mkjobs.sh <in jobs95> <out>: "<bench> <ckpt> <weight>" for every point (weight from sp/<bench>.weights by the sp index), mcf first
R=/nfs_global/S/houruyao/spec06_rv64gc
while read b n; do
  i=$(echo $n | sed 's/^ckpt_sp\([0-9]*\)_.*/\1/')
  w=$(awk -v i=$i '$2==i{print $1}' $R/sp/$b.weights)
  echo "$b $n ${w:-0}"
done < $1 | awk '{print ($1=="429.mcf"?0:($1=="471.omnetpp"||$1=="483.xalancbmk"?1:2)), $0}' | sort -s -k1,1n | cut -d' ' -f2- > $2
wc -l < $2; head -2 $2; awk '$3==0' $2 | wc -l

#!/usr/bin/env bash
# Third Shoulder: heavy jobs for a many-core machine. Run from the repo root (the folder containing cad/ and stl/).
set -u
mkdir -p results logs
N=$(python3 -c "import os;print(os.cpu_count())")
echo "cores: $N"
cd cad
# 1) wire loop re-search, 30 extra combined-pose bases (args: joint, n_candidates, seed, n_bases)
for J in W2 W3; do nohup python3 -u wire_search_combined.py $J 400 2 30 > ../logs/search_$J.log 2>&1 & done
# 2) full sweeps (need cowl/pauldron/wearer/harness STLs: run harness_chain.sh first)
nohup python3 -u sweep_full.py            > ../logs/sweep_full.log 2>&1 &
nohup python3 -u sweep_full.py --restricted > ../logs/sweep_full_restricted.log 2>&1 &
wait
cp ../stl/electronics/wire_search_*_CB.json ../stl/sweep_full*.json ../stl/elbow_limits_15mm*.json ../results/ 2>/dev/null
echo ALL_DONE

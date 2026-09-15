#!/bin/bash
set -e 

cd "$(dirname "$0")/.."

echo "--- Building C++ Solver ---"
cd build
make
cd ..

CSV_FILE="data/benchmark_results.csv"
if [ ! -f "$CSV_FILE" ]; then
    echo "map_file,scen_file,solver,agents,solved,path_cost,runtime_us,nodes_generated" > "$CSV_FILE"
fi

echo "--- Running Solver (Prioritized Planning) ---"
./build/mapf_solver -m data/gen_map.txt -s data/gen_scen.txt --out data/output_paths.json --hl cbs --ll astar | tee -a "$CSV_FILE"

echo "--- Launching Viser ---"
python visualise.py -m data/gen_map.txt -p data/output_paths.json
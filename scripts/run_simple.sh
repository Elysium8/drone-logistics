#!/bin/bash
set -e 

echo "--- Building C++ Solver ---"
cd build
make

echo "--- Running Simple Scenario ---"
./mapf_solver -m ../data/simple_map.txt -s ../data/simple_scen.txt --out ../data/output_paths.json

echo "--- Launching Viser ---"
cd ..
python visualise.py -m data/simple_map.txt -p data/output_paths.json
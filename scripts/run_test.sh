#!/bin/bash
set -e

# 1. Always run from the project root
cd "$(dirname "$0")/.."

# 2. Default Variables
HL_SOLVER=""
LL_SOLVER=""
MAP_FILE=""
SCEN_FILE=""
GEN_ARGS=""
OUT_FILE="data/output_paths.json"
CSV_FILE="data/benchmark_results.csv"

# 3. Parse Command Line Arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --hl) HL_SOLVER="$2"; shift ;;
        --ll) LL_SOLVER="$2"; shift ;;
        -m) MAP_FILE="$2"; shift ;;
        -s) SCEN_FILE="$2"; shift ;;
        --gen) GEN_ARGS="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# 4. Validate Inputs
if [ -z "$HL_SOLVER" ] || [ -z "$LL_SOLVER" ]; then
    echo "Error: You must provide both --hl and --ll."
    echo "Usage: ./scripts/run_test.sh --hl <solver> --ll <solver> [--gen \"args\" | -m <map> -s <scen>]"
    exit 1
fi

# 5. Handle Generation vs. Existing Files
if [ -n "$GEN_ARGS" ]; then
    echo "--- Generating New Scenario ---"
    # Word-splitting $GEN_ARGS passes the string as individual arguments to Python
    python scenario_generator.py $GEN_ARGS
    
    # Assuming your generator always saves to these defaults:
    MAP_FILE="data/gen_map.txt"
    SCEN_FILE="data/gen_scen.txt"
elif [ -z "$MAP_FILE" ] || [ -z "$SCEN_FILE" ]; then
    echo "Error: You must provide either --gen \"args\" OR both -m <map> and -s <scen>."
    exit 1
fi

# 6. Ensure C++ is compiled
echo "--- Compiling ---"
make -C build > /dev/null

# 7. Ensure CSV headers exist
if [ ! -f "$CSV_FILE" ]; then
    echo "map_file,scen_file,solver,agents,solved,path_cost,runtime_us,nodes_generated" > "$CSV_FILE"
fi

# 8. Run the Solver
echo "--- Running $HL_SOLVER with $LL_SOLVER ---"
./build/mapf_solver -m "$MAP_FILE" -s "$SCEN_FILE" --out "$OUT_FILE" --hl "$HL_SOLVER" --ll "$LL_SOLVER" | tee -a "$CSV_FILE"

# 9. Visualize
echo "--- Launching Visualizer ---"
python visualise.py -m "$MAP_FILE" -p "$OUT_FILE"
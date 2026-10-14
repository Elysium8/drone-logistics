#!/bin/bash
set -e

# 1. Always run from the project root
cd "$(dirname "$0")/.."

# 2. Default Variables
LL_SOLVER=""
PARENT_DIR=""
PROFILE_MODE=0
AGENTS=""

# 3. Parse Command Line Arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --ll) LL_SOLVER="$2"; shift ;;
        --dir) PARENT_DIR="$2"; shift ;;
        --agents) AGENTS="$2"; shift ;;
        --profile) PROFILE_MODE=1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# 4. Validate Inputs
if [ -z "$LL_SOLVER" ] || [ -z "$PARENT_DIR" ]; then
    echo "Error: You must provide --ll and --dir."
    echo "Usage: ./scripts/run_suite.sh --ll <solver> --dir tests [--agents 100,200,500] [--profile]"
    exit 1
fi

if [ ! -d "$PARENT_DIR" ]; then
    echo "Error: Directory '$PARENT_DIR' does not exist."
    exit 1
fi

# 5. Ensure C++ is compiled
echo "--- Compiling ---"
make -C build > /dev/null

# 6. Iterate over all subfolders in the given parent directory
for SUITE_DIR in "$PARENT_DIR"/*/; do
    SUITE_DIR=${SUITE_DIR%/}
    
    echo "========================================================"
    echo "Processing Test Suite: $SUITE_DIR"
    echo "========================================================"
    
    CSV_FILE="$SUITE_DIR/benchmark_results.csv"
    rm -f "$CSV_FILE"

    shopt -s nullglob
    map_files=("$SUITE_DIR"/map_*.*)
    total_maps=${#map_files[@]}
    shopt -u nullglob

    if [ "$total_maps" -eq 0 ]; then
        echo "  No map files found. Skipping."
        continue
    fi

    # 7. Execute the Native Runner (with or without profiling)
    # ${AGENTS:+--agents "$AGENTS"} expands to nothing when --agents was not given
    if [ "$PROFILE_MODE" -eq 1 ]; then
        echo "  Running Valgrind Callgrind on $SUITE_DIR (this will take a while)..."
        valgrind --tool=callgrind --dump-instr=yes --collect-jumps=yes ./build/benchmark_runner --dir "$SUITE_DIR" --csv "$CSV_FILE" --ll "$LL_SOLVER" ${AGENTS:+--agents "$AGENTS"}
    else
        ./build/benchmark_runner --dir "$SUITE_DIR" --csv "$CSV_FILE" --ll "$LL_SOLVER" ${AGENTS:+--agents "$AGENTS"}
    fi

    # 8. Calculate Aggregate Statistics and Generate Plots
    python3 -c "
import csv
import sys
import os
from collections import defaultdict
try:
    import matplotlib.pyplot as plt
except ImportError:
    print('  Matplotlib not installed. Skipping plot generation.')
    plt = None

csv_file = '$CSV_FILE'
suite_dir = '$SUITE_DIR'

stats = defaultdict(lambda: {
    'total': 0, 'solved': 0, 'makespan': 0.0, 'runtime': 0.0,
    'path_cost': 0.0, 'path_cost_sq': 0.0, 'longest_path': 0.0, 'total_mvmt': 0.0, 
    'll_gen': 0.0, 'll_exp': 0.0, 'hl_gen': 0.0, 'hl_exp': 0.0
})

plot_data = defaultdict(lambda: defaultdict(lambda: {
    'total': 0, 'solved': 0, 'runtime': 0.0, 'path_cost': 0.0
}))

try:
    if not os.path.exists(csv_file):
        print('  No CSV generated. Skipping statistics.')
        sys.exit(0)

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            solver = str(row.get('solver') or 'unknown').strip()
            try:
                agents = int(row.get('agents') or 0)
            except ValueError:
                continue

            stats[solver]['total'] += 1
            plot_data[solver][agents]['total'] += 1
            
            solved_flag = str(row.get('solved') or '0').strip()
            if solved_flag == '1':
                stats[solver]['solved'] += 1
                plot_data[solver][agents]['solved'] += 1
                try:
                    runtime = float(row.get('runtime_us') or 0)
                    path_cost = float(row.get('path_cost') or 0)
                    
                    stats[solver]['makespan'] += float(row.get('makespan') or 0)
                    stats[solver]['runtime'] += runtime
                    stats[solver]['path_cost'] += path_cost
                    stats[solver]['path_cost_sq'] += float(row.get('path_cost_squared') or 0)
                    stats[solver]['longest_path'] += float(row.get('longest_path') or 0)
                    stats[solver]['total_mvmt'] += float(row.get('total_movement') or 0)
                    stats[solver]['ll_gen'] += float(row.get('ll_nodes_generated') or 0)
                    stats[solver]['ll_exp'] += float(row.get('ll_nodes_expanded') or 0)
                    stats[solver]['hl_gen'] += float(row.get('hl_nodes_generated') or 0)
                    stats[solver]['hl_exp'] += float(row.get('hl_nodes_expanded') or 0)
                    
                    plot_data[solver][agents]['runtime'] += runtime
                    plot_data[solver][agents]['path_cost'] += path_cost
                except ValueError:
                    pass

    if not stats:
        print('  No valid data found in CSV for this suite.\n')
        sys.exit(0)

    output_lines = []
    header = f'| {\"Solver\":<22} | {\"Success\":<10} | {\"Time(ms)\":<10} | {\"MKSPN\":<8} | {\"Cost\":<8} | {\"CostSq\":<10} | {\"MaxPath\":<8} | {\"Mvmt\":<8} | {\"LL_Gen\":<8} | {\"LL_Exp\":<8} | {\"HL_Gen\":<8} | {\"HL_Exp\":<8} |'
    output_lines.append(header)
    output_lines.append('-' * len(header))
    
    for solver, data in stats.items():
        total = data['total']
        solved = data['solved']
        if total == 0: continue
            
        avg_runtime_ms = (data['runtime'] / solved) / 1000 if solved > 0 else 0.0
        avg_mkspn = data['makespan'] / solved if solved > 0 else 0.0
        avg_cost = data['path_cost'] / solved if solved > 0 else 0.0
        avg_cost_sq = data['path_cost_sq'] / solved if solved > 0 else 0.0
        avg_maxpath = data['longest_path'] / solved if solved > 0 else 0.0
        avg_mvmt = data['total_mvmt'] / solved if solved > 0 else 0.0
        avg_ll_gen = data['ll_gen'] / solved if solved > 0 else 0.0
        avg_ll_exp = data['ll_exp'] / solved if solved > 0 else 0.0
        avg_hl_gen = data['hl_gen'] / solved if solved > 0 else 0.0
        avg_hl_exp = data['hl_exp'] / solved if solved > 0 else 0.0
        
        rate_str = f'{solved}/{total}'
        row_str = f'| {solver:<22} | {rate_str:<10} | {avg_runtime_ms:<10.2f} | {avg_mkspn:<8.1f} | {avg_cost:<8.1f} | {avg_cost_sq:<10.1f} | {avg_maxpath:<8.1f} | {avg_mvmt:<8.1f} | {avg_ll_gen:<8.1f} | {avg_ll_exp:<8.1f} | {avg_hl_gen:<8.1f} | {avg_hl_exp:<8.1f} |'
        output_lines.append(row_str)

    report = '\n'.join(output_lines)
    print('\n' + report + '\n')
    
    with open(os.path.join(suite_dir, 'comparative_summary.txt'), 'w') as out:
        out.write(report + '\n')

    if plt is not None:
        fig, axs = plt.subplots(3, 1, figsize=(8, 12), sharex=True)
        suite_name = os.path.basename(os.path.normpath(suite_dir))
        fig.suptitle(f'Performance Metrics: {suite_name}', fontsize=16)

        markers = ['o', 's', '^', 'D', 'v', 'p']
        
        for idx, solver in enumerate(sorted(plot_data.keys())):
            agent_counts = sorted(plot_data[solver].keys())
            if not agent_counts: continue
                
            success_rates, runtimes_ms, path_costs = [], [], []
            for a in agent_counts:
                d = plot_data[solver][a]
                success_rates.append(d['solved'] / d['total'] if d['total'] > 0 else 0.0)
                runtimes_ms.append((d['runtime'] / d['solved'] / 1000) if d['solved'] > 0 else None)
                path_costs.append((d['path_cost'] / d['solved']) if d['solved'] > 0 else None)
            
            m = markers[idx % len(markers)]
            axs[0].plot(agent_counts, success_rates, marker=m, label=solver)
            axs[1].plot(agent_counts, runtimes_ms, marker=m, label=solver)
            axs[2].plot(agent_counts, path_costs, marker=m, label=solver)

        axs[0].set_ylabel('Success Rate')
        axs[0].set_ylim(-0.05, 1.05)
        axs[0].grid(True, linestyle='--', alpha=0.7)
        axs[0].legend()

        axs[1].set_ylabel('Avg Runtime (ms)')
        axs[1].grid(True, linestyle='--', alpha=0.7)

        axs[2].set_ylabel('Avg Path Cost')
        axs[2].set_xlabel('Agents: |A|')
        axs[2].grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout()
        plot_path = os.path.join(suite_dir, 'summary_plots.png')
        plt.savefig(plot_path, dpi=150)
        print(f'  Saved plot to {plot_path}')

except Exception as e:
    print(f'  Could not calculate statistics or generate plot: {e}\n')
"
done

echo "========================================================"
echo "All test suites in $PARENT_DIR completed!"

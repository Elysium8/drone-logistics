#!/bin/bash
set -e

# 1. Always run from the project root
cd "$(dirname "$0")/.."

# 2. Default Variables
LL_SOLVER=""
PARENT_DIR=""
PROFILE_MODE=0
AGENTS=""
GLOBAL_ONLY=0
BASELINE=""

# 3. Parse Command Line Arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --ll) LL_SOLVER="$2"; shift ;;
        --dir) PARENT_DIR="$2"; shift ;;
        --agents) AGENTS="$2"; shift ;;
        --profile) PROFILE_MODE=1 ;;
        --global-only) GLOBAL_ONLY=1 ;;
        --baseline) BASELINE="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# 4. Validate Inputs
if [ -z "$PARENT_DIR" ] || { [ -z "$LL_SOLVER" ] && [ "$GLOBAL_ONLY" -eq 0 ]; }; then
    echo "Error: You must provide --ll and --dir."
    echo "Usage: ./scripts/run_suite.sh --ll <solver> --dir tests [--agents 100,200,500] [--profile] [--baseline <solver>]"
    echo "       ./scripts/run_suite.sh --dir tests --global-only [--baseline <solver>]   (re-aggregate the existing global CSV)"
    exit 1
fi

if [ ! -d "$PARENT_DIR" ]; then
    echo "Error: Directory '$PARENT_DIR' does not exist."
    exit 1
fi

GLOBAL_CSV="$PARENT_DIR/global_benchmark_results.csv"

# --global-only: skip the benchmarks and just re-aggregate the existing global CSV
if [ "$GLOBAL_ONLY" -eq 0 ]; then

# 5. Ensure C++ is compiled
echo "--- Compiling ---"
make -C build > /dev/null

# ==========================================
# GLOBAL AGGREGATION SETUP
# ==========================================
echo "map_file,scen_file,solver,agents,solved,makespan,path_cost,path_cost_squared,longest_path,total_movement,runtime_us,ll_nodes_generated,ll_nodes_expanded,hl_nodes_generated,hl_nodes_expanded" > "$GLOBAL_CSV"


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
    if [ "$PROFILE_MODE" -eq 1 ]; then
        echo "  Running Valgrind Callgrind on $SUITE_DIR (this will take a while)..."
        valgrind --tool=callgrind --dump-instr=yes --collect-jumps=yes ./build/benchmark_runner --dir "$SUITE_DIR" --csv "$CSV_FILE" --ll "$LL_SOLVER" ${AGENTS:+--agents "$AGENTS"}
    else
        ./build/benchmark_runner --dir "$SUITE_DIR" --csv "$CSV_FILE" --ll "$LL_SOLVER" ${AGENTS:+--agents "$AGENTS"}
    fi

    # Append to global CSV (skipping the header line of the local CSV)
    if [ -f "$CSV_FILE" ]; then
        tail -n +2 "$CSV_FILE" >> "$GLOBAL_CSV"
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
    header = f'| {\"Solver\":<26} | {\"Success\":<10} | {\"Time(ms)\":<10} | {\"MKSPN\":<8} | {\"Cost\":<8} | {\"CostSq\":<10} | {\"MaxPath\":<8} | {\"Mvmt\":<8} | {\"LL_Gen\":<8} | {\"LL_Exp\":<8} | {\"HL_Gen\":<8} | {\"HL_Exp\":<8} |'
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
        row_str = f'| {solver:<26} | {rate_str:<10} | {avg_runtime_ms:<10.2f} | {avg_mkspn:<8.1f} | {avg_cost:<8.1f} | {avg_cost_sq:<10.1f} | {avg_maxpath:<8.1f} | {avg_mvmt:<8.1f} | {avg_ll_gen:<8.1f} | {avg_ll_exp:<8.1f} | {avg_hl_gen:<8.1f} | {avg_hl_exp:<8.1f} |'
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
echo "All test suites completed! Generating GLOBAL summary..."
echo "========================================================"

fi

# ==========================================
# GENERATE GLOBAL SUMMARY FROM COMBINED CSV
# ==========================================
# Solvers finish different sets of instances, so summing a metric (or averaging each solver over
# its own solved set) rewards a solver that only finishes the easy instances. Quality metrics are
# therefore compared on the instances that EVERY solver solved, and divided by the number of
# agents where they grow with it, so 50- and 1000-agent instances carry comparable weight.
python3 - "$GLOBAL_CSV" "$PARENT_DIR" "$BASELINE" <<'PYTHON'
import csv
import math
import os
import statistics
import sys
from collections import defaultdict

global_csv, parent_dir = sys.argv[1], sys.argv[2]
baseline = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3] else None

if not os.path.exists(global_csv):
    print('  No global CSV found.')
    sys.exit(0)

# ==========================================
# 1. LOAD: one record per (instance, solver), instance = (map, scen, agents)
# ==========================================
def stem(path, prefix):
    name = os.path.splitext(os.path.basename(path))[0]
    return name[len(prefix):] if name.startswith(prefix) else name

results = defaultdict(dict)   # solver -> instance -> metrics dict, or None if unsolved
solvers = []                  # order of first appearance (= order in benchmark_runner)
instances = set()

with open(global_csv) as f:
    for row in csv.DictReader(f):
        try:
            solver = row['solver'].strip()
            agents = int(row['agents'])
            solved = row['solved'].strip() == '1'
            key = (stem(row['map_file'], 'map_'), stem(row['scen_file'], 'scen_'), agents)
        except (KeyError, ValueError, AttributeError):
            continue

        if solver not in results:
            solvers.append(solver)
        instances.add(key)
        results[solver][key] = None

        if solved and agents > 0:
            try:
                cost = float(row['path_cost'])
                moves = float(row['total_movement'])
                results[solver][key] = {
                    'soc': cost / agents,                                     # mean path length per agent
                    'rms': math.sqrt(float(row['path_cost_squared']) / agents),  # RMS path length (punishes long paths)
                    'makespan': float(row['makespan']),
                    'longest': float(row['longest_path']),
                    'moves': moves / agents,                                  # moves per agent
                    'wait_pct': 100.0 * (cost - moves) / cost if cost > 0 else 0.0,
                    'runtime_ms': float(row['runtime_us']) / 1000.0,
                }
            except (KeyError, ValueError):
                pass

if not solvers:
    print('  No valid rows in global CSV.')
    sys.exit(0)

# Baseline: --baseline may omit the low-level suffix ("pibt_frustration" matches "pibt_frustration-astar")
if baseline:
    matches = [s for s in solvers if s == baseline or s.startswith(baseline + '-')]
    if not matches:
        print(f'  Baseline \'{baseline}\' not in CSV, using {solvers[0]}')
    baseline = matches[0] if matches else solvers[0]
else:
    baseline = solvers[0]

# Display names: drop a low-level suffix shared by every solver ("-astar")
suffixes = {s.rsplit('-', 1)[1] for s in solvers if '-' in s}
shared_suffix = '-' + suffixes.pop() if len(suffixes) == 1 and all('-' in s for s in solvers) else ''
def short(s):
    return s[:-len(shared_suffix)] if shared_suffix and s.endswith(shared_suffix) else s

instances = sorted(instances)
agent_counts = sorted({k[2] for k in instances})
maps = sorted({k[0] for k in instances})

def is_solved(s, k):
    return results[s].get(k) is not None

def attempted(s, ks):
    return [k for k in ks if k in results[s]]

common = [k for k in instances if all(is_solved(s, k) for s in solvers)]
common_at = {a: [k for k in common if k[2] == a] for a in agent_counts}

def mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')

def ci95(xs):
    return 1.96 * statistics.stdev(xs) / math.sqrt(len(xs)) if len(xs) > 1 else 0.0

def gmean(xs):
    xs = [x for x in xs if x > 0]
    return math.exp(mean([math.log(x) for x in xs])) if xs else float('nan')

def values(s, ks, metric):
    return [results[s][k][metric] for k in ks]

METRICS = [  # key, column header, format
    ('soc', 'SOC/agent', '.2f'),
    ('rms', 'RMS path', '.2f'),
    ('makespan', 'Makespan', '.1f'),
    ('longest', 'Longest', '.1f'),
    ('moves', 'Moves/agent', '.2f'),
    ('wait_pct', 'Wait %', '.1f'),
    ('runtime_ms', 'Time ms', '.2f'),
]

def fmt(x, spec):
    return '-' if x is None or (isinstance(x, float) and math.isnan(x)) else format(x, spec)

def table(headers, rows):
    widths = [max(len(str(c)) for c in col) for col in zip(headers, *rows)]
    line = lambda cells: '| ' + ' | '.join(str(c).ljust(w) for c, w in zip(cells, widths)) + ' |'
    header = line(headers)
    return [header, '-' * len(header)] + [line(r) for r in rows]

# ==========================================
# 2. PER-SOLVER STATISTICS
# ==========================================
summary = {}
for s in solvers:
    tried = attempted(s, instances)
    solved_keys = [k for k in tried if is_solved(s, k)]
    unique = [k for k in solved_keys if not any(is_solved(o, k) for o in solvers if o != s)]

    # Pairwise against the baseline: every instance both solved
    pair = [k for k in instances if is_solved(s, k) and is_solved(baseline, k)]
    soc_s, soc_b = values(s, pair, 'soc'), values(baseline, pair, 'soc')

    summary[s] = {
        'attempted': len(tried),
        'solved': len(solved_keys),
        'success': 100.0 * len(solved_keys) / len(tried) if tried else float('nan'),
        'unique': len(unique),
        'success_at': {a: (100.0 * sum(is_solved(s, k) for k in attempted(s, [k for k in instances if k[2] == a]))
                           / max(1, len(attempted(s, [k for k in instances if k[2] == a]))))
                       for a in agent_counts},
        'mean': {m: mean(values(s, common, m)) for m, _, _ in METRICS},
        'ci95': {m: ci95(values(s, common, m)) for m, _, _ in METRICS},
        'median_runtime': statistics.median(values(s, common, 'runtime_ms')) if common else float('nan'),
        'pair_n': len(pair),
        'ratio': {m: gmean([results[s][k][m] / results[baseline][k][m]
                            for k in pair if results[baseline][k][m] > 0]) for m in ('soc', 'rms', 'makespan', 'runtime_ms')},
        'better': sum(a < b for a, b in zip(soc_s, soc_b)),
        'tie': sum(a == b for a, b in zip(soc_s, soc_b)),
        'worse': sum(a > b for a, b in zip(soc_s, soc_b)),
        'vs_best': mean([results[s][k]['soc'] / min(results[o][k]['soc'] for o in solvers) for k in common]),
        'soc_at': {a: mean(values(s, common_at[a], 'soc')) for a in agent_counts},
    }

# ==========================================
# 3. TEXT REPORT
# ==========================================
lines = []
out = lines.append
out('GLOBAL AGGREGATED METRICS ACROSS ALL TEST SUITES')
out(f'{len(solvers)} solvers | {len(instances)} instances | {len(maps)} maps | agent counts: {", ".join(map(str, agent_counts))}')
out(f'Baseline: {short(baseline)}')
out('')

out('1. COVERAGE  (success rate over every attempted instance; Unique = solved by no other solver)')
lines += table(['Solver', 'Solved', 'Success %'] + [f'{a} ag' for a in agent_counts] + ['Unique'],
               [[short(s), f'{summary[s]["solved"]}/{summary[s]["attempted"]}', fmt(summary[s]['success'], '.1f')]
                + [fmt(summary[s]['success_at'][a], '.1f') for a in agent_counts] + [summary[s]['unique']] for s in solvers])
out('')

out(f'2. SOLUTION QUALITY  (mean over the {len(common)} instances solved by EVERY solver)')
lines += table(['Solver'] + [h for _, h, _ in METRICS] + ['Median ms'],
               [[short(s)] + [fmt(summary[s]['mean'][m], f) for m, _, f in METRICS] + [fmt(summary[s]['median_runtime'], '.2f')]
                for s in solvers])
out('')

out(f'3. RELATIVE TO BASELINE {short(baseline)}  (geometric mean of per-instance ratios over instances both solved; <1 = better)')
out('   SOC W/T/L = instances where the solver\'s sum of costs is lower / equal / higher than the baseline\'s')
out('   SOC vs best = mean of SOC / best SOC of any solver, over the commonly solved instances')
lines += table(['Solver', 'Both solved', 'SOC ratio', 'RMS ratio', 'Makespan ratio', 'Time ratio', 'SOC W/T/L', 'SOC vs best'],
               [[short(s), summary[s]['pair_n'], fmt(summary[s]['ratio']['soc'], '.4f'), fmt(summary[s]['ratio']['rms'], '.4f'),
                 fmt(summary[s]['ratio']['makespan'], '.4f'), fmt(summary[s]['ratio']['runtime_ms'], '.3f'),
                 '-' if s == baseline else f'{summary[s]["better"]}/{summary[s]["tie"]}/{summary[s]["worse"]}',
                 fmt(summary[s]['vs_best'], '.4f')]
                for s in solvers])
out('')

out('4. SOC/AGENT BY AGENT COUNT  (mean over instances solved by every solver at that count; n = instances)')
lines += table(['Solver'] + [f'{a} (n={len(common_at[a])})' for a in agent_counts],
               [[short(s)] + [fmt(summary[s]['soc_at'][a], '.2f') for a in agent_counts] for s in solvers])
out('')

out('5. SUCCESS % BY MAP')
map_success = {}
for m in maps:
    keys = [k for k in instances if k[0] == m]
    for s in solvers:
        tried = attempted(s, keys)
        map_success[(m, s)] = 100.0 * sum(is_solved(s, k) for k in tried) / len(tried) if tried else float('nan')
lines += table(['Map'] + [short(s) for s in solvers],
               [[m] + [fmt(map_success[(m, s)], '.1f') for s in solvers] for m in maps])

report = '\n'.join(lines)
print('\n' + report + '\n')
with open(os.path.join(parent_dir, 'global_comparative_summary.txt'), 'w') as f:
    f.write(report + '\n')

# ==========================================
# 4. MACHINE-READABLE AND LATEX TABLES
# ==========================================
with open(os.path.join(parent_dir, 'global_summary_table.csv'), 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['solver', 'attempted', 'solved', 'success_pct', 'unique_solves', 'n_common']
                    + [f'{m}_{stat}' for m, _, _ in METRICS for stat in ('mean', 'ci95')]
                    + ['runtime_ms_median', 'n_both_solved_with_baseline', 'soc_ratio', 'rms_ratio', 'makespan_ratio',
                       'runtime_ratio', 'soc_better', 'soc_tie', 'soc_worse', 'soc_vs_best'])
    for s in solvers:
        d = summary[s]
        writer.writerow([s, d['attempted'], d['solved'], d['success'], d['unique'], len(common)]
                        + [v for m, _, _ in METRICS for v in (d['mean'][m], d['ci95'][m])]
                        + [d['median_runtime'], d['pair_n'], d['ratio']['soc'], d['ratio']['rms'], d['ratio']['makespan'],
                           d['ratio']['runtime_ms'], d['better'], d['tie'], d['worse'], d['vs_best']])

# Best value per column in bold (highest success, lowest everything else)
TEX_COLUMNS = [  # header, value getter, format, higher is better
    (r'Success (\%)', lambda d: d['success'], '.1f', True),
    ('SOC/agent', lambda d: d['mean']['soc'], '.2f', False),
    ('RMS path', lambda d: d['mean']['rms'], '.2f', False),
    ('Makespan', lambda d: d['mean']['makespan'], '.1f', False),
    ('Time (ms)', lambda d: d['mean']['runtime_ms'], '.1f', False),
    ('SOC ratio', lambda d: d['ratio']['soc'], '.4f', False),
]
tex = [r'\begin{table}[t]', r'\centering', r'\begin{tabular}{l' + 'r' * len(TEX_COLUMNS) + '}', r'\toprule',
       'Solver & ' + ' & '.join(h for h, _, _, _ in TEX_COLUMNS) + r' \\', r'\midrule']
for s in solvers:
    cells = [short(s).replace('_', r'\_')]
    for _, get, spec, higher in TEX_COLUMNS:
        column = [get(summary[o]) for o in solvers if not math.isnan(get(summary[o]))]
        value = get(summary[s])
        text = fmt(value, spec)
        if column and not math.isnan(value) and format(value, spec) == format(max(column) if higher else min(column), spec):
            text = r'\textbf{' + text + '}'
        cells.append(text)
    tex.append(' & '.join(cells) + r' \\')
tex += [r'\bottomrule', r'\end{tabular}',
        r'\caption{Success rate over all ' + str(len(instances)) + ' instances. Quality metrics are means over the '
        + str(len(common)) + ' instances solved by every solver. SOC ratio is the geometric mean of per-instance '
        r'sum-of-costs ratios against ' + short(baseline).replace('_', r'\_') + ' over instances both solved.}',
        r'\end{table}']
with open(os.path.join(parent_dir, 'global_summary_table.tex'), 'w') as f:
    f.write('\n'.join(tex) + '\n')

# ==========================================
# 5. PLOTS (PNG for viewing, PDF for the paper)
# ==========================================
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.ticker
    from matplotlib.colors import LinearSegmentedColormap
except ImportError:
    print('  Matplotlib not installed. Skipping plot generation.')
    sys.exit(0)

INK, INK_2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
# Fixed categorical order: a solver keeps its colour and marker in every figure
PALETTE = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
MARKERS = ['o', 's', '^', 'D', 'v', 'P', 'X', '*']
style = {s: {'color': PALETTE[i % 8], 'marker': MARKERS[i % 8], 'linestyle': '-' if i < 8 else '--'}
         for i, s in enumerate(solvers)}

plt.rcParams.update({
    'font.size': 10, 'axes.titlesize': 11, 'axes.labelsize': 10, 'axes.edgecolor': INK_2,
    'axes.labelcolor': INK, 'xtick.color': INK_2, 'ytick.color': INK_2, 'text.color': INK,
    'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True, 'grid.color': GRID,
    'grid.linewidth': 0.8, 'axes.axisbelow': True, 'legend.frameon': False, 'lines.linewidth': 2,
    'lines.markersize': 6, 'savefig.bbox': 'tight', 'figure.facecolor': 'white',
})

def save(fig, name):
    for ext in ('png', 'pdf'):
        fig.savefig(os.path.join(parent_dir, f'{name}.{ext}'), dpi=200)
    plt.close(fig)
    print(f'  Saved {name}.png / .pdf')

def agent_axis(ax):
    # Log scale keeps small agent counts (50, 100) apart
    ax.set_xscale('log')
    ax.set_xlabel('Agents |A| (log scale)')
    if len(agent_counts) <= 10:
        ax.set_xticks(agent_counts)
        ax.set_xticklabels([str(a) for a in agent_counts])
        ax.xaxis.set_minor_locator(matplotlib.ticker.NullLocator())

# Figure 1: scaling with the number of agents
fig, axs = plt.subplots(2, 2, figsize=(10, 7.5))
panels = [
    (axs[0][0], '(a) Success rate', 'Success rate (%)', None),
    (axs[0][1], '(b) Sum of costs per agent', 'SOC / agent', 'soc'),
    (axs[1][0], '(c) Makespan', 'Makespan (timesteps)', 'makespan'),
    (axs[1][1], '(d) Runtime', 'Runtime (ms, log scale)', 'runtime_ms'),
]
for ax, title, ylabel, metric in panels:
    for s in solvers:
        if metric is None:
            xs = agent_counts
            ys = [summary[s]['success_at'][a] for a in agent_counts]
            ax.plot(xs, ys, label=short(s), **style[s])
            continue
        xs = [a for a in agent_counts if common_at[a]]
        ys = [mean(values(s, common_at[a], metric)) for a in xs]
        ax.plot(xs, ys, label=short(s), **style[s])
    if metric is None:
        ax.set_ylim(-2, 102)
    if metric == 'runtime_ms':
        ax.set_yscale('log')
    ax.set_title(title, loc='left')
    ax.set_ylabel(ylabel)
    agent_axis(ax)
handles, labels = axs[0][0].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=min(4, len(solvers)), bbox_to_anchor=(0.5, 1.0))
fig.text(0.5, -0.01, '(a) all attempted instances.  (b)-(d) means over the instances solved by every solver at that agent count.',
         ha='center', color=INK_2, fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.94))
save(fig, 'global_summary_plots')

# Figure 2: cactus plot, instances solved within a runtime budget
fig, ax = plt.subplots(figsize=(7, 4.5))
for s in solvers:
    times = sorted(results[s][k]['runtime_ms'] for k in instances if is_solved(s, k))
    if times:
        ax.step(times, range(1, len(times) + 1), where='post', label=short(s),
                color=style[s]['color'], linestyle=style[s]['linestyle'], linewidth=2)
ax.axhline(len(instances), color=INK_2, linestyle=':', linewidth=1)
ax.text(ax.get_xlim()[0], len(instances), f' {len(instances)} instances', va='bottom', color=INK_2, fontsize=9)
ax.set_xscale('log')
ax.set_xlabel('Runtime budget (ms, log scale)')
ax.set_ylabel('Instances solved')
ax.set_title('Instances solved within a runtime budget', loc='left')
ax.legend(loc='lower right')
save(fig, 'global_cactus')

# Figure 3: success rate per map (sequential blue: darker = higher)
BLUES = LinearSegmentedColormap.from_list('blues', ['#cde2fb', '#86b6ef', '#3987e5', '#1c5cab', '#0d366b'])
rows = maps + ['All maps']
grid = [[map_success[(m, s)] for s in solvers] for m in maps] + [[summary[s]['success'] for s in solvers]]
fig, ax = plt.subplots(figsize=(1.3 * len(solvers) + 3, 0.3 * len(rows) + 1.6))
mesh = ax.pcolormesh([[float('nan') if math.isnan(v) else v for v in r] for r in grid], cmap=BLUES, vmin=0, vmax=100,
                     edgecolors='white', linewidth=2)
for i, r in enumerate(grid):
    for j, v in enumerate(r):
        if not math.isnan(v):
            ax.text(j + 0.5, i + 0.5, f'{v:.0f}', ha='center', va='center', fontsize=8,
                    color='white' if v >= 55 else INK)
ax.set_xticks([j + 0.5 for j in range(len(solvers))])
ax.set_xticklabels([short(s) for s in solvers], rotation=30, ha='right')
ax.set_yticks([i + 0.5 for i in range(len(rows))])
ax.set_yticklabels(rows)
ax.get_yticklabels()[-1].set_fontweight('bold')
ax.invert_yaxis()
ax.grid(False)
ax.tick_params(length=0)
for spine in ax.spines.values():
    spine.set_visible(False)
ax.set_title('Success rate by map (%)', loc='left')
fig.colorbar(mesh, ax=ax, fraction=0.04, pad=0.02).outline.set_visible(False)
save(fig, 'global_success_by_map')

# Figure 4: per-instance sum-of-costs ratio against the baseline
others = [s for s in solvers if s != baseline]
if others:
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(1.4 * len(others) + 8, 4.5), gridspec_kw={'width_ratios': [1, 1.2]})
    data = [[results[s][k]['soc'] / results[baseline][k]['soc'] for k in instances
             if is_solved(s, k) and is_solved(baseline, k) and results[baseline][k]['soc'] > 0] for s in others]
    box = ax.boxplot(data, patch_artist=True, widths=0.5, showfliers=True,
                     medianprops={'color': INK, 'linewidth': 1.5},
                     flierprops={'marker': '.', 'markersize': 3, 'markeredgecolor': INK_2, 'alpha': 0.5})
    for patch, s in zip(box['boxes'], others):
        patch.set_facecolor(style[s]['color'])
        patch.set_alpha(0.85)
        patch.set_edgecolor('white')
    ax.axhline(1.0, color=INK_2, linestyle='--', linewidth=1)
    ax.set_xticks(range(1, len(others) + 1))
    ax.set_xticklabels([short(s) for s in others], rotation=30, ha='right')
    ax.set_ylabel(f'SOC ratio vs {short(baseline)}  (<1 = better)')
    ax.set_title('(a) Per-instance sum of costs ratio', loc='left')
    ax.grid(axis='x', visible=False)

    # Geometric mean ratio at each agent count (instances both solved)
    for s in others:
        xs, ys = [], []
        for a in agent_counts:
            ratios = [results[s][k]['soc'] / results[baseline][k]['soc'] for k in instances
                      if k[2] == a and is_solved(s, k) and is_solved(baseline, k) and results[baseline][k]['soc'] > 0]
            if ratios:
                xs.append(a)
                ys.append(gmean(ratios))
        ax2.plot(xs, ys, label=short(s), **style[s])
    ax2.axhline(1.0, color=INK_2, linestyle='--', linewidth=1)
    ax2.set_ylabel('Geometric mean SOC ratio')
    ax2.set_title('(b) By agent count', loc='left')
    agent_axis(ax2)
    ax2.legend()
    fig.suptitle(f'Sum of costs relative to {short(baseline)} (instances both solved, <1 = better)', x=0.01, ha='left')
    fig.tight_layout()
    save(fig, 'global_cost_ratio')
PYTHON

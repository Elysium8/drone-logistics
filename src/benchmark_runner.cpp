#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>

// Add Timer if needed based on previous compilation steps
#include "../include/Timer.hpp"

#include "../include/types.hpp"
#include "../include/Environment.hpp"
#include "../include/Expanders.hpp"
#include "../include/Heuristics.hpp"
#include "../include/FileIO.hpp"
#include "../include/PathExporter.hpp"

// High-Level Solvers
#include "../include/solvers/PrioritizedPlanning.hpp"
#include "../include/solvers/CBS.hpp"
#include "../include/solvers/PIBT.hpp"
#include "../include/solvers/PIBT_naive.hpp"

#include "../include/solvers/pull_tie_max_dist.hpp"
#include "../include/solvers/pull_tie_max_length.hpp"
#include "../include/solvers/pull_tie_sum_dist.hpp"
#include "../include/solvers/pull_tie_sum_frust.hpp"



// Low-Level Components
#include "../include/solvers/CBSConstraints.hpp" 
#include "../include/ReservationTable.hpp"       
#include "../include/solvers/SpaceTimeAStar.hpp"

// Planners 
#include "../include/planners/RandomPlanner.hpp"
#include "../include/planners/SafePlanner.hpp"
#include "../include/test/SolutionValidator.hpp" 

namespace fs = std::filesystem;

// Passed to load_scenario to mean "use every agent in the file"
constexpr int ALL_AGENTS = 100000;

struct Job {
    std::string map_path;
    std::string scen_path;
    int num_agents;   // take the first num_agents lines of the scenario (ALL_AGENTS = whole file)
};

int main(int argc, char *argv[]) {
    std::string suite_dir = "";
    std::string csv_file = "";
    std::string ll_solver = "astar";
    std::vector<int> agent_counts = {ALL_AGENTS};

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--dir" && i + 1 < argc) suite_dir = argv[++i];
        else if (arg == "--csv" && i + 1 < argc) csv_file = argv[++i];
        else if (arg == "--ll" && i + 1 < argc) ll_solver = argv[++i];
        else if (arg == "--agents" && i + 1 < argc) {
            // Comma-separated list, e.g. --agents 100,200,500,1000
            agent_counts.clear();
            std::stringstream ss(argv[++i]);
            std::string tok;
            while (std::getline(ss, tok, ',')) {
                if (tok.empty()) continue;
                int k = std::stoi(tok);
                if (k <= 0) {
                    std::cerr << "Error: agent counts must be positive (got " << k << ")\n";
                    return 1;
                }
                agent_counts.push_back(k);
            }
            if (agent_counts.empty()) {
                std::cerr << "Error: --agents needs at least one count\n";
                return 1;
            }
        }
    }

    if (suite_dir.empty() || csv_file.empty()) {
        std::cerr << "Usage: ./benchmark_runner --dir <path> --csv <path> --ll <solver> [--agents <n1,n2,...>]\n";
        return 1;
    }

    std::vector<std::string> target_solvers = {
        "pibt_frustration", 
        "pibt_dynamic", 
        "pull_max_dist", 
        "pull_max_length", 
        "pull_sum_dist", 
        "pull_sum_frust"
    };

    // 1. Gather all jobs: match every scenario to its map by name
    struct NamedFile { std::string name; std::string path; };
    std::vector<NamedFile> maps, scens;

    for (const auto& entry : fs::directory_iterator(suite_dir)) {
        if (!entry.is_regular_file()) continue;
        std::string stem = entry.path().stem().string();   // filename only, not the directory
        if (stem.rfind("map_", 0) == 0)       maps.push_back({stem.substr(4), entry.path().string()});
        else if (stem.rfind("scen_", 0) == 0) scens.push_back({stem.substr(5), entry.path().string()});
    }

    // directory_iterator order is arbitrary, so sort for reproducible runs
    std::sort(scens.begin(), scens.end(), [](const NamedFile& a, const NamedFile& b) { return a.name < b.name; });

    // "Berlin_1_256-random-1" belongs to map "Berlin_1_256"; a scen named exactly like the map also matches
    auto belongs_to = [](const std::string& scen, const std::string& map) {
        if (scen == map) return true;
        return scen.size() > map.size()
            && scen.compare(0, map.size(), map) == 0
            && scen[map.size()] == '-';
    };

    std::vector<Job> jobs;
    for (const auto& s : scens) {
        const NamedFile* best = nullptr;
        for (const auto& m : maps) {
            // longest matching map name wins, in case one map name is a prefix of another
            if (belongs_to(s.name, m.name) && (!best || m.name.size() > best->name.size()))
                best = &m;
        }
        if (best) {
            // One job per (scenario, agent count) so threads balance the load
            for (int k : agent_counts) jobs.push_back({best->path, s.path, k});
        } else {
            std::cerr << "Warning: no map found for scenario " << s.path << "\n";
        }
    }

    if (jobs.empty()) return 0;

    std::ofstream csv_out(csv_file);
    csv_out << "map_file,scen_file,solver,agents,solved,makespan,path_cost,path_cost_squared,longest_path,total_movement,runtime_us,ll_nodes_generated,ll_nodes_expanded,hl_nodes_generated,hl_nodes_expanded\n";

    // 2. Concurrency Primitives
    std::mutex csv_mutex;
    std::atomic<size_t> job_index{0};
    std::atomic<size_t> completed_jobs{0};
    size_t total_jobs = jobs.size();

    unsigned int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;

    // 3. Worker Lambda
    auto worker = [&]() {
        while (true) {
            size_t i = job_index.fetch_add(1);
            if (i >= total_jobs) break;

            const auto& job = jobs[i];
            
            // Isolate memory allocations to the thread
            Environment env = FileIO::load_map(job.map_path);
            MAPFInstance instance = FileIO::load_scenario(job.scen_path, job.num_agents);
            int actual_agents = instance.starts.size();

            // Asked for more agents than the file has: skip, otherwise the
            // largest size would be re-run and recorded once per oversized request
            if (job.num_agents != ALL_AGENTS && actual_agents < job.num_agents) {
                completed_jobs++;
                continue;
            }

            ManhattanExpander expander;
            TrueDistanceHeuristic<ManhattanExpander> heuristic(env, instance.goals, expander);
            SafePlanner planner;

            std::ostringstream local_csv_buffer;

            for (const auto& hl : target_solvers) {
                SearchMetrics metrics;
                if (hl == "pibt_dynamic") {
                    PIBT solver; solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
                } else if (hl == "pibt_frustration") {
                    PIBT_naive solver; solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
                } else if (hl == "pull_max_dist") {
                    pull_tie_max_dist solver; solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
                } else if (hl == "pull_max_length") {
                    pull_tie_max_length solver; solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
                } else if (hl == "pull_sum_dist") {
                    pull_tie_sum_dist solver; solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
                } else if (hl == "pull_sum_frust") {
                    pull_tie_sum_frust solver; solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
                }

                local_csv_buffer << job.map_path << "," << job.scen_path << "," << hl << "-" << ll_solver << ","
                        << actual_agents << "," << metrics.solved << "," << metrics.makespan << ","
                        << metrics.path_cost << "," << metrics.path_cost_squared << "," << metrics.longest_path << ","
                        << metrics.total_movement << "," << metrics.runtime_us << "," << metrics.ll_nodes_generated << ","
                        << metrics.ll_nodes_expanded << "," << metrics.hl_nodes_generated << "," << metrics.hl_nodes_expanded << "\n";
            }

            // Lock purely to write the pre-formatted string to the file
            {
                std::lock_guard<std::mutex> lock(csv_mutex);
                csv_out << local_csv_buffer.str();
                completed_jobs++;
                std::cout << "\r  Progress: " << completed_jobs << " / " << total_jobs << " (" << num_threads << " threads active)..." << std::flush;
            }
        }
    };

    // 4. Launch and Join Threads
    std::vector<std::thread> threads;
    for (unsigned int t = 0; t < num_threads; ++t) {
        threads.emplace_back(worker);
    }
    for (auto& t : threads) {
        t.join();
    }
    
    std::cout << "\n";
    return 0;
}

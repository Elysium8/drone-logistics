#include <iostream>
#include <string>
#include <vector>
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
#include "../include/solvers/PULL_IBT.hpp"

// Low-Level Components
#include "../include/solvers/CBSConstraints.hpp" 
#include "../include/ReservationTable.hpp"       
#include "../include/solvers/SpaceTimeAStar.hpp"

// Planners 
#include "../include/planners/RandomPlanner.hpp"
#include "../include/planners/SafePlanner.hpp"
#include "../include/test/SolutionValidator.hpp" 

namespace fs = std::filesystem;

struct Job {
    std::string map_path;
    std::string scen_path;
};

int main(int argc, char *argv[]) {
    std::string suite_dir = "";
    std::string csv_file = "";
    std::string ll_solver = "astar";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--dir" && i + 1 < argc) suite_dir = argv[++i];
        else if (arg == "--csv" && i + 1 < argc) csv_file = argv[++i];
        else if (arg == "--ll" && i + 1 < argc) ll_solver = argv[++i];
    }

    if (suite_dir.empty() || csv_file.empty()) {
        std::cerr << "Usage: ./benchmark_runner --dir <path> --csv <path> --ll <solver>\n";
        return 1;
    }

    std::vector<std::string> target_solvers = {"pibt_frustration", "pibt_dynamic", "pull_ibt"};

    // 1. Gather all jobs
    std::vector<Job> jobs;
    for (const auto& entry : fs::directory_iterator(suite_dir)) {
        std::string map_path = entry.path().string();
        if (map_path.find("map_") == std::string::npos) continue;

        std::string base_name = entry.path().stem().string().substr(4);
        std::string scen_path = suite_dir + "/scen_" + base_name + ".txt";
        
        if (!fs::exists(scen_path)) {
            scen_path = suite_dir + "/scen_" + base_name + ".scen";
            if (!fs::exists(scen_path)) continue;
        }
        jobs.push_back({map_path, scen_path});
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
            MAPFInstance instance = FileIO::load_scenario(job.scen_path, 100000);
            int actual_agents = instance.starts.size();

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
                } else if (hl == "pull_ibt") {
                    PULL_IBT solver; solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
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
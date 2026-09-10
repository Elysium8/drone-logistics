#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include "../include/types.hpp"
#include "../include/Environment.hpp"
#include "../include/Expanders.hpp" 
#include "../include/Heuristics.hpp"
#include "../include/FileIO.hpp"
#include "../include/PathExporter.hpp"

#include "../include/solvers/PrioritizedPlanning.hpp"

void print_usage() {
    std::cout << "Usage: ./mapf_solver -m <map.txt> -s <scen.txt> [-a <num_agents>]\n";
}

int main(int argc, char* argv[]) {
    std::string map_file = "";
    std::string scen_file = "";
    std::string out_file = ""; 
    int num_agents = 1000000; 

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-m" && i + 1 < argc) map_file = argv[++i];
        else if (arg == "-s" && i + 1 < argc) scen_file = argv[++i];
        else if (arg == "-a" && i + 1 < argc) num_agents = std::stoi(argv[++i]);
        else if (arg == "--out" && i + 1 < argc) out_file = argv[++i];
    }

    if (map_file.empty() || scen_file.empty()) {
        std::cerr << "Error: Map and scenario files are required.\n";
        print_usage();
        return 1;
    }

    try {
        Environment env = FileIO::load_map(map_file);
        
        MAPFInstance instance = FileIO::load_scenario(scen_file, num_agents);
        
        int actual_agents = instance.starts.size();

        ManhattanExpander expander;
        ManhattanHeuristic heuristic;
        SearchMetrics metrics;
        PrioritizedPlanning<ManhattanExpander, ManhattanHeuristic> solver;

        solver.solve(env, instance, expander, heuristic, metrics);
        
        if (metrics.solved && !out_file.empty()) {
        PathExporter::export_paths_json(metrics.paths, out_file);
    }

        std::cout << map_file << "," 
                  << scen_file << "," 
                  << actual_agents << ","
                  << metrics.solved << "," 
                  << metrics.path_cost << "," 
                  << metrics.runtime_us << "," 
                  << metrics.nodes_generated << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
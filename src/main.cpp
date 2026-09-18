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

// High-Level Solvers
#include "../include/solvers/PrioritizedPlanning.hpp"
#include "../include/solvers/CBS.hpp"

// Low-Level Components
#include "../include/solvers/CBSConstraints.hpp" // For ConstraintTable
#include "../include/ReservationTable.hpp"       // For PP's ReservationTable
#include "../include/solvers/SpaceTimeAStar.hpp"

void print_usage()
{
    std::cout << "Usage: ./mapf_solver -m <map.txt> -s <scen.txt> [-a <num_agents>] [--out <paths.json>] [--hl <pp|cbs>] [--ll <astar|sipp>]\n";
}

int main(int argc, char *argv[])
{
    std::string map_file = "";
    std::string scen_file = "";
    std::string out_file = "";

    // Default stack
    std::string hl_solver = "pp";
    std::string ll_solver = "astar";
    int num_agents = 1000000;

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "-m" && i + 1 < argc)
            map_file = argv[++i];
        else if (arg == "-s" && i + 1 < argc)
            scen_file = argv[++i];
        else if (arg == "-a" && i + 1 < argc)
            num_agents = std::stoi(argv[++i]);
        else if (arg == "--out" && i + 1 < argc)
            out_file = argv[++i];
        else if (arg == "--hl" && i + 1 < argc)
            hl_solver = argv[++i];
        else if (arg == "--ll" && i + 1 < argc)
            ll_solver = argv[++i];
    }

    if (map_file.empty() || scen_file.empty())
    {
        std::cerr << "Error: Map and scenario files are required.\n";
        print_usage();
        return 1;
    }

    try
    {
        Environment env = FileIO::load_map(map_file);
        MAPFInstance instance = FileIO::load_scenario(scen_file, num_agents);

        int actual_agents = instance.starts.size();

        ManhattanExpander expander;
        ManhattanHeuristic heuristic;
        SearchMetrics metrics;

        // ==========================================
        // SOLVER SELECTION MATRIX
        // ==========================================
        if (hl_solver == "cbs")
        {
            CBS solver;

            if (ll_solver == "astar")
            {
                // Compile A* specifically for CBS Constraints
                SpaceTimeAStar<ManhattanExpander, ConstraintTable, ManhattanHeuristic> ll;
                solver.solve(env, instance, expander, heuristic, ll, metrics);
            }
            else if (ll_solver == "sipp")
            {
                throw std::runtime_error("SIPP not yet implemented for CBS!");
            }
            else
            {
                throw std::invalid_argument("Unknown Low-Level solver: " + ll_solver);
            }
        }
        else if (hl_solver == "pp")
        {
            PrioritizedPlanning solver; // No class template arguments required!

            if (ll_solver == "astar")
            {
                // Compile A* specifically for PP Reservations
                SpaceTimeAStar<ManhattanExpander, ReservationTable, ManhattanHeuristic> ll;
                solver.solve(env, instance, expander, heuristic, ll, metrics);
            }
            else if (ll_solver == "sipp")
            {
                throw std::runtime_error("SIPP not yet implemented for PP!");
            }
            else
            {
                throw std::invalid_argument("Unknown Low-Level solver: " + ll_solver);
            }
        }
        else
        {
            throw std::invalid_argument("Unknown High-Level solver: " + hl_solver);
        }
        // ==========================================

        if (metrics.solved && !out_file.empty())
        {
            PathExporter::export_paths_json(metrics.paths, out_file);
        }

        // e.g., "cbs-astar" or "pp-astar"
        std::string combo_name = hl_solver + "-" + ll_solver;

        std::cout << map_file << ","
                  << scen_file << ","
                  << combo_name << ","
                  << actual_agents << ","
                  << metrics.solved << ","
                  << metrics.path_cost << ","
                  << metrics.runtime_us << ","
                  << metrics.astar_nodes_generated << ","
                  << metrics.astar_nodes_expanded << ","
                  << metrics.cbs_nodes_generated << ","
                  << metrics.cbs_nodes_expanded << "\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
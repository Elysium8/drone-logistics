#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream> // Added for file I/O

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
#include "../include/solvers/CBSConstraints.hpp" // For ConstraintTable
#include "../include/ReservationTable.hpp"       // For PP's ReservationTable
#include "../include/solvers/SpaceTimeAStar.hpp"

//Planners 
#include "../include/planners/RandomPlanner.hpp"
#include "../include/planners/SafePlanner.hpp"

// Validation
#include "../include/test/SolutionValidator.hpp" 

void print_usage()
{
    std::cout << "Usage: ./mapf_solver -m <map.txt> -s <scen.txt> [-a <num_agents>] [--out <paths.json>] [--csv <results.csv>] [--hl <pp|cbs>] [--ll <astar|sipp>]\n";
}

int main(int argc, char *argv[])
{
    std::string map_file = "";
    std::string scen_file = "";
    std::string out_file = "";
    std::string csv_file = "data/benchmark_results.csv"; // Default CSV name

    // Default solvers
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
        else if (arg == "--csv" && i + 1 < argc)
            csv_file = argv[++i];
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
        TrueDistanceHeuristic<ManhattanExpander> heuristic(env, instance.goals, expander);
        SearchMetrics metrics;
        SafePlanner planner; 
        
        // ==========================================
        // SOLVER SELECTION MATRIX
        // ==========================================
        if (hl_solver == "pibt_dynamic") {
            PIBT solver;
            solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
        }

        else if (hl_solver == "pibt_frustration") {
            PIBT_naive solver;
            solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
        }

        else if (hl_solver == "pull_ibt_max_dist") {
            PULL_IBT solver;
            solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
        }
        else if (hl_solver == "pull_ibt_max_length") {
            PULL_IBT solver;
            solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
        }
        else if (hl_solver == "pull_ibt_sum_dist") {
            PULL_IBT solver;
            solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
        }
        else if (hl_solver == "pull_ibt_sum_frust") {
            PULL_IBT solver;
            solver.solve<ManhattanExpander, TrueDistanceHeuristic<ManhattanExpander>, SafePlanner>(env, instance, expander, heuristic, planner, metrics);
        }

        else if (hl_solver == "cbs")
        {
            CBS solver;
            if (ll_solver == "astar")
            {
                SpaceTimeAStar<ManhattanExpander, ConstraintTable, TrueDistanceHeuristic<ManhattanExpander>> ll;
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
            PrioritizedPlanning solver;
            if (ll_solver == "astar")
            {
                SpaceTimeAStar<ManhattanExpander, ReservationTable, TrueDistanceHeuristic<ManhattanExpander>> ll;
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

        if (metrics.solved && !metrics.paths.empty()) {
            auto result = SolutionValidator::validate(env, metrics.paths, expander);
            if (!result.is_valid) {
                std::cerr << "--- VALIDATION FAILED ---\n";
                std::cerr << result.error_message << "\n";
                std::cerr << "-------------------------\n";
            } else {
                std::cout << "Validation: SUCCESS\n";
            }
        }

        if (!out_file.empty())
        {
            PathExporter::export_paths_json(metrics.paths, out_file);
        }

        std::string combo_name = hl_solver + "-" + ll_solver;

        // ==========================================
        // WRITE DIRECTLY TO CSV
        // ==========================================
        
        // Open file in append mode
        std::ofstream csv_out(csv_file, std::ios::app);
        if (csv_out.is_open())
        {
            // Check if file is completely empty, if so, write headers
            csv_out.seekp(0, std::ios::end);
            if (csv_out.tellp() == 0) {
                csv_out << "map_file,scen_file,solver,agents,solved,makespan,path_cost,path_cost_squared,longest_path,total_movement,runtime_us,ll_nodes_generated,ll_nodes_expanded,hl_nodes_generated,hl_nodes_expanded\n";
            }
            // print metrics

            std::cout << map_file << ","
                  << scen_file << ","
                  << combo_name << ","
                  << actual_agents << ","
                  << metrics.solved << ","
                  << metrics.makespan << ","
                  << metrics.path_cost << ","
                  << metrics.path_cost_squared << ","
                  << metrics.longest_path << ","
                  << metrics.total_movement << ","
                  << metrics.runtime_us << ","
                  << metrics.ll_nodes_generated << ","
                  << metrics.ll_nodes_expanded << ","
                  << metrics.hl_nodes_generated << ","
                  << metrics.hl_nodes_expanded << "\n";

            // Append the metrics
            csv_out << map_file << ","
                    << scen_file << ","
                    << combo_name << ","
                    << actual_agents << ","
                    << metrics.solved << ","
                    << metrics.makespan << ","
                    << metrics.path_cost << ","
                    << metrics.path_cost_squared << ","
                    << metrics.longest_path << ","
                    << metrics.total_movement << ","
                    << metrics.runtime_us << ","
                    << metrics.ll_nodes_generated << ","
                    << metrics.ll_nodes_expanded << ","
                    << metrics.hl_nodes_generated << ","
                    << metrics.hl_nodes_expanded << "\n";
            
            csv_out.close();
            std::cout << "Metrics successfully appended to " << csv_file << "\n";
        }
        else
        {
            std::cerr << "Error: Could not open CSV file " << csv_file << " for writing.\n";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
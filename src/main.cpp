#include <iostream>
#include <vector>

// Core architecture
#include "../include/types.hpp"
#include "../include/Environment.hpp"
#include "../include/Expanders.hpp" 
#include "../include/Heuristics.hpp"

// Solvers
#include "../include/solvers/PrioritizedPlanning.hpp"

int main() {
    // 1. Create a 5x5x5 grid environment
    Environment env(5, 5, 5);

    // 2. Setup a multi-agent scenario
    MAPFInstance instance;
    
    // Agent 0 goes from bottom-left to top-right
    // Agent 1 goes from bottom-right to top-left
    // They will likely cross paths in the middle
    instance.starts = {{0, 0, 0}, {4, 0, 0}};
    instance.goals  = {{4, 4, 0}, {0, 4, 0}};

    // 3. Instantiate the templated components
    ManhattanExpander expander; // Assuming this is your expander class name
    ManhattanHeuristic heuristic;
    
    // Notice how we pass the types into the solver template
    PrioritizedPlanning<ManhattanExpander, ManhattanHeuristic> solver;
    SearchMetrics metrics;

    std::cout << "Starting Prioritized Planning..." << std::endl;

    // 4. Run the solver
    solver.solve(env, instance, expander, heuristic, metrics);

    // 5. Output the results
    std::cout << "====================================\n";
    if (metrics.solved) {
        std::cout << "Status:          SUCCESS\n";
        std::cout << "Total Path Cost: " << metrics.path_cost << "\n";
    } else {
        std::cout << "Status:          FAILED\n";
    }
    std::cout << "Nodes Expanded:  " << metrics.nodes_expanded << "\n";
    std::cout << "Nodes Generated: " << metrics.nodes_generated << "\n";
    std::cout << "Runtime (us):    " << metrics.runtime_us << "\n";
    std::cout << "====================================\n";

    return 0;
}
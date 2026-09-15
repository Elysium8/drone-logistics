#pragma once
#include <vector>
#include <queue>
#include <memory>
#include "../Environment.hpp"
#include <algorithm>
#include <iostream>
#include "../types.hpp"
#include "CBSNode.hpp"
#include "CBSConstraints.hpp"
#include "../Timer.hpp"

class CBS {
private:
    // Helper: Safely get the location of an agent at time t. 
    // If t exceeds the path length, the agent sits at its goal.
    Location get_pos(const std::vector<Location>& path, int t) const {
        if (t < path.size()) return path[t];
        return path.back(); 
    }

    // Helper: Scan all paths to find the first spatial conflict
    bool find_conflict(const std::vector<std::vector<Location>>& paths, Conflict& out_conflict) const {
        int num_agents = paths.size();
        for (int i = 0; i < num_agents; ++i) {
            for (int j = i + 1; j < num_agents; ++j) {
                int max_t = std::max(paths[i].size(), paths[j].size());
                
                for (int t = 0; t < max_t; ++t) {
                    Location loc_i = get_pos(paths[i], t);
                    Location loc_j = get_pos(paths[j], t);

                    // 1. Vertex Conflict
                    if (loc_i == loc_j) {
                        out_conflict = {i, j, loc_i, loc_i, t};
                        return true;
                    }

                    // 2. Edge Conflict (Swapping)
                    if (t > 0) {
                        Location prev_i = get_pos(paths[i], t - 1);
                        Location prev_j = get_pos(paths[j], t - 1);
                        if (loc_i == prev_j && loc_j == prev_i) {
                            out_conflict = {i, j, prev_i, loc_i, t - 1};
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

public:
    template <typename ExpanderType, typename HeuristicType, typename LowLevelSolver>
    bool solve(const Environment& env, 
               const MAPFInstance& instance, 
               const ExpanderType& expander, 
               const HeuristicType& heuristic, 
               LowLevelSolver& low_level_solver, 
               SearchMetrics& metrics) 
    {
        Timer timer;
        // Min-heap for the High-Level CT (Constraint Tree)
        auto cmp = [](const std::shared_ptr<CBSNode>& left, const std::shared_ptr<CBSNode>& right) {
            return *left > *right;
        };
        std::priority_queue<std::shared_ptr<CBSNode>, std::vector<std::shared_ptr<CBSNode>>, decltype(cmp)> open_list(cmp);

        // --- 1. Generate the Root Node ---
        auto root = std::make_shared<CBSNode>();
        root->paths.resize(instance.starts.size());
        root->cost = 0;

        // Solve individually for all agents with 0 constraints
        for (size_t i = 0; i < instance.starts.size(); ++i) {
            ConstraintTable ct;
            ct.build_for_agent(i, root->constraints); // Empty constraints at root

            std::vector<Location> path = low_level_solver.solve(
                instance.starts[i], instance.goals[i], env, expander, ct, heuristic, metrics
            );

            if (path.empty()) return false; // Unsolvable single-agent path

            root->paths[i] = path;
            root->cost += (int)path.size() - 1;
        }

        open_list.push(root);

        // --- 2. High-Level Search Loop ---
        while (!open_list.empty()) {
            auto curr = open_list.top();
            open_list.pop();

            Conflict conflict;
            if (!find_conflict(curr->paths, conflict)) {
                // Goal found! No conflicts mean this is the optimal valid solution.
                metrics.solved = true;
                metrics.path_cost = curr->cost;
                metrics.paths = curr->paths;
                metrics.runtime_us = timer.elapsed_microseconds();
                return true;
            }

            // --- 3. Branching (Resolve Conflict) ---
            int agents_involved[2] = {conflict.agent1, conflict.agent2};
            
            for (int a : agents_involved) {
                auto child = std::make_shared<CBSNode>();
                child->constraints = curr->constraints; // Inherit parent's rules
                
                if (conflict.loc1 == conflict.loc2) {
                    // Vertex constraint
                    child->constraints.push_back({a, conflict.loc1, conflict.loc1, conflict.t});
                } else {
                    // Edge constraint
                    if (a == conflict.agent1) {
                        child->constraints.push_back({a, conflict.loc1, conflict.loc2, conflict.t});
                    } else {
                        child->constraints.push_back({a, conflict.loc2, conflict.loc1, conflict.t});
                    }
                }

                child->paths = curr->paths; // Copy existing paths
                
                // Replan ONLY for the affected agent 'a'
                ConstraintTable ct;
                ct.build_for_agent(a, child->constraints);

                std::vector<Location> new_path = low_level_solver.solve(
                    instance.starts[a], instance.goals[a], env, expander, ct, heuristic, metrics
                );

                if (!new_path.empty()) {
                    // Update the cost difference
                    int cost_diff = (int)new_path.size() - (int)curr->paths[a].size();
                    child->paths[a] = new_path;
                    child->cost = curr->cost + cost_diff;
                    open_list.push(child);
                }
            }
        }

        metrics.solved = false;
        metrics.runtime_us = timer.elapsed_microseconds();
        return false;
    }
};
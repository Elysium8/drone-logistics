#pragma once 
#include "../Environment.hpp"
#include "../types.hpp"
#include <vector> 
#include <array>
#include <random>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <functional>

class PULL_IBT {
private:
    template <typename ExpanderType, typename HeuristicType>
    __attribute__((noinline))
    bool solve_one_agent(int a_i, int a_j, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander, 
                         const HeuristicType &heuristic, std::vector<int>& occupied_at_t, std::vector<int>& claimed_next) {
        
        // CRITICAL FIX: If the agent hasn't spawned yet, safely pad its path and exit 
        // without attempting any array lookups or grid operations.
        if (paths[a_i][t].x == -1) {
            paths[a_i].push_back(paths[a_i][t]);
            return false;
        }

        // Get neighbours 
        std::array<Location, expander.max_neighbours> neighbours;
        int valid_neighbours = 0;
        for (const Location &neighbor : expander.get_neighbours(paths[a_i][t], env)) {
            neighbours[valid_neighbours] = neighbor;
            valid_neighbours++;
        }
        
        // Sort by Heuristic 
        std::stable_sort(neighbours.begin(), neighbours.begin() + valid_neighbours, [&](const Location& a, const Location& b) {
            float h_a = heuristic.get_h_value(a, instance.goals[a_i]);
            float h_b = heuristic.get_h_value(b, instance.goals[a_i]);

            // 1. Primary: Distance to goal (Heuristic)
            if (std::abs(h_a - h_b) > 1e-5) {
                return h_a < h_b;
            }

            // 2. Secondary (Tie-break): Presence of agents
            // Prefer vertices that do NOT currently contain an agent
            bool a_occupied = occupied_at_t[env.get_index(a)] != -1;
            bool b_occupied = occupied_at_t[env.get_index(b)] != -1;

            if (a_occupied != b_occupied) {
                return !a_occupied; 
            }

            return false;
        });

        for (int i = 0; i < valid_neighbours; i++) {
            // Swap conflict with the inheriting agent
            if (a_j != -1 && paths[a_j][t] == neighbours[i]) {
                continue;
            }

            int n_idx = env.get_index(neighbours[i]);

            // O(1) VERTEX CONFLICT CHECK
            if (claimed_next[n_idx] != -1) {
                continue; 
            }

            // Speculatively claim the location
            paths[a_i].push_back(neighbours[i]);
            claimed_next[n_idx] = a_i;

            // O(1) PRIORITY INHERITANCE CHECK
            int occupier = occupied_at_t[n_idx];
            if (occupier != -1 && !reached_goal[occupier] && paths[occupier].size() == t + 1) {
                if (!solve_one_agent(occupier, a_i, t, env, instance, expander, heuristic, occupied_at_t, claimed_next)) {
                    paths[a_i].pop_back();
                    claimed_next[n_idx] = occupier;
                    continue;
                }
            }

            if (paths[a_i][t+1] == instance.goals[a_i]) {
                reached_goal[a_i] = true;
                Location final_loc = paths[a_i].back();
                for (int d = 0; d < instance.delays[a_i]; ++d) {
                    paths[a_i].push_back(final_loc);
                }
            }
            return true;
        }
        
        // Agent failed all moves and is forced to wait in place.
        paths[a_i].push_back(paths[a_i][t]);
        claimed_next[env.get_index(paths[a_i][t])] = a_i; 
        return false;
    }

public:
    std::vector<std::vector<Location>> paths {};
    std::vector<bool> reached_goal {};
    int n {};
    
    template <typename ExpanderType, typename HeuristicType, typename PlannerType>
    __attribute__((noinline))
    bool solve(const Environment &env,
               MAPFInstance &instance,
               const ExpanderType &expander,
               const HeuristicType &heuristic,
               PlannerType &planner,
               SearchMetrics &metrics) 
    {
        Timer timer;
        int MAX_TIMESTEPS {4500}; 

        n = instance.starts.size();
        std::vector<int> frustration(n, 0);
        std::vector<float> current_h_vals(n, 0.0f);
        paths.resize(n);
        reached_goal.assign(n, false);

        for (int i = 0; i < n; i++) {
            paths[i].insert(paths[i].begin(), instance.start_times[i], {-1, -1, -1});
            paths[i].push_back(instance.starts[i]);
            current_h_vals[i] = heuristic.get_h_value(instance.starts[i], instance.goals[i]);
        }

        int total_cells = env.get_width() * env.get_height() * env.get_depth();

        for (int t=0; t < MAX_TIMESTEPS; t++) {
            
            if (std::all_of(reached_goal.begin(), reached_goal.end(), [](bool v) { return v;})) {
                metrics.paths = paths;
                metrics.runtime_us = timer.elapsed_microseconds();
                metrics.solved = true;
                int longest_airborne = 0;

                for (int i = 0; i < n; i++) {
                    int airborne = paths[i].size() - instance.start_times[i] - instance.delays[i] - 1;
                    metrics.path_cost += airborne; 
                    metrics.path_cost_squared += airborne*airborne;
                    if (airborne > longest_airborne) longest_airborne = airborne ;
                }

                metrics.longest_path = longest_airborne;
                int max_len {0};
                for (const auto& path : metrics.paths) {
                    if (path.size() > max_len) max_len = path.size();
                }
                metrics.makespan = max_len;

                for (int i = 0; i < n; i++) {
                    int movement_cost = 0;
                    for (size_t pt = 1; pt < paths[i].size(); ++pt) {
                        if (paths[i][pt].x != -1 && paths[i][pt-1].x != -1) {
                            if (!(paths[i][pt] == paths[i][pt-1])) movement_cost++;
                        }
                    }
                    metrics.total_movement += movement_cost; 
                }
                return true; 
            }

            std::vector<int> occupied_at_t(total_cells, -1);
            std::vector<int> claimed_next(total_cells, -1);

            for (int i = 0; i < n; i++) {
                if (paths[i].size() > t && paths[i][t].x != -1) {
                    int idx = env.get_index(paths[i][t]);
                    occupied_at_t[idx] = i;
                }
                if (paths[i].size() > t + 1 && paths[i][t+1].x != -1) {
                    int idx = env.get_index(paths[i][t+1]);
                    claimed_next[idx] = i;
                }
            }

            for (int i = 0; i < n; i++) {
                // 1. Spawn logic
                if (instance.start_times[i] == t && paths[i][t].x == -1) {
                    auto [start_loc, new_start_time] = planner.plan_one_agent_start(t, occupied_at_t, env.get_pads(), env);
                    if (start_loc.x != -1) {
                        paths[i][t] = start_loc; 
                        instance.starts[i] = start_loc;
                        occupied_at_t[env.get_index(start_loc)] = i;
                    } else {
                        instance.start_times[i] = new_start_time; 
                        while (paths[i].size() <= new_start_time) {
                            paths[i].push_back({-1, -1, -1});
                        }
                    }
                }
                
                // 2. Skip finished or unspawned agents
                if (reached_goal[i] || paths[i].size() <= t || paths[i][t].x == -1) continue;
                
                // 3. Update frustration
                if (paths[i][t] == instance.goals[i]) {
                    frustration[i] = 0;
                } else {
                    frustration[i] += 1;
                }

                // 4. Update dynamic priority 
                current_h_vals[i] = heuristic.get_h_value(paths[i][t], instance.goals[i]);
            }

            // ==========================================
            // LAZY-EVALUATED PRIORITY CHAINS (SUM FRUSTRATION)
            // ==========================================
            std::vector<int> chain_sum_frust(n, -1);
            std::vector<bool> in_stack(n, false); 

            std::function<int(int)> get_chain_sum_frust = [&](int curr) -> int {
                if (chain_sum_frust[curr] != -1) return chain_sum_frust[curr];
                if (in_stack[curr]) return 0; 

                if (paths[curr].empty() || (t < paths[curr].size() && paths[curr][t].x == -1)) {
                    return chain_sum_frust[curr] = 0;
                }
                
                Location curr_loc = (t < paths[curr].size()) ? paths[curr][t] : paths[curr].back();

                in_stack[curr] = true;
                int max_pull_sum = 0; // Tracks the heaviest branch behind this agent

                for (const Location &neighbor_loc : expander.get_neighbours(curr_loc, env)) {
                    int puller_id = occupied_at_t[env.get_index(neighbor_loc)];
                    
                    if (puller_id != -1 && puller_id != curr && !reached_goal[puller_id]) {
                        float puller_current_h = current_h_vals[puller_id]; 
                        float puller_next_h = heuristic.get_h_value(curr_loc, instance.goals[puller_id]);
                        
                        if (puller_current_h - puller_next_h > 1e-5f) { 
                            int puller_sum = get_chain_sum_frust(puller_id);
                            if (puller_sum > max_pull_sum) {
                                max_pull_sum = puller_sum;
                            }
                        }
                    }
                }

                in_stack[curr] = false;
                
                // Add this agent's frustration to the heaviest branch pulling it
                return chain_sum_frust[curr] = frustration[curr] + max_pull_sum;
            };

            // ==========================================
            // BASELINE SORTING (Merit + Chain Tiebreak)
            // ==========================================
            std::vector<size_t> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);

            std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](size_t left, size_t right) {
                // 1. Primary: Distance to goal
                if (std::abs(current_h_vals[left] - current_h_vals[right]) > 1e-5f) {
                    return current_h_vals[left] < current_h_vals[right]; 
                }
                                

                // 2. Secondary: Max frustration inside the dependency chain
                int l_chain_max = get_chain_sum_frust(left);
                int r_chain_max = get_chain_sum_frust(right);
                if (l_chain_max != r_chain_max) {
                    return l_chain_max > r_chain_max;
                }
                // 3. Tertiary: Personal frustration
                if (frustration[left] != frustration[right]) {
                    return frustration[left] > frustration[right];
                }
                // 4. Quaternary: ID
                return left < right;
            });

            // ==========================================
            // EXECUTE STANDARD PIBT LOOP
            // ==========================================
            for (const int a_i : sorted_priorities) {
                if (paths[a_i].size() < t+2 && !reached_goal[a_i]) {
                    solve_one_agent(a_i, -1, t, env, instance, expander, heuristic, occupied_at_t, claimed_next);
                }
            }
        }
        
        metrics.solved = false;
        metrics.runtime_us = timer.elapsed_microseconds();
        metrics.paths = paths;    
        return false;
    }
};
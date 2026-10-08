#pragma once 
#include "../Environment.hpp"
#include "../types.hpp"
#include <vector> 
#include <array>
#include <random>
#include <algorithm>
#include <numeric>
#include <iostream>

class PIBT_naive {
private:
    template <typename ExpanderType, typename HeuristicType>
    __attribute__((noinline))
    bool solve_one_agent(int a_i, int a_j, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander, 
                         const HeuristicType &heuristic, std::vector<int>& occupied_at_t, std::vector<int>& claimed_next) {
        if (t == 8 && a_i == 0) 
            {std::cout << "here";}
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
        std::stable_sort(neighbours.begin(), neighbours.begin()+valid_neighbours, [&](const Location& a, const Location& b)
                  {return heuristic.get_h_value(a, instance.goals[a_i]) < heuristic.get_h_value(b, instance.goals[a_i]);});

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
        std::vector<float> initial_priorities {};
        std::vector<float> current_priorities {};
        paths.resize(n);
        reached_goal.assign(n, false);
        for (int i = 0; i < n; i++) {
            paths[i].insert(paths[i].begin(), instance.start_times[i], {-1, -1, -1});
            paths[i].push_back(instance.starts[i]);
            float h_val = heuristic.get_h_value(instance.starts[i], instance.goals[i]);
            float tiebreak = (float)i / (float)n;
            float start_priority = 1.0f / (h_val + 2.0f + tiebreak);
            initial_priorities.push_back(start_priority);
            current_priorities.push_back(start_priority);
        }


        int total_cells = env.get_width() * env.get_height() * env.get_depth();

        for (int t=0; t < MAX_TIMESTEPS; t++) {
            if (std::all_of(reached_goal.begin(), reached_goal.end(), [](bool v) { return v;})) {
                metrics.paths = paths;
                metrics.runtime_us = timer.elapsed_microseconds();
                metrics.solved = true;
                int longest_airborne = 0;

                for (int i = 0; i < n; i++) {
                    // Total path length minus delayed spawn time and time spent waiting on the destination pad
                    int airborne = paths[i].size() - instance.start_times[i] - instance.delays[i] - 1;
                    metrics.path_cost += airborne; 
                    
                    metrics.path_cost_squared += airborne*airborne;
                    if (airborne > longest_airborne) {
                        longest_airborne = airborne ;
                    }
                }

                metrics.longest_path = longest_airborne;
                int max_len {0};
                for (const auto& path : metrics.paths) {
                    if (path.size() > max_len) {
                        max_len = path.size();
                    }
                }
                metrics.makespan = max_len;

                for (int i = 0; i < n; i++) {
                    int movement_cost = 0;
                    
                    // Iterate through the agent's path to count actual movements
                    for (size_t t = 1; t < paths[i].size(); ++t) {
                        // Ensure the agent is spawned at both t and t-1
                        if (paths[i][t].x != -1 && paths[i][t-1].x != -1) {
                            // Increment cost only if the location changed (agent didn't wait in place)
                            if (!(paths[i][t] == paths[i][t-1])) {
                                movement_cost++;
                            }
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

            for (int a=0; a<n; a++) {
                if (instance.start_times[a] == t && paths[a][t].x == -1) {
                    auto [start_loc, new_start_time] = planner.plan_one_agent_start(t, paths, instance, env.get_pads());
                    
                    if (start_loc.x != -1) {
                        paths[a][t] = start_loc; 
                        instance.starts[a] = start_loc;
                        
                        // Seed newly spawned agents directly into the O(1) occupier lookup
                        int idx = env.get_index(start_loc);
                        occupied_at_t[idx] = a;
                    } else {
                        instance.start_times[a] = new_start_time; 
                        while (paths[a].size() <= new_start_time) {
                            paths[a].push_back({-1, -1, -1});
                        }
                    }
                }
                
                if (reached_goal[a] || paths[a][t].x == -1) continue;
                if (paths[a][t] == instance.goals[a]) {
                    current_priorities[a] = initial_priorities[a];
                }
                else {
                    current_priorities[a] += 1;
                }
            }

            // for (int i = 0; i < n; i++) {
            //     // Short-circuit to skip expensive heuristic updates for finished or unspawned agents
            //     if (reached_goal[i] || paths[i].size() <= t || paths[i][t].x == -1) continue; 
                
            //     float h_val = heuristic.get_h_value(paths[i][t], instance.goals[i]);
            //     float tiebreak = (float)i / (float)n;
            //     float priority = 1.0f / (h_val + 2.0f + tiebreak);
            //     current_priorities[i] = priority;
            // }


            
            std::vector<size_t> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);
            std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&current_priorities](size_t left, size_t right) {
                return current_priorities[left] > current_priorities[right];
            });
            
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
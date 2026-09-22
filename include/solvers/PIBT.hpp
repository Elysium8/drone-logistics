#pragma once 
#include "../Environment.hpp"
#include "../types.hpp"
#include <vector> 
#include <array>
#include <random>
#include <algorithm>
#include <numeric>

class PIBT {
private:
    template <typename ExpanderType, typename HeuristicType>
    bool solve_one_agent(int a_i, int a_j, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander, 
                         const HeuristicType &heuristic) {
        
        paths[a_i].push_back(paths[a_i][t]);  // Just to make sure the path at t+1 exists
        // Get neighbours and 
        const int MAX_NEIGHBOURS {4};  //will need to change for 26 expanded in the future
        std::array<Location, MAX_NEIGHBOURS> neighbours;
        int valid_neighbours = 0;
        for (const Location &neighbor : expander.get_neighbours(paths[a_i][t], env)) {
            neighbours[valid_neighbours] = neighbor;
        }
        // Sort by Heuristic 
        std::sort(neighbours.begin(), neighbours.begin()+valid_neighbours, [&](const Location& a, const Location& b)
                  {return heuristic.get_h_value(a, instance.goals[a_i]) > heuristic.get_h_value(b, instance.goals[a_i]);});

        for (const Location &neighbour : neighbours) {
            for (int a=0; a<n; a++) {
                if (paths[a].size() > t + 1)  {
                    if (paths[a][t+1] == neighbour) {
                        goto next_neighbour; // vertex conflict 
                    }
                    if (a_j != -1 && paths[a_j][t] == neighbour) {
                        goto next_neighbour;  //swap conflict
                    }
                }
            }
        paths[a_i][t+1] == neighbour;
        for (int a_k=0; a_k<n; a_k++) {
            if (paths[a_k].size() == t+1 && paths[a_k][t] == neighbour) {
                if (!solve_one_agent(a_k, a_i, t, env, instance, expander, heuristic)) {
                    goto next_neighbour;
                }
                if (paths[a_i][t+1] == instance.goals[a_i]) {
                    reached_goal[a_i] == true;
                    Location final_loc = paths[a_i].back();
                    for (int d = 0; d < instance.delays[a_i]; ++d) {
                        paths[a_i].push_back(final_loc);
                    }
                }
                return true;
            }
        }
            next_neighbour: ;
        }
        paths[a_i][t+1] = paths[a_i][t];
        return false;
    }

public:
    std::vector<std::vector<Location>> paths {};
    std::vector<bool> reached_goal {};
    int n {};
    template <typename ExpanderType, typename HeuristicType>
    bool solve(const Environment &env,
               const MAPFInstance &instance,
               const ExpanderType &expander,
               const HeuristicType &heuristic,
               SearchMetrics &metrics) 
    {
        int MAX_TIMESTEPS {1000}; // should update to reflect map size/complexity 
        std::vector<float> initial_priorities {};
        std::vector<float> current_priorities {};
        int n = instance.starts.size();
        reached_goal.assign(n, false);
        for (int i = 0; i < n; i++) {
            paths[i].push_back(instance.starts[i]);
            initial_priorities.push_back(1.0f/(i+1)); // lazy for now 
            current_priorities.push_back(1.0f/(i+1));
        }

        for (int t=1; t < MAX_TIMESTEPS; t++) {
            if (std::all_of(reached_goal.begin(), reached_goal.end(), [](bool v) { return v;})) {
                metrics.paths = paths;
                metrics.solved = true;
                return true; // break if every agent has reached its goal
            }
            for (int a=0; a<n; a++) {
                if (paths[a][t] == instance.goals[a]) {
                    current_priorities[a] = initial_priorities[a];
                }
                else {
                    current_priorities[a] += 1;
                }
            }
            // sort agents by priorty
            std::vector<size_t> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);
            std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&current_priorities](size_t left, size_t right) {
            return current_priorities[left] < current_priorities[right];
            });
            

            for (const int a_i : sorted_priorities) {
                if (paths[a_i].size() < t+1 && !reached_goal[a_i]) {
                    solve_one_agent(a_i, -1, t, env, instance, expander, heuristic);
                }
            }
        }
    return false;
    }
};

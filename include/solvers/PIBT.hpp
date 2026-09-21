#pragma once 
#include "../Environment.hpp"
#include "../types.hpp"
#include <vector> 
#include <random>
#include <algorithm>
#include <numeric>

class PIBT {
private:
    template <typename ExpanderType, typename HeuristicType>
    bool solve_one_agent(int a_i, auto a_j, int t, const Environment &env, const ExpanderType &expander, 
                         const HeuristicType &heuristic, std::vector<std::vector<Location>> &paths) {
        
        // Get neighbours and 
        const int MAX_NEIGHBOURS {4};  //will need to change for 26 expanded in the future
        std::array<Location, MAX_NEIGHBOURS> neighbours;
        int valid_neighbours = 0;
        for (const Location &neighbor : expander.get_neighbours(paths[a_i][t], env)) {
            neighbours[valid_neighbours] = neighbor
        }
        // Sort by Heuristic 
        std::sort(neighbours.begin(), neighbours.begin+valid_neighbours, [](const Location& a, const Location& b)
                  {return heuristic.get_h_value(a, env.goals[a_i]) > heuristic.get_h_value(b, env.goals[a_i])})

        for (const Location &neighbor : neighbours) {
            
        }

        // for each neighbour check for vertex and swap conflicts at t+1 

        // if an existing agent without a next step already occupies the space then recurse on that agent 



    }
public:
    template <typename ExpanderType, typename HeuristicType>
    bool solve(const Environment &env,
               const MAPFInstance &instance,
               const ExpanderType &expander,
               const HeuristicType &heuristic,
               SearchMetrics &metrics) 
    {
        int MAX_TIMESTEPS {1000}; // should update to reflect map size/complexity 
        std::vector<std::vector<Location>> paths {};
        std::vector<float> initial_priorities {};
        std::vector<float> current_priorities {};
        int n = instance.starts.size();
        for (int i = 0; i < n; i++) {
            paths[i].push_back(instance.starts[i]);
            initial_priorities.push_back(1.0f/(i+1)); // lazy for now 
            current_priorities.push_back(1.0f/(i+1));
        }

        for (int t=1; t < MAX_TIMESTEPS; t++) {
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
                if (paths[a_i].size() < t+1) {
                    solve_one_agent(a_i, nullptr);
                }
            }
        }
    }
    




};

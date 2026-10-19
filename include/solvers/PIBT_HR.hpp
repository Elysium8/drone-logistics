#pragma once
#include "../Environment.hpp"
#include "../types.hpp"
#include <vector>
#include <array>
#include <random>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <string>
#include <stdexcept>
#include <tuple>
#include <utility>

// ==========================================
// PIBT_HR
// ==========================================
// The PIBT of Okumura & Nagai 2025, "Lightweight and Effective Preference Construction in
// PIBT for Large-Scale Multi-Agent Pathfinding" (arXiv:2505.12623), ported from the authors'
// lifelong implementation (github.com/HirokiNagai-39/pibt-tiebreaking, better_pibt/src/pibt.cpp
// and planner.cpp) into this framework. Spawning, landing delays and finished agents work as
// in the other solvers here; everything else follows their code.
//
// Agent order (their planner): priority starts at dist(start, goal) / 10000 and grows by 1
// every timestep the agent ends away from its goal. Highest priority is planned first.
//
// Candidate moves are sorted by <distance to goal, hindrance, regret, random>.
//     hindrance : number of neighbouring agents for whom the candidate cell is a step towards their goal
//     regret    : learned by running the PIBT pass several times per timestep; how much extra
//                 distance the agents pushed by priority inheritance had to accept
//     random    : uniform float from a seeded mt19937, as in their code
//
// Solver names: pibt_hr (paper's recommended HR), pibt_hindrance, pibt_regret,
//               pibt_original (both terms off: their baseline PIBT)
class PIBT_HR {
private:
    std::mt19937 MT {};

    // regret[a][k] = learned regret of agent a taking its k-th candidate move this timestep
    std::vector<std::array<float, 32>> regret {};

    // Returns {valid, regret}, regret = extra distance accepted by this agent and every agent it pushed
    template <typename ExpanderType, typename HeuristicType>
    __attribute__((noinline))
    std::pair<bool, int> solve_one_agent(int a_i, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                         const HeuristicType &heuristic, std::vector<int>& occupied_at_t, std::vector<int>& claimed_next) {
        // CRITICAL FIX: If the agent hasn't spawned yet, safely pad its path and exit
        // without attempting any array lookups or grid operations.
        if (paths[a_i][t].x == -1) {
            paths[a_i].push_back(paths[a_i][t]);
            return {false, 0};
        }

        // Get neighbours. Index 0 is the wait move, the rest are moves (expander order)
        static_assert(expander.max_neighbours <= 32, "regret table too small");
        std::array<Location, expander.max_neighbours> neighbours;
        int valid_neighbours = 0;
        for (const Location &neighbor : expander.get_neighbours(paths[a_i][t], env)) {
            neighbours[valid_neighbours] = neighbor;
            valid_neighbours++;
        }

        // Agents standing next to a_i, for hindrance
        std::array<int, expander.max_neighbours> neighbour_agents;
        int num_neighbour_agents = 0;
        if (use_hindrance) {
            for (int k = 0; k < valid_neighbours; k++) {
                int a_k = occupied_at_t[env.get_index(neighbours[k])];
                if (a_k != -1 && a_k != a_i) {
                    neighbour_agents[num_neighbour_agents] = a_k;
                    num_neighbour_agents++;
                }
            }
        }

        // Sort key of every candidate: <distance to goal, hindrance, regret, random>
        auto get_candidate_key = [&](int k) {
            int hindrance = 0;
            for (int n_k = 0; n_k < num_neighbour_agents; n_k++) {
                int a_k = neighbour_agents[n_k];
                if (!(paths[a_k][t] == neighbours[k])
                    && heuristic.get_h_value(neighbours[k], instance.goals[a_k]) < heuristic.get_h_value(paths[a_k][t], instance.goals[a_k])) {
                    hindrance++;
                }
            }
            return std::make_tuple(heuristic.get_h_value(neighbours[k], instance.goals[a_i]), (float)hindrance,
                                   regret[a_i][k], std::uniform_real_distribution<float>(0.0f, 1.0f)(MT));
        };

        // Random numbers are drawn for the moves first and the wait last, matching their candidate order
        std::array<std::tuple<int, float, float, float>, expander.max_neighbours> keys;
        for (int k = 1; k < valid_neighbours; k++) {
            keys[k] = get_candidate_key(k);
        }
        keys[0] = get_candidate_key(0);

        std::array<int, expander.max_neighbours> order;
        std::iota(order.begin(), order.begin() + valid_neighbours, 0);
        std::sort(order.begin(), order.begin() + valid_neighbours, [&](int a, int b) {
            return keys[a] < keys[b];
        });

        int best_h = std::get<0>(keys[order[0]]);

        for (int i = 0; i < valid_neighbours; i++) {
            int k = order[i];
            int n_idx = env.get_index(neighbours[k]);

            // O(1) VERTEX CONFLICT CHECK
            if (claimed_next[n_idx] != -1) {
                continue;
            }

            // Swap conflict with an agent already moving into a_i's cell
            int occupier = occupied_at_t[n_idx];
            if (occupier != -1 && occupier != a_i && paths[occupier].size() > t + 1 && paths[occupier][t+1] == paths[a_i][t]) {
                continue;
            }

            // Speculatively claim the location
            paths[a_i].push_back(neighbours[k]);
            claimed_next[n_idx] = a_i;
            int pushed_regret = 0;

            // O(1) PRIORITY INHERITANCE CHECK
            if (occupier != -1 && !reached_goal[occupier] && paths[occupier].size() == t + 1) {
                auto [occupier_valid, occupier_regret] = solve_one_agent(occupier, t, env, instance, expander, heuristic, occupied_at_t, claimed_next);
                pushed_regret = occupier_regret;

                // Blend the regret this move caused into the running estimate
                regret[a_i][k] = (1.0f - regret_weight) * regret[a_i][k] + regret_weight * occupier_regret;

                if (!occupier_valid) {
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
            return {true, pushed_regret + std::get<0>(keys[k]) - best_h};
        }

        // Agent failed all moves and is forced to wait in place.
        paths[a_i].push_back(paths[a_i][t]);
        claimed_next[env.get_index(paths[a_i][t])] = a_i;
        return {false, std::get<0>(keys[0]) - best_h};
    }

public:
    std::vector<std::vector<Location>> paths {};
    std::vector<bool> reached_goal {};
    int n {};

    // Options (their defaults: hindrance on, m = 3, w = 0.9, seed 0)
    bool use_hindrance {true};
    int regret_iterations {3};   // extra PIBT passes per timestep used to learn regret (0 = no regret)
    float regret_weight {0.9f};  // weight of the newest regret value
    int seed {0};

    // Set options from a solver name
    void set_options(const std::string &name) {
        if (name == "pibt_hr") {
            use_hindrance = true;
            regret_iterations = 3;
        } else if (name == "pibt_hindrance") {
            use_hindrance = true;
            regret_iterations = 0;
        } else if (name == "pibt_regret") {
            use_hindrance = false;
            regret_iterations = 3;
        } else if (name == "pibt_original") {
            use_hindrance = false;
            regret_iterations = 0;
        } else {
            throw std::invalid_argument("Unknown PIBT_HR solver name: " + name);
        }
    }

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
        std::vector<float> priorities(n, 0.0f);
        std::vector<bool> priority_set(n, false);
        paths.resize(n);
        reached_goal.assign(n, false);
        regret.assign(n, {});
        MT.seed(seed);

        for (int i = 0; i < n; i++) {
            paths[i].insert(paths[i].begin(), instance.start_times[i], {-1, -1, -1});
            paths[i].push_back(instance.starts[i]);
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
                    auto [start_loc, new_start_time] = planner.plan_one_agent_start(t, occupied_at_t, env.get_pads(), env);

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

                // Initial priority once the agent is in the air: dist(start, goal) / 10000
                if (!priority_set[a] && paths[a][t].x != -1) {
                    priorities[a] = heuristic.get_h_value(paths[a][t], instance.goals[a]) / 10000.0f;
                    priority_set[a] = true;
                }
            }

            std::vector<int> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);
            std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](int left, int right) {
                return priorities[left] > priorities[right];
            });

            // ==========================================
            // REGRET LEARNING: extra PIBT passes, undone afterwards
            // ==========================================
            for (int i = 0; i < n; i++) {
                regret[i].fill(0.0f);
            }

            if (regret_iterations > 0) {
                std::vector<int> claimed_next_start = claimed_next;
                std::vector<bool> reached_goal_start = reached_goal;
                std::vector<size_t> path_len_start(n);
                for (int i = 0; i < n; i++) {
                    path_len_start[i] = paths[i].size();
                }

                for (int iteration = 0; iteration < regret_iterations; iteration++) {
                    for (const int a_i : sorted_priorities) {
                        if (paths[a_i].size() < t+2 && !reached_goal[a_i]) {
                            solve_one_agent(a_i, t, env, instance, expander, heuristic, occupied_at_t, claimed_next);
                        }
                    }

                    // Undo this pass, keeping only the learned regret
                    for (int i = 0; i < n; i++) {
                        paths[i].resize(path_len_start[i]);
                    }
                    claimed_next = claimed_next_start;
                    reached_goal = reached_goal_start;
                }
            }

            // ==========================================
            // EXECUTE PIBT LOOP
            // ==========================================
            for (const int a_i : sorted_priorities) {
                if (paths[a_i].size() < t+2 && !reached_goal[a_i]) {
                    solve_one_agent(a_i, t, env, instance, expander, heuristic, occupied_at_t, claimed_next);
                }
            }

            // Update priorities: +1 for every agent still away from its goal (their planner)
            for (int i = 0; i < n; i++) {
                if (priority_set[i] && !reached_goal[i] && paths[i].size() > t + 1 && paths[i][t].x != -1) {
                    priorities[i] += 1.0f;
                }
            }
        }

        metrics.solved = false;
        metrics.runtime_us = timer.elapsed_microseconds();
        metrics.paths = paths;
        return false;
    }
};

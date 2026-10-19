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
#include <string>
#include <sstream>
#include <stdexcept>
#include <climits>

// ==========================================
// PULL_IBT_window
// ==========================================
// Every `window` timesteps agents are ranked by the length of the chain of pullers behind them
// (longest chain first, ties by the usual PIBT order). That ranking is then frozen until the next
// recompute. Agents that spawn mid-window go after all ranked agents until the next recompute.
//
// A puller of agent a is an agent standing next to a whose distance to goal would drop by
// stepping into a's cell.
//
// Solver names: pull2_win<M>[_train|_count][_old][_age<N>][_d<N>]
//     win<M> : recompute the ranking every M timesteps (default 5)
//     train  : chain = longest line of pullers behind the agent (default)
//     count  : chain = number of distinct agents queued behind the agent
//     old    : order by frustration first, distance to goal second
//     age<N> : agents with frustration >= N at recompute time go first (most frustrated first)
//     d<N>   : depth limit for "count" (default 16)
class PULL_IBT_window {
private:
    std::vector<int> visited {};
    int visit_id {};

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

    // Options (see set_options)
    int window {5};
    bool count_queue {false};
    bool oldest_first {false};
    int age_threshold {0};
    int max_depth {16};

    // Set options from a solver name, e.g. "pull2_win20_age150"
    void set_options(const std::string &name) {
        std::vector<std::string> tokens {};
        std::stringstream ss(name);
        std::string token;
        while (std::getline(ss, token, '_')) {
            tokens.push_back(token);
        }

        if (tokens.size() < 2 || tokens[0] != "pull2" || tokens[1].rfind("win", 0) != 0) {
            throw std::invalid_argument("Not a pull2_win solver name: " + name);
        }
        if (tokens[1].size() > 3) window = std::stoi(tokens[1].substr(3));
        if (window < 1) throw std::invalid_argument("window must be >= 1 in " + name);

        for (size_t i = 2; i < tokens.size(); i++) {
            const std::string &option = tokens[i];
            if (option == "old") oldest_first = true;
            else if (option == "count") count_queue = true;
            else if (option == "train") count_queue = false;
            else if (option.rfind("age", 0) == 0) age_threshold = std::stoi(option.substr(3));
            else if (option.rfind("d", 0) == 0 && option.size() > 1) max_depth = std::stoi(option.substr(1));
            else throw std::invalid_argument("Unknown option '" + option + "' in " + name);
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
        std::vector<int> frustration(n, 0);
        std::vector<float> current_h_vals(n, 0.0f);
        std::vector<int> frozen_rank(n, -1); // Rank from the last recompute (-1 = unranked)
        paths.resize(n);
        reached_goal.assign(n, false);

        visited.assign(n, 0);
        visit_id = 0;
        int window_len = std::max(1, window);

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

            // Usual PIBT order, used to break ties
            auto pibt_order = [&](size_t left, size_t right) -> bool {
                if (!oldest_first) {
                    // 1. Primary: Distance to goal
                    if (std::abs(current_h_vals[left] - current_h_vals[right]) > 1e-5f) {
                        return current_h_vals[left] < current_h_vals[right];
                    }
                    // 2. Secondary: Frustration
                    if (frustration[left] != frustration[right]) {
                        return frustration[left] > frustration[right];
                    }
                    // 3. Tertiary: ID
                    return left < right;
                }

                // 1. Primary: Frustration
                if (frustration[left] != frustration[right]) {
                    return frustration[left] > frustration[right];
                }
                // 2. Secondary: Distance to goal
                if (std::abs(current_h_vals[left] - current_h_vals[right]) > 1e-5f) {
                    return current_h_vals[left] < current_h_vals[right];
                }
                // 3. Tertiary: ID
                return left < right;
            };

            std::vector<size_t> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);

            // ==========================================
            // RECOMPUTE CHAIN RANKING EVERY WINDOW
            // ==========================================
            if (t % window_len == 0) {
                auto is_active = [&](int i) {
                    return !reached_goal[i] && paths[i].size() > t && paths[i][t].x != -1;
                };

                // pullers[a] = agents next to a that would get closer to their goal by stepping into a's cell
                std::vector<std::vector<int>> pullers(n);
                for (int a = 0; a < n; a++) {
                    if (!is_active(a)) continue;

                    Location curr_loc = paths[a][t];
                    for (const Location &neighbor_loc : expander.get_neighbours(curr_loc, env)) {
                        int puller_id = occupied_at_t[env.get_index(neighbor_loc)];
                        if (puller_id == -1 || puller_id == a || reached_goal[puller_id]) continue;

                        float puller_current_h = current_h_vals[puller_id];
                        float puller_next_h = heuristic.get_h_value(curr_loc, instance.goals[puller_id]);

                        if (puller_current_h - puller_next_h > 1e-5f) {
                            pullers[a].push_back(puller_id);
                        }
                    }
                }

                // Number of agents behind each agent (0 if nobody is behind it)
                std::vector<int> chain_length(n, 0);

                if (!count_queue) {
                    // Longest line of pullers behind the agent. Cycles are cut where they close,
                    // so inside a cycle the value depends on evaluation order.
                    std::vector<int> train_length(n, -1);
                    std::vector<bool> in_stack(n, false);

                    std::function<int(int)> get_train_length = [&](int curr) -> int {
                        if (train_length[curr] != -1) return train_length[curr];
                        if (in_stack[curr]) return 0;

                        in_stack[curr] = true;
                        int max_puller_length = 0;
                        for (int puller_id : pullers[curr]) {
                            max_puller_length = std::max(max_puller_length, get_train_length(puller_id));
                        }
                        in_stack[curr] = false;

                        return train_length[curr] = 1 + max_puller_length;
                    };

                    for (int i = 0; i < n; i++) {
                        if (is_active(i)) chain_length[i] = get_train_length(i) - 1;
                    }
                } else {
                    // Distinct agents queued behind (breadth first, capped at max_depth)
                    std::vector<int> frontier {};
                    std::vector<int> next_frontier {};

                    for (int i = 0; i < n; i++) {
                        if (!is_active(i)) continue;

                        visit_id++;
                        visited[i] = visit_id;
                        frontier.assign(1, i);
                        int depth = 0;
                        int count = 0;

                        while (!frontier.empty() && depth < max_depth) {
                            next_frontier.clear();
                            for (int curr : frontier) {
                                for (int puller_id : pullers[curr]) {
                                    if (visited[puller_id] == visit_id) continue;
                                    visited[puller_id] = visit_id;
                                    next_frontier.push_back(puller_id);
                                    count++;
                                }
                            }
                            depth++;
                            frontier.swap(next_frontier);
                        }
                        chain_length[i] = count;
                    }
                }

                std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](size_t left, size_t right) {
                    // 1. Primary: Agents past the age threshold, most frustrated first
                    if (age_threshold > 0) {
                        bool left_aged = frustration[left] >= age_threshold;
                        bool right_aged = frustration[right] >= age_threshold;
                        if (left_aged != right_aged) {
                            return left_aged;
                        }
                        if (left_aged && right_aged) {
                            if (frustration[left] != frustration[right]) {
                                return frustration[left] > frustration[right];
                            }
                            return left < right;
                        }
                    }
                    // 2. Secondary: Longest chain
                    if (chain_length[left] != chain_length[right]) {
                        return chain_length[left] > chain_length[right];
                    }
                    // 3. Tertiary: Usual PIBT order
                    return pibt_order(left, right);
                });

                std::fill(frozen_rank.begin(), frozen_rank.end(), -1);
                for (size_t k = 0; k < sorted_priorities.size(); k++) {
                    int i = sorted_priorities[k];
                    if (is_active(i)) frozen_rank[i] = k;
                }
            }

            // ==========================================
            // FROZEN SORTING
            // ==========================================
            std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](size_t left, size_t right) {
                // 1. Primary: Frozen rank (agents without one go last)
                int left_rank = frozen_rank[left] >= 0 ? frozen_rank[left] : INT_MAX;
                int right_rank = frozen_rank[right] >= 0 ? frozen_rank[right] : INT_MAX;
                if (left_rank != right_rank) {
                    return left_rank < right_rank;
                }
                // 2. Secondary: Usual PIBT order
                return pibt_order(left, right);
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

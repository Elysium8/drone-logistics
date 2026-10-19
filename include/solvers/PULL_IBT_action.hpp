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
#include <sstream>
#include <stdexcept>
#include <climits>

// ==========================================
// PULL_IBT_action
// ==========================================
// Agents are planned in the usual PIBT order. When an agent moves, the cell it just left is
// handed to one of its pullers, who steps into it straight away. That puller's old cell is then
// handed to one of *its* pullers, and so on down the chain.
//
// A puller of agent a is an unplanned agent standing next to a whose distance to goal would
// drop by stepping into a's cell.
//
// Solver names: pull2_act[_old][_long][_cap<N>][_d<N>]
//     old    : order by frustration first, distance to goal second
//     long   : pull the puller with the longest chain behind it (ties by priority)
//     cap<N> : pull at most N agents per moving agent (cap0 = plain PIBT)
//     d<N>   : depth limit when measuring chain length for "long" (default 16)
class PULL_IBT_action {
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

    // Can puller_id step into leader_loc (the cell leader is leaving) this timestep?
    template <typename HeuristicType>
    bool can_pull(int puller_id, int leader, const Location &leader_loc, int t, const MAPFInstance &instance,
                  const HeuristicType &heuristic, const std::vector<float> &current_h_vals) {
        if (puller_id == -1 || puller_id == leader || reached_goal[puller_id]) return false;

        // Already planned this timestep
        if (paths[puller_id].size() != t + 1) return false;

        // Not spawned yet
        if (paths[puller_id][t].x == -1) return false;

        // Would swap places with the leader
        if (paths[leader][t+1] == paths[puller_id][t]) return false;

        // Must get closer to its goal
        float puller_current_h = current_h_vals[puller_id];
        float puller_next_h = heuristic.get_h_value(leader_loc, instance.goals[puller_id]);
        return puller_current_h - puller_next_h > 1e-5f;
    }

    // Number of layers of unplanned pullers queued behind agent a_i (breadth first, capped at max_depth)
    template <typename ExpanderType, typename HeuristicType>
    int get_chain_length(int a_i, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                         const HeuristicType &heuristic, const std::vector<int> &occupied_at_t, const std::vector<float> &current_h_vals) {
        visit_id++;
        visited[a_i] = visit_id;

        std::vector<int> frontier {a_i};
        std::vector<int> next_frontier {};
        int depth = 0;

        while (!frontier.empty() && depth < max_depth) {
            next_frontier.clear();

            for (int curr : frontier) {
                Location curr_loc = paths[curr][t];

                for (const Location &neighbor_loc : expander.get_neighbours(curr_loc, env)) {
                    int puller_id = occupied_at_t[env.get_index(neighbor_loc)];

                    if (puller_id == -1 || visited[puller_id] == visit_id) continue;
                    if (reached_goal[puller_id] || paths[puller_id].size() != t + 1 || paths[puller_id][t].x == -1) continue;

                    float puller_current_h = current_h_vals[puller_id];
                    float puller_next_h = heuristic.get_h_value(curr_loc, instance.goals[puller_id]);

                    if (puller_current_h - puller_next_h > 1e-5f) {
                        visited[puller_id] = visit_id;
                        next_frontier.push_back(puller_id);
                    }
                }
            }

            if (!next_frontier.empty()) depth++;
            frontier.swap(next_frontier);
        }
        return depth;
    }

    // a_i has just been planned. Pull its pullers into the freed cells one after another.
    // Returns the number of agents pulled.
    template <typename ExpanderType, typename HeuristicType>
    int pull_chain(int a_i, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                   const HeuristicType &heuristic, std::vector<int> &occupied_at_t, std::vector<int> &claimed_next,
                   const std::vector<int> &priority_rank, const std::vector<float> &current_h_vals) {
        int num_pulled = 0;
        int leader = a_i;

        while (num_pulled < max_pull) {
            // The cell the leader is leaving
            Location freed_loc = paths[leader][t];

            // Leader waited in place, nothing was freed
            if (paths[leader][t+1] == freed_loc) break;

            // Someone else has already claimed the freed cell
            int freed_idx = env.get_index(freed_loc);
            if (claimed_next[freed_idx] != -1) break;

            // Pick which puller gets the freed cell
            int best_puller = -1;
            int best_rank = INT_MAX;
            int best_length = -1;

            for (const Location &neighbor_loc : expander.get_neighbours(freed_loc, env)) {
                int puller_id = occupied_at_t[env.get_index(neighbor_loc)];
                if (!can_pull(puller_id, leader, freed_loc, t, instance, heuristic, current_h_vals)) continue;

                if (pull_longest) {
                    // 1. Primary: Longest chain behind the puller
                    // 2. Secondary: Highest priority
                    int length = get_chain_length(puller_id, t, env, instance, expander, heuristic, occupied_at_t, current_h_vals);
                    if (length > best_length || (length == best_length && priority_rank[puller_id] < best_rank)) {
                        best_puller = puller_id;
                        best_length = length;
                        best_rank = priority_rank[puller_id];
                    }
                } else if (priority_rank[puller_id] < best_rank) {
                    // Highest priority puller
                    best_puller = puller_id;
                    best_rank = priority_rank[puller_id];
                }
            }

            if (best_puller == -1) break;

            // Move the puller into the freed cell
            paths[best_puller].push_back(freed_loc);
            claimed_next[freed_idx] = best_puller;

            if (freed_loc == instance.goals[best_puller]) {
                reached_goal[best_puller] = true;
                for (int d = 0; d < instance.delays[best_puller]; ++d) {
                    paths[best_puller].push_back(freed_loc);
                }
            }

            leader = best_puller;
            num_pulled++;
        }
        return num_pulled;
    }

public:
    std::vector<std::vector<Location>> paths {};
    std::vector<bool> reached_goal {};
    int n {};

    // Options (see set_options)
    bool oldest_first {false};
    bool pull_longest {false};
    int max_pull {INT_MAX};
    int max_depth {16};

    // Diagnostics, reset every solve
    long long num_pull_events {};    // Moves that pulled at least one agent
    long long num_agents_pulled {};  // Total agents moved by pulls

    // Set options from a solver name, e.g. "pull2_act_long_cap5"
    void set_options(const std::string &name) {
        std::vector<std::string> tokens {};
        std::stringstream ss(name);
        std::string token;
        while (std::getline(ss, token, '_')) {
            tokens.push_back(token);
        }

        if (tokens.size() < 2 || tokens[0] != "pull2" || tokens[1] != "act") {
            throw std::invalid_argument("Not a pull2_act solver name: " + name);
        }

        for (size_t i = 2; i < tokens.size(); i++) {
            const std::string &option = tokens[i];
            if (option == "old") oldest_first = true;
            else if (option == "long") pull_longest = true;
            else if (option.rfind("cap", 0) == 0) max_pull = std::stoi(option.substr(3));
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
        std::vector<int> priority_rank(n, 0);
        paths.resize(n);
        reached_goal.assign(n, false);

        visited.assign(n, 0);
        visit_id = 0;
        num_pull_events = 0;
        num_agents_pulled = 0;

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
            // PIBT SORTING
            // ==========================================
            std::vector<size_t> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);

            std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](size_t left, size_t right) {
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
            });

            for (size_t k = 0; k < sorted_priorities.size(); k++) {
                priority_rank[sorted_priorities[k]] = k;
            }

            // ==========================================
            // EXECUTE PIBT LOOP WITH PULL CHAINS
            // ==========================================
            for (const int a_i : sorted_priorities) {
                if (paths[a_i].size() < t+2 && !reached_goal[a_i]) {
                    bool moved = solve_one_agent(a_i, -1, t, env, instance, expander, heuristic, occupied_at_t, claimed_next);

                    if (moved && max_pull > 0) {
                        int num_pulled = pull_chain(a_i, t, env, instance, expander, heuristic, occupied_at_t, claimed_next, priority_rank, current_h_vals);
                        if (num_pulled > 0) {
                            num_pull_events++;
                            num_agents_pulled += num_pulled;
                        }
                    }
                }
            }
        }

        metrics.solved = false;
        metrics.runtime_us = timer.elapsed_microseconds();
        metrics.paths = paths;
        return false;
    }
};

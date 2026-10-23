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
#include <tuple>

// ==========================================
// PULL_IBT_HR
// ==========================================
// Pull-chain ideas combined with the PIBT of Okumura & Nagai 2025 (see PIBT_HR.hpp): their
// agent ordering and move preference <distance, hindrance, random>, with
//     - a choice of how agents that have waited equally long are ordered (closest, farthest,
//       longest train of pullers behind them), or the frozen chain ranking of PULL_IBT_window
//     - yield: waiting costs one extra step when an unplanned agent wants your cell
//     - the pull action of PULL_IBT_action
//
// A puller of agent a is an agent standing next to a whose distance to goal would drop by
// stepping into a's cell. Waiting time = timesteps spent away from the goal since spawning.
//
// Solver names: phr_<order>[_age<N>][_h][_y][_r][_act][_seed<N>]
//     close  : longest waiting first, ties closest to goal first (pibt_frustration's convention)
//     far    : longest waiting first, ties farthest from goal first (their PIBT)
//     chain  : longest waiting first, ties longest train behind, then farthest
//     win<M> : PULL_IBT_window ranking (longest train first) recomputed every M timesteps,
//              ties by the "far" order
//     age<N> : (win) agents that have waited >= N timesteps at recompute form a top tier
//     h      : hindrance tiebreak
//     y      : yield
//     r      : rescue: if an unplanned agent is stuck in a dead end whose only exit is your cell, step
//              aside instead of waiting and pull it out into the cell you leave
//     act    : pull action (movers pull their pullers into the cell they leave)
//     seed<N>: seed for the random tiebreak (default 0)
class PULL_IBT_HR {
private:
    std::mt19937 MT {};

    template <typename ExpanderType, typename HeuristicType>
    __attribute__((noinline))
    bool solve_one_agent(int a_i, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                         const HeuristicType &heuristic, std::vector<int>& occupied_at_t, std::vector<int>& claimed_next) {
        // CRITICAL FIX: If the agent hasn't spawned yet, safely pad its path and exit
        // without attempting any array lookups or grid operations.
        if (paths[a_i][t].x == -1) {
            paths[a_i].push_back(paths[a_i][t]);
            return false;
        }

        // Get neighbours. Index 0 is the wait move, the rest are moves (expander order)
        std::array<Location, expander.max_neighbours> neighbours;
        int valid_neighbours = 0;
        for (const Location &neighbor : expander.get_neighbours(paths[a_i][t], env)) {
            neighbours[valid_neighbours] = neighbor;
            valid_neighbours++;
        }

        // Agents standing next to a_i, and whether any unplanned one is waiting for a_i's cell
        std::array<int, expander.max_neighbours> neighbour_agents;
        int num_neighbour_agents = 0;
        bool yielding = false;
        int trapped_agent = -1;
        for (int k = 1; k < valid_neighbours; k++) {
            int a_k = occupied_at_t[env.get_index(neighbours[k])];
            if (a_k == -1) continue;
            neighbour_agents[num_neighbour_agents] = a_k;
            num_neighbour_agents++;

            if (use_rescue && trapped_agent == -1 && is_trapped_behind(a_k, a_i, t, env, instance, expander, heuristic)) {
                trapped_agent = a_k;
            }

            if (use_yield && !reached_goal[a_k] && paths[a_k].size() == t + 1
                && heuristic.get_h_value(paths[a_i][t], instance.goals[a_k]) < heuristic.get_h_value(paths[a_k][t], instance.goals[a_k])) {
                yielding = true;
            }
        }

        // Sort key of every candidate: <distance to goal (+ yield cost), hindrance, random>
        auto get_candidate_key = [&](int k) {
            int dist = heuristic.get_h_value(neighbours[k], instance.goals[a_i]);
            if (yielding) {
                // Waiting blocks the queue behind; stepping into another agent pushes it around
                if (k == 0 || occupied_at_t[env.get_index(neighbours[k])] != -1) dist++;
            }
            if (trapped_agent != -1 && k == 0) {
                // Someone can only get out through a_i's cell: any move beats waiting
                dist += 2;
            }

            int hindrance = 0;
            if (use_hindrance) {
                for (int n_k = 0; n_k < num_neighbour_agents; n_k++) {
                    int a_k = neighbour_agents[n_k];
                    if (!(paths[a_k][t] == neighbours[k])
                        && heuristic.get_h_value(neighbours[k], instance.goals[a_k]) < heuristic.get_h_value(paths[a_k][t], instance.goals[a_k])) {
                        hindrance++;
                    }
                }
            }
            return std::make_tuple(dist, hindrance, std::uniform_real_distribution<float>(0.0f, 1.0f)(MT));
        };

        // Random numbers are drawn for the moves first and the wait last, as in PIBT_HR
        std::array<std::tuple<int, int, float>, expander.max_neighbours> keys;
        for (int k = 1; k < valid_neighbours; k++) {
            keys[k] = get_candidate_key(k);
        }
        keys[0] = get_candidate_key(0);

        std::array<int, expander.max_neighbours> order;
        std::iota(order.begin(), order.begin() + valid_neighbours, 0);
        std::sort(order.begin(), order.begin() + valid_neighbours, [&](int a, int b) {
            return keys[a] < keys[b];
        });

        for (int i = 0; i < valid_neighbours; i++) {
            int k = order[i];
            int n_idx = env.get_index(neighbours[k]);

            // Rescue: pushing into the trapped agent's dead end can only fail (its one exit is a_i's cell)
            if (trapped_agent != -1 && neighbours[k] == paths[trapped_agent][t]) {
                continue;
            }

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

            // O(1) PRIORITY INHERITANCE CHECK
            if (occupier != -1 && !reached_goal[occupier] && paths[occupier].size() == t + 1) {
                if (!solve_one_agent(occupier, t, env, instance, expander, heuristic, occupied_at_t, claimed_next)) {
                    paths[a_i].pop_back();
                    continue;
                }
            }

            // Rescue: pull the trapped agent out into the cell a_i just left
            Location curr_loc = paths[a_i][t];
            int curr_idx = env.get_index(curr_loc);
            if (trapped_agent != -1 && k != 0 && !reached_goal[trapped_agent] && paths[trapped_agent].size() == t + 1
                && claimed_next[curr_idx] == -1 && !(neighbours[k] == paths[trapped_agent][t])) {
                paths[trapped_agent].push_back(curr_loc);
                claimed_next[curr_idx] = trapped_agent;
                if (curr_loc == instance.goals[trapped_agent]) {
                    reached_goal[trapped_agent] = true;
                    for (int d = 0; d < instance.delays[trapped_agent]; ++d) {
                        paths[trapped_agent].push_back(curr_loc);
                    }
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

    // Is unplanned agent a_k in a dead end whose only exit is a_i's cell, and does it need to come out?
    template <typename ExpanderType, typename HeuristicType>
    bool is_trapped_behind(int a_k, int a_i, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                           const HeuristicType &heuristic) {
        if (reached_goal[a_k] || paths[a_k].size() != t + 1) return false;

        Location a_i_loc = paths[a_i][t];
        Location a_k_loc = paths[a_k][t];
        if (heuristic.get_h_value(a_i_loc, instance.goals[a_k]) >= heuristic.get_h_value(a_k_loc, instance.goals[a_k])) return false;

        // Any open cell other than a_i's is another way out
        for (const Location &neighbor_loc : expander.get_neighbours(a_k_loc, env)) {
            if (!(neighbor_loc == a_k_loc) && !(neighbor_loc == a_i_loc)) return false;
        }
        return true;
    }

    // Could unplanned agent puller_id step into leader_loc (the cell leader is leaving) this timestep?
    template <typename HeuristicType>
    bool can_pull(int puller_id, int leader, const Location &leader_loc, int t, const MAPFInstance &instance,
                  const HeuristicType &heuristic) {
        if (puller_id == -1 || puller_id == leader || reached_goal[puller_id]) return false;
        if (paths[puller_id].size() != t + 1) return false;
        if (paths[puller_id][t].x == -1) return false;
        if (paths[leader][t+1] == paths[puller_id][t]) return false;
        return heuristic.get_h_value(leader_loc, instance.goals[puller_id]) < heuristic.get_h_value(paths[puller_id][t], instance.goals[puller_id]);
    }

    // a_i has just been planned. Pull its pullers into the freed cells one after another (PULL_IBT_action)
    template <typename ExpanderType, typename HeuristicType>
    void pull_chain(int a_i, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                    const HeuristicType &heuristic, std::vector<int> &occupied_at_t, std::vector<int> &claimed_next,
                    const std::vector<int> &priority_rank) {
        int leader = a_i;
        while (true) {
            // The cell the leader is leaving
            Location freed_loc = paths[leader][t];
            if (paths[leader][t+1] == freed_loc) break;
            int freed_idx = env.get_index(freed_loc);
            if (claimed_next[freed_idx] != -1) break;

            // Highest priority puller gets the freed cell
            int best_puller = -1;
            int best_rank = INT_MAX;
            for (const Location &neighbor_loc : expander.get_neighbours(freed_loc, env)) {
                int puller_id = occupied_at_t[env.get_index(neighbor_loc)];
                if (can_pull(puller_id, leader, freed_loc, t, instance, heuristic) && priority_rank[puller_id] < best_rank) {
                    best_puller = puller_id;
                    best_rank = priority_rank[puller_id];
                }
            }
            if (best_puller == -1) break;

            paths[best_puller].push_back(freed_loc);
            claimed_next[freed_idx] = best_puller;
            if (freed_loc == instance.goals[best_puller]) {
                reached_goal[best_puller] = true;
                for (int d = 0; d < instance.delays[best_puller]; ++d) {
                    paths[best_puller].push_back(freed_loc);
                }
            }
            leader = best_puller;
        }
    }

public:
    std::vector<std::vector<Location>> paths {};
    std::vector<bool> reached_goal {};
    int n {};

    enum class Order { CLOSE, FAR, CHAIN, WINDOW };

    // Options (see set_options)
    Order order {Order::FAR};
    int window {20};
    int age_threshold {0};
    bool use_hindrance {false};
    bool use_yield {false};
    bool use_pull {false};
    bool use_rescue {false};
    int seed {0};

    // Set options from a solver name, e.g. "phr_win20_age150_h_y"
    void set_options(const std::string &name) {
        std::vector<std::string> tokens {};
        std::stringstream ss(name);
        std::string token;
        while (std::getline(ss, token, '_')) {
            tokens.push_back(token);
        }
        if (tokens.size() < 2 || tokens[0] != "phr") {
            throw std::invalid_argument("Not a phr solver name: " + name);
        }

        if (tokens[1] == "close") order = Order::CLOSE;
        else if (tokens[1] == "far") order = Order::FAR;
        else if (tokens[1] == "chain") order = Order::CHAIN;
        else if (tokens[1].rfind("win", 0) == 0) {
            order = Order::WINDOW;
            if (tokens[1].size() > 3) window = std::stoi(tokens[1].substr(3));
            if (window < 1) throw std::invalid_argument("window must be >= 1 in " + name);
        }
        else throw std::invalid_argument("Unknown phr order '" + tokens[1] + "' in " + name);

        for (size_t i = 2; i < tokens.size(); i++) {
            const std::string &option = tokens[i];
            if (option == "h") use_hindrance = true;
            else if (option == "y") use_yield = true;
            else if (option == "act") use_pull = true;
            else if (option == "r") use_rescue = true;
            else if (option.rfind("seed", 0) == 0) seed = std::stoi(option.substr(4));
            else if (option.rfind("age", 0) == 0 && order == Order::WINDOW) age_threshold = std::stoi(option.substr(3));
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
        std::vector<int> waiting(n, 0);       // timesteps spent away from the goal since spawning
        std::vector<int> start_h(n, -1);      // distance to goal at spawn (-1 = not spawned yet)
        std::vector<int> chain_length(n, 0);  // longest train of pullers behind each agent
        std::vector<int> frozen_rank(n, -1);  // WINDOW: rank from the last recompute (-1 = unranked)
        std::vector<int> priority_rank(n, 0); // position in this timestep's order
        paths.resize(n);
        reached_goal.assign(n, false);
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

                if (start_h[a] == -1 && paths[a][t].x != -1) {
                    start_h[a] = heuristic.get_h_value(paths[a][t], instance.goals[a]);
                }
            }

            auto is_active = [&](int i) {
                return !reached_goal[i] && paths[i].size() > t && paths[i][t].x != -1;
            };

            // ==========================================
            // TRAIN LENGTHS (only for the chain and window orders)
            // ==========================================
            bool recompute_window = order == Order::WINDOW && t % window == 0;
            if (order == Order::CHAIN || recompute_window) {
                // pullers[a] = agents next to a that would get closer to their goal by stepping into a's cell
                std::vector<std::vector<int>> pullers(n);
                for (int a = 0; a < n; a++) {
                    if (!is_active(a)) continue;

                    Location curr_loc = paths[a][t];
                    for (const Location &neighbor_loc : expander.get_neighbours(curr_loc, env)) {
                        int puller_id = occupied_at_t[env.get_index(neighbor_loc)];
                        if (puller_id == -1 || puller_id == a || reached_goal[puller_id]) continue;
                        if (heuristic.get_h_value(curr_loc, instance.goals[puller_id]) < heuristic.get_h_value(paths[puller_id][t], instance.goals[puller_id])) {
                            pullers[a].push_back(puller_id);
                        }
                    }
                }

                // Longest line of pullers behind each agent (cycles are cut where they close)
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

                std::fill(chain_length.begin(), chain_length.end(), 0);
                for (int i = 0; i < n; i++) {
                    if (is_active(i)) chain_length[i] = get_train_length(i) - 1;
                }
            }

            // ==========================================
            // PRIORITY SORTING
            // ==========================================
            // Longest waiting first, ties by the chosen order, then agent ID
            auto base_order = [&](int left, int right) -> bool {
                if (waiting[left] != waiting[right]) {
                    return waiting[left] > waiting[right];
                }
                if (order == Order::CLOSE) {
                    if (start_h[left] != start_h[right]) return start_h[left] < start_h[right];
                } else if (order == Order::CHAIN) {
                    if (chain_length[left] != chain_length[right]) return chain_length[left] > chain_length[right];
                    if (start_h[left] != start_h[right]) return start_h[left] > start_h[right];
                } else {
                    if (start_h[left] != start_h[right]) return start_h[left] > start_h[right];
                }
                return left < right;
            };

            std::vector<int> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);

            if (order != Order::WINDOW) {
                std::sort(sorted_priorities.begin(), sorted_priorities.end(), base_order);
            } else {
                if (recompute_window) {
                    // Rank: aging tier, then longest train, then base order
                    std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](int left, int right) {
                        if (age_threshold > 0) {
                            bool left_aged = waiting[left] >= age_threshold;
                            bool right_aged = waiting[right] >= age_threshold;
                            if (left_aged != right_aged) {
                                return left_aged;
                            }
                        }
                        if (chain_length[left] != chain_length[right]) {
                            return chain_length[left] > chain_length[right];
                        }
                        return base_order(left, right);
                    });

                    std::fill(frozen_rank.begin(), frozen_rank.end(), -1);
                    for (int k = 0; k < n; k++) {
                        if (is_active(sorted_priorities[k])) frozen_rank[sorted_priorities[k]] = k;
                    }
                }

                // Frozen rank first, agents without one (spawned mid-window) after, by base order
                std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](int left, int right) {
                    int left_rank = frozen_rank[left] >= 0 ? frozen_rank[left] : INT_MAX;
                    int right_rank = frozen_rank[right] >= 0 ? frozen_rank[right] : INT_MAX;
                    if (left_rank != right_rank) {
                        return left_rank < right_rank;
                    }
                    return base_order(left, right);
                });
            }

            for (int k = 0; k < n; k++) {
                priority_rank[sorted_priorities[k]] = k;
            }

            // ==========================================
            // EXECUTE PIBT LOOP
            // ==========================================
            for (const int a_i : sorted_priorities) {
                if (paths[a_i].size() < t+2 && !reached_goal[a_i]) {
                    bool moved = solve_one_agent(a_i, t, env, instance, expander, heuristic, occupied_at_t, claimed_next);
                    if (moved && use_pull) {
                        pull_chain(a_i, t, env, instance, expander, heuristic, occupied_at_t, claimed_next, priority_rank);
                    }
                }
            }

            // Every agent still away from its goal has waited one more timestep
            for (int i = 0; i < n; i++) {
                if (start_h[i] != -1 && !reached_goal[i] && paths[i][t].x != -1) {
                    waiting[i]++;
                }
            }
        }

        metrics.solved = false;
        metrics.runtime_us = timer.elapsed_microseconds();
        metrics.paths = paths;
        return false;
    }
};

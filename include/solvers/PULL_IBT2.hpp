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
#include <cmath>

// ============================================================================
// PULL_IBT2 : two ways of using "pull chains" inside PIBT.
//
// Pull graph: agent p is a *follower* of agent o if p stands next to o's cell
// and stepping into that cell reduces p's distance to its goal.
// A chain of followers behind o is the "train" that can advance if o moves.
//
// VARIANT 1  (PULL_ACTION)  -- priority order untouched, movers yank followers
//   Agents are planned in the usual PIBT order. Whenever an agent is planned
//   and actually moves, the cell it just left is offered to its best unplanned
//   follower, who is pulled into it immediately; that follower's vacated cell
//   is then offered to *its* followers, and so on down the train.
//   Priority inheritance pushes occupants out of the way; this is the mirror
//   image, pulling followers into the space that was just freed.
//   Because an improving move is always optimal progress on a unit-cost grid,
//   a pulled agent loses nothing individually: what changes is that followers
//   follow (overriding their own tie-breaks), so whole queues advance together.
//
// VARIANT 2  (CHAIN_ORDER)  -- chain length decides priority, frozen for M steps
//   Every M timesteps agents are ranked by the length of the train behind them
//   (longest chain first; ties by the base order). That ranking is then kept
//   static for the next M-1 steps. Agents that spawn mid-window are placed after
//   all ranked agents until the next recompute.
//
// Names (for SolverRegistry / --solvers):
//   pull2_act[_old][_long][_cap<N>]
//   pull2_win<M>[_count|_train][_old][_age<N>][_d<N>]
//     old    : base order = oldest (most frustrated) first instead of closest first
//     long   : (act) among several candidate followers, pull the one with the deepest train behind it
//     cap<N> : (act) yank at most N agents per mover (cap0 disables pulling = plain closest-first PIBT)
//     win<M> : (win) recompute the priority order every M timesteps
//     train  : (win) chain = longest train behind the agent (default)
//     count  : (win) chain = number of distinct agents queued behind the agent
//     age<N> : (win) agents with frustration >= N at recompute time form a top tier (oldest first)
//     d<N>   : depth cap for the count measure / train-depth search (default 16)
// ============================================================================
class PULL_IBT2 {
public:
    enum class Variant { PULL_ACTION, CHAIN_ORDER };
    enum class BaseOrder { CLOSEST_FIRST, OLDEST_FIRST };
    enum class ChainMeasure { TRAIN_LENGTH, QUEUE_COUNT };

    struct Config {
        Variant variant = Variant::PULL_ACTION;
        BaseOrder base_order = BaseOrder::CLOSEST_FIRST;  // closest -> frustration -> id (your original order)

        // Variant 1
        bool pull_pick_longest = false;   // choose the follower with the deepest train instead of the highest priority
        int  max_pull_len = INT_MAX;      // most agents one mover may yank (0 = never pull)

        // Variant 2
        int window = 5;                   // M
        ChainMeasure measure = ChainMeasure::TRAIN_LENGTH;
        int aging_threshold = 0;          // >0: frustration >= this -> top tier at recompute time

        int max_depth = 16;               // depth cap for BFS style searches

        static Config from_name(const std::string &name) {
            Config c;
            std::vector<std::string> tok;
            std::stringstream ss(name);
            std::string t;
            while (std::getline(ss, t, '_')) tok.push_back(t);
            if (tok.size() < 2 || tok[0] != "pull2")
                throw std::invalid_argument("Not a pull2 solver name: " + name);

            const std::string &v = tok[1];
            if (v == "act") {
                c.variant = Variant::PULL_ACTION;
            } else if (v.rfind("win", 0) == 0) {
                c.variant = Variant::CHAIN_ORDER;
                if (v.size() > 3) c.window = std::stoi(v.substr(3));
                if (c.window < 1) throw std::invalid_argument("window must be >= 1 in " + name);
            } else {
                throw std::invalid_argument("Unknown pull2 variant: " + v);
            }

            for (size_t i = 2; i < tok.size(); ++i) {
                const std::string &o = tok[i];
                if (o == "old") c.base_order = BaseOrder::OLDEST_FIRST;
                else if (o == "long") c.pull_pick_longest = true;
                else if (o == "count") c.measure = ChainMeasure::QUEUE_COUNT;
                else if (o == "train") c.measure = ChainMeasure::TRAIN_LENGTH;
                else if (o.rfind("cap", 0) == 0) c.max_pull_len = std::stoi(o.substr(3));
                else if (o.rfind("age", 0) == 0) c.aging_threshold = std::stoi(o.substr(3));
                else if (o.rfind("d", 0) == 0 && o.size() > 1) c.max_depth = std::stoi(o.substr(1));
                else throw std::invalid_argument("Unknown option '" + o + "' in " + name);
            }
            return c;
        }
    };

    Config cfg;

    // Diagnostics (reset at the start of every solve)
    long long pull_events = 0;     // movers whose pull chain yanked at least one agent
    long long agents_yanked = 0;   // total agents moved by pulls

private:
    std::vector<int> visit_stamp_;
    int stamp_ = 0;

    // ------------------------------------------------------------------
    // Unchanged PIBT recursion from PULL_IBT
    // ------------------------------------------------------------------
    template <typename ExpanderType, typename HeuristicType>
    __attribute__((noinline))
    bool solve_one_agent(int a_i, int a_j, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander, 
                         const HeuristicType &heuristic, std::vector<int>& occupied_at_t, std::vector<int>& claimed_next) {
        
        // If the agent hasn't spawned yet, safely pad its path and exit 
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

    // ------------------------------------------------------------------
    // VARIANT 1 helpers
    // ------------------------------------------------------------------

    // Could unplanned agent p step into `cell` this timestep as a useful follower of `leader`?
    template <typename HeuristicType>
    bool can_follow(int p, int leader, const Location &cell, int t, const MAPFInstance &instance,
                    const HeuristicType &heuristic, const std::vector<float> &current_h_vals) const {
        if (p == -1 || p == leader || reached_goal[p]) return false;
        if (paths[p].size() != static_cast<size_t>(t) + 1) return false;      // already planned (or not present)
        if (paths[p][t].x == -1) return false;                                 // not spawned
        if (paths[leader][t + 1] == paths[p][t]) return false;                 // would be a swap with the leader
        return current_h_vals[p] - heuristic.get_h_value(cell, instance.goals[p]) > 1e-5f;  // must make progress
    }

    // How many hops the train of unplanned followers extends behind agent p (BFS layers, depth capped)
    template <typename ExpanderType, typename HeuristicType>
    int train_depth(int p, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                    const HeuristicType &heuristic, const std::vector<int> &occupied_at_t,
                    const std::vector<float> &current_h_vals) {
        ++stamp_;
        visit_stamp_[p] = stamp_;
        std::vector<int> frontier{p}, next;
        int depth = 0;
        while (!frontier.empty() && depth < cfg.max_depth) {
            next.clear();
            for (int u : frontier) {
                const Location uloc = paths[u][t];
                for (const Location &nb : expander.get_neighbours(uloc, env)) {
                    const int q = occupied_at_t[env.get_index(nb)];
                    if (q == -1 || visit_stamp_[q] == stamp_) continue;
                    if (reached_goal[q] || paths[q].size() != static_cast<size_t>(t) + 1 || paths[q][t].x == -1) continue;
                    if (current_h_vals[q] - heuristic.get_h_value(uloc, instance.goals[q]) > 1e-5f) {
                        visit_stamp_[q] = stamp_;
                        next.push_back(q);
                    }
                }
            }
            if (!next.empty()) ++depth;
            frontier.swap(next);
        }
        return depth;
    }

    // After `leader` has been planned to move, yank its followers forward one after another.
    // Returns the number of agents pulled.
    template <typename ExpanderType, typename HeuristicType>
    int pull_chain(int leader, int t, const Environment &env, const MAPFInstance &instance, const ExpanderType &expander,
                   const HeuristicType &heuristic, std::vector<int> &occupied_at_t, std::vector<int> &claimed_next,
                   const std::vector<int> &rank_pos, const std::vector<float> &current_h_vals) {
        int yanked = 0;
        int cur = leader;

        while (yanked < cfg.max_pull_len) {
            const Location cell = paths[cur][t];                 // the cell `cur` is leaving
            if (paths[cur][t + 1] == cell) break;                // cur stayed put: nothing was freed
            const int cell_idx = env.get_index(cell);
            if (claimed_next[cell_idx] != -1) break;             // somebody already takes that cell

            int best = -1, best_rank = INT_MAX, best_depth = -1;
            for (const Location &nb : expander.get_neighbours(cell, env)) {
                const int p = occupied_at_t[env.get_index(nb)];
                if (!can_follow(p, cur, cell, t, instance, heuristic, current_h_vals)) continue;

                if (cfg.pull_pick_longest) {
                    const int depth = train_depth(p, t, env, instance, expander, heuristic, occupied_at_t, current_h_vals);
                    if (depth > best_depth || (depth == best_depth && rank_pos[p] < best_rank)) {
                        best = p; best_depth = depth; best_rank = rank_pos[p];
                    }
                } else if (rank_pos[p] < best_rank) {
                    best = p; best_rank = rank_pos[p];
                }
            }
            if (best == -1) break;

            // Commit the pulled agent into the freed cell
            paths[best].push_back(cell);
            claimed_next[cell_idx] = best;
            if (cell == instance.goals[best]) {
                reached_goal[best] = true;
                for (int d = 0; d < instance.delays[best]; ++d) paths[best].push_back(cell);
            }
            cur = best;
            ++yanked;
        }
        return yanked;
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

        std::vector<int> rank_pos(n, 0);       // position in this timestep's order
        std::vector<int> static_rank(n, -1);   // CHAIN_ORDER: rank frozen at the last recompute (-1 = unranked)
        visit_stamp_.assign(n, 0);
        stamp_ = 0;
        pull_events = 0;
        agents_yanked = 0;
        const int M = std::max(1, cfg.window);

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
            // BASE ORDER (the "original" PIBT-style priority)
            // ==========================================
            auto base_less = [&](size_t l, size_t r) -> bool {
                if (cfg.base_order == BaseOrder::CLOSEST_FIRST) {
                    if (std::abs(current_h_vals[l] - current_h_vals[r]) > 1e-5f) return current_h_vals[l] < current_h_vals[r];
                    if (frustration[l] != frustration[r]) return frustration[l] > frustration[r];
                    return l < r;
                }
                if (frustration[l] != frustration[r]) return frustration[l] > frustration[r];
                if (std::abs(current_h_vals[l] - current_h_vals[r]) > 1e-5f) return current_h_vals[l] < current_h_vals[r];
                return l < r;
            };

            std::vector<size_t> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);

            if (cfg.variant == Variant::PULL_ACTION) {
                // VARIANT 1: plain PIBT order; pulls happen while executing it
                std::sort(sorted_priorities.begin(), sorted_priorities.end(), base_less);
            } else {
                // VARIANT 2: order dictated by chain length, recomputed every M timesteps and frozen in between
                if (t % M == 0) {
                    auto is_active = [&](int i) {
                        return !reached_goal[i] && paths[i].size() > static_cast<size_t>(t) && paths[i][t].x != -1;
                    };

                    // Pull graph over all active agents: pullers[o] = followers of o
                    std::vector<std::vector<int>> pullers(n);
                    for (int o = 0; o < n; ++o) {
                        if (!is_active(o)) continue;
                        const Location o_loc = paths[o][t];
                        for (const Location &c : expander.get_neighbours(o_loc, env)) {
                            const int p = occupied_at_t[env.get_index(c)];
                            if (p == -1 || p == o || reached_goal[p]) continue;
                            if (current_h_vals[p] - heuristic.get_h_value(o_loc, instance.goals[p]) > 1e-5f) {
                                pullers[o].push_back(p);
                            }
                        }
                    }

                    // Chain value = number of agents behind each agent (0 for agents with nobody behind them)
                    std::vector<int> chain(n, 0);
                    if (cfg.measure == ChainMeasure::TRAIN_LENGTH) {
                        // Longest train: longest path through the follower graph (cycles are cut where they close,
                        // so inside a cycle the value depends on evaluation order)
                        std::vector<int> memo(n, -1);
                        std::vector<char> in_stack(n, 0);
                        std::function<int(int)> train = [&](int a) -> int {
                            if (memo[a] != -1) return memo[a];
                            if (in_stack[a]) return 0;
                            in_stack[a] = 1;
                            int best = 0;
                            for (int p : pullers[a]) best = std::max(best, train(p));
                            in_stack[a] = 0;
                            return memo[a] = 1 + best;
                        };
                        for (int i = 0; i < n; ++i) if (is_active(i)) chain[i] = train(i) - 1;
                    } else {
                        // Distinct agents queued behind (BFS, depth capped)
                        std::vector<int> frontier, next;
                        for (int i = 0; i < n; ++i) {
                            if (!is_active(i)) continue;
                            ++stamp_;
                            visit_stamp_[i] = stamp_;
                            frontier.assign(1, i);
                            int depth = 0, count = 0;
                            while (!frontier.empty() && depth < cfg.max_depth) {
                                next.clear();
                                for (int u : frontier) {
                                    for (int p : pullers[u]) {
                                        if (visit_stamp_[p] == stamp_) continue;
                                        visit_stamp_[p] = stamp_;
                                        next.push_back(p);
                                        ++count;
                                    }
                                }
                                ++depth;
                                frontier.swap(next);
                            }
                            chain[i] = count;
                        }
                    }

                    // Rank: aging tier, then longest chain, then base order
                    std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](size_t l, size_t r) {
                        if (cfg.aging_threshold > 0) {
                            const bool sl = frustration[l] >= cfg.aging_threshold;
                            const bool sr = frustration[r] >= cfg.aging_threshold;
                            if (sl != sr) return sl;
                            if (sl && sr) {
                                if (frustration[l] != frustration[r]) return frustration[l] > frustration[r];
                                return l < r;
                            }
                        }
                        if (chain[l] != chain[r]) return chain[l] > chain[r];
                        return base_less(l, r);
                    });

                    std::fill(static_rank.begin(), static_rank.end(), -1);
                    for (size_t k = 0; k < sorted_priorities.size(); ++k) {
                        const int i = static_cast<int>(sorted_priorities[k]);
                        if (is_active(i)) static_rank[i] = static_cast<int>(k);
                    }
                }

                // Frozen order: stored rank first, agents without one (spawned mid-window) after, by base order
                std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](size_t l, size_t r) {
                    const int rl = static_rank[l] >= 0 ? static_rank[l] : INT_MAX;
                    const int rr = static_rank[r] >= 0 ? static_rank[r] : INT_MAX;
                    if (rl != rr) return rl < rr;
                    return base_less(l, r);
                });
            }

            for (size_t k = 0; k < sorted_priorities.size(); ++k) rank_pos[sorted_priorities[k]] = static_cast<int>(k);

            // ==========================================
            // EXECUTE PIBT LOOP (with pull chains for variant 1)
            // ==========================================
            for (const int a_i : sorted_priorities) {
                if (paths[a_i].size() < t+2 && !reached_goal[a_i]) {
                    const bool ok = solve_one_agent(a_i, -1, t, env, instance, expander, heuristic, occupied_at_t, claimed_next);
                    if (ok && cfg.variant == Variant::PULL_ACTION && cfg.max_pull_len > 0) {
                        const int yanked = pull_chain(a_i, t, env, instance, expander, heuristic,
                                                      occupied_at_t, claimed_next, rank_pos, current_h_vals);
                        if (yanked > 0) {
                            ++pull_events;
                            agents_yanked += yanked;
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

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
#include <cmath>

// ============================================================================
// PULL_IBT with configurable "pull chains".
//
// Pull graph: agent p has an edge to agent o if p stands next to o's cell and
// stepping into that cell would reduce p's distance to its goal. The upstream
// set U(o) is everything reachable backwards along those edges, i.e. the agents
// queued up behind o who all benefit if o moves.
//
// Two independent choices:
//   ChainMetric : WHAT number we compute from U(o)
//   ChainUse    : HOW that number feeds into the priority order
//
// Default Config reproduces the original behaviour exactly
// (heaviest-frustration path as a tiebreak after distance to goal).
//
// Runner names (see Config::from_name):
//   pull_ibt                       -> original behaviour
//   pull_<use>_<metric>[_opts]
//     use    : tb | add | ratio | first
//     metric : none | path | maxf | sumf | count | depth | sumh | maxh
//     opts   : age<N>  starvation tier once frustration >= N
//              bk<X>   (tb only) distances within the same X-wide bucket count as tied
//              d<N>    cap upstream search depth at N (default 8)
//   e.g. pull_tb_count, pull_ratio_count, pull_add_sumf_age150, pull_tb_maxh_bk2
// ============================================================================
class PULL_IBT {
public:
    enum class ChainMetric {
        NONE,        // no chain information (plain closest-first)
        PATH_FRUST,  // ORIGINAL: heaviest path of frustration through the chain (includes self)
        MAX_FRUST,   // largest single frustration among upstream agents
        SUM_FRUST,   // total frustration of all distinct upstream agents
        COUNT,       // number of distinct upstream agents (queue size behind me)
        DEPTH,       // how many "hops" the queue extends (chain length)
        SUM_H,       // total distance-to-goal of upstream agents
        MAX_H        // largest distance-to-goal among upstream agents
    };

    enum class ChainUse {
        TIEBREAK,    // sort by h; chain only breaks ties (optionally within an h-bucket)
        ADDITIVE,    // sort by  h - w*chain                       (lower first)
        RATIO,       // sort by  (1 + w*chain) / (h + c)           (higher first, Smith's rule)
        CHAIN_FIRST  // sort by chain (larger first), then h
    };

    struct Config {
        ChainMetric metric = ChainMetric::PATH_FRUST;
        ChainUse use = ChainUse::TIEBREAK;
        int    max_depth = 8;          // cap on upstream search depth (set-based metrics)
        double chain_weight = 1.0;     // w in ADDITIVE / RATIO
        double chain_cap = 1e18;       // chain values are clamped to this before use in ADDITIVE / RATIO
        double ratio_c = 1.0;          // c in RATIO, keeps h=0..1 agents from dominating
        float  h_bucket = 0.0f;        // TIEBREAK only: 0 = exact ties, >0 = bucket width
        int    aging_threshold = 0;    // >0: agents with frustration >= this go first (oldest first)

        static Config from_name(const std::string &name) {
            Config c;
            std::vector<std::string> tok;
            std::stringstream ss(name);
            std::string t;
            while (std::getline(ss, t, '_')) tok.push_back(t);
            if (tok.size() < 3) return c;  // "pull_ibt" -> original behaviour

            const std::string &u = tok[1];
            if (u == "tb") c.use = ChainUse::TIEBREAK;
            else if (u == "add") c.use = ChainUse::ADDITIVE;
            else if (u == "ratio") c.use = ChainUse::RATIO;
            else if (u == "first") c.use = ChainUse::CHAIN_FIRST;
            else throw std::invalid_argument("Unknown chain use: " + u);

            const std::string &m = tok[2];
            if (m == "none") c.metric = ChainMetric::NONE;
            else if (m == "path") c.metric = ChainMetric::PATH_FRUST;
            else if (m == "maxf") c.metric = ChainMetric::MAX_FRUST;
            else if (m == "sumf") c.metric = ChainMetric::SUM_FRUST;
            else if (m == "count") c.metric = ChainMetric::COUNT;
            else if (m == "depth") c.metric = ChainMetric::DEPTH;
            else if (m == "sumh") c.metric = ChainMetric::SUM_H;
            else if (m == "maxh") c.metric = ChainMetric::MAX_H;
            else throw std::invalid_argument("Unknown chain metric: " + m);

            // Untuned starting weights. Counts/depths are small integers; the
            // frustration/distance metrics are in timesteps, so scale them down.
            const bool small_scale = (c.metric == ChainMetric::COUNT || c.metric == ChainMetric::DEPTH);
            const double scale = small_scale ? 1.0 : 0.02;
            if (c.use == ChainUse::ADDITIVE) c.chain_weight = 0.5 * scale;
            else if (c.use == ChainUse::RATIO) c.chain_weight = 1.0 * scale;

            for (size_t i = 3; i < tok.size(); ++i) {
                if (tok[i].rfind("age", 0) == 0) c.aging_threshold = std::stoi(tok[i].substr(3));
                else if (tok[i].rfind("bk", 0) == 0) c.h_bucket = std::stof(tok[i].substr(2));
                else if (tok[i].rfind("d", 0) == 0) c.max_depth = std::stoi(tok[i].substr(1));
                else throw std::invalid_argument("Unknown option: " + tok[i]);
            }
            return c;
        }
    };

    Config cfg;

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

        // Scratch buffers for the pull graph (reused every timestep)
        std::vector<std::vector<int>> pullers(n);   // pullers[o] = agents who want o's cell
        std::vector<int> built_at(n, -1);           // timestep at which pullers[o] was last built
        std::vector<int> visit_stamp(n, 0);
        int stamp = 0;
        std::vector<int> frontier, next_frontier;
        std::vector<double> chain_val(n, -1.0);
        std::vector<int> path_memo(n, -1);
        std::vector<char> in_stack(n, 0);

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
            // PULL GRAPH (built lazily, per agent, only when a chain is needed)
            // pullers[o] = agents standing next to o whose goal-distance
            // would shrink by stepping into o's current cell.
            // ==========================================
            const bool need_chain = (cfg.metric != ChainMetric::NONE);
            if (need_chain) {
                std::fill(chain_val.begin(), chain_val.end(), -1.0);
                std::fill(path_memo.begin(), path_memo.end(), -1);
            }

            auto ensure_pullers = [&](int o) {
                if (built_at[o] == t) return;
                built_at[o] = t;
                pullers[o].clear();
                if (paths[o].empty()) return;
                const Location o_loc = (t < static_cast<int>(paths[o].size())) ? paths[o][t] : paths[o].back();
                if (o_loc.x == -1) return;  // not spawned
                for (const Location &c : expander.get_neighbours(o_loc, env)) {
                    const int p = occupied_at_t[env.get_index(c)];
                    if (p == -1 || p == o || reached_goal[p]) continue;
                    if (current_h_vals[p] - heuristic.get_h_value(o_loc, instance.goals[p]) > 1e-5f) {
                        pullers[o].push_back(p);
                    }
                }
            };

            // ==========================================
            // CHAIN METRICS (lazy, cached per agent per timestep)
            // ==========================================

            // ORIGINAL metric: heaviest path of frustration through the pull DAG (includes self).
            // Note: with cycles in the pull graph the memoised value depends on evaluation order.
            std::function<int(int)> path_frust = [&](int cur) -> int {
                if (path_memo[cur] != -1) return path_memo[cur];
                if (in_stack[cur]) return 0;
                in_stack[cur] = 1;
                int best = 0;
                ensure_pullers(cur);
                for (int p : pullers[cur]) best = std::max(best, path_frust(p));
                in_stack[cur] = 0;
                return path_memo[cur] = frustration[cur] + best;
            };

            // Set-based metrics: BFS over distinct upstream agents (self excluded), depth-capped.
            struct ChainStats { int count = 0; int depth = 0; double sum_f = 0; int max_f = 0; double sum_h = 0; float max_h = 0; };
            auto upstream_stats = [&](int a) -> ChainStats {
                ChainStats s;
                ++stamp;
                visit_stamp[a] = stamp;
                frontier.assign(1, a);
                int depth = 0;
                while (!frontier.empty() && depth < cfg.max_depth) {
                    next_frontier.clear();
                    for (int u : frontier) {
                        ensure_pullers(u);
                        for (int p : pullers[u]) {
                            if (visit_stamp[p] == stamp) continue;
                            visit_stamp[p] = stamp;
                            next_frontier.push_back(p);
                            s.count++;
                            s.sum_f += frustration[p];
                            s.max_f = std::max(s.max_f, frustration[p]);
                            s.sum_h += current_h_vals[p];
                            s.max_h = std::max(s.max_h, current_h_vals[p]);
                        }
                    }
                    if (!next_frontier.empty()) ++depth;
                    frontier.swap(next_frontier);
                }
                s.depth = depth;
                return s;
            };

            auto compute_chain = [&](int a) -> double {
                if (cfg.metric == ChainMetric::PATH_FRUST) return static_cast<double>(path_frust(a));
                const ChainStats s = upstream_stats(a);
                switch (cfg.metric) {
                    case ChainMetric::MAX_FRUST: return s.max_f;
                    case ChainMetric::SUM_FRUST: return s.sum_f;
                    case ChainMetric::COUNT:     return s.count;
                    case ChainMetric::DEPTH:     return s.depth;
                    case ChainMetric::SUM_H:     return s.sum_h;
                    case ChainMetric::MAX_H:     return s.max_h;
                    default:                     return 0.0;
                }
            };

            auto chain_of = [&](int a) -> double {
                if (!need_chain) return 0.0;
                if (chain_val[a] < 0.0) chain_val[a] = compute_chain(a);
                return chain_val[a];
            };

            // ==========================================
            // PRIORITY ORDER
            // ==========================================
            std::vector<size_t> sorted_priorities(n);
            std::iota(sorted_priorities.begin(), sorted_priorities.end(), 0);

            std::sort(sorted_priorities.begin(), sorted_priorities.end(), [&](size_t l, size_t r) {
                // 0. Optional starvation tier: very frustrated agents go first, oldest first.
                if (cfg.aging_threshold > 0) {
                    const bool sl = frustration[l] >= cfg.aging_threshold;
                    const bool sr = frustration[r] >= cfg.aging_threshold;
                    if (sl != sr) return sl;
                    if (sl && sr) {
                        if (frustration[l] != frustration[r]) return frustration[l] > frustration[r];
                        return l < r;
                    }
                }

                const float hl = current_h_vals[l], hr = current_h_vals[r];

                switch (cfg.use) {
                    case ChainUse::TIEBREAK: {
                        if (cfg.h_bucket <= 0.0f) {
                            if (std::abs(hl - hr) > 1e-5f) return hl < hr;
                        } else {
                            const long bl = static_cast<long>(std::floor(hl / cfg.h_bucket));
                            const long br = static_cast<long>(std::floor(hr / cfg.h_bucket));
                            if (bl != br) return bl < br;
                        }
                        // Chains are only evaluated when distance does not decide (lazy, as in the original)
                        const double cl = chain_of(static_cast<int>(l));
                        const double cr = chain_of(static_cast<int>(r));
                        if (cl != cr) return cl > cr;
                        if (std::abs(hl - hr) > 1e-5f) return hl < hr;
                        break;
                    }
                    case ChainUse::ADDITIVE: {
                        const double cl = chain_of(static_cast<int>(l));
                        const double cr = chain_of(static_cast<int>(r));
                        const double kl = hl - cfg.chain_weight * std::min(cl, cfg.chain_cap);
                        const double kr = hr - cfg.chain_weight * std::min(cr, cfg.chain_cap);
                        if (kl != kr) return kl < kr;
                        break;
                    }
                    case ChainUse::RATIO: {
                        const double cl = chain_of(static_cast<int>(l));
                        const double cr = chain_of(static_cast<int>(r));
                        const double kl = (1.0 + cfg.chain_weight * std::min(cl, cfg.chain_cap)) / (hl + cfg.ratio_c);
                        const double kr = (1.0 + cfg.chain_weight * std::min(cr, cfg.chain_cap)) / (hr + cfg.ratio_c);
                        if (kl != kr) return kl > kr;
                        break;
                    }
                    case ChainUse::CHAIN_FIRST: {
                        const double cl = chain_of(static_cast<int>(l));
                        const double cr = chain_of(static_cast<int>(r));
                        if (cl != cr) return cl > cr;
                        if (std::abs(hl - hr) > 1e-5f) return hl < hr;
                        break;
                    }
                }

                // Personal frustration, then ID
                if (frustration[l] != frustration[r]) return frustration[l] > frustration[r];
                return l < r;
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
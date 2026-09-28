#pragma once
#include "Environment.hpp"
#include "types.hpp"
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

class SolutionValidator {
public:
    struct ValidationResult {
        bool is_valid;
        std::string error_message;
    };

    template <typename Expander>
    static ValidationResult validate(const Environment& env, 
                                     const std::vector<std::vector<Location>>& paths,
                                     const Expander& expander) {
        
        if (auto res = check_continuity(env, paths, expander); !res.is_valid) return res;
        if (auto res = check_vertex_collisions(paths); !res.is_valid) return res;
        if (auto res = check_edge_collisions(paths); !res.is_valid) return res;
        
        return {true, "Solution is valid."};
    }

private:
    template <typename Expander>
    static ValidationResult check_continuity(const Environment& env, 
                                             const std::vector<std::vector<Location>>& paths,
                                             const Expander& expander) {
        Location unstarted = {-1, -1, -1};

        for (size_t a = 0; a < paths.size(); ++a) {
            const auto& path = paths[a];
            if (path.empty()) continue;

            // 1. Verify the starting location (unless it's the unstarted prefix)
            if (path[0] != unstarted && !env.is_free(path[0])) {
                return {false, "Agent " + std::to_string(a) + " starts on an obstacle."};
            }

            // 2. Verify step-by-step continuity
            for (size_t t = 0; t < path.size() - 1; ++t) {
                const Location& curr = path[t];
                const Location& next = path[t + 1];

                // If currently off-board
                if (curr == unstarted) {
                    // Spawning onto the board
                    if (next != unstarted && !env.is_free(next)) {
                        return {false, "Agent " + std::to_string(a) + " spawned into an obstacle at t=" + std::to_string(t+1)};
                    }
                    continue; // Skip kinematic check since they are teleporting from nowhere
                }

                // Standard kinematic and obstacle check for agents currently on-board
                std::vector<Location> valid_moves = expander.get_neighbours(curr, env);
                if (std::find(valid_moves.begin(), valid_moves.end(), next) == valid_moves.end()) {
                    return {false, "Agent " + std::to_string(a) + " made an invalid move at t=" + std::to_string(t)};
                }
            }
        }
        return {true, ""};
    }

    static inline ValidationResult check_vertex_collisions(const std::vector<std::vector<Location>>& paths) {
        if (paths.empty()) return {true, ""};
        Location unstarted = {-1, -1, -1};

        size_t max_len = 0;
        for (const auto& path : paths) max_len = std::max(max_len, path.size());

        for (size_t t = 0; t < max_len; ++t) {
            std::unordered_map<Location, size_t, LocationHasher> occupied_nodes; 
            
            for (size_t a = 0; a < paths.size(); ++a) {
                if (t < paths[a].size()) {
                    const Location& loc = paths[a][t];
                    
                    if (loc == unstarted) continue; // Ignore off-board agents

                    auto [it, inserted] = occupied_nodes.insert({loc, a});
                    if (!inserted) {
                        return {false, "Vertex collision detected at t=" + std::to_string(t) + 
                                       " between Agent " + std::to_string(it->second) + " and Agent " + std::to_string(a)};
                    }
                }
            }
        }
        return {true, ""};
    }


static inline ValidationResult check_edge_collisions(const std::vector<std::vector<Location>>& paths) {
        if (paths.empty()) return {true, ""};
        Location unstarted = {-1, -1, -1};

        size_t max_len = 0;
        for (const auto& path : paths) max_len = std::max(max_len, path.size());
        if (max_len < 2) return {true, ""}; 

        for (size_t t = 0; t < max_len - 1; ++t) {
            std::unordered_map<Location, size_t, LocationHasher> locations_at_t;
            
            for (size_t a = 0; a < paths.size(); ++a) {
                if (t + 1 < paths[a].size() && paths[a][t] != unstarted) {
                    locations_at_t[paths[a][t]] = a;
                }
            }

            for (size_t a = 0; a < paths.size(); ++a) {
                if (t + 1 < paths[a].size()) {
                    const Location& u_a = paths[a][t];
                    const Location& v_a = paths[a][t+1];

                    if (u_a == v_a || u_a == unstarted || v_a == unstarted) continue; 

                    auto it = locations_at_t.find(v_a);
                    if (it != locations_at_t.end()) {
                        size_t b = it->second;
                        if (paths[b][t+1] == u_a && a < b) {
                            return {false, "Edge collision detected between t=" + std::to_string(t) + 
                                           " and t=" + std::to_string(t+1) + " for agents " + std::to_string(a) + " and " + std::to_string(b)};
                        }
                    }
                }
            }
        }
        return {true, ""};
    }
};
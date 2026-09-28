// include/SolutionValidator.hpp
#pragma once
#include "Environment.hpp"
#include "Expanders.hpp"
#include "types.hpp"
#include <vector>
#include <string>
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
        for (size_t a = 0; a < paths.size(); ++a) {
            const auto& path = paths[a];
            if (path.empty()) continue;

            // 1. Verify the starting location is not an obstacle
            if (!env.is_free(path[0])) {
                return {false, "Agent " + std::to_string(a) + " starts on an obstacle at (" + 
                               std::to_string(path[0].x) + "," + 
                               std::to_string(path[0].y) + "," + 
                               std::to_string(path[0].z) + ")."};
            }

            // 2. Verify step-by-step continuity
            for (size_t t = 0; t < path.size() - 1; ++t) {
                const Location& curr = path[t];
                const Location& next = path[t + 1];

                // Get all valid kinematic moves that are also obstacle-free
                std::vector<Location> valid_moves = expander.get_neighbours(curr, env);
                
                // Check if the next step is in the list of valid moves
                auto it = std::find(valid_moves.begin(), valid_moves.end(), next);
                
                if (it == valid_moves.end()) {
                    return {false, "Agent " + std::to_string(a) + " made an invalid move from (" +
                                   std::to_string(curr.x) + "," + std::to_string(curr.y) + "," + std::to_string(curr.z) + 
                                   ") to (" + 
                                   std::to_string(next.x) + "," + std::to_string(next.y) + "," + std::to_string(next.z) + 
                                   ") at t=" + std::to_string(t)};
                }
            }
        }
        return {true, ""};
    }

    static ValidationResult check_vertex_collisions(const std::vector<std::vector<Location>>& paths) {
    if (paths.empty()) return {true, ""};

    size_t max_len = 0;
    for (const auto& path : paths) {
        max_len = std::max(max_len, path.size());
    }

    for (size_t t = 0; t < max_len; ++t) {
        // Map: Location -> Agent ID
        std::unordered_map<Location, size_t, LocationHasher> occupied_nodes; 
        
        for (size_t a = 0; a < paths.size(); ++a) {
            if (t < paths[a].size()) {
                const Location& loc = paths[a][t];

                // Attempt to insert the location along with the current agent ID (a)
                auto [it, inserted] = occupied_nodes.insert({loc, a});
                
                if (!inserted) {
                    // it->second retrieves the value (Agent ID) already stored at that location
                    size_t other_agent_id = it->second;
                    
                    return {false, "Vertex collision detected at t=" + std::to_string(t) + 
                                   " between Agent " + std::to_string(other_agent_id) + 
                                   " and Agent " + std::to_string(a) + 
                                   " at (" + std::to_string(loc.x) + "," + 
                                   std::to_string(loc.y) + "," + std::to_string(loc.z) + ")."};
                }
            }
        }
    }
    return {true, ""};
}



    #include <unordered_map>
#include <algorithm>
#include <string>

// ... inside SolutionValidator

static ValidationResult check_edge_collisions(const std::vector<std::vector<Location>>& paths) {
    if (paths.empty()) return {true, ""};

    // 1. Find the maximum path length
    size_t max_len = 0;
    for (const auto& path : paths) {
        max_len = std::max(max_len, path.size());
    }

    if (max_len < 2) return {true, ""}; // No edges exist if paths are 0 or 1 step long

    // 2. Check each timestep transition from t to t+1
    for (size_t t = 0; t < max_len - 1; ++t) {
        // Map to store where each agent is at time t: Location -> Agent ID
        std::unordered_map<Location, size_t, LocationHasher> locations_at_t;
        
        // Pass 1: Record all starting locations for agents taking a step from t to t+1
        for (size_t a = 0; a < paths.size(); ++a) {
            if (t + 1 < paths[a].size()) {
                locations_at_t[paths[a][t]] = a;
            }
        }

        // Pass 2: Check for crossing paths
        for (size_t a = 0; a < paths.size(); ++a) {
            if (t + 1 < paths[a].size()) {
                const Location& u_a = paths[a][t];
                const Location& v_a = paths[a][t+1];

                // If the agent waited in place, it cannot trigger an edge collision
                if (u_a == v_a) continue;

                // Did another agent start at our destination (v_a) at time t?
                auto it = locations_at_t.find(v_a);
                if (it != locations_at_t.end()) {
                    size_t b = it->second;

                    // Did Agent b move into Agent a's starting location (u_a) at t+1?
                    if (paths[b][t+1] == u_a) {
                        
                        // Prevent printing duplicate errors for the same pair (e.g. A->B and B->A)
                        if (a < b) {
                            return {false, "Edge collision detected between t=" + std::to_string(t) + 
                                           " and t=" + std::to_string(t+1) + 
                                           "\nAgent " + std::to_string(a) + " moved (" + 
                                           std::to_string(u_a.x) + "," + std::to_string(u_a.y) + "," + std::to_string(u_a.z) + ") -> (" +
                                           std::to_string(v_a.x) + "," + std::to_string(v_a.y) + "," + std::to_string(v_a.z) + ")" +
                                           "\nAgent " + std::to_string(b) + " moved (" + 
                                           std::to_string(v_a.x) + "," + std::to_string(v_a.y) + "," + std::to_string(v_a.z) + ") -> (" +
                                           std::to_string(u_a.x) + "," + std::to_string(u_a.y) + "," + std::to_string(u_a.z) + ")."};
                        }
                    }
                }
            }
        }
    }
    return {true, ""};
}
};
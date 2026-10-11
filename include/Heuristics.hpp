#pragma once
#include "types.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stdexcept>

// Standard 6-way movement heuristic
class ManhattanHeuristic {
public:
    int get_h_value(const Location& current, const Location& goal) const {
        return std::abs(current.x - goal.x) + 
               std::abs(current.y - goal.y) + 
               std::abs(current.z - goal.z);
    }
};

// Chebyshev distance
class DiagonalHeuristic {
public:
    int get_h_value(const Location& current, const Location& goal) const {
        int dx = std::abs(current.x - goal.x);
        int dy = std::abs(current.y - goal.y);
        int dz = std::abs(current.z - goal.z);
        return std::max({dx, dy, dz});
    }
};

class ZeroHeuristic {
public:
    int get_h_value(const Location& current, const Location& goal) const {
        return 0;
    }
};


template <typename ExpanderType>
class TrueDistanceHeuristic {
private:
    // Maps a goal's 1D grid index to a complete grid of BFS distances
    std::unordered_map<int, std::vector<int>> memoized_distances;
    const Environment* env_ptr;

public:
    TrueDistanceHeuristic(const Environment& env, const std::vector<Location>& goals, const ExpanderType& expander) 
        : env_ptr(&env) 
    {
        int total_cells = env.get_width() * env.get_height() * env.get_depth();

        for (const Location& goal : goals) {
            int goal_idx = env.get_index(goal);
            
            // Skip if multiple agents share the same goal pad (avoids redundant BFS)
            if (memoized_distances.find(goal_idx) != memoized_distances.end()) {
                continue; 
            }

            std::vector<int> distances(total_cells, -1);
            std::queue<Location> q;

            distances[goal_idx] = 0;
            q.push(goal);

            while (!q.empty()) {
                Location curr = q.front();
                q.pop();
                
                int curr_dist = distances[env.get_index(curr)];

                for (const Location& neighbor : expander.get_neighbours(curr, env)) {
                    int n_idx = env.get_index(neighbor);
                    // If undiscovered, set distance and queue
                    if (distances[n_idx] == -1) {
                        distances[n_idx] = curr_dist + 1;
                        q.push(neighbor);
                    }
                }
            }
            memoized_distances[goal_idx] = std::move(distances);
        }
    }
    int get_h_value(const Location& current, const Location& goal) const {
        if (current.x == -1) return 0;
        int goal_idx = env_ptr->get_index(goal);
        int curr_idx = env_ptr->get_index(current);
        
        int dist = memoized_distances.at(goal_idx)[curr_idx];
        
        // Fallback to a large number if unreachable (disconnected graph components)
        if (dist == -1) {
            throw std::runtime_error("Fatal Error: Location (" + 
                                     std::to_string(current.x) + "," + 
                                     std::to_string(current.y) + "," + 
                                     std::to_string(current.z) + 
                                     ") is unreachable from goal.");
        }
        return dist;
    }
};
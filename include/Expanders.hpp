#pragma once 
#include <vector>
#include "types.hpp"
#include "Environment.hpp"

class ManhattanExpander {
public:
    static constexpr int max_neighbours {7};
    std::vector<Location> get_neighbours(const Location& loc, const Environment &env) const {
      std::vector<Location> neighbours;
      neighbours.push_back(loc); // Wait 
      int dx[] {1, -1, 0, 0, 0, 0};
      int dy[] {0, 0, 1, -1, 0, 0};
      int dz[] {0, 0, 0, 0, 1, -1 };
      for (int i = 0; i < 6; i++) {
        Location next_loc = {loc.x + dx[i], loc.y + dy[i], loc.z + dz[i]};
        if (env.is_free(next_loc)) {neighbours.push_back(next_loc);
        }
      }   
    return neighbours;
    }
};

class DiagonalExpander {
public:
    static constexpr int max_neighbours {27};
    std::vector<Location> get_neighbours(const Location& loc, const Environment& env) const {
        std::vector<Location> neighbors;
        neighbors.push_back(loc); // Wait 

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) continue; 
                    
                    Location next_loc = {loc.x + dx, loc.y + dy, loc.z + dz};
                    if (env.is_free(next_loc)) {
                        neighbors.push_back(next_loc);
                    }
                }
            }
        }
        return neighbors;
    }
};

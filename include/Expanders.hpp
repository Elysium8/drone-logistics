#pragma once 
#include <vector>
#include "types.hpp"
#include "Environment.hpp"

class ManhattanExpander {
public:
    std::vector<Location> get_neighbours(const Location& loc, const Environment &env){
      std::vector<Location> neighbours;
      neighbours.push_back(loc); // Wait 
      int dx[] {1, -1, 0, 0, 0, 0};
      int dy[] {0, 0, 1, -1, 0, 0};
      int dz[] {0, 0, 0, 0, 1, 1 };
      for (int i = 0; i < 6; i++) {
        Location next_loc = {loc.x + dx[i], loc.y + dy[i], loc.z + dz[i]};
        if (env.is_free(next_loc)) {neighbours.push_back(next_loc);
        }
      }   
    return neighbours;
    }
};



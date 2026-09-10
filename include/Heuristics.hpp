#pragma once
#include "types.hpp"
#include <cmath>
#include <algorithm>

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
#pragma once
#include <vector>

struct Location {
    int x, y, z;
    bool operator==(const Location& other) const {
        return x == other.x && y == other.y && z == other.z;}
    
    bool operator!=(const Location& other) const {
        return !(*this == other);
    }
};

struct MAPFInstance {
    std::vector<Location> starts;
    std::vector<Location> goals;
};

struct SearchMetrics{
    int nodes_expanded = 0;
    int nodes_generated = 0;
    int nodes_touched = 0; 
    double runtime_us = 0; 
    int path_cost = -1;
    bool solved = false;
};


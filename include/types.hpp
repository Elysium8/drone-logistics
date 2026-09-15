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
    std::vector<std::vector<Location>> paths;
};

struct LocationHasher {
    std::size_t operator()(const Location& loc) const {
        std::size_t h1 = std::hash<int>()(loc.x);
        std::size_t h2 = std::hash<int>()(loc.y);
        std::size_t h3 = std::hash<int>()(loc.z);
        
        // Bitwise XOR and shift to combine the hashes
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};
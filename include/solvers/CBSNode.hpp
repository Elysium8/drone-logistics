#pragma once
#include "CBSConstraints.hpp"
#include <vector>
#include <memory>

struct CBSNode {
    std::vector<Constraint> constraints;
    
    std::vector<std::vector<Location>> paths;
    
    int cost = 0;

    bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }
};
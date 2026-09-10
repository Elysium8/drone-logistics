#pragma once 
#include "types.hpp"
#include <functional>


struct STNode {
    Location loc;
    int t;
    int g_score;
    int f_score;

    const STNode* parent;

    bool operator>(const STNode& other) const {
        return (f_score > other.f_score);
    }

    bool operator==(const STNode& other) const {
        return loc == other.loc && t == other.t;
    }
};

struct STNodeHasher {
    std::size_t operator()(const STNode& node) const {
        std::size_t h1 = LocationHasher()(node.loc);
        std::size_t h2 = std::hash<int>()(node.t);
        return h1 ^ (h2 << 1);
    }
};
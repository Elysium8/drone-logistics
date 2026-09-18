#pragma once
#include <vector>
#include <memory>
# include "../types.hpp"
#include <unordered_set>

struct CBSNode {
    std::vector<Constraint> constraints;
    
    std::vector<std::vector<Location>> paths;
    
    int cost = 0;

    bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }
};


// A collision 
struct Conflict { 
    int agent1;
    int agent2;
    Location loc1;
    Location loc2; 
    int t;
};

struct Constraint {
    int agent_id; 
    Location loc1;  // if these are the same then it's a vertex constraint 
    Location loc2; 
    int t;
    
    bool operator==(const Constraint& o) const {
        return agent_id == o.agent_id && loc1 == o.loc1 && loc2 == o.loc2 && t == o.t;
    }
};


struct ConstraintHasher {
    std::size_t operator()(const Constraint& c) const {
        LocationHasher lh;
        return std::hash<int>()(c.agent_id) ^ 
              (lh(c.loc1) << 1) ^ 
              (lh(c.loc2) << 2) ^ 
              (std::hash<int>()(c.t) << 3);
    }
};

class ConstraintTable {
private:
    std::unordered_set<Constraint, ConstraintHasher> constraints;
    int current_agent = -1;

public:
    void build_for_agent(int agent_id, const std::vector<Constraint>& node_constraints) {
        current_agent = agent_id;
        constraints.clear();
        for (const auto& c : node_constraints) {
            if (c.agent_id == agent_id) {
                constraints.insert(c);
            }
    }
}

    bool is_free(const Location loc, int t) const {
        Constraint c = {current_agent, loc, loc, t};
        return constraints.find(c) == constraints.end();
    }

    bool is_edge_safe(const Location& from, const Location& to, int t) const {
        Constraint c = {current_agent, from, to, t};
        return constraints.find(c) == constraints.end();
    }
};


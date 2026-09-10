#pragma once 
# include <vector>
#include "types.hpp"
#include <unordered_map>
#include <unordered_set>

struct EdgeConstraint {
    Location from, to;
    int t;
    bool operator==(const EdgeConstraint& o) const {
        return from == o.from && to == o.to && t == o.t;
    }
};

struct EdgeHasher {
    std::size_t operator()(const EdgeConstraint& ec) const {
        LocationHasher lh;
        return lh(ec.from) ^ (lh(ec.to) << 1) ^ (std::hash<int>()(ec.t) << 2);
    }
};


class ReservationTable{
private:
    std::unordered_map<int, std::unordered_set<Location, LocationHasher>> table;
    std::unordered_map<Location, int, LocationHasher> goal_reservations;
    std::unordered_set<EdgeConstraint, EdgeHasher> edge_table;


public:
    bool is_free(const Location&loc, int t) const {
        if (table.count(t) && table.at(t).count(loc)) {
            return false;
        }
        if (goal_reservations.count(loc) && t >= goal_reservations.at(loc)) {
            return false;
        }
        return true;
    }

    bool is_edge_safe(const Location& from, const Location& to, int t) const {
        return edge_table.find({from, to, t}) == edge_table.end();
    }

    void reserve_path(const std::vector<Location>& path) {
        if (path.empty()) {return;}

        for (int t = 0; t < path.size(); ++t) {
            table[t].insert(path[t]);
            if (t > 0) {
                edge_table.insert({path[t], path[t-1], t - 1});
            }
        }
        goal_reservations[path.back()] = path.size() - 1;
    }
};



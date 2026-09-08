#pragma once 
# include <vector>
#include "types.hpp"
#include <unordered_map>
#include <unordered_set>


class ReservationTable{
private:
    std::unordered_map<int, std::unordered_set<Location, LocationHasher>> table;
    std::unordered_map<Location, int, LocationHasher> goal_reservations;


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

    void reserve_path(const std::vector<Location>& path) {
        if (path.empty()) {return;}

        for (int t = 0; t < path.size(); ++t) {
            table[t].insert(path[t]);
        }
        goal_reservations[path.back()] = path.size() - 1;
    }
};



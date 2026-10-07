#pragma once 
#include <tuple>
#include "../types.hpp"

class SafePlanner {
private:
public:
/*
Arguments: Current agent positions and goals. Plus agents that are about to becoming "ready" to spawn (either incoming or outgoing)
Returns: A start location and a start time
*/
    std::tuple<Location, int> plan_one_agent_start(int t, const std::vector<std::vector<Location>>& paths, const MAPFInstance &instance, const std::vector<Location>& pads) {
        for (const auto &pad : pads) {
            bool safe = true;
            for (const auto &path : paths) {
                // Safely check current or last known location
                Location current_loc = (t < path.size()) ? path[t] : path.back();
                if (current_loc == pad) {
                    safe = false;
                    break;
                }
            }
            if (safe) return {pad, t};
        }
        // No pads free. Return a failure location and delay the spawn time.
        return {{-1, -1, -1}, t + 1};
    };

    /*
    Arguments: 
    Returns: A goal location

    */
    Location plan_one_agent_goal() {
        return {-1, -1, -1};

    };


};
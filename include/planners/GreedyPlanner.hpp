#pragma once 
#include <tuple>
#include "../types.hpp"
#include <random>

class GreedyPlanner {
private:
    std::mt19937 gen;
public:
/*
Arguments: Current agent positions and goals. Plus agents that are about to becoming "ready" to spawn (either incoming or outgoing)
Returns: A start location and a start time
*/
    GreedyPlanner(int seed = 72) : gen(seed) {}
    std::tuple<Location, int> plan_one_agent_start(int t, const std::vector<std::vector<Location>>& paths, const MAPFInstance &instance, const std::vector<Location>& pads) {
        //random placeholder 
        std::uniform_int_distribution<std::size_t> dist(0, pads.size() - 1);
        return {pads[dist(gen)], t};
    };

    /*
    Arguments: 
    Returns: A goal location

    */
    Location plan_one_agent_goal() {


    };


};
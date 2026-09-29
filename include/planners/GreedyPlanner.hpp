#pragma once 
#include <tuple>
#include "../types.hpp"

class GreedyPlanner {
private:

public:
/*
Arguments: Current agent positions and goals. Plus agents that are about to becoming "ready" to spawn (either incoming or outgoing)
Returns: A start location and a start time
*/
std::tuple<Location, int> plan_one_agent_start(std::vector<Location> &agent_locations, std::vector<Location> &goals) {


};

/*
Arguments: 
Returns: A goal location

*/
Location plan_one_agent_goal() {


};


};
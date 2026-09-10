#pragma once
#include "../types.hpp"
#include "../Environment.hpp"
#include "../ReservationTable.hpp"
#include "../Timer.hpp"
#include "SpaceTimeAStar.hpp"

template <typename ExpanderType, typename HeuristicType>
class PrioritizedPlanning {
public:
    void solve(const Environment& env, 
               const MAPFInstance& instance, 
               const ExpanderType& expander, 
               const HeuristicType& heuristic, 
               SearchMetrics& metrics) 
    {
        Timer timer;
        ReservationTable calendar;
        int total_cost = 0;

        SpaceTimeAStar<ExpanderType, ReservationTable, HeuristicType> st_astar;

        for (size_t agent_id = 0; agent_id < instance.starts.size(); ++agent_id) {
            Location start = instance.starts[agent_id];
            Location goal = instance.goals[agent_id];

            std::vector<Location> path = st_astar.solve(start, goal, env, expander, calendar, heuristic, metrics);
            
            if (path.empty()) {
                metrics.solved = false;
                metrics.runtime_us = timer.elapsed_microseconds();
                return;
            }

            calendar.reserve_path(path);
            total_cost += (path.size() - 1);
            metrics.paths.push_back(path);
        }

        metrics.solved = true;
        metrics.path_cost = total_cost;
        metrics.runtime_us = timer.elapsed_microseconds();
    }
};
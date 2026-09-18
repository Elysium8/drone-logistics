#pragma once
#include "../types.hpp"
#include "../Expanders.hpp"
#include "../Environment.hpp"
#include "../SpaceTimeTypes.hpp"
#include <queue>
#include <unordered_set>
#include <vector>
#include <algorithm>

template <typename ExpanderType, typename ConstraintOracle, typename HeuristicType>
class SpaceTimeAStar
{
public:
    std::vector<Location> solve(const Location &start, const Location &goal,
                                const Environment &env, int delay, int start_time, const ExpanderType &expander,
                                const ConstraintOracle &constraints, const HeuristicType &heuristic, SearchMetrics &metrics,
                                const std::vector<std::vector<Location>> &paths)
    {
        std::vector<STNode *> all_nodes;
        auto create_node = [&](Location loc, int t, int g, int f, int con, const STNode *parent)
        {
            STNode *n = new STNode{loc, t, g, f, con, parent};
            all_nodes.push_back(n);
            return n;
        };

        std::priority_queue<STNode, std::vector<STNode>, std::greater<STNode>> open_list;
        std::unordered_set<STNode, STNodeHasher> closed_list;
        int TIME_LIMIT = env.get_index({env.get_width() - 1, env.get_height() - 1, env.get_depth() - 1});
        if (!constraints.is_free(start, start_time))
        {
            return std::vector<Location>();
        }
        STNode *start_node = create_node(start, start_time, 0, 0, 0, nullptr);
        open_list.push(*start_node);
        metrics.astar_nodes_generated++;
        STNode *goal_node = nullptr;

        while (!open_list.empty())
        {
            STNode current = open_list.top();
            open_list.pop();
            metrics.astar_nodes_expanded++;
            STNode *current_ptr = create_node(current.loc, current.t, current.g_score, current.f_score, current.conflict_score, current.parent);

            if (current.loc == goal)
            {
                goal_node = current_ptr;
                break;
            }

            if (closed_list.count(current))
                continue;
            closed_list.insert(current);

            if (current.t >= TIME_LIMIT)
                continue; // Don't expand past time limit

            for (const Location &neighbor : expander.get_neighbours(current.loc, env))
            {
                metrics.nodes_touched++;
                int next_t = current.t + 1;

                if (!constraints.is_free(neighbor, next_t))
                    continue; // Don't add neighbour to closed if reserved at next time
                if (!constraints.is_edge_safe(current.loc, neighbor, next_t))
                    continue; // edge constraints

                STNode next_state = {neighbor, next_t, 0, 0, 0, nullptr}; // Keep values 0 to check against closed

                if (closed_list.count(next_state) == 0)
                {
                    int g = next_t;
                    int h = heuristic.get_h_value(neighbor, goal);
                    int conflicts = current.conflict_score;
                    for (const auto &path : paths)
                    {
                        if (path.size() > next_t)
                        {
                            conflicts += path[next_t] == neighbor;
                        }
                    }
                    open_list.push({neighbor, next_t, g, g + h, conflicts, current_ptr});
                    metrics.astar_nodes_generated++;
                }
            }
        }
        std::vector<Location> path;
        if (goal_node)
        {
            const STNode *curr = goal_node;
            while (curr != nullptr)
            {
                path.push_back(curr->loc);
                curr = curr->parent;
            }
            std::reverse(path.begin(), path.end());

            // All padding logic moved safely inside
            Location final_loc = path.back();
            for (int d = 0; d < delay; ++d)
            {
                path.push_back(final_loc);
            }
            path.insert(path.begin(), start_time, {-1, -1, -1});
        }

        for (STNode *n : all_nodes)
            delete n;
        return path; // Returns {} if goal_node was nullptr, correctly signaling failure to CBS
    }
};
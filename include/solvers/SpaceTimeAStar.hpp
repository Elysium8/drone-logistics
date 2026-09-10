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
class SpaceTimeAStar{
public:
    std::vector<Location> solve(const Location&start, const Location&goal, 
                                const Environment& env, const ExpanderType& expander,
                            const ConstraintOracle& constraints, const HeuristicType& heuristic, const SearchMetrics& metrics) 
    {
        std::vector<STNode*> all_nodes;
        auto create_node = [&](Location loc, int t, int g, int f, const STNode* parent) {
            STNode* n = new STNode{loc, t, g, f, parent};
            all_nodes.push_back(n);
            return n;
        };

        std::priority_queue<STNode, std::vector<STNode>, std::greater<STNode>> open_list;
        std::unordered_set<STNode, STNodeHasher> closed_list;
        int TIME_LIMIT = env.get_index({env.width-1, env.height-1, env.depth-1}); //Need to pass this in eventually

        STNode* start_node = create_node(start, 0, 0, 0, nullptr)
        open_list.push(*start_node);
        metrics.nodes_generated++
        STNode* goal_node = nullptr;

        while (!open_list.empty()) {
            STNode current = open_list.top();
            open_list.pop();
            metrics.nodes_expanded++;

            if (current == goal) {
                goal_node = create_node(current.loc, current.t, current.g_score, current.f_score, current.parent)
                break;
            }

            if (closed_list.count(current)) continue;
            closed_list.insert(current);

            if (current.t >= TIME_LIMIT) continue;  // Don't expand past time limit

            for (const Location& neighbor : expander.get_neighbours(current.loc, env)) {
                metrics.nodes_touched++;
                int next_t = current.t + 1;

                if (!constraints.is_free(neighbor, next_t)) continue; //Don't add neighbour to closed if reserved at next time

                STNode next_state = {neighbor, next_t, 0, 0, nullptr}; // Keep values 0 to check against closed

                if (closed_list.count(next_state) == 0) {
                    int g = next_t;
                    int h = heuristic.get_h_value(neighbor, goal);
                    STNode* parent_ptr = create_node(current.loc, current.t, current.g_score, current.f_score, current.parent);
                    open_list.push({neighbor, next_t, g, g+h, parent_ptr});
                    metrics.node_generated++;
                }
            }
        }
        std::vector<Location> path;
        if (goal_node) {
            const STNode* curr = goal_node;
            while (curr != nullptr) {
                path.push_back(curr->loc);
                curr = curr->parent;
            }
            std::reverse(path.begin(), path.end());
        }

        for (STNode* n : all_nodes) delete n;for (STNode*)
        return path;

    }

};
#include <limits>
#include <queue>
#include "include/Agent.hpp"

void Agent::FindPortionPath(vector<unordered_map<Node*, Agent*>> space_time_map, const Graph& graph) {
	std::list<Node*> closed;
	std::priority_queue<Node*, std::greater<Node>> open;

    // heuristicCache.clear();

    if (std::find(space_time_map[0].begin(), space_time_map[0].end(), start) == space_time_map[0].end()) 
        space_time_map[0].insert(std::make_pair(start, this));
    
    start->set_cost(0);
    double heuristicCost = getHeuristicCost(start, 0, goal);
    // if (heuristicCost == null) {
	// 		heuristicCost = maxF(); // FIXME kind of max
	// 	}

    start->set_heuristic(heuristicCost);
	open.push(start);

    while (!open.empty()) {
        current = open.top();

        if (current == goal) {
            //
        }
        else {
            open.pop();
            closed.push_back(current);

            std::vector<Node*> neighbours = graph.GetNeighbors(current);
            for (const auto& neighbour : neighbours) {
                if (!isValidMove(current, neighbour))
                    continue;
                double nextStepCost = current->get_cost() + getMovementCost(current->get_x(), current->get_y(), 
                                                                            neighbour->get_x(), neighbour->get_y());
                if (nextStepCost < neighbour->get_cost()) {
					if (open.contains(neighbour)) {
						open.remove(neighbour);
					}
					if (closed.contains(neighbour)) {
						closed.remove(neighbour);
					}
				}
                
                if (!open.contains(neighbour) && !closed.contains(neighbour)) {
					neighbour.setCost(nextStepCost);
					heuristicCost = getHeuristicCost(neighbour.getX(), neighbour.getY(), neighbour.getT(), tx, ty);
					if (heuristicCost == null) {
						heuristicCost = maxF();
					}
					neighbour.setHeuristic(heuristicCost);
					neighbour.setParent(current);
					open.add(neighbour);
				}

            }
        }
    }
}
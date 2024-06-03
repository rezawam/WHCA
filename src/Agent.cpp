#include <limits>
#include <set>
#include "include/Agent.hpp"

#define t 1

void Agent::FindPortionPath(array<unordered_map<Node*, Agent*>, WINDOW_SIZE>& space_time_map, const Graph& graph) {
	std::list<Node*> closed;
	std::set<Node*, std::greater<Node>> open;

    // heuristicCache.clear();

    if (std::find(space_time_map[0].begin(), space_time_map[0].end(), start) == space_time_map[0].end()) 
        space_time_map[0].insert(std::make_pair(start, this));
    
    start->set_cost(0);
    double heuristicCost = getHeuristicCost(start, 0, goal);
    // if (heuristicCost == null) {
	// 		heuristicCost = maxF(); // FIXME kind of max
	// 	}

    start->set_heuristic(heuristicCost);
	open.insert(start);

    while (!open.empty()) {
        current = *open.begin();

        if (current == goal) {
            return;
        }
        
        open.remove(current);
        closed.push_back(current);

        std::vector<Node*> neighbours = graph.GetNeighbors(current);
        for (const auto& neighbour : neighbours) {

            // Node isn't free to be visited at t, ignore it
            if (find(space_time_map[t].begin(), space_time_map[t].end(), neighbour) != space_time_map[t].end()) 
                continue;

            Edge* current_edge;
            for (const auto& e : graph.adjacency_list.at(current)) {
                if (e->destination == neighbour) {
                    current_edge = e;
                    break;
                }
            }

            double nextStepCost = current->get_cost() + current_edge->weight;
            if (nextStepCost < neighbour->get_cost()) {
                if (find(open.begin(), open.end(), neighbour) != open.end()) {
                    open.remove(neighbour);
                }
                if (find(closed.begin(), closed.end(), neighbour) != closed.end()) {
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
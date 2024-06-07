#include "Solver.hpp"
#include <queue>
#include <limits>
#include <unordered_set>
#include <cmath>
#include <algorithm>

double WHCAPathFinder::GetHeuristicCost(Node* start, Node* goal) { // TO-DO: rewrite - add checking if path exists
    return sqrt(pow(goal->get_x() - start->get_x(), 2) + pow(goal->get_y() - start->get_y(), 2)); // returns hypotenuse
}

struct NodeState {
   
};


Path WHCAPathFinder::FindPortionPath(Agent* agent) {
    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> open;
    std::unordered_set<Node*> closed;

    std::vector<Graph> space_time_map(WINDOW_SIZE, graph);
    for (int i = 0; i < WINDOW_SIZE; i++) {
        for (auto& node : space_time_map[i].nodes) {
            node->set_cost(std::numeric_limits<double>::max()); 
            node->set_t(i);
        }
    }

    Node* startNode = space_time_map[0].GetNodeById(agent->start->get_id());
    startNode->set_cost(0);

    double heuristicCost = GetHeuristicCost(startNode, agent->goal);
    startNode->set_heuristic(heuristicCost);
    open.push(startNode);

    while(!open.empty()) {
        Node* current = open.top();

        // If reached the goal at the end of the window:
        if (current == agent->goal && current->get_t() == WINDOW_SIZE - 1) {
            // Start to recreate partial path from parents
            Path path;
            Node* node = space_time_map[current->get_t()].GetNodeById(agent->goal->get_id());
            while (node != startNode) {
                agent->portion_path.push_front(node);
                node = node->get_parent();
                if (node == nullptr)
                    std::cout << "target = null";
            }

            agent->portion_path.push_front(agent->start);
            return path;
        }

        open.pop();
        closed.insert(current);

        // search through all the neighbours of the current node evaluating
			// them as next steps

			vector<Node*> neighbours = graph.GetNeighbors(current);
			for (Node neighbour : neighbours) {
				if (!isValidLocation(sx, sy, neighbour.getX(), neighbour.getY(), neighbour.getT()))
					continue;
				
				if (!isValidMove(current.getX(), current.getY(), current.getT(), neighbour.getX(), neighbour
						.getY(), neighbour.getT()))
					continue;
    }

}

void WHCAPathFinder::FindPaths() {
    bool all_agents_found_path = false;

    while(!all_agents_found_path) {
        for (auto& agent : agents) {
            FindPortionPath(agent);
        }
    }
}
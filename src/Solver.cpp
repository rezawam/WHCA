#include "include/Solver.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

int WHCAPathFinder::GetHeuristicCost(Node* start, Node* goal) { // TO-DO: rewrite - add checking if path exists
    return sqrt(pow(goal->get_x() - start->get_x(), 2) + pow(goal->get_y() - start->get_y(), 2)); // returns hypotenuse
}

struct NodeState {
    Node* node;
    NodeState* parent;
    int g;  // cost from start
    int h;  // heuristic cost to goal
    int t;  // time step

    bool operator>(const NodeState& other) const {
        return g + h > other.g + other.h;
    }
};

Path WHCAPathFinder::FindPortionPath(Agent* agent) {
    std::priority_queue<NodeState, std::vector<NodeState>, std::greater<NodeState>> open_list;
    std::unordered_set<Node*> closed_set;

    open_list.push({agent->start, nullptr, 0, GetHeuristicCost(agent->start, agent->goal), 0});

    while (!open_list.empty()) {
        NodeState current = open_list.top();
        open_list.pop();

        if (current.node == agent->goal) {
            Path path;
            NodeState* node_state = &current;
            while (node_state) {
                path.push_back(node_state->node);
                node_state = node_state->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed_set.find(current.node) != closed_set.end()) {
            continue;
        }
        closed_set.insert(current.node);

        for (Node* neighbor : graph.GetNeighbors(current.node)) {
            if (!is_reserved(neighbor, current.t + 1, agent)) {
                open_list.push({neighbor, &current, current.g + 1, GetHeuristicCost(neighbor, agent->goal), current.t + 1});
            }
        }
    }

    return Path{};  // return empty path if no path found
}

void WHCAPathFinder::FindPaths() {
    bool all_agents_found_path = false;

    while(!all_agents_found_path) {
        for (auto& agent : agents) {
            FindPortionPath(agent);
        }
    }
}
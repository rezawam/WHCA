#include "Solver.hpp"
#include <queue>
#include <limits>
#include <unordered_set>
#include <cmath>
#include <algorithm>

int WHCAPathFinder::GetHeuristicCost(Node* start, Node* goal) { // TO-DO: rewrite - add checking if path exists
    return sqrt(pow(goal->get_x() - start->get_x(), 2) + pow(goal->get_y() - start->get_y(), 2)); // returns hypotenuse
}

struct NodeState {
   
};

// void PrintOpen(std::priority_queue<NodeState, std::vector<NodeState>, std::greater<NodeState>> open) {
//     while (! open.empty() ) {
//     cout << open.top().node->get_id() << " ";
//     open.pop();
// }
//     cout << "\n"; 
// }

Path WHCAPathFinder::FindPortionPath(Agent* agent) {
    std::priority_queue<NodeState, std::vector<NodeState>, std::greater<NodeState>> open_list;
    std::unordered_set<Node*> closed_set;

    std::vector<Graph> space_time_map(WINDOW_SIZE, graph);
    for (auto& g : space_time_map) {
        for (auto& node : g.nodes) {
            node->set_cost(INT_MAX); // should be dbl_max???
        }
    }

    Node* startNode = std::find(space_time_map[0].nodes.begin(), space_time_map[0].nodes.end(), agent->start);
}

void WHCAPathFinder::FindPaths() {
    bool all_agents_found_path = false;

    while(!all_agents_found_path) {
        for (auto& agent : agents) {
            FindPortionPath(agent);
        }
    }
}
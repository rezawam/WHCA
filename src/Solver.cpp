#include "Solver.hpp"
#include <queue>
#include <limits>
#include <unordered_set>
#include <cmath>
#include <algorithm>


template<
    class T,
    class Container = std::vector<T>,
    class Compare = std::less<typename Container::value_type>
> class MyQueue : public std::priority_queue<T, Container, Compare>
{
public:
    typedef typename
        std::priority_queue<
        T,
        Container,
        Compare>::container_type::const_iterator const_iterator;

    const_iterator find(const T&val) const
    {
        auto first = this->c.cbegin();
        auto last = this->c.cend();
        while (first!=last) {
            if (*first==val) return first;
            ++first;
        }
        return last;
    }
};

double WHCAPathFinder::GetHeuristicCost(Node* start, Node* goal) { // TO-DO: rewrite - add checking if path exists
    return sqrt(pow(goal->get_x() - start->get_x(), 2) + pow(goal->get_y() - start->get_y(), 2)); // returns hypotenuse
}

void PrintOpenList(std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> q) {
    while (! q.empty() ) {
    cout << q.top()->get_id() << " ";
    q.pop();
}
    cout << "\n";
}

bool WHCAPathFinder::IsValidMove(Node* neighbour, int step) {
    if (reservations[step].find(neighbour) == reservations[step].end())
        return 1;
    return 0;
}

Path WHCAPathFinder::FindPortionPath(Agent* agent) {
    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> open;
    std::unordered_set<Node*> closed;

    // std::vector<Graph> space_time_map(WINDOW_SIZE);
    // for (int i = 0; i < WINDOW_SIZE; i++) {
    //     Graph copy_graph = graph;
    //     space_time_map[i] = copy_graph;
    //     for (auto& node : space_time_map[i].nodes) {
    //         node->set_cost(std::numeric_limits<double>::max()); 
    //         node->set_heuristic(std::numeric_limits<double>::max()); 
    //         node->set_t(i);
    //     }
    // } 

    graph.ClearParents();

    Node* startNode = graph.GetNodeById(agent->start->get_id());

    startNode->set_cost(0);

    double heuristicCost = GetHeuristicCost(startNode, agent->goal);
    startNode->set_heuristic(heuristicCost);
    open.push(startNode);

    while(!open.empty()) {
        Node* current = open.top();

        int current_step = current->count_parents();
        if (current_step >= WINDOW_SIZE)
            // We're out of window
            break;

        // If reached the goal
        if (current->get_id() == agent->goal->get_id()) { // && current->get_t() == WINDOW_SIZE - 1
            // Start to recreate partial path from parents
            Path path;
            Node* node = current;
            int path_size = current->count_parents() + 1;
            while (node != startNode) {
                path_size--;
                agent->portion_path.push_front(node);
                // Make reservations
                reservations[path_size].insert({node, agent});
                node = node->get_parent();
                if (node == nullptr)
                    std::cout << "target = null";
            }

            agent->portion_path.push_front(agent->start);
            reservations[0].insert({agent->start, agent});

            std::cout << "Found partial path for " << agent->name << endl;
            return path;
        }

        open.pop();
        closed.insert(current);

        // search through all the neighbours of the current node evaluating
		// them as next steps

        vector<Node*> neighbours = graph.GetNeighbors(current);
        for (Node* neighbour : neighbours) {
            if (!IsValidMove(neighbour, current_step + 1)) {
                static Node current_copy = *current;
                current_copy.set_parent(current); // We wait
                open.push(&current_copy);
                continue;
            }
                
            //cout << "current: " << current->get_id() << ", neighbour: " << neighbour->get_id() << endl;
            double next_step_cost = current->get_cost() + graph.GetEdgeCost(current, neighbour);

            if (next_step_cost <= neighbour->get_cost()) {
                if (closed.find(neighbour) != closed.end()) {
                    closed.erase(closed.find(neighbour));
                }
                else {
                    neighbour->set_cost(next_step_cost);
                    heuristicCost = GetHeuristicCost(neighbour, agent->goal);
                    neighbour->set_heuristic(heuristicCost);
                    neighbour->set_parent(current);
                    open.push(neighbour);
                }
            }
        }
    }
    Path empty;
    std::cout << "Path is empty\n";
    return empty;
}

void WHCAPathFinder::FindPaths() {
    bool all_agents_found_path = false;

    while(!all_agents_found_path) {
        for (auto& agent : agents) {
            FindPortionPath(agent);
        }

        for (const auto& agent : agents) {
            if (agent->isAtGoal() == false)
                break;
            all_agents_found_path = true;
        }
    }
}
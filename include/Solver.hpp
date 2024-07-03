#pragma once
#define WINDOW_SIZE 8 

#include "Agent.hpp"

#include <map>

class WHCAPathFinder {
private:
    Graph graph;
    std::vector<Agent*> agents;
	std::array<std::unordered_map<Node*, Agent*>, WINDOW_SIZE> reservations;
    std::map<std::pair<int, int>, Path> heuristic_cache;

    bool is_reserved(Node* node, int time, Agent* agent) {
    int index = time % WINDOW_SIZE;
    auto it = reservations[index].find(node);
    return it != reservations[index].end() && it->second != agent;
}


public:
    WHCAPathFinder(Graph& graph_, std::vector<Agent*>& agents_) : agents(agents_), graph(graph_) {}

    Path FindPortionPath(Agent* agent);
	void FindPaths();

    double GetHeuristicCost(Node* start, Node* goal);
    bool IsValidMove(Node* node, Node* neighbour);
};
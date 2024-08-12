#pragma once
#define WINDOW_SIZE 8 

#include "Agent.hpp"

#include <map>
#include <stack>

class WHCAPathFinder {
private:
    Graph graph;
    std::vector<Agent*> agents;
	std::array<std::unordered_map<int, Agent*>, WINDOW_SIZE> reservations;
    std::map<std::pair<int, int>, Path> heuristic_cache;
    std::stack<Node> waiting_list;


public:
    WHCAPathFinder(Graph& graph, std::vector<Agent*>& agents) : graph(graph), agents(agents){}

    Path FindPortionPath(Agent* agent);
	void FindPaths();

    double GetHeuristicCost(Node* start, Node* goal);
    bool IsValidMove(Node* neighbour, int step);
};
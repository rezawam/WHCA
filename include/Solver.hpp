#pragma once

#include "Agent.hpp"

class WHCAPathFinder {
private:
    Graph graph;
    std::vector<Agent*> agents;
	std::array<std::unordered_map<Node*, Agent*>, WINDOW_SIZE> space_time_map;

public:
    WHCAPathFinder(Graph& graph_, std::vector<Agent*>& agents_) : agents(agents_), graph(graph_) {}

	void FindPaths();
	
};
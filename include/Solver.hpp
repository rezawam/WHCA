#pragma once

#include <list>
#include "Agent.hpp"

#define WINDOW_SIZE 16

class WHCAPathFinder {
private:
    Graph graph;
    std::vector<Agent*> agents;
	// private Node[][][] nodes;
	
	std::vector<std::unordered_map<Node*, Agent*>> space_time_map[WINDOW_SIZE];
	// private TileMap map;
	// private BiHashMap<Integer, Integer, igrek.robopath.pathfinder.astar.Path> heuristicCache = new BiHashMap<>();\

public:
    WHCAPathFinder(Graph& graph_, std::vector<Agent*>& agents_) : agents(agents_), graph(graph_) {}

	void FindPaths() {
        
        while()
    }
	
}
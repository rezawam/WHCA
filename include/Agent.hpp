#pragma once
#define WINDOW_SIZE 16

#include <list>
#include <array>
#include <string>

#include "include/Graph.hpp"

using std::string, std::array;

class Agent {
public:
    Agent(string name_, Node* start_, Node* goal_) : name(name_), start(start_), goal(goal_) { }; 

    string get_name() const { return name; }
    Node* get_current_node() const { return current; }
    Node* get_start() const { return start; }
    Node* get_goal() const { return goal; }

    void FindPortionPath(array<unordered_map<Node*, Agent*>, WINDOW_SIZE>& space_time_map, const Graph& graph);
private:
    std::list<Node*> path;
    // std::list<Node*> portion_path;
    Node* start, *goal, *next, *current, *prev;
    string name;
};
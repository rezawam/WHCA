#pragma once

#include <list>
#include <string>

#include "include/Graph.hpp"

using std::string;

class Agent {
public:
    Agent(string name_, Node start_, Node goal_) : name(name_), start(start_), goal(goal_) { }; 

    string get_name() const { return name; }
    Node* get_current_node() const { return current; }
    Node* get_start() const { return start; }
    Node* get_goal() const { return goal; }

    void FindPortionPath(vector<unordered_map<Node*, Agent*>> space_time_map, const Graph& graph);
private:
    std::list<Node*> path;
    // std::list<Node*> portion_path;
    Node* start, *goal, *next, *current, *prev;
    string name;
};
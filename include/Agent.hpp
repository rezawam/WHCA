#pragma once
#define WINDOW_SIZE 16

#include <list>
#include <array>
#include <string>

#include "Graph.hpp"

using std::string, std::array;

typedef std::list<Node*> Path;

class Agent {
public:
    Agent(string name_, Node* start_, Node* goal_) : name(name_), start(start_), goal(goal_) { }; 

    // string get_name() const { return name; }
    // Node* get_current_node() const { return current; }
    // Node* get_start() const { return start; }
    // Node* get_goal() const { return goal; }

    Path path;
    Path portion_path;
    Node* start, *goal, *next, *current, *prev;
    string name;
};
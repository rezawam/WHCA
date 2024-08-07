#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <limits>

using std::vector, std::unordered_map, std::cout, std::endl;

// Класс для представления вершины графа
class Node {
public:
    Node() {}
    Node(int id_, int x_, int y_) : id(id_), x(x_), y(y_), parent(nullptr) {};

    void swap(Node other) {
        using std::swap;
        swap(id, other.id);
        swap(x, other.x);
        swap(y, other.y);
        swap(parent, other.parent);
        swap(cost, other.cost);
        swap(heuristic, other.heuristic);
    }

    Node(const Node&) = default;

    Node& operator=(Node other) {
        swap(other);
        cout << "Copied node" << endl;
        return *this;
    }

    int get_id() const { return id; }
    int get_x() const { return x; }
    int get_y() const { return y; }

    int count_parents() const {
        int number_of_parents = 0;
        Node* parent = get_parent();
        while (parent != nullptr) {
            number_of_parents++;
            parent = parent->get_parent();
        }
        return number_of_parents;
    }

    double get_cost() const { return cost; }
    double get_heuristic() const { return heuristic; }
    Node* get_parent() const { return parent; }

    void set_cost(double cost_) { cost = cost_; }
    void set_heuristic(double heuristic_) { heuristic = heuristic_; }
    // void set_t(int t_) {t = t_; }
    void set_parent(Node* parent_) { parent = parent_; }

    bool operator<(const Node& node) {
        return (heuristic + cost < node.heuristic + node.cost);
    }

    bool operator==(const Node& other) const {
        return (id == other.id) && (x == other.x) && (y == other.y);
    }

    void PrintNode() {
        std::cout << "id: " << id << " x: " << x << " y: " << y  << " cost: " << cost << " heur: " << heuristic << "\n";
    }

private:
    int id;
    int x;
    int y;
    Node* parent;
    // int t = 0;  // time step

    double cost = std::numeric_limits<double>::max();  // cost from start
    double heuristic = std::numeric_limits<double>::max();  // heuristic cost to goal

    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
    
};

// Класс для представления ребра графа
class Edge {
public:
    Node* source;
    Node* destination;
    double weight;
    
    Edge(Node* source, Node* destination, double weight) : source(source), destination(destination), weight(weight) {}
};


class Graph {
public:
    vector<Node*> nodes;
    unordered_map<Node*, vector<Edge*>> adjacency_list;

    Graph() = default;
    
    // Добавление вершины в граф
    void add_node(Node* node) {
        nodes.push_back(node);
        adjacency_list[node] = vector<Edge*>();
    }
    
    // Добавление ребра в граф
    void add_edge(Node* source, Node* destination, double weight) {
        Edge* edge = new Edge(source, destination, weight);
        adjacency_list[source].push_back(edge);
    }
    
    // Вывод списка смежности
    void print_adjacency_list()  {
        for (auto& pair : adjacency_list) {
            Node* node = pair.first;
            vector<Edge*>& edges = pair.second;
            node->PrintNode();
            cout << "Node " << node->get_id() << " is connected to:";
            for (Edge* edge : edges) {
                cout << " (" << edge->source->get_id() << " -> " << edge->destination->get_id() << ", weight: " << edge->weight << ")";
            }
            cout << endl;
        }
    }

    Node* GetNodeById(int id) const {
        for (Node* node : nodes) {
            if (node->get_id() == id) {
                return node;
            }
        }
        return nullptr; 
    }

    vector<Node*> GetNeighbors(Node* node)  {
        vector<Node*> neighbors;

        for (const auto& i : adjacency_list) {
            if (i.first->get_id() == node->get_id()) {
                Node* curr = GetNodeById(node->get_id());
                vector<Edge*> edges = adjacency_list.at(curr);
                for (Edge* edge : edges) {
                    neighbors.push_back(edge->destination);
                }
            }
        }
        return neighbors;
    }

    double GetEdgeCost(Node* node1, Node* node2) {
        for (const auto& i : adjacency_list) {
            if (i.first->get_id() == node1->get_id()) {
                for (const auto& node : i.second)
                    if (node->destination->get_id() == node2->get_id())
                        return node->weight;
            }
        }
        return std::numeric_limits<double>::max(); // node is unreachable
    }

    void PrintGraph() {
        cout << "Nodes in the graph:" << endl;
        for (Node* node : nodes) {
            cout << "Node " << node->get_id() << " at " << node << endl;
        }
        cout << "Edges in the graph:" << endl;
        for (auto& pair : adjacency_list) {
            vector<Edge*>& edges = pair.second;
            for (Edge* edge : edges) {
                cout << edge->source->get_id() << " (" << edge->source << ") -> " << 
                edge->destination->get_id() << " (" << edge->destination << "), weight: " << edge->weight << " at " << edge << endl;
            }
        }
    }

    void ClearParents() {
        for (auto& node : nodes) {
            node->set_parent(nullptr);
        }
    }

    void SetInfHeuristic() {
        for (auto& node : nodes) 
            node->set_heuristic(std::numeric_limits<double>::max());
    }

    void SetInfCost() {
        for (auto& node : nodes)
            node->set_cost(std::numeric_limits<double>::max());
    }
};

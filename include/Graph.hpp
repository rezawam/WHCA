#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>

using std::vector, std::unordered_map, std::cout, std::endl;

// Класс для представления вершины графа
class Node {
public:
    Node() {}
    Node(int id_, int x_, int y_) : id(id_), x(x_), y(y_) {};

    int get_id() const { return id; }
    int get_x() const { return x; }
    int get_y() const { return y; }
    int get_cost() const { return cost; }
    int get_heuristic() const { return heuristic; }

    void set_cost(double cost_) { cost = cost_; }
    void set_heuristic(double heuristic_) { heuristic = heuristic_; }

    bool operator<(const Node& node) {
        return (heuristic + cost < node.heuristic + node.cost);
    }

private:
    int id;
    int x;
    int y;
    double cost;
    double heuristic;
};

// Класс для представления ребра графа
class Edge {
public:
    Node* source;
    Node* destination;
    double weight;
    
    Edge(Node* source, Node* destination, double weight) : source(source), destination(destination), weight(weight) {}
};

// Класс для представления графа
class Graph {
public:
    vector<Node*> nodes;
    unordered_map<Node*, vector<Edge*>> adjacency_list;
    
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
    void print_adjacency_list() {
        for (auto& pair : adjacency_list) {
            Node* node = pair.first;
            vector<Edge*>& edges = pair.second;
            cout << "Node " << node->get_id() << " is connected to:";
            for (Edge* edge : edges) {
                cout << " (" << edge->source->get_id() << " -> " << edge->destination->get_id() << ", weight: " << edge->weight << ")";
            }
            cout << endl;
        }
    }

    vector<Node*> GetNeighbors(Node* node) const {
        vector<Node*> neighbors;
        if (adjacency_list.find(node) != adjacency_list.end()) {
            vector<Edge*> edges = adjacency_list.at(node);
            for (Edge* edge : edges) {
                neighbors.push_back(edge->destination);
            }
        }
        return neighbors;
    }

    void PrintGraph() {
        cout << "Nodes in the graph:" << endl;
        for (Node* node : nodes) {
            cout << "Node " << node->get_id() << endl;
        }
        cout << "Edges in the graph:" << endl;
        for (auto& pair : adjacency_list) {
            Node* node = pair.first;
            vector<Edge*>& edges = pair.second;
            for (Edge* edge : edges) {
                cout << edge->source->get_id() << " -> " << edge->destination->get_id() << ", weight: " << edge->weight << endl;
            }
        }
    }
};

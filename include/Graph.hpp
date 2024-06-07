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
    int get_t() const { return t; }
    double get_cost() const { return cost; }
    double get_heuristic() const { return heuristic; }
    Node* get_parent() const { return parent; }

    void set_cost(double cost_) { cost = cost_; }
    void set_heuristic(double heuristic_) { heuristic = heuristic_; }
    void set_t(int t_) {t = t_; }
    void set_parent(Node* parent_) { parent = parent_; }

    bool operator<(const Node& node) {
        return (heuristic + cost < node.heuristic + node.cost);
    }

    bool operator==(const Node& other) const {
        return (id == other.id) && (x == other.x) && (y == other.y) && (t == other.t);
    }

    void PrintNode() {
        std::cout << "id: " << id << " x: " << x << " y: " << y << " t: " << t << " cost: " << cost << " heur: " << heuristic << "\n";
    }

private:
    Node* parent;
    int id;
    int x;
    int y;
    int t = 0;  // time step
    double cost;  // cost from start
    double heuristic;  // heuristic cost to goal

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

// Класс для представления графа
class Graph {
public:

    Graph() { };
    Graph(const Graph& other) {
        nodes.reserve(other.nodes.size());
        for (const auto& node : other.nodes) {
            nodes.push_back(new Node(*node));
        }

        for (const auto& pair : other.adjacency_list) { 
            Node* new_node = GetNodeById(pair.first->get_id());
            std::vector<Edge*> new_edges;
            for (const auto& edge : pair.second) { 
                new_edges.push_back(new Edge(*edge)); 
                } 
            adjacency_list[new_node] = new_edges; 
        }
    }

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

    Node* GetNodeById(int id) {
        for (Node* node : nodes) {
            if (node->get_id() == id) {
                return node;
            }
        }
        return nullptr; 
    }

    vector<Node*> GetNeighbors(Node* node)  {
        vector<Node*> neighbors;

        // print_adjacency_list();
        // node->PrintNode();

        // for (const auto& i : adjacency_list) {
        //     cout << i.first->get_id() << ": " << i.first << " ";
        // }
        // cout << "\n";
        // cout << node << "\n";
        if (adjacency_list.find(node) != adjacency_list.end()) {
            // cout << "gud\n";
            vector<Edge*> edges = adjacency_list.at(node);
            for (Edge* edge : edges) {
                neighbors.push_back(edge->destination);
            }
        }
        return neighbors;

        // for (const auto& i : adjacency_list) {
        //     if (i.first->get_id() == node->get_id()) {
        //         cout << "gud\n";
                
        //         Node* curr = GetNodeById(node->get_id());

        //         vector<Edge*> edges = adjacency_list.at(curr);
        //         for (Edge* edge : edges) {
        //             neighbors.push_back(edge->destination);
        //         }
        //     }
        // }
        // return neighbors;
    }

    double GetEdgeCost(Node* node1, Node* node2) {

        // for (const auto& i : adjacency_list) {
        //     cout << i.first->get_id() << ": " << i.first << " ";
        // }
        // cout << "\n";
        // cout << node1 << "\n";
        for (const auto& e : adjacency_list.at(node1)) {
            if (e->destination == node2)
                return e->weight;
        }
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

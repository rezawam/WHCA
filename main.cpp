#include "include/Graph.hpp"

int main() {
    Graph graph;
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2,2 ,2);
    Node* node3 = new Node(3, 3, 3);
    
    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    
    graph.add_edge(node1, node2, 5);
    graph.add_edge(node1, node3, 3);
    graph.add_edge(node2, node3, 2);
    
    graph.PrintGraph();
    
    return 0;
}
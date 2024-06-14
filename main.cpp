#include "include/Graph.hpp"
#include "include/Solver.hpp"

int main() {
   Graph graph;
    Node* node1 = new Node(1, 1, 1);

    Node* node2 = new Node(2, 2 ,2);
    Node* node3 = new Node(3, 2 ,3);

    Node* node4 = new Node(4, 3, 2);
    Node* node5 = new Node(5, 4, 4);
    Node* node6 = new Node(6, 5, 5);
    
    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);
    graph.add_node(node5);
    graph.add_node(node6);
    
    graph.add_edge(node1, node2, 1);
    graph.add_edge(node2, node3, 1);
    graph.add_edge(node2, node4, 1);
    graph.add_edge(node3, node5, 1);
    graph.add_edge(node5, node6, 10000);
    graph.add_edge(node4, node6, 1000);

   // graph.PrintGraph();

    vector<Agent*> v;
    Agent a("a", node1, node6);
    Agent* b = &a;
    v.push_back(b);
    WHCAPathFinder pf(graph, v);
    std::cout << "Finding partial path...\n";
    Path p = pf.FindPortionPath(b);
    std::cout << "Done!\n";

    cout << "Curernt portion path is: ";
    for(const auto& n : a.portion_path)
       std::cout << n->get_id();
    cout << "\n";

   return 0;
}
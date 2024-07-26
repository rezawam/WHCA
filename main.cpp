#include "include/Graph.hpp"
#include "include/Solver.hpp"

int main() {
   Graph graph;

   // TEST CASE 1
   //  Node* node1 = new Node(1, 1, 1);

   //  Node* node2 = new Node(2, 2 ,2);
   //  Node* node3 = new Node(3, 2 ,3);

   //  Node* node4 = new Node(4, 3, 2);
   //  Node* node5 = new Node(5, 4, 4);
   //  Node* node6 = new Node(6, 5, 5);
    
   //  graph.add_node(node1);
   //  graph.add_node(node2);
   //  graph.add_node(node3);
   //  graph.add_node(node4);
   //  graph.add_node(node5);
   //  graph.add_node(node6);
    
   //  graph.add_edge(node1, node2, 1);
   //  graph.add_edge(node2, node3, 1);
   //  graph.add_edge(node2, node4, 1);
   //  graph.add_edge(node3, node5, 1);
   //  graph.add_edge(node4, node6, 100);
   //  graph.add_edge(node5, node6, 10);

   // graph.PrintGraph();

   // TEST CASE 2

   //  Node* node1 = new Node(1, 1, 1);

   //  Node* node2 = new Node(2, 2 ,2);
   //  Node* node3 = new Node(3, 2 ,3);

   //  Node* node4 = new Node(4, 3, 2);
   //  Node* node5 = new Node(5, 4, 4);
   //  Node* node6 = new Node(6, 5, 5);
   //  Node* node7 = new Node(7, 7, 7);
   //  Node* node8 = new Node(8, 8, 8);
    
   //  graph.add_node(node1);
   //  graph.add_node(node2);
   //  graph.add_node(node3);
   //  graph.add_node(node4);
   //  graph.add_node(node5);
   //  graph.add_node(node6);
   //  graph.add_node(node7);
   //  graph.add_node(node8);
    
   //  graph.add_edge(node1, node2, 1);
   //  graph.add_edge(node1, node3, 10);
   //  graph.add_edge(node2, node4, 1);
   //  graph.add_edge(node3, node5, 10);
   //  graph.add_edge(node4, node6, 1);
   //  graph.add_edge(node5, node7, 10);
   //  graph.add_edge(node6, node8, 1);
   //  graph.add_edge(node7, node8, 10);


   // TEST CASE 3

    Node* node1 = new Node(1, 1, 1);

    Node* node2 = new Node(2, 1 ,2);
    Node* node3 = new Node(3, 2 ,1);

    Node* node4 = new Node(4, 3, 1);
    Node* node5 = new Node(5, 3, 2);

    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);
    graph.add_node(node5);

    graph.add_edge(node1, node3, 1);
    graph.add_edge(node2, node3, 1);
    graph.add_edge(node3, node4, 1);
    graph.add_edge(node3, node5, 1);


    vector<Agent*> v;
    Agent a("b", node1, node5);
    Agent* b = &a;

    Agent c("c", node2, node4);
    Agent* d = &c;

    v.push_back(d);
    v.push_back(b);

    WHCAPathFinder pf(graph, v);

    std::cout << "Finding paths...\n";
    pf.FindPaths();
    std::cout << "Done!\n";

    cout << "Curernt portion path for a is: ";
    for(const auto& n : a.portion_path)
       std::cout << n->get_id();
    cout << "\n";

    cout << "Curernt portion path for c is: ";
    for(const auto& n : c.portion_path)
       std::cout << n->get_id();
    cout << "\n";


   // cout << "Graph 1: ";
   // graph.PrintGraph();
   // Graph graph_copy = graph;
   // cout << "Graph 2: ";
   // graph_copy.PrintGraph();

   return 0;
}
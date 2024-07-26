#include "../include/Solver.hpp"
#include "../include/Agent.hpp"
#include "../include/Graph.hpp"
#include <gtest/gtest.h>

void CheckPathsEquality(const Path& result, const Path& expected) {
    auto it1 = result.begin();
    auto it2 = expected.begin();
    while (it1 != result.end() && it2 != expected.end()) {
        ASSERT_EQ((*it1)->get_id(), (*it2)->get_id()) << "Paths are different";
        ++it1;
        ++it2;
    }
}

TEST(SingleAgentTest, TwoNodesInGraph) {
  Graph graph;
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2, 2 ,2);

    graph.add_node(node1);
    graph.add_node(node2);

    graph.add_edge(node1, node2, 1);

  Agent a("a", node1, node2);
  vector<Agent*> v;
  v.push_back(&a);
  WHCAPathFinder pf(graph, v);
  pf.FindPortionPath(&a);

  std::list<Node*> expected_path = { node1, node2 };

  ASSERT_EQ(a.portion_path.size(), expected_path.size()) << "Paths are of unequal length";
  CheckPathsEquality(a.portion_path, expected_path);
}

TEST(SingleAgentTest, TwoPathsDifferentEdgeWeights) {
    Graph graph;
    Node* node1 = new Node(1, 1, 1);

    Node* node2 = new Node(2, 2 ,2);
    Node* node3 = new Node(3, 2 ,3);

    Node* node4 = new Node(4, 3, 2);
    
    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);

    
    graph.add_edge(node1, node2, 1);
    graph.add_edge(node1, node3, 100);
    graph.add_edge(node2, node4, 1);
    graph.add_edge(node3, node4, 100);

  Agent a("a", node1, node4);
  vector<Agent*> v;
  v.push_back(&a);
  WHCAPathFinder pf(graph, v);
  pf.FindPortionPath(&a);

  std::list<Node*> expected_path = { node1, node2, node4, };

  ASSERT_EQ(a.portion_path.size(), expected_path.size()) << "Paths are of unequal length";
  CheckPathsEquality(a.portion_path, expected_path);
}

TEST(SingleAgentTest, TwoPathsDifferentHeuristics) {
    Graph graph;
    Node* node1 = new Node(1, 1, 1);

    Node* node2 = new Node(2, 1000, 200);
    Node* node3 = new Node(3, 2 ,3);

    Node* node4 = new Node(4, 3, 2);
    
    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);

    
    graph.add_edge(node1, node2, 1);
    graph.add_edge(node1, node3, 1);
    graph.add_edge(node2, node4, 1);
    graph.add_edge(node3, node4, 1);

  Agent a("a", node1, node4);
  vector<Agent*> v;
  v.push_back(&a);
  WHCAPathFinder pf(graph, v);
  pf.FindPortionPath(&a);

  std::list<Node*> expected_path = { node1, node3, node4, };

  ASSERT_EQ(a.portion_path.size(), expected_path.size()) << "Paths are of unequal length";
  CheckPathsEquality(a.portion_path, expected_path);
}

TEST(SingleAgentTest, LargeGraph) {
    Graph graph;
      Node* node1 = new Node(1, 1, 1);

    Node* node2 = new Node(2, 2 ,2);
    Node* node3 = new Node(3, 2 ,3);

    Node* node4 = new Node(4, 3, 2);
    Node* node5 = new Node(5, 4, 4);
    Node* node6 = new Node(6, 5, 5);
    Node* node7 = new Node(7, 7, 7);
    Node* node8 = new Node(8, 8, 8);
    
    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);
    graph.add_node(node5);
    graph.add_node(node6);
    graph.add_node(node7);
    graph.add_node(node8);
    
    graph.add_edge(node1, node2, 1);
    graph.add_edge(node1, node3, 1);
    graph.add_edge(node2, node4, 1);
    graph.add_edge(node3, node5, 1);
    graph.add_edge(node4, node6, 1);
    graph.add_edge(node5, node7, 1);
    graph.add_edge(node6, node8, 1);

  Agent a("a", node1, node8);
  vector<Agent*> v;
  v.push_back(&a);
  WHCAPathFinder pf(graph, v);
  pf.FindPortionPath(&a);

  std::list<Node*> expected_path = { node1, node2, node4, node6, node8 };

  ASSERT_EQ(a.portion_path.size(), expected_path.size()) << "Paths are of unequal length";
  CheckPathsEquality(a.portion_path, expected_path);
}

TEST(SingleAgentTest, RingGraph) {
  Graph graph;
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2, 2 ,2);
    Node* node3 = new Node(3, 3, 3);
    Node* node4 = new Node(4, 4 ,4);

    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);

    graph.add_edge(node1, node2, 1);
    graph.add_edge(node2, node3, 1);
    graph.add_edge(node3, node4, 1);
    graph.add_edge(node4, node1, 1);

  Agent a("a", node1, node4);
  vector<Agent*> v;
  v.push_back(&a);
  WHCAPathFinder pf(graph, v);
  pf.FindPortionPath(&a);

  std::list<Node*> expected_path = { node1, node2, node3, node4 };

  ASSERT_EQ(a.portion_path.size(), expected_path.size()) << "Paths are of unequal length";
  CheckPathsEquality(a.portion_path, expected_path);
}

TEST(SingleAgentTest, NoPathToGoal) {
  Graph graph;
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2, 2 ,2);

    graph.add_node(node1);
    graph.add_node(node2);

  Agent a("a", node1, node2);
  vector<Agent*> v;
  v.push_back(&a);
  WHCAPathFinder pf(graph, v);
  pf.FindPortionPath(&a);

  std::list<Node*> expected_path = { };

  ASSERT_EQ(a.portion_path.size(), expected_path.size()) << "Paths are of unequal length";
  CheckPathsEquality(a.portion_path, expected_path);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

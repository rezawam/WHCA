#include "../include/Solver.hpp"
#include "../include/Agent.hpp"
#include "../include/Graph.hpp"
#include <gtest/gtest.h>

void CheckPathsEquality(const Path& result, const Path& expected) {
    auto it1 = result.begin();
    auto it2 = expected.begin();
    while (it1 != result.end() && it2 != expected.end()) {
        ASSERT_EQ(*it1, *it2) << "Paths are different";
        ++it1;
        ++it2;
    }
}

TEST(GraphTest, NodeCopying) {
  Node* node = new Node(1, 1, 1);
  Node* parent = new Node(2, 2, 2);
  Node node_copy = *node;

  node->set_cost(12);
  node->set_heuristic(12);
  node->set_parent(parent);

  ASSERT_EQ(node_copy.get_parent(), nullptr);
  ASSERT_EQ(node_copy.get_cost(), std::numeric_limits<double>::max());
  ASSERT_EQ(node_copy.get_heuristic(), std::numeric_limits<double>::max());
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

  Path expected_path = { 1, 2 };

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

  Path expected_path = { 1, 2, 4 };

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

  Path expected_path = { 1, 3, 4, };

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

  Path expected_path = { 1, 2, 4, 6, 8 };

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

  Path expected_path = { 1, 2, 3, 4 };

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

  Path expected_path = { };

  ASSERT_EQ(a.portion_path.size(), expected_path.size()) << "Paths are of unequal length";
  CheckPathsEquality(a.portion_path, expected_path);
}

TEST(TwoAgentsTest, CrossGraph) {
  Graph graph;
    
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

    Agent a("a", node1, node5);
    Agent b("b", node2, node4);
  vector<Agent*> v;
  v.push_back(&a);
  v.push_back(&b);
  WHCAPathFinder pf(graph, v);
  pf.FindPaths();

  Path a_expected_path = {1, 3, 5};
  Path b_expected_path = {2, 2, 3, 4};

  ASSERT_EQ(a.portion_path.size(), a_expected_path.size()) << "a paths are of unequal length";
  ASSERT_EQ(b.portion_path.size(), b_expected_path.size()) << "b paths are of unequal length";
  CheckPathsEquality(a.portion_path,a_expected_path);
  CheckPathsEquality(b.portion_path, b_expected_path);
}

TEST(TwoAgentsTest, LineGraphFirstInVectorIsA) {
  Graph graph;
    
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2, 1, 2);
    Node* node3 = new Node(3, 1, 3);
    Node* node4 = new Node(4, 1, 4);

    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);

    graph.add_edge(node1, node2, 1);
    graph.add_edge(node2, node3, 1);
    graph.add_edge(node3, node4, 1);

    Agent a("a", node2, node4);
    Agent b("b", node1, node3);
  vector<Agent*> v;
  v.push_back(&a);
  v.push_back(&b);
  WHCAPathFinder pf(graph, v);
  pf.FindPaths();

  Path a_expected_path = {2, 3, 4};
  Path b_expected_path = {1, 2, 3};

  ASSERT_EQ(a.portion_path.size(), a_expected_path.size()) << "a paths are of unequal length";
  ASSERT_EQ(b.portion_path.size(), b_expected_path.size()) << "b paths are of unequal length";
  CheckPathsEquality(a.portion_path,a_expected_path);
  CheckPathsEquality(b.portion_path, b_expected_path);
}

TEST(TwoAgentsTest, LineGraphFirstInVectorIsB) {
  Graph graph;
    
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2, 1, 2);
    Node* node3 = new Node(3, 1, 3);
    Node* node4 = new Node(4, 1, 4);

    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);

    graph.add_edge(node1, node2, 1);
    graph.add_edge(node2, node3, 1);
    graph.add_edge(node3, node4, 1);

    Agent a("a", node2, node4);
    Agent b("b", node1, node3);
  vector<Agent*> v;
  v.push_back(&b);
  v.push_back(&a);
  WHCAPathFinder pf(graph, v);
  pf.FindPaths();

  Path a_expected_path = {2, 3, 4};
  Path b_expected_path = {1, 2, 3};

  ASSERT_EQ(a.portion_path.size(), a_expected_path.size()) << "a paths are of unequal length";
  ASSERT_EQ(b.portion_path.size(), b_expected_path.size()) << "b paths are of unequal length";
  CheckPathsEquality(a.portion_path,a_expected_path);
  CheckPathsEquality(b.portion_path, b_expected_path);
}

TEST(ThreeAgentsTest, CrossGraph) {
  Graph graph;

    Node* node0 = new Node(0, 0, 1);
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2, 1 ,2);
    Node* node3 = new Node(3, 2 ,1);
    Node* node4 = new Node(4, 3, 1);
    Node* node5 = new Node(5, 3, 2);
    Node* node6 = new Node(6, 6, 6);

    graph.add_node(node0);
    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);
    graph.add_node(node5);
    graph.add_node(node6);

    graph.add_edge(node0, node3, 1);
    graph.add_edge(node1, node3, 1);
    graph.add_edge(node2, node3, 1);
    graph.add_edge(node3, node4, 1);
    graph.add_edge(node3, node5, 1);
    graph.add_edge(node3, node6, 1);

    Agent a("a", node1, node5);
    Agent b("b", node2, node4);
    Agent c("c", node0, node6);
  vector<Agent*> v;
  v.push_back(&a);
  v.push_back(&b);
  v.push_back(&c);
  WHCAPathFinder pf(graph, v);
  pf.FindPaths();

  Path a_expected_path = {1, 3, 5};
  Path b_expected_path = {2, 2, 3, 4};
  Path c_expected_path = {0, 0, 0, 3, 6};

  ASSERT_EQ(a.portion_path.size(), a_expected_path.size()) << "a paths are of unequal length";
  ASSERT_EQ(b.portion_path.size(), b_expected_path.size()) << "b paths are of unequal length";
  ASSERT_EQ(c.portion_path.size(), c_expected_path.size()) << "c paths are of unequal length";
  CheckPathsEquality(a.portion_path,a_expected_path);
  CheckPathsEquality(b.portion_path, b_expected_path);
  CheckPathsEquality(c.portion_path, c_expected_path);
}

TEST(ThreeAgentsTest, LineGraph) {
  Graph graph;

    Node* node0 = new Node(0, 0, 1);
    Node* node1 = new Node(1, 1, 1);
    Node* node2 = new Node(2, 1 ,2);
    Node* node3 = new Node(3, 2 ,1);
    Node* node4 = new Node(4, 3, 1);
    Node* node5 = new Node(5, 3, 2);

    graph.add_node(node0);
    graph.add_node(node1);
    graph.add_node(node2);
    graph.add_node(node3);
    graph.add_node(node4);
    graph.add_node(node5);

    graph.add_edge(node0, node1, 1);
    graph.add_edge(node1, node2, 1);
    graph.add_edge(node2, node3, 1);
    graph.add_edge(node3, node4, 1);
    graph.add_edge(node4, node5, 1);

    Agent a("a", node0, node3);
    Agent b("b", node1, node4);
    Agent c("c", node2, node5);
  vector<Agent*> v;
  v.push_back(&a);
  v.push_back(&b);
  v.push_back(&c);
  WHCAPathFinder pf(graph, v);
  pf.FindPaths();

  Path a_expected_path = {0, 1, 2, 3};
  Path b_expected_path = {1, 2, 3, 4};
  Path c_expected_path = {2, 3, 4, 5};

  ASSERT_EQ(a.portion_path.size(), a_expected_path.size()) << "a paths are of unequal length";
  ASSERT_EQ(b.portion_path.size(), b_expected_path.size()) << "b paths are of unequal length";
  ASSERT_EQ(c.portion_path.size(), c_expected_path.size()) << "c paths are of unequal length";
  CheckPathsEquality(a.portion_path,a_expected_path);
  CheckPathsEquality(b.portion_path, b_expected_path);
  CheckPathsEquality(c.portion_path, c_expected_path);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

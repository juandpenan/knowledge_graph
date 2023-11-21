#include <gtest/gtest.h>
#include "knowledge_graph/knowledge_graph.hpp"

// Test fixture for graph operations
class GraphOperationsTest : public ::testing::Test
{
protected:
  knowledge_graph_interfaces::msg::Graph graph;

  void SetUp() override
  {
    // Initialize the graph before each test
    graph.nodes.clear();
    graph.edges.clear();
  }
};

// Tests for AddEdge functions
TEST_F(GraphOperationsTest, AddEdgeTest) {
  knowledge_graph_interfaces::msg::Edge edge;
  edge.source_node_id = "source";
  edge.destination_node_id = "destination";

  // Test AddEdge with Edge parameter
  EXPECT_TRUE(knowledge_graph::AddEdge(edge, graph));
  EXPECT_TRUE(knowledge_graph::AddEdge<bool>("source", "destination", true, graph));
  EXPECT_TRUE(knowledge_graph::AddEdge("source", "destination", 42, graph));

}

// Tests for RemoveEdges functions
TEST_F(GraphOperationsTest, RemoveEdgesTest) {
  knowledge_graph_interfaces::msg::Edge edge;
  edge.source_node_id = "source";
  edge.destination_node_id = "destination";
  knowledge_graph::AddEdge(edge, graph);   // Add edge to be removed

  // Test RemoveEdges with Edge parameter
  EXPECT_TRUE(knowledge_graph::RemoveEdges(edge, graph));

  // Test RemoveEdges with node IDs
  EXPECT_TRUE(knowledge_graph::RemoveEdges("source", "destination", graph));
  // Add more test cases for different scenarios if needed
}

// Tests for AddNode functions
TEST_F(GraphOperationsTest, AddNodeTest) {
  knowledge_graph_interfaces::msg::Content content;
  content.type = knowledge_graph_interfaces::msg::Content::BOOL;
  content.bool_value = true;

  // Test AddNode with Node parameter
  knowledge_graph_interfaces::msg::Node node;
  node.id = "node_id";
  node.content = content;
  EXPECT_TRUE(knowledge_graph::AddNode(node, graph));

  // Test AddNode with different data types
  EXPECT_TRUE(knowledge_graph::AddNode("node_id", content, graph));
  EXPECT_TRUE(knowledge_graph::AddNode("node_id", true, graph));
  EXPECT_TRUE(knowledge_graph::AddNode("node_id", 42, graph));
  // Add more test cases for other data types if needed
}

// Test for RemoveNode function
TEST_F(GraphOperationsTest, RemoveNodeTest) {
  knowledge_graph_interfaces::msg::Node node;
  node.id = "node_id";
  knowledge_graph::AddNode(node, graph);   // Add node to be removed

  // Test RemoveNode
  EXPECT_TRUE(knowledge_graph::RemoveNode("node_id", graph));
  // Add more test cases for different scenarios if needed
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <filesystem>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "knowledge_graph/knowledge_graph_server.hpp"

#include "knowledge_graph_interfaces/msg/node.hpp"
#include "knowledge_graph_interfaces/msg/edge.hpp"
#include "knowledge_graph_interfaces/msg/content.hpp"
#include "knowledge_graph_interfaces/msg/graph.hpp"

#include "knowledge_graph_interfaces/srv/add_edge.hpp"
#include "knowledge_graph_interfaces/srv/add_node.hpp"
#include "knowledge_graph_interfaces/srv/get_edges.hpp"
#include "knowledge_graph_interfaces/srv/get_nodes.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;

class TestKgServer : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);

    auto options = rclcpp::NodeOptions();

    kg_node = std::make_shared<knowledge_graph::KnowledgeGraphServer>(
      "knowledge_graph_server",
      "", options);
    kg_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    kg_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    add_node_req = std::make_shared<knowledge_graph_interfaces::srv::AddNode::Request>();
    get_node_req = std::make_shared<knowledge_graph_interfaces::srv::GetNodes::Request>();

    add_edge_req = std::make_shared<knowledge_graph_interfaces::srv::AddEdge::Request>();
    get_edge_req = std::make_shared<knowledge_graph_interfaces::srv::GetEdges::Request>();


  }

  void TearDown()
  {
    graph = nullptr;
    kg_node.reset();
    add_node_req.reset();
    get_node_req.reset();

    add_edge_req.reset();
    get_edge_req.reset();

    rclcpp::shutdown();
  }

  knowledge_graph_interfaces::msg::Graph::UniquePtr graph = nullptr;

  knowledge_graph_interfaces::srv::AddNode::Request::SharedPtr add_node_req;
  knowledge_graph_interfaces::srv::GetNodes::Request::SharedPtr get_node_req;

  knowledge_graph_interfaces::srv::AddEdge::Request::SharedPtr add_edge_req;
  knowledge_graph_interfaces::srv::GetEdges::Request::SharedPtr get_edge_req;

  geometry_msgs::msg::TransformStamped t;
  std::shared_ptr<knowledge_graph::KnowledgeGraphServer> kg_node;

};

template<class T>
typename T::Response::SharedPtr send_request(
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node,
  typename rclcpp::Client<T>::SharedPtr client,
  typename T::Request::SharedPtr request)
{
  auto result = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    return result.get();
  } else {
    return nullptr;
  }
}

TEST_F(TestKgServer, AddEdgeService)
{
  RCLCPP_INFO(kg_node->get_logger(), "Testing AddEdge service");

  auto client = kg_node->create_client<knowledge_graph_interfaces::srv::AddEdge>(
    "/add_edge");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<knowledge_graph_interfaces::srv::AddEdge>(
    kg_node->get_node_base_interface(), client, add_edge_req);

  // is false since the edge has no edges
  ASSERT_FALSE(resp->success);

}

TEST_F(TestKgServer, AddNodeService)
{
  RCLCPP_INFO(kg_node->get_logger(), "Testing AddNode service");
  auto client = kg_node->create_client<knowledge_graph_interfaces::srv::AddNode>(
    "/add_node");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<knowledge_graph_interfaces::srv::AddNode>(
    kg_node->get_node_base_interface(), client, add_node_req);
  ASSERT_TRUE(resp->success);
}

TEST_F(TestKgServer, AddNode)
{

  auto graph_callback =
    [&](knowledge_graph_interfaces::msg::Graph::UniquePtr msg) -> void {
      RCLCPP_INFO(kg_node->get_logger(), "Graph received");
      graph = std::move(msg);
    };

  auto sub =
    kg_node->create_subscription<knowledge_graph_interfaces::msg::Graph>(
    "graph",
    rclcpp::QoS(
      rclcpp::KeepLast(
        1)).transient_local(), graph_callback);

  auto node = knowledge_graph_interfaces::msg::Node();
  auto content = knowledge_graph::AddContent(6);

  RCLCPP_INFO(kg_node->get_logger(), "Testing AddNode");


  node.id = "test_node";
  node.content = content;
  add_node_req->node = node;

  auto client = kg_node->create_client<knowledge_graph_interfaces::srv::AddNode>(
    "/add_node");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<knowledge_graph_interfaces::srv::AddNode>(
    kg_node->get_node_base_interface(), client, add_node_req);

  ASSERT_TRUE(resp->success);
  EXPECT_NE(graph, nullptr);
  EXPECT_EQ(graph->nodes.size(), 1);
  EXPECT_EQ(graph->nodes[0].id, "test_node");
  EXPECT_EQ(graph->nodes[0].content.int_value, content.int_value);

}

TEST_F(TestKgServer, AddEdge)
{
  auto source_node = knowledge_graph_interfaces::msg::Node();
  auto destination_node = knowledge_graph_interfaces::msg::Node();

  source_node.id = "source_node";
  destination_node.id = "destination_node";

  auto graph_callback =
    [&](knowledge_graph_interfaces::msg::Graph::UniquePtr msg) -> void {
      RCLCPP_INFO(kg_node->get_logger(), "Graph received");
      graph = std::move(msg);
    };

  auto sub =
    kg_node->create_subscription<knowledge_graph_interfaces::msg::Graph>(
    "graph",
    rclcpp::QoS(
      rclcpp::KeepLast(
        1)).transient_local(), graph_callback);

  auto edge = knowledge_graph_interfaces::msg::Edge();

  auto node_client = kg_node->create_client<knowledge_graph_interfaces::srv::AddNode>(
    "/add_node");
  ASSERT_TRUE(node_client->wait_for_service());

  add_node_req->node = source_node;

  send_request<knowledge_graph_interfaces::srv::AddNode>(
    kg_node->get_node_base_interface(), node_client, add_node_req);

  add_node_req->node = destination_node;

  send_request<knowledge_graph_interfaces::srv::AddNode>(
    kg_node->get_node_base_interface(), node_client, add_node_req);


  t.header.stamp = kg_node->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "base_link";

  t.transform.translation.x = 0.5;
  t.transform.translation.y = 0.5;
  t.transform.translation.z = 0.5;

  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  auto content = knowledge_graph::AddContent(t, true);

  RCLCPP_INFO(kg_node->get_logger(), "Testing AddEdge");


  edge.source_node_id = "source_node";
  edge.destination_node_id = "destination_node";
  edge.content = content;
  add_edge_req->edge = edge;

  auto edge_client = kg_node->create_client<knowledge_graph_interfaces::srv::AddEdge>(
    "/add_edge");
  RCLCPP_INFO(kg_node->get_logger(), "AddEdge service is up");
  ASSERT_TRUE(edge_client->wait_for_service());

  auto edge_resp = send_request<knowledge_graph_interfaces::srv::AddEdge>(
    kg_node->get_node_base_interface(), edge_client, add_edge_req);


  ASSERT_TRUE(edge_resp->success);
  RCLCPP_INFO(kg_node->get_logger(), "Is graph empty ?");
  EXPECT_EQ(graph->nodes.size(), 2);
  EXPECT_EQ(graph->edges[0].content.tf_value.child_frame_id, "base_link");

}

// check that the edge is added to the graph
// check that the nodes and edges are removed
// check that the transform is published static and normal

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

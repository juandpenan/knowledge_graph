#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <filesystem>

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

  }

  void TearDown()
  {

    kg_node.reset();

    rclcpp::shutdown();
  }

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

TEST_F(TestKgServer, AddEdgeBasic)
{
  RCLCPP_INFO(kg_node->get_logger(), "Testing AddEdge service");
  auto req = std::make_shared<knowledge_graph_interfaces::srv::AddEdge::Request>();
  auto client = kg_node->create_client<knowledge_graph_interfaces::srv::AddEdge>(
    "/add_edge");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<knowledge_graph_interfaces::srv::AddEdge>(
    kg_node->get_node_base_interface(), client, req);
  ASSERT_TRUE(resp->success);

}

TEST_F(TestKgServer, AddNodeBasic)
{
  RCLCPP_INFO(kg_node->get_logger(), "Testing AddNode service");
  auto req = std::make_shared<knowledge_graph_interfaces::srv::AddNode::Request>();
  auto client = kg_node->create_client<knowledge_graph_interfaces::srv::AddNode>(
    "/add_node");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<knowledge_graph_interfaces::srv::AddNode>(
    kg_node->get_node_base_interface(), client, req);
  ASSERT_TRUE(resp->success);
}
// check that a nodde is added to the graph
// check that the edge is added to the graph
// check that the nodes and edges are removed 
// check that the transform is published static and normal

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

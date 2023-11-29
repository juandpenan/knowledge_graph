#include <memory>

#include "knowledge_graph/knowledge_graph_server.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto kg_node = std::make_shared<knowledge_graph::KnowledgeGraphServer>(
    "knowledge_graph_server",
    "", options);
  kg_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  kg_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::spin(kg_node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

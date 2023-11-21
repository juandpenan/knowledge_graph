#include <memory>

#include "knowledge_graph/knowledge_graph_server.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto kg_node = std::make_shared<knowledge_graph::KnowledgeGraphServer>();
  rclcpp::spin(kg_node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
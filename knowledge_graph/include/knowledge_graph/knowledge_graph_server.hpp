#ifndef KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_SERVER_HPP_
#define KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_SERVER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph/knowledge_graph_class.hpp"
#include "knowledge_graph_interfaces/msg/scene_graph.hpp"
#include "knowledge_graph_interfaces/srv/add_edge.hpp"
#include "knowledge_graph_interfaces/srv/add_node.hpp"
#include "knowledge_graph_interfaces/srv/get_edges.hpp"
#include "knowledge_graph_interfaces/srv/get_nodes.hpp"
#include "knowledge_graph_interfaces/srv/update_graph.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"


namespace knowledge_graph
{
class KnowledgeGraphServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit KnowledgeGraphServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
  );
  explicit KnowledgeGraphServer(
    const std::string & node_name,
    const std::string & ns,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
  );
  virtual ~KnowledgeGraphServer();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<knowledge_graph_interfaces::msg::SceneGraph>::SharedPtr graph_pub_;

  // rclcpp::Service<knowledge_graph_interfaces::srv::GetEdges>::SharedPtr get_edges_service_;
  // rclcpp::Service<knowledge_graph_interfaces::srv::GetNodes>::SharedPtr get_nodes_service_;
  // rclcpp::Service<knowledge_graph_interfaces::srv::AddEdge>::SharedPtr add_edge_service_;
  // rclcpp::Service<knowledge_graph_interfaces::srv::AddNode>::SharedPtr add_node_service_;
  rclcpp::Service<knowledge_graph_interfaces::srv::UpdateGraph>::SharedPtr update_graph_service_;
  knowledge_graph_interfaces::msg::SceneGraph graph_;
  pluginlib::ClassLoader<knowledge_graph::KnowledgeGraphBase> plugin_loader_{"knowledge_graph", "knowledge_graph::KnowledgeGraphBase"};
  
  std::string filename_;
  std::string frame_id_;
  std::string plugin_;
  std::shared_ptr<knowledge_graph::KnowledgeGraphBase> parser_;
  void UpdateGraphCallback(
    const std::shared_ptr<knowledge_graph_interfaces::srv::UpdateGraph::Request> request,
    std::shared_ptr<knowledge_graph_interfaces::srv::UpdateGraph::Response> response);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std_msgs::msg::Header header_;
};
} // namespace knowledge_graph

#endif // KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_SERVER_HPP_

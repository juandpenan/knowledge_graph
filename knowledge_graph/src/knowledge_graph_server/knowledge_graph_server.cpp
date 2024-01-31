#include <type_traits>
#include "knowledge_graph/knowledge_graph_server.hpp"
#include "std_msgs/msg/header.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;


namespace knowledge_graph
{
KnowledgeGraphServer::KnowledgeGraphServer(
  const rclcpp::NodeOptions & options
)
: rclcpp_lifecycle::LifecycleNode("knowledge_graph_server", "", options, false)
{}
KnowledgeGraphServer::KnowledgeGraphServer(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options
)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options, false)
{
  declare_parameter("file_name", rclcpp::PARAMETER_STRING);
  declare_parameter("plugin", rclcpp::PARAMETER_STRING);
  declare_parameter("frame_id", "map");
  RCLCPP_INFO(get_logger(), "KnowledgeGraphServer constructed");
}
KnowledgeGraphServer::~KnowledgeGraphServer()
{
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  filename_ = get_parameter("file_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  plugin_ = get_parameter("plugin").as_string();

  graph_ = knowledge_graph_interfaces::msg::SceneGraph();

  update_graph_service_ = create_service<knowledge_graph_interfaces::srv::UpdateGraph>(
    "update_graph",
    std::bind(&KnowledgeGraphServer::UpdateGraphCallback, this, _1, _2));

  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  graph_pub_ = create_publisher<knowledge_graph_interfaces::msg::SceneGraph>(
    "graph",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  parser_ = plugin_loader_.createSharedInstance(plugin_);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  graph_ = parser_->load_graph(filename_);
  graph_pub_->on_activate();
  graph_pub_->publish(graph_);

  header_ = std_msgs::msg::Header();
  header_.frame_id = frame_id_;
  header_.stamp = now();
  PublishTransforms(graph_, tf_static_broadcaster_, header_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  graph_pub_->on_deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  graph_pub_.reset();
  update_graph_service_.reset();
  // get_nodes_service_.reset();
  // add_edge_service_.reset();
  // add_node_service_.reset();

  graph_ = knowledge_graph_interfaces::msg::SceneGraph();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void KnowledgeGraphServer::UpdateGraphCallback(
  const std::shared_ptr<knowledge_graph_interfaces::srv::UpdateGraph::Request> request,
  std::shared_ptr<knowledge_graph_interfaces::srv::UpdateGraph::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received UpdateGraph request but not in ACTIVE state, ignoring!");
    response->success = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "Handling UpdateGraph request");
  // response->edges = GetEdges(request->source_node_id, request->destination_node_id, graph_);
  graph_pub_->publish(request->scene_graph);

  header_.frame_id = frame_id_;
  header_.stamp = now();
  PublishTransforms(request->scene_graph, tf_static_broadcaster_, header_);

  response->success = true;
}

} // namespace knowledge_graph


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(knowledge_graph::KnowledgeGraphServer)

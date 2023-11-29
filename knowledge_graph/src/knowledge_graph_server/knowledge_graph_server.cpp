#include <type_traits>
#include "knowledge_graph/knowledge_graph_server.hpp"


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
  // declare parameters
  RCLCPP_INFO(get_logger(), "KnowledgeGraphServer constructed");
}
KnowledgeGraphServer::~KnowledgeGraphServer()
{
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // (todo juandpenan) get params

  graph_ = knowledge_graph_interfaces::msg::Graph();

  get_edges_service_ = create_service<knowledge_graph_interfaces::srv::GetEdges>(
    "get_edges",
    std::bind(&KnowledgeGraphServer::GetEdgesCallback, this, _1, _2));
  get_nodes_service_ = create_service<knowledge_graph_interfaces::srv::GetNodes>(
    "get_nodes",
    std::bind(&KnowledgeGraphServer::GetNodesCallback, this, _1, _2));

  add_edge_service_ = create_service<knowledge_graph_interfaces::srv::AddEdge>(
    "add_edge",
    std::bind(&KnowledgeGraphServer::AddEdgeCallback, this, _1, _2));

  add_node_service_ = create_service<knowledge_graph_interfaces::srv::AddNode>(
    "add_node",
    std::bind(&KnowledgeGraphServer::AddNodeCallback, this, _1, _2));

  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  graph_pub_ = create_publisher<knowledge_graph_interfaces::msg::Graph>(
    "graph",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  graph_pub_->on_activate();
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
  get_edges_service_.reset();
  get_nodes_service_.reset();
  add_edge_service_.reset();
  add_node_service_.reset();

  graph_ = knowledge_graph_interfaces::msg::Graph();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KnowledgeGraphServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void KnowledgeGraphServer::GetEdgesCallback(
  const std::shared_ptr<knowledge_graph_interfaces::srv::GetEdges::Request> request,
  std::shared_ptr<knowledge_graph_interfaces::srv::GetEdges::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received GetEdges request but not in ACTIVE state, ignoring!");
    response->success = false;
    return;
  }
  // if graph is empty ignore reqeuest
  else if (graph_.nodes.empty() || graph_.edges.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Graph is empty, ignoring request!");
    response->success = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "Handling GetEdges request");
  response->edges = GetEdges(request->source_node_id, request->destination_node_id, graph_);
  response->success = true;
}

void KnowledgeGraphServer::GetNodesCallback(
  const std::shared_ptr<knowledge_graph_interfaces::srv::GetNodes::Request> request,
  std::shared_ptr<knowledge_graph_interfaces::srv::GetNodes::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received GetNodes request but not in ACTIVE state, ignoring!");
    response->success = false;
    return;
  }
  // if graph is empty ignore reqeuest
  else if (graph_.nodes.empty() || graph_.edges.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Graph is empty, ignoring request!");
    response->success = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "Handling GetNodes request");
  response->nodes = GetNodes(request->nodes_id, graph_);
  response->success = true;
}

void KnowledgeGraphServer::AddNodeCallback(
  const std::shared_ptr<knowledge_graph_interfaces::srv::AddNode::Request> request,
  std::shared_ptr<knowledge_graph_interfaces::srv::AddNode::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received AddNode request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling AddNode request");
  response->success = AddNode(request->node, graph_);
  if (request->node.content.type == knowledge_graph_interfaces::msg::Content::STATICTF &&
    !request->node.content.tf_value.header.frame_id.empty())
  {
    tf_static_broadcaster_->sendTransform(request->node.content.tf_value);
  } else if (request->node.content.type == knowledge_graph_interfaces::msg::Content::TF &&
    !request->node.content.tf_value.header.frame_id.empty())
  {
    tf_broadcaster_->sendTransform(request->node.content.tf_value);
  }

  graph_pub_->publish(graph_);
}

void KnowledgeGraphServer::AddEdgeCallback(
  const std::shared_ptr<knowledge_graph_interfaces::srv::AddEdge::Request> request,
  std::shared_ptr<knowledge_graph_interfaces::srv::AddEdge::Response> response)
{
  // if not in ACTIVE state, ignore request
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(
      get_logger(),
      "Received AddEdge request but not in ACTIVE state, ignoring!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Handling AddEdge request");
  response->success = AddEdge(request->edge, graph_);
  if (request->edge.content.type == knowledge_graph_interfaces::msg::Content::STATICTF &&
    !request->edge.content.tf_value.header.frame_id.empty())
  {
    tf_static_broadcaster_->sendTransform(request->edge.content.tf_value);
  } else if (request->edge.content.type == knowledge_graph_interfaces::msg::Content::TF &&
    !request->edge.content.tf_value.header.frame_id.empty())
  {
    tf_broadcaster_->sendTransform(request->edge.content.tf_value);
  }
  graph_pub_->publish(graph_);
}


} // namespace knowledge_graph


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(knowledge_graph::KnowledgeGraphServer)

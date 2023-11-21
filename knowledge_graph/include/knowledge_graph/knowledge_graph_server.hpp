#ifndef KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_SERVER_HPP_
#define KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_SERVER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "knowledge_graph/knowledge_graph.hpp"
#include "knowledge_graph_interfaces/srv/add_edge.hpp"
#include "knowledge_graph_interfaces/srv/add_node.hpp"
#include "knowledge_graph_interfaces/srv/get_edges.hpp"
#include "knowledge_graph_interfaces/srv/get_nodes.hpp"


namespace knowledge_graph
{
class KnowledgeGraphServer : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit KnowledgeGraphServer(
        // const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
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
    rclcpp_lifecycle::LifecyclePublisher<knowledge_graph_interfaces::msg::Graph>::SharedPtr graph_pub_;

    rclcpp::Service<knowledge_graph_interfaces::srv::GetEdges>::SharedPtr get_edges_service_;
    rclcpp::Service<knowledge_graph_interfaces::srv::GetNodes>::SharedPtr get_nodes_service_;
    rclcpp::Service<knowledge_graph_interfaces::srv::AddEdge>::SharedPtr add_edge_service_;
    rclcpp::Service<knowledge_graph_interfaces::srv::AddNode>::SharedPtr add_node_service_;

    knowledge_graph_interfaces::msg::Graph graph_;

    void GetNodesCallback(
        const std::shared_ptr<knowledge_graph_interfaces::srv::GetNodes::Request> request,
        std::shared_ptr<knowledge_graph_interfaces::srv::GetNodes::Response> response);

    void GetEdgesCallback(
        const std::shared_ptr<knowledge_graph_interfaces::srv::GetEdges::Request> request,
        std::shared_ptr<knowledge_graph_interfaces::srv::GetEdges::Response> response);
    
    void AddNodeCallback(
        const std::shared_ptr<knowledge_graph_interfaces::srv::AddNode::Request> request,
        std::shared_ptr<knowledge_graph_interfaces::srv::AddNode::Response> response);

    void AddEdgeCallback(
        const std::shared_ptr<knowledge_graph_interfaces::srv::AddEdge::Request> request,
        std::shared_ptr<knowledge_graph_interfaces::srv::AddEdge::Response> response);
};
} // namespace knowledge_graph

#endif // KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_SERVER_HPP_
#ifndef KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_
#define KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_

#include <optional>
#include <vector>
#include <memory>

#include "knowledge_graph_interfaces/msg/node.hpp"
#include "knowledge_graph_interfaces/msg/scene_graph.hpp"
#include "knowledge_graph_interfaces/msg/edge.hpp"
#include "knowledge_graph_interfaces/msg/content.hpp"
#include "knowledge_graph_interfaces/msg/graph.hpp"
#include <tf2_ros/static_transform_broadcaster.h>


namespace knowledge_graph
{
// enum class PluginList
// {
//   SceneGraphParser,
//   PddlParser,
  
// };

bool PublishTransforms(const knowledge_graph_interfaces::msg::SceneGraph & graph,
                       std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster,
                       const std_msgs::msg::Header& header);



} // namespace knowledge_graph


#endif // KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_

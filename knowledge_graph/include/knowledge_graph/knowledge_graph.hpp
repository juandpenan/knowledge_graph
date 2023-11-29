#ifndef KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_
#define KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_

#include <optional>
#include <vector>

#include "knowledge_graph_interfaces/msg/node.hpp"
#include "knowledge_graph_interfaces/msg/edge.hpp"
#include "knowledge_graph_interfaces/msg/content.hpp"
#include "knowledge_graph_interfaces/msg/graph.hpp"


namespace knowledge_graph
{

enum class ContentType
{
  Bool,
  Int,
  Float,
  Double,
  String,
  VBool,
  VDouble,
  VString,
  Pose,
  Tf,
  StaticTf,
  Vpose,
  VTf,
  Error,
  NumType,
};

template<class T>
knowledge_graph_interfaces::msg::Content AddContent(const T & content)
{
  // remove unused parameter warning
  static_cast<void>(content);
  knowledge_graph_interfaces::msg::Content ret;
  ret.type = knowledge_graph_interfaces::msg::Content::ERROR;

  return ret;
}
template<class T>
knowledge_graph_interfaces::msg::Content AddContent(const T & content, const bool tf_static)
{
  // remove unused parameter warning
  static_cast<void>(tf_static);
  static_cast<void>(content);
  knowledge_graph_interfaces::msg::Content ret;
  ret.type = knowledge_graph_interfaces::msg::Content::ERROR;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<bool>(const bool & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.bool_value = content;
  ret.type = knowledge_graph_interfaces::msg::Content::BOOL;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<int>(const int & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.int_value = content;
  ret.type = knowledge_graph_interfaces::msg::Content::INT;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<float>(const float & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.float_value = content;
  ret.type = knowledge_graph_interfaces::msg::Content::FLOAT;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<double>(const double & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.double_value = content;
  ret.type = knowledge_graph_interfaces::msg::Content::DOUBLE;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::string>(const std::string & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.string_value = content;
  ret.type = knowledge_graph_interfaces::msg::Content::STRING;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::vector<bool>>(const std::vector<bool> & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.bool_vector = content;
  ret.type = knowledge_graph_interfaces::msg::Content::VBOOL;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::vector<int>>(const std::vector<int> & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.int_vector = content;
  ret.type = knowledge_graph_interfaces::msg::Content::VINT;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::vector<float>>(const std::vector<float> & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.float_vector = content;
  ret.type = knowledge_graph_interfaces::msg::Content::VFLOAT;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::vector<double>>(const std::vector<double> & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.double_vector = content;
  ret.type = knowledge_graph_interfaces::msg::Content::VDOUBLE;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::vector<std::string>>(
  const std::vector<std::string> & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.string_vector = content;
  ret.type = knowledge_graph_interfaces::msg::Content::VSTRING;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<geometry_msgs::msg::PoseStamped>(
  const geometry_msgs::msg::PoseStamped & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.pose_value = content;
  ret.type = knowledge_graph_interfaces::msg::Content::POSE;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<geometry_msgs::msg::TransformStamped>(
  const geometry_msgs::msg::TransformStamped & content, const bool static_tf)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.tf_value = content;

  if (static_tf) {
    ret.type = knowledge_graph_interfaces::msg::Content::STATICTF;
  } else {
    ret.type = knowledge_graph_interfaces::msg::Content::TF;
  }

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::vector<geometry_msgs::msg::PoseStamped>>(
  const std::vector<geometry_msgs::msg::PoseStamped> & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.pose_vector = content;
  ret.type = knowledge_graph_interfaces::msg::Content::VPOSE;

  return ret;
}

template<>
knowledge_graph_interfaces::msg::Content
AddContent<std::vector<geometry_msgs::msg::TransformStamped>>(
  const std::vector<geometry_msgs::msg::TransformStamped> & content)
{
  knowledge_graph_interfaces::msg::Content ret;
  ret.tf_vector = content;
  ret.type = knowledge_graph_interfaces::msg::Content::VTF;

  return ret;
}

template<class T>
std::optional<T> GetContent(const knowledge_graph_interfaces::msg::Content & content)
{
  return {};
}

template<>
std::optional<bool> GetContent(const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::BOOL) {
    return content.bool_value;
  } else {
    return {};
  }
}

template<>
std::optional<int> GetContent(const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::INT) {
    return content.int_value;
  } else {
    return {};
  }
}

template<>
std::optional<float> GetContent(const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::FLOAT) {
    return content.float_value;
  } else {
    return {};
  }
}

template<>
std::optional<double> GetContent(const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::DOUBLE) {
    return content.double_value;
  } else {
    return {};
  }
}

template<>
std::optional<std::string> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::STRING) {
    return content.string_value;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<bool>> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::VBOOL) {
    return content.bool_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<int>> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::VINT) {
    return content.int_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<float>> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::VFLOAT) {
    return content.float_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<double>> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::VDOUBLE) {
    return content.double_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<std::string>> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::VSTRING) {
    return content.string_vector;
  } else {
    return {};
  }
}

template<>
std::optional<geometry_msgs::msg::PoseStamped> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::POSE) {
    return content.pose_value;
  } else {
    return {};
  }
}

template<>
std::optional<geometry_msgs::msg::TransformStamped> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::TF ||
    content.type == knowledge_graph_interfaces::msg::Content::STATICTF)
  {
    return content.tf_value;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<geometry_msgs::msg::PoseStamped>> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::VPOSE) {
    return content.pose_vector;
  } else {
    return {};
  }
}

template<>
std::optional<std::vector<geometry_msgs::msg::TransformStamped>> GetContent(
  const knowledge_graph_interfaces::msg::Content & content)
{
  if (content.type == knowledge_graph_interfaces::msg::Content::VTF) {
    return content.tf_vector;
  } else {
    return {};
  }
}


bool AddEdge(
  const knowledge_graph_interfaces::msg::Edge & edge,
  knowledge_graph_interfaces::msg::Graph graph)
{
  try {
    graph.edges.push_back(edge);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error adding edge: " << e.what() << std::endl;
    return false;
  }
}

template<class T>
bool AddEdge(
  const std::string & source_node, const std::string & destination_node,
  const T & content, knowledge_graph_interfaces::msg::Graph graph)
{
  knowledge_graph_interfaces::msg::Edge edge;

  edge.source_node_id = source_node;
  edge.destination_node_id = destination_node;

  try {
    edge.content = AddContent<T>(content, false);
    graph.edges.push_back(edge);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error adding edge: " << e.what() << std::endl;
    return false;
  }
}

bool AddEdge(
  const std::string & source_node, const std::string & destination_node,
  const knowledge_graph_interfaces::msg::Content & content,
  knowledge_graph_interfaces::msg::Graph & graph)
{
  knowledge_graph_interfaces::msg::Edge edge;

  edge.source_node_id = source_node;
  edge.destination_node_id = destination_node;
  edge.content = content;

  try {
    graph.edges.push_back(edge);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error adding edge: " << e.what() << std::endl;
    return false;
  }
}


bool RemoveEdges(
  const knowledge_graph_interfaces::msg::Edge & edge,
  knowledge_graph_interfaces::msg::Graph & graph)
{


  try {
    graph.edges.erase(
      std::remove_if(
        graph.edges.begin(), graph.edges.end(),
        [&](const knowledge_graph_interfaces::msg::Edge & current_edge) {
          return current_edge.source_node_id == edge.source_node_id &&
          current_edge.destination_node_id == edge.destination_node_id;
        }), graph.edges.end());
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error removing edge: " << e.what() << std::endl;
    return false;
  }

}

bool RemoveEdges(
  const std::string & source_node, const std::string & destination_node,
  knowledge_graph_interfaces::msg::Graph & graph)
{
  try {
    graph.edges.erase(
      std::remove_if(
        graph.edges.begin(), graph.edges.end(),
        [&](const knowledge_graph_interfaces::msg::Edge & current_edge) {
          return current_edge.source_node_id == source_node &&
          current_edge.destination_node_id == destination_node;
        }), graph.edges.end());
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error removing edge: " << e.what() << std::endl;
    return false;
  }
}

bool RemoveEdges(
  const std::string & node_id, knowledge_graph_interfaces::msg::Graph & graph)
{
  try {
    graph.edges.erase(
      std::remove_if(
        graph.edges.begin(), graph.edges.end(),
        [&](const knowledge_graph_interfaces::msg::Edge & current_edge) {
          return current_edge.source_node_id == node_id ||
          current_edge.destination_node_id == node_id;
        }), graph.edges.end());
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error removing edge: " << e.what() << std::endl;
    return false;
  }
}

bool RemoveEdges(
  const knowledge_graph_interfaces::msg::Node & node,
  knowledge_graph_interfaces::msg::Graph & graph)
{
  try {
    graph.edges.erase(
      std::remove_if(
        graph.edges.begin(), graph.edges.end(),
        [&](const knowledge_graph_interfaces::msg::Edge & current_edge) {
          return current_edge.source_node_id == node.id ||
          current_edge.destination_node_id == node.id;
        }), graph.edges.end());
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error removing edge: " << e.what() << std::endl;
    return false;
  }
}

bool AddNode(
  const knowledge_graph_interfaces::msg::Node & node,
  knowledge_graph_interfaces::msg::Graph & graph)
{
  try {
    graph.nodes.push_back(node);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error adding node: " << e.what() << std::endl;
    return false;
  }
}

bool AddNode(
  const std::string & node_id, const knowledge_graph_interfaces::msg::Content & content,
  knowledge_graph_interfaces::msg::Graph & graph)
{
  knowledge_graph_interfaces::msg::Node node;
  node.id = node_id;
  node.content = content;
  try {
    graph.nodes.push_back(node);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error adding node: " << e.what() << std::endl;
    return false;
  }
}

template<class T>
bool AddNode(
  const std::string & node_id, const T content,
  knowledge_graph_interfaces::msg::Graph & graph)
{
  knowledge_graph_interfaces::msg::Node node;
  node.id = node_id;
  node.content = AddContent<T>(content, false);
  try {
    graph.nodes.push_back(node);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error adding node: " << e.what() << std::endl;
    return false;
  }
}

bool RemoveNode(const std::string & node_id, knowledge_graph_interfaces::msg::Graph & graph)
{
  try {

    for (auto node = graph.nodes.begin(); node != graph.nodes.end(); ++node) {
      if (node->id == node_id) {
        graph.nodes.erase(node);
        RemoveEdges(node->id, graph);
        break;
      }
    }
    return true;

  } catch (const std::exception & e) {
    std::cerr << "Error removing node: " << e.what() << std::endl;
    return false;
  }
}


// // bool UpdateGraph
// // bool AppendGraph
std::vector<knowledge_graph_interfaces::msg::Edge> GetEdges(
  const std::string & source_node,
  const std::string & destination_node,
  const knowledge_graph_interfaces::msg::Graph & graph)
{
  std::vector<knowledge_graph_interfaces::msg::Edge> result_edges;

  for (const auto & edge : graph.edges) {
    if (edge.source_node_id == source_node && edge.destination_node_id == destination_node) {
      result_edges.push_back(edge);
    }
  }

  return result_edges;
}

std::optional<const knowledge_graph_interfaces::msg::Node> GetNode(
  const std::string & node_id,
  const knowledge_graph_interfaces::msg::Graph & graph)
{
  for (const auto & node : graph.nodes) {
    if (node.id == node_id) {
      return node;
    }
  }
  return {};
}

std::vector<knowledge_graph_interfaces::msg::Node> GetNodes(
  const std::vector<std::string> & node_ids, const knowledge_graph_interfaces::msg::Graph & graph)
{
  std::vector<knowledge_graph_interfaces::msg::Node> result_nodes;

  for (const auto & node_id : node_ids) {
    for (const auto & node : graph.nodes) {
      if (node.id == node_id) {
        result_nodes.push_back(node);
        break;
      }
    }
  }
  return result_nodes;
}

std::pair<std::optional<knowledge_graph_interfaces::msg::Node>,
  std::optional<knowledge_graph_interfaces::msg::Node>>
GetNodesFromEdge(
  const knowledge_graph_interfaces::msg::Edge & edge,
  const knowledge_graph_interfaces::msg::Graph & graph)
{
  std::optional<knowledge_graph_interfaces::msg::Node> source_node = std::nullopt;
  std::optional<knowledge_graph_interfaces::msg::Node> destination_node = std::nullopt;

  for (const auto & node : graph.nodes) {
    if (node.id == edge.source_node_id) {
      source_node = node;
    }
    if (node.id == edge.destination_node_id) {
      destination_node = node;
    }
    if (source_node.has_value() && destination_node.has_value()) {
      break;
    }
  }

  return {source_node, destination_node};
}


std::vector<std::pair<std::optional<knowledge_graph_interfaces::msg::Node>,
  std::optional<knowledge_graph_interfaces::msg::Node>>>
GetNodesFromEdges(
  const std::vector<knowledge_graph_interfaces::msg::Edge> & edges,
  const knowledge_graph_interfaces::msg::Graph & graph)
{
  std::vector<std::pair<std::optional<knowledge_graph_interfaces::msg::Node>,
    std::optional<knowledge_graph_interfaces::msg::Node>>> nodes_pairs;

  for (const auto & edge : edges) {
    std::optional<knowledge_graph_interfaces::msg::Node> source_node = std::nullopt;
    std::optional<knowledge_graph_interfaces::msg::Node> destination_node = std::nullopt;

    for (const auto & node : graph.nodes) {
      if (node.id == edge.source_node_id) {
        source_node = node;
      }
      if (node.id == edge.destination_node_id) {
        destination_node = node;
      }

      if (source_node.has_value() && destination_node.has_value()) {
        break;
      }
    }

    nodes_pairs.push_back({source_node, destination_node});
  }

  return nodes_pairs;
}


} // namespace knowledge_graph


#endif // KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_HPP_

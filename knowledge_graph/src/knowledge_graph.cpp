#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>

#include "knowledge_graph/knowledge_graph.hpp"

namespace knowledge_graph
{

// bool AddEdge(const knowledge_graph_interfaces::msg::Edge & edge, knowledge_graph_interfaces::msg::Graph graph)
// {
//     try {
//         graph.edges.push_back(edge);
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error adding edge: " << e.what() << std::endl;
//         return false;
//     }
// }

// template <class T>
// bool AddEdge(const std::string & source_node, const std::string & destination_node, const T & content, knowledge_graph_interfaces::msg::Graph graph)
// {
//     knowledge_graph_interfaces::msg::Edge  edge;

//     edge.source_node_id = source_node;
//     edge.destination_node_id = destination_node;

//     try {
//         edge.content = AddContent<T>(content, false);
//         graph.edges.push_back(edge);
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error adding edge: " << e.what() << std::endl;
//         return false;
//     }
// }

// bool AddEdge(const std::string & source_node, const std::string & destination_node, const knowledge_graph_interfaces::msg::Content & content,  knowledge_graph_interfaces::msg::Graph graph)
// {
//     knowledge_graph_interfaces::msg::Edge  edge;

//     edge.source_node_id = source_node;
//     edge.destination_node_id = destination_node;
//     edge.content = content;

//     try {
//         graph.edges.push_back(edge);
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error adding edge: " << e.what() << std::endl;
//         return false;
//     }
// }


// bool RemoveEdges(const knowledge_graph_interfaces::msg::Edge & edge, knowledge_graph_interfaces::msg::Graph graph)
// {


//     try {
//         graph.edges.erase(std::remove_if(graph.edges.begin(), graph.edges.end(), [&](const knowledge_graph_interfaces::msg::Edge & current_edge) {
//         return current_edge.source_node_id == edge.source_node_id && current_edge.destination_node_id == edge.destination_node_id ;
//         }), graph.edges.end());
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error removing edge: " << e.what() << std::endl;
//         return false;
//     }

// }

// bool RemoveEdges(const std::string & source_node, const std::string & destination_node, knowledge_graph_interfaces::msg::Graph graph)
// {
//     try {
//         graph.edges.erase(std::remove_if(graph.edges.begin(), graph.edges.end(), [&](const knowledge_graph_interfaces::msg::Edge & current_edge) {
//         return current_edge.source_node_id == source_node && current_edge.destination_node_id == destination_node ;
//         }), graph.edges.end());
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error removing edge: " << e.what() << std::endl;
//         return false;
//     }
// }

// bool AddNode(const knowledge_graph_interfaces::msg::Node & node, knowledge_graph_interfaces::msg::Graph graph)
// {
//     try {
//         graph.nodes.push_back(node);
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error adding node: " << e.what() << std::endl;
//         return false;
//     }
// }

// bool AddNode(const std::string & node_id,  const knowledge_graph_interfaces::msg::Content & content, knowledge_graph_interfaces::msg::Graph graph)
// {
//     knowledge_graph_interfaces::msg::Node node;
//     node.id = node_id;
//     node.content = content;
//     try {
//         graph.nodes.push_back(node);
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error adding node: " << e.what() << std::endl;
//         return false;
//     }
// }

// template <class T>
// bool AddNode(const std::string & node_id,  const T content, knowledge_graph_interfaces::msg::Graph graph)
// {
//     knowledge_graph_interfaces::msg::Node node;
//     node.id = node_id;
//     node.content = AddContent<T>(content, false);
//     try {
//         graph.nodes.push_back(node);
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error adding node: " << e.what() << std::endl;
//         return false;
//     }
// }

// bool RemoveNode(const std::string & node_id, knowledge_graph_interfaces::msg::Graph graph)
// {
//     try {
//         graph.nodes.erase(std::remove_if(graph.nodes.begin(), graph.nodes.end(), [&](const knowledge_graph_interfaces::msg::Node & current_node) {
//         return current_node.id == node_id;
//         }), graph.nodes.end());
//         return true;
//     } catch (const std::exception& e) {
//         std::cerr << "Error removing node: " << e.what() << std::endl;
//         return false;
//     }
// }

// template<class T>
// knowledge_graph_interfaces::msg::Content AddContent(const T & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.type = knowledge_graph_interfaces::msg::Content::ERROR;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<bool>(const bool & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.bool_value = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::BOOL;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<int>(const int & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.int_value = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::INT;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<float>(const float & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.float_value = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::FLOAT;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<double>(const double & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.double_value = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::DOUBLE;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::string>(const std::string & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.string_value = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::STRING;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::vector<bool>>(const std::vector<bool> & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.bool_vector = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::VBOOL;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::vector<int>>(const std::vector<int> & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.int_vector = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::VINT;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::vector<float>>(const std::vector<float> & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.float_vector = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::VFLOAT;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::vector<double>>(const std::vector<double> & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.double_vector = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::VDOUBLE;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::vector<std::string>>(
//   const std::vector<std::string> & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.string_vector = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::VSTRING;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<geometry_msgs::msg::PoseStamped>(
//   const geometry_msgs::msg::PoseStamped & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.pose_value = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::POSE;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<geometry_msgs::msg::TransformStamped>(
//   const geometry_msgs::msg::TransformStamped & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.tf_value = content;

//   if (static_tf) {
//     ret.type = knowledge_graph_interfaces::msg::Content::STATICTF;
//   } else {
//     ret.type = knowledge_graph_interfaces::msg::Content::TF;
//   }

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::vector<geometry_msgs::msg::PoseStamped>>(
//   const std::vector<geometry_msgs::msg::PoseStamped> & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.pose_vector = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::VPOSE;

//   return ret;
// }

// template<>
// knowledge_graph_interfaces::msg::Content
// AddContent<std::vector<geometry_msgs::msg::TransformStamped>>(
//   const std::vector<geometry_msgs::msg::TransformStamped> & content, const bool static_tf)
// {
//   knowledge_graph_interfaces::msg::Content ret;
//   ret.tf_vector = content;
//   ret.type = knowledge_graph_interfaces::msg::Content::VTF;

//   return ret;
// }
// bool AddEdge(const std::string & source, const std::string & destination, const T content,  knowledge_graph_interfaces::msg::Graph graph)
// {}

} // namespace knowledge_graph

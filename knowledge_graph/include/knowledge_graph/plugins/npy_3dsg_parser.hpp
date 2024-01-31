#ifndef KNOWLEDGE_GRAPH__PLUGINS__NPY_3DSG_PARSER_HPP_
#define KNOWLEDGE_GRAPH__PLUGINS__NPY_3DSG_PARSER_HPP_

#include "arch_utils/knowledge_graph/npy_parser1.hpp"
#include "knowledge_graph/knowledge_graph_class.hpp"
#include "knowledge_graph_interfaces/msg/building.hpp"
#include "knowledge_graph_interfaces/msg/room.hpp"
#include "knowledge_graph_interfaces/msg/object.hpp"
#include "knowledge_graph_interfaces/msg/scene_graph.hpp"
#include <vector>
// #include "knowledge_graph_interfaces/msg/robot.hpp"


namespace knowledge_graph
{

class SceneGraphParser : public KnowledgeGraphBase
{
public:
  SceneGraphParser();
  ~SceneGraphParser();
  
  knowledge_graph_interfaces::msg::SceneGraph  load_graph(const std::string & path) override;
  bool save_graph(const std::string & path, const bool & graph) override;
private:
  knowledge_graph_interfaces::msg::SceneGraph scene_graph_;
  std::vector<knowledge_graph_interfaces::msg::Object> objects_;
  std::vector<knowledge_graph_interfaces::msg::Room> rooms_;
  knowledge_graph_interfaces::msg::Building building_;

  // knowledge_graph_interfaces::msg::Robot robot_;
};
}



#endif  // KNOWLEDGE_GRAPH__PLUGINS__NPY_3DSG_PARSER_HPP_
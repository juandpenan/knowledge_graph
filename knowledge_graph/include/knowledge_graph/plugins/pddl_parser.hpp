#ifndef KNOWLEDGE_GRAPH__PLUGINS__PDDL_PARSER_HPP_
#define KNOWLEDGE_GRAPH__PLUGINS__PDDL_PARSER_HPP_

#include "knowledge_graph/knowledge_graph_class.hpp"
#include "knowledge_graph_interfaces/msg/scene_graph.hpp"
#include <sstream>

namespace knowledge_graph
{
class PddlParser : public KnowledgeGraphBase
{
public:
  PddlParser();
  ~PddlParser();
  
  knowledge_graph_interfaces::msg::SceneGraph  load_graph(const std::string & path) override;
  bool save_graph(const std::string & path, const bool & graph) override;
private:
  std::stringstream buffer_;

};
}
#endif  // KNOWLEDGE_GRAPH__PLUGINS__PDDL_PARSER_HPP_
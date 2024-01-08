#ifndef KNOWLEDGE_GRAPH__PLUGINS__PDDL_PARSER_HPP_
#define KNOWLEDGE_GRAPH__PLUGINS__PDDL_PARSER_HPP_

#include "knowledge_graph/knowledge_graph_class.hpp"

namespace knowledge_graph
{
class PddlParser : public KnowledgeGraphBase
{
public:
  PddlParser();
  ~PddlParser();
  
  bool load_graph(const std::string & path) override;
  bool save_graph(const std::string & path, const bool & graph) override;


};
}
#endif  // KNOWLEDGE_GRAPH__PLUGINS__PDDL_PARSER_HPP_
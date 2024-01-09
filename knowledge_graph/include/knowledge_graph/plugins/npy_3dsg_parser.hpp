#ifndef KNOWLEDGE_GRAPH__PLUGINS__NPY_3DSG_PARSER_HPP_
#define KNOWLEDGE_GRAPH__PLUGINS__NPY_3DSG_PARSER_HPP_

#include "arch_utils/knowledge_graph/npy_parser1.hpp"
#include "knowledge_graph/knowledge_graph_class.hpp"

namespace knowledge_graph
{

class SceneGraphParser : public KnowledgeGraphBase
{
public:
  SceneGraphParser();
  ~SceneGraphParser();
  
  bool load_graph(const std::string & path) override;
  bool save_graph(const std::string & path, const bool & graph) override;

};
}



#endif  // KNOWLEDGE_GRAPH__PLUGINS__NPY_3DSG_PARSER_HPP_
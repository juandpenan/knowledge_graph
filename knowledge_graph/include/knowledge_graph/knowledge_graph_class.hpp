#ifndef KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_CLASS_HPP_
#define KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_CLASS_HPP_
#include <fstream>


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/pending/property.hpp>
#include <boost/graph/filtered_graph.hpp>
#include "knowledge_graph_interfaces/msg/scene_graph.hpp"

using graph = boost::adjacency_list<boost::listS, boost::vecS>;

namespace knowledge_graph
{

class KnowledgeGraphBase
{
private:
  
public:
  // boost::adjacency_list<> current_graph(N);
  
  
  // virtual graph load_graph(const std::string & path) = 0; 
  virtual knowledge_graph_interfaces::msg::SceneGraph load_graph(const std::string & path) = 0; 
  virtual bool save_graph(const std::string & path, const bool & graph) = 0;
  
  virtual ~KnowledgeGraphBase() {}

protected:
  std::ifstream in_file;
  KnowledgeGraphBase(/* args */);



};



    
} // namespace knowledge_graph

#endif  // KNOWLEDGE_GRAPH__KNOWLEDGE_GRAPH_CLASS_HPP_
#include <memory>
#include <string>

#include "knowledge_graph/plugins/pddl_parser.hpp"
#include "plansys2_pddl_parser/Instance.h"
#include "ament_index_cpp/get_package_share_directory.hpp"


using graph = boost::adjacency_list<boost::listS, boost::vecS>;
namespace knowledge_graph
{

PddlParser::PddlParser(/* args */)
: KnowledgeGraphBase()
{
}
PddlParser::~PddlParser()
{
}
knowledge_graph_interfaces::msg::SceneGraph PddlParser::load_graph(const std::string & path)
{ 
    // todo(juandpenan) make the path a param
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_pddl_parser");
  std::string domain_file = pkgpath + "/pddl/domain.pddl";


  std::ifstream domain_ifs(domain_file);
 
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
                         std::istreambuf_iterator<char>());
  
 


 
  bool okparse = false;
  bool okprint = false;
  try {
    parser::pddl::Domain domain( domain_str );
    
    okparse = true;
    try {
      std::cout << domain << std::endl;
   
      okprint = true;
    } catch (std::runtime_error e) {
      std::cout << "Error printing domain" << e.what() <<std::endl;
    }
  } catch (std::runtime_error e) {
     std::cout << "Error printing domain" << e.what() <<std::endl;
  }
  if (!okprint) { std::cout << "Error parsing domain" << std::endl; return knowledge_graph_interfaces::msg::SceneGraph(); }

    return knowledge_graph_interfaces::msg::SceneGraph();
} 
bool PddlParser::save_graph(const std::string & path, const bool & graph) {

    return true; // Return true if saving is successful
}


} // namespace knowledge_graph


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(knowledge_graph::PddlParser, knowledge_graph::KnowledgeGraphBase)
#include <memory>
#include <string>
#include <sstream>
#include "knowledge_graph/plugins/pddl_parser.hpp"
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
bool PddlParser::load_graph(const std::string & path)
{ 
    std::string domain_pth = ament_index_cpp::get_package_share_directory("knowledge_graph") + "/pddl/domain.pddl"; 
    return false;
} 
bool PddlParser::save_graph(const std::string & path, const bool & graph) {

    return true; // Return true if saving is successful
}


} // namespace knowledge_graph


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(knowledge_graph::PddlParser, knowledge_graph::KnowledgeGraphBase)
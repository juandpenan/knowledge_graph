#include "knowledge_graph/plugins/npy_3dsg_parser.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"



namespace knowledge_graph
{

SceneGraphParser::SceneGraphParser(/* args */)
: KnowledgeGraphBase()
{
}
SceneGraphParser::~SceneGraphParser()
{
}
bool SceneGraphParser::load_graph(const std::string & path)
{ 
    std::string pkgpath = ament_index_cpp::get_package_share_directory("knowledge_graph");
    cnpy::NpyArray arr = cnpy::npy_load(pkgpath + "/3dsg/3DSceneGraph_Adairsville.npz");
    return true;
} 
bool SceneGraphParser::save_graph(const std::string & path, const bool & graph) {

    return true; // Return true if saving is successful
}


} // namespace knowledge_graph


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(knowledge_graph::SceneGraphParser, knowledge_graph::KnowledgeGraphBase)
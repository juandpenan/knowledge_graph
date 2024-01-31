#include "knowledge_graph/plugins/npy_3dsg_parser.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// #include <Python.h>
#include <filesystem>
#include <cstdlib>
// #include <json/json.h>
#include <yaml-cpp/yaml.h>
#include <fstream>



namespace knowledge_graph
{

SceneGraphParser::SceneGraphParser(/* args */)
: KnowledgeGraphBase()
{
}
SceneGraphParser::~SceneGraphParser()
{
}
knowledge_graph_interfaces::msg::SceneGraph SceneGraphParser::load_graph(const std::string & path)
{ 
    std::filesystem::path file_path(path);

    // std::string pkgpath = ament_index_cpp::get_package_share_directory("knowledge_graph");
    // std::string filepath = pkgpath + "/3dsg/3DSceneGraph_Anaheim.npz";
    std::string script = ament_index_cpp::get_package_share_directory("arch_utils");
    script.append("/python/knowledge_graph/npz_parser.py");
    std::string command = "python3 " + script + " --file_path "+ path;
  
    YAML::Node scene_graph_yaml;

    std::system(command.c_str());

    scene_graph_yaml = YAML::LoadFile("/tmp/"+file_path.stem().string()+".yaml");
  
    building_.id = scene_graph_yaml["building"]["id"].as<int>();
    building_.name = scene_graph_yaml["building"]["name"].as<std::string>();

    building_.reference_point.x = scene_graph_yaml["building"]["reference_point"][0].as<double>();
    building_.reference_point.y = scene_graph_yaml["building"]["reference_point"][1].as<double>();
    building_.reference_point.z = scene_graph_yaml["building"]["reference_point"][2].as<double>(); 

    building_.size[0] = scene_graph_yaml["building"]["size"][0].as<double>();
    building_.size[1] = scene_graph_yaml["building"]["size"][1].as<double>();
    building_.size[2] = scene_graph_yaml["building"]["size"][2].as<double>();

    for (std::size_t i = 1; i <= scene_graph_yaml["object"].size(); ++i) 
    {
        knowledge_graph_interfaces::msg::Object object;

        if (scene_graph_yaml["object"][i]["id"].IsDefined() && scene_graph_yaml["object"][i]["id"].IsScalar()) {
            object.id = scene_graph_yaml["object"][i]["id"].as<int>();
        } else {
            // Handle the case when "id" is not present or not convertible to int
            std::cerr << "Error: Invalid or missing 'id' field in YAML.\n";
            continue;  // Skip this object and move to the next one
        }
        object.object_class = scene_graph_yaml["object"][i]["class_"].as<std::string>();

   
        if (!scene_graph_yaml["object"][i]["material"].IsNull()) {
            object.material = scene_graph_yaml["object"][i]["material"].as<std::string>();
        }

        if (!scene_graph_yaml["object"][i]["parent_room"].IsNull()) {
            object.parent_room = scene_graph_yaml["object"][i]["parent_room"].as<int>();
        }

        // Check if the node exists before accessing its value
        if (!scene_graph_yaml["object"][i]["tactile_texture"].IsNull()) {
            object.tactile_texture = scene_graph_yaml["object"][i]["tactile_texture"].as<std::string>();
        }

        // Check if the node exists before accessing its value
        if (!scene_graph_yaml["object"][i]["visual_texture"].IsNull()) {
            object.visual_texture = scene_graph_yaml["object"][i]["visual_texture"].as<std::string>();
        }

        object.size[0] = scene_graph_yaml["object"][i]["size"][0].as<double>();
        object.size[1] = scene_graph_yaml["object"][i]["size"][1].as<double>();
        object.size[2] = scene_graph_yaml["object"][i]["size"][2].as<double>();

        if (scene_graph_yaml["object"][i]["action_affordance"].IsSequence()) {
            for (std::size_t j = 0; j < scene_graph_yaml["object"][i]["action_affordance"].size(); ++j) {
                // Check if the node exists before accessing its value
                if (!scene_graph_yaml["object"][i]["action_affordance"][j].IsNull()) {
                    object.action_affordance.push_back(scene_graph_yaml["object"][i]["action_affordance"][j].as<std::string>());
                }
            }
        }
        objects_.push_back(object);
    }

    for (std::size_t i = 1; i <= scene_graph_yaml["room"].size(); ++i) 
    {
        knowledge_graph_interfaces::msg::Room room;

        // Check if the "floor_number" node exists and is convertible to int
        if (scene_graph_yaml["room"][i]["floor_number"].IsDefined() ) {
            room.floor_number = scene_graph_yaml["room"][i]["floor_number"].as<char>();
        } else {
            // Handle the case when "floor_number" is not present or not convertible to int
            std::cerr << "Error: Invalid or missing 'floor_number' field in YAML for room " << i << ".\n";
            continue;  // Skip this room and move to the next one
        }

        // Check if the "id" node exists and is convertible to int
        if (scene_graph_yaml["room"][i]["id"].IsDefined() && scene_graph_yaml["room"][i]["id"].IsScalar()) {
            room.id = scene_graph_yaml["room"][i]["id"].as<int>();
        } else {
            // Handle the case when "id" is not present or not convertible to int
            std::cerr << "Error: Invalid or missing 'id' field in YAML for room " << i << ".\n";
            continue;  // Skip this room and move to the next one
        }

        // Check if the "location" node exists and is a sequence with three elements
        if (scene_graph_yaml["room"][i]["location"].IsDefined() &&
            scene_graph_yaml["room"][i]["location"].IsSequence() &&
            scene_graph_yaml["room"][i]["location"].size() == 3) {
            room.location.x = scene_graph_yaml["room"][i]["location"][0].as<double>();
            room.location.y = scene_graph_yaml["room"][i]["location"][1].as<double>();
            room.location.z = scene_graph_yaml["room"][i]["location"][2].as<double>();
        } else {
            // Handle the case when "location" is not present or has an invalid structure
            std::cerr << "Error: Invalid or missing 'location' field in YAML for room " << i << ".\n";
            continue;  // Skip this room and move to the next one
        }

        // Check if the "parent_building" node exists and is convertible to int
        if (scene_graph_yaml["room"][i]["parent_building"].IsDefined() && scene_graph_yaml["room"][i]["parent_building"].IsScalar()) {
            room.parent_building = scene_graph_yaml["room"][i]["parent_building"].as<int>();
        } else {
            // Handle the case when "parent_building" is not present or not convertible to int
            std::cerr << "Error: Invalid or missing 'parent_building' field in YAML for room " << i << ".\n";
            continue;  // Skip this room and move to the next one
        }

        // Check if the "scene_category" node exists and is convertible to string
        if (scene_graph_yaml["room"][i]["scene_category"].IsDefined() && scene_graph_yaml["room"][i]["scene_category"].IsScalar()) {
            room.scene_category = scene_graph_yaml["room"][i]["scene_category"].as<std::string>();
        } else {
            // Handle the case when "scene_category" is not present or not convertible to string
            std::cerr << "Error: Invalid or missing 'scene_category' field in YAML for room " << i << ".\n";
            continue;  // Skip this room and move to the next one
        }

        rooms_.push_back(room);
    }

    scene_graph_.building = building_;
    scene_graph_.rooms = rooms_;
    scene_graph_.objects = objects_;

    return scene_graph_;
} 
bool SceneGraphParser::save_graph(const std::string & path, const bool & graph) {

    return true; // Return true if saving is successful
}


} // namespace knowledge_graph


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(knowledge_graph::SceneGraphParser, knowledge_graph::KnowledgeGraphBase)
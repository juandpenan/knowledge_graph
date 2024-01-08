#include <pluginlib/class_loader.hpp>
#include <knowledge_graph/knowledge_graph_class.hpp>

int main(int argc, char** argv)
{
  // To avoid unused parameter warnings
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<knowledge_graph::KnowledgeGraphBase> pddl_loader("knowledge_graph", "knowledge_graph::KnowledgeGraphBase");

  try
  {
    std::shared_ptr<knowledge_graph::KnowledgeGraphBase> pddl_parser = pddl_loader.createSharedInstance("knowledge_graph::PddlParser");
   
    printf("The plugin was loaded successfully\n");

  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}

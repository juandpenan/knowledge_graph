cmake_minimum_required(VERSION 3.8)
project(knowledge_graph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(knowledge_graph_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(arch_utils REQUIRED)

set(dependencies
    rclcpp
    knowledge_graph_interfaces
    tf2
    tf2_geometry_msgs
    lifecycle_msgs
    rclcpp_lifecycle
    rclcpp_components
    pluginlib
    plansys2_pddl_parser
    arch_utils
)
include_directories(include)

# plugins
add_library(parsers SHARED
  plugins/pddl_parser.cpp
  plugins/npy_3dsg_parser.cpp
)
target_include_directories(parsers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(parsers
  ${dependencies}
)
target_link_libraries(parsers ${PROJECT_NAME})


pluginlib_export_plugin_description_file(knowledge_graph plugins.xml)

add_library(${PROJECT_NAME} SHARED
  src/knowledge_graph_server/knowledge_graph_server.cpp
  src/knowledge_graph_class.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(graph_server src/knowledge_graph_server/main.cpp)
ament_target_dependencies(graph_server ${dependencies})
target_link_libraries(graph_server ${PROJECT_NAME})

add_executable(graph_plugin src/plugin_loader.cpp)
ament_target_dependencies(graph_plugin ${dependencies})
target_link_libraries(graph_plugin parsers ${PROJECT_NAME})


rclcpp_components_register_nodes(${PROJECT_NAME}
    PLUGIN "knowledge_graph::KnowledgeGraphServer"
)

install(TARGETS
  ${PROJECT_NAME}
  graph_server
  graph_plugin
  parsers

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY pddl
 DESTINATION share/${PROJECT_NAME})

 install(DIRECTORY 3dsg
 DESTINATION share/${PROJECT_NAME})

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}
  parsers
  # 3dsg_graph
)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
    rclcpp
    knowledge_graph_interfaces
    tf2
    tf2_geometry_msgs
    lifecycle_msgs
    rclcpp_lifecycle
    rclcpp_components
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/knowledge_graph_server/knowledge_graph_server.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(graph_server src/knowledge_graph_server/main.cpp)
ament_target_dependencies(graph_server ${dependencies})
target_link_libraries(graph_server ${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME}
    PLUGIN "knowledge_graph::KnowledgeGraphServer"
)

install(TARGETS
  ${PROJECT_NAME}
  graph_server

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ament_export_include_directories(include)
# ament_export_dependencies(${dependencies})
# ament_export_libraries(${PROJECT_NAME})

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

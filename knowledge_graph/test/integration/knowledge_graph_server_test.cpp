// #include <gtest/gtest.h>

// #include <string>
// #include <memory>
// #include <filesystem>

// #include <rclcpp/rclcpp.hpp>

// #include "knowledge_graph/knowledge_graph_server.hpp"

// #include "knowledge_graph_interfaces/msg/node.hpp"
// #include "knowledge_graph_interfaces/msg/edge.hpp"
// #include "knowledge_graph_interfaces/msg/content.hpp"
// #include "knowledge_graph_interfaces/msg/graph.hpp"

// #include "knowledge_graph_interfaces/srv/add_edge.hpp"
// #include "knowledge_graph_interfaces/srv/add_node.hpp"
// #include "knowledge_graph_interfaces/srv/get_edges.hpp"
// #include "knowledge_graph_interfaces/srv/get_nodes.hpp"


// using namespace std::chrono_literals;
// using namespace rclcpp;  // NOLINT

// #define TEST_DIR TEST_DIRECTORY

// using std::filesystem::path;

// using lifecycle_msgs::msg::Transition;

// class RclCppFixture
// {
// public:
//   RclCppFixture() {rclcpp::init(0, nullptr);}
//   ~RclCppFixture() {rclcpp::shutdown();}
// };

// RclCppFixture g_rclcppfixture;

// class KgServerTest : public ::testing::Test
// {
// public:
//   static void SetUpTestCase()
//   {
//     node_ = rclcpp::Node::make_shared("kg_client_test");
//     server_ = std::make_shared<knowledge_graph::KnowledgeGraphServer>();

//     executor_.add_node(node_);
//     executor_.add_node(server_>get_node_base_interface());
//     RCLCPP_INFO(node_->get_logger(), "Creating Test Node");

//     std::this_thread::sleep_for(std::chrono::seconds(5));  // allow node to start up
//     const std::chrono::seconds timeout(5);

//     kg_node_ = knowledge_graph_interfaces::msg::Node();
//     kg_edge_ = knowledge_graph_interfaces::msg::Edge();
//     kg_graph_ = knowledge_graph_interfaces::msg::Graph();
//     kg_content_ = knowledge_graph_interfaces::msg::Content();
//   }

//   static void TearDownTestCase()
//   {
//     node_.reset();

//     kg_node_ = knowledge_graph_interfaces::msg::Node();
//     kg_edge_ = knowledge_graph_interfaces::msg::Edge();
//     kg_graph_ = knowledge_graph_interfaces::msg::Graph();
//     kg_content_ = knowledge_graph_interfaces::msg::Content();
//   }

//   template<class T>
//   typename T::Response::SharedPtr send_request(
//     rclcpp::executors::SingleThreadedExecutor exec,
//     typename rclcpp::Client<T>::SharedPtr client,
//     typename T::Request::SharedPtr request)
//   {
//     auto result = client->async_send_request(request);

//     // Wait for the result
//     if (exec.spin_until_future_complete(result) == rclcpp::FutureReturnCode::SUCCESS) {
//       return result.get();
//     } else {
//       return nullptr;
//     }
//   }

// protected:
//   // Check that map_msg corresponds to reference pattern
//   // Input: map_msg
// //   void verifyMapMsg(const nav_msgs::msg::OccupancyGrid & map_msg)
// //   {
// //     ASSERT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
// //     ASSERT_EQ(map_msg.info.width, g_valid_image_width);
// //     ASSERT_EQ(map_msg.info.height, g_valid_image_height);
// //     for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
// //       ASSERT_EQ(g_valid_image_content[i], map_msg.data[i]);
// //     }
// //   }

//   static rclcpp::Node::SharedPtr node_;
//   static std::shared_ptr<knowledge_graph::KnowledgeGraphServer> server_;
//   static knowledge_graph_interfaces::msg::Node kg_node_;
//   static knowledge_graph_interfaces::msg::Edge kg_edge_;
//   static knowledge_graph_interfaces::msg::Graph kg_graph_;
//   static knowledge_graph_interfaces::msg::Content kg_content_;
//   static rclcpp::executors::SingleThreadedExecutor executor_;
  
// };


// rclcpp::Node::SharedPtr KgServerTest::node_ = nullptr;
// std::shared_ptr<knowledge_graph::KnowledgeGraphServer> KgServerTest::server_ = nullptr;
// knowledge_graph_interfaces::msg::Node
//   KgServerTest::kg_node_ = knowledge_graph_interfaces::msg::Node();
// knowledge_graph_interfaces::msg::Edge
//   KgServerTest::kg_edge_ = knowledge_graph_interfaces::msg::Edge();
// knowledge_graph_interfaces::msg::Graph
//   KgServerTest::kg_graph_ = knowledge_graph_interfaces::msg::Graph();
// knowledge_graph_interfaces::msg::Content
//   KgServerTest::kg_content_ = knowledge_graph_interfaces::msg::Content();
// // std::shared_ptr<nav2_util::LifecycleServiceClient> MapServerTestFixture::lifecycle_client_ =
// //   nullptr;


// // Send map getting service request and verify obtained OccupancyGrid
// TEST_F(KgServerTest, AddNode)
// {
//   RCLCPP_INFO(node_->get_logger(), "Testing AddNode service");
//   auto req = std::make_shared<knowledge_graph_interfaces::srv::AddNode::Request>();

//   auto client = node_->create_client<knowledge_graph_interfaces::srv::AddNode>(
//     "add_node");

//   RCLCPP_INFO(node_->get_logger(), "Waiting for map service");
//   ASSERT_TRUE(client->wait_for_service());

//   auto resp = send_request<knowledge_graph_interfaces::srv::AddNode>(node_, client, req);

//   // verifyMapMsg(resp->map);
// }

// // // Send map loading service request and verify obtained OccupancyGrid
// // TEST_F(MapServerTestFixture, LoadMap)
// // {
// //   RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
// //   auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
// //   auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
// //     "/map_server/load_map");

// //   RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
// //   ASSERT_TRUE(client->wait_for_service());

// //   req->map_url = path(TEST_DIR) / path(g_valid_yaml_file);
// //   auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

// //   ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);
// //   verifyMapMsg(resp->map);
// // }

// // // Send map loading service request without specifying which map to load
// // TEST_F(MapServerTestFixture, LoadMapNull)
// // {
// //   RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
// //   auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
// //   auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
// //     "/map_server/load_map");

// //   RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
// //   ASSERT_TRUE(client->wait_for_service());

// //   req->map_url = "";
// //   RCLCPP_INFO(node_->get_logger(), "Sending load_map request with null file name");
// //   auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

// //   ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST);
// // }

// // // Send map loading service request with non-existing yaml file
// // TEST_F(MapServerTestFixture, LoadMapInvalidYaml)
// // {
// //   RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
// //   auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
// //   auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
// //     "/map_server/load_map");

// //   RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
// //   ASSERT_TRUE(client->wait_for_service());

// //   req->map_url = "invalid_file.yaml";
// //   RCLCPP_INFO(node_->get_logger(), "Sending load_map request with invalid yaml file name");
// //   auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

// //   ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA);
// // }

// // // Send map loading service request with yaml file containing non-existing map
// // TEST_F(MapServerTestFixture, LoadMapInvalidImage)
// // {
// //   RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
// //   auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
// //   auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
// //     "/map_server/load_map");

// //   RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
// //   ASSERT_TRUE(client->wait_for_service());

// //   req->map_url = path(TEST_DIR) / "invalid_image.yaml";
// //   RCLCPP_INFO(node_->get_logger(), "Sending load_map request with invalid image file name");
// //   auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

// //   ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA);
// // }

// // /**
// //  * Test behaviour of server if yaml_filename is set to an empty string.
// //  */
// // TEST_F(MapServerTestFixture, NoInitialMap)
// // {
// //   // turn off node into unconfigured state
// //   lifecycle_client_->change_state(Transition::TRANSITION_DEACTIVATE);
// //   lifecycle_client_->change_state(Transition::TRANSITION_CLEANUP);

// //   auto client = node_->create_client<nav_msgs::srv::GetMap>("/map_server/map");
// //   auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();

// //   auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "map_server");
// //   ASSERT_TRUE(parameters_client->wait_for_service(3s));

// //   // set yaml_filename-parameter to empty string (essentially restart the node)
// //   RCLCPP_INFO(node_->get_logger(), "Removing yaml_filename-parameter before restarting");
// //   parameters_client->set_parameters({Parameter("yaml_filename", ParameterValue(""))});

// //   // only configure node, to test behaviour of service while node is not active
// //   lifecycle_client_->change_state(Transition::TRANSITION_CONFIGURE, 3s);

// //   RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service while not being active");
// //   auto load_map_req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
// //   auto load_map_cl = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

// //   ASSERT_TRUE(load_map_cl->wait_for_service(3s));
// //   auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, load_map_cl, load_map_req);

// //   ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE);

// //   // activate server and load map:
// //   lifecycle_client_->change_state(Transition::TRANSITION_ACTIVATE, 3s);
// //   RCLCPP_INFO(node_->get_logger(), "active again");

// //   load_map_req->map_url = path(TEST_DIR) / path(g_valid_yaml_file);
// //   auto load_res = send_request<nav2_msgs::srv::LoadMap>(node_, load_map_cl, load_map_req);

// //   ASSERT_EQ(load_res->result, nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);
// //   verifyMapMsg(load_res->map);
// // }
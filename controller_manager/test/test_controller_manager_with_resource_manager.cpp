// Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "test_controller_manager_with_resource_manager.hpp"

std::shared_ptr<rclcpp::Node> ControllerManagerTest::node_ = nullptr;
std::unique_ptr<hardware_interface::ResourceManager> ControllerManagerTest::test_resource_manager_ =
  nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ControllerManagerTest::executor_ =
  nullptr;

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

void ControllerManagerTest::SetUp()
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("controller_manager_test_node");
  auto clock = node_->get_clock();
  auto logger = node_->get_logger();

  test_resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(clock, logger);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

void ControllerManagerTest::TearDown()
{
  node_.reset();
  test_resource_manager_.reset();
  executor_.reset();
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_urdf_without_hardware_plugin)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = ros2_control_test_assets::invalid_urdf_without_hardware_plugin;

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_invalid_urdf)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = R"(<robot malformed></robot>)";

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_empty_urdf)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = "";

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_wrong_plugins)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = ros2_control_test_assets::invalid_urdf_with_wrong_plugin;

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, robot_description_callback_handles_no_geometry)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  std_msgs::msg::String invalid_urdf_msg;
  invalid_urdf_msg.data = ros2_control_test_assets::invalid_urdf_no_geometry;

  cm.robot_description_callback(invalid_urdf_msg);

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, init_controller_manager_with_invalid_urdf)
{
  const std::string invalid_urdf = ros2_control_test_assets::invalid_urdf_with_wrong_plugin;

  TestControllerManager cm(
    executor_, invalid_urdf, false, "test_controller_manager", "", rclcpp::NodeOptions{});

  EXPECT_FALSE(cm.is_resource_manager_initialized());

  EXPECT_TRUE(cm.has_valid_robot_description());
}

TEST_F(ControllerManagerTest, check_list_controller_with_missing_interface_does_not_throw)
{
  TestControllerManager cm(std::move(test_resource_manager_), executor_);

  auto dummy = std::make_shared<DummyController>();

  controller_manager::ControllerSpec spec;
  spec.c = dummy;
  spec.info.name = "dummy_controller";
  spec.info.type = "DummyController";

  cm.add_controller(spec);
  auto ret = cm.configure_controller("dummy_controller");  // transitions to INACTIVE
  EXPECT_EQ(ret, controller_interface::return_type::OK);

  // Prepare request/response
  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto res = std::make_shared<controller_manager_msgs::srv::ListControllers::Response>();

  // Call the service callback (should NOT throw even though interface is missing)
  EXPECT_NO_THROW(cm.list_controllers_srv_cb_public(req, res));

  ASSERT_EQ(res->controller.size(), 1);
  EXPECT_EQ(res->controller[0].name, "dummy_controller");
}

TEST_F(ControllerManagerTest, list_hardware_components_with_data_type_errors_does_not_throw)
{
  auto mock_rm =
    std::make_unique<MockResourceManagerWithErrors>(node_->get_clock(), rclcpp::get_logger("test"));

  TestControllerManager cm(std::move(mock_rm), executor_);

  auto req = std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>();
  auto res = std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Response>();

  // Should not throw, errors should be caught and logged
  EXPECT_NO_THROW(cm.list_hardware_components_srv_cb_public(req, res));

  const auto & component = res->component[0];
  EXPECT_EQ(component.name, "dummy_component");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}

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

#ifndef TEST_CONTROLLER_MANAGER_WITH_RESOURCE_MANAGER_HPP_
#define TEST_CONTROLLER_MANAGER_WITH_RESOURCE_MANAGER_HPP_

#include <memory>
#include <string>

#include <utility>
#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "gtest/gtest.h"
#include "hardware_interface/hardware_component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "std_msgs/msg/string.hpp"

class TestControllerManager : public controller_manager::ControllerManager
{
public:
  using ControllerManager::ControllerManager;

  // Expose callbacks
  using ControllerManager::robot_description_callback;

  using ControllerManager::is_resource_manager_initialized;

  using ControllerManager::resource_manager_;

  using ControllerManager::has_valid_robot_description;
  void list_controllers_srv_cb_public(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> req,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> res)
  {
    list_controllers_srv_cb(req, res);
  }
  void list_hardware_components_srv_cb_public(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request> req,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> res)
  {
    list_hardware_components_srv_cb(req, res);
  }
};

class ControllerManagerTest : public ::testing::Test
{
protected:
  static std::shared_ptr<rclcpp::Node> node_;
  static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  static std::unique_ptr<hardware_interface::ResourceManager> test_resource_manager_;
  virtual void SetUp();
  virtual void TearDown();
};

class DummyController : public controller_interface::ControllerInterface
{
public:
  DummyController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back("nonexistent_joint/position");  // intentionally missing
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back("nonexistent_joint/velocity");  // intentionally missing
    return config;
  }

  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }
};

class MockResourceManagerWithErrors : public hardware_interface::ResourceManager
{
public:
  MockResourceManagerWithErrors(rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
  : hardware_interface::ResourceManager(clock, logger)
  {
  }

  // Return a fake component with "nonexistent" interfaces
  const std::unordered_map<std::string, hardware_interface::HardwareComponentInfo> &
  get_components_status() const
  {
    _mock_components.clear();

    hardware_interface::HardwareComponentInfo comp;
    comp.state_interfaces.push_back("nonexistent_joint/velocity");
    comp.command_interfaces.push_back("nonexistent_joint/position");
    RCLCPP_ERROR(get_logger(), "here");

    _mock_components["dummy_component"] = comp;
    return _mock_components;
  }

  // Override availability to always return false
  bool state_interface_is_available(const std::string &) const { return false; }

  bool command_interface_is_available(const std::string &) const { return false; }

  // Override data type accessors to throw errors
  std::string get_state_interface_data_type(const std::string &) const
  {
    throw std::runtime_error("State interface error");
  }

  std::string get_command_interface_data_type(const std::string &) const
  {
    throw std::runtime_error("Command interface error");
  }

private:
  mutable std::unordered_map<std::string, hardware_interface::HardwareComponentInfo>
    _mock_components;
};

#endif  // TEST_CONTROLLER_MANAGER_WITH_RESOURCE_MANAGER_HPP_

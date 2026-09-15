#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"

namespace controller_manager
{
class ControllerManager; // forward declare

// TODO: move ControllerManagerStateMachine into this file, merge them!
class LifecycleNodeFacade
{
public:
  explicit LifecycleNodeFacade(ControllerManager * cm);

  void init_services(rclcpp::CallbackGroup::SharedPtr callback_group);
  void reset_services();

private:
  ControllerManager * cm_;

  rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr get_state_srv_;
  rclcpp::Service<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_srv_;
  rclcpp::Service<lifecycle_msgs::srv::GetAvailableStates>::SharedPtr get_available_states_srv_;
  rclcpp::Service<lifecycle_msgs::srv::GetAvailableTransitions>::SharedPtr get_available_transitions_srv_;

  void get_state_cb(const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> req, std::shared_ptr<lifecycle_msgs::srv::GetState::Response> res);
  void change_state_cb(const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> req, std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> res);
  void get_available_states_cb(const std::shared_ptr<lifecycle_msgs::srv::GetAvailableStates::Request> req, std::shared_ptr<lifecycle_msgs::srv::GetAvailableStates::Response> res);
  void get_available_transitions_cb(const std::shared_ptr<lifecycle_msgs::srv::GetAvailableTransitions::Request> req, std::shared_ptr<lifecycle_msgs::srv::GetAvailableTransitions::Response> res);

  // helpers to create state objects
  static lifecycle_msgs::msg::State create_state(uint8_t id, const std::string & label);
  static lifecycle_msgs::msg::TransitionDescription create_transition(uint8_t trans_id, const std::string & trans_label, uint8_t start_id, const std::string & start_label, uint8_t goal_id, const std::string & goal_label);
};

}  // namespace controller_manager
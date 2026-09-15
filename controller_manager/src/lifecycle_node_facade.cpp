#include "controller_manager/lifecycle_node_facade.hpp"
#include "controller_manager/controller_manager.hpp"

namespace controller_manager
{
using namespace std::placeholders;

LifecycleNodeFacade::LifecycleNodeFacade(ControllerManager * cm) : cm_(cm) {}

void LifecycleNodeFacade::init_services(rclcpp::CallbackGroup::SharedPtr callback_group)
{
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1)).reliable().durability_volatile();

  if (!get_state_srv_) {
    get_state_srv_ = cm_->create_service<lifecycle_msgs::srv::GetState>("~/get_state", std::bind(&LifecycleNodeFacade::get_state_cb, this, _1, _2), qos, callback_group);
  }
  if (!change_state_srv_) {
    change_state_srv_ = cm_->create_service<lifecycle_msgs::srv::ChangeState>("~/change_state", std::bind(&LifecycleNodeFacade::change_state_cb, this, _1, _2), qos, callback_group);
  }
  if (!get_available_states_srv_) {
    get_available_states_srv_ = cm_->create_service<lifecycle_msgs::srv::GetAvailableStates>("~/get_available_states", std::bind(&LifecycleNodeFacade::get_available_states_cb, this, _1, _2), qos, callback_group);
  }
  if (!get_available_transitions_srv_) {
    get_available_transitions_srv_ = cm_->create_service<lifecycle_msgs::srv::GetAvailableTransitions>("~/get_available_transitions", std::bind(&LifecycleNodeFacade::get_available_transitions_cb, this, _1, _2), qos, callback_group);
  }
}

void LifecycleNodeFacade::reset_services()
{
  get_state_srv_.reset();
  change_state_srv_.reset();
  get_available_states_srv_.reset();
  get_available_transitions_srv_.reset();
}

void LifecycleNodeFacade::get_state_cb(
  const std::shared_ptr<lifecycle_msgs::srv::GetState::Request> /*req*/,
  std::shared_ptr<lifecycle_msgs::srv::GetState::Response> res)
{
  rclcpp_lifecycle::State rclcpp_lifecycle_state = cm_->state_machine_->get_state();

  res->current_state.id = rclcpp_lifecycle_state.id();
  res->current_state.label = rclcpp_lifecycle_state.label();
}

void LifecycleNodeFacade::change_state_cb(
  const std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> req,
  std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> res)
{
  uint8_t target_state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;

  switch (req->transition.id)
  {
    case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
      target_state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
      target_state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
      RCLCPP_WARN(cm_->get_logger(), "Transition to 'active' depends on the controllers. Manual lifecycle transition not possible.");
      res->success = false;
      return;
    case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
      target_state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
    case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
      target_state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
      break;
    default:
      RCLCPP_WARN(cm_->get_logger(), "Unknown transition requested.");
      res->success = false;
      return;
  }

  uint8_t initial_state = cm_->state_machine_->get_state_id();
  
  cm_->lifecycle_transition_to(target_state_id);

  res->success = (cm_->state_machine_->get_state_id() != initial_state);
}

lifecycle_msgs::msg::State LifecycleNodeFacade::create_state(uint8_t id, const std::string & label)
{
  lifecycle_msgs::msg::State state;
  state.id = id;
  state.label = label;
  return state;
}

lifecycle_msgs::msg::TransitionDescription LifecycleNodeFacade::create_transition(
  uint8_t trans_id, const std::string & trans_label, 
  uint8_t start_id, const std::string & start_label, 
  uint8_t goal_id, const std::string & goal_label)
{
  lifecycle_msgs::msg::TransitionDescription desc;
  desc.transition.id = trans_id;
  desc.transition.label = trans_label;
  desc.start_state = create_state(start_id, start_label);
  desc.goal_state = create_state(goal_id, goal_label);
  return desc;
}

void LifecycleNodeFacade::get_available_states_cb(
  const std::shared_ptr<lifecycle_msgs::srv::GetAvailableStates::Request> /*req*/,
  std::shared_ptr<lifecycle_msgs::srv::GetAvailableStates::Response> res)
{
  res->available_states.push_back(create_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"));
  res->available_states.push_back(create_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
  res->available_states.push_back(create_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
  res->available_states.push_back(create_state(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized"));
}

void LifecycleNodeFacade::get_available_transitions_cb(
  const std::shared_ptr<lifecycle_msgs::srv::GetAvailableTransitions::Request> /*req*/,
  std::shared_ptr<lifecycle_msgs::srv::GetAvailableTransitions::Response> res)
{
  uint8_t current_state = cm_->state_machine_->get_state_id();

  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    res->available_transitions.push_back(create_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure", current_state, "unconfigured", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
  } 
  else if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    res->available_transitions.push_back(create_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate", current_state, "inactive", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
    res->available_transitions.push_back(create_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "cleanup", current_state, "inactive", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"));
  }
  else if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    res->available_transitions.push_back(create_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "deactivate", current_state, "active", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
  }
  
  if (current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
    res->available_transitions.push_back(create_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, "shutdown", current_state, "current", lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized"));
  }
}

} // namespace controller_manager
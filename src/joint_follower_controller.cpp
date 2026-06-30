// Copyright (c) 2026 Franka Robotics GmbH
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

#include <franka_follower_controllers/joint_follower_controller.hpp>

#include <Eigen/Eigen>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace franka_follower_controllers
{

controller_interface::InterfaceConfiguration
JointFollowerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(namespace_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointFollowerController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(namespace_prefix_ + robot_type_ + "_joint" + std::to_string(i) +
                           "/position");
    config.names.push_back(namespace_prefix_ + robot_type_ + "_joint" + std::to_string(i) +
                           "/velocity");
  }
  return config;
}

controller_interface::return_type JointFollowerController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  updateJointStates_();
  Vector7d q_goal;
  Vector7d tau_d_calculated;

  if (sync_after_activation_) {
    if (!motion_generator_initialized_) {
      // After starting the controller, we wait for a valid joint state from the target topic
      // Until we get valid joint states, we will send zero torques to the robot
      //  to allow the user to reposition the robot
      motion_generator_initialized_ = initializeMotionGenerator_();

      if (!motion_generator_initialized_) {
        for (int i = 0; i < num_joints; ++i) {
          command_interfaces_[i].set_value(0.0);
        }

        return controller_interface::return_type::OK;
      }
    }

    if (!move_to_start_position_finished_) {
      // We have received valid joint states and initialized the motion generator
      // Now we move smoothly to the first joint position received from the target topic
      auto trajectory_time = this->get_node()->now() - start_time_;
      auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
      move_to_start_position_finished_ = motion_generator_output.second;

      q_goal = motion_generator_output.first;
    }
  }

  if (move_to_start_position_finished_ || !sync_after_activation_) {
    // After reaching the start position, we follow the joint position from the target topic
    // This is the normal operation mode of the controller
    publishSyncState_("FOLLOWING");
    if (!target_joint_state_valid_) {
      RCLCPP_FATAL(get_node()->get_logger(), "Timeout: No valid target joint states received!");
      rclcpp::shutdown();  // Exit the node permanently
    }
    for (int i = 0; i < num_joints; ++i) {
      q_goal(i) = target_joint_state_[i];
    }
  }

  tau_d_calculated = calculateTauDGains_(q_goal);

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  return controller_interface::return_type::OK;
}

void JointFollowerController::jointStateCallback_(const sensor_msgs::msg::JointState msg)
{
  if (last_target_joint_state_time_.seconds() == 0.0) {
    return;
  }

  if (msg.position.size() < target_joint_state_.size()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Received joint state size is smaller than expected size.");
    return;
  }

  std::copy(msg.position.begin(), msg.position.begin() + target_joint_state_.size(),
            target_joint_state_.begin());

  validateTargetJointState_(msg);
  last_target_joint_state_time_ = msg.header.stamp;
}

CallbackReturn JointFollowerController::on_init()
{
  try {
    auto_declare<std::string>("robot_type", "");
    auto_declare<std::string>("target_joint_states_topic_name", "");
    auto_declare<bool>("sync_after_activation", false);
    auto_declare<double>("k_alpha", 0.99);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointFollowerController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  namespace_prefix_ = get_node()->get_namespace();
  if (namespace_prefix_ == "/" || namespace_prefix_.empty()) {
    namespace_prefix_.clear();
  } else {
    // Remove leading slash and add trailing underscore
    namespace_prefix_ = namespace_prefix_.substr(1) + "_";
  }

  sync_after_activation_ = get_node()->get_parameter("sync_after_activation").as_bool();

  target_joint_states_topic_name_ =
    get_node()->get_parameter("target_joint_states_topic_name").as_string();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto k_alpha = get_node()->get_parameter("k_alpha").as_double();


  RCLCPP_INFO(get_node()->get_logger(),
              "Controller parameters: robot_type: %s, target_joint_states_topic_name: %s, sync_after_activation: "
              "%s, k_alpha: %f",
              robot_type_.c_str(), target_joint_states_topic_name_.c_str(),
              sync_after_activation_ ? "true" : "false", k_alpha);

  if (!validateGains_(k_gains, "k_gains") || !validateGains_(d_gains, "d_gains")) {
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }

  if (k_alpha < 0.0 || k_alpha > 1.0) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "k_alpha should be in the range [0, 1]");
    return CallbackReturn::FAILURE;
  }

  k_alpha_ = k_alpha;

  dq_filtered_.setZero();

  auto parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  target_joint_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
          target_joint_states_topic_name_, 1,
    [this](const sensor_msgs::msg::JointState & msg) {jointStateCallback_(msg);});

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  state_publisher_ = get_node()->create_publisher<std_msgs::msg::String>("~/state", qos);

  // Publish initial state without using the publishSyncState_ function due to its dedup check
  auto msg = std_msgs::msg::String();
  msg.data = current_state_;
  state_publisher_->publish(msg);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointFollowerController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_target_joint_state_time_ = get_node()->now();
  dq_filtered_.setZero();
  move_to_start_position_finished_ = false;
  motion_generator_initialized_ = false;

  publishSyncState_(sync_after_activation_ ? "SYNCING" : "FOLLOWING");

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointFollowerController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  publishSyncState_("INACTIVE");

  return CallbackReturn::SUCCESS;
}

auto JointFollowerController::calculateTauDGains_(const Vector7d & q_goal) -> Vector7d
{
  dq_filtered_ = (1 - k_alpha_) * dq_filtered_ + k_alpha_ * dq_;
  Vector7d tau_d_calculated;
  tau_d_calculated = k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);

  return tau_d_calculated;
}

bool JointFollowerController::validateGains_(
  const std::vector<double> & gains,
  const std::string & gains_name)
{
  if (gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s parameter not set", gains_name.c_str());
    return false;
  }

  if (gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "%s should be of size %d but is of size %ld",
                 gains_name.c_str(), num_joints, gains.size());
    return false;
  }

  return true;
}

void JointFollowerController::validateTargetJointState_(const sensor_msgs::msg::JointState & msg)
{
  const double max_time_diff = 0.5;
  auto current_time = get_node()->now();
  auto time_since_last_joint_state = (current_time - last_target_joint_state_time_).seconds();
  auto time_since_msg_stamp = (current_time - msg.header.stamp).seconds();
  target_joint_state_valid_ =
    (time_since_last_joint_state < max_time_diff && time_since_msg_stamp < max_time_diff);
  if (!target_joint_state_valid_) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Target joint state is not valid. Time since last joint state: %f // Time "
                "since message stamp: %f",
                time_since_last_joint_state, time_since_msg_stamp);
  }
}

void JointFollowerController::updateJointStates_()
{
  for (auto i = 0; i < num_joints; ++i) {
    const auto & position_interface = state_interfaces_.at(2 * i);
    const auto & velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

bool JointFollowerController::initializeMotionGenerator_()
{
  if (!target_joint_state_valid_) {
    // Only send a warning once every 10 seconds in order not to spam the log
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 10 * 1000,
                         "Waiting for valid joint states...");
    return false;
  }

  Vector7d q_goal;
  updateJointStates_();
  for (int i = 0; i < num_joints; ++i) {
    q_goal(i) = target_joint_state_[i];
  }
  RCLCPP_INFO(get_node()->get_logger(), "q_goal of motion generator: [%f, %f, %f, %f, %f, %f, %f]",
              q_goal(0), q_goal(1), q_goal(2), q_goal(3), q_goal(4), q_goal(5), q_goal(6));

  const double motion_generator_speed_factor = 0.2;
  motion_generator_ = std::make_unique<MotionGenerator>(motion_generator_speed_factor, q_, q_goal);
  start_time_ = this->get_node()->now();
  return true;
}

void JointFollowerController::publishSyncState_(const std::string & state)
{
  if (state == current_state_) {
    return;
  }
  current_state_ = state;
  auto msg = std_msgs::msg::String();
  msg.data = state;
  state_publisher_->publish(msg);
  RCLCPP_INFO(get_node()->get_logger(), "Sync state: %s", state.c_str());
}

}  // namespace franka_follower_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_follower_controllers::JointFollowerController,
                       controller_interface::ControllerInterface)

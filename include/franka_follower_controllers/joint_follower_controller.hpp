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

#pragma once

#include "franka_follower_controllers/motion_generator.hpp"
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_follower_controllers
{

/**
 * Controller to move the robot to a desired joint position.
 */
class JointFollowerController
  : public controller_interface::ControllerInterface {
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type
  update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  CallbackReturn on_init() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  std::string arm_id_;
  std::string namespace_prefix_;
  std::string robot_description_;
  std::string target_joint_states_topic_name_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d tau_filtered_;   // LPF state for the commanded-torque low-pass (DROID-style)
  Vector7d k_gains_;          // joint null-space stiffness Kq (DROID default_Kq)
  Vector7d d_gains_;          // joint null-space damping  Kqd (DROID default_Kqd)
  Vector6d cartesian_stiffness_;  // task-space Kx (DROID default_Kx), mapped via Jᵀ Kx J
  Vector6d cartesian_damping_;    // task-space Kxd (DROID default_Kxd), mapped via Jᵀ Kxd J
  double k_alpha_;
  // Commanded-torque low-pass cutoff [Hz]. DROID/polymetis ran libfranka's 100 Hz
  // torque LPF (robot->control(..., cutoff)); franka_hardware's startTorqueControl()
  // applies NO torque LPF (only a 1000 Nm/s rate clamp), so our raw PD+staircase
  // torque reaches the joints with high-freq chatter -> grinding. We replicate the
  // 100 Hz LPF here. Set >= controller rate (e.g. 1000) to disable.
  double torque_lpf_cutoff_hz_{100.0};
  bool sync_after_activation_{false};
  bool move_to_start_position_finished_{false};
  bool motion_generator_initialized_{false};
  rclcpp::Time start_time_;
  std::unique_ptr<MotionGenerator> motion_generator_;
  // Coriolis feedforward: libfranka auto-compensates GRAVITY in torque mode but NOT
  // Coriolis/centrifugal terms. Without this, those torques are an uncompensated
  // disturbance the PD must fight during motion (rough/sluggish tracking). DROID's
  // HybridJointImpedanceControl added the same term, so this is dataset-faithful.
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    target_joint_state_subscriber_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  std::string current_state_{"INACTIVE"}; // "INACTIVE", "SYNCING", "FOLLOWING"
  bool target_joint_state_valid_ = false;
  std::array<double, 7> target_joint_state_{0, 0, 0, 0, 0, 0, 0};
  rclcpp::Time last_target_joint_state_time_;

  Vector7d calculateTauDGains_(const Vector7d & q_goal);
  bool validateGains_(
    const std::vector<double> & gains,
    const std::string & gains_name);
  bool initializeMotionGenerator_();
  void updateJointStates_();
  void validateTargetJointState_(const sensor_msgs::msg::JointState & msg);
  void jointStateCallback_(const sensor_msgs::msg::JointState msg);
  void publishSyncState_(const std::string & state);
};

} // namespace franka_follower_controllers

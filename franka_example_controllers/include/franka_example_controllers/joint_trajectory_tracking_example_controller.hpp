// Copyright (c) 2023 Franka Robotics GmbH
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

#include <array>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * @brief Structure to store one trajectory point.
 *
 * For joint trajectory, the YAML file key is "pose" which must contain exactly 7 joint values.
 */
struct TrajectoryPoint {
  double time_sec;
  std::array<double, 7> joints;
};

/**
 * @brief The joint trajectory tracking example controller.
 *
 * This controller loads a trajectory from a YAML file (with key "trajectory") where each point uses the key "pose"
 * to specify 7 joint values. It then interpolates (using cubic smoothstep) the desired joint positions based on elapsed time.
 */
class JointTrajectoryTrackingExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time & time,
                                           const rclcpp::Duration & period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

 private:
  // Load trajectory from YAML file.
  bool loadTrajectoryFromFile(const std::string & file_path);
  // Compute desired joint positions at time t using cubic smoothstep interpolation.
  void getDesiredJointPositions(double t, Eigen::VectorXd & desired_joint_positions);

  std::string arm_id_;
  bool is_gazebo_{false};
  std::string robot_description_;
  const int num_joints = 7;
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
  double elapsed_time_ = 0.0;
  double initial_robot_time_ = 0.0;
  double robot_time_ = 0.0;
  double trajectory_period_ = 0.001;
  bool initialization_flag_{true};

  // Trajectory file parameter.
  std::string trajectory_file_;
  // Vector of trajectory points.
  std::vector<TrajectoryPoint> trajectory_;
};

}  // namespace franka_example_controllers

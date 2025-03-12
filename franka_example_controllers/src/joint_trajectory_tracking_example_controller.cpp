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

#include "franka_example_controllers/joint_trajectory_tracking_example_controller.hpp"
#include "franka_example_controllers/robot_utils.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointTrajectoryTrackingExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Use command interface names: arm_id_joint{1..num_joints}/position.
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointTrajectoryTrackingExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Use state interface names: arm_id_joint{1..num_joints}/position.
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  // Add robot time interface if not running in Gazebo.
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/robot_time");
  }
  return config;
}

controller_interface::return_type
JointTrajectoryTrackingExampleController::update(const rclcpp::Time & /*time*/,
                                                   const rclcpp::Duration & /*period*/) {
  if (initialization_flag_) {
    // Store initial joint positions.
    for (int i = 0; i < num_joints; ++i) {
      initial_q_[i] = state_interfaces_[i].get_value();
    }
    initialization_flag_ = false;
    if (!is_gazebo_) {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
    elapsed_time_ = 0.0;
  } else {
    if (!is_gazebo_) {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    } else {
      elapsed_time_ += trajectory_period_;
    }
  }

  Eigen::VectorXd desired_joint_positions(num_joints);
  getDesiredJointPositions(elapsed_time_, desired_joint_positions);

  // Send desired joint commands to each joint.
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(desired_joint_positions[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("trajectory_file", "");
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
  trajectory_file_ = get_node()->get_parameter("trajectory_file").as_string();
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  std::string robot_description;
  if (!result.empty()) {
    robot_description = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }
  // Optionally, arm_id_ can be extracted using robot_utils if desired:
  // arm_id_ = robot_utils::getRobotNameFromDescription(robot_description, get_node()->get_logger());

  // Load the trajectory from the YAML file.
  if (!loadTrajectoryFromFile(trajectory_file_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load trajectory from file: %s", trajectory_file_.c_str());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

bool JointTrajectoryTrackingExampleController::loadTrajectoryFromFile(const std::string & file_path) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["trajectory"]) {
      RCLCPP_ERROR(get_node()->get_logger(), "No 'trajectory' key found in YAML file.");
      return false;
    }
    auto traj = config["trajectory"];
    for (size_t i = 0; i < traj.size(); ++i) {
      TrajectoryPoint point;
      point.time_sec = traj[i]["time_sec"].as<double>();
      // Note: The YAML file uses the key "pose" (not "joints") to specify the joint values.
      std::vector<double> pose = traj[i]["pose"].as<std::vector<double>>();
      if (pose.size() != static_cast<size_t>(num_joints)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Each trajectory point must have %d joint values.", num_joints);
        return false;
      }
      for (int j = 0; j < num_joints; ++j) {
        point.joints[j] = pose[j];
      }
      trajectory_.push_back(point);
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "YAML Exception: %s", e.what());
    return false;
  }
  return true;
}

void JointTrajectoryTrackingExampleController::getDesiredJointPositions(double t,
                                                                        Eigen::VectorXd & desired_joint_positions) {
  desired_joint_positions = Eigen::VectorXd(num_joints);
  if (trajectory_.empty()) {
    // If no trajectory is loaded, use the initial joint positions.
    for (int i = 0; i < num_joints; ++i) {
      desired_joint_positions[i] = initial_q_[i];
    }
    return;
  }
  if (t <= trajectory_.front().time_sec) {
    for (int i = 0; i < num_joints; ++i) {
      desired_joint_positions[i] = trajectory_.front().joints[i];
    }
    return;
  }
  if (t >= trajectory_.back().time_sec) {
    for (int i = 0; i < num_joints; ++i) {
      desired_joint_positions[i] = trajectory_.back().joints[i];
    }
    return;
  }
  // Find the interval [t_i, t_{i+1}] that contains t.
  for (size_t i = 0; i < trajectory_.size() - 1; ++i) {
    double t_i = trajectory_[i].time_sec;
    double t_next = trajectory_[i+1].time_sec;
    if (t >= t_i && t < t_next) {
      double dt = t_next - t_i;
      double s = (t - t_i) / dt;
      // Cubic smoothstep interpolation: f(s) = 3*s^2 - 2*s^3.
      double smooth_factor = 3 * s * s - 2 * s * s * s;
      for (int j = 0; j < num_joints; ++j) {
        double pos_i = trajectory_[i].joints[j];
        double pos_next = trajectory_[i+1].joints[j];
        desired_joint_positions[j] = pos_i + smooth_factor * (pos_next - pos_i);
      }
      return;
    }
  }
  // Fallback.
  for (int i = 0; i < num_joints; ++i) {
    desired_joint_positions[i] = trajectory_.back().joints[i];
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointTrajectoryTrackingExampleController,
                       controller_interface::ControllerInterface)

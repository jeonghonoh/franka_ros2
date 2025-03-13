// joint_trajectory_tracking_example_controller.cpp
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

#include <franka_example_controllers/joint_trajectory_tracking_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>
#include <chrono>
#include <yaml-cpp/yaml.h>

namespace franka_example_controllers {

JointTrajectoryTrackingExampleController::JointTrajectoryTrackingExampleController()
  : arm_id_(""), trajectory_file_(""), initialization_flag_(false)
{
  q_.setZero();
  dq_.setZero();
  dq_filtered_.setZero();
  k_gains_.setZero();
  d_gains_.setZero();
}

controller_interface::InterfaceConfiguration JointTrajectoryTrackingExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Command interface for each joint: "<arm_id>_jointX/effort"
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration JointTrajectoryTrackingExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Each joint provides a position and a velocity interface.
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointTrajectoryTrackingExampleController::update(const rclcpp::Time & /*time*/,
                                                                                   const rclcpp::Duration & /*period*/) {
  updateJointStates();
  rclcpp::Duration elapsed = this->get_node()->now() - start_time_;
  double t = elapsed.seconds();
  auto desired = getDesiredJointPositions(t);
  Vector7d q_desired = desired.first;
  bool finished = desired.second;

  if (!finished) {
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d = k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(tau_d(i));
    }
  } else {
    for (auto &interface : command_interfaces_) {
      interface.set_value(0);
    }
    this->get_node()->set_parameter({"process_finished", true});
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<std::string>("trajectory_file", "");
    auto_declare<bool>("process_finished", false);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  trajectory_file_ = get_node()->get_parameter("trajectory_file").as_string();
  RCLCPP_INFO(get_node()->get_logger(), "Configuring with arm_id: '%s', trajectory_file: '%s'",
              arm_id_.c_str(), trajectory_file_.c_str());

  // Set default collision behavior.
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>("service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  auto future_result = client->async_send_request(request);
  future_result.wait_for(std::chrono::seconds(2));
  if (!future_result.get()) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  // Load joint trajectory from YAML file.
  if (!loadTrajectoryFromFile(trajectory_file_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to load trajectory from file: %s", trajectory_file_.c_str());
    return CallbackReturn::FAILURE;
  }

  // Read PD gains.
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty() || k_gains.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter must be set with %d elements", num_joints);
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty() || d_gains.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter must be set with %d elements", num_joints);
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_gains_(i) = d_gains.at(i);
  }
  dq_filtered_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  updateJointStates();
  start_time_ = this->get_node()->now();
  get_node()->set_parameter({"process_finished", false});
  initialization_flag_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointTrajectoryTrackingExampleController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto &interface : command_interfaces_) {
    interface.set_value(0);
  }
  return CallbackReturn::SUCCESS;
}

bool JointTrajectoryTrackingExampleController::loadTrajectoryFromFile(const std::string &file_path) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["trajectory"]) {
      RCLCPP_ERROR(get_node()->get_logger(), "No 'trajectory' key found in YAML file.");
      return false;
    }
    YAML::Node traj = config["trajectory"];
    for (size_t i = 0; i < traj.size(); ++i) {
      JointTrajectoryPoint point;
      point.time_sec = traj[i]["time_sec"].as<double>();
      std::vector<double> q = traj[i]["pose"].as<std::vector<double>>();
      if (q.size() != static_cast<size_t>(num_joints)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Each trajectory point must have %d elements.", num_joints);
        return false;
      }
      point.q = Eigen::Map<Vector7d>(q.data(), num_joints);
      trajectory_.push_back(point);
    }
  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "YAML Exception: %s", e.what());
    return false;
  }
  return true;
}

std::pair<Vector7d, bool> JointTrajectoryTrackingExampleController::getDesiredJointPositions(double t) const {
  if (trajectory_.empty()) {
    return std::make_pair(q_, true);
  }
  if (t <= trajectory_.front().time_sec) {
    return std::make_pair(trajectory_.front().q, false);
  }
  if (t >= trajectory_.back().time_sec) {
    return std::make_pair(trajectory_.back().q, true);
  }
  for (size_t i = 0; i < trajectory_.size() - 1; ++i) {
    double t_i = trajectory_[i].time_sec;
    double t_next = trajectory_[i+1].time_sec;
    if (t >= t_i && t < t_next) {
      double ratio = (t - t_i) / (t_next - t_i);
      Vector7d q_desired = trajectory_[i].q + ratio * (trajectory_[i+1].q - trajectory_[i].q);
      return std::make_pair(q_desired, false);
    }
  }
  return std::make_pair(trajectory_.back().q, true);
}

void JointTrajectoryTrackingExampleController::updateJointStates() {
  for (int i = 0; i < num_joints; ++i) {
    const auto &pos_if = state_interfaces_.at(2 * i);
    const auto &vel_if = state_interfaces_.at(2 * i + 1);
    assert(pos_if.get_interface_name() == "position");
    assert(vel_if.get_interface_name() == "velocity");
    q_(i) = pos_if.get_value();
    dq_(i) = vel_if.get_value();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointTrajectoryTrackingExampleController,
                       controller_interface::ControllerInterface)

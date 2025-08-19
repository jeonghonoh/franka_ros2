// Copyright (c) 2023 Franka Robotics GmbH
// Modified by Google Gemini
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

#include <franka_example_controllers/simple_joint_position_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cassert>
#include <cmath>
#include <exception>

namespace franka_example_controllers {

SimpleJointPositionController::SimpleJointPositionController() : arm_id_("") {
  q_.setZero();
  dq_.setZero();
  dq_filtered_.setZero();
  k_gains_.setZero();
  d_gains_.setZero();
  
  Vector7d initial_pose;
  initial_pose.setZero();
  q_desired_buffer_.initRT(initial_pose);
}

controller_interface::InterfaceConfiguration SimpleJointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration SimpleJointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type SimpleJointPositionController::update(const rclcpp::Time & /*time*/,
                                                                        const rclcpp::Duration & /*period*/) {
  updateJointStates();

  // 실시간 버퍼에서 가장 최근의 목표 위치를 가져옴
  Vector7d q_desired = *q_desired_buffer_.readFromRT();

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  
  // PD 제어 토크 계산
  Vector7d tau_d = k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  
  // 계산된 토크를 각 관절에 전달
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SimpleJointPositionController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3"); // 기본값 설정
    auto_declare<std::string>("joint_command_topic", "joint_position_command");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleJointPositionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  std::string joint_command_topic = get_node()->get_parameter("joint_command_topic").as_string();

  // PD 이득 파라미터 읽기
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.size() != num_joints || d_gains.size() != num_joints) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains and d_gains must have %d elements", num_joints);
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_gains_(i) = d_gains.at(i);
  }
  dq_filtered_.setZero();

  // ROS2 Subscriber 설정
  auto callback = [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void {
    if (msg->data.size() != num_joints) {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Received command with wrong size: %zu, expected %d", msg->data.size(), num_joints);
      return;
    }
    Vector7d q_new_desired = Eigen::Map<const Vector7d>(msg->data.data());
    q_desired_buffer_.writeFromNonRT(q_new_desired); // 버퍼에 새로운 목표 위치 쓰기
  };

  joint_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      joint_command_topic, 1, callback);

  RCLCPP_INFO(get_node()->get_logger(), "PD gains configured and subscriber created on topic '%s'.", joint_command_topic.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleJointPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // 컨트롤러 활성화 시, 현재 위치를 목표 위치로 설정하여 급격한 움직임 방지
  updateJointStates();
  q_desired_buffer_.initRT(q_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleJointPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // 컨트롤러 비활성화 시, 안전을 위해 토크를 0으로 설정
  for (auto &interface : command_interfaces_) {
    interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

void SimpleJointPositionController::updateJointStates() {
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

// 컨트롤러를 플러그인으로 등록
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SimpleJointPositionController,
                       controller_interface::ControllerInterface)
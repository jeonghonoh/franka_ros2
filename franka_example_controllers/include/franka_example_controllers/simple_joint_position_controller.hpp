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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.h>
#include "std_msgs/msg/float64_multi_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

// 7-DOF 벡터 타입
using Vector7d = Eigen::Matrix<double, 7, 1>;

/**
 * @brief SimpleJointPositionController
 * 이 컨트롤러는 ROS2 토픽으로부터 목표 관절 위치(waypoint)를 수신하고,
 * PD 제어를 사용하여 로봇이 해당 위치에 도달하도록 명령합니다.
 */
class SimpleJointPositionController : public controller_interface::ControllerInterface {
 public:
  SimpleJointPositionController();

  // ros2_control 인터페이스 설정 함수들
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // 컨트롤러 핵심 로직 및 생명주기 함수들
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

 private:
  // 파라미터
  std::string arm_id_;
  const int num_joints = 7;

  // PD 제어 관련 벡터
  Vector7d q_;           // 현재 관절 위치
  Vector7d dq_;          // 현재 관절 속도
  Vector7d dq_filtered_; // 필터링된 관절 속도
  Vector7d k_gains_;     // P 이득
  Vector7d d_gains_;     // D 이득

  // 목표 위치를 저장하기 위한 실시간 버퍼
  realtime_tools::RealtimeBuffer<Vector7d> q_desired_buffer_;

  // ROS2 Subscriber
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_subscriber_ = nullptr;
  
  // 현재 로봇 상태를 업데이트하는 도우미 함수
  void updateJointStates();
};

}  // namespace franka_example_controllers
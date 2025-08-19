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

#include <Eigen/Dense>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

// 목표 Pose를 저장하기 위한 구조체
struct PoseCommand {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

/**
 * @brief SimpleCartesianPositionController
 * 이 컨트롤러는 ROS2 토픽으로부터 목표 Cartesian Pose를 수신하고,
 * 로봇이 해당 Pose에 도달하도록 명령합니다.
 */
class SimpleCartesianPositionController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Franka 하드웨어 인터페이스
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface>
      franka_cartesian_pose_;

  // 목표 Pose를 저장하기 위한 실시간 버퍼
  realtime_tools::RealtimeBuffer<PoseCommand> pose_desired_buffer_;

  // ROS2 Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      cartesian_command_subscriber_ = nullptr;

  // 파라미터
  std::string arm_id_;

  // 컨트롤러 초기화 플래그
  bool initialization_flag_{true};
  const bool k_elbow_activated_{false}; // Elbow 제어는 사용하지 않음
};

}  // namespace franka_example_controllers
// Copyright (c) 2023 Franka Robotics GmbH
// Modified by Google Gemini
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law governin g permissions and
// limitations under the License.

#include <franka_example_controllers/simple_cartesian_position_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
SimpleCartesianPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
SimpleCartesianPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  return config;
}

controller_interface::return_type SimpleCartesianPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  
  if (initialization_flag_) {
    // 활성화 후 첫 업데이트 시, 현재 로봇 Pose를 읽어와 목표로 설정 (급격한 움직임 방지)
    auto [current_orientation, current_position] =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    
    PoseCommand initial_pose;
    initial_pose.position = current_position;
    initial_pose.orientation = current_orientation;
    pose_desired_buffer_.initRT(initial_pose);
    
    initialization_flag_ = false;
  }

  // 실시간 버퍼에서 가장 최근의 목표 Pose를 가져옴
  const auto& desired_pose = *pose_desired_buffer_.readFromRT();

  // Franka 인터페이스를 통해 목표 Pose 명령 전달
  if (franka_cartesian_pose_->setCommand(desired_pose.orientation, desired_pose.position)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set Cartesian pose command.");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn SimpleCartesianPositionController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));
  
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::string>("cartesian_command_topic", "cartesian_pose_command");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleCartesianPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto cartesian_command_topic = get_node()->get_parameter("cartesian_command_topic").as_string();


  // // Franka 로봇 기본 충돌 동작 설정 (안전 기능)
  // auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
  //     "service_server/set_full_collision_behavior");
  // auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  // auto future_result = client->async_send_request(request);
  // if (rclcpp::spin_until_future_complete(get_node(), future_result, std::chrono::seconds(2)) !=
  //     rclcpp::FutureReturnCode::SUCCESS) {
  //   RCLCPP_FATAL(get_node()->get_logger(), "Failed to call service to set collision behavior");
  //   return CallbackReturn::ERROR;
  // }

  // ROS2 Subscriber 콜백 함수 설정
  auto callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void {
    PoseCommand new_pose;
    new_pose.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    new_pose.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                              msg->pose.orientation.y, msg->pose.orientation.z);
    pose_desired_buffer_.writeFromNonRT(new_pose);
  };
  
  // Subscriber 생성
  cartesian_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      cartesian_command_topic, 1, callback);

  RCLCPP_INFO(get_node()->get_logger(), "Subscriber created on topic '%s'.", cartesian_command_topic.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleCartesianPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  initialization_flag_ = true; // 첫 update 시 초기화하도록 플래그 설정
  
  // 버퍼 초기화
  PoseCommand initial_pose;
  initial_pose.position.setZero();
  initial_pose.orientation.setIdentity();
  pose_desired_buffer_.initRT(initial_pose);

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian position controller activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleCartesianPositionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SimpleCartesianPositionController,
                       controller_interface::ControllerInterface)
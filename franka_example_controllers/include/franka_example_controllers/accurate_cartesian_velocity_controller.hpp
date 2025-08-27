#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <chrono> // chrono 추가

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

struct PoseCommand {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  bool is_initialized = false;
};

class AccurateCartesianVelocityController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface> franka_cartesian_velocity_;
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_state_;
  realtime_tools::RealtimeBuffer<PoseCommand> pose_desired_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriber_ = nullptr;
  
  // 속도 램프업을 위한 변수
  rclcpp::Time first_target_received_time_;
  const rclcpp::Duration kRampDuration{std::chrono::milliseconds(200)}; // 0.2초 동안 램프업

  double p_gain_{1.0};
  double d_gain_linear_{0.0};    // 선속도 D 게인
  double d_gain_angular_{0.0};   // 각속도 D 게인
  double vel_limit_linear_{0.2}; // 선속도 제한
  double vel_limit_angular_{1.0}; // 각속도 제한
  double vel_filter_tau_{0.02};  // 속도 필터 타우
  rclcpp::Time prev_time_;
  bool has_prev_pose_{false};

  Eigen::Vector3d prev_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond prev_orientation_{1.0, 0.0, 0.0, 0.0};

  Eigen::Vector3d filtered_linear_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_angular_vel_{Eigen::Vector3d::Zero()};
  std::string arm_id_;
};

} // namespace franka_example_controllers
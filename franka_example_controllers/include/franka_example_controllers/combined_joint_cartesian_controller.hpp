#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>

// Velocity 인터페이스를 직접 사용하므로 해당 헤더는 제거
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

enum class ControlMode {
  JOINT_POSITION,
  CARTESIAN_VELOCITY
};

using Vector7d = Eigen::Matrix<double, 7, 1>;
struct JointPositionCommand {
  Vector7d joints;
};
struct CartesianPoseCommand {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  bool is_initialized = false;
};

class CombinedJointCartesianController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void update_joint_states();

  // Velocity Semantic Component 제거
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_state_;

  realtime_tools::RealtimeBuffer<JointPositionCommand> joint_cmd_buffer_;
  realtime_tools::RealtimeBuffer<CartesianPoseCommand> cartesian_cmd_buffer_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_cmd_subscriber_;

  ControlMode current_mode_ = ControlMode::JOINT_POSITION;
  
  std::string arm_id_;
  const int num_joints = 7;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double p_gain_{1.0};

  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;

  rclcpp::Time first_cartesian_target_time_;
  const rclcpp::Duration kRampDuration{std::chrono::milliseconds(200)};
};
} // namespace franka_example_controllers
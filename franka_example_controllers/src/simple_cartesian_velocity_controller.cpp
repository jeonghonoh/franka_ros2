#include "franka_example_controllers/simple_cartesian_velocity_controller.hpp"
#include <Eigen/Dense>
#include "pluginlib/class_list_macros.hpp"
#include <algorithm> 
#include <chrono>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
SimpleCartesianVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
SimpleCartesianVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_state_->get_state_interface_names();
  return config;
}

controller_interface::return_type SimpleCartesianVelocityController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {

  const auto& desired_pose = *pose_desired_buffer_.readFromRT();

  if (!desired_pose.is_initialized) {
    franka_cartesian_velocity_->setCommand(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return controller_interface::return_type::OK;
  }

  auto [current_orientation, current_position] = franka_cartesian_state_->getCurrentOrientationAndTranslation();

  Eigen::Vector3d position_error = desired_pose.position - current_position;
  Eigen::Quaterniond orientation_error = desired_pose.orientation * current_orientation.inverse();
  Eigen::AngleAxisd angle_axis_error(orientation_error);
  Eigen::Vector3d rotation_error = angle_axis_error.axis() * angle_axis_error.angle();

  Eigen::Vector3d target_linear_velocity = p_gain_ * position_error;
  Eigen::Vector3d target_angular_velocity = p_gain_ * rotation_error;

  auto time_since_first_target = time - first_target_received_time_;
  double ramp_factor = time_since_first_target.seconds() / kRampDuration.seconds();
  ramp_factor = std::clamp(ramp_factor, 0.0, 1.0);

  Eigen::Vector3d commanded_linear_velocity = ramp_factor * target_linear_velocity;
  Eigen::Vector3d commanded_angular_velocity = ramp_factor * target_angular_velocity;

  double linear_vel_norm = commanded_linear_velocity.norm();
  if (linear_vel_norm > 0.2) {
    commanded_linear_velocity = 0.2 * commanded_linear_velocity / linear_vel_norm;
  }
  
  if (franka_cartesian_velocity_->setCommand(commanded_linear_velocity, commanded_angular_velocity)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set Cartesian velocity command.");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn SimpleCartesianVelocityController::on_init() {
  franka_cartesian_velocity_ = std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(false);
  franka_cartesian_state_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(false);

  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("target_pose_topic", "cartesian_pose_command");
    auto_declare<double>("p_gain", p_gain_);
  } catch(const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleCartesianVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto target_pose_topic = get_node()->get_parameter("target_pose_topic").as_string();
  p_gain_ = get_node()->get_parameter("p_gain").as_double();

  auto target_pose_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    PoseCommand new_pose;
    if (!pose_desired_buffer_.readFromRT()->is_initialized) {
        first_target_received_time_ = this->get_node()->now();
    }
    new_pose.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    // --- 시작: 오타 수정 ---
    // 수정 전: msg.pose.orientation.y
    // 수정 후: msg->pose.orientation.y
    new_pose.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    // --- 종료: 오타 수정 ---
    new_pose.is_initialized = true;
    pose_desired_buffer_.writeFromNonRT(new_pose);
  };

  target_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(target_pose_topic, 1, target_pose_callback);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleCartesianVelocityController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_state_->assign_loaned_state_interfaces(state_interfaces_);
  pose_desired_buffer_.initRT(PoseCommand{});
  first_target_received_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleCartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  franka_cartesian_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}
} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::SimpleCartesianVelocityController, controller_interface::ControllerInterface)
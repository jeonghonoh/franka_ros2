#include "franka_example_controllers/combined_joint_cartesian_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cassert>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration CombinedJointCartesianController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Joint Effort 인터페이스 이름
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  // Cartesian Velocity 인터페이스 이름
  config.names.push_back(arm_id_ + "_cartesian_velocity/linear.x");
  config.names.push_back(arm_id_ + "_cartesian_velocity/linear.y");
  config.names.push_back(arm_id_ + "_cartesian_velocity/linear.z");
  config.names.push_back(arm_id_ + "_cartesian_velocity/angular.x");
  config.names.push_back(arm_id_ + "_cartesian_velocity/angular.y");
  config.names.push_back(arm_id_ + "_cartesian_velocity/angular.z");

  return config;
}

controller_interface::InterfaceConfiguration CombinedJointCartesianController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  auto cartesian_state_names = franka_cartesian_state_->get_state_interface_names();
  config.names.insert(config.names.end(), cartesian_state_names.begin(), cartesian_state_names.end());

  return config;
}

controller_interface::return_type CombinedJointCartesianController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {

  switch (current_mode_) {
    case ControlMode::JOINT_POSITION: {
      update_joint_states();
      const auto& desired_cmd = *joint_cmd_buffer_.readFromRT();
      const double kAlpha = 0.99;
      dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
      Vector7d tau_d = k_gains_.cwiseProduct(desired_cmd.joints - q_) + d_gains_.cwiseProduct(-dq_filtered_);
      for (int i = 0; i < num_joints; ++i) {
          command_interfaces_[i].set_value(tau_d(i));
      }
      break;
    }
    case ControlMode::CARTESIAN_VELOCITY: {
      const auto& desired_cmd = *cartesian_cmd_buffer_.readFromRT();
      if (!desired_cmd.is_initialized) {
        for(size_t i = num_joints; i < command_interfaces_.size(); ++i) {
            command_interfaces_[i].set_value(0.0);
        }
        break;
      }
      auto [current_orientation, current_position] = franka_cartesian_state_->getCurrentOrientationAndTranslation();
      Eigen::Vector3d position_error = desired_cmd.position - current_position;
      Eigen::Quaterniond orientation_error = desired_cmd.orientation * current_orientation.inverse();
      Eigen::AngleAxisd angle_axis_error(orientation_error);
      Eigen::Vector3d rotation_error = angle_axis_error.axis() * angle_axis_error.angle();
      Eigen::Vector3d target_linear_velocity = p_gain_ * position_error;
      Eigen::Vector3d target_angular_velocity = p_gain_ * rotation_error;
      auto time_since_first_target = time - first_cartesian_target_time_;
      double ramp_factor = std::clamp(time_since_first_target.seconds() / kRampDuration.seconds(), 0.0, 1.0);
      Eigen::Vector3d commanded_linear_velocity = ramp_factor * target_linear_velocity;
      Eigen::Vector3d commanded_angular_velocity = ramp_factor * target_angular_velocity;
      if (commanded_linear_velocity.norm() > 0.2) {
        commanded_linear_velocity.normalize();
        commanded_linear_velocity *= 0.2;
      }
      // command_interfaces_에 직접 Cartesian Velocity 명령 쓰기
      // 순서: 7:lx, 8:ly, 9:lz, 10:ax, 11:ay, 12:az
      command_interfaces_[7].set_value(commanded_linear_velocity.x());
      command_interfaces_[8].set_value(commanded_linear_velocity.y());
      command_interfaces_[9].set_value(commanded_linear_velocity.z());
      command_interfaces_[10].set_value(commanded_angular_velocity.x());
      command_interfaces_[11].set_value(commanded_angular_velocity.y());
      command_interfaces_[12].set_value(commanded_angular_velocity.z());
      break;
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn CombinedJointCartesianController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    
    // Velocity Semantic Component 초기화 제거
    franka_cartesian_state_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(false);

    auto_declare<std::string>("joint_command_topic", "joint_position_command");
    auto_declare<std::string>("cartesian_command_topic", "cartesian_pose_command");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("p_gain", p_gain_);
  } catch(const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CombinedJointCartesianController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    // on_configure 로직은 변경 없음
    auto k_gains_vec = get_node()->get_parameter("k_gains").as_double_array();
    auto d_gains_vec = get_node()->get_parameter("d_gains").as_double_array();
    p_gain_ = get_node()->get_parameter("p_gain").as_double();
    for (int i = 0; i < num_joints; i++) {
        k_gains_(i) = k_gains_vec.at(i);
        d_gains_(i) = d_gains_vec.at(i);
    }
    auto joint_callback = [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        JointPositionCommand cmd;
        cmd.joints = Eigen::Map<const Vector7d>(msg->data.data());
        joint_cmd_buffer_.writeFromNonRT(cmd);
        current_mode_ = ControlMode::JOINT_POSITION;
    };
    auto cartesian_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        CartesianPoseCommand cmd;
        if (!cartesian_cmd_buffer_.readFromRT()->is_initialized) {
        first_cartesian_target_time_ = this->get_node()->now();
        }
        cmd.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        cmd.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        cmd.is_initialized = true;
        cartesian_cmd_buffer_.writeFromNonRT(cmd);
        current_mode_ = ControlMode::CARTESIAN_VELOCITY;
    };
    auto joint_topic = get_node()->get_parameter("joint_command_topic").as_string();
    auto cartesian_topic = get_node()->get_parameter("cartesian_command_topic").as_string();
    joint_cmd_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(joint_topic, 1, joint_callback);
    cartesian_cmd_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(cartesian_topic, 1, cartesian_callback);
    return CallbackReturn::SUCCESS;
}

CallbackReturn CombinedJointCartesianController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // Velocity Semantic Component 관련 코드 제거
  franka_cartesian_state_->assign_loaned_state_interfaces(state_interfaces_);
  
  update_joint_states();
  JointPositionCommand initial_joint_cmd;
  initial_joint_cmd.joints = q_;
  joint_cmd_buffer_.initRT(initial_joint_cmd);

  cartesian_cmd_buffer_.initRT(CartesianPoseCommand{});
  first_cartesian_target_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  current_mode_ = ControlMode::JOINT_POSITION;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CombinedJointCartesianController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // Velocity Semantic Component 관련 코드 제거
  franka_cartesian_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CombinedJointCartesianController::update_joint_states() {
  for (size_t i = 0; i < 7; ++i) {
    const auto& pos_if = state_interfaces_.at(2 * i);
    const auto& vel_if = state_interfaces_.at(2 * i + 1);
    q_(i) = pos_if.get_value();
    dq_(i) = vel_if.get_value();
  }
}

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CombinedJointCartesianController, controller_interface::ControllerInterface)
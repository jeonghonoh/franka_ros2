#include <franka_example_controllers/trajectory_tracking_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>
#include <chrono>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
TrajectoryTrackingExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Use the command interface names from the FrankaCartesianPoseInterface.
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
TrajectoryTrackingExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

TrajectoryTrackingExampleController::CallbackReturn
TrajectoryTrackingExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", arm_id_);
    auto_declare<std::string>("trajectory_file", "");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

TrajectoryTrackingExampleController::CallbackReturn
TrajectoryTrackingExampleController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  trajectory_file_ = get_node()->get_parameter("trajectory_file").as_string();

  // Set default collision behavior (similar to CartesianPoseExampleController)
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  // Use a literal timeout (2 seconds) instead of robot_utils::time_out.
  auto future_result = client->async_send_request(request);
  future_result.wait_for(std::chrono::seconds(2));
  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  // Load trajectory from YAML file.
  if (!loadTrajectoryFromFile(trajectory_file_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to load trajectory from file: %s", trajectory_file_.c_str());
    return CallbackReturn::FAILURE;
  }

  // Initialize the Franka Cartesian Pose Interface.
  franka_cartesian_pose_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(false);
  
  return CallbackReturn::SUCCESS;
}

TrajectoryTrackingExampleController::CallbackReturn
TrajectoryTrackingExampleController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  // Loan the command and state interfaces from the Cartesian Pose Interface.
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  // Get the current pose from the robot.
  std::tie(current_orientation_, current_position_) =
      franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  initial_robot_time_ = state_interfaces_.back().get_value();

  // Override the first waypoint in the trajectory with the current pose,
  // ensuring a continuous transition.
  if (!trajectory_.empty()) {
    trajectory_.front().position = current_position_;
    trajectory_.front().orientation = current_orientation_;
    RCLCPP_INFO(get_node()->get_logger(),
                "Overridden first trajectory waypoint with current pose.");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Trajectory is empty.");
  }

  return CallbackReturn::SUCCESS;
}
// TrajectoryTrackingExampleController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
//   initialization_flag_ = true;
//   elapsed_time_ = 0.0;
//   // Loan the command and state interfaces from the Cartesian Pose Interface.
//   franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
//   franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
//   return CallbackReturn::SUCCESS;
// }

controller_interface::CallbackReturn
TrajectoryTrackingExampleController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
TrajectoryTrackingExampleController::update(const rclcpp::Time & /*time*/,
                                              const rclcpp::Duration & /*period*/) {
  if (initialization_flag_) {
    std::tie(current_orientation_, current_position_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    initial_robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = 0.0;
    initialization_flag_ = false;
  } else {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }

  Eigen::Quaterniond desired_orientation;
  Eigen::Vector3d desired_position;
  getDesiredPose(elapsed_time_, desired_orientation, desired_position);

  if (franka_cartesian_pose_->setCommand(desired_orientation, desired_position)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Check if the Cartesian command interface is activated.");
    return controller_interface::return_type::ERROR;
  }
}

bool TrajectoryTrackingExampleController::loadTrajectoryFromFile(const std::string &file_path) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["trajectory"]) {
      RCLCPP_ERROR(get_node()->get_logger(), "No 'trajectory' key found in YAML file.");
      return false;
    }
    auto traj = config["trajectory"];
    for (size_t i = 0; i < traj.size(); i++) {
      PoseCheckpoint checkpoint;
      checkpoint.time_sec = traj[i]["time_sec"].as<double>();
      std::vector<double> pose = traj[i]["pose"].as<std::vector<double>>();
      if (pose.size() != 7) {
        RCLCPP_ERROR(get_node()->get_logger(), "Each pose must have 7 elements (3 for position, 4 for orientation).");
        return false;
      }
      checkpoint.position = Eigen::Vector3d(pose[0], pose[1], pose[2]);
      // Construct quaternion as Eigen::Quaterniond(qw, qx, qy, qz)
      checkpoint.orientation = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
      trajectory_.push_back(checkpoint);
    }
  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "YAML Exception: %s", e.what());
    return false;
  }
  return true;
}

void TrajectoryTrackingExampleController::getDesiredPose(double t, Eigen::Quaterniond &desired_orientation, Eigen::Vector3d &desired_position) {
  if (trajectory_.empty()) {
    desired_orientation = current_orientation_;
    desired_position = current_position_;
    return;
  }
  if (t <= trajectory_.front().time_sec) {
    desired_orientation = trajectory_.front().orientation;
    desired_position = trajectory_.front().position;
    return;
  }
  if (t >= trajectory_.back().time_sec) {
    desired_orientation = trajectory_.back().orientation;
    desired_position = trajectory_.back().position;
    return;
  }
  
  // Find the interval [t_i, t_{i+1}] that contains t.
  for (size_t i = 0; i < trajectory_.size() - 1; i++) {
    double t_i = trajectory_[i].time_sec;
    double t_next = trajectory_[i+1].time_sec;
    if (t >= t_i && t < t_next) {
      double dt = t_next - t_i;
      double s = (t - t_i) / dt;
      // Cubic smoothstep: f(s) = 3*s^2 - 2*s^3; f(0)=0, f(1)=1, and f'(0)=f'(1)=0.
      double smooth_factor = 3 * s * s - 2 * s * s * s;
      desired_position = trajectory_[i].position + smooth_factor * (trajectory_[i+1].position - trajectory_[i].position);
      // Orientation: slerp (이미 연속적인 보간)
      desired_orientation = trajectory_[i].orientation.slerp(s, trajectory_[i+1].orientation);
      return;
    }
  }
  // Fallback (should not reach here)
  desired_orientation = trajectory_.back().orientation;
  desired_position = trajectory_.back().position;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::TrajectoryTrackingExampleController,
                       controller_interface::ControllerInterface)

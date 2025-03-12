#ifndef TRAJECTORY_TRACKING_EXAMPLE_CONTROLLER_HPP_
#define TRAJECTORY_TRACKING_EXAMPLE_CONTROLLER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <chrono>
#include <yaml-cpp/yaml.h>

// Include header for FrankaCartesianPoseInterface
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

namespace franka_example_controllers {

// Structure for a pose checkpoint (each checkpoint contains time and a 7-element pose vector)
struct PoseCheckpoint {
  double time_sec;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

class CartesianTrajectoryTrackingExampleController : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  
  // Overridden interface configuration functions
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  // Lifecycle methods
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Loads trajectory from YAML file and fills trajectory_ vector.
  // This function expects each checkpoint to have a "pose" key with 7 elements.
  bool loadTrajectoryFromFile(const std::string &file_path);

  // Returns desired pose (orientation & position) at time t by interpolating between checkpoints.
  void getDesiredPose(double t, Eigen::Quaterniond &desired_orientation, Eigen::Vector3d &desired_position);

  // Trajectory checkpoints
  std::vector<PoseCheckpoint> trajectory_;

  // Parameter for trajectory file path
  std::string trajectory_file_;

  // Cartesian pose interface pointer.
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  // Variables for storing current/initial pose and time.
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d current_position_;

  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  double elapsed_time_{0.0};
  bool initialization_flag_{true};

  // Arm id parameter for interface naming.
  std::string arm_id_;
};

}  // namespace franka_example_controllers

#endif  // TRAJECTORY_TRACKING_EXAMPLE_CONTROLLER_HPP_

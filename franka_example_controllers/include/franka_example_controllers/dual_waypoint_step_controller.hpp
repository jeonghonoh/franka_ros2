#pragma once
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

namespace franka_example_controllers {

using CallbackReturn = controller_interface::CallbackReturn;
using Vector7d       = Eigen::Matrix<double,7,1>;

struct Waypoint { Vector7d q; };

class DualWaypointStepController : public controller_interface::ControllerInterface {
public:
  DualWaypointStepController();

  // ★ 여기서 명시적으로 인터페이스 이름을 나열합니다.
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()   const override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init()        override;
  CallbackReturn on_configure (const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate  (const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

private:
  // parameters
  std::string arm_id_;
  std::string waypoint_file_;
  double      pos_tol_{0.01}, vel_tol_{0.02};
  Eigen::Array<double,7,1> k_gains_, d_gains_;

  // execution state
  enum class Phase { MOVING, WAIT_STABLE, WAIT_ACK, FINISHED } phase_{Phase::MOVING};
  std::vector<Waypoint> waypoints_;
  size_t       current_{0};
  Vector7d     q_, dq_, dq_filt_;
  rclcpp::Time stable_since_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr    pub_ready_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_ack_;

  bool loadWaypoints(const std::string& yaml_path);
  bool isStable() const;
  void nextWaypoint();
  void updateJointStates();
};

}  // namespace franka_example_controllers

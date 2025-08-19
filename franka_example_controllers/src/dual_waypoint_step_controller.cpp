#include "franka_example_controllers/dual_waypoint_step_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cassert>

namespace franka_example_controllers {

constexpr int kDoF     = 7;
constexpr double kTauMax = 120.0;

DualWaypointStepController::DualWaypointStepController() {
  q_.setZero();
  dq_.setZero();
  dq_filt_.setZero();
  k_gains_.setConstant(400.0);
  d_gains_.setConstant(20.0);
}

// -------------------------------------------------------------------
// 1) 명시적 InterfaceConfiguration
// -------------------------------------------------------------------
controller_interface::InterfaceConfiguration
DualWaypointStepController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kDoF; ++i) {
    cfg.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
DualWaypointStepController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kDoF; ++i) {
    cfg.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    cfg.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return cfg;
}

// -------------------------------------------------------------------
// lifecycle hooks
// -------------------------------------------------------------------
CallbackReturn DualWaypointStepController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::string>("waypoint_file", "");
    auto_declare<double>("pos_tol", 0.01);
    auto_declare<double>("vel_tol", 0.02);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init error: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualWaypointStepController::on_configure(
    const rclcpp_lifecycle::State&) {
  arm_id_        = get_node()->get_parameter("arm_id").as_string();
  waypoint_file_ = get_node()->get_parameter("waypoint_file").as_string();
  pos_tol_       = get_node()->get_parameter("pos_tol").as_double();
  vel_tol_       = get_node()->get_parameter("vel_tol").as_double();

  auto kg = get_node()->get_parameter("k_gains").as_double_array();
  auto dg = get_node()->get_parameter("d_gains").as_double_array();
  if (kg.size() == kDoF && dg.size() == kDoF) {
    for (int i = 0; i < kDoF; ++i) {
      k_gains_[i] = kg[i];
      d_gains_[i] = dg[i];
    }
  }

  if (!loadWaypoints(waypoint_file_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load waypoints");
    return CallbackReturn::FAILURE;
  }

  pub_ready_ = get_node()->create_publisher<std_msgs::msg::Empty>(
      "/ready_for_capture", 1);
  sub_ack_ = get_node()->create_subscription<std_msgs::msg::Empty>(
      "/capture_done", 1,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        if (phase_ == Phase::WAIT_ACK) nextWaypoint();
      });

  return CallbackReturn::SUCCESS;
}

CallbackReturn DualWaypointStepController::on_activate(
    const rclcpp_lifecycle::State&) {
  current_      = 0;
  phase_        = Phase::MOVING;
  stable_since_ = get_node()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualWaypointStepController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  // 모두 0으로 초기화
  for (int i = 0; i < kDoF; ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------
// realtime update loop
// -------------------------------------------------------------------
controller_interface::return_type
DualWaypointStepController::update(
    const rclcpp::Time&, const rclcpp::Duration&) {
  updateJointStates();

  if (phase_ == Phase::FINISHED) {
    for (int i = 0; i < kDoF; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // PD 제어
  const Vector7d &q_des = waypoints_[current_].q;
  dq_filt_ = 0.9 * dq_filt_ + 0.1 * dq_;
  Vector7d tau =
      k_gains_.matrix().asDiagonal() * (q_des - q_) -
      d_gains_.matrix().asDiagonal() * dq_filt_;
  tau = tau.cwiseMax(-kTauMax).cwiseMin(kTauMax);

  for (int i = 0; i < kDoF; ++i) {
    command_interfaces_[i].set_value(tau[i]);
  }

  // 상태 전이
  switch (phase_) {
    case Phase::MOVING:
      if (isStable()) {
        phase_        = Phase::WAIT_STABLE;
        stable_since_ = get_node()->now();
      }
      break;
    case Phase::WAIT_STABLE:
      if (!isStable()) {
        phase_ = Phase::MOVING;
      } else if ((get_node()->now() - stable_since_).seconds() > 0.25) {
        pub_ready_->publish(std_msgs::msg::Empty());
        phase_ = Phase::WAIT_ACK;
      }
      break;
    default:
      break;
  }

  return controller_interface::return_type::OK;
}

// -------------------------------------------------------------------
// helpers
// -------------------------------------------------------------------
bool DualWaypointStepController::loadWaypoints(
    const std::string &yaml_path) {
  try {
    auto root = YAML::LoadFile(yaml_path);
    std::string ns = get_node()->get_namespace();
    bool is_left = (ns.find("leftarm") != std::string::npos);
    std::string key = is_left ? "left_trajectory" : "right_trajectory";

    auto seq = root[key];
    if (!seq || !seq.IsSequence()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "YAML key '%s' missing", key.c_str());
      return false;
    }
    waypoints_.clear();
    // ⇩ 여기도 const auto& 로 변경
    for (const auto &n : seq) {
      auto vec = n.as<std::vector<double>>();
      if (vec.size() != kDoF) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Waypoint length must be %d", kDoF);
        return false;
      }
      Waypoint w;
      w.q = Eigen::Map<Vector7d>(vec.data());
      waypoints_.push_back(w);
    }
    return !waypoints_.empty();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "YAML parse error: %s", e.what());
    return false;
  }
}

bool DualWaypointStepController::isStable() const {
  double pos_err = (waypoints_[current_].q - q_).cwiseAbs().maxCoeff();
  double vel_mag = dq_.cwiseAbs().maxCoeff();
  return pos_err < pos_tol_ && vel_mag < vel_tol_;
}

void DualWaypointStepController::nextWaypoint() {
  ++current_;
  phase_ = (current_ >= waypoints_.size()) ? Phase::FINISHED : Phase::MOVING;
}

void DualWaypointStepController::updateJointStates() {
  for (int i = 0; i < kDoF; ++i) {
    // state_interface_configuration() 순서와 1:1 매핑
    q_[i]  = state_interfaces_[2 * i].get_value();
    dq_[i] = state_interfaces_[2 * i + 1].get_value();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::DualWaypointStepController,
    controller_interface::ControllerInterface);

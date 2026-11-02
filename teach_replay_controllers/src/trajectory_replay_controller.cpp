#include <teach_replay_controllers/trajectory_replay_controller.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace teach_replay_controllers {

namespace {
// Largest period the timeline may advance by in one cycle. A stalled control loop must not
// make the target jump ahead when it resumes.
constexpr double kMaxStep = 0.005;
// Peak of the derivative of the quintic blend 10s^3 - 15s^4 + 6s^5 (at s = 0.5).
constexpr double kQuinticPeakVelocity = 1.875;
}  // namespace

controller_interface::InterfaceConfiguration
TrajectoryReplayController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
TrajectoryReplayController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

void TrajectoryReplayController::desired(Vector7d& q_d, Vector7d& dq_d) const {
  if (phase_ == kApproach) {
    const double s = std::clamp(phase_time_ / approach_duration_, 0.0, 1.0);
    const double blend = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
    const double dblend = 30.0 * s * s * (1.0 - s) * (1.0 - s) / approach_duration_;
    const Vector7d delta = q_traj_.front() - q_start_;
    q_d = q_start_ + blend * delta;
    dq_d = dblend * delta;
    return;
  }
  if (phase_ == kDone) {
    q_d = q_traj_.back();
    dq_d.setZero();
    return;
  }
  const double index = phase_time_ / sample_period_;
  const size_t k = std::min(static_cast<size_t>(index), q_traj_.size() - 2);
  const double a = std::clamp(index - static_cast<double>(k), 0.0, 1.0);
  q_d = (1.0 - a) * q_traj_[k] + a * q_traj_[k + 1];
  dq_d = (1.0 - a) * dq_traj_[k] + a * dq_traj_[k + 1];
}

controller_interface::return_type TrajectoryReplayController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  Vector7d q;
  Vector7d dq;
  for (int i = 0; i < kNumJoints; ++i) {
    q(i) = state_interfaces_[2 * i].get_value();
    dq(i) = state_interfaces_[2 * i + 1].get_value();
  }

  Vector7d q_d;
  Vector7d dq_d;
  desired(q_d, dq_d);

  const double error = (q_d - q).cwiseAbs().maxCoeff();
  if (!paused_ && error > max_tracking_error_) {
    paused_ = true;
  } else if (paused_ && error < 0.5 * max_tracking_error_) {
    paused_ = false;
  }

  Vector7d tau = k_gains_.cwiseProduct(q_d - q) + d_gains_.cwiseProduct(dq_d - dq);
  tau = tau.cwiseMax(-tau_max_).cwiseMin(tau_max_);
  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau(i));
  }

  const double step = std::clamp(period.seconds(), 0.0, kMaxStep);
  if (!paused_ && phase_ != kDone) {
    phase_time_ += step;
    if (phase_ == kApproach && phase_time_ >= approach_duration_) {
      phase_ = kMain;
      phase_time_ = 0.0;
    } else if (phase_ == kMain && phase_time_ >= main_duration_) {
      phase_ = kDone;
      phase_time_ = main_duration_;
    }
  }

  since_progress_ += step;
  if (since_progress_ >= progress_period_ && rt_progress_->trylock()) {
    since_progress_ = 0.0;
    auto& data = rt_progress_->msg_.data;
    data[0] = static_cast<double>(phase_);
    data[1] = phase_ == kApproach ? phase_time_ / approach_duration_ : phase_time_;
    data[2] = paused_ ? 1.0 : 0.0;
    data[3] = main_duration_;
    rt_progress_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

bool TrajectoryReplayController::loadTrajectory(const std::string& path, std::string& error) {
  std::ifstream file(path);
  if (!file) {
    error = "cannot open trajectory_file '" + path + "'";
    return false;
  }
  std::vector<double> times;
  q_traj_.clear();
  dq_traj_.clear();
  std::string line;
  size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty() || line[0] == '#' || std::isalpha(static_cast<unsigned char>(line[0]))) {
      continue;  // header / comments
    }
    std::stringstream ss(line);
    std::vector<double> values;
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      values.push_back(std::stod(cell));
    }
    if (values.size() != 1 + 2 * kNumJoints) {
      error = "line " + std::to_string(line_number) + ": expected " +
              std::to_string(1 + 2 * kNumJoints) + " columns (t, q1..q7, dq1..dq7), got " +
              std::to_string(values.size());
      return false;
    }
    for (double v : values) {
      if (!std::isfinite(v)) {
        error = "line " + std::to_string(line_number) + ": non-finite value";
        return false;
      }
    }
    Vector7d q;
    Vector7d dq;
    for (int i = 0; i < kNumJoints; ++i) {
      q(i) = values[1 + i];
      dq(i) = values[1 + kNumJoints + i];
    }
    times.push_back(values[0]);
    q_traj_.push_back(q);
    dq_traj_.push_back(dq);
  }
  if (times.size() < 2) {
    error = "trajectory_file has fewer than 2 samples";
    return false;
  }
  sample_period_ = times[1] - times[0];
  if (sample_period_ <= 0.0) {
    error = "sample times must increase";
    return false;
  }
  for (size_t k = 1; k < times.size(); ++k) {
    if (std::abs((times[k] - times[k - 1]) - sample_period_) > 1e-6) {
      error = "samples must be evenly spaced (row " + std::to_string(k) + ")";
      return false;
    }
  }
  main_duration_ = sample_period_ * static_cast<double>(times.size() - 1);
  return true;
}

CallbackReturn TrajectoryReplayController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::string>("trajectory_file", "");
    auto_declare<std::vector<double>>("k_gains", {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
    auto_declare<std::vector<double>>("d_gains", {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0});
    auto_declare<std::vector<double>>("tau_max", {60.0, 60.0, 60.0, 60.0, 10.0, 10.0, 10.0});
    auto_declare<double>("approach_max_velocity", 0.2);
    auto_declare<double>("min_approach_duration", 3.0);
    auto_declare<double>("max_tracking_error", 0.15);
    auto_declare<double>("progress_rate", 100.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn TrajectoryReplayController::on_configure(
    const rclcpp_lifecycle::State& /*previous*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();
  const auto k = node->get_parameter("k_gains").as_double_array();
  const auto d = node->get_parameter("d_gains").as_double_array();
  const auto t = node->get_parameter("tau_max").as_double_array();
  if (k.size() != kNumJoints || d.size() != kNumJoints || t.size() != kNumJoints) {
    RCLCPP_FATAL(node->get_logger(), "k_gains, d_gains, tau_max must each have %d entries",
                 kNumJoints);
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < kNumJoints; ++i) {
    k_gains_(i) = k[i];
    d_gains_(i) = d[i];
    tau_max_(i) = t[i];
  }
  approach_max_velocity_ = node->get_parameter("approach_max_velocity").as_double();
  min_approach_duration_ = node->get_parameter("min_approach_duration").as_double();
  max_tracking_error_ = node->get_parameter("max_tracking_error").as_double();
  const double progress_rate = node->get_parameter("progress_rate").as_double();
  if (approach_max_velocity_ <= 0.0 || min_approach_duration_ <= 0.0 ||
      max_tracking_error_ <= 0.0 || progress_rate <= 0.0) {
    RCLCPP_FATAL(node->get_logger(),
                 "approach_max_velocity, min_approach_duration, max_tracking_error and "
                 "progress_rate must be > 0");
    return CallbackReturn::FAILURE;
  }
  progress_period_ = 1.0 / progress_rate;

  const auto path = node->get_parameter("trajectory_file").as_string();
  if (path.empty()) {
    RCLCPP_FATAL(node->get_logger(), "trajectory_file parameter not set");
    return CallbackReturn::FAILURE;
  }
  std::string error;
  try {
    if (!loadTrajectory(path, error)) {
      RCLCPP_FATAL(node->get_logger(), "Failed to load trajectory: %s", error.c_str());
      return CallbackReturn::FAILURE;
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to parse '%s': %s", path.c_str(), e.what());
    return CallbackReturn::FAILURE;
  }

  progress_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/progress", rclcpp::SystemDefaultsQoS());
  rt_progress_ = std::make_unique<ProgressPublisher>(progress_publisher_);
  rt_progress_->msg_.data.assign(4, 0.0);

  RCLCPP_INFO(node->get_logger(), "Loaded %zu samples (%.3f s at %.0f Hz) from %s",
              q_traj_.size(), main_duration_, 1.0 / sample_period_, path.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn TrajectoryReplayController::on_activate(
    const rclcpp_lifecycle::State& /*previous*/) {
  if (q_traj_.size() < 2) {
    RCLCPP_ERROR(get_node()->get_logger(), "No trajectory loaded");
    return CallbackReturn::ERROR;
  }
  for (int i = 0; i < kNumJoints; ++i) {
    q_start_(i) = state_interfaces_[2 * i].get_value();
  }
  const double distance = (q_traj_.front() - q_start_).cwiseAbs().maxCoeff();
  approach_duration_ = std::max(min_approach_duration_,
                                kQuinticPeakVelocity * distance / approach_max_velocity_);
  phase_ = kApproach;
  phase_time_ = 0.0;
  paused_ = false;
  since_progress_ = progress_period_;
  RCLCPP_INFO(get_node()->get_logger(),
              "Activated: approaching trajectory start (max offset %.3f rad) in %.2f s, "
              "then replaying %.2f s",
              distance, approach_duration_, main_duration_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TrajectoryReplayController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous*/) {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace teach_replay_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(teach_replay_controllers::TrajectoryReplayController,
                       controller_interface::ControllerInterface)

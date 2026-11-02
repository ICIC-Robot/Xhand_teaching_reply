#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace teach_replay_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Deterministic replay of a preprocessed joint trajectory with joint impedance tracking.
 *
 * on_configure loads `trajectory_file` (CSV, header row, then rows of
 * t, q1..q7, dq1..dq7 at a fixed sample period -- written by teach_replay's process_demo).
 * Loading happens here, outside the realtime loop, because on_activate runs inside it.
 *
 * Phases, published on ~/progress as [phase, value, paused, main_duration]:
 *   0 APPROACH  quintic move from the pose at activation to the trajectory start, slow enough
 *               that no joint exceeds approach_max_velocity; value = progress in [0, 1]
 *   1 MAIN      the trajectory, indexed by time and linearly interpolated between samples;
 *               value = trajectory time in seconds
 *   2 DONE      holds the last sample; value = main_duration
 *
 * Command: tau = K (q_d - q) + D (dq_d - dq), clamped to tau_max (the robot adds gravity).
 * If any joint lags its target by more than max_tracking_error, time stops advancing (the target
 * waits for the arm) until the error is back under half that value.
 */
class TrajectoryReplayController : public controller_interface::ControllerInterface {
 public:
  static constexpr int kNumJoints = 7;
  using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;

  enum Phase { kApproach = 0, kMain = 1, kDone = 2 };

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  using ProgressPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;

  bool loadTrajectory(const std::string& path, std::string& error);
  void desired(Vector7d& q_d, Vector7d& dq_d) const;

  std::string arm_id_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d tau_max_;
  double approach_max_velocity_{0.2};
  double min_approach_duration_{3.0};
  double max_tracking_error_{0.15};
  double progress_period_{0.01};

  std::vector<Vector7d> q_traj_;
  std::vector<Vector7d> dq_traj_;
  double sample_period_{0.001};
  double main_duration_{0.0};

  Phase phase_{kApproach};
  Vector7d q_start_;
  double approach_duration_{0.0};
  double phase_time_{0.0};
  bool paused_{false};
  double since_progress_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr progress_publisher_;
  std::unique_ptr<ProgressPublisher> rt_progress_;
};

}  // namespace teach_replay_controllers

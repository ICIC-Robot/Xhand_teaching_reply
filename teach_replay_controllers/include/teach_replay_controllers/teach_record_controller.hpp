#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace teach_replay_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * Hand-guided teaching controller.
 *
 * Starts from the gravity compensation controller: in libfranka torque mode the robot already
 * compensates its own weight, so commanding zero torque leaves the arm free to be pushed by hand.
 * On top of that it commands a joint damping torque tau = -D * dq (clamped to tau_max) so a fast
 * push is resisted before it can approach the joint velocity limits.
 *
 * Every 1 / publish_rate seconds it publishes q, dq, the commanded torque and the controller
 * timestamp on ~/joint_states through a RealtimePublisher, for `ros2 bag record`.
 */
class TeachRecordController : public controller_interface::ControllerInterface {
 public:
  static constexpr int kNumJoints = 7;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  using JointStatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;

  std::string arm_id_;
  std::array<double, kNumJoints> damping_{};
  std::array<double, kNumJoints> tau_max_{};
  double publish_period_{0.002};
  double since_publish_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::unique_ptr<JointStatePublisher> rt_publisher_;
};

}  // namespace teach_replay_controllers

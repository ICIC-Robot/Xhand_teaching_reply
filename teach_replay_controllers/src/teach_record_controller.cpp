#include <teach_replay_controllers/teach_record_controller.hpp>

#include <algorithm>
#include <exception>
#include <string>
#include <vector>

namespace teach_replay_controllers {

controller_interface::InterfaceConfiguration
TeachRecordController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
TeachRecordController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type TeachRecordController::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period) {
  std::array<double, kNumJoints> q{};
  std::array<double, kNumJoints> dq{};
  std::array<double, kNumJoints> tau{};
  for (int i = 0; i < kNumJoints; ++i) {
    q[i] = state_interfaces_[2 * i].get_value();
    dq[i] = state_interfaces_[2 * i + 1].get_value();
    tau[i] = std::clamp(-damping_[i] * dq[i], -tau_max_[i], tau_max_[i]);
    command_interfaces_[i].set_value(tau[i]);
  }

  since_publish_ += period.seconds();
  if (since_publish_ >= publish_period_) {
    since_publish_ = 0.0;
    if (rt_publisher_->trylock()) {
      auto& msg = rt_publisher_->msg_;
      msg.header.stamp = time;
      std::copy(q.begin(), q.end(), msg.position.begin());
      std::copy(dq.begin(), dq.end(), msg.velocity.begin());
      std::copy(tau.begin(), tau.end(), msg.effort.begin());
      rt_publisher_->unlockAndPublish();
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn TeachRecordController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("damping", {5.0, 5.0, 5.0, 5.0, 2.0, 2.0, 1.0});
    auto_declare<std::vector<double>>("tau_max", {20.0, 20.0, 20.0, 20.0, 6.0, 6.0, 6.0});
    auto_declare<double>("publish_rate", 500.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn TeachRecordController::on_configure(const rclcpp_lifecycle::State& /*previous*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();
  auto damping = node->get_parameter("damping").as_double_array();
  auto tau_max = node->get_parameter("tau_max").as_double_array();
  double rate = node->get_parameter("publish_rate").as_double();
  if (damping.size() != kNumJoints || tau_max.size() != kNumJoints) {
    RCLCPP_FATAL(node->get_logger(), "damping and tau_max must each have %d entries", kNumJoints);
    return CallbackReturn::FAILURE;
  }
  if (rate <= 0.0 || rate > 1000.0) {
    RCLCPP_FATAL(node->get_logger(), "publish_rate must be in (0, 1000] Hz, got %f", rate);
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < kNumJoints; ++i) {
    if (damping[i] < 0.0 || tau_max[i] < 0.0) {
      RCLCPP_FATAL(node->get_logger(), "damping and tau_max must be >= 0");
      return CallbackReturn::FAILURE;
    }
    damping_[i] = damping[i];
    tau_max_[i] = tau_max[i];
  }
  publish_period_ = 1.0 / rate;

  publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("~/joint_states",
                                                                    rclcpp::SystemDefaultsQoS());
  rt_publisher_ = std::make_unique<JointStatePublisher>(publisher_);
  auto& msg = rt_publisher_->msg_;
  msg.name.clear();
  for (int i = 1; i <= kNumJoints; ++i) {
    msg.name.push_back(arm_id_ + "_joint" + std::to_string(i));
  }
  msg.position.assign(kNumJoints, 0.0);
  msg.velocity.assign(kNumJoints, 0.0);
  msg.effort.assign(kNumJoints, 0.0);

  RCLCPP_INFO(node->get_logger(), "Configured: publishing ~/joint_states at %.0f Hz", rate);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TeachRecordController::on_activate(const rclcpp_lifecycle::State& /*previous*/) {
  since_publish_ = publish_period_;  // publish on the first cycle
  return CallbackReturn::SUCCESS;
}

CallbackReturn TeachRecordController::on_deactivate(const rclcpp_lifecycle::State& /*previous*/) {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace teach_replay_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(teach_replay_controllers::TeachRecordController,
                       controller_interface::ControllerInterface)

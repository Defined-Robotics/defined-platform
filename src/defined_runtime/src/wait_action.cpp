#include "defined_runtime/wait_action.hpp"

namespace defined_runtime {

WaitAction::WaitAction(const std::string& name, const BT::NodeConfig& config,
                       rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList WaitAction::providedPorts() {
  return {
      BT::InputPort<double>("duration", "Wait duration in seconds"),
  };
}

BT::NodeStatus WaitAction::onStart() {
  if (!getInput("duration", duration_sec_)) {
    throw BT::RuntimeError("WaitAction: missing required input port [duration]");
  }
  if (duration_sec_ <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }
  start_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "WaitAction: waiting %.1f seconds",
              duration_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitAction::onRunning() {
  auto elapsed = (node_->now() - start_time_).seconds();
  if (elapsed >= duration_sec_) {
    RCLCPP_INFO(node_->get_logger(), "WaitAction: done waiting");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "WaitAction: halted");
}

}  // namespace defined_runtime

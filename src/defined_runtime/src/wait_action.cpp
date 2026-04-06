/*!
 * \file wait_action.cpp
 * \brief Implementation of WaitAction — pauses BT execution for a duration.
 */

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

/*!
 * \brief Read the duration port and start the wall-clock timer.
 *
 * \retval BT::NodeStatus::SUCCESS  Duration is zero or negative; completes immediately.
 * \retval BT::NodeStatus::RUNNING  Timer started; onRunning() will poll elapsed time.
 *
 * \warning Throws \c BT::RuntimeError if the \c duration port is missing.
 */
BT::NodeStatus WaitAction::onStart() {
  if (!getInput("duration", duration_sec_)) {
    throw BT::RuntimeError("WaitAction: missing required input port [duration]");
  }
  if (duration_sec_ <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }
  start_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "WaitAction: waiting %.1f seconds", duration_sec_);
  return BT::NodeStatus::RUNNING;
}

/*!
 * \brief Check whether the requested duration has elapsed.
 *
 * \retval BT::NodeStatus::SUCCESS  Elapsed time has met or exceeded \c duration_sec_.
 * \retval BT::NodeStatus::RUNNING  Duration not yet reached; call again next tick.
 *
 * \warning onStart() must have been called and returned RUNNING before this
 *          method is invoked.
 */
BT::NodeStatus WaitAction::onRunning() {
  auto elapsed = (node_->now() - start_time_).seconds();
  if (elapsed >= duration_sec_) {
    RCLCPP_INFO(node_->get_logger(), "WaitAction: done waiting");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

/*!
 * \brief Log the halt event; no active resources to release.
 *
 * \note No cleanup is required because WaitAction holds no handles or
 *       subscriptions that need explicit cancellation.
 */
void WaitAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "WaitAction: halted");
}

}  // namespace defined_runtime

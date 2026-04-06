/*!
 * \file goto_action.cpp
 * \brief Implementation of GoToAction — sends NavigateToPose goals to Nav2.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <cmath>

#include "defined_runtime/goto_action.hpp"

namespace defined_runtime {

/*******************************************************************************
 * Public Function Bodies
 ******************************************************************************/

GoToAction::GoToAction(const std::string& name, const BT::NodeConfig& config,
                       rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {}

BT::PortsList GoToAction::providedPorts() {
  return {
      BT::InputPort<double>("x", "Target X coordinate (meters)"),
      BT::InputPort<double>("y", "Target Y coordinate (meters)"),
      BT::InputPort<double>("theta", 0.0, "Target orientation (radians)"),
      BT::InputPort<double>("timeout", 60.0, "Navigation timeout in seconds"),
      BT::InputPort<std::string>("frame_id", "map", "Reference frame"),
      BT::InputPort<std::string>("server_name", "/navigate_to_pose", "Nav2 action server"),
      BT::OutputPort<int>("error_code", "Navigation error code"),
  };
}

/*!
 * \brief Convert a yaw angle to a unit quaternion (2-D rotation about Z).
 *
 * \param[in]  theta  Yaw angle in radians.
 * \param[out] qz     Quaternion Z component.
 * \param[out] qw     Quaternion W component.
 */
void GoToAction::ThetaToQuaternion(double theta, double& qz, double& qw) {
  qz = std::sin(theta / 2.0);
  qw = std::cos(theta / 2.0);
}

/*!
 * \brief Get or create an action client for the given Nav2 action server.
 *
 * \param[in] server_name  Fully-qualified action server name.
 *
 * \retval Client::SharedPtr  Cached or newly created action client.
 */
GoToAction::Client::SharedPtr GoToAction::GetClient(const std::string& server_name) {
  auto it = client_cache_.find(server_name);
  if (it != client_cache_.end()) {
    return it->second;
  }
  auto client = rclcpp_action::create_client<NavigateToPose>(node_, server_name);
  client_cache_[server_name] = client;
  return client;
}

/*!
 * \brief Read BT ports, acquire the action client, and send a NavigateToPose goal.
 *
 * \retval BT::NodeStatus::RUNNING  Goal sent successfully; waiting for acceptance.
 * \retval BT::NodeStatus::FAILURE  Required port missing, action server unavailable,
 *                                  or client creation failed.
 *
 * \warning The action server must be reachable within 5 seconds; otherwise the
 *          node returns FAILURE immediately.
 */
BT::NodeStatus GoToAction::onStart() {
  double x, y, theta;
  std::string frame_id, server_name;

  if (!getInput("x", x) || !getInput("y", y)) {
    throw BT::RuntimeError("GoToAction: missing required input port [x] or [y]");
  }
  getInput("theta", theta);
  getInput("timeout", timeout_sec_);
  getInput("frame_id", frame_id);
  getInput("server_name", server_name);

  client_ = GetClient(server_name);

  if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "GoToAction: action server '%s' not available",
                 server_name.c_str());
    setOutput("error_code", -1);
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;

  double qz, qw;
  ThetaToQuaternion(theta, qz, qw);
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = qz;
  goal.pose.pose.orientation.w = qw;

  RCLCPP_INFO(node_->get_logger(), "GoToAction: navigating to (%.2f, %.2f, %.2f) in '%s'", x, y,
              theta, frame_id.c_str());

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  goal_future_ = client_->async_send_goal(goal, send_goal_options);

  start_time_ = node_->now();
  goal_accepted_ = false;
  result_pending_ = false;
  goal_handle_ = nullptr;

  return BT::NodeStatus::RUNNING;
}

/*!
 * \brief Poll goal acceptance and navigation result each BT tick.
 *
 * \retval BT::NodeStatus::RUNNING  Goal in flight; call again next tick.
 * \retval BT::NodeStatus::SUCCESS  Navigation reached the target pose.
 * \retval BT::NodeStatus::FAILURE  Timeout exceeded, goal rejected, or Nav2
 *                                  reported ABORTED/CANCELED.
 *
 * \warning onStart() must have been called and returned RUNNING before this
 *          method is invoked.
 */
BT::NodeStatus GoToAction::onRunning() {
  // Check timeout.
  if ((node_->now() - start_time_).seconds() > timeout_sec_) {
    RCLCPP_ERROR(node_->get_logger(), "GoToAction: timeout after %.0f seconds", timeout_sec_);
    CancelGoal();
    setOutput("error_code", -2);
    return BT::NodeStatus::FAILURE;
  }

  // Wait for goal acceptance.
  if (!goal_accepted_) {
    if (goal_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    goal_handle_ = goal_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "GoToAction: goal rejected");
      setOutput("error_code", -3);
      return BT::NodeStatus::FAILURE;
    }
    goal_accepted_ = true;
    result_future_ = client_->async_get_result(goal_handle_);
    result_pending_ = true;
  }

  // Wait for result.
  if (result_pending_) {
    if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    auto result = result_future_.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "GoToAction: navigation succeeded");
      setOutput("error_code", 0);
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "GoToAction: navigation failed (code %d)",
                   static_cast<int>(result.code));
      setOutput("error_code", static_cast<int>(result.code));
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

/*!
 * \brief Cancel the active goal and release the goal handle.
 *
 * \note Called by the BT.CPP executor when the tree halts this node mid-flight.
 *       Safe to call even if no goal is active.
 */
void GoToAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "GoToAction: halted");
  CancelGoal();
}

/*******************************************************************************
 * Private Function Bodies
 ******************************************************************************/

/*!
 * \brief Asynchronously cancel the active navigation goal, if any.
 *
 * \note No-op when \c goal_handle_ is null or the goal has not been accepted.
 *       Cancel errors are logged as warnings and not propagated.
 */
void GoToAction::CancelGoal() {
  if (goal_handle_ && goal_accepted_) {
    try {
      client_->async_cancel_goal(goal_handle_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(), "GoToAction: cancel failed: %s", e.what());
    }
  }
}

}  // namespace defined_runtime

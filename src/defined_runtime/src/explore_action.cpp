/*!
 * \file explore_action.cpp
 * \brief Implementation of ExploreAction — controls frontier-based exploration.
 *
 * Lifecycle per BT activation:
 *   1. onStart:   create transient_local publisher + subscribers,
 *                 publish resume=true to wake explore_lite
 *   2. onRunning: wait for explore_started_ (status callback), then monitor
 *                 /map growth; SUCCESS on EXPLORATION_COMPLETE or map stale
 *   3. onHalted:  publish resume=false, tear down subscriptions
 *
 * Key design decisions:
 *   - transient_local QoS on /explore/resume so explore_lite gets the message
 *     even if it finishes initializing after ExploreAction publishes
 *   - Periodic re-publish of resume (every 3s) as belt-and-suspenders
 *   - Map staleness check gated on explore_started_ to avoid false completion
 */

#include "defined_runtime/explore_action.hpp"

#include <explore_lite_msgs/msg/explore_status.hpp>

namespace defined_runtime {

ExploreAction::ExploreAction(const std::string& name, const BT::NodeConfig& config,
                             rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
}

BT::PortsList ExploreAction::providedPorts() {
  return {
      BT::InputPort<double>("timeout", 300.0, "Max exploration time (seconds)"),
  };
}

BT::NodeStatus ExploreAction::onStart() {
  getInput("timeout", timeout_sec_);

  RCLCPP_INFO(node_->get_logger(), "ExploreAction: starting frontier exploration (timeout=%.0fs)",
              timeout_sec_);

  // Subscribe to map to track exploration progress
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { OnMapReceived(msg); });

  start_time_ = node_->now();
  last_map_change_ = node_->now();
  last_known_cells_ = 0;
  map_received_ = false;
  explore_done_ = false;
  explore_started_ = false;

  // Subscribe to explore_lite status to detect when it actually starts/finishes
  status_sub_ = node_->create_subscription<explore_lite_msgs::msg::ExploreStatus>(
      "/explore/status", rclcpp::QoS(10).transient_local(),
      [this](const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg) { OnExploreStatus(msg); });

  // Use transient_local so explore_lite gets the message even if it subscribes after us
  resume_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/explore/resume", rclcpp::QoS(1).transient_local());

  // Tell explore_lite to start exploring
  PublishResume(true);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExploreAction::onRunning() {
  auto elapsed = (node_->now() - start_time_).seconds();

  // Check timeout
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(), "ExploreAction: timeout after %.0f seconds", elapsed);
    PublishResume(false);
    map_sub_.reset();
    status_sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  // Check if explore_lite reported completion
  if (explore_done_) {
    RCLCPP_INFO(node_->get_logger(),
                "ExploreAction: explore_lite reported completion (%.0fs elapsed)", elapsed);
    map_sub_.reset();
    status_sub_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  // Don't check for completion until explore_lite has confirmed it's running
  if (!explore_started_) {
    // Re-publish resume periodically in case explore_lite missed the first one
    auto waiting = (node_->now() - start_time_).seconds();
    if (static_cast<int>(waiting) % 3 == 0) {
      PublishResume(true);
    }
    return BT::NodeStatus::RUNNING;
  }

  // Don't check for completion until we've received at least one map
  if (!map_received_) {
    return BT::NodeStatus::RUNNING;
  }

  // If the map hasn't changed significantly for stale_threshold_sec_, exploration is done
  auto stale_duration = (node_->now() - last_map_change_).seconds();
  if (stale_duration > stale_threshold_sec_) {
    RCLCPP_INFO(node_->get_logger(),
                "ExploreAction: map stable for %.0fs — exploration complete (%.0fs elapsed, %d known cells)",
                stale_duration, elapsed, last_known_cells_);
    PublishResume(false);
    map_sub_.reset();
    status_sub_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ExploreAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "ExploreAction: halted — stopping exploration");
  PublishResume(false);
  map_sub_.reset();
  status_sub_.reset();
}

void ExploreAction::PublishResume(bool resume) {
  auto msg = std_msgs::msg::Bool();
  msg.data = resume;
  resume_pub_->publish(msg);
}

void ExploreAction::OnMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Count known cells (not -1 / unknown)
  int known = 0;
  for (const auto& cell : msg->data) {
    if (cell >= 0) {
      known++;
    }
  }

  // If the map grew by more than 10 cells, it's still changing
  if (std::abs(known - last_known_cells_) > 10) {
    last_map_change_ = node_->now();
    last_known_cells_ = known;

    if (!map_received_) {
      RCLCPP_INFO(node_->get_logger(), "ExploreAction: first map received (%d known cells)", known);
      map_received_ = true;
    }
  }
}

void ExploreAction::OnExploreStatus(
    const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg) {
  if (msg->status == explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE) {
    RCLCPP_INFO(node_->get_logger(), "ExploreAction: received EXPLORATION_COMPLETE from explore_lite");
    explore_done_ = true;
  } else if (msg->status == explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED ||
             msg->status == explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS) {
    if (!explore_started_) {
      RCLCPP_INFO(node_->get_logger(), "ExploreAction: explore_lite confirmed running");
      explore_started_ = true;
      // Reset the stale timer now that exploration has actually begun
      last_map_change_ = node_->now();
    }
  }
}

}  // namespace defined_runtime

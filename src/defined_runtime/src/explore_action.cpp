/*!
 * \file explore_action.cpp
 * \brief Implementation of ExploreAction — monitors frontier-based exploration.
 *
 * Works alongside explore_lite (m-explore-ros2). The BT node starts/stops
 * exploration via /explore/resume and monitors the /map topic. When the map
 * stops changing (no new frontiers being explored), exploration is complete.
 */

#include "defined_runtime/explore_action.hpp"

namespace defined_runtime {

ExploreAction::ExploreAction(const std::string& name, const BT::NodeConfig& config,
                             rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
  // Create publisher for explore_lite resume/stop control
  resume_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/explore/resume", 10);
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
    return BT::NodeStatus::FAILURE;
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
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ExploreAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "ExploreAction: halted — stopping exploration");
  PublishResume(false);
  map_sub_.reset();
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

}  // namespace defined_runtime

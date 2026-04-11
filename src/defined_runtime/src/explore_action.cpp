/*!
 * \file explore_action.cpp
 * \brief Implementation of ExploreAction — controls frontier-based exploration.
 *
 * Lifecycle per BT activation:
 *   1. onStart:   reset state, create transient_local publisher + subscribers,
 *                 publish resume=true to wake explore_lite
 *   2. onRunning: wait for explore_started_ (status callback), then monitor
 *                 /map growth; SUCCESS on EXPLORATION_COMPLETE or map stale
 *   3. onHalted:  publish resume=false, tear down publisher and subscriptions
 *
 * Key design decisions:
 *   - transient_local QoS on /explore/resume so explore_lite gets the message
 *     even if it finishes initializing after ExploreAction publishes
 *   - Periodic re-publish of resume (once every 3s) as belt-and-suspenders
 *   - Map staleness check gated on explore_started_ to avoid false completion
 *   - State initialised before subscriptions are created (avoids stale
 *     transient_local delivery writing into uninitialised fields)
 */

#include "defined_runtime/explore_action.hpp"

namespace defined_runtime {

ExploreAction::ExploreAction(const std::string& name, const BT::NodeConfig& config,
                             rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {
}

BT::PortsList ExploreAction::providedPorts() {
  return {
      BT::InputPort<double>("timeout", 300.0, "Max exploration time (seconds)"),
      BT::InputPort<double>("stale_threshold", 30.0, "Seconds of no map change → exploration complete"),
  };
}

BT::NodeStatus ExploreAction::onStart() {
  getInput("timeout", timeout_sec_);
  getInput("stale_threshold", stale_threshold_sec_);

  RCLCPP_INFO(node_->get_logger(), "ExploreAction: starting frontier exploration (timeout=%.0fs)",
              timeout_sec_);

  // Initialise all state variables BEFORE creating subscriptions.
  // With transient_local QoS a cached message may be delivered on the first
  // spin_some; all fields must be in a known state before that happens.
  start_time_ = node_->now();
  last_resume_pub_time_ = node_->now();
  {
    std::lock_guard<std::mutex> lock(map_time_mutex_);
    last_map_change_ = node_->now();
  }
  last_known_cells_.store(0);
  map_received_.store(false);
  explore_done_.store(false);
  explore_started_.store(false);

  // Subscribe to map to track exploration progress.
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { OnMapReceived(msg); });

  // Subscribe to explore_lite status to detect when it actually starts/finishes.
  status_sub_ = node_->create_subscription<explore_lite_msgs::msg::ExploreStatus>(
      "/explore/status", rclcpp::QoS(10).transient_local(),
      [this](const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg) { OnExploreStatus(msg); });

  // Use transient_local so explore_lite gets the message even if it subscribes after us.
  resume_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/explore/resume", rclcpp::QoS(1).transient_local());

  // Tell explore_lite to start exploring.
  PublishResume(true);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExploreAction::onRunning() {
  auto elapsed = (node_->now() - start_time_).seconds();

  // Check timeout.
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(), "ExploreAction: timeout after %.0f seconds", elapsed);
    PublishResume(false);
    Cleanup();
    return BT::NodeStatus::FAILURE;
  }

  // Check if explore_lite reported completion.
  if (explore_done_.load()) {
    RCLCPP_INFO(node_->get_logger(),
                "ExploreAction: explore_lite reported completion (%.0fs elapsed)", elapsed);
    Cleanup();
    return BT::NodeStatus::SUCCESS;
  }

  // Don't check for completion until explore_lite has confirmed it's running.
  if (!explore_started_.load()) {
    // Re-publish resume once every 3s in case explore_lite missed the initial message.
    if ((node_->now() - last_resume_pub_time_).seconds() >= 3.0) {
      PublishResume(true);
    }
    return BT::NodeStatus::RUNNING;
  }

  // Don't check for map staleness until we've received at least one map.
  if (!map_received_.load()) {
    return BT::NodeStatus::RUNNING;
  }

  // If the map hasn't changed significantly for stale_threshold_sec_, exploration is done.
  double stale_duration;
  {
    std::lock_guard<std::mutex> lock(map_time_mutex_);
    stale_duration = (node_->now() - last_map_change_).seconds();
  }
  if (stale_duration > stale_threshold_sec_) {
    RCLCPP_INFO(node_->get_logger(),
                "ExploreAction: map stable for %.0fs — exploration complete (%.0fs elapsed, %d known cells)",
                stale_duration, elapsed, last_known_cells_.load());
    PublishResume(false);
    Cleanup();
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ExploreAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "ExploreAction: halted — stopping exploration");
  PublishResume(false);  // must precede Cleanup() which nulls resume_pub_
  Cleanup();
}

void ExploreAction::Cleanup() {
  resume_pub_.reset();
  map_sub_.reset();
  status_sub_.reset();
}

void ExploreAction::PublishResume(bool resume) {
  if (!resume_pub_) {
    return;
  }
  last_resume_pub_time_ = node_->now();
  auto msg = std_msgs::msg::Bool();
  msg.data = resume;
  resume_pub_->publish(msg);
}

void ExploreAction::OnMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Count known cells (not -1 / unknown).
  int known = 0;
  for (const auto& cell : msg->data) {
    if (cell >= 0) {
      known++;
    }
  }

  // Record first map receipt unconditionally so the staleness clock can start
  // regardless of how many cells the initial map contains.
  if (!map_received_.load()) {
    RCLCPP_INFO(node_->get_logger(), "ExploreAction: first map received (%d known cells)", known);
    map_received_.store(true);
    last_known_cells_.store(known);
    std::lock_guard<std::mutex> lock(map_time_mutex_);
    last_map_change_ = node_->now();
    return;
  }

  // Update the staleness clock whenever the map grows by more than 10 cells.
  if (std::abs(known - last_known_cells_.load()) > 10) {
    last_known_cells_.store(known);
    std::lock_guard<std::mutex> lock(map_time_mutex_);
    last_map_change_ = node_->now();
  }
}

void ExploreAction::OnExploreStatus(
    const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg) {
  if (msg->status == explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE) {
    RCLCPP_INFO(node_->get_logger(), "ExploreAction: received EXPLORATION_COMPLETE from explore_lite");
    explore_done_.store(true);
  } else if (msg->status == explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED ||
             msg->status == explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS) {
    if (!explore_started_.load()) {
      RCLCPP_INFO(node_->get_logger(), "ExploreAction: explore_lite confirmed running");
      explore_started_.store(true);
      // Reset the stale timer now that exploration has actually begun.
      std::lock_guard<std::mutex> lock(map_time_mutex_);
      last_map_change_ = node_->now();
    }
  }
}

}  // namespace defined_runtime

#ifndef DEFINED_RUNTIME_EXPLORE_ACTION_HPP
#define DEFINED_RUNTIME_EXPLORE_ACTION_HPP

/*!
 * \file explore_action.hpp
 * \brief BT.CPP action node that triggers and monitors autonomous exploration.
 *
 * Works with explore_lite (m-explore-ros2, patched) running as a standalone
 * node launched alongside the BT executor via bt_executor.launch.py.
 *
 * explore_lite is patched at Docker build time (see Dockerfile.ros2) with:
 *   - autostart=false: starts paused, waits for /explore/resume
 *   - blacklist clear on resume(): fresh frontier search each activation
 *
 * Completion is detected via two signals (whichever fires first):
 *   1. /explore/status → EXPLORATION_COMPLETE (explore_lite's own signal)
 *   2. /map staleness — no new cells for stale_threshold_sec_ (fallback)
 *
 * \see bt_executor.launch.py for explore_lite parameter tuning
 * \see Dockerfile.ros2 for the sed/python patches applied to explore_lite
 */

#include <string>

#include <behaviortree_cpp/action_node.h>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>

namespace defined_runtime {

/*!
 * \brief Asynchronous BT action node that monitors frontier-based exploration.
 *
 * Requires explore_lite to be running as a separate ROS2 node.
 * This node acts as a coordinator:
 *   - onStart: publishes True to /explore/resume to start exploration
 *   - onRunning: monitors /map for new data; considers exploration done
 *     when map hasn't changed for a sustained period (no new frontiers)
 *   - onHalted: publishes False to /explore/resume to stop exploration
 *
 * \par BT Ports
 * | Direction | Name             | Type   | Default | Description                                      |
 * |-----------|------------------|--------|---------|--------------------------------------------------|
 * | Input     | timeout          | double | 300.0   | Max exploration time (seconds)                   |
 * | Input     | stale_threshold  | double | 30.0    | Seconds of no map change → exploration complete  |
 *
 * \par ROS2 Topics
 * | Direction | Topic             | Type                  | QoS              |
 * |-----------|-------------------|-----------------------|------------------|
 * | Pub       | /explore/resume   | std_msgs/Bool         | transient_local  |
 * | Sub       | /explore/status   | ExploreStatus         | transient_local  |
 * | Sub       | /map              | OccupancyGrid         | transient_local  |
 */
class ExploreAction : public BT::StatefulActionNode {
 public:
  ExploreAction(const std::string& name, const BT::NodeConfig& config,
                rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_sub_;

  rclcpp::Time start_time_;
  double timeout_sec_{300.0};

  // Track map changes to detect exploration completion
  int last_known_cells_{0};         /*!< Count of known (non-unknown) cells in last map */
  rclcpp::Time last_map_change_;    /*!< Time when map last changed significantly */
  double stale_threshold_sec_{30.0}; /*!< Seconds of no map change → exploration done */
  bool map_received_{false};
  bool explore_done_{false};    /*!< Set when explore_lite reports EXPLORATION_COMPLETE */
  bool explore_started_{false}; /*!< Set when explore_lite confirms it's actively exploring */

  void PublishResume(bool resume);
  void OnMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void OnExploreStatus(const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg);
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_EXPLORE_ACTION_HPP

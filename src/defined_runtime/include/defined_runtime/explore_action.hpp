#ifndef DEFINED_RUNTIME_EXPLORE_ACTION_HPP
#define DEFINED_RUNTIME_EXPLORE_ACTION_HPP

/*!
 * \file explore_action.hpp
 * \brief BT.CPP action node that triggers and monitors autonomous exploration.
 *
 * Works with explore_lite (m-explore-ros2) running as a standalone node.
 * The ExploreAction publishes to /explore/resume to start exploration,
 * monitors the /map topic for coverage progress, and returns SUCCESS
 * when no new frontiers are found (exploration complete) or FAILURE
 * on timeout.
 */

#include <string>

#include <behaviortree_cpp/action_node.h>
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
 * | Direction | Name    | Type   | Default | Description                    |
 * |-----------|---------|--------|---------|--------------------------------|
 * | Input     | timeout | double | 300.0   | Max exploration time (seconds) |
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

  rclcpp::Time start_time_;
  double timeout_sec_{300.0};

  // Track map changes to detect exploration completion
  int last_known_cells_{0};         /*!< Count of known (non-unknown) cells in last map */
  rclcpp::Time last_map_change_;    /*!< Time when map last changed significantly */
  double stale_threshold_sec_{30.0}; /*!< Seconds of no map change → exploration done */
  bool map_received_{false};

  void PublishResume(bool resume);
  void OnMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_EXPLORE_ACTION_HPP

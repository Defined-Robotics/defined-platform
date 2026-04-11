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
 * \see Dockerfile.ros2 for the patches applied to explore_lite
 */

#include <atomic>
#include <mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

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

  /*!
   * \warning onStart() must have been called and returned RUNNING before this
   *          method is invoked.
   */
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_sub_;

  rclcpp::Time start_time_;
  rclcpp::Time last_resume_pub_time_;  /*!< Time of last /explore/resume publish (rate-limiting) */
  double timeout_sec_{300.0};
  double stale_threshold_sec_{30.0};  /*!< Seconds of no map change → exploration done */

  // Shared state between the BT tick thread (onRunning) and ROS2 subscription
  // callbacks (OnMapReceived, OnExploreStatus). Atomics for cheap scalar flags;
  // map_time_mutex_ guards last_map_change_ which is a non-trivial type.
  mutable std::mutex map_time_mutex_;
  rclcpp::Time last_map_change_;                 /*!< Time of last significant map update */
  std::atomic<int> last_known_cells_{0};         /*!< Known (non-unknown) cell count in last map */
  std::atomic<bool> map_received_{false};
  std::atomic<bool> explore_done_{false};        /*!< Set when explore_lite reports EXPLORATION_COMPLETE */
  std::atomic<bool> explore_started_{false};     /*!< Set when explore_lite confirms it's actively exploring */

  void PublishResume(bool resume);
  void Cleanup();  /*!< Resets publisher and subscriptions; safe to call from any exit path */
  void OnMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void OnExploreStatus(const explore_lite_msgs::msg::ExploreStatus::SharedPtr msg);
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_EXPLORE_ACTION_HPP

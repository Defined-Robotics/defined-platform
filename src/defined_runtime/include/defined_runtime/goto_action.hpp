#ifndef DEFINED_RUNTIME_GOTO_ACTION_HPP_
#define DEFINED_RUNTIME_GOTO_ACTION_HPP_

/*!
 * \file goto_action.hpp
 * \brief BT.CPP action node that sends a NavigateToPose goal to Nav2.
 */

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <behaviortree_cpp/action_node.h>

namespace defined_runtime {

/*!
 * \brief Asynchronous BT action node that drives the robot to a 2-D pose
 *        via the Nav2 NavigateToPose action server.
 *
 * Action clients are cached per \c server_name so that multiple GoTo nodes
 * targeting the same server share a single client instance.
 *
 * \par BT Ports
 * | Direction | Name        | Type   | Default              | Description                  |
 * |-----------|-------------|--------|----------------------|------------------------------|
 * | Input     | x           | double |                      | Target X coordinate (meters) |
 * | Input     | y           | double |                      | Target Y coordinate (meters) |
 * | Input     | theta       | double | 0.0                  | Target orientation (radians) |
 * | Input     | timeout     | double | 60.0                 | Navigation timeout (seconds) |
 * | Input     | frame_id    | string | "map"                | Reference frame              |
 * | Input     | server_name | string | "/navigate_to_pose"  | Nav2 action server name      |
 * | Output    | error_code  | int    |                      | Navigation error code        |
 */
class GoToAction : public BT::StatefulActionNode {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Client = rclcpp_action::Client<NavigateToPose>;

  /*!
   * \brief Construct a GoToAction.
   * \param name BT node instance name.
   * \param config BT node configuration (ports, blackboard).
   * \param node ROS2 node used for action client creation.
   */
  GoToAction(const std::string& name, const BT::NodeConfig& config,
             rclcpp::Node::SharedPtr node);

  /*!
   * \brief Declare the BT input/output ports.
   * \return Port list containing x, y, theta, timeout, frame_id,
   *         server_name, and error_code ports.
   */
  static BT::PortsList providedPorts();

  /*! \copydoc BT::StatefulActionNode::onStart */
  BT::NodeStatus onStart() override;

  /*! \copydoc BT::StatefulActionNode::onRunning */
  BT::NodeStatus onRunning() override;

  /*! \copydoc BT::StatefulActionNode::onHalted */
  void onHalted() override;

  /*!
   * \brief Convert a yaw angle to a quaternion (2-D rotation about Z).
   * \param[in]  theta Yaw angle in radians.
   * \param[out] qz    Quaternion Z component.
   * \param[out] qw    Quaternion W component.
   */
  static void ThetaToQuaternion(double theta, double& qz, double& qw);

 private:
  rclcpp::Node::SharedPtr node_;  /*!< ROS2 node for action client creation. */
  Client::SharedPtr client_;      /*!< Active action client. */

  std::shared_future<GoalHandle::SharedPtr> goal_future_;        /*!< Goal send future. */
  GoalHandle::SharedPtr goal_handle_;                            /*!< Active goal handle. */
  std::shared_future<GoalHandle::WrappedResult> result_future_;  /*!< Result future. */

  rclcpp::Time start_time_;       /*!< Timestamp when navigation started. */
  double timeout_sec_{60.0};      /*!< Navigation timeout in seconds. */
  bool goal_accepted_{false};     /*!< Whether the goal has been accepted. */
  bool result_pending_{false};    /*!< Whether we are waiting for a result. */

  /*!< Action client cache keyed by server name. */
  std::unordered_map<std::string, Client::SharedPtr> client_cache_;

  /*!
   * \brief Get or create an action client for the given server.
   * \param server_name Fully-qualified action server name.
   * \return Shared pointer to the action client.
   */
  Client::SharedPtr GetClient(const std::string& server_name);

  /*! \brief Cancel the active navigation goal, if any. */
  void CancelGoal();
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_GOTO_ACTION_HPP_

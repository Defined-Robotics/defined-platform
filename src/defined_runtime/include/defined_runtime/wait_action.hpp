#ifndef DEFINED_RUNTIME_WAIT_ACTION_HPP
#define DEFINED_RUNTIME_WAIT_ACTION_HPP

/*!
 * \file wait_action.hpp
 * \brief BT.CPP action node that waits for a specified duration.
 */

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>

namespace defined_runtime {

/*!
 * \brief BT action node that pauses execution for a configurable duration.
 *
 * Uses the ROS2 node clock (`node->now()`) so that `use_sim_time` is
 * respected automatically. A duration of zero or less completes immediately.
 *
 * \par BT Ports
 * | Direction | Name     | Type   | Description              |
 * |-----------|----------|--------|--------------------------|
 * | Input     | duration | double | Wait duration in seconds |
 */
class WaitAction : public BT::StatefulActionNode {
 public:
  /*! \brief Construct a WaitAction. */
  WaitAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  /*! \brief Declare the BT input/output port: duration. */
  static BT::PortsList providedPorts();

  /*! \copydoc BT::StatefulActionNode::onStart */
  BT::NodeStatus onStart() override;

  /*! \copydoc BT::StatefulActionNode::onRunning */
  BT::NodeStatus onRunning() override;

  /*! \copydoc BT::StatefulActionNode::onHalted */
  void onHalted() override;

 private:
  rclcpp::Node::SharedPtr node_; /*!< ROS2 node for clock access. */
  rclcpp::Time start_time_;      /*!< Timestamp when the wait began. */
  double duration_sec_{0.0};     /*!< Target wait duration in seconds. */
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_WAIT_ACTION_HPP

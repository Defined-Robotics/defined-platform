#ifndef DEFINED_RUNTIME_REPORT_ACTION_HPP
#define DEFINED_RUNTIME_REPORT_ACTION_HPP

/*!
 * \file report_action.hpp
 * \brief BT.CPP action node that publishes a report message to a ROS2 topic.
 */

#include <string>
#include <unordered_map>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace defined_runtime {

/*!
 * \brief Synchronous BT action node that publishes a string message and logs.
 *
 * Publishes a \c std_msgs/String to a configurable topic and logs the message
 * at the requested severity level. Publishers are cached per topic name so
 * repeated reports to the same topic reuse a single publisher.
 *
 * \par BT Ports
 * | Direction | Name    | Type   | Default         | Description                      |
 * |-----------|---------|--------|-----------------|----------------------------------|
 * | Input     | message | string | "checkpoint"    | Report message text              |
 * | Input     | topic   | string | "/task_reports" | ROS2 topic to publish to         |
 * | Input     | level   | string | "info"          | Log level: debug/info/warn/error |
 * | Output    | success | bool   |                 | Whether the report was published  |
 */
class ReportAction : public BT::SyncActionNode {
 public:
  /*! \brief Construct a ReportAction. */
  ReportAction(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node);

  /*! \brief Declare the BT input/output ports: message, topic, level, success. */
  static BT::PortsList providedPorts();

  /*! \copydoc BT::SyncActionNode::tick */
  BT::NodeStatus tick() override;

 private:
  rclcpp::Node::SharedPtr node_; /*!< ROS2 node for publishing and logging. */

  /*!< Publisher cache keyed by topic name. */
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_REPORT_ACTION_HPP

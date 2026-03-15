#include "defined_runtime/report_action.hpp"

namespace defined_runtime {

ReportAction::ReportAction(const std::string& name,
                           const BT::NodeConfig& config,
                           rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node) {}

BT::PortsList ReportAction::providedPorts() {
  return {
      BT::InputPort<std::string>("message", "checkpoint", "Report message"),
      BT::InputPort<std::string>("topic", "/task_reports",
                                 "ROS topic to publish to"),
      BT::InputPort<std::string>("level", "info",
                                 "Severity: debug, info, warn, error"),
      BT::OutputPort<bool>("success", "Whether report was published"),
  };
}

BT::NodeStatus ReportAction::tick() {
  std::string message, topic, level;
  getInput("message", message);
  getInput("topic", topic);
  getInput("level", level);

  // Get or create publisher.
  auto it = publishers_.find(topic);
  if (it == publishers_.end()) {
    auto pub = node_->create_publisher<std_msgs::msg::String>(topic, 10);
    publishers_[topic] = pub;
    it = publishers_.find(topic);
  }

  std_msgs::msg::String msg;
  msg.data = message;
  it->second->publish(msg);

  if (level == "debug") {
    RCLCPP_DEBUG(node_->get_logger(), "Report: %s", message.c_str());
  } else if (level == "warn") {
    RCLCPP_WARN(node_->get_logger(), "Report: %s", message.c_str());
  } else if (level == "error") {
    RCLCPP_ERROR(node_->get_logger(), "Report: %s", message.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "Report: %s", message.c_str());
  }

  setOutput("success", true);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace defined_runtime

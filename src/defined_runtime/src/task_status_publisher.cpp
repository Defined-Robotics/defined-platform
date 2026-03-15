#include "defined_runtime/task_status_publisher.hpp"

#include <sstream>

namespace defined_runtime {

TaskStatusPublisher::TaskStatusPublisher(rclcpp::Node::SharedPtr node,
                                         const std::string& topic) {
  pub_ = node->create_publisher<std_msgs::msg::String>(topic, 10);
}

std::string TaskStatusPublisher::ToJson(const std::string& step_name,
                                         const std::string& status,
                                         int current_step, int total_steps) {
  int progress = (total_steps > 0) ? (current_step * 100 / total_steps) : 0;
  std::ostringstream ss;
  ss << "{\"step\":\"" << step_name
     << "\",\"status\":\"" << status
     << "\",\"current\":" << current_step
     << ",\"total\":" << total_steps
     << ",\"progress\":" << progress << "}";
  return ss.str();
}

void TaskStatusPublisher::Publish(const std::string& step_name,
                                   const std::string& status,
                                   int current_step, int total_steps) {
  std_msgs::msg::String msg;
  msg.data = ToJson(step_name, status, current_step, total_steps);
  pub_->publish(msg);
}

}  // namespace defined_runtime

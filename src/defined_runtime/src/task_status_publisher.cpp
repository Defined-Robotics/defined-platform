/*!
 * \file task_status_publisher.cpp
 * \brief Implementation of TaskStatusPublisher — publishes JSON task progress.
 */

#include <sstream>

#include "defined_runtime/task_status_publisher.hpp"

namespace defined_runtime {

TaskStatusPublisher::TaskStatusPublisher(rclcpp::Node::SharedPtr node, const std::string& topic) {
  pub_ = node->create_publisher<std_msgs::msg::String>(topic, 10);
}

/*!
 * \brief Serialise step progress into a compact JSON string.
 *
 * \param[in] step_name    Name of the currently executing BT leaf node.
 * \param[in] status       Execution status string (e.g., "RUNNING", "SUCCESS", "IDLE").
 * \param[in] current_step Zero-based index of the running leaf among all leaves.
 * \param[in] total_steps  Total number of leaf action/condition nodes in the tree.
 *
 * \retval std::string  JSON object: \c {"step":…,"status":…,"current":…,"total":…,"progress":…}
 *
 * \note \c progress is integer percent (0–100). When \c total_steps is zero the
 *       progress field is 0 to avoid division by zero.
 */
std::string TaskStatusPublisher::ToJson(const std::string& step_name, const std::string& status,
                                        int current_step, int total_steps) {
  int progress = (total_steps > 0) ? (current_step * 100 / total_steps) : 0;
  std::ostringstream ss;
  ss << "{\"step\":\"" << step_name << "\",\"status\":\"" << status
     << "\",\"current\":" << current_step << ",\"total\":" << total_steps
     << ",\"progress\":" << progress << "}";
  return ss.str();
}

/*!
 * \brief Build a JSON status string via ToJson() and publish it on the topic.
 *
 * \param[in] step_name    Name of the currently executing BT leaf node.
 * \param[in] status       Execution status string.
 * \param[in] current_step Zero-based running leaf index.
 * \param[in] total_steps  Total leaf count in the tree.
 *
 * \retval *  Forwarded from ToJson() — result is published, not returned.
 *
 * \note This method is a thin wrapper around ToJson(); all serialisation logic
 *       lives in ToJson() to keep it independently testable.
 */
void TaskStatusPublisher::Publish(const std::string& step_name, const std::string& status,
                                  int current_step, int total_steps) {
  std_msgs::msg::String msg;
  msg.data = ToJson(step_name, status, current_step, total_steps);
  pub_->publish(msg);
}

}  // namespace defined_runtime

#ifndef DEFINED_RUNTIME_TASK_STATUS_PUBLISHER_HPP_
#define DEFINED_RUNTIME_TASK_STATUS_PUBLISHER_HPP_

/*!
 * \file task_status_publisher.hpp
 * \brief Publishes JSON task-execution status to a ROS2 topic.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace defined_runtime {

/*!
 * \brief Publishes task execution progress as JSON on a ROS2 topic.
 *
 * Each message is a JSON object:
 * \code{.json}
 * {"step":"go_to_1","status":"RUNNING","current":2,"total":8,"progress":25}
 * \endcode
 */
class TaskStatusPublisher {
 public:
  /*!
   * \brief Construct a TaskStatusPublisher.
   * \param node  ROS2 node used for publisher creation.
   * \param topic Topic name to publish on (default: "/task_status").
   */
  TaskStatusPublisher(rclcpp::Node::SharedPtr node, const std::string& topic = "/task_status");

  /*!
   * \brief Publish a status update.
   * \param step_name   Name of the currently executing BT node.
   * \param status      Execution status string (RUNNING, SUCCESS, FAILURE).
   * \param current_step  Current step index (0-based).
   * \param total_steps   Total number of leaf action nodes in the tree.
   */
  void Publish(const std::string& step_name, const std::string& status, int current_step,
               int total_steps);

  /*!
   * \brief Build a JSON status string without publishing.
   * \param step_name   Name of the currently executing BT node.
   * \param status      Execution status string.
   * \param current_step  Current step index.
   * \param total_steps   Total number of leaf action nodes.
   * \return JSON-formatted status string.
   */
  static std::string ToJson(const std::string& step_name, const std::string& status,
                            int current_step, int total_steps);

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; /*!< ROS2 publisher. */
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_TASK_STATUS_PUBLISHER_HPP_

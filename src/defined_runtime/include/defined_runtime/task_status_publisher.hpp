#ifndef DEFINED_RUNTIME_TASK_STATUS_PUBLISHER_HPP
#define DEFINED_RUNTIME_TASK_STATUS_PUBLISHER_HPP

/*!
 * \file task_status_publisher.hpp
 * \brief Publishes JSON task-execution status to a ROS2 topic.
 */

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
  /*! \brief Construct a TaskStatusPublisher on the given node and topic. */
  TaskStatusPublisher(rclcpp::Node::SharedPtr node, const std::string& topic = "/task_status");

  /*! \brief Publish a JSON status update for the currently executing step. */
  void Publish(const std::string& step_name, const std::string& status, int current_step,
               int total_steps);

  /*! \brief Build a JSON status string without publishing. */
  static std::string ToJson(const std::string& step_name, const std::string& status,
                            int current_step, int total_steps);

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; /*!< ROS2 publisher. */
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_TASK_STATUS_PUBLISHER_HPP

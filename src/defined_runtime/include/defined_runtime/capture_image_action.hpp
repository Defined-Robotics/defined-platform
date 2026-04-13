#ifndef DEFINED_RUNTIME_CAPTURE_IMAGE_ACTION_HPP
#define DEFINED_RUNTIME_CAPTURE_IMAGE_ACTION_HPP

/*!
 * \file capture_image_action.hpp
 * \brief BT.CPP action node that captures a single image from a camera topic.
 */

#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace defined_runtime {

/*!
 * \brief Stateful BT action node that subscribes to a camera topic,
 *        saves one frame to disk, and reports the file path.
 *
 * On start, creates a subscription to the configured camera topic.
 * On each tick, checks if a frame has arrived. Once received, saves
 * the raw image data to the configured path and publishes the path
 * to ``/task_reports``.
 *
 * \par BT Ports
 * | Direction | Name      | Type   | Default              | Description                |
 * |-----------|-----------|--------|----------------------|----------------------------|
 * | Input     | topic     | string | /camera/image_raw    | Camera topic to subscribe  |
 * | Input     | save_path | string | /tmp/capture         | Directory to save images   |
 * | Input     | timeout   | double | 5.0                  | Seconds to wait for frame  |
 * | Output    | file_path | string |                      | Path to saved image file   |
 */
class CaptureImageAction : public BT::StatefulActionNode {
 public:
  /*! \brief Construct with BT config and a shared ROS2 node. */
  CaptureImageAction(const std::string& name, const BT::NodeConfig& config,
                     rclcpp::Node::SharedPtr node);

  /*! \brief Register BT ports: topic, save_path, timeout (input), file_path (output). */
  static BT::PortsList providedPorts();

  /*! \brief Subscribe to the camera topic and start waiting for a frame. */
  BT::NodeStatus onStart() override;
  /*! \brief Check for received frame; save to disk on arrival or fail on timeout. */
  BT::NodeStatus onRunning() override;
  /*! \brief Cancel the subscription if the node is halted mid-capture. */
  void onHalted() override;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_pub_;

  sensor_msgs::msg::Image::SharedPtr received_image_;
  rclcpp::Time start_time_;
  double timeout_sec_{5.0};
  std::string save_dir_;
  bool image_received_{false};
};

}  // namespace defined_runtime

#endif  // DEFINED_RUNTIME_CAPTURE_IMAGE_ACTION_HPP

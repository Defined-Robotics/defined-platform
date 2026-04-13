/*!
 * \file capture_image_action.cpp
 * \brief Implementation of CaptureImageAction — captures a single camera frame.
 */

#include "defined_runtime/capture_image_action.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace defined_runtime {

CaptureImageAction::CaptureImageAction(const std::string& name,
                                       const BT::NodeConfig& config,
                                       rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node) {}

/*!
 * \brief Register BT input/output ports for the CaptureImage action.
 *
 * \retval BT::PortsList  Four ports: topic, save_path, timeout (input),
 *                         file_path (output).
 */
BT::PortsList CaptureImageAction::providedPorts() {
  return {
      BT::InputPort<std::string>("topic", "/camera/image_raw", "Camera topic"),
      BT::InputPort<std::string>("save_path", "/tmp/capture", "Save directory"),
      BT::InputPort<double>("timeout", 5.0, "Timeout in seconds"),
      BT::OutputPort<std::string>("file_path", "Path to saved image"),
  };
}

/*!
 * \brief Subscribe to the camera topic and begin waiting for a frame.
 *
 * Creates the save directory if it does not exist, initialises a
 * ``/task_reports`` publisher (reused across ticks), and subscribes
 * to the configured image topic with SensorDataQoS.
 *
 * \retval BT::NodeStatus::RUNNING  Always — the actual capture happens in onRunning().
 */
BT::NodeStatus CaptureImageAction::onStart() {
  std::string topic;
  getInput("topic", topic);
  getInput("save_path", save_dir_);
  getInput("timeout", timeout_sec_);

  // Validate topic is not empty.
  if (topic.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "CaptureImage: topic port is empty");
    return BT::NodeStatus::FAILURE;
  }

  image_received_ = false;
  received_image_ = nullptr;
  start_time_ = node_->now();

  // Create save directory if needed.
  try {
    std::filesystem::create_directories(save_dir_);
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_ERROR(node_->get_logger(), "CaptureImage: cannot create save_path '%s': %s",
                 save_dir_.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Create report publisher (reused across ticks).
  if (!report_pub_) {
    report_pub_ = node_->create_publisher<std_msgs::msg::String>("/task_reports", 10);
  }

  // Subscribe to camera topic.
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) {
        if (!image_received_) {
          received_image_ = msg;
          image_received_ = true;
        }
      });

  RCLCPP_INFO(node_->get_logger(), "CaptureImage: waiting for frame on %s", topic.c_str());
  return BT::NodeStatus::RUNNING;
}

/*!
 * \brief Poll for a received frame; save to disk or fail on timeout.
 *
 * If the timeout elapses before a frame arrives, resets the subscription
 * and returns FAILURE.  When a frame is received, writes the raw byte
 * data to ``<save_dir>/capture_<nanoseconds>.raw``, publishes the path
 * on ``/task_reports``, and sets the ``file_path`` output port.
 *
 * \retval BT::NodeStatus::RUNNING  Still waiting for a frame.
 * \retval BT::NodeStatus::SUCCESS  Frame captured and saved.
 * \retval BT::NodeStatus::FAILURE  Timeout elapsed or file write error.
 */
BT::NodeStatus CaptureImageAction::onRunning() {
  // Check timeout.
  double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(), "CaptureImage: timeout after %.1fs", elapsed);
    image_sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  if (!image_received_) {
    return BT::NodeStatus::RUNNING;
  }

  // Save image data to file.
  auto timestamp = node_->now().nanoseconds();
  std::ostringstream filename;
  filename << save_dir_ << "/capture_" << timestamp << ".raw";
  std::string file_path = filename.str();

  std::ofstream out(file_path, std::ios::binary);
  if (!out.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "CaptureImage: failed to open %s", file_path.c_str());
    image_sub_.reset();
    return BT::NodeStatus::FAILURE;
  }

  out.write(reinterpret_cast<const char*>(received_image_->data.data()),
            static_cast<std::streamsize>(received_image_->data.size()));
  out.close();

  RCLCPP_INFO(node_->get_logger(), "CaptureImage: saved %zu bytes to %s",
              received_image_->data.size(), file_path.c_str());

  // Report the file path.
  std_msgs::msg::String report_msg;
  report_msg.data = "captured:" + file_path;
  report_pub_->publish(report_msg);

  setOutput("file_path", file_path);
  image_sub_.reset();
  return BT::NodeStatus::SUCCESS;
}

/*!
 * \brief Clean up the image subscription when the node is halted.
 */
void CaptureImageAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "CaptureImage: halted");
  image_sub_.reset();
}

}  // namespace defined_runtime

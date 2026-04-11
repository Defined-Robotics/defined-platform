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

BT::PortsList CaptureImageAction::providedPorts() {
  return {
      BT::InputPort<std::string>("topic", "/camera/image_raw", "Camera topic"),
      BT::InputPort<std::string>("save_path", "/tmp/capture", "Save directory"),
      BT::InputPort<double>("timeout", 5.0, "Timeout in seconds"),
      BT::OutputPort<std::string>("file_path", "Path to saved image"),
  };
}

BT::NodeStatus CaptureImageAction::onStart() {
  std::string topic;
  getInput("topic", topic);
  getInput("save_path", save_dir_);
  getInput("timeout", timeout_sec_);

  image_received_ = false;
  received_image_ = nullptr;
  start_time_ = node_->now();

  // Create save directory if needed.
  std::filesystem::create_directories(save_dir_);

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

void CaptureImageAction::onHalted() {
  RCLCPP_INFO(node_->get_logger(), "CaptureImage: halted");
  image_sub_.reset();
}

}  // namespace defined_runtime

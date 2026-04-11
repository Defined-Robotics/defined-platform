#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "defined_runtime/capture_image_action.hpp"

class CaptureImageActionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_capture_image");

    factory_.registerBuilder<defined_runtime::CaptureImageAction>(
        "CaptureImage",
        [this](const std::string& name, const BT::NodeConfig& cfg) {
          return std::make_unique<defined_runtime::CaptureImageAction>(
              name, cfg, node_);
        });
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(CaptureImageActionTest, RegistersInFactory) {
  // Verify the node can be instantiated from the factory.
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="CaptureImage" name="cap"
                topic="/test_camera" save_path="/tmp/test_capture"
                timeout="0.5" />
      </BehaviorTree>
    </root>)";

  // Should not throw — node is registered and XML is valid.
  EXPECT_NO_THROW(factory_.createTreeFromText(xml));
}

TEST_F(CaptureImageActionTest, ReturnsRunningWhenNoImage) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="CaptureImage" name="cap"
                topic="/test_no_image" save_path="/tmp/test_capture"
                timeout="0.5" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);

  // First tick starts subscription, returns RUNNING.
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

TEST_F(CaptureImageActionTest, TimesOutWithoutImage) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="CaptureImage" name="cap"
                topic="/test_timeout" save_path="/tmp/test_capture"
                timeout="0.1" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  tree.tickOnce();  // Start

  // Wait past timeout.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(CaptureImageActionTest, CapturesImageAndSaves) {
  std::string save_dir = "/tmp/test_capture_save";
  std::filesystem::remove_all(save_dir);

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="CaptureImage" name="cap"
                topic="/test_capture_topic" save_path=")" +
                     save_dir + R"(" timeout="2.0" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  tree.tickOnce();  // Start — subscribes

  // Publish a fake image.
  auto pub = node_->create_publisher<sensor_msgs::msg::Image>("/test_capture_topic", 10);
  sensor_msgs::msg::Image img;
  img.width = 2;
  img.height = 2;
  img.encoding = "rgb8";
  img.step = 6;
  img.data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

  // Give subscription time to set up.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node_);
  pub->publish(img);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node_);

  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // Verify file was saved.
  bool found = false;
  for (auto& entry : std::filesystem::directory_iterator(save_dir)) {
    if (entry.path().extension() == ".raw") {
      found = true;
      auto size = std::filesystem::file_size(entry.path());
      EXPECT_EQ(size, 12u);
    }
  }
  EXPECT_TRUE(found) << "No .raw file found in " << save_dir;

  // Cleanup.
  std::filesystem::remove_all(save_dir);
}

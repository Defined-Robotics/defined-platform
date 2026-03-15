#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "defined_runtime/task_status_publisher.hpp"

class TaskStatusPublisherTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_status");
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskStatusPublisherTest, JsonFormat) {
  auto json =
      defined_runtime::TaskStatusPublisher::ToJson("go_to_1", "RUNNING", 2, 8);
  EXPECT_NE(json.find("\"step\":\"go_to_1\""), std::string::npos);
  EXPECT_NE(json.find("\"status\":\"RUNNING\""), std::string::npos);
  EXPECT_NE(json.find("\"current\":2"), std::string::npos);
  EXPECT_NE(json.find("\"total\":8"), std::string::npos);
  EXPECT_NE(json.find("\"progress\":25"), std::string::npos);
}

TEST_F(TaskStatusPublisherTest, ProgressMath) {
  auto json =
      defined_runtime::TaskStatusPublisher::ToJson("step", "RUNNING", 3, 8);
  EXPECT_NE(json.find("\"progress\":37"), std::string::npos);

  auto json2 =
      defined_runtime::TaskStatusPublisher::ToJson("step", "RUNNING", 0, 8);
  EXPECT_NE(json2.find("\"progress\":0"), std::string::npos);

  auto json3 =
      defined_runtime::TaskStatusPublisher::ToJson("step", "SUCCESS", 8, 8);
  EXPECT_NE(json3.find("\"progress\":100"), std::string::npos);
}

TEST_F(TaskStatusPublisherTest, PublishesToTopic) {
  auto pub = std::make_shared<defined_runtime::TaskStatusPublisher>(
      node_, "/test_task_status");

  std::string received;
  auto sub = node_->create_subscription<std_msgs::msg::String>(
      "/test_task_status", 10,
      [&received](const std_msgs::msg::String::SharedPtr msg) {
        received = msg->data;
      });

  pub->Publish("go_to", "RUNNING", 1, 4);
  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node_);

  EXPECT_NE(received.find("\"step\":\"go_to\""), std::string::npos);
}

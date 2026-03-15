#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "defined_runtime/report_action.hpp"

class ReportActionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_report");

    factory_.registerBuilder<defined_runtime::ReportAction>(
        "Report",
        [this](const std::string& name, const BT::NodeConfig& cfg) {
          return std::make_unique<defined_runtime::ReportAction>(
              name, cfg, node_);
        });
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(ReportActionTest, PublishesMessage) {
  std::string received;
  auto sub = node_->create_subscription<std_msgs::msg::String>(
      "/test_reports", 10,
      [&received](const std_msgs::msg::String::SharedPtr msg) {
        received = msg->data;
      });

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Report" name="r" message="hello"
                topic="/test_reports" level="info" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rclcpp::spin_some(node_);
  EXPECT_EQ(received, "hello");
}

TEST_F(ReportActionTest, CachesPublisher) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Sequence>
          <Action ID="Report" name="r1" message="first"
                  topic="/cached_topic" level="info" />
          <Action ID="Report" name="r2" message="second"
                  topic="/cached_topic" level="info" />
        </Sequence>
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ReportActionTest, LogLevels) {
  for (const auto& level : {"debug", "info", "warn", "error"}) {
    std::string xml =
        R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)"
        R"(<Action ID="Report" name="r" message="test")"
        R"( topic="/log_test" level=")" +
        std::string(level) +
        R"(" /></BehaviorTree></root>)";

    auto tree = factory_.createTreeFromText(xml);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  }
}

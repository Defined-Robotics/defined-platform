#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "defined_runtime/wait_action.hpp"

class WaitActionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_wait");

    factory_.registerBuilder<defined_runtime::WaitAction>(
        "Wait",
        [this](const std::string& name, const BT::NodeConfig& cfg) {
          return std::make_unique<defined_runtime::WaitAction>(
              name, cfg, node_);
        });
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(WaitActionTest, ZeroDurationCompletesImmediately) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Wait" name="w" duration="0.0" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(WaitActionTest, ReturnsRunningThenSuccess) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Wait" name="w" duration="0.2" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(WaitActionTest, HaltedMidWait) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Wait" name="w" duration="10.0" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  tree.haltTree();
}

TEST_F(WaitActionTest, PortParsing) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Wait" name="w" duration="5.5" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
}

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "defined_runtime/explore_action.hpp"

class ExploreActionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_explore");

    factory_.registerBuilder<defined_runtime::ExploreAction>(
        "Explore",
        [this](const std::string& name, const BT::NodeConfig& cfg) {
          return std::make_unique<defined_runtime::ExploreAction>(name, cfg, node_);
        });
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(ExploreActionTest, ProvidedPortsIncludeTimeoutAndStaleThreshold) {
  auto ports = defined_runtime::ExploreAction::providedPorts();
  EXPECT_TRUE(ports.count("timeout") > 0);
  EXPECT_TRUE(ports.count("stale_threshold") > 0);
}

TEST_F(ExploreActionTest, OnStartReturnsRunning) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Explore" name="e" timeout="300.0" stale_threshold="30.0" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  tree.haltTree();
}

TEST_F(ExploreActionTest, HaltedAfterStartDoesNotCrash) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Explore" name="e" timeout="300.0" stale_threshold="30.0" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  tree.haltTree();
  // haltTree must not crash; no assertion needed beyond reaching this line
}

TEST_F(ExploreActionTest, TimeoutCausesFailure) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Explore" name="e" timeout="0.1" stale_threshold="30.0" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ExploreActionTest, CanBeReusedAfterTimeout) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="Explore" name="e" timeout="0.1" stale_threshold="30.0" />
      </BehaviorTree>
    </root>)";

  // First activation — timeout → FAILURE
  auto tree = factory_.createTreeFromText(xml);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Re-activation — must return RUNNING (not crash on re-entered onStart)
  auto tree2 = factory_.createTreeFromText(xml);
  EXPECT_EQ(tree2.tickOnce(), BT::NodeStatus::RUNNING);
  tree2.haltTree();
}

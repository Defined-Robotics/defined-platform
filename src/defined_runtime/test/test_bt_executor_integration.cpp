#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "defined_runtime/goto_action.hpp"
#include "defined_runtime/report_action.hpp"
#include "defined_runtime/wait_action.hpp"

class BtExecutorIntegrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_integration");

    factory_.registerBuilder<defined_runtime::GoToAction>(
        "GoTo",
        [this](const std::string& name, const BT::NodeConfig& cfg) {
          return std::make_unique<defined_runtime::GoToAction>(
              name, cfg, node_);
        });
    factory_.registerBuilder<defined_runtime::WaitAction>(
        "Wait",
        [this](const std::string& name, const BT::NodeConfig& cfg) {
          return std::make_unique<defined_runtime::WaitAction>(
              name, cfg, node_);
        });
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

TEST_F(BtExecutorIntegrationTest, LoadsTreeFromXml) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Sequence>
          <Action ID="Wait" name="w" duration="0.0" />
          <Action ID="Report" name="r" message="done"
                  topic="/int_test" level="info" />
        </Sequence>
      </BehaviorTree>
    </root>)";

  EXPECT_NO_THROW({
    auto tree = factory_.createTreeFromText(xml);
  });
}

TEST_F(BtExecutorIntegrationTest, WaitReportTreeTicksToSuccess) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Sequence>
          <Action ID="Wait" name="w" duration="0.0" />
          <Action ID="Report" name="r" message="done"
                  topic="/int_test2" level="info" />
        </Sequence>
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BtExecutorIntegrationTest, AllThreeNodeTypesRegistered) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Sequence>
          <Action ID="GoTo" name="g" x="0" y="0" theta="0"
                  frame_id="map" server_name="/nav" timeout="1.0" />
          <Action ID="Wait" name="w" duration="0.0" />
          <Action ID="Report" name="r" message="ok"
                  topic="/t" level="info" />
        </Sequence>
      </BehaviorTree>
    </root>)";

  EXPECT_NO_THROW({
    auto tree = factory_.createTreeFromText(xml);
  });
}

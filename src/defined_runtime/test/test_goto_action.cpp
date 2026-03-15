#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include "defined_runtime/goto_action.hpp"

class GoToActionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_goto");

    factory_.registerBuilder<defined_runtime::GoToAction>(
        "GoTo",
        [this](const std::string& name, const BT::NodeConfig& cfg) {
          return std::make_unique<defined_runtime::GoToAction>(
              name, cfg, node_);
        });
  }

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(GoToActionTest, QuaternionConversionZero) {
  double qz, qw;
  defined_runtime::GoToAction::ThetaToQuaternion(0.0, qz, qw);
  EXPECT_NEAR(qz, 0.0, 1e-6);
  EXPECT_NEAR(qw, 1.0, 1e-6);
}

TEST_F(GoToActionTest, QuaternionConversionPiOver2) {
  double qz, qw;
  defined_runtime::GoToAction::ThetaToQuaternion(M_PI / 2.0, qz, qw);
  EXPECT_NEAR(qz, 0.7071068, 1e-5);
  EXPECT_NEAR(qw, 0.7071068, 1e-5);
}

TEST_F(GoToActionTest, QuaternionConversionPi) {
  double qz, qw;
  defined_runtime::GoToAction::ThetaToQuaternion(M_PI, qz, qw);
  EXPECT_NEAR(qz, 1.0, 1e-5);
  EXPECT_NEAR(qw, 0.0, 1e-5);
}

TEST_F(GoToActionTest, TimeoutWhenNoServer) {
  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="GoTo" name="g" x="1.0" y="2.0" theta="0.0"
                frame_id="map" server_name="/nonexistent_server"
                timeout="1.0" />
      </BehaviorTree>
    </root>)";

  auto tree = factory_.createTreeFromText(xml);
  auto status = tree.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

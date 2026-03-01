/**
 * BT Executor Node — Loads compiled BehaviorTree XML and ticks the tree.
 *
 * This is the main runtime for Defined Robotics. It:
 * 1. Loads a BT XML file (compiled by the verb compiler)
 * 2. Registers custom BT action nodes (GoTo, Wait, Report)
 * 3. Ticks the tree at a configurable rate
 * 4. Publishes task status to /task_status
 *
 * TODO: Implement in Phase 3
 */

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_executor");
    RCLCPP_INFO(node->get_logger(), "BT Executor node started (placeholder)");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

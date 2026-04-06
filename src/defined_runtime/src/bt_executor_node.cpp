/*!
 * \file bt_executor_node.cpp
 * \brief BT Executor Node -- loads compiled BehaviorTree XML and ticks the tree.
 *
 * Registers custom BT action nodes (GoTo, Wait, Report), ticks the tree
 * at a configurable rate, and publishes task status to `/task_status`.
 * New tasks can be loaded at runtime via `/task_command` (`std_msgs/String`).
 *
 * \par ROS2 Parameters
 * | Name        | Type   | Default | Description                        |
 * |-------------|--------|---------|------------------------------------|
 * | bt_xml_path | string | ""      | Initial BT XML file to load        |
 * | tick_rate   | double | 100.0   | BT tick frequency in Hz            |
 * | enable_groot| bool   | true    | Enable Groot2 ZMQ on port 1667     |
 *
 * \par Subscriptions
 * - `/task_command` (`std_msgs/String`) -- file path to a BT XML to load.
 *
 * \par Published Topics
 * - `/task_status` (`std_msgs/String`) -- JSON progress updates.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "defined_runtime/goto_action.hpp"
#include "defined_runtime/report_action.hpp"
#include "defined_runtime/task_status_publisher.hpp"
#include "defined_runtime/wait_action.hpp"

/*******************************************************************************
 * Public Function Bodies
 ******************************************************************************/

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_executor");

  node->declare_parameter("bt_xml_path", std::string(""));
  node->declare_parameter("tick_rate", 100.0);
  node->declare_parameter("enable_groot", true);

  // Register BT nodes.
  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<defined_runtime::GoToAction>(
      "GoTo", [node](const std::string& name, const BT::NodeConfig& cfg) {
        return std::make_unique<defined_runtime::GoToAction>(name, cfg, node);
      });

  factory.registerBuilder<defined_runtime::WaitAction>(
      "Wait", [node](const std::string& name, const BT::NodeConfig& cfg) {
        return std::make_unique<defined_runtime::WaitAction>(name, cfg, node);
      });

  factory.registerBuilder<defined_runtime::ReportAction>(
      "Report", [node](const std::string& name, const BT::NodeConfig& cfg) {
        return std::make_unique<defined_runtime::ReportAction>(name, cfg, node);
      });

  RCLCPP_INFO(node->get_logger(), "Registered BT nodes: GoTo, Wait, Report");

  // Task status publisher.
  auto status_pub = std::make_shared<defined_runtime::TaskStatusPublisher>(node);

  // Tree state.
  std::unique_ptr<BT::Tree> tree;
  std::unique_ptr<BT::Groot2Publisher> groot_pub;
  bool tree_loaded = false;
  bool tree_done = false;
  int total_leaves = 0;
  int idle_counter = 0;

  // Lambda to load a tree from file.
  auto load_tree = [&](const std::string& xml_path) {
    RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_path.c_str());
    try {
      tree = std::make_unique<BT::Tree>(factory.createTreeFromFile(xml_path));
      tree_loaded = true;
      tree_done = false;

      bool enable_groot = node->get_parameter("enable_groot").as_bool();
      if (enable_groot) {
        groot_pub = std::make_unique<BT::Groot2Publisher>(*tree, 1667);
        RCLCPP_INFO(node->get_logger(), "Groot2 publisher on port 1667");
      }

      // Count leaf nodes once (tree structure is immutable after load).
      total_leaves = 0;
      tree->applyVisitor([&total_leaves](const BT::TreeNode* tn) {
        if (tn->type() == BT::NodeType::ACTION || tn->type() == BT::NodeType::CONDITION) {
          total_leaves++;
        }
      });

      RCLCPP_INFO(node->get_logger(), "Tree loaded (%d leaf nodes), starting execution",
                  total_leaves);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to load BT XML: %s", e.what());
      tree_loaded = false;
    }
  };

  // Subscribe to /task_command.
  auto task_cmd_sub = node->create_subscription<std_msgs::msg::String>(
      "/task_command", 10,
      [&load_tree](const std_msgs::msg::String::SharedPtr msg) { load_tree(msg->data); });

  // Load initial tree if parameter set.
  auto xml_path = node->get_parameter("bt_xml_path").as_string();
  if (!xml_path.empty()) {
    load_tree(xml_path);
  } else {
    RCLCPP_INFO(node->get_logger(), "No bt_xml_path set, waiting for /task_command");
  }

  // Tick loop.
  double tick_rate = node->get_parameter("tick_rate").as_double();
  rclcpp::Rate rate(tick_rate);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (!tree_loaded || tree_done) {
      // Publish IDLE heartbeat at ~1Hz so the CLI knows the executor is ready.
      if (++idle_counter >= static_cast<int>(tick_rate)) {
        status_pub->Publish("none", "IDLE", 0, 0);
        idle_counter = 0;
      }
    }

    if (tree_loaded && !tree_done) {
      auto status = tree->tickOnce();

      // Find the currently running leaf and its index among all leaves.
      // BT.CPP resets completed nodes to IDLE, so we can't count SUCCESS.
      // Instead, find the running leaf's position — everything before it is done.
      std::string current_node_name = "unknown";
      int running_leaf_index = -1;
      int leaf_index = 0;
      tree->applyVisitor([&](const BT::TreeNode* tn) {
        bool is_leaf = (tn->type() == BT::NodeType::ACTION ||
                        tn->type() == BT::NodeType::CONDITION);
        if (tn->status() == BT::NodeStatus::RUNNING) {
          current_node_name = tn->name();
          if (is_leaf) {
            running_leaf_index = leaf_index;
          }
        }
        if (is_leaf) {
          leaf_index++;
        }
      });
      // completed = index of running leaf (everything before it is done).
      // On final SUCCESS no node is RUNNING so completed = total.
      // On FAILURE use running_leaf_index if available, else 0.
      int completed = (running_leaf_index >= 0)
                          ? running_leaf_index
                          : (status == BT::NodeStatus::SUCCESS ? total_leaves : 0);

      std::string status_str;
      switch (status) {
        case BT::NodeStatus::RUNNING:
          status_str = "RUNNING";
          break;
        case BT::NodeStatus::SUCCESS:
          status_str = "SUCCESS";
          tree_done = true;
          RCLCPP_INFO(node->get_logger(), "Tree completed: SUCCESS");
          break;
        case BT::NodeStatus::FAILURE:
          status_str = "FAILURE";
          tree_done = true;
          RCLCPP_WARN(node->get_logger(),
                      "Tree completed: FAILURE (Goal Manager stub: not restarting)");
          break;
        default:
          status_str = "IDLE";
          break;
      }

      status_pub->Publish(current_node_name, status_str, completed, total_leaves);

      if (tree_done) {
        groot_pub.reset();
        idle_counter = 0;  // reset so heartbeat timing is fresh after tree completes
        RCLCPP_INFO(node->get_logger(), "Waiting for next /task_command");
      }
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

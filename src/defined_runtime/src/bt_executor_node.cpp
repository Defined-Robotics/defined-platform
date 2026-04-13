/*!
 * \file bt_executor_node.cpp
 * \brief BT Executor Node -- loads compiled BehaviorTree XML and ticks the tree.
 *
 * Registers custom BT action nodes (GoTo, Wait, Report, Explore, CaptureImage),
 * ticks the tree at a configurable rate, and publishes task status to `/task_status`.
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

#include "defined_runtime/capture_image_action.hpp"
#include "defined_runtime/explore_action.hpp"
#include "defined_runtime/goto_action.hpp"
#include "defined_runtime/report_action.hpp"
#include "defined_runtime/task_status_publisher.hpp"
#include "defined_runtime/wait_action.hpp"

/*******************************************************************************
 * Private Data
 ******************************************************************************/

/*! \brief Groot2 ZMQ publisher port. */
static constexpr int kGroot2Port = 1667;

/*!
 * \brief Mutable state for the currently loaded behaviour tree.
 *
 * Grouped into a struct so tick_tree() can operate on a single reference
 * instead of a long parameter list.
 */
struct TreeState {
  std::unique_ptr<BT::Tree> tree;
  std::unique_ptr<BT::Groot2Publisher> groot_pub;
  bool loaded = false;
  bool done = false;
  int total_leaves = 0;
  int idle_counter = 0;
};

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void register_bt_nodes(BT::BehaviorTreeFactory& factory,
                              const rclcpp::Node::SharedPtr& node);

static void load_tree(TreeState& state,
                      BT::BehaviorTreeFactory& factory,
                      const rclcpp::Node::SharedPtr& node,
                      const std::string& xml_path);

static void tick_tree(TreeState& state,
                      const rclcpp::Node::SharedPtr& node,
                      const std::shared_ptr<defined_runtime::TaskStatusPublisher>& status_pub);

/*******************************************************************************
 * Private Function Bodies
 ******************************************************************************/

/*!
 * \brief Register all custom BT action nodes with the factory.
 *
 * Each action node receives a shared pointer to the ROS2 node so it can
 * create publishers, subscriptions, and action clients.
 *
 * \param[in,out] factory  BT.CPP factory to register nodes with.
 * \param[in]     node     Shared ROS2 node passed to each action constructor.
 */
static void register_bt_nodes(BT::BehaviorTreeFactory& factory,
                              const rclcpp::Node::SharedPtr& node) {
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

  factory.registerBuilder<defined_runtime::ExploreAction>(
      "Explore", [node](const std::string& name, const BT::NodeConfig& cfg) {
        return std::make_unique<defined_runtime::ExploreAction>(name, cfg, node);
      });

  factory.registerBuilder<defined_runtime::CaptureImageAction>(
      "CaptureImage", [node](const std::string& name, const BT::NodeConfig& cfg) {
        return std::make_unique<defined_runtime::CaptureImageAction>(name, cfg, node);
      });

  RCLCPP_INFO(node->get_logger(),
              "Registered BT nodes: GoTo, Wait, Report, Explore, CaptureImage");
}

/*!
 * \brief Load a BT XML file into the tree state.
 *
 * Replaces any previously loaded tree. Optionally attaches a Groot2
 * ZMQ publisher for live introspection.  Counts leaf nodes once so
 * tick_tree() can report completion progress without re-scanning.
 *
 * \param[in,out] state    Tree state to populate.
 * \param[in]     factory  Factory with registered action nodes.
 * \param[in]     node     ROS2 node (for parameters and logging).
 * \param[in]     xml_path Absolute path to the BT XML file.
 */
static void load_tree(TreeState& state,
                      BT::BehaviorTreeFactory& factory,
                      const rclcpp::Node::SharedPtr& node,
                      const std::string& xml_path) {
  RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_path.c_str());
  try {
    state.tree = std::make_unique<BT::Tree>(factory.createTreeFromFile(xml_path));
    state.loaded = true;
    state.done = false;

    const bool enable_groot = node->get_parameter("enable_groot").as_bool();
    if (enable_groot) {
      state.groot_pub = std::make_unique<BT::Groot2Publisher>(*state.tree, kGroot2Port);
      RCLCPP_INFO(node->get_logger(), "Groot2 publisher on port %d", kGroot2Port);
    }

    // Count leaf nodes once (tree structure is immutable after load).
    state.total_leaves = 0;
    state.tree->applyVisitor([&state](const BT::TreeNode* tn) {
      if (tn->type() == BT::NodeType::ACTION || tn->type() == BT::NodeType::CONDITION) {
        state.total_leaves++;
      }
    });

    RCLCPP_INFO(node->get_logger(), "Tree loaded (%d leaf nodes), starting execution",
                state.total_leaves);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load BT XML: %s", e.what());
    state.loaded = false;
  }
}

/*!
 * \brief Tick the behaviour tree once and publish status.
 *
 * Finds the currently running leaf to determine completion progress,
 * translates the BT status to a string, and publishes via the
 * TaskStatusPublisher.  On tree completion (SUCCESS or FAILURE),
 * resets the Groot publisher and transitions to idle.
 *
 * \param[in,out] state       Tree state (updated on completion).
 * \param[in]     node        ROS2 node (for logging).
 * \param[in]     status_pub  Publisher for JSON task status messages.
 */
static void tick_tree(TreeState& state,
                      const rclcpp::Node::SharedPtr& node,
                      const std::shared_ptr<defined_runtime::TaskStatusPublisher>& status_pub) {
  const auto status = state.tree->tickOnce();

  // Find the currently running leaf and its index among all leaves.
  // BT.CPP resets completed nodes to IDLE, so we can't count SUCCESS.
  // Instead, find the running leaf's position — everything before it is done.
  std::string current_node_name = "unknown";
  int running_leaf_index = -1;
  int leaf_index = 0;
  state.tree->applyVisitor([&](const BT::TreeNode* tn) {
    const bool is_leaf = (tn->type() == BT::NodeType::ACTION ||
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
  const int completed = (running_leaf_index >= 0)
                            ? running_leaf_index
                            : (status == BT::NodeStatus::SUCCESS ? state.total_leaves : 0);

  std::string status_str;
  switch (status) {
    case BT::NodeStatus::RUNNING:
      status_str = "RUNNING";
      break;
    case BT::NodeStatus::SUCCESS:
      status_str = "SUCCESS";
      state.done = true;
      RCLCPP_INFO(node->get_logger(), "Tree completed: SUCCESS");
      break;
    case BT::NodeStatus::FAILURE:
      status_str = "FAILURE";
      state.done = true;
      RCLCPP_WARN(node->get_logger(),
                  "Tree completed: FAILURE (Goal Manager stub: not restarting)");
      break;
    default:
      status_str = "IDLE";
      break;
  }

  status_pub->Publish(current_node_name, status_str, completed, state.total_leaves);

  if (state.done) {
    state.groot_pub.reset();
    state.idle_counter = 0;
    RCLCPP_INFO(node->get_logger(), "Waiting for next /task_command");
  }
}

/*******************************************************************************
 * Public Function Bodies
 ******************************************************************************/

/*!
 * \brief Entry point — init ROS2, register BT nodes, run tick loop.
 *
 * \param[in] argc  Argument count (forwarded to rclcpp::init).
 * \param[in] argv  Argument values (forwarded to rclcpp::init).
 *
 * \retval 0  Always (clean shutdown).
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_executor");

  node->declare_parameter("bt_xml_path", std::string(""));
  node->declare_parameter("tick_rate", 100.0);
  node->declare_parameter("enable_groot", true);

  BT::BehaviorTreeFactory factory;
  register_bt_nodes(factory, node);

  auto status_pub = std::make_shared<defined_runtime::TaskStatusPublisher>(node);
  TreeState state;

  // Subscribe to /task_command.
  auto task_cmd_sub = node->create_subscription<std_msgs::msg::String>(
      "/task_command", 10,
      [&state, &factory, &node, &status_pub](const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "STOP") {
          RCLCPP_WARN(node->get_logger(), "STOP received — halting tree");
          if (state.loaded && state.tree && !state.done) {
            state.tree->haltTree();
            state.done = true;
            state.groot_pub.reset();
            status_pub->Publish("ESTOP", "FAILURE", state.total_leaves, state.total_leaves);
          }
          return;
        }
        load_tree(state, factory, node, msg->data);
      });

  // Load initial tree if parameter set.
  const auto xml_path = node->get_parameter("bt_xml_path").as_string();
  if (!xml_path.empty()) {
    load_tree(state, factory, node, xml_path);
  } else {
    RCLCPP_INFO(node->get_logger(), "No bt_xml_path set, waiting for /task_command");
  }

  // Tick loop.
  const double tick_rate = node->get_parameter("tick_rate").as_double();
  rclcpp::Rate rate(tick_rate);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (!state.loaded || state.done) {
      // Publish IDLE heartbeat at ~1Hz so the CLI knows the executor is ready.
      if (++state.idle_counter >= static_cast<int>(tick_rate)) {
        status_pub->Publish("none", "IDLE", 0, 0);
        state.idle_counter = 0;
      }
    }

    if (state.loaded && !state.done) {
      tick_tree(state, node, status_pub);
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

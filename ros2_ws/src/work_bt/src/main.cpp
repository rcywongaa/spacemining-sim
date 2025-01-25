#include <functional>
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// #include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rover_interfaces/action/navigate_to_position.hpp"
#include "work_bt/action/do_work.hpp"
#include "work_bt/action/empty_action.hpp"

#include "sleep_bt_node.hpp"
#include "simple_ros_action_node.hpp"

using DoWork = work_bt::action::DoWork;
using EmptyAction = work_bt::action::EmptyAction;
using NavigateToPosition = rover_interfaces::action::NavigateToPosition;
using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * @brief We do not use TreeExecutionServer because we have complex cancellation logic.
 * We follow TreeExecutionServer in keeping a member Node instead of inheriting from Node
 *
 */
class ExcavatorWork {
public:
  ExcavatorWork() : node(std::make_shared<rclcpp::Node>("excavator_bt")) {
    setup(factory);
    node->declare_parameter("bt_xml", std::string("my_tree.xml"));

    is_working = false;
    is_cancelling = false;

    std::string bt_xml = node->get_parameter("bt_xml").as_string();
    factory.registerBehaviorTreeFromFile(bt_xml);

    bt_server = rclcpp_action::create_server<DoWork>(
        node, "do_work", std::bind(&ExcavatorWork::handle_goal, this, _1, _2),
        std::bind(&ExcavatorWork::handle_cancel, this, _1),
        std::bind(&ExcavatorWork::handle_accepted, this, _1)
    );
  }

  std::shared_ptr<rclcpp::Node> get_node() {
    return node;
  }

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const DoWork::Goal> goal) {
    if (active_goal_handle) {
      RCLCPP_WARN(node->get_logger(), "Received work goal but already working");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(node->get_logger(), "Received new work goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoWork>> goal_handle) {
    std::scoped_lock<std::mutex> lock(mtx);
    RCLCPP_INFO(node->get_logger(), "Received cancel request");
    if (!active_goal_handle) {
      RCLCPP_WARN(node->get_logger(), "No active goal, rejecting cancel request");
      return rclcpp_action::CancelResponse::REJECT;
    } else if (active_goal_handle != goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Received cancel request for different goal, this shouldn't happen...");
      return rclcpp_action::CancelResponse::REJECT;
    } else if (is_cancelling) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Already cancelling, accepting duplicate cancel request");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    assert(active_goal_handle == goal_handle);
    is_cancelling = true;
    std::thread(&ExcavatorWork::cancel, this).detach();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoWork>> goal_handle) {
    RCLCPP_INFO(node->get_logger(), "Start executing goal...");
    std::scoped_lock<std::mutex> lock(mtx);
    active_goal_handle = goal_handle;
    is_working = true;
    is_cancelling = false;
    std::thread(&ExcavatorWork::run_work_bt, this).detach();
  }

  void run_work_bt() {
    /* Create tree here instead of constructor since we want the tree to always start from the beginning */
    auto work_bt = factory.createTree("ExcavatorBT");
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      {
        std::scoped_lock<std::mutex> lock(mtx);
        if (is_cancelling) {
          is_working = false;
          work_bt.haltTree();
          return;
        }
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Ticking work_bt");
        if (work_bt.tickOnce() != BT::NodeStatus::RUNNING) {
          return;
        }
      }
      rate.sleep();
    }
  }

  void cancel() {
    auto stow_bt = factory.createTree("StowBT");
    rclcpp::Rate rate(10);
    // Wait for work_bt to stop
    while (rclcpp::ok() && is_working) {
      rate.sleep();
    }
    while (rclcpp::ok()) {
      {
        std::scoped_lock<std::mutex> lock(mtx);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Ticking stow_bt");
        if (stow_bt.tickOnce() != BT::NodeStatus::RUNNING) {
          is_cancelling = false;
          /* A successful cancel is treated as a succeeded goal */
          // active_goal_handle->canceled({});
          return;
        }
      }
      rate.sleep();
    }
  }

  void send_result(work_bt::msg::WorkResult::_result_type result) {
    auto msg = std::make_shared<DoWork::Result>();
    msg->result.result = result;
    active_goal_handle->succeed(msg);
    active_goal_handle.reset();
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp_action::Server<DoWork>::SharedPtr bt_server;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<DoWork>> active_goal_handle;

  std::mutex mtx;
  std::atomic_bool is_working;
  std::atomic_bool is_cancelling;

  /********** BT Functions **********/
  BT::NodeStatus SendSuccess(BT::TreeNode &self) {
    RCLCPP_INFO(node->get_logger(), "SendSuccess");
    send_result(work_bt::msg::WorkResult::SUCCESS);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus Fail(BT::TreeNode &self) {
    RCLCPP_INFO(node->get_logger(), "Fail");
    send_result(work_bt::msg::WorkResult::FAILURE);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus CriticalFail(BT::TreeNode &self) {
    RCLCPP_INFO(node->get_logger(), "CriticalFail");
    send_result(work_bt::msg::WorkResult::CRITICAL_FAILURE);
    return BT::NodeStatus::FAILURE;
  }

  void setup(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<SimpleRosActionNode<NavigateToPosition>>("GoToWorksite", node, "navigate_to_position", []() {
      auto goal = NavigateToPosition::Goal();
      goal.target.position.latitude = 80.0;
      goal.target.position.longitude = 80.0;
      return goal;
    });
    factory.registerNodeType<SimpleRosActionNode<EmptyAction>>("AssumeReadyPose", node, "assume_ready_pose", []() {
      return EmptyAction::Goal();
    });
    factory.registerNodeType<SimpleRosActionNode<EmptyAction>>("Excavate", node, "excavate", []() {
      return EmptyAction::Goal();
    });
    factory.registerNodeType<SimpleRosActionNode<EmptyAction>>("Stow", node, "stow", []() {
      return EmptyAction::Goal();
    });
    factory.registerSimpleAction("SendSuccess", std::bind(&ExcavatorWork::SendSuccess, this, _1));
    factory.registerSimpleAction("Fail", std::bind(&ExcavatorWork::Fail, this, _1));
    factory.registerSimpleAction("CriticalFail", std::bind(&ExcavatorWork::CriticalFail, this, _1));
  }

  BT::BehaviorTreeFactory factory;
  /********************/

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto excavator_work = std::make_shared<ExcavatorWork>();
  rclcpp::spin(excavator_work->get_node());
  rclcpp::shutdown();
  return 0;

}

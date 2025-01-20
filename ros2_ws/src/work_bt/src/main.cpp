#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// #include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "work_bt/action/do_work.hpp"

using DoWork = work_bt::action::DoWork;
using namespace std::placeholders;
using namespace std::chrono_literals;

class SleepBtNode : public BT::StatefulActionNode {
public:
  SleepBtNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node& node, std::chrono::milliseconds duration)
      : BT::StatefulActionNode(name, config), node(node), duration(duration) {
  }

  BT::NodeStatus onStart() override {
    start_time = std::chrono::system_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    RCLCPP_INFO_THROTTLE(node.get_logger(), *node.get_clock(), 500, "Running %s", this->name().c_str());
    if (std::chrono::system_clock::now() - start_time > duration) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  void onHalted() override {}

  /* Required implementation */
  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  rclcpp::Node& node;
  std::chrono::system_clock::time_point start_time;
  std::chrono::milliseconds duration;
};

class ExcavatorWork : public rclcpp::Node {
public:
  ExcavatorWork() : Node("excavator_bt") {
    BT::BehaviorTreeFactory factory;
    setup(factory);
    this->declare_parameter("bt_xml", std::string("my_tree.xml"));

    is_working = false;
    is_cancelling = false;

    std::string bt_xml = this->get_parameter("bt_xml").as_string();
    // work_bt = factory.createTreeFromFile(bt_xml);
    factory.registerBehaviorTreeFromFile(bt_xml);
    work_bt = factory.createTree("ExcavatorBT");
    stow_bt = factory.createTree("StowBT");

    bt_server = rclcpp_action::create_server<DoWork>(
        this, "do_work", std::bind(&ExcavatorWork::handle_goal, this, _1, _2),
        std::bind(&ExcavatorWork::handle_cancel, this, _1),
        std::bind(&ExcavatorWork::handle_accepted, this, _1)
    );
  }

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                   std::shared_ptr<const DoWork::Goal> goal) {
    if (active_goal_handle) {
      RCLCPP_WARN(this->get_logger(), "Received work goal but already working");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Received new work goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoWork>> goal_handle) {
    std::scoped_lock<std::mutex> lock(mtx);
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    if (!active_goal_handle) {
      RCLCPP_WARN(this->get_logger(), "No active goal, rejecting cancel request");
      return rclcpp_action::CancelResponse::REJECT;
    } else if (active_goal_handle != goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Received cancel request for different goal, this shouldn't happen...");
      return rclcpp_action::CancelResponse::REJECT;
    } else if (is_cancelling) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Already cancelling, accepting duplicate cancel request");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    assert(active_goal_handle == goal_handle);
    is_working = false;
    is_cancelling = true;
    std::thread(&ExcavatorWork::cancel, this).detach();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoWork>> goal_handle) {
    std::scoped_lock<std::mutex> lock(mtx);
    active_goal_handle = goal_handle;
    is_working = true;
    is_cancelling = false;
    std::thread(&ExcavatorWork::run_work_bt, this).detach();
  }

  void run_work_bt() {
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && work_bt.tickOnce() == BT::NodeStatus::RUNNING) {
      rate.sleep();
    }
  }

  void cancel() {
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && stow_bt.tickOnce() == BT::NodeStatus::RUNNING) {
      rate.sleep();
    }
    active_goal_handle->canceled({});
    active_goal_handle.reset();
  }

  void send_result(work_bt::msg::WorkResult::_result_type result) {
    auto msg = std::make_shared<DoWork::Result>();
    msg->result.result = result;
    active_goal_handle->succeed(msg);
    active_goal_handle.reset();
  }

  rclcpp_action::Server<DoWork>::SharedPtr bt_server;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<DoWork>> active_goal_handle;

  std::mutex mtx;
  BT::Tree work_bt;
  std::atomic_bool is_working;
  BT::Tree stow_bt;
  std::atomic_bool is_cancelling;

  /********** BT Functions **********/
  BT::NodeStatus SendSuccess(BT::TreeNode &self) {
    RCLCPP_INFO(this->get_logger(), "SendSuccess");
    send_result(work_bt::msg::WorkResult::SUCCESS);
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus Fail(BT::TreeNode &self) {
    RCLCPP_INFO(this->get_logger(), "Fail");
    send_result(work_bt::msg::WorkResult::FAILURE);
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus CriticalFail(BT::TreeNode &self) {
    RCLCPP_INFO(this->get_logger(), "CriticalFail");
    send_result(work_bt::msg::WorkResult::CRITICAL_FAILURE);
    return BT::NodeStatus::FAILURE;
  }

  void setup(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<SleepBtNode>("GoToWorksite", std::ref(*this), 5000ms);
    factory.registerNodeType<SleepBtNode>("AssumeReadyPose", std::ref(*this), 2000ms);
    factory.registerNodeType<SleepBtNode>("Excavate", std::ref(*this), 10000ms);
    factory.registerNodeType<SleepBtNode>("Stow", std::ref(*this), 2000ms);
    factory.registerSimpleAction("SendSuccess", std::bind(&ExcavatorWork::SendSuccess, this, _1));
    factory.registerSimpleAction("Fail", std::bind(&ExcavatorWork::Fail, this, _1));
    factory.registerSimpleAction("CriticalFail", std::bind(&ExcavatorWork::CriticalFail, this, _1));
  }
  /********************/

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavatorWork>());
  rclcpp::shutdown();
  return 0;

}

#include <functional>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "../fsm.hpp"

using namespace fsm;

template<typename ActionType>
class SucceedingResultActionServer : public rclcpp::Node {


public:
  SucceedingResultActionServer(const std::string& name, std::string action_server_name, std::function<typename ActionType::Result()> result_generator)
      : Node(name), result_generator(result_generator) {
    this->action_server = rclcpp_action::create_server<ActionType>(
        this, action_server_name, std::bind(&SucceedingResultActionServer::handle_goal, this, _1, _2),
        std::bind(&SucceedingResultActionServer::handle_cancel, this, _1),
        std::bind(&SucceedingResultActionServer::handle_accepted, this, _1));

  }

private:
  std::function<typename ActionType::Result()> result_generator;
  typename rclcpp_action::Server<ActionType>::SharedPtr action_server;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const typename ActionType::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request: %d", goal);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle) {
    goal_handle->succeed(std::make_shared<DoWork::Result>(result_generator()));
  }
};

void detach_and_spin(std::shared_ptr<rclcpp::Node> node) {
  std::thread{[node]() {
    rclcpp::spin(node);
  }}.detach();
}

TEST(FsmTest, critical_state_on_critical_failure) {
  auto result_generator = []() {
    DoWork::Result result;
    result.result.result = work_bt::msg::WorkResult::CRITICAL_FAILURE;
    return result;
  };
  auto server = std::make_shared<SucceedingResultActionServer<work_bt::action::DoWork>>("mock_do_work_server", "/do_work", result_generator);
  detach_and_spin(server);

  auto node = std::make_shared<rclcpp::Node>("rover_fsm");
	Context context{node};
	FSM::Instance machine{context};
  std::thread([node](){rclcpp::spin(node);}).detach();

  EXPECT_TRUE(machine.isActive<LsWorking>());
  for (auto start_time = std::chrono::system_clock::now(); std::chrono::system_clock::now() - start_time < std::chrono::seconds(5);) {
    machine.update();
  }
  EXPECT_TRUE(machine.isActive<LsCritical>());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

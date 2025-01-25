#pragma once

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "arm/action/arm.hpp"

const unsigned int STOW_TIME_s = 2;
const unsigned int READY_TIME_s = 5;
const unsigned int EXCAVATE_TIME_s = 10;

class DummyArmActionServer : public rclcpp::Node
{
public:
  using Arm = arm::action::Arm;
  using ArmGoalHandle = rclcpp_action::ServerGoalHandle<Arm>;

  DummyArmActionServer()
      : Node("dummy_arm_action_server") {
    using namespace std::placeholders;
    this->action_server = rclcpp_action::create_server<Arm>(
        this, "arm_server", std::bind(&DummyArmActionServer::handle_goal, this, _1, _2),
        std::bind(&DummyArmActionServer::handle_cancel, this, _1),
        std::bind(&DummyArmActionServer::handle_accepted, this, _1));
    is_active = false;
  }

private:
  rclcpp_action::Server<Arm>::SharedPtr action_server;
  std::atomic_bool is_active;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Arm::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request: %d",
                goal->motion);
    if (is_active) {
      RCLCPP_WARN(this->get_logger(), "Goal rejected: server is busy");
      return rclcpp_action::GoalResponse::REJECT;
    } else {
      is_active = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<ArmGoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<ArmGoalHandle> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&DummyArmActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<ArmGoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Executing goal: %d", goal->motion);
    rclcpp::Rate loop_rate(5);
    auto result = std::make_shared<Arm::Result>();

    int wait_time = 0;
    switch (goal->motion) {
      case Arm::Goal::STOW:
        RCLCPP_INFO(this->get_logger(), "Stowing arm...");
        wait_time = STOW_TIME_s;
        break;
      case Arm::Goal::READY_POSE:
        RCLCPP_INFO(this->get_logger(), "Readying arm...");
        wait_time = READY_TIME_s;
        break;
      case Arm::Goal::EXCAVATE:
        RCLCPP_INFO(this->get_logger(), "Excavating...");
        wait_time = EXCAVATE_TIME_s;
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown motion request");
        break;
    }
    rclcpp::Time deadline = get_clock()->now() + rclcpp::Duration::from_seconds(wait_time);
    while(rclcpp::ok() && get_clock()->now() < deadline) {
      if(goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        is_active = false;
        return;
      }
      loop_rate.sleep();
    }

    // Check if goal is done
    if(rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      is_active = false;
    }
  }
};

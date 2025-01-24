#pragma once
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "work_bt/action/bt_action_client.hpp"

template<typename ActionType>
class ActionClientBtNode : public BT::StatefulActionNode {

public:
  ActionClientBtNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node& node, std::string action_server_name, std::function<typename ActionType::Goal()> goal_generator)
      : BT::StatefulActionNode(name, config), node(node), goal_generator(goal_generator) {
    action_client = rclcpp_action::create_client<ActionType>(&node, action_server_name);
    while (rclcpp::ok() && !action_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(node.get_logger(), "Waiting for %s to appear...", action_server_name.c_str());
    }
  }

  BT::NodeStatus onStart() override {
    is_done = false;
    auto goal_msg = ActionType::Goal();
    auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&ActionClientBtNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&ActionClientBtNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ActionClientBtNode::result_callback, this, std::placeholders::_1);
    action_client->async_send_goal(goal_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (is_done) {
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
  void goal_response_callback(const rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node.get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node.get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr,
      const std::shared_ptr<const typename ActionType::Feedback> feedback) {
    //TODO
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult &result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node.get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node.get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node.get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(node.get_logger(), "Unknown result code");
        break;
    }
    is_done = true;
  }

  rclcpp::Node& node;
  rclcpp_action::Client<ActionType>::SharedPtr action_client;
  std::atomic_bool is_done;
  std::function<typename ActionType::Goal()> goal_generator;
};

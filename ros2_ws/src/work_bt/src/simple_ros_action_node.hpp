#pragma once
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "work_bt/action/bt_action_client.hpp"

template<typename ActionType>
class SimpleRosActionNode : public BT::RosActionNode<ActionType> {
using RosActionNode = BT::RosActionNode<ActionType>;
public:
  SimpleRosActionNode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node, std::string action_server_name, std::function<typename ActionType::Goal()> goal_generator)
      : RosActionNode(name, config, BT::RosNodeParams(node, action_server_name)), node(node), goal_generator(goal_generator) {
  }

  bool setGoal(typename RosActionNode::Goal& goal) override {
    goal = goal_generator();
    return true;
  }

  BT::NodeStatus onResultReceived(const typename RosActionNode::WrappedResult &result) override {
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp::Node> node;
  std::function<typename ActionType::Goal()> goal_generator;
};

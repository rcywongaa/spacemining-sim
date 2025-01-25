#pragma once

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"

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


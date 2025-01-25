#include "rclcpp/rclcpp.hpp"

#include "dummy_arm_action_server.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyArmActionServer>();

  rclcpp::spin(node);

  return 0;
}

#include "fsm.hpp"

using namespace fsm;
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
	// shared data storage instance
  auto node = std::make_shared<rclcpp::Node>("rover_fsm");
	Context context{node};

	FSM::Instance machine{context};

  /* This needs to be a separate thread since machine.update() may block */
  std::thread([node](){rclcpp::spin(node);}).detach();

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
      machine.update();
      rate.sleep();
  }

	return 0;
}

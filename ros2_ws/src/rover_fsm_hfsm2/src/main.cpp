#include <chrono>
#include <thread>
#include <memory>
#include <functional>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "hfsm2/machine.hpp"

#include "rover_interfaces/msg/geo_pose2_d.hpp"
#include "rover_interfaces/srv/send_nav_target.hpp"
#include "rover_interfaces/action/navigate_to_position.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::placeholders;
using NavigateToPosition = rover_interfaces::action::NavigateToPosition;
using GeoPose2D = rover_interfaces::msg::GeoPose2D;

/********** Define State Machine ***********/
struct Context {
  /**
   * Context must be copyable.
   * Using shared_ptr to follow ROS convention.
   */
  std::shared_ptr<rclcpp::Node> node;
};
using Payload = std::variant<GeoPose2D>;
using Config = hfsm2::Config::ContextT<Context>::PayloadT<Payload>;
// using Config = hfsm2::Config::ContextT<Context>;
using M = hfsm2::MachineT<Config>;

#define S(s) struct s

/** Abbreviations
  * Ls - Leaf State
  * Ss - Super State
  * Ev - Event
  * Cb - Callback
  */

/* Top most leaf state is the initial state */
/* clang-format off */
using FSM = M::PeerRoot <
              M::Composite<S(SsAutonomous),
                  S(LsWorking),
                  S(LsCharging)
              >,
              M::Composite<S(SsManual),
                  S(LsManualTask),
                  S(LsIdle)
              >,
              S(LsSleep)
            >;
/* clang-format on */

#undef S
/*********************/

struct SsManual : FSM::State {
  void enter(Control &control) {
    printf("Start manual...\n");
  }

  void update(FullControl &control) {
  }
};

const unsigned int MANUAL_TIMEOUT_SEC = 10;
struct LsIdle : FSM::State {
  void enter(Control &_) {
    printf("Finished task, idling...\n");
    start_time = std::chrono::system_clock::now();
  }

  void update(FullControl &control) {
    if (std::chrono::system_clock::now() - start_time > std::chrono::seconds(MANUAL_TIMEOUT_SEC)) {
      printf("No input for %u secs, switching to autonomous\n", MANUAL_TIMEOUT_SEC);
      control.changeTo<SsAutonomous>();
    }
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start_time;
};

struct LsWorking : FSM::State {
  void enter(Control &control) {
    printf("Start working...\n");
    start_time = std::chrono::system_clock::now();
  }

  void update(FullControl &control) {
    if (std::chrono::system_clock::now() - start_time > std::chrono::seconds(3)) {
      printf("Low battery, switching to charging\n");
      control.changeTo<LsCharging>();
    }
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start_time;
};

struct LsCharging : FSM::State {
  void enter(Control &control) {
    printf("Start charging...\n");
    start_time = std::chrono::system_clock::now();
  }

  void update(FullControl &control) {
    if (std::chrono::system_clock::now() - start_time > std::chrono::seconds(2)) {
      printf("Fully charged, switching to working\n");
      control.changeTo<LsWorking>();
    }
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start_time;
};

struct SsAutonomous : FSM::State {

  void enter(Control &control) {
    printf("Start autonomous...\n");
    node = control.context().node;
    target_pose = {};
    battery_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", 1, std::bind(&SsAutonomous::battery_cb, this, std::placeholders::_1));

    nav_target_service = node->create_service<rover_interfaces::srv::SendNavTarget>(
        "send_nav_target", std::bind(&SsAutonomous::nav_target_cb, this, _1, _2));

  }

  void update(FullControl &control) {
    if (battery_msg) {
      if (battery_msg->percentage < 0.2) {
        printf("Low battery, switching to charging\n");
        control.changeTo<LsCharging>();
      }
    }
    if (target_pose) {
      printf("Navigating to target...\n");
      control.changeWith<LsManualTask>(*target_pose);
    }
  }

private:
  void battery_cb(sensor_msgs::msg::BatteryState::SharedPtr msg) {
    RCLCPP_DEBUG_THROTTLE(node->get_logger(), *(node->get_clock()), 1000, "Received battery msg: %f", msg->percentage);
    battery_msg = *msg;
  }

  void nav_target_cb(
      const std::shared_ptr<rover_interfaces::srv::SendNavTarget::Request> request,
      std::shared_ptr<rover_interfaces::srv::SendNavTarget::Response> response) {
    target_pose = request->target;
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
  std::optional<sensor_msgs::msg::BatteryState> battery_msg;

  rclcpp::Service<rover_interfaces::srv::SendNavTarget>::SharedPtr nav_target_service;
  std::optional<GeoPose2D> target_pose;

};

/**
 * @brief This state can only be entered if a GeoPose2D payload is provided.
 *
 */
struct LsManualTask : FSM::State {

  void entryGuard(GuardControl &control) {
		const auto& pendingTransitions = control.pendingTransitions();

    if (auto nav_target = std::get_if<GeoPose2D>(pendingTransitions[0].payload())) {
      target_pose = *nav_target;
    } else {
      RCLCPP_WARN(control.context().node->get_logger(), "LsManualTask : No target pose provided, cancelling transition");
      control.cancelPendingTransitions();
    }
  }
  void enter(Control &control) {
    node = control.context().node;
    navigate_to_position_client = rclcpp_action::create_client<
        NavigateToPosition>(node,
                                                      "navigate_to_position");
    navigate_to_position_client->wait_for_action_server();
  }

  void update(FullControl &control) {
  }

private:
  void nav_target_cb(
      const std::shared_ptr<rover_interfaces::srv::SendNavTarget::Request> request,
      std::shared_ptr<rover_interfaces::srv::SendNavTarget::Response> response) {
    RCLCPP_INFO(node->get_logger(), "Received target pose: %f, %f",
                request->target.position.latitude,
                request->target.position.longitude);
    auto goal_msg = NavigateToPosition::Goal();
    goal_msg.target = request->target;
    auto send_goal_options = rclcpp_action::Client<NavigateToPosition>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LsManualTask::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&LsManualTask::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&LsManualTask::result_callback, this, _1);
    navigate_to_position_client->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateToPosition>::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<NavigateToPosition>::SharedPtr,
      const std::shared_ptr<const NavigateToPosition::Feedback> feedback) {

    //TODO
    // RCLCPP_INFO(node->get_logger(), "Received feedback: %f, %f",
    //             feedback->current.position.latitude,
    //             feedback->current.position.longitude);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPosition>::WrappedResult &result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        break;
    }
  }

  std::shared_ptr<rclcpp::Node> node;
  GeoPose2D target_pose;
  rclcpp_action::Client<NavigateToPosition>::SharedPtr navigate_to_position_client;
};

struct LsSleep : FSM::State {
  void enter(Control &control) {
    printf("Start sleep...\n");
  }

  void update(FullControl &control) {
    // if (std::chrono::system_clock::now() - start_time > std::chrono::seconds(2)) {
    //   printf("Woke up, switching to working\n");
    //   control.changeTo<LsWorking>();
    // }
  }
};

int main() {
	// shared data storage instance
  auto node = std::make_shared<rclcpp::Node>("rover_fsm");
	Context context{node};
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);


	FSM::Instance machine{context};

  while (rclcpp::ok()) {
      machine.update();
      executor.spin_once(std::chrono::milliseconds(100));
  }

	return 0;
}

#include <chrono>
#include <atomic>
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
                  S(LsCharging),
                  S(LsSleeping)
              >,
              M::Composite<S(SsManual),
                  S(LsManualTask),
                  S(LsIdle)
              >
            >;
/* clang-format on */

#undef S
/*********************/

template <typename MsgType> class TopicListener {
public:
  void init(std::shared_ptr<rclcpp::Node> node, const std::string &topic_name) {
    sub = node->create_subscription<MsgType>(
        topic_name, 1, std::bind(&TopicListener::cb, this, std::placeholders::_1));
  }
  void deinit() {
    sub.reset();
  }

  std::optional<MsgType> get_msg() {
    return last_msg;
  }

private:
  virtual void cb(const typename MsgType::SharedPtr msg) {
    last_msg = *msg;
  }

  typename rclcpp::Subscription<MsgType>::SharedPtr sub;
  std::optional<MsgType> last_msg;
};

struct SsManual : FSM::State {
  void enter(Control &control) {
    node = control.context().node;
    RCLCPP_INFO(node->get_logger(), "Start manual...");
    target_pose = {};
    nav_target_service = node->create_service<rover_interfaces::srv::SendNavTarget>(
        "send_nav_target", std::bind(&SsManual::nav_target_cb, this, _1, _2));
  }

  void update(FullControl &control) {
    if (target_pose) {
      RCLCPP_INFO(node->get_logger(), "SsManual: Received manual input...");
      control.changeWith<LsManualTask>(*target_pose);
      target_pose = {};
      return;
    }
  }

private:
  void nav_target_cb(
      const std::shared_ptr<rover_interfaces::srv::SendNavTarget::Request> request,
      std::shared_ptr<rover_interfaces::srv::SendNavTarget::Response> response) {
    RCLCPP_INFO(node->get_logger(), "Received target pose: %f, %f",
                request->target.position.latitude,
                request->target.position.longitude);
    target_pose = request->target;
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Service<rover_interfaces::srv::SendNavTarget>::SharedPtr nav_target_service;
  std::optional<GeoPose2D> target_pose;
};

const unsigned int MANUAL_TIMEOUT_SEC = 10;
struct LsIdle : FSM::State {
  void enter(Control &control) {
    RCLCPP_INFO(control.context().node->get_logger(), "Finished task, idling...");
    start_time = std::chrono::system_clock::now();
  }

  void update(FullControl &control) {
    if (std::chrono::system_clock::now() - start_time > std::chrono::seconds(MANUAL_TIMEOUT_SEC)) {
      RCLCPP_INFO(control.context().node->get_logger(), "No input for %u secs, switching to autonomous", MANUAL_TIMEOUT_SEC);
      control.changeTo<SsAutonomous>();
      return;
    }
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start_time;
};

struct LsWorking : FSM::State {
  void enter(Control &control) {
    RCLCPP_INFO(control.context().node->get_logger(), "Start working...");
    battery_listener.init(control.context().node, "/battery_state");
  }

  void exit(Control &control) {
    battery_listener.deinit();
  }

  void update(FullControl &control) {
    if (auto battery_msg = battery_listener.get_msg()) {
      if (battery_msg->percentage < 0.3) {
        RCLCPP_INFO(control.context().node->get_logger(), "Low battery, switching to charging");
        control.changeTo<LsCharging>();
        return;
      }
    }
  }

private:
  TopicListener<sensor_msgs::msg::BatteryState> battery_listener;
};

struct LsCharging : FSM::State {
  void enter(Control &control) {
    RCLCPP_INFO(control.context().node->get_logger(), "Start charging...");
    battery_listener.init(control.context().node, "/battery_state");
  }

  void exit(Control &control) {
    battery_listener.deinit();
  }

  void update(FullControl &control) {
    if (auto battery_msg = battery_listener.get_msg()) {
      if (battery_msg->percentage < 0.2) {
        RCLCPP_INFO(control.context().node->get_logger(), "Battery critical, switching to sleeping");
        control.changeTo<LsSleeping>();
        return;
      }
      if (battery_msg->percentage >= 1.0) {
        RCLCPP_INFO(control.context().node->get_logger(), "Fully charged, switching to working");
        control.changeTo<LsWorking>();
        return;
      }
    }
    // TODO: Travel to charging station and charge
  }

private:
  TopicListener<sensor_msgs::msg::BatteryState> battery_listener;
};

struct SsAutonomous : FSM::State {
  void enter(Control &control) {
    node = control.context().node;
    RCLCPP_INFO(node->get_logger(), "Start autonomous...");
    target_pose = {};
    nav_target_service = control.context().node->create_service<rover_interfaces::srv::SendNavTarget>(
        "send_nav_target", std::bind(&SsAutonomous::nav_target_cb, this, _1, _2));
  }

  void exit(Control &control) {
    nav_target_service.reset();
  }

  void update(FullControl &control) {
    /* Manual commands always have priority */
    if (target_pose) {
      RCLCPP_INFO(control.context().node->get_logger(), "Received manual input...");
      control.changeWith<LsManualTask>(*target_pose);
      return;
    }
  }

private:
  void nav_target_cb(
      const std::shared_ptr<rover_interfaces::srv::SendNavTarget::Request> request,
      std::shared_ptr<rover_interfaces::srv::SendNavTarget::Response> response) {
    RCLCPP_INFO(node->get_logger(), "Received target pose: %f, %f",
                request->target.position.latitude,
                request->target.position.longitude);
    target_pose = request->target;
  }

  std::shared_ptr<rclcpp::Node> node;
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
    RCLCPP_INFO(node->get_logger(), "Begin ManualTask...");
    is_done = false;
    navigate_to_position_client = rclcpp_action::create_client<NavigateToPosition>(node, "navigate_to_position");
    while (rclcpp::ok() && !navigate_to_position_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(node->get_logger(), "Waiting for action server to appear...");
    }
    auto goal_msg = NavigateToPosition::Goal();
    goal_msg.target = target_pose;
    auto send_goal_options = rclcpp_action::Client<NavigateToPosition>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LsManualTask::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&LsManualTask::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&LsManualTask::result_callback, this, _1);
    navigate_to_position_client->async_send_goal(goal_msg, send_goal_options);
  }

  void reenter(Control &control) {
    enter(control);
  }

  void exit(Control &control) {
    navigate_to_position_client->async_cancel_all_goals();
    navigate_to_position_client.reset();
  }

  void update(FullControl &control) {
    if (is_done) {
      control.changeTo<LsIdle>();
    }
  }

private:
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
    is_done = true;
  }

  std::shared_ptr<rclcpp::Node> node;
  GeoPose2D target_pose;
  rclcpp_action::Client<NavigateToPosition>::SharedPtr navigate_to_position_client;
  std::atomic_bool is_done;
};

struct LsSleeping : FSM::State {
  void enter(Control &control) {
    RCLCPP_INFO(control.context().node->get_logger(), "Start sleep...");
  }

  void update(FullControl &control) {
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
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

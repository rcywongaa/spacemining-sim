#pragma once

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
#include "work_bt/action/do_work.hpp"

namespace fsm {
// Avoid using namespace std::placeholders;
inline constexpr auto& _1 = std::placeholders::_1;
inline constexpr auto& _2 = std::placeholders::_2;
using NavigateToPosition = rover_interfaces::action::NavigateToPosition;
using GeoPose2D = rover_interfaces::msg::GeoPose2D;
using DoWork = work_bt::action::DoWork;

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
                  S(LsCritical)
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
        topic_name, 1, std::bind(&TopicListener::cb, this, _1));
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

template <typename SrvType> class ServiceRequestListener {
public:
  void init(std::shared_ptr<rclcpp::Node> node, const std::string &service_name) {
    request.reset();
    service = node->create_service<SrvType>(service_name, std::bind(&ServiceRequestListener::cb, this, _1, _2));
  }

  void deinit() {
    service.reset();
    request.reset();
  }

  std::optional<typename SrvType::Request::SharedPtr> pop() {
    auto ret = request;
    request.reset();
    return ret;
  }

private:
  virtual void cb(const typename SrvType::Request::SharedPtr req, typename SrvType::Response::SharedPtr res) {
    request = req;
  }

  typename rclcpp::Service<SrvType>::SharedPtr service;
  std::optional<typename SrvType::Request::SharedPtr> request;
};

struct SsManual : FSM::State {
  void enter(Control &control) {
    node = control.context().node;
    RCLCPP_INFO(node->get_logger(), "Start manual...");
    nav_target_listener.init(node, "/send_nav_target");
  }

  void exit(Control &control) {
    nav_target_listener.deinit();
  }

  void update(FullControl &control) {
    if (auto nav_target = nav_target_listener.pop()) {
      RCLCPP_INFO(node->get_logger(), "SsManual: Received manual input");
      control.changeWith<LsManualTask>(nav_target.value()->target);
      return;
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node;
  ServiceRequestListener<rover_interfaces::srv::SendNavTarget> nav_target_listener;
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
    has_critical_failure = false;
    node = control.context().node;

    // FIXME: Parameter cannot be declared twice
    if (!node->has_parameter("battery_topic")) {
      node->declare_parameter("battery_topic", "/battery_state");
    }
    std::string battery_topic = node->get_parameter("battery_topic").as_string();
    RCLCPP_INFO(node->get_logger(), "Start working...");
    battery_listener.init(control.context().node, battery_topic);

    if (!node->has_parameter("do_work_ac_name")) {
      node->declare_parameter("do_work_ac_name", "/do_work");
    }
    std::string do_work_ac_name = node->get_parameter("do_work_ac_name").as_string();
    do_work_client = rclcpp_action::create_client<DoWork>(node, do_work_ac_name);
    while (rclcpp::ok() && !do_work_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(node->get_logger(), "Waiting for %s to appear...", do_work_ac_name.c_str());
    }
  }

  void update(FullControl &control) {
    if (has_critical_failure) {
      RCLCPP_INFO(control.context().node->get_logger(), "Critical failure, switching to Critical");
      control.changeTo<LsCritical>();
      return;
    }

    if (auto battery_msg = battery_listener.get_msg()) {
      if (battery_msg->percentage < 0.3) {
        RCLCPP_INFO(control.context().node->get_logger(), "Low battery, switching to charging");
        control.changeTo<LsCharging>();
        return;
      }
    }

    {
      std::scoped_lock lock(mtx);
      if (!active_goal_handle) {
        /* If done, restart */
        auto send_goal_options = rclcpp_action::Client<DoWork>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&LsWorking::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
          std::bind(&LsWorking::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
          std::bind(&LsWorking::result_callback, this, _1);
        do_work_client->async_send_goal(DoWork::Goal(), send_goal_options);
      }
    }
  }

  /* Note exitGuard() is called before exit() */
  void exitGuard(GuardControl &control) {
    if (active_goal_handle) {
      do_work_client->async_cancel_goal(active_goal_handle);
      /* We should not cancel the transition since LsWorking will then just restart the cancelled job */
      // control.cancelPendingTransitions();
      while (rclcpp::ok() && active_goal_handle) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        RCLCPP_WARN(node->get_logger(), "Waiting for goal to cancel...");
      }
    }
  }

  void exit(Control &control) {
    battery_listener.deinit();
    active_goal_handle.reset();
  }

private:
  void goal_response_callback(const rclcpp_action::ClientGoalHandle<DoWork>::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_WARN(node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node->get_logger(), "Goal accepted by server");
      active_goal_handle = goal_handle;
    }
  }

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<DoWork>::SharedPtr,
      const std::shared_ptr<const DoWork::Feedback> feedback) {
    //TODO
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<DoWork>::WrappedResult &result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        switch(result.result->result.result) {
          case work_bt::msg::WorkResult::SUCCESS:
            RCLCPP_INFO(node->get_logger(), "Work succeeded");
            break;
          case work_bt::msg::WorkResult::FAILURE:
            RCLCPP_WARN(node->get_logger(), "Work failed");
            break;
          case work_bt::msg::WorkResult::CRITICAL_FAILURE:
            RCLCPP_ERROR(node->get_logger(), "Work failed critically");
            has_critical_failure = true;
            break;
          default:
            RCLCPP_ERROR(node->get_logger(), "Unknown work result");
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        break;
    }
    active_goal_handle.reset();
  }

  std::shared_ptr<rclcpp::Node> node;
  TopicListener<sensor_msgs::msg::BatteryState> battery_listener;
  rclcpp_action::Client<DoWork>::SharedPtr do_work_client;
  rclcpp_action::ClientGoalHandle<DoWork>::SharedPtr active_goal_handle;
  std::atomic_bool has_critical_failure;
  std::mutex mtx;
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
        control.changeTo<LsCritical>();
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
    nav_target_listener.init(node, "/send_nav_target");
  }

  void exit(Control &control) {
    nav_target_listener.deinit();
  }

  void update(FullControl &control) {
    /* Manual commands always have priority */
    if (auto nav_target = nav_target_listener.pop()) {
      RCLCPP_INFO(control.context().node->get_logger(), "SsAutonomous : Received manual input...");
      control.changeWith<LsManualTask>(nav_target.value()->target);
      return;
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node;
  ServiceRequestListener<rover_interfaces::srv::SendNavTarget> nav_target_listener;
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

struct LsCritical : FSM::State {
  void enter(Control &control) {
    RCLCPP_INFO(control.context().node->get_logger(), "Start sleep...");
  }

  void update(FullControl &control) {
  }
};

}

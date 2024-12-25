#include <chrono>
#include <memory>
#include <tuple>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rover_interfaces/srv/send_nav_target.hpp"
#include "rover_interfaces/msg/geo_pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <GeographicLib/Geodesic.hpp>

using namespace GeographicLib;
using namespace std::chrono_literals;
using namespace std::placeholders;

const double LINEAR_P = 0.5;
const double ANGULAR_P = 0.5;
const double DISTANCE_THRESHOLD = 1e-3;
const double MAX_ANGULAR_VELOCITY = M_PI/4.0;
const double MAX_LINEAR_VELOCITY = 1.0;
const double ASTEROID_RADIUS = 50.0;
static const Geodesic geo(ASTEROID_RADIUS, 0);

/**
  * Calculate the error between two GeoPose2D messages
  * @param from The starting pose
  * @param to The target pose
  * @return The (heading error in radians, distance error in meters) tuple
  */
std::tuple<double, double> calc_error(rover_interfaces::msg::GeoPose2D from, rover_interfaces::msg::GeoPose2D to) {
  double lat1 = from.position.latitude;
  double lon1 = from.position.longitude;
  double lat2 = to.position.latitude;
  double lon2 = to.position.longitude;
  double distance = 0.0;
  double azimuth1 = 0.0; // deg
  double azimuth2 = 0.0; // deg
  geo.Inverse(lat1, lon1, lat2, lon2, distance, azimuth1, azimuth2);
  // FIXME: This should be between -PI and PI
  double heading_error = (from.heading - azimuth1) * M_PI / 180.0;
  return std::make_tuple(heading_error, distance);
}

class Rover : public rclcpp::Node {
public:
  Rover() : Node("rover") {
    nav_target_service = this->create_service<rover_interfaces::srv::SendNavTarget>(
        "target_pose", std::bind(&Rover::nav_target_cb, this, _1, _2));

    position_listener =
        this->create_subscription<rover_interfaces::msg::GeoPose2D>(
            "current_pose", 1, std::bind(&Rover::position_cb, this, _1));
    velocity_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    runner = this->create_wall_timer(100ms, std::bind(&Rover::timer_cb, this));
  }

private:
  void nav_target_cb(
      const std::shared_ptr<rover_interfaces::srv::SendNavTarget::Request> request,
      std::shared_ptr<rover_interfaces::srv::SendNavTarget::Response> response) {
    target_pose = request->target;
  }

  void position_cb(const rover_interfaces::msg::GeoPose2D &msg) {
    current_pose = msg;
  }

  geometry_msgs::msg::Twist calc_cmd_vel() {
    geometry_msgs::msg::Twist ret;
    auto [heading_error, distance] = calc_error(current_pose, target_pose);

    /* Close enough */
    if (distance < DISTANCE_THRESHOLD) {
      return ret;
    }

    /* Target behind rover */
    if (heading_error > M_PI/2.0 || heading_error < -M_PI/2.0) {
      ret.angular.z = heading_error > 0 ? MAX_ANGULAR_VELOCITY : -MAX_ANGULAR_VELOCITY;
      return ret;
    }

    ret.angular.z = std::clamp(ANGULAR_P * heading_error, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    ret.linear.x = std::clamp(LINEAR_P * distance, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);

    return ret;
  }

  void timer_cb() {
    velocity_publisher->publish(calc_cmd_vel());
  }

  rclcpp::Service<rover_interfaces::srv::SendNavTarget>::SharedPtr nav_target_service;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
  rclcpp::Subscription<rover_interfaces::msg::GeoPose2D>::SharedPtr position_listener;
  rover_interfaces::msg::GeoPose2D target_pose;
  rover_interfaces::msg::GeoPose2D current_pose;
  rclcpp::TimerBase::SharedPtr runner;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rover>());
  rclcpp::shutdown();
  return 0;
}

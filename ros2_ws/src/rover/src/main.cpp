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
#include "angles/angles.h"

#include <GeographicLib/Geodesic.hpp>

using namespace GeographicLib;
using namespace std::chrono_literals;
using namespace std::placeholders;

const double LINEAR_P = 0.5;
const double ANGULAR_P = 1.0;
const double DISTANCE_THRESHOLD = 1e-3;
const double MAX_ANGULAR_VELOCITY = M_PI/4.0;
const double MAX_LINEAR_VELOCITY = 2.0;
const double ASTEROID_RADIUS = 50.0;
static const Geodesic geo(ASTEROID_RADIUS, 0);

double azimuth_to_heading(double azimuth) {
  return 2*M_PI - azimuth;
}
double heading_to_azimuth(double heading) {
  return 2*M_PI - heading;
}

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
  double azimuth1_deg = 0.0; // deg
  double azimuth2_deg = 0.0; // deg
  geo.Inverse(lat1, lon1, lat2, lon2, distance, azimuth1_deg, azimuth2_deg);

  RCLCPP_INFO(rclcpp::get_logger("rover"), "az1: %f, az2: %f", azimuth1_deg, azimuth2_deg);

  // Convert azimuth to a heading
  double desired_heading = azimuth_to_heading(azimuth1_deg);

  double heading_error = angles::shortest_angular_distance(angles::from_degrees(from.heading), angles::from_degrees(desired_heading));
  RCLCPP_INFO(rclcpp::get_logger("rover"), "Current heading: %f, Desired heading: %f, heading error: %f", angles::from_degrees(from.heading), angles::from_degrees(desired_heading), heading_error);
  return std::make_tuple(heading_error, distance);
}

class Rover : public rclcpp::Node {
public:
  Rover() : Node("rover") {
    nav_target_service = this->create_service<rover_interfaces::srv::SendNavTarget>(
        "send_nav_target", std::bind(&Rover::nav_target_cb, this, _1, _2));

    pose_listener =
        this->create_subscription<rover_interfaces::msg::GeoPose2D>(
            "current_pose", 1, std::bind(&Rover::pose_cb, this, _1));
    velocity_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    runner = this->create_wall_timer(100ms, std::bind(&Rover::timer_cb, this));
  }

private:
  void nav_target_cb(
      const std::shared_ptr<rover_interfaces::srv::SendNavTarget::Request> request,
      std::shared_ptr<rover_interfaces::srv::SendNavTarget::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received target pose: %f, %f",
                request->target.position.latitude,
                request->target.position.longitude);
    target_pose = request->target;
  }

  void pose_cb(const rover_interfaces::msg::GeoPose2D &msg) {
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Received current pose: (%f, %f), %f",
    //             msg.position.latitude, msg.position.longitude, msg.heading);
    current_pose = msg;
  }

  geometry_msgs::msg::Twist calc_cmd_vel() {
    geometry_msgs::msg::Twist ret;
    auto [heading_error, distance] = calc_error(current_pose, target_pose);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Heading error: %f, Distance error: %f", heading_error, distance);

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
    auto cmd_vel = calc_cmd_vel();
    RCLCPP_INFO(this->get_logger(), "Linear velocity: %f, Angular velocity: %f", cmd_vel.linear.x, cmd_vel.angular.z);
    velocity_publisher->publish(cmd_vel);
  }

  rclcpp::Service<rover_interfaces::srv::SendNavTarget>::SharedPtr nav_target_service;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
  rclcpp::Subscription<rover_interfaces::msg::GeoPose2D>::SharedPtr pose_listener;
  rclcpp::Publisher<rover_interfaces::msg::GeoPose2D>::SharedPtr target_publisher;
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

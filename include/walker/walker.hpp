/**
 * @file walker.hpp
 * @author koustubh (koustubh@umd.edu)
 * @brief Contains the declaration of the Walker class and the RobotState class
 * @version 0.1
 * @date 2024-11-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#ifndef WALKER_HPP_
#define WALKER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cxxabi.h>

// Forward declaration of Walker class
class Walker;

/**
 * @brief Robot state class to define the state of the robot
 * @class RobotState
 */
class RobotState {
 public:
  virtual void execute(Walker &context, float min_distance) = 0;
  virtual ~RobotState() = default;
};

/**
 * @brief Moving forward state of the robot
 * @class MovingForward
 */
class MovingForward : public RobotState {
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Rotating clockwise state of the robot
 * @class RotatingClockwise
 */
class RotatingClockwise : public RobotState {
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Rotating counter clockwise state of the robot
 * @class RotatingCounterClockwise
 */
class RotatingCounterClockwise : public RobotState {
  void execute(Walker &context, float min_distance) override;
};

/**
 * @brief Walker class to control the robot
 * @class Walker
 */
class Walker : public rclcpp::Node {
 public:
  Walker();
  // Callback function for the laser scan data
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  // Function to change the state of the robot
  void changeState(std::shared_ptr<RobotState> new_state);
  // Function to demangle the type name
  std::string demangle(const char *name);
  // create a publisher to publish the velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  // create a subscription to subscribe to the laser scan data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  // string to store the current state of the robot
  std::string current_state_;
  // string to store the previous state of the robot
  std::string previous_state_;
  // state of the robot
  std::shared_ptr<RobotState> state_;
  // variable to store the previous rotation direction
  bool rotating_clockwise_;
};

#endif  // WALKER_HPP_
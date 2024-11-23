/**
 * @file walker.cpp
 * @author koustubh (koustubh@umd.edu)
 * @brief Contains the implementation of the Walker and RobotState class
 * @version 0.1
 * @date 2024-11-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "walker/walker.hpp"

#define MAX_FORWARD_SPEED 0.2
#define MAX_ROTATION_SPEED 0.4
#define DISTANCE_THRESHOLD 0.8

// Walker class definition
Walker::Walker() : Node("walker") {
  // Create a publisher to publish Twist messages to cmd_vel topic
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Create a subscription to subscribe to LaserScan messages from scan topic
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Walker::scanCallback, this, std::placeholders::_1));
  // Initialize the state of the robot to MovingForward
  state_ = std::make_shared<MovingForward>();
  // Initialize the current and previous state of the robot
  current_state_ = "MovingForward";
  previous_state_ = "MovingForward";
  // Initialize the rotating direction to false
  rotating_clockwise_ = false;
  RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
  RCLCPP_INFO(this->get_logger(), "Current state: %s", current_state_.c_str());
}

// Callback function to process LaserScan messages
void Walker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Define the forward range of the robot
  const double forward_min_angle = -M_PI / 5;
  const double forward_max_angle = M_PI / 5;

  // Initialize the minimum distance to infinity
  float min_distance = std::numeric_limits<float>::infinity();

  // Iterate through the ranges of the LaserScan message
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;
    if (angle >= forward_min_angle && angle <= forward_max_angle) {
      float range = msg->ranges[i];
      if (std::isfinite(range) && range < min_distance) {
        min_distance = range;
      }
    }
  }

  state_->execute(*this, min_distance);
}

// Function to change the state of the robot
void Walker::changeState(std::shared_ptr<RobotState> new_state) {
  previous_state_ = current_state_;
  current_state_ = typeid(new_state).name();
  state_ = new_state;
  RCLCPP_INFO(this->get_logger(), "State changed from %s to %s",
              previous_state_.c_str(), current_state_.c_str());
}

// Implement the execute function for forward state
void MovingForward::execute(Walker &context, float min_distance) {
  // Check if the minimum distance is less than the threshold
  if (min_distance < DISTANCE_THRESHOLD) {
    // check if the robot was rotating clockwise
    if (context.rotating_clockwise_) {
      // change the state to rotating counter clockwise
      context.changeState(std::make_shared<RotatingCounterClockwise>());
      context.rotating_clockwise_ = false;
    } else {
      // change the state to rotating clockwise
      context.changeState(std::make_shared<RotatingClockwise>());
      context.rotating_clockwise_ = true;
    }
  } else {
    auto msg = geometry_msgs::msg::Twist();
    // Set the linear velocity to the maximum forward speed
    msg.linear.x = MAX_FORWARD_SPEED;
    context.publisher_->publish(msg);
  }
}

// Implement the execute function for rotating clockwise state
void RotatingClockwise::execute(Walker &context, float min_distance) {
  // Check if the minimum distance is greater than the threshold
  if (min_distance > DISTANCE_THRESHOLD) {
    // change the state to moving forward
    context.changeState(std::make_shared<MovingForward>());
  } else {
    // Publish the angular velocity to rotate clockwise
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = MAX_ROTATION_SPEED;
    context.publisher_->publish(msg);
  }
}

// Implement the execute function for rotating counter clockwise state
void RotatingCounterClockwise::execute(Walker &context, float min_distance) {
  // Check if the minimum distance is greater than the threshold
  if (min_distance > DISTANCE_THRESHOLD) {
    // change the state to moving forward
    context.changeState(std::make_shared<MovingForward>());
  } else {
    // Publish the angular velocity to rotate counter clockwise
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = -MAX_ROTATION_SPEED;
    context.publisher_->publish(msg);
  }
}

// Main function to run the walker node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}

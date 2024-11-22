#pragma once

#ifndef WALKER_HPP_
#define WALKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>

class Walker;

class RobotState {
    virtual void execute() = 0;
};

class MovingForward : public RobotState {
    void execute(Walker &context, float min_distance) override;
};

class RotatingClockwise : public RobotState {
    void execute(Walker &context, float min_distance) override;
};

class RotatingCounterClockwise : public RobotState {
    void execute(Walker &context, float min_distance) override;
};

class Walker : public rclcpp::Node {
public:
    Walker();
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void changeState(std::shared_ptr<RobotState> new_state);
    rcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::string current_state_;
    std::string previous_state_;
    std::shared_ptr<RobotState> state_;
    bool rotating_clockwise_;
};

#endif  // WALKER_HPP_
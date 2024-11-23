#include "walker/walker.hpp"

#define MAX_FORWARD_SPEED 0.3
#define MAX_ROTATION_SPEED 0.3
#define DISTANCE_THRESHOLD 0.6

Walker::Walker() : Node("walker") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Walker::scanCallback, this, std::placeholders::_1));
    state_ = std::make_shared<MovingForward>();
    current_state_ = "MovingForward";
    previous_state_ = "MovingForward";
    rotating_clockwise_ = false;
    RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
    RCLCPP_INFO(this->get_logger(), "Current state: %s", current_state_.c_str());
}

void Walker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const double forward_min_angle = -M_PI / 5;
    const double forward_max_angle = M_PI / 5;

    float min_distance = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {

        double angle = msg->angle_min + i * msg->angle_increment;
        // Check if the angle is within the forward range
        if (angle >= forward_min_angle && angle <= forward_max_angle) {
        float range = msg->ranges[i];
        if (std::isfinite(range) && range < min_distance) {
            min_distance = range;
            }
        }
    }

    state_->execute(*this, min_distance);
}


void Walker::changeState(std::shared_ptr<RobotState> new_state){
    previous_state_ = current_state_;
    current_state_ = typeid(*new_state).name();
    state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "State changed from %s to %s", previous_state_.c_str(), current_state_.c_str());
}

void MovingForward::execute(Walker &context, float min_distance){
    if (min_distance < DISTANCE_THRESHOLD){
        if (context.rotating_clockwise_){
            context.changeState(std::make_shared<RotatingCounterClockwise>());
            context.rotating_clockwise_ = false;
        } else {
            context.changeState(std::make_shared<RotatingClockwise>());
            context.rotating_clockwise_ = true;
        }
    }
    else {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = MAX_FORWARD_SPEED;
        context.publisher_->publish(msg);
    }
}

void RotatingClockwise::execute(Walker &context, float min_distance){
    if(min_distance > DISTANCE_THRESHOLD){
        context.changeState(std::make_shared<MovingForward>());
    }
    else {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = MAX_ROTATION_SPEED;
        context.publisher_->publish(msg);
    }
}

void RotatingCounterClockwise::execute(Walker &context, float min_distance){
    if(min_distance > DISTANCE_THRESHOLD){
        context.changeState(std::make_shared<MovingForward>());
    }
    else {
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = -MAX_ROTATION_SPEED;
        context.publisher_->publish(msg);
        }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walker>());
    rclcpp::shutdown();
    return 0;
}

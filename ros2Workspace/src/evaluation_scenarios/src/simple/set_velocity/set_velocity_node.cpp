#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

#include "std_msgs/msg/float64.hpp"

#include "set_velocity_node.hpp"
#include "../constants.hpp"
#include "../../roboter_parameter.hpp"
#include "../../default_robot_node.hpp"

using namespace std;
using std::placeholders::_1;

void SetVelocityNode::forwards() {
    this->publish_velocity(this->pub_right_wheel_velocity, this->outer_wheel_velocity);
    this->publish_velocity(this->pub_left_wheel_velocity, this->outer_wheel_velocity);
}

void SetVelocityNode::left() {
    this->publish_velocity(this->pub_right_wheel_velocity, this->outer_wheel_velocity);
    this->publish_velocity(this->pub_left_wheel_velocity, this->inner_wheel_velocity);
}

void SetVelocityNode::right() {
    this->publish_velocity(this->pub_right_wheel_velocity, this->inner_wheel_velocity);
    this->publish_velocity(this->pub_left_wheel_velocity, this->outer_wheel_velocity);
}

void SetVelocityNode::stop() {
    this->publish_velocity(this->pub_right_wheel_velocity, 0.0);
    this->publish_velocity(this->pub_left_wheel_velocity, 0.0);
}
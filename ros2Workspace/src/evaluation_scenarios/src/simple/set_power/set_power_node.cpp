#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

#include "std_msgs/msg/float64.hpp"

#include "set_power_node.hpp"
#include "../../roboter_parameter.hpp"
#include "../../default_robot_node.hpp"

using namespace std;
using std::placeholders::_1;

void SetPowerNode::forwards() {
    this->publish_power(this->pub_right_wheel_power, this->outer_wheel_power);
    this->publish_power(this->pub_left_wheel_power, this->outer_wheel_power);
}

void SetPowerNode::left() {
    this->publish_power(this->pub_right_wheel_power, this->outer_wheel_power);
    this->publish_power(this->pub_left_wheel_power, this->inner_wheel_power);
}

void SetPowerNode::right() {
    this->publish_power(this->pub_right_wheel_power, this->inner_wheel_power);
    this->publish_power(this->pub_left_wheel_power, this->outer_wheel_power);
}

void SetPowerNode::stop() {
    this->publish_power(this->pub_right_wheel_power, 0.0);
    this->publish_power(this->pub_left_wheel_power, 0.0);
}
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

#include "set_position_node.hpp"
#include "../constants.hpp"
#include "../../roboter_parameter.hpp"
#include "../../default_robot_node.hpp"

using namespace std;
using std::placeholders::_1;

void SetPositionNode::handle_current_direction() {
    switch (this->direction) {
        case FORWARD:
            return this->forwards();
        case LEFT:
            return this->left();
        case RIGHT:
            return this->right();
    }
}

void SetPositionNode::forwards() {
    this->publish_position(this->pub_right_wheel_position, this->current_position_right);
    this->publish_position(this->pub_left_wheel_position, this->current_position_left);

    this->current_position_left += this->increment;
    this->current_position_right += this->increment;
}

void SetPositionNode::left() {
    this->publish_position(this->pub_right_wheel_position, this->current_position_right);
    
    this->current_position_right += this->increment;
}

void SetPositionNode::right() {
    this->publish_position(this->pub_left_wheel_position, this->current_position_left);

    this->current_position_left += this->increment;
}

void SetPositionNode::init_publishers() {
    this->pub_set_position_offset_left = this->create_publisher<std_msgs::msg::Float64>(
        "/" + ((string) ROBOT_NAME) + "/" + LEFT_WHEEL_PORT + "/setPositionOffset", 10
    );
    this->pub_set_position_offset_right = this->create_publisher<std_msgs::msg::Float64>(
        "/" + ((string) ROBOT_NAME) + "/" + RIGHT_WHEEL_PORT + "/setPositionOffset", 10
    );
}

void SetPositionNode::offset_position() {
    std_msgs::msg::Float64 left_position_offset = std_msgs::msg::Float64();
    std_msgs::msg::Float64 right_position_offset = std_msgs::msg::Float64();

    left_position_offset.data = 0;
    right_position_offset.data = 0;

    this->pub_set_position_offset_left->publish(left_position_offset);
    this->pub_set_position_offset_right->publish(right_position_offset);
}
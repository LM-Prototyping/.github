#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/empty.hpp"

#include "default_robot_node.hpp"
#include "roboter_parameter.hpp"

using namespace std;
using std::placeholders::_1;

DefaultRobotNode::DefaultRobotNode(string node_name, string robot_name): Node(node_name) {
    string topic_prefix = "/" + ((string) robot_name) + "/";

    // set position
    this->pub_left_wheel_position = this->create_publisher<std_msgs::msg::Float64>(
        topic_prefix + LEFT_WHEEL_PORT + "/setPosition", 10
    );
    this->pub_right_wheel_position = this->create_publisher<std_msgs::msg::Float64>(
        topic_prefix + RIGHT_WHEEL_PORT + "/setPosition", 10
    );

    // set velocity
    this->pub_left_wheel_velocity = this->create_publisher<std_msgs::msg::Float32>(
        topic_prefix + LEFT_WHEEL_PORT + "/setVelocity", 10
    );
    this->pub_right_wheel_velocity = this->create_publisher<std_msgs::msg::Float32>(
        topic_prefix + RIGHT_WHEEL_PORT + "/setVelocity", 10
    );

    // set power
    this->pub_left_wheel_power = this->create_publisher<std_msgs::msg::Int16>(
        topic_prefix + LEFT_WHEEL_PORT + "/setPower", 10
    );
    this->pub_right_wheel_power = this->create_publisher<std_msgs::msg::Int16>(
        topic_prefix + RIGHT_WHEEL_PORT + "/setPower", 10
    );

    // publish finished
    this->pub_finished = this->create_publisher<std_msgs::msg::Empty>(
        "/eval/has_finished", 10
    );

    this->sub_step = this->create_subscription<std_msgs::msg::Empty>(
        "/step",
        10,
        bind(&DefaultRobotNode::step_callback, this, _1)
    );
}

void DefaultRobotNode::publish_position(
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher, 
    float position
) {
    std_msgs::msg::Float64 msg = std_msgs::msg::Float64();
    msg.data = position;

    publisher->publish(msg);
}

void DefaultRobotNode::publish_velocity(
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher, 
    float velocity
) {
    std_msgs::msg::Float32 msg = std_msgs::msg::Float32();
    msg.data = velocity;

    publisher->publish(msg);
}

void DefaultRobotNode::publish_power(
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher,
    int power 
) {
    std_msgs::msg::Int16 msg = std_msgs::msg::Int16();
    msg.data = power;

    publisher->publish(msg);
}

void DefaultRobotNode::publish_finished() {
    std_msgs::msg::Empty msg = std_msgs::msg::Empty();

    this->pub_finished->publish(msg);
}
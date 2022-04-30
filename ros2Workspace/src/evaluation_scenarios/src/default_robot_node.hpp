#ifndef DEFAULT_ROBOT_NODE_HPP
#define DEFAULT_ROBOT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/empty.hpp"

#include <string>

class DefaultRobotNode: public rclcpp::Node {
    protected:
        bool should_send = false;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_wheel_position;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_wheel_position;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_left_wheel_velocity;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_right_wheel_velocity;

        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_left_wheel_power;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_right_wheel_power;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_step;

        // Publishes a message when finished eval run so camera can stop streaming
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_finished;

        void publish_position(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher, float position);
        void publish_velocity(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher, float velocity);
        void publish_power(rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher, int power);

        void publish_finished();

        virtual void step_callback(const std_msgs::msg::Empty::SharedPtr) {};

        virtual void forwards() {};
        virtual void backwards() {};

        virtual void left() {};
        virtual void right() {};
        virtual void stop() {};

    public:
        DefaultRobotNode(std::string node_name, std::string robot_name);

};

#endif
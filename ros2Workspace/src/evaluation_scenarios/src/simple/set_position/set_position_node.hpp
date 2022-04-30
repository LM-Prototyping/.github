#ifndef SET_POSITION_NODE_HPP
#define SET_POSITION_NODE_HPP

#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "../../default_robot_node.hpp"

#include <string>

enum Direction {
    FORWARD,
    LEFT,
    RIGHT
};

class SetPositionNode: public DefaultRobotNode {
    private:
        float current_position_left = 0.0;
        float current_position_right = 0.0;
        int counter = 0;
        
        Direction direction = FORWARD;
        float increment = M_PI / 20;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_set_position_offset_left;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_set_position_offset_right;

        void init_publishers();

        void handle_current_direction();

        void offset_position();

    protected:
        void forwards() override;
        void left() override;
        void right() override;
        
        void step_callback(const std_msgs::msg::Empty::SharedPtr) override;

    public:
        SetPositionNode();
        
};

#endif